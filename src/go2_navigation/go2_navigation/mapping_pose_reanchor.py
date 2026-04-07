import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import Clear, DeserializePoseGraph, Pause, SerializePoseGraph


class MappingPoseReanchor(Node):
    def __init__(self) -> None:
        super().__init__("go2_mapping_pose_reanchor")

        self.declare_parameter("input_topic", "/initialpose")
        self.declare_parameter("serialize_service", "/slam_toolbox/serialize_map")
        self.declare_parameter("deserialize_service", "/slam_toolbox/deserialize_map")
        self.declare_parameter("pause_service", "/slam_toolbox/pause_new_measurements")
        self.declare_parameter("clear_service", "/slam_toolbox/clear_changes")
        self.declare_parameter("pose_graph_file", "/tmp/go2_mapping_reanchor.posegraph")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("max_linear_speed", 0.03)
        self.declare_parameter("max_angular_speed", 0.08)

        input_topic = str(self.get_parameter("input_topic").value)
        self.serialize_service_name = str(self.get_parameter("serialize_service").value)
        self.deserialize_service_name = str(self.get_parameter("deserialize_service").value)
        self.pause_service_name = str(self.get_parameter("pause_service").value)
        self.clear_service_name = str(self.get_parameter("clear_service").value)
        self.pose_graph_file = str(self.get_parameter("pose_graph_file").value)
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.serialize_client = self.create_client(SerializePoseGraph, self.serialize_service_name)
        self.deserialize_client = self.create_client(DeserializePoseGraph, self.deserialize_service_name)
        self.pause_client = self.create_client(Pause, self.pause_service_name)
        self.clear_client = self.create_client(Clear, self.clear_service_name)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            input_topic,
            self._handle_pose,
            qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._handle_odom,
            10,
        )

        self.in_flight = False
        self.pending_pose = None
        self.latest_motion = None

        self.get_logger().warn(
            "Experimental mapping re-anchor enabled. Use only as a recovery tool."
        )

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _handle_odom(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        linear_speed = math.hypot(linear.x, linear.y)
        angular_speed = abs(angular.z)
        self.latest_motion = (linear_speed, angular_speed)

    def _is_stationary_enough(self) -> bool:
        if self.latest_motion is None:
            return False
        linear_speed, angular_speed = self.latest_motion
        return (
            linear_speed <= self.max_linear_speed
            and angular_speed <= self.max_angular_speed
        )

    def _handle_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.goal_frame_id:
            self.get_logger().warn(
                f"Ignoring pose in frame '{msg.header.frame_id}', expected '{self.goal_frame_id}'"
            )
            return

        if self.in_flight:
            self.get_logger().warn("Re-anchor already in progress")
            return

        if not self._is_stationary_enough():
            self.get_logger().warn(
                "Ignoring re-anchor while robot is moving; stop first and try again"
            )
            return

        if not self.serialize_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Service unavailable: {self.serialize_service_name}")
            return
        if not self.deserialize_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Service unavailable: {self.deserialize_service_name}")
            return
        if not self.pause_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Service unavailable: {self.pause_service_name}")
            return
        if not self.clear_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Service unavailable: {self.clear_service_name}")
            return

        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation.z, pose.orientation.w)
        self.pending_pose = (pose.position.x, pose.position.y, yaw)
        self.in_flight = True

        Path(self.pose_graph_file).parent.mkdir(parents=True, exist_ok=True)

        future = self.pause_client.call_async(Pause.Request())
        future.add_done_callback(self._on_pause_for_reanchor_done)
        self.get_logger().info("Pausing new SLAM measurements before re-anchor")

    def _on_pause_for_reanchor_done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Pause request failed: {exc}")
            self.in_flight = False
            return

        request = SerializePoseGraph.Request()
        request.filename = self.pose_graph_file
        future = self.serialize_client.call_async(request)
        future.add_done_callback(self._on_serialize_done)
        self.get_logger().info(f"Serializing pose graph to '{self.pose_graph_file}'")

    def _on_serialize_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Serialize request failed: {exc}")
            self._resume_measurements()
            return

        if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            self.get_logger().error(f"Serialize failed with result code {response.result}")
            self._resume_measurements()
            return

        x, y, yaw = self.pending_pose
        request = DeserializePoseGraph.Request()
        request.filename = self.pose_graph_file
        request.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        request.initial_pose.x = float(x)
        request.initial_pose.y = float(y)
        request.initial_pose.theta = float(yaw)
        future = self.deserialize_client.call_async(request)
        future.add_done_callback(self._on_deserialize_done)
        self.get_logger().info(
            f"Re-anchoring to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def _on_deserialize_done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Deserialize request failed: {exc}")
        else:
            self.get_logger().info("Re-anchor request completed, clearing SLAM changes")
        future = self.clear_client.call_async(Clear.Request())
        future.add_done_callback(self._on_clear_done)

    def _on_clear_done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f"Clear changes request failed: {exc}")
        self._resume_measurements()

    def _resume_measurements(self) -> None:
        future = self.pause_client.call_async(Pause.Request())
        future.add_done_callback(self._on_resume_done)

    def _on_resume_done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f"Resume request failed: {exc}")
        finally:
            self.in_flight = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingPoseReanchor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
