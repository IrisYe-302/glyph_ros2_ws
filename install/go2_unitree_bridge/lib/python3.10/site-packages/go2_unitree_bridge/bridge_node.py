import json
from typing import Final

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState


JOINT_NAMES: Final[list[str]] = [
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
]

MOTOR_INDEX_ORDER: Final[list[int]] = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

API_ID_BALANCE_STAND: Final[int] = 1002
API_ID_STOP_MOVE: Final[int] = 1003
API_ID_MOVE: Final[int] = 1008


class Go2UnitreeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_unitree_bridge_node")

        self.declare_parameter("sport_state_topic", "lf/sportmodestate")
        self.declare_parameter("sport_state_fallback_topic", "/sportmodestate")
        self.declare_parameter("low_state_topic", "lf/lowstate")
        self.declare_parameter("low_state_fallback_topic", "/lowstate")
        self.declare_parameter("cmd_topic", "/api/sport/request")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)

        sport_state_topic = self.get_parameter("sport_state_topic").value
        sport_state_fallback_topic = self.get_parameter("sport_state_fallback_topic").value
        low_state_topic = self.get_parameter("low_state_topic").value
        low_state_fallback_topic = self.get_parameter("low_state_fallback_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.sport_request_publisher = self.create_publisher(Request, cmd_topic, 10)

        self.create_subscription(SportModeState, sport_state_topic, self._on_sport_state, 10)
        if sport_state_fallback_topic != sport_state_topic:
            self.create_subscription(
                SportModeState,
                sport_state_fallback_topic,
                self._on_sport_state,
                10,
            )
        self.create_subscription(LowState, low_state_topic, self._on_low_state, 10)
        if low_state_fallback_topic != low_state_topic:
            self.create_subscription(
                LowState,
                low_state_fallback_topic,
                self._on_low_state,
                10,
            )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self._request_id = 1
        self.get_logger().info(
            "Bridging "
            f"{sport_state_topic} ({sport_state_fallback_topic}) -> /odom,/tf and "
            f"{low_state_topic} ({low_state_fallback_topic}) -> /joint_states,/imu/data"
        )

    def _next_request_id(self) -> int:
        self._request_id += 1
        return self._request_id

    def _on_sport_state(self, msg: SportModeState) -> None:
        stamp = self.get_clock().now().to_msg()

        if self.publish_odom_enabled:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = float(msg.position[0])
            odom.pose.pose.position.y = float(msg.position[1])
            odom.pose.pose.position.z = float(msg.position[2])
            odom.pose.pose.orientation.x = float(msg.imu_state.quaternion[0])
            odom.pose.pose.orientation.y = float(msg.imu_state.quaternion[1])
            odom.pose.pose.orientation.z = float(msg.imu_state.quaternion[2])
            odom.pose.pose.orientation.w = float(msg.imu_state.quaternion[3])
            odom.twist.twist.linear.x = float(msg.velocity[0])
            odom.twist.twist.linear.y = float(msg.velocity[1])
            odom.twist.twist.linear.z = float(msg.velocity[2])
            odom.twist.twist.angular.z = float(msg.yaw_speed)
            self.odom_publisher.publish(odom)

        if self.publish_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(msg.position[0])
            transform.transform.translation.y = float(msg.position[1])
            transform.transform.translation.z = float(msg.position[2])
            transform.transform.rotation.x = float(msg.imu_state.quaternion[0])
            transform.transform.rotation.y = float(msg.imu_state.quaternion[1])
            transform.transform.rotation.z = float(msg.imu_state.quaternion[2])
            transform.transform.rotation.w = float(msg.imu_state.quaternion[3])
            self.tf_broadcaster.sendTransform(transform)

    def _on_low_state(self, msg: LowState) -> None:
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = [float(msg.motor_state[index].q) for index in MOTOR_INDEX_ORDER]
        joint_state.velocity = [float(msg.motor_state[index].dq) for index in MOTOR_INDEX_ORDER]
        self.joint_state_publisher.publish(joint_state)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation.x = float(msg.imu_state.quaternion[0])
        imu.orientation.y = float(msg.imu_state.quaternion[1])
        imu.orientation.z = float(msg.imu_state.quaternion[2])
        imu.orientation.w = float(msg.imu_state.quaternion[3])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        imu.orientation_covariance[0] = 0.0025
        imu.orientation_covariance[4] = 0.0025
        imu.orientation_covariance[8] = 0.0025
        imu.angular_velocity_covariance[0] = 0.000001
        imu.angular_velocity_covariance[4] = 0.000001
        imu.angular_velocity_covariance[8] = 0.000001
        imu.linear_acceleration_covariance[0] = 0.0001
        imu.linear_acceleration_covariance[4] = 0.0001
        imu.linear_acceleration_covariance[8] = 0.0001
        self.imu_publisher.publish(imu)

    def _on_cmd_vel(self, msg: Twist) -> None:
        request = Request()
        request.header.identity.id = self._next_request_id()

        moving = any(
            abs(value) > 1e-4
            for value in (msg.linear.x, msg.linear.y, msg.angular.z)
        )

        if moving:
            request.header.identity.api_id = API_ID_MOVE
            request.parameter = json.dumps(
                {
                    "x": float(msg.linear.x),
                    "y": float(msg.linear.y),
                    "z": float(msg.angular.z),
                }
            )
        else:
            request.header.identity.api_id = API_ID_STOP_MOVE

        self.sport_request_publisher.publish(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2UnitreeBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
