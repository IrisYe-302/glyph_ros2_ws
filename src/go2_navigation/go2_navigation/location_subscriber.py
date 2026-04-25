import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener


class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('go2_location_subscriber')

        self.declare_parameter('target_topic', '/target_location')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('target_frame_override', '')
        self.declare_parameter('target_xy_rotation_deg', 0.0)
        self.declare_parameter('orient_toward_goal_center', True)
        self.declare_parameter('goal_cleared_topic', '/target_location_cleared')
        self.declare_parameter('goal_failed_topic', '')
        self.declare_parameter('duplicate_goal_position_tol', 0.05)
        self.declare_parameter('duplicate_goal_yaw_tol', 0.20)

        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.goal_frame_id = self.get_parameter('goal_frame_id').get_parameter_value().string_value
        self.target_frame_override = (
            self.get_parameter('target_frame_override').get_parameter_value().string_value
        )
        self.target_xy_rotation_deg = float(self.get_parameter('target_xy_rotation_deg').value)
        self.orient_toward_goal_center = (
            self.get_parameter('orient_toward_goal_center').get_parameter_value().bool_value
        )
        goal_cleared_topic = self.get_parameter('goal_cleared_topic').get_parameter_value().string_value
        goal_failed_topic = self.get_parameter('goal_failed_topic').get_parameter_value().string_value
        self.duplicate_goal_position_tol = float(self.get_parameter('duplicate_goal_position_tol').value)
        self.duplicate_goal_yaw_tol = float(self.get_parameter('duplicate_goal_yaw_tol').value)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_ready = False
        self.frame_ready = False
        self.base_frame_candidates = ['base_footprint', 'base_link']
        self.target_qos = self._target_qos_for_topic(target_topic)
        self._active_goal_signature: tuple[float, float, float] | None = None
        self._pending_pose: PoseStamped | None = None

        self.subscription = self.create_subscription(
            PoseStamped,
            target_topic,
            self.location_callback,
            self.target_qos,
        )
        self.goal_cleared_publisher = self.create_publisher(Empty, goal_cleared_topic, 10)
        self.goal_failed_publisher = (
            self.create_publisher(Empty, goal_failed_topic, 10)
            if goal_failed_topic
            else None
        )

        self.get_logger().info(f'Listening for target locations on {target_topic}')
        self.get_logger().info('Waiting for Nav2 action server and goal frame...')
        self.readiness_timer = self.create_timer(0.5, self._check_readiness)

    def _target_qos_for_topic(self, topic: str) -> QoSProfile:
        durability = (
            DurabilityPolicy.VOLATILE
            if topic == '/move_base_simple/goal'
            else DurabilityPolicy.TRANSIENT_LOCAL
        )
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=durability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def _check_readiness(self):
        if not self.nav_ready:
            self.nav_ready = self.nav_client.wait_for_server(timeout_sec=0.2)
            if self.nav_ready:
                self.get_logger().info('Nav2 action server connected')

        if not self.frame_ready:
            for base_frame in self.base_frame_candidates:
                try:
                    if self.tf_buffer.can_transform(
                        self.goal_frame_id,
                        base_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.2),
                    ):
                        self.tf_buffer.lookup_transform(
                            self.goal_frame_id,
                            base_frame,
                            rclpy.time.Time(),
                            timeout=Duration(seconds=0.2),
                        )
                        self.frame_ready = True
                        self.get_logger().info(
                            f"Goal frame '{self.goal_frame_id}' is available via '{base_frame}'"
                        )
                        break
                except Exception:
                    continue

        if self.nav_ready and self.frame_ready:
            self.readiness_timer.cancel()
            if self._pending_pose is not None:
                pending_pose = self._pending_pose
                self._pending_pose = None
                self.get_logger().info('Dispatching pending target after readiness became available')
                self.send_navigation_goal(pending_pose)

    def location_callback(self, msg: PoseStamped):
        if not self.nav_ready:
            self.nav_ready = self.nav_client.wait_for_server(timeout_sec=1.0)
            if self.nav_ready:
                self.get_logger().info('Nav2 action server connected')

        if not self.frame_ready:
            self._check_readiness()

        if not self.nav_ready or not self.frame_ready:
            self._pending_pose = msg
            self.get_logger().warn(
                f"Deferring target until Nav2 and frame '{self.goal_frame_id}' are ready "
                f"(nav_ready={self.nav_ready}, frame_ready={self.frame_ready})"
            )
            return

        msg_time = Time.from_msg(msg.header.stamp)
        goal_age = (self.get_clock().now() - msg_time).nanoseconds / 1e9
        if goal_age > 2.0:
            self.get_logger().warn(f'Ignoring stale latched target ({goal_age:.2f}s old)')
            return

        self.get_logger().info(
            f'Received target: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}'
        )
        self.send_navigation_goal(msg)

    def _rotate_target_xy(self, pose: PoseStamped):
        if abs(self.target_xy_rotation_deg) < 1e-6:
            return
        theta = math.radians(self.target_xy_rotation_deg)
        x = pose.pose.position.x
        y = pose.pose.position.y
        pose.pose.position.x = math.cos(theta) * x - math.sin(theta) * y
        pose.pose.position.y = math.sin(theta) * x + math.cos(theta) * y
        self.get_logger().info(
            f"Rotated incoming target XY by {self.target_xy_rotation_deg:.1f} deg"
        )

    def send_navigation_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if self.target_xy_rotation_deg:
            self._rotate_target_xy(goal_msg.pose)

        if self.target_frame_override:
            original_frame = goal_msg.pose.header.frame_id or '<empty>'
            goal_msg.pose.header.frame_id = self.target_frame_override
            self.get_logger().info(
                f"Overriding target frame from '{original_frame}' to '{self.target_frame_override}'"
            )
        elif not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = self.goal_frame_id

        if goal_msg.pose.header.frame_id != self.goal_frame_id:
            source_frame = goal_msg.pose.header.frame_id
            try:
                goal_msg.pose = self.tf_buffer.transform(
                    goal_msg.pose,
                    self.goal_frame_id,
                    timeout=Duration(seconds=0.2),
                )
                self.get_logger().info(
                    f"Transformed target from '{source_frame}' to '{self.goal_frame_id}'"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to transform target from '{source_frame}' to '{self.goal_frame_id}': {exc}"
                )
                self._publish_goal_failed()
                return

        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0

        if self.orient_toward_goal_center:
            self._orient_goal_toward_center(goal_msg.pose)

        goal_signature = self._goal_signature(goal_msg.pose)
        if self._goal_is_duplicate(goal_signature):
            self.get_logger().info('Ignoring duplicate active target')
            return
        self._active_goal_signature = goal_signature

        self.get_logger().info('Sending robot to target...')
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal_signature = None
            self.get_logger().error(f'Failed to send goal: {exc}')
            self._publish_goal_failed()
            return

        if not goal_handle.accepted:
            self._active_goal_signature = None
            self.get_logger().error('Goal rejected')
            self._publish_goal_failed()
            return

        self.get_logger().info('Goal accepted — waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        try:
            self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f} m')
        except Exception:
            self.get_logger().debug('Received feedback')

    def _result_callback(self, future):
        try:
            action_result = future.result()
        except Exception as exc:
            self._active_goal_signature = None
            self.get_logger().error(f'Failed to receive navigation result: {exc}')
            self._publish_goal_failed()
            return

        result = action_result.result
        status = action_result.status
        self._active_goal_signature = None
        self.get_logger().info(f'Navigation finished with status {status}: {result}')
        if status == 4:
            self.goal_cleared_publisher.publish(Empty())
            return
        if status in {5, 6}:
            self._publish_goal_failed()

    def _orient_goal_toward_center(self, pose: PoseStamped):
        for base_frame in self.base_frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    pose.header.frame_id,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                dx = pose.pose.position.x - transform.transform.translation.x
                dy = pose.pose.position.y - transform.transform.translation.y
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    return

                yaw = math.atan2(dy, dx)
                half_yaw = 0.5 * yaw
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(half_yaw)
                pose.pose.orientation.w = math.cos(half_yaw)
                self.get_logger().info(
                    f"Adjusted goal yaw toward target center using '{base_frame}'"
                )
                return
            except Exception:
                continue

    def _goal_signature(self, pose: PoseStamped) -> tuple[float, float, float]:
        yaw = self._yaw_from_quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return (float(pose.pose.position.x), float(pose.pose.position.y), yaw)

    def _goal_is_duplicate(self, signature: tuple[float, float, float]) -> bool:
        if self._active_goal_signature is None:
            return False
        dx = signature[0] - self._active_goal_signature[0]
        dy = signature[1] - self._active_goal_signature[1]
        dyaw = math.atan2(
            math.sin(signature[2] - self._active_goal_signature[2]),
            math.cos(signature[2] - self._active_goal_signature[2]),
        )
        return (
            math.hypot(dx, dy) <= self.duplicate_goal_position_tol
            and abs(dyaw) <= self.duplicate_goal_yaw_tol
        )

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_goal_failed(self) -> None:
        if self.goal_failed_publisher is None:
            return
        self.goal_failed_publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = LocationSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
