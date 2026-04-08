import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class SimBehaviorSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_behavior_supervisor")

        self.declare_parameter("target_topic", "/move_base_simple/goal")
        self.declare_parameter("target_location_topic", "/target_location")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("target_wait_sec", 10.0)
        self.declare_parameter("return_home_trigger_topic", "/return_home_trigger")
        self.declare_parameter("return_home_delay_sec", 5.0)
        self.declare_parameter("target_reached_radius", 0.5)
        self.declare_parameter("home_reached_radius", 0.25)
        self.declare_parameter("home_reached_yaw_tol", 0.2)
        self.declare_parameter("home_resume_radius", 0.35)
        self.declare_parameter("home_resume_yaw_tol", 0.35)
        self.declare_parameter("dance_enabled", True)
        self.declare_parameter("dance_angular_speed", 0.45)
        self.declare_parameter("dance_frequency_hz", 0.65)
        self.declare_parameter("dance_bend_speed", 0.10)
        self.declare_parameter("dance_bend_back_bias", 0.02)
        self.declare_parameter("dance_bend_left_bias", 0.06)
        self.declare_parameter("dance_min_interval_sec", 8.0)
        self.declare_parameter("dance_max_interval_sec", 20.0)
        self.declare_parameter("dance_min_duration_sec", 3.0)
        self.declare_parameter("dance_max_duration_sec", 12.0)
        self.declare_parameter("command_quiet_sec", 4.0)
        self.declare_parameter("return_home_republish_sec", 0.5)

        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.home_yaw = float(self.get_parameter("home_yaw").value)
        self.target_wait_sec = float(self.get_parameter("target_wait_sec").value)
        self.return_home_delay_sec = float(self.get_parameter("return_home_delay_sec").value)
        self.target_reached_radius = float(self.get_parameter("target_reached_radius").value)
        self.home_reached_radius = float(self.get_parameter("home_reached_radius").value)
        self.home_reached_yaw_tol = float(self.get_parameter("home_reached_yaw_tol").value)
        self.home_resume_radius = float(self.get_parameter("home_resume_radius").value)
        self.home_resume_yaw_tol = float(self.get_parameter("home_resume_yaw_tol").value)
        self.dance_enabled = bool(self.get_parameter("dance_enabled").value)
        self.dance_angular_speed = float(self.get_parameter("dance_angular_speed").value)
        self.dance_frequency_hz = float(self.get_parameter("dance_frequency_hz").value)
        self.dance_bend_speed = float(self.get_parameter("dance_bend_speed").value)
        self.dance_bend_back_bias = float(self.get_parameter("dance_bend_back_bias").value)
        self.dance_bend_left_bias = float(self.get_parameter("dance_bend_left_bias").value)
        self.dance_min_interval_sec = float(self.get_parameter("dance_min_interval_sec").value)
        self.dance_max_interval_sec = float(self.get_parameter("dance_max_interval_sec").value)
        self.dance_min_duration_sec = float(self.get_parameter("dance_min_duration_sec").value)
        self.dance_max_duration_sec = float(self.get_parameter("dance_max_duration_sec").value)
        self.command_quiet_sec = float(self.get_parameter("command_quiet_sec").value)
        self.return_home_republish_sec = float(self.get_parameter("return_home_republish_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame_candidates = ["base_footprint", "base_link"]

        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.home_goal_pub = self.create_publisher(PoseStamped, "/target_location", target_qos)

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_topic").value),
            self._on_command,
            target_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_location_topic").value),
            self._on_command,
            target_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("return_home_trigger_topic").value),
            self._on_return_home_trigger,
            target_qos,
        )

        self.random = random.Random()
        self.last_command_time = self.get_clock().now()
        self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
        self.dance_deadline_ns: Optional[int] = None
        self.dwell_deadline_ns: Optional[int] = None
        self.active_target: Optional[tuple[float, float]] = None
        self.returning_home = False
        self.return_home_pending = False
        self.return_home_signal_high = False
        self.return_home_start_deadline_ns: Optional[int] = None
        self.last_home_publish_ns: Optional[int] = None
        self.dance_mode = "combo"
        self.at_home_last = True

        self.timer = self.create_timer(0.25, self._tick)
        self.get_logger().info("Sim behavior supervisor active")

    def _on_command(self, msg: PoseStamped) -> None:
        msg_time = Time.from_msg(msg.header.stamp)
        goal_age = (self.get_clock().now() - msg_time).nanoseconds / 1e9
        if goal_age > 2.0:
            return

        pose = self._normalize_pose(msg)
        if pose is None:
            return

        target_xy = (pose.pose.position.x, pose.pose.position.y)
        self.last_command_time = self.get_clock().now()
        self.dance_deadline_ns = None
        self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
        self._publish_stop()

        if self._is_near_point(*target_xy, self.home_reached_radius):
            self.active_target = None
            self._queue_return_home()
            self.dwell_deadline_ns = None
            return

        self.active_target = target_xy
        self.returning_home = False
        self.return_home_pending = False
        self.return_home_start_deadline_ns = None
        self.dwell_deadline_ns = None

    def _on_return_home_trigger(self, msg: Bool) -> None:
        self.return_home_signal_high = bool(msg.data)
        if self.return_home_signal_high and self.return_home_pending and self.return_home_start_deadline_ns is None:
            now_ns = self.get_clock().now().nanoseconds
            self.return_home_start_deadline_ns = now_ns + int(self.return_home_delay_sec * 1e9)
            self.get_logger().info(
                f"Return-home trigger received; delaying home navigation by {self.return_home_delay_sec:.1f}s"
            )

    def _normalize_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        normalized = PoseStamped()
        normalized.header = pose.header
        normalized.pose = pose.pose

        if not normalized.header.frame_id:
            normalized.header.frame_id = self.goal_frame_id

        if normalized.header.frame_id != self.goal_frame_id:
            try:
                normalized = self.tf_buffer.transform(
                    normalized,
                    self.goal_frame_id,
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                return None

        normalized.header.stamp.sec = 0
        normalized.header.stamp.nanosec = 0
        return normalized

    def _lookup_robot_pose(self) -> Optional[tuple[float, float, float]]:
        for base_frame in self.base_frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.goal_frame_id,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                q = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw,
                )
            except Exception:
                continue
        return None

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _is_near_point(self, x: float, y: float, radius: float) -> bool:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        dx = robot_pose[0] - x
        dy = robot_pose[1] - y
        return dx * dx + dy * dy <= radius * radius

    def _is_at_home(self, radius: float, yaw_tol: float) -> bool:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        dx = robot_pose[0] - self.home_x
        dy = robot_pose[1] - self.home_y
        if dx * dx + dy * dy > radius * radius:
            return False
        yaw_error = abs(self._wrap_angle(robot_pose[2] - self.home_yaw))
        return yaw_error <= yaw_tol

    def _is_at_home_exact(self) -> bool:
        return self._is_at_home(self.home_reached_radius, self.home_reached_yaw_tol)

    def _is_at_home_resume(self) -> bool:
        return self._is_at_home(self.home_resume_radius, self.home_resume_yaw_tol)

    def _schedule_next_dance(self, now_ns: int) -> int:
        wait_sec = self.random.uniform(self.dance_min_interval_sec, self.dance_max_interval_sec)
        return now_ns + int(wait_sec * 1e9)

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        at_home = self._is_at_home_resume()
        self.at_home_last = at_home

        if self.active_target is not None:
            self._publish_stop()
            if self._is_near_point(self.active_target[0], self.active_target[1], self.target_reached_radius):
                if self.dwell_deadline_ns is None:
                    self.dwell_deadline_ns = now_ns + int(self.target_wait_sec * 1e9)
                elif now_ns >= self.dwell_deadline_ns:
                    self.active_target = None
                    self._queue_return_home()
                    self.dwell_deadline_ns = None
            else:
                self.dwell_deadline_ns = None
            return

        if self.return_home_pending:
            self._publish_stop()
            if self.return_home_signal_high and self.return_home_start_deadline_ns is None:
                self.return_home_start_deadline_ns = now_ns + int(self.return_home_delay_sec * 1e9)
                self.get_logger().info(
                    f"Return-home trigger received; delaying home navigation by {self.return_home_delay_sec:.1f}s"
                )
            if self.return_home_start_deadline_ns is not None and now_ns >= self.return_home_start_deadline_ns:
                self.return_home_pending = False
                self.returning_home = True
                self.return_home_start_deadline_ns = None
                self.last_home_publish_ns = None
                self._publish_home_goal()
            return

        if self.returning_home:
            self._publish_stop()
            if at_home:
                self.returning_home = False
                self.last_home_publish_ns = None
                self.last_command_time = now
                self.next_dance_ns = self._schedule_next_dance(now_ns)
            elif (
                self.last_home_publish_ns is None
                or (now_ns - self.last_home_publish_ns) >= int(self.return_home_republish_sec * 1e9)
            ):
                self._publish_home_goal()
            return

        if not self.dance_enabled or not at_home:
            self._publish_stop()
            return

        quiet_for_sec = (now - self.last_command_time).nanoseconds / 1e9
        if quiet_for_sec < self.command_quiet_sec:
            self._publish_stop()
            return

        if self.dance_deadline_ns is not None:
            if now_ns < self.dance_deadline_ns:
                self._publish_dance(now.nanoseconds / 1e9)
                return
            self.dance_deadline_ns = None
            self._publish_stop()
            if not self._is_at_home_resume():
                self.returning_home = True
                self.last_home_publish_ns = None
                return
            self.next_dance_ns = self._schedule_next_dance(now_ns)
            return

        if now_ns >= self.next_dance_ns:
            duration_sec = self.random.uniform(self.dance_min_duration_sec, self.dance_max_duration_sec)
            self.dance_deadline_ns = now_ns + int(duration_sec * 1e9)
            self.dance_mode = self.random.choice(["twist", "bend", "combo"])
            self._publish_dance(now.nanoseconds / 1e9)
            return

        self._publish_stop()

    def _publish_home_goal(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.home_x
        msg.pose.position.y = self.home_y
        msg.pose.position.z = 0.0
        robot_pose = self._lookup_robot_pose()
        if robot_pose is not None and not self._is_near_point(self.home_x, self.home_y, self.home_reached_radius):
            yaw = math.atan2(self.home_y - robot_pose[1], self.home_x - robot_pose[0])
        else:
            yaw = self.home_yaw
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.home_goal_pub.publish(msg)
        self.last_home_publish_ns = self.get_clock().now().nanoseconds
        self.last_command_time = self.get_clock().now()

    def _queue_return_home(self) -> None:
        self.returning_home = False
        self.return_home_pending = True
        self.return_home_start_deadline_ns = None
        self.last_home_publish_ns = None

    def _publish_dance(self, now_sec: float) -> None:
        twist = Twist()
        cycle = 2.0 * math.pi * self.dance_frequency_hz * now_sec
        phase = math.sin(cycle)
        deadband = 0.15

        mode = self.dance_mode
        if mode == "combo":
            mode = "twist" if int(now_sec * self.dance_frequency_hz * 2.0) % 2 == 0 else "bend"

        if mode == "twist":
            if abs(phase) >= deadband:
                twist.angular.z = self.dance_angular_speed * math.copysign(1.0, phase)
        else:
            bend_phase = math.sin(2.0 * math.pi * (self.dance_frequency_hz * 1.35) * now_sec)
            if abs(bend_phase) >= deadband:
                twist.linear.x = (
                    self.dance_bend_speed * math.copysign(1.0, bend_phase)
                    - self.dance_bend_back_bias
                )
                twist.angular.z = self.dance_bend_left_bias
        self.cmd_pub.publish(twist)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBehaviorSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
