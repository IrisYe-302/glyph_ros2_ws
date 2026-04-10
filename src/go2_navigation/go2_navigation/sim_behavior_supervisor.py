import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class SimBehaviorSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_behavior_supervisor")

        self.declare_parameter("target_topic", "/move_base_simple/goal")
        self.declare_parameter("target_location_topic", "/target_location")
        self.declare_parameter("dispatch_target_topic", "/behavior_supervisor_dispatch_goal")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("target_wait_sec", 10.0)
        self.declare_parameter("return_home_trigger_topic", "/return_home_trigger")
        self.declare_parameter("home_target_topic", "/return_home_target_location")
        self.declare_parameter("body_motion_topic", "/sim_body_motion")
        self.declare_parameter("goal_cleared_topic", "/target_location_cleared")
        self.declare_parameter("queue_marker_topic", "/behavior_supervisor_queue")
        self.declare_parameter("set_home_topic", "/set_home_here")
        self.declare_parameter("return_home_delay_sec", 5.0)
        self.declare_parameter("target_reached_radius", 0.5)
        self.declare_parameter("home_reached_radius", 0.25)
        self.declare_parameter("home_reached_yaw_tol", 0.2)
        self.declare_parameter("home_resume_radius", 0.35)
        self.declare_parameter("home_resume_yaw_tol", 0.35)
        self.declare_parameter("dance_drift_radius", 0.60)
        self.declare_parameter("dance_drift_yaw_tol", 0.60)
        self.declare_parameter("dance_enabled", True)
        self.declare_parameter("dance_min_interval_sec", 4.0)
        self.declare_parameter("dance_max_interval_sec", 10.0)
        self.declare_parameter("dance_min_duration_sec", 3.0)
        self.declare_parameter("dance_max_duration_sec", 8.0)
        self.declare_parameter("command_quiet_sec", 4.0)
        self.declare_parameter("return_home_republish_sec", 0.5)
        self.declare_parameter("home_back_in_distance", 1.4)
        self.declare_parameter("home_back_in_yaw_tol", 1.0)
        self.declare_parameter("home_back_in_bearing_min_abs", 1.9)

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
        self.dance_drift_radius = float(self.get_parameter("dance_drift_radius").value)
        self.dance_drift_yaw_tol = float(self.get_parameter("dance_drift_yaw_tol").value)
        self.dance_enabled = bool(self.get_parameter("dance_enabled").value)
        self.dance_min_interval_sec = float(self.get_parameter("dance_min_interval_sec").value)
        self.dance_max_interval_sec = float(self.get_parameter("dance_max_interval_sec").value)
        self.dance_min_duration_sec = float(self.get_parameter("dance_min_duration_sec").value)
        self.dance_max_duration_sec = float(self.get_parameter("dance_max_duration_sec").value)
        self.command_quiet_sec = float(self.get_parameter("command_quiet_sec").value)
        self.return_home_republish_sec = float(self.get_parameter("return_home_republish_sec").value)
        self.home_back_in_distance = float(self.get_parameter("home_back_in_distance").value)
        self.home_back_in_yaw_tol = float(self.get_parameter("home_back_in_yaw_tol").value)
        self.home_back_in_bearing_min_abs = float(
            self.get_parameter("home_back_in_bearing_min_abs").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame_candidates = ["base_footprint", "base_link"]

        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.body_motion_pub = self.create_publisher(
            String,
            str(self.get_parameter("body_motion_topic").value),
            10,
        )
        self.dispatch_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("dispatch_target_topic").value),
            target_qos,
        )
        self.home_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("home_target_topic").value),
            target_qos,
        )
        self.queue_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("queue_marker_topic").value),
            target_qos,
        )

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
        self.create_subscription(
            Bool,
            str(self.get_parameter("set_home_topic").value),
            self._on_set_home_here,
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("goal_cleared_topic").value),
            self._on_goal_cleared,
            10,
        )

        self.random = random.Random()
        self.last_command_time = self.get_clock().now()
        self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
        self.dance_deadline_ns: Optional[int] = None
        self.dwell_deadline_ns: Optional[int] = None
        self.command_queue: list[PoseStamped] = []
        self.active_target_pose: Optional[PoseStamped] = None
        self.returning_home = False
        self.return_home_pending = False
        self.return_home_signal_high = False
        self.return_home_authorized = False
        self.return_home_start_deadline_ns: Optional[int] = None
        self.last_home_publish_ns: Optional[int] = None
        self.home_final_align_sent = False
        self.dance_mode = "dance1"
        self.at_home_last = True
        self.dance_recovery_pending = False

        self.timer = self.create_timer(0.25, self._tick)
        self.get_logger().info("Sim behavior supervisor active")
        self._publish_queue_markers()

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
        self.dance_recovery_pending = False
        self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
        self._publish_stop()
        self.return_home_authorized = False

        if self._is_near_point(target_xy[0], target_xy[1], self.home_reached_radius):
            self.get_logger().info("Ignoring explicit home goal because home is implicit at the end of the queue")
            self._publish_queue_markers()
            return

        self.command_queue.append(pose)
        if self.returning_home or self.return_home_pending:
            self.returning_home = False
            self.return_home_pending = False
            self.return_home_authorized = False
            self.return_home_start_deadline_ns = None
            self.last_home_publish_ns = None
            self.home_final_align_sent = False
        else:
            self.return_home_pending = False
            self.return_home_start_deadline_ns = None
        self.dwell_deadline_ns = None
        self._publish_queue_markers()

    def _on_return_home_trigger(self, msg: Bool) -> None:
        self.return_home_signal_high = bool(msg.data)
        if self.return_home_signal_high:
            self.return_home_authorized = True
        self._publish_queue_markers()

    def _on_set_home_here(self, msg: Bool) -> None:
        if not msg.data:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Ignoring set-home request because robot pose is unavailable")
            return

        self.home_x = robot_pose[0]
        self.home_y = robot_pose[1]
        self.home_yaw = robot_pose[2]
        self.returning_home = False
        self.return_home_pending = False
        self.return_home_authorized = False
        self.return_home_start_deadline_ns = None
        self.last_home_publish_ns = None
        self.home_final_align_sent = False
        self.dance_recovery_pending = False
        self.last_command_time = self.get_clock().now()
        self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
        self.command_queue.clear()
        self.active_target_pose = None
        self.dwell_deadline_ns = None
        self.get_logger().info(
            f"Updated home pose to x={self.home_x:.2f}, y={self.home_y:.2f}, yaw={self.home_yaw:.2f}"
        )
        self._publish_queue_markers()

    def _on_goal_cleared(self, _: Empty) -> None:
        if self.active_target_pose is not None:
            now_ns = self.get_clock().now().nanoseconds
            target_x = self.active_target_pose.pose.position.x
            target_y = self.active_target_pose.pose.position.y
            if not self._is_near_point(target_x, target_y, self.target_reached_radius):
                self.get_logger().info("Ignoring stale goal-cleared event for non-active target")
                return
            if self.target_wait_sec > 0.0:
                self.dwell_deadline_ns = now_ns + int(self.target_wait_sec * 1e9)
            else:
                self._complete_active_target(now_ns)
            return

        if not self.returning_home:
            return

        if self.dance_recovery_pending:
            if self._is_at_home_resume():
                self.returning_home = False
                self.dance_recovery_pending = False
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self.last_command_time = self.get_clock().now()
                self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
                self._publish_queue_markers()
            return

        if self._is_at_home_exact() or self._is_at_home_resume():
            self.returning_home = False
            self.last_home_publish_ns = None
            self.home_final_align_sent = False
            self.last_command_time = self.get_clock().now()
            self.next_dance_ns = self._schedule_next_dance(self.last_command_time.nanoseconds)
            self._publish_queue_markers()

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

    def _is_within_dance_drift_limit(self) -> bool:
        return self._is_at_home(self.dance_drift_radius, self.dance_drift_yaw_tol)

    def _schedule_next_dance(self, now_ns: int) -> int:
        wait_sec = self.random.uniform(self.dance_min_interval_sec, self.dance_max_interval_sec)
        return now_ns + int(wait_sec * 1e9)

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        at_home = self._is_at_home_resume()
        self.at_home_last = at_home

        if self.active_target_pose is not None:
            self._publish_stop()
            if self.dwell_deadline_ns is not None and now_ns >= self.dwell_deadline_ns:
                self._complete_active_target(now_ns)
            return

        if self.return_home_pending:
            self._publish_stop()
            if self.return_home_signal_high and self.return_home_start_deadline_ns is None:
                self.return_home_start_deadline_ns = now_ns + int(self.return_home_delay_sec * 1e9)
                self.get_logger().info(
                    f"Pin 7 HIGH; dispatching home after {self.return_home_delay_sec:.1f}s"
                )
            elif not self.return_home_signal_high:
                self.return_home_start_deadline_ns = None
            if self.return_home_start_deadline_ns is not None and now_ns >= self.return_home_start_deadline_ns:
                self.return_home_pending = False
                self.returning_home = True
                self.return_home_authorized = False
                self.return_home_start_deadline_ns = None
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self._publish_home_goal()
                self._publish_queue_markers()
            return

        if self.returning_home:
            self._publish_stop()
            if self.dance_recovery_pending and self._is_at_home_resume():
                self.returning_home = False
                self.dance_recovery_pending = False
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self.last_command_time = now
                self.next_dance_ns = self._schedule_next_dance(now_ns)
            elif self._is_at_home_exact():
                self.returning_home = False
                self.dance_recovery_pending = False
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self.last_command_time = now
                self.next_dance_ns = self._schedule_next_dance(now_ns)
            elif (
                not self.home_final_align_sent
                and self._is_near_point(self.home_x, self.home_y, self.home_resume_radius)
            ):
                self._publish_home_goal()
                self.home_final_align_sent = True
            elif (
                self.last_home_publish_ns is None
                or now_ns - self.last_home_publish_ns
                >= int(self.return_home_republish_sec * 1e9)
            ):
                self._publish_home_goal()
            return

        if self.command_queue:
            self._publish_stop()
            if self.return_home_signal_high:
                self._dispatch_next_goal()
            return

        if not self.dance_enabled or not at_home:
            if (
                self.dance_recovery_pending
                and not self.returning_home
                and not self.return_home_pending
                and not self._is_at_home_exact()
            ):
                self.returning_home = True
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self._publish_home_goal()
                return
            self._publish_stop()
            return

        quiet_for_sec = (now - self.last_command_time).nanoseconds / 1e9
        if quiet_for_sec < self.command_quiet_sec:
            self._publish_stop()
            return

        if self.dance_deadline_ns is not None:
            if now_ns < self.dance_deadline_ns:
                if not self._is_within_dance_drift_limit():
                    self.dance_deadline_ns = None
                    self._publish_stop()
                    self.dance_recovery_pending = True
                    self.returning_home = True
                    self.last_home_publish_ns = None
                    self.home_final_align_sent = False
                    self._publish_home_goal()
                    return
                self._publish_dance()
                return
            self.dance_deadline_ns = None
            self._publish_stop()
            if not self._is_at_home_exact():
                self.dance_recovery_pending = True
                self.returning_home = True
                self.last_home_publish_ns = None
                self.home_final_align_sent = False
                self._publish_home_goal()
                return
            self.next_dance_ns = self._schedule_next_dance(now_ns)
            return

        if now_ns >= self.next_dance_ns:
            duration_sec = self.random.uniform(self.dance_min_duration_sec, self.dance_max_duration_sec)
            self.dance_deadline_ns = now_ns + int(duration_sec * 1e9)
            self.dance_mode = self.random.choice(["dance1", "dance2"])
            self._publish_dance()
            return

        self._publish_stop()

    def _publish_home_goal(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.home_x
        msg.pose.position.y = self.home_y
        msg.pose.position.z = 0.0
        yaw = self.home_yaw
        robot_pose = self._lookup_robot_pose()
        if robot_pose is not None:
            dx = self.home_x - robot_pose[0]
            dy = self.home_y - robot_pose[1]
            distance_sq = dx * dx + dy * dy
            if distance_sq > self.home_resume_radius * self.home_resume_radius:
                bearing_to_home = math.atan2(dy, dx)
                relative_bearing = self._wrap_angle(bearing_to_home - robot_pose[2])
                yaw_error_to_home_target = abs(self._wrap_angle(robot_pose[2] - self.home_yaw))
                can_back_into_home = (
                    distance_sq <= self.home_back_in_distance * self.home_back_in_distance
                    and yaw_error_to_home_target <= self.home_back_in_yaw_tol
                    and abs(relative_bearing) >= self.home_back_in_bearing_min_abs
                )
                if not can_back_into_home:
                    yaw = bearing_to_home
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.home_goal_pub.publish(msg)
        self.last_home_publish_ns = self.get_clock().now().nanoseconds
        self.last_command_time = self.get_clock().now()

    def _queue_return_home(self) -> None:
        if self._is_at_home_resume():
            self.returning_home = False
            self.return_home_pending = False
            self.return_home_start_deadline_ns = None
            self._publish_queue_markers()
            return
        self.returning_home = False
        self.return_home_pending = True
        self.return_home_start_deadline_ns = None
        self.last_home_publish_ns = None
        self.home_final_align_sent = False
        self._publish_queue_markers()

    def _publish_dance(self) -> None:
        msg = String()
        msg.data = self.dance_mode
        self.body_motion_pub.publish(msg)

    def _publish_stop(self) -> None:
        msg = String()
        msg.data = "stop"
        self.body_motion_pub.publish(msg)

    def _dispatch_next_goal(self) -> None:
        if not self.command_queue:
            return
        self.active_target_pose = self.command_queue.pop(0)
        self.active_target_pose.header.stamp = self.get_clock().now().to_msg()
        self.dwell_deadline_ns = None
        self.return_home_authorized = False
        self.dispatch_goal_pub.publish(self.active_target_pose)
        self.last_command_time = self.get_clock().now()
        self._publish_queue_markers()

    def _complete_active_target(self, now_ns: int) -> None:
        self.active_target_pose = None
        self.dwell_deadline_ns = None
        self.last_command_time = self.get_clock().now()
        if self.command_queue:
            self._publish_queue_markers()
        else:
            self._queue_return_home()
            self._publish_queue_markers()

    def _publish_queue_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = self.goal_frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0

        def add_goal_marker(goal: PoseStamped, text: str, r: float, g: float, b: float) -> None:
            nonlocal marker_id
            sphere = Marker()
            sphere.header.frame_id = self.goal_frame_id
            sphere.header.stamp = stamp
            sphere.ns = "behavior_supervisor_queue"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = goal.pose
            sphere.pose.position.z = 0.10
            sphere.scale.x = 0.20
            sphere.scale.y = 0.20
            sphere.scale.z = 0.20
            sphere.color.a = 0.85
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.goal_frame_id
            label.header.stamp = stamp
            label.ns = "behavior_supervisor_queue_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = goal.pose
            label.pose.position.z = 0.45
            label.scale.z = 0.18
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.text = text
            markers.markers.append(label)

        if self.active_target_pose is not None:
            add_goal_marker(self.active_target_pose, "ACTIVE", 0.94, 0.51, 0.33)

        for index, goal in enumerate(self.command_queue, start=1):
            add_goal_marker(goal, f"Q{index}", 0.95, 0.7, 0.1)

        robot_pose = self._lookup_robot_pose()
        if robot_pose is not None:
            status = Marker()
            status.header.frame_id = self.goal_frame_id
            status.header.stamp = stamp
            status.ns = "behavior_supervisor_status"
            status.id = marker_id
            status.type = Marker.TEXT_VIEW_FACING
            status.action = Marker.ADD
            status.pose.position.x = robot_pose[0]
            status.pose.position.y = robot_pose[1]
            status.pose.position.z = 0.85
            status.pose.orientation.w = 1.0
            status.scale.z = 0.16
            status.color.a = 1.0
            status.color.r = 0.9
            status.color.g = 0.95
            status.color.b = 1.0
            if self.active_target_pose is not None:
                status.text = f"Executing | queued {len(self.command_queue)}"
            elif self.command_queue:
                gate = "HIGH" if self.return_home_signal_high else "LOW"
                status.text = f"Queued {len(self.command_queue)} | pin {gate}"
            elif self.returning_home:
                status.text = "Returning home"
            elif self.return_home_pending:
                gate = "HIGH" if self.return_home_signal_high else "LOW"
                status.text = f"Home queued | pin {gate}"
            else:
                gate = "HIGH" if self.return_home_signal_high else "LOW"
                status.text = f"At home | pin {gate}"
            markers.markers.append(status)

        self.queue_marker_pub.publish(markers)


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
