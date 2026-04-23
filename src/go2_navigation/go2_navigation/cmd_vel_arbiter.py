from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass
class TimedTwist:
    msg: Twist
    stamp_ns: int


class CmdVelArbiter(Node):
    def __init__(self) -> None:
        super().__init__("go2_cmd_vel_arbiter")

        self.declare_parameter("teleop_input_topic", "/cmd_vel")
        self.declare_parameter("nav_input_topic", "/cmd_vel_nav")
        self.declare_parameter("dance_input_topic", "/cmd_vel_dance")
        self.declare_parameter("output_topic", "/cmd_vel_muxed")
        self.declare_parameter("motion_allowed_topic", "/stability_guard/allow_motion")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("teleop_timeout_sec", 0.35)
        self.declare_parameter("teleop_hold_sec", 0.75)
        self.declare_parameter("nav_timeout_sec", 0.30)
        self.declare_parameter("dance_timeout_sec", 0.30)
        self.declare_parameter("teleop_deadband", 0.01)

        teleop_input_topic = str(self.get_parameter("teleop_input_topic").value)
        nav_input_topic = str(self.get_parameter("nav_input_topic").value)
        dance_input_topic = str(self.get_parameter("dance_input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        motion_allowed_topic = str(self.get_parameter("motion_allowed_topic").value)
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))
        self.teleop_timeout_ns = int(float(self.get_parameter("teleop_timeout_sec").value) * 1e9)
        self.teleop_hold_ns = int(float(self.get_parameter("teleop_hold_sec").value) * 1e9)
        self.nav_timeout_ns = int(float(self.get_parameter("nav_timeout_sec").value) * 1e9)
        self.dance_timeout_ns = int(float(self.get_parameter("dance_timeout_sec").value) * 1e9)
        self.teleop_deadband = float(self.get_parameter("teleop_deadband").value)

        self.output_pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, teleop_input_topic, self._on_teleop, 10)
        self.create_subscription(Twist, nav_input_topic, self._on_nav, 10)
        self.create_subscription(Twist, dance_input_topic, self._on_dance, 10)
        self.create_subscription(Bool, motion_allowed_topic, self._on_motion_allowed, 10)

        self._teleop_last: Optional[TimedTwist] = None
        self._nav_last: Optional[TimedTwist] = None
        self._dance_last: Optional[TimedTwist] = None
        self._teleop_override_until_ns: Optional[int] = None
        self._last_selected_source: Optional[str] = None
        self._last_published_zero = False
        self._motion_allowed = True

        self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(
            f"Arbitrating teleop={teleop_input_topic}, nav={nav_input_topic}, "
            f"dance={dance_input_topic} -> {output_topic}"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _twist_is_effective(self, msg: Twist) -> bool:
        return (
            abs(msg.linear.x) > self.teleop_deadband
            or abs(msg.linear.y) > self.teleop_deadband
            or abs(msg.angular.z) > self.teleop_deadband
        )

    def _on_teleop(self, msg: Twist) -> None:
        now_ns = self._now_ns()
        self._teleop_last = TimedTwist(msg=msg, stamp_ns=now_ns)
        # Any teleop packet should preserve operator ownership briefly,
        # including explicit zero commands used to stop.
        self._teleop_override_until_ns = now_ns + self.teleop_hold_ns

    def _on_nav(self, msg: Twist) -> None:
        self._nav_last = TimedTwist(msg=msg, stamp_ns=self._now_ns())

    def _on_dance(self, msg: Twist) -> None:
        self._dance_last = TimedTwist(msg=msg, stamp_ns=self._now_ns())

    def _on_motion_allowed(self, msg: Bool) -> None:
        if self._motion_allowed == bool(msg.data):
            return
        self._motion_allowed = bool(msg.data)
        self.get_logger().info(f"motion_allowed -> {self._motion_allowed}")

    def _is_recent(self, timed: Optional[TimedTwist], timeout_ns: int, now_ns: int) -> bool:
        return timed is not None and (now_ns - timed.stamp_ns) <= timeout_ns

    def _tick(self) -> None:
        now_ns = self._now_ns()

        if not self._motion_allowed:
            if not self._last_published_zero:
                self.output_pub.publish(Twist())
                self._last_published_zero = True
            if self._last_selected_source != "guard_blocked":
                self.get_logger().info("cmd_vel source -> guard_blocked")
                self._last_selected_source = "guard_blocked"
            return

        selected_source: Optional[str] = None
        selected_msg: Optional[Twist] = None

        teleop_hold_active = (
            self._teleop_override_until_ns is not None
            and now_ns <= self._teleop_override_until_ns
        )

        if teleop_hold_active and self._teleop_last is not None:
            selected_source = "teleop"
            if self._is_recent(self._teleop_last, self.teleop_timeout_ns, now_ns):
                selected_msg = self._teleop_last.msg
            else:
                # Hold teleop priority without replaying stale motion.
                selected_msg = Twist()
        elif self._is_recent(self._teleop_last, self.teleop_timeout_ns, now_ns) and self._twist_is_effective(self._teleop_last.msg):
            selected_source = "teleop"
            selected_msg = self._teleop_last.msg
        elif self._is_recent(self._nav_last, self.nav_timeout_ns, now_ns):
            selected_source = "nav"
            selected_msg = self._nav_last.msg
        elif self._is_recent(self._dance_last, self.dance_timeout_ns, now_ns):
            selected_source = "dance"
            selected_msg = self._dance_last.msg

        if selected_msg is not None:
            self.output_pub.publish(selected_msg)
            self._last_published_zero = False
        elif not self._last_published_zero:
            self.output_pub.publish(Twist())
            self._last_published_zero = True

        if selected_source != self._last_selected_source:
            self.get_logger().info(f"cmd_vel source -> {selected_source or 'idle'}")
            self._last_selected_source = selected_source


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
