import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class StabilityGuard(Node):
    def __init__(self) -> None:
        super().__init__("go2_stability_guard")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("allow_motion_topic", "/stability_guard/allow_motion")
        self.declare_parameter("state_topic", "/stability_guard/state")
        self.declare_parameter("recovery_command_topic", "/stability_guard/recovery_command")
        self.declare_parameter("startup_hold_sec", 3.0)
        self.declare_parameter("startup_balance_stand", True)
        self.declare_parameter("stable_dwell_sec", 1.0)
        self.declare_parameter("recovery_hold_sec", 2.0)
        self.declare_parameter("retry_interval_sec", 4.0)
        self.declare_parameter("max_roll_pitch_rad", 0.45)
        self.declare_parameter("max_angular_velocity_rad_s", 1.8)
        self.declare_parameter("accel_deviation_threshold", 3.0)
        self.declare_parameter("max_odom_linear_speed", 0.8)
        self.declare_parameter("picked_up_accel_floor", 6.0)

        imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        allow_motion_topic = str(self.get_parameter("allow_motion_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.recovery_command_topic = str(self.get_parameter("recovery_command_topic").value)

        self.startup_hold_sec = float(self.get_parameter("startup_hold_sec").value)
        self.startup_balance_stand = bool(self.get_parameter("startup_balance_stand").value)
        self.stable_dwell_sec = float(self.get_parameter("stable_dwell_sec").value)
        self.recovery_hold_sec = float(self.get_parameter("recovery_hold_sec").value)
        self.retry_interval_sec = float(self.get_parameter("retry_interval_sec").value)
        self.max_roll_pitch_rad = float(self.get_parameter("max_roll_pitch_rad").value)
        self.max_angular_velocity_rad_s = float(self.get_parameter("max_angular_velocity_rad_s").value)
        self.accel_deviation_threshold = float(self.get_parameter("accel_deviation_threshold").value)
        self.max_odom_linear_speed = float(self.get_parameter("max_odom_linear_speed").value)
        self.picked_up_accel_floor = float(self.get_parameter("picked_up_accel_floor").value)

        self.allow_motion_pub = self.create_publisher(Bool, allow_motion_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.recovery_command_pub = self.create_publisher(String, self.recovery_command_topic, 10)

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._latest_roll: Optional[float] = None
        self._latest_pitch: Optional[float] = None
        self._latest_ang_vel = 0.0
        self._latest_accel_norm = 9.81
        self._latest_odom_speed = 0.0

        now_ns = self.get_clock().now().nanoseconds
        self._startup_release_ns = now_ns + int(self.startup_hold_sec * 1e9)
        self._blocked_until_ns = self._startup_release_ns
        self._last_retry_ns: Optional[int] = None
        self._stable_since_ns: Optional[int] = None
        self._motion_allowed = False
        self._state = "startup_recovering"

        if self.startup_balance_stand:
            self._publish_recovery_command("stop")
            self._publish_recovery_command("balance_stand")
            self._last_retry_ns = now_ns

        self._publish_initial_state()
        self.create_timer(0.1, self._tick)
        self.get_logger().info("Stability guard active")

    def _publish_initial_state(self) -> None:
        allow_motion = Bool()
        allow_motion.data = False
        self.allow_motion_pub.publish(allow_motion)

        state = String()
        state.data = self._state
        self.state_pub.publish(state)

    def _on_imu(self, msg: Imu) -> None:
        roll, pitch = self._quaternion_to_rpy(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )
        self._latest_roll = roll
        self._latest_pitch = pitch
        self._latest_ang_vel = max(
            abs(float(msg.angular_velocity.x)),
            abs(float(msg.angular_velocity.y)),
            abs(float(msg.angular_velocity.z)),
        )
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        self._latest_accel_norm = math.sqrt(ax * ax + ay * ay + az * az)

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self._latest_odom_speed = math.sqrt(
            float(twist.linear.x) ** 2 + float(twist.linear.y) ** 2 + float(twist.linear.z) ** 2
        )

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        unstable_reason = self._current_instability_reason()

        if unstable_reason is not None:
            self._blocked_until_ns = max(self._blocked_until_ns, now_ns + int(self.recovery_hold_sec * 1e9))
            self._stable_since_ns = None
            self._set_state(unstable_reason)
            self._maybe_retry_recovery(now_ns)
        else:
            if self._stable_since_ns is None:
                self._stable_since_ns = now_ns

            startup_done = now_ns >= self._startup_release_ns
            dwell_done = (now_ns - self._stable_since_ns) >= int(self.stable_dwell_sec * 1e9)
            recovery_done = now_ns >= self._blocked_until_ns
            if startup_done and dwell_done and recovery_done:
                self._set_state("stable")
                self._set_motion_allowed(True)
                return
            self._set_state("stabilizing")

        self._set_motion_allowed(False)

    def _current_instability_reason(self) -> Optional[str]:
        if self._latest_roll is None or self._latest_pitch is None:
            return "waiting_for_imu"
        if abs(self._latest_roll) > self.max_roll_pitch_rad or abs(self._latest_pitch) > self.max_roll_pitch_rad:
            return "reorienting"
        if self._latest_ang_vel > self.max_angular_velocity_rad_s:
            return "disturbed"
        if abs(self._latest_accel_norm - 9.81) > self.accel_deviation_threshold:
            if self._latest_accel_norm < self.picked_up_accel_floor:
                return "picked_up"
            return "disturbed"
        if self._latest_odom_speed > self.max_odom_linear_speed:
            return "disturbed"
        return None

    def _maybe_retry_recovery(self, now_ns: int) -> None:
        if self._last_retry_ns is not None and (now_ns - self._last_retry_ns) < int(self.retry_interval_sec * 1e9):
            return
        self._publish_recovery_command("stop")
        self._publish_recovery_command("balance_stand")
        self._last_retry_ns = now_ns

    def _set_motion_allowed(self, allowed: bool) -> None:
        if allowed == self._motion_allowed:
            return
        msg = Bool()
        msg.data = allowed
        self.allow_motion_pub.publish(msg)
        self._motion_allowed = allowed
        self.get_logger().info(f"Motion {'enabled' if allowed else 'blocked'}")

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self._state = state
        self.get_logger().info(f"stability_state={state}")

    def _publish_recovery_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.recovery_command_pub.publish(msg)

    @staticmethod
    def _quaternion_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        return roll, pitch


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StabilityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
