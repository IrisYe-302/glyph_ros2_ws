import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class SimBodyMotionController(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_body_motion_controller")

        self.declare_parameter("motion_topic", "/sim_body_motion")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("twist_angular_speed", 0.28)
        self.declare_parameter("twist_frequency_hz", 0.55)
        self.declare_parameter("bend_speed", 0.035)
        self.declare_parameter("bend_angular_speed", 0.0)
        self.declare_parameter("publish_hz", 20.0)

        motion_topic = str(self.get_parameter("motion_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.twist_angular_speed = float(self.get_parameter("twist_angular_speed").value)
        self.twist_frequency_hz = float(self.get_parameter("twist_frequency_hz").value)
        self.bend_speed = float(self.get_parameter("bend_speed").value)
        self.bend_angular_speed = float(self.get_parameter("bend_angular_speed").value)
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(String, motion_topic, self._on_motion_command, 10)

        self.current_mode = "stop"
        self._last_published_stop = False
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

    def _on_motion_command(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode not in {"stop", "dance1", "dance2"}:
            self.get_logger().warn(f"Ignoring unsupported body motion mode '{msg.data}'")
            return
        self.current_mode = mode
        self._last_published_stop = False

    def _tick(self) -> None:
        if self.current_mode == "stop":
            self.cmd_pub.publish(Twist())
            self._last_published_stop = True
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()
        if self.current_mode == "dance1":
            phase = math.sin(2.0 * math.pi * self.twist_frequency_hz * now_sec)
            cmd.angular.z = self.twist_angular_speed * phase
        else:
            phase = 2.0 * math.pi * (self.twist_frequency_hz * 0.75) * now_sec
            bend_phase = math.sin(phase)
            cmd.linear.x = self.bend_speed * bend_phase * abs(bend_phase)
            cmd.angular.z = self.bend_angular_speed

        self.cmd_pub.publish(cmd)
        self._last_published_stop = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBodyMotionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
