import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8


class SimBodyMotionController(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_body_motion_controller")

        self.declare_parameter("motion_topic", "/sim_body_motion")
        self.declare_parameter("state_topic", "/body_motion_state")
        self.declare_parameter("state_plot_topic", "/body_motion_state_plot")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("twist_angular_speed", 0.60)
        self.declare_parameter("twist_frequency_hz", 0.65)
        self.declare_parameter("bend_speed", 0.085)
        self.declare_parameter("bend_angular_speed", 0.0)
        self.declare_parameter("publish_hz", 20.0)

        motion_topic = str(self.get_parameter("motion_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        state_plot_topic = str(self.get_parameter("state_plot_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.twist_angular_speed = float(self.get_parameter("twist_angular_speed").value)
        self.twist_frequency_hz = float(self.get_parameter("twist_frequency_hz").value)
        self.bend_speed = float(self.get_parameter("bend_speed").value)
        self.bend_angular_speed = float(self.get_parameter("bend_angular_speed").value)
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(UInt8, state_topic, 10)
        self.state_plot_pub = self.create_publisher(Float32, state_plot_topic, 10)
        self.create_subscription(String, motion_topic, self._on_motion_command, 10)

        self.current_mode = "stop"
        self._last_published_stop = False
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

    def _on_motion_command(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode == "arrival_twist":
            mode = "dance1"
        if mode not in {"stop", "dance1", "dance2"}:
            self.get_logger().warn(f"Ignoring unsupported body motion mode '{msg.data}'")
            return
        self.current_mode = mode
        self._last_published_stop = False

    def _tick(self) -> None:
        state = UInt8()
        state_plot = Float32()
        if self.current_mode == "stop":
            if not self._last_published_stop:
                self.cmd_pub.publish(Twist())
            state.data = 0
            state_plot.data = 0.0
            self.state_pub.publish(state)
            self.state_plot_pub.publish(state_plot)
            self._last_published_stop = True
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()
        if self.current_mode == "dance1":
            phase = math.sin(2.0 * math.pi * self.twist_frequency_hz * now_sec)
            cmd.angular.z = self.twist_angular_speed * phase
            state.data = 1
            state_plot.data = 1.0
        else:
            phase = 2.0 * math.pi * (self.twist_frequency_hz * 0.75) * now_sec
            bend_phase = math.sin(phase)
            cmd.linear.x = self.bend_speed * bend_phase * abs(bend_phase)
            cmd.angular.z = self.bend_angular_speed
            state.data = 2
            state_plot.data = 2.0

        self.cmd_pub.publish(cmd)
        self.state_pub.publish(state)
        self.state_plot_pub.publish(state_plot)
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
