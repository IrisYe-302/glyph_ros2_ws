import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class OdomReadyGate(Node):
    def __init__(self) -> None:
        super().__init__("go2_odom_ready_gate")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("warn_interval_sec", 2.0)

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.warn_interval_ns = int(float(self.get_parameter("warn_interval_sec").value) * 1e9)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._ready = False
        self._odom_received = False
        self._last_warn_ns = 0

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(0.25, self._tick)

        self.get_logger().info(
            f"Waiting for TF {self.target_frame} -> {self.base_frame} and odometry on {odom_topic}"
        )

    @property
    def ready(self) -> bool:
        return self._ready

    def _on_odom(self, _: Odometry) -> None:
        self._odom_received = True

    def _tick(self) -> None:
        try:
            if self.tf_buffer.can_transform(
                self.target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            ):
                self._ready = True
                self.get_logger().info(
                    f"TF ready: {self.target_frame} -> {self.base_frame}. Releasing Nav2 startup gate."
                )
                return
        except Exception:
            pass

        now_ns = self.get_clock().now().nanoseconds
        if self.warn_interval_ns > 0 and (now_ns - self._last_warn_ns) >= self.warn_interval_ns:
            odom_state = "seen /odom" if self._odom_received else "no /odom yet"
            self.get_logger().warn(
                f"Still waiting for TF {self.target_frame} -> {self.base_frame} ({odom_state})"
            )
            self._last_warn_ns = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomReadyGate()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
