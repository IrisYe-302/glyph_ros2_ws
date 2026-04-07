import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class GlobalLocalizationTrigger(Node):
    def __init__(self) -> None:
        super().__init__("go2_global_localization_trigger")
        self.declare_parameter("service_name", "/reinitialize_global_localization")
        self.declare_parameter("delay_sec", 8.0)

        self.service_name = str(self.get_parameter("service_name").value)
        delay_sec = float(self.get_parameter("delay_sec").value)

        self.client = self.create_client(Empty, self.service_name)
        self.timer = self.create_timer(delay_sec, self._trigger_once)
        self.sent = False

    def _trigger_once(self) -> None:
        if self.sent:
            return

        if not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(
                f"Waiting for AMCL global localization service '{self.service_name}'"
            )
            return

        future = self.client.call_async(Empty.Request())
        future.add_done_callback(self._handle_response)
        self.sent = True
        self.timer.cancel()
        self.get_logger().info("Requested AMCL global localization")

    def _handle_response(self, future) -> None:
        try:
            future.result()
            self.get_logger().info("AMCL global localization request completed")
        except Exception as exc:  # pragma: no cover - ROS async error path
            self.get_logger().error(f"AMCL global localization request failed: {exc}")


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
