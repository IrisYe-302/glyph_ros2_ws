from copy import deepcopy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class SessionMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("go2_session_map_publisher")

        self.declare_parameter("base_map_topic", "/map")
        self.declare_parameter("overlay_topic", "/global_costmap/costmap")
        self.declare_parameter("output_topic", "/session_map")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("overlay_threshold", 20)

        base_map_topic = str(self.get_parameter("base_map_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._overlay_threshold = int(self.get_parameter("overlay_threshold").value)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        volatile_qos = QoSProfile(depth=1)
        volatile_qos.reliability = ReliabilityPolicy.RELIABLE
        volatile_qos.durability = DurabilityPolicy.VOLATILE

        self._publisher = self.create_publisher(OccupancyGrid, output_topic, transient_qos)
        self._base_map = None
        self._overlay = None

        self.create_subscription(
            OccupancyGrid,
            base_map_topic,
            self._handle_base_map,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            overlay_topic,
            self._handle_overlay,
            volatile_qos,
        )

    def _handle_base_map(self, msg: OccupancyGrid) -> None:
        self._base_map = msg
        self._publish_if_ready()

    def _handle_overlay(self, msg: OccupancyGrid) -> None:
        self._overlay = msg
        self._publish_if_ready()

    def _publish_if_ready(self) -> None:
        if self._base_map is None or self._overlay is None:
            return

        if not self._compatible(self._base_map, self._overlay):
            self.get_logger().warn(
                "Base map and global costmap geometry differ; not publishing merged session map"
            )
            return

        merged = deepcopy(self._base_map)
        merged.header.stamp = self.get_clock().now().to_msg()

        merged_data = list(merged.data)
        overlay_data = self._overlay.data

        for i, base_value in enumerate(merged_data):
            overlay_value = int(overlay_data[i])
            if overlay_value < 0:
                continue
            if overlay_value >= self._overlay_threshold:
                merged_data[i] = 100
            elif base_value < 0:
                merged_data[i] = 0
            elif int(base_value) >= self._occupied_threshold:
                merged_data[i] = 100
            else:
                merged_data[i] = 0

        merged.data = merged_data
        self._publisher.publish(merged)

    @staticmethod
    def _compatible(base_map: OccupancyGrid, overlay: OccupancyGrid) -> bool:
        base_info = base_map.info
        overlay_info = overlay.info
        return (
            base_info.width == overlay_info.width
            and base_info.height == overlay_info.height
            and abs(base_info.resolution - overlay_info.resolution) < 1e-6
            and abs(base_info.origin.position.x - overlay_info.origin.position.x) < 1e-6
            and abs(base_info.origin.position.y - overlay_info.origin.position.y) < 1e-6
        )


def main() -> None:
    rclpy.init()
    node = SessionMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
