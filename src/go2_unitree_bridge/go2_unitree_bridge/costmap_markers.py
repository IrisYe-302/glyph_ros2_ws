import rclpy
from geometry_msgs.msg import Point
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class CostmapMarkers(Node):
    def __init__(self) -> None:
        super().__init__("go2_costmap_markers")

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        sub_qos = QoSProfile(depth=1)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE
        sub_qos.durability = DurabilityPolicy.VOLATILE

        self._local_pub = self.create_publisher(
            MarkerArray, "/local_costmap/markers", pub_qos
        )
        self._global_pub = self.create_publisher(
            MarkerArray, "/global_costmap/markers", pub_qos
        )

        self.create_subscription(
            Costmap, "/local_costmap/costmap", self._local_callback, sub_qos
        )
        self.create_subscription(
            Costmap, "/global_costmap/costmap", self._global_callback, sub_qos
        )

    def _local_callback(self, msg: Costmap) -> None:
        self._local_pub.publish(self._to_markers(msg, "local_costmap"))

    def _global_callback(self, msg: Costmap) -> None:
        self._global_pub.publish(self._to_markers(msg, "global_costmap"))

    def _to_markers(self, msg: Costmap, namespace: str) -> MarkerArray:
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.header = msg.header
        delete_marker.ns = namespace
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        occupied = Marker()
        occupied.header = msg.header
        occupied.ns = namespace
        occupied.id = 1
        occupied.type = Marker.CUBE_LIST
        occupied.action = Marker.ADD
        occupied.pose.orientation.w = 1.0
        occupied.scale.x = msg.metadata.resolution
        occupied.scale.y = msg.metadata.resolution
        occupied.scale.z = 0.03
        occupied.color.r = 0.95
        occupied.color.g = 0.25
        occupied.color.b = 0.15
        occupied.color.a = 0.65

        inflated = Marker()
        inflated.header = msg.header
        inflated.ns = namespace
        inflated.id = 2
        inflated.type = Marker.CUBE_LIST
        inflated.action = Marker.ADD
        inflated.pose.orientation.w = 1.0
        inflated.scale.x = msg.metadata.resolution
        inflated.scale.y = msg.metadata.resolution
        inflated.scale.z = 0.02
        inflated.color.r = 1.0
        inflated.color.g = 0.85
        inflated.color.b = 0.2
        inflated.color.a = 0.28

        res = float(msg.metadata.resolution)
        ox = float(msg.metadata.origin.position.x)
        oy = float(msg.metadata.origin.position.y)
        oz = float(msg.metadata.origin.position.z)
        width = int(msg.metadata.size_x)

        for idx, value in enumerate(msg.data):
            if value <= 0 or value >= 255:
                continue

            x = idx % width
            y = idx // width
            point = Point()
            point.x = ox + (x + 0.5) * res
            point.y = oy + (y + 0.5) * res
            point.z = oz

            if value >= 253:
                occupied.points.append(point)
            elif value >= 80:
                inflated.points.append(point)

        if occupied.points:
            markers.markers.append(occupied)
        if inflated.points:
            markers.markers.append(inflated)
        return markers


def main() -> None:
    rclpy.init()
    node = CostmapMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
