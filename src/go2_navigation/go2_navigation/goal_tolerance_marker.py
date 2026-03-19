import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class GoalToleranceMarker(Node):
    def __init__(self) -> None:
        super().__init__("go2_goal_tolerance_marker")

        self.declare_parameter("target_topic", "/target_location")
        self.declare_parameter("marker_topic", "/target_location_tolerance")
        self.declare_parameter("radius", 0.5)
        self.declare_parameter("height", 0.03)
        self.declare_parameter("z_offset", 0.015)

        target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self._radius = self.get_parameter("radius").get_parameter_value().double_value
        self._height = self.get_parameter("height").get_parameter_value().double_value
        self._z_offset = self.get_parameter("z_offset").get_parameter_value().double_value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(Marker, marker_topic, qos)
        self.create_subscription(PoseStamped, target_topic, self._callback, qos)

    def _callback(self, msg: PoseStamped) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.ns = "target_location_tolerance"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.pose.position.z += self._z_offset
        marker.scale.x = 2.0 * self._radius
        marker.scale.y = 2.0 * self._radius
        marker.scale.z = self._height
        marker.color.r = 0.10
        marker.color.g = 0.85
        marker.color.b = 0.35
        marker.color.a = 0.28
        self._publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = GoalToleranceMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
