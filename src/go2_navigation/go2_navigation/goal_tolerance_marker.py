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
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("default_x", 0.0)
        self.declare_parameter("default_y", 0.0)
        self.declare_parameter("default_yaw", 0.0)
        self.declare_parameter("radius", 0.5)
        self.declare_parameter("height", 0.03)
        self.declare_parameter("z_offset", 0.015)

        target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._default_x = self.get_parameter("default_x").get_parameter_value().double_value
        self._default_y = self.get_parameter("default_y").get_parameter_value().double_value
        self._default_yaw = self.get_parameter("default_yaw").get_parameter_value().double_value
        self._radius = self.get_parameter("radius").get_parameter_value().double_value
        self._height = self.get_parameter("height").get_parameter_value().double_value
        self._z_offset = self.get_parameter("z_offset").get_parameter_value().double_value

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        sub_qos = QoSProfile(depth=1)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE
        sub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(Marker, marker_topic, pub_qos)
        self.create_subscription(PoseStamped, target_topic, self._callback, sub_qos)
        self._publish_marker(
            self._frame_id,
            self._default_x,
            self._default_y,
            self._default_yaw,
        )

    def _callback(self, msg: PoseStamped) -> None:
        self._publish_marker(
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            0.0,
        )

    def _publish_marker(self, frame_id: str, x: float, y: float, yaw: float) -> None:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_location_tolerance"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self._z_offset
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
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
