import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudRestamper(Node):
    def __init__(self) -> None:
        super().__init__("go2_pointcloud_restamper")
        self.declare_parameter("input_topic", "/utlidar/cloud_base")
        self.declare_parameter("output_topic", "/utlidar/cloud_base_restamped")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Restamping point clouds {input_topic} -> {output_topic}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
