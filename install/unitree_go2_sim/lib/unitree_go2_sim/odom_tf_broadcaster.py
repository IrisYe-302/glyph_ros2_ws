#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self._tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter("publish_odom_path", False)
        self.declare_parameter("max_path_points", 2000)
        self.declare_parameter("path_publish_period", 0.2)
        self.declare_parameter("path_min_distance", 0.03)
        self._publish_odom_path = bool(self.get_parameter("publish_odom_path").value)
        self._max_path_points = int(self.get_parameter("max_path_points").value)
        self._path_publish_period = float(self.get_parameter("path_publish_period").value)
        self._path_min_distance = float(self.get_parameter("path_min_distance").value)
        self._path_publisher = self.create_publisher(Path, "/odom_path", 10)
        self._path = Path()
        self._path.header.frame_id = "odom"
        self._last_path_time = None
        self._last_path_position = None
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

    def _odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        if self._publish_odom_path:
            position = msg.pose.pose.position
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self._last_path_time is not None:
                elapsed = (stamp_ns - self._last_path_time) / 1_000_000_000.0
                dx = position.x - self._last_path_position[0]
                dy = position.y - self._last_path_position[1]
                dz = position.z - self._last_path_position[2]
                distance = (dx * dx + dy * dy + dz * dz) ** 0.5
                if elapsed < self._path_publish_period and distance < self._path_min_distance:
                    return

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self._path.header = msg.header
            self._path.poses.append(pose)
            self._last_path_time = stamp_ns
            self._last_path_position = (position.x, position.y, position.z)
            if len(self._path.poses) > self._max_path_points:
                self._path.poses = self._path.poses[-self._max_path_points :]
            self._path_publisher.publish(self._path)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
