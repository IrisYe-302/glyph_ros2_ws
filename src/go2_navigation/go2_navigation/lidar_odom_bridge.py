import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class LidarOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_lidar_odom_bridge")

        self.declare_parameter("input_topic", "/utlidar/robot_odom")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_current_time", True)
        self.declare_parameter("smooth_xy_alpha", 0.25)
        self.declare_parameter("smooth_yaw_alpha", 0.2)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_current_time = bool(self.get_parameter("use_current_time").value)
        self.smooth_xy_alpha = float(self.get_parameter("smooth_xy_alpha").value)
        self.smooth_yaw_alpha = float(self.get_parameter("smooth_yaw_alpha").value)

        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, input_topic, self._on_odom, 10)
        self._smoothed_x = None
        self._smoothed_y = None
        self._smoothed_yaw = None

    @staticmethod
    def _rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _smooth_value(self, previous: float | None, current: float, alpha: float) -> float:
        if previous is None:
            return current
        return previous + alpha * (current - previous)

    def _smooth_yaw(self, current_yaw: float) -> float:
        if self._smoothed_yaw is None:
            self._smoothed_yaw = current_yaw
            return current_yaw
        delta = self._wrap_angle(current_yaw - self._smoothed_yaw)
        self._smoothed_yaw = self._wrap_angle(self._smoothed_yaw + self.smooth_yaw_alpha * delta)
        return self._smoothed_yaw

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._rpy_from_quaternion(q.x, q.y, q.z, q.w)
        smoothed_x = self._smooth_value(self._smoothed_x, msg.pose.pose.position.x, self.smooth_xy_alpha)
        smoothed_y = self._smooth_value(self._smoothed_y, msg.pose.pose.position.y, self.smooth_xy_alpha)
        smoothed_yaw = self._smooth_yaw(yaw)
        self._smoothed_x = smoothed_x
        self._smoothed_y = smoothed_y

        planar_qx, planar_qy, planar_qz, planar_qw = self._quaternion_from_rpy(0.0, 0.0, smoothed_yaw)
        body_qx, body_qy, body_qz, body_qw = self._quaternion_from_rpy(roll, pitch, 0.0)
        stamp = self.get_clock().now().to_msg() if self.use_current_time else msg.header.stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = smoothed_x
        odom.pose.pose.position.y = smoothed_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = planar_qx
        odom.pose.pose.orientation.y = planar_qy
        odom.pose.pose.orientation.z = planar_qz
        odom.pose.pose.orientation.w = planar_qw
        odom.twist.twist = msg.twist.twist
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        planar_tf = TransformStamped()
        planar_tf.header.stamp = stamp
        planar_tf.header.frame_id = self.odom_frame
        planar_tf.child_frame_id = self.base_frame
        planar_tf.transform.translation.x = smoothed_x
        planar_tf.transform.translation.y = smoothed_y
        planar_tf.transform.translation.z = 0.0
        planar_tf.transform.rotation.x = planar_qx
        planar_tf.transform.rotation.y = planar_qy
        planar_tf.transform.rotation.z = planar_qz
        planar_tf.transform.rotation.w = planar_qw

        body_tf = TransformStamped()
        body_tf.header.stamp = stamp
        body_tf.header.frame_id = self.base_frame
        body_tf.child_frame_id = self.body_frame
        body_tf.transform.translation.x = 0.0
        body_tf.transform.translation.y = 0.0
        body_tf.transform.translation.z = msg.pose.pose.position.z
        body_tf.transform.rotation.x = body_qx
        body_tf.transform.rotation.y = body_qy
        body_tf.transform.rotation.z = body_qz
        body_tf.transform.rotation.w = body_qw

        self.tf_broadcaster.sendTransform([planar_tf, body_tf])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
