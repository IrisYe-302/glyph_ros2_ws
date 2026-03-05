import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('go2_location_subscriber')
        
        self.declare_parameter('target_topic', '/target_location')
        target_topic = self.get_parameter('target_topic').value
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            target_topic,
            self.location_callback,
            10
        )
        
        self.get_logger().info(f'Listening for target locations on {target_topic}')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected')

    def location_callback(self, msg):
        self.get_logger().info(
            f'Received target: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}'
        )
        self.send_navigation_goal(msg)

    def send_navigation_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.frame_id = 'map'
        
        self.get_logger().info(f'Sending robot to target...')
        
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = LocationSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
