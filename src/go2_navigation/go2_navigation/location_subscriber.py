# ros2_ws/src/go2_navigation/go2_navigation/location_subscriber.py
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener

class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('go2_location_subscriber')

        # declare parameter with default and read it safely as a string
        self.declare_parameter('target_topic', '/target_location')
        self.declare_parameter('goal_frame_id', 'map')
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.goal_frame_id = self.get_parameter('goal_frame_id').get_parameter_value().string_value

        # action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_ready = False
        self.frame_ready = False
        self.base_frame_candidates = ["base_footprint", "base_link"]

        # subscription
        self.subscription = self.create_subscription(
            PoseStamped,
            target_topic,
            self.location_callback,
            10
        )
        self.goal_cleared_publisher = self.create_publisher(Empty, '/target_location_cleared', 10)

        self.get_logger().info(f'Listening for target locations on {target_topic}')
        self.get_logger().info('Waiting for Nav2 action server and goal frame...')
        self.readiness_timer = self.create_timer(0.5, self._check_readiness)

    def _check_readiness(self):
        if not self.nav_ready:
            self.nav_ready = self.nav_client.wait_for_server(timeout_sec=0.0)
            if self.nav_ready:
                self.get_logger().info('Nav2 action server connected')

        if not self.frame_ready:
            for base_frame in self.base_frame_candidates:
                if self.tf_buffer.can_transform(
                    self.goal_frame_id,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.0),
                ):
                    self.frame_ready = True
                    self.get_logger().info(
                        f"Goal frame '{self.goal_frame_id}' is available via '{base_frame}'"
                    )
                    break

        if self.nav_ready and self.frame_ready:
            self.readiness_timer.cancel()

    def location_callback(self, msg: PoseStamped):
        if not self.nav_ready or not self.frame_ready:
            self.get_logger().warn(
                f"Ignoring target until Nav2 and frame '{self.goal_frame_id}' are ready"
            )
            return

        self.get_logger().info(
            f'Received target: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}'
        )
        self.send_navigation_goal(msg)

    def send_navigation_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        if not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = self.goal_frame_id

        self.get_logger().info('Sending robot to target...')

        # send goal async; feedback callback is optional
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.goal_cleared_publisher.publish(Empty())
            return

        self.get_logger().info('Goal accepted — waiting for result...')
        # request result and set callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def feedback_callback(self, feedback_msg):
        # feedback_msg is a GoalHandle feedback object in rclpy
        feedback = feedback_msg.feedback
        # nav2 feedback usually has distance_remaining
        try:
            self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f} m')
        except Exception:
            # be defensive if feedback doesn't have that field
            self.get_logger().debug('Received feedback')

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status {status}: {result}')
        self.goal_cleared_publisher.publish(Empty())

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
