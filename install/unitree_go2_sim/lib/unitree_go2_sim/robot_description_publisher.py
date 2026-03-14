#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import sys
import os
from rclpy.qos import QoSProfile, DurabilityPolicy


def get_xacro_path(param_xacro_path):
    if param_xacro_path and os.path.exists(param_xacro_path):
        return param_xacro_path
    
    search_paths = [
        '/home/ming/ros2_ws/install/unitree_go2_description/share/unitree_go2_description/urdf/unitree_go2_robot.xacro',
        '/opt/ros/humble/share/unitree_go2_description/urdf/unitree_go2_robot.xacro',
    ]
    
    for path in search_paths:
        if os.path.exists(path):
            return path
    
    return search_paths[0]


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/robot_description', qos)
        
        self.declare_parameter('xacro_path', '')
        self.declare_parameter('ros_control_file', '')
        
        param_xacro_path = self.get_parameter('xacro_path').value
        self.ros_control_file = self.get_parameter('ros_control_file').value
        
        self.xacro_path = get_xacro_path(param_xacro_path)
        
        self.get_logger().info(f'Using xacro path: {self.xacro_path}')
        self.get_logger().info(f'Using ros_control_file: {self.ros_control_file}')
        self.timer = self.create_timer(1.0, self.publish_description)
        
    def publish_description(self):
        if os.path.exists(self.xacro_path):
            cmd = ['xacro', self.xacro_path]
            if self.ros_control_file:
                cmd.extend(['robot_controllers:=' + self.ros_control_file])
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                msg = String(data=result.stdout)
                self.publisher.publish(msg)
                self.get_logger().info('Published robot description')
            else:
                self.get_logger().error(f'xacro failed: {result.stderr}')
        else:
            self.get_logger().error(f'xacro file not found: {self.xacro_path}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
