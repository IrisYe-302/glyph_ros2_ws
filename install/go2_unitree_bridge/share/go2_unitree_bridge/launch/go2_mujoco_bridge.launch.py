import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_pkg = get_package_share_directory("unitree_go2_description")
    xacro_path = os.path.join(description_pkg, "urdf", "unitree_go2_robot.xacro")

    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("domain_id", default_value="1"),
            DeclareLaunchArgument("interface", default_value="lo"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_mujoco_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(["xacro ", xacro_path]),
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="go2_unitree_bridge",
                executable="go2_mujoco_bridge_node",
                name="go2_mujoco_bridge",
                output="screen",
                parameters=[
                    {
                        "domain_id": LaunchConfiguration("domain_id"),
                        "interface": LaunchConfiguration("interface"),
                    }
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                condition=IfCondition(foxglove),
                parameters=[
                    {
                        "port": foxglove_port,
                        "topic_whitelist": [
                            "^/imu/data$",
                            "^/joint_states$",
                            "^/odom$",
                            "^/parameter_events$",
                            "^/robot_description$",
                            "^/rosout$",
                            "^/tf$",
                            "^/tf_static$",
                        ],
                        "capabilities": ["assets"],
                        "ignore_unresponsive_param_nodes": True,
                    }
                ],
            ),
        ]
    )
