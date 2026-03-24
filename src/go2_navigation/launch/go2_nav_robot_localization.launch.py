import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    map_yaml = LaunchConfiguration("map")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")

    robot_launch = os.path.join(
        get_package_share_directory("go2_navigation"),
        "launch",
        "go2_nav_robot.launch.py",
    )
    localization_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "localization_launch.py",
    )
    navigation_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "robot_nav2_localization.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/target_location"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/robot_map.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={"foxglove": foxglove, "slam": "false", "nav2": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": "False",
                    "autostart": "True",
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "use_sim_time": "False",
                    "autostart": "True",
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="go2_navigation",
                executable="initial_pose_publisher",
                name="go2_initial_pose_publisher",
                parameters=[
                    {
                        "frame_id": "map",
                        "x": 0.0,
                        "y": 0.0,
                        "yaw": 0.0,
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": target_topic,
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="goal_tolerance_marker",
                name="go2_goal_tolerance_marker",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": target_topic,
                        "marker_topic": "/target_location_tolerance",
                        "radius": 0.5,
                    }
                ],
                output="screen",
            ),
        ]
    )
