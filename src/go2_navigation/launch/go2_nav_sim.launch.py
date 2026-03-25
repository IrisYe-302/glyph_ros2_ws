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
    rviz = LaunchConfiguration("rviz")
    use_slam = LaunchConfiguration("slam")
    use_nav2 = LaunchConfiguration("nav2")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")

    sim_launch = os.path.join(
        get_package_share_directory("unitree_go2_sim"),
        "launch",
        "unitree_go2_launch.py",
    )
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    slam_params = os.path.join(
        get_package_share_directory("go2_robot_sdk"),
        "config",
        "mapper_params_online_async.yaml",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_robot_sdk"),
        "config",
        "nav2_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("nav2", default_value="true"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("cloud_topic", default_value="/unitree_lidar/points"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "rviz": rviz,
                    "foxglove": foxglove,
                    "static_map_tf": "false",
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="go2_sim_pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_frame": "base_link",
                        "transform_tolerance": 0.2,
                        "min_height": -0.5,
                        "max_height": 0.5,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": 0.15,
                        "range_max": 20.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(use_slam),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(use_nav2),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": target_topic,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber_target_location",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": "/target_location",
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
