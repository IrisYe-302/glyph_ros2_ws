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
    use_nav2 = LaunchConfiguration("nav2")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")

    sim_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_rlsar_sim.launch.py",
    )
    bringup_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "rlsar_nav2_localization.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("nav2", default_value="true"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/target_location"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/rlsar_scene.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={"foxglove": foxglove}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                condition=IfCondition(use_nav2),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": "False",
                    "params_file": nav2_params,
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
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            ),
        ]
    )
