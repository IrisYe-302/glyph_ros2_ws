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

    sport_state_topic = LaunchConfiguration("sport_state_topic")
    sport_state_fallback_topic = LaunchConfiguration("sport_state_fallback_topic")
    low_state_topic = LaunchConfiguration("low_state_topic")
    low_state_fallback_topic = LaunchConfiguration("low_state_fallback_topic")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_simple_visuals = LaunchConfiguration("foxglove_simple_visuals")
    foxglove_include_velodyne = LaunchConfiguration("foxglove_include_velodyne")
    foxglove_include_realsense = LaunchConfiguration("foxglove_include_realsense")

    return LaunchDescription(
        [
            DeclareLaunchArgument("sport_state_topic", default_value="lf/sportmodestate"),
            DeclareLaunchArgument("sport_state_fallback_topic", default_value="/sportmodestate"),
            DeclareLaunchArgument("low_state_topic", default_value="lf/lowstate"),
            DeclareLaunchArgument("low_state_fallback_topic", default_value="/lowstate"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_simple_visuals", default_value="false"),
            DeclareLaunchArgument("foxglove_include_velodyne", default_value="false"),
            DeclareLaunchArgument("foxglove_include_realsense", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                xacro_path,
                                " simple_visuals:=",
                                foxglove_simple_visuals,
                                " include_velodyne:=",
                                foxglove_include_velodyne,
                                " include_realsense:=",
                                foxglove_include_realsense,
                            ]
                        ),
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="unitree_go2_sim",
                executable="robot_description_publisher.py",
                name="go2_robot_description_publisher",
                output="screen",
                parameters=[
                    {
                        "xacro_path": xacro_path,
                        "simple_visuals": foxglove_simple_visuals,
                        "include_velodyne": foxglove_include_velodyne,
                        "include_realsense": foxglove_include_realsense,
                    }
                ],
            ),
            Node(
                package="go2_unitree_bridge",
                executable="go2_unitree_bridge_node",
                name="go2_unitree_bridge",
                output="screen",
                parameters=[
                    {
                        "sport_state_topic": sport_state_topic,
                        "sport_state_fallback_topic": sport_state_fallback_topic,
                        "low_state_topic": low_state_topic,
                        "low_state_fallback_topic": low_state_fallback_topic,
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
                            "^/cmd_vel$",
                            "^/imu/data$",
                            "^/joint_states$",
                            "^/local_costmap/published_footprint$",
                            "^\/plan$",
                            "^\/local_plan$",
                            "^\/global_plan$",
                            "^\/transformed_global_plan$",
                            "^\/follow_path\/transformed_global_plan$",
                            "^\/local_costmap\/costmap$",
                            "^\/local_costmap\/costmap_raw$",
                            "^\/global_costmap\/costmap$",
                            "^\/global_costmap\/costmap_raw$",
                            "^\/trajectory$",
                            "^\/trajectories$",
                            "^\/mppi_trajectory$",
                            "^\/mppi_trajectories$",
                            "^\/mppi_controller\/.*$",
                            "^/map$",
                            "^/map_metadata$",
                            "^/odom$",
                            "^/scan$",
                            "^/target_location_tolerance$",
                            "^/parameter_events$",
                            "^/robot_description$",
                            "^/rosout$",
                            "^/tf$",
                            "^/tf_static$",
                        ],
                        "client_topic_whitelist": [
                            "^/clicked_point$",
                            "^/initialpose$",
                            "^/move_base_simple/goal$",
                        ],
                        "capabilities": [
                            "clientPublish",
                            "assets",
                            "connectionGraph",
                        ],
                        "ignore_unresponsive_param_nodes": True,
                    }
                ],
            ),
        ]
    )
