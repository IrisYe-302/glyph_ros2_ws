import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    map_yaml = LaunchConfiguration("map")
    target_topic = LaunchConfiguration("target_topic")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")

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
    nav2_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "robot_nav2_localization_mppi.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_delay", default_value="5.0"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/robot_map.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    "foxglove": foxglove,
                    "slam": "false",
                    "nav2": "false",
                    "location_subscriber": "false",
                    "use_ekf": "true",
                    "use_lidar_odom": "false",
                }.items(),
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
            TimerAction(
                period=6.5,
                actions=[
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_smoother",
                        executable="smoother_server",
                        name="smoother_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        arguments=["--ros-args", "--log-level", "info"],
                        parameters=[
                            {"use_sim_time": False},
                            {"autostart": True},
                            {
                                "node_names": [
                                    "controller_server",
                                    "smoother_server",
                                    "planner_server",
                                    "behavior_server",
                                    "bt_navigator",
                                    "waypoint_follower",
                                ]
                            },
                        ],
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber",
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
                        executable="location_subscriber",
                        name="go2_location_subscriber_target_location",
                        parameters=[
                            {
                                "target_topic": "/target_location",
                                "use_sim_time": False,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker",
                        parameters=[
                            {
                                "target_topic": target_topic,
                                "marker_topic": "/target_location_tolerance",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_target_location",
                        parameters=[
                            {
                                "target_topic": "/target_location",
                                "marker_topic": "/target_location_tolerance",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="go2_navigation",
                executable="initial_pose_publisher",
                name="go2_initial_pose_publisher",
                parameters=[
                    {
                        "topic": "/initialpose",
                        "frame_id": "map",
                        "x": initial_pose_x,
                        "y": initial_pose_y,
                        "yaw": initial_pose_yaw,
                        "use_sim_time": False,
                        "delay_sec": initial_pose_delay,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="initial_pose_restamper",
                name="go2_initial_pose_restamper",
                parameters=[
                    {
                        "input_topic": "/initialpose_raw",
                        "output_topic": "/initialpose",
                    }
                ],
                output="screen",
            ),
        ]
    )
