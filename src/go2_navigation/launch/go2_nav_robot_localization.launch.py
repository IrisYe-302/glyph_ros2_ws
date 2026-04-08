import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    map_yaml = LaunchConfiguration("map")
    target_topic = LaunchConfiguration("target_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    global_localization = LaunchConfiguration("global_localization")
    global_localization_delay = LaunchConfiguration("global_localization_delay")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    return_home_trigger_topic = LaunchConfiguration("return_home_trigger_topic")
    home_target_topic = LaunchConfiguration("home_target_topic")
    set_home_topic = LaunchConfiguration("set_home_topic")
    gpio_return_home_pin = LaunchConfiguration("gpio_return_home_pin")

    bridge_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_unitree_bridge.launch.py",
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
            DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_delay", default_value="5.0"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument("global_localization", default_value="false"),
            DeclareLaunchArgument("global_localization_delay", default_value="8.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="12.0"),
            DeclareLaunchArgument("return_home_trigger_topic", default_value="/return_home_trigger"),
            DeclareLaunchArgument("home_target_topic", default_value="/return_home_target_location"),
            DeclareLaunchArgument("set_home_topic", default_value="/set_home_here"),
            DeclareLaunchArgument("gpio_return_home_pin", default_value="7"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/robot_map.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "foxglove": foxglove,
                    "use_ekf": "false",
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="go2_nav_pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", "/scan_raw"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "transform_tolerance": 0.2,
                        "min_height": -0.10,
                        "max_height": 0.40,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": 0.20,
                        "range_max": 6.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="scan_restamper",
                name="go2_scan_restamper",
                parameters=[
                    {
                        "input_topic": "/scan_raw",
                        "output_topic": "/scan",
                    }
                ],
                output="screen",
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
                period=nav2_start_delay,
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
                        executable="sim_behavior_supervisor",
                        name="go2_behavior_supervisor",
                        parameters=[
                            {
                                "target_topic": target_topic,
                                "target_location_topic": "/target_location",
                                "return_home_trigger_topic": return_home_trigger_topic,
                                "home_target_topic": home_target_topic,
                                "set_home_topic": set_home_topic,
                                "home_x": initial_pose_x,
                                "home_y": initial_pose_y,
                                "home_yaw": initial_pose_yaw,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="gpio_return_home_publisher",
                        name="go2_gpio_return_home_publisher",
                        parameters=[
                            {
                                "topic": return_home_trigger_topic,
                                "pin_number": gpio_return_home_pin,
                                "pin_mode": "BOARD",
                                "pull": "DOWN",
                            }
                        ],
                        output="screen",
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
                        executable="location_subscriber",
                        name="go2_location_subscriber_return_home",
                        parameters=[
                            {
                                "target_topic": home_target_topic,
                                "use_sim_time": False,
                                "orient_toward_goal_center": False,
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
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_return_home",
                        parameters=[
                            {
                                "target_topic": home_target_topic,
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
                condition=IfCondition(publish_initial_pose),
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
                executable="global_localization_trigger",
                name="go2_global_localization_trigger",
                condition=IfCondition(global_localization),
                parameters=[
                    {
                        "service_name": "/reinitialize_global_localization",
                        "delay_sec": global_localization_delay,
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
            Node(
                package="go2_navigation",
                executable="session_map_publisher",
                name="go2_session_map_publisher",
                parameters=[
                    {
                        "base_map_topic": "/map",
                        "overlay_topic": "/global_costmap/costmap",
                        "output_topic": "/session_map",
                    }
                ],
                output="screen",
            ),
        ]
    )
