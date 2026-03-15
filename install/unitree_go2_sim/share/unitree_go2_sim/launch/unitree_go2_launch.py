import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = "base_link"

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_foxglove = DeclareLaunchArgument(
        "foxglove", default_value="true", description="Launch foxglove bridge"
    )
    declare_static_map_tf = DeclareLaunchArgument(
        "static_map_tf", default_value="true", description="Publish static map to odom transform"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.50")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )
    
    # Description nodes and parameters
    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path"),
                                                       " robot_controllers:=", LaunchConfiguration("ros_control_file")])}
    
    robot_description_publisher = Node(
        package='unitree_go2_sim',
        executable='robot_description_publisher.py',
        parameters=[{
            'xacro_path': LaunchConfiguration("unitree_go2_description_path"),
            'ros_control_file': LaunchConfiguration("ros_control_file")
        }],
        output='screen',
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    odom_tf_broadcaster_node = Node(
        package="unitree_go2_sim",
        executable="odom_tf_broadcaster.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    declare_use_champ_state_estimation = DeclareLaunchArgument(
        "use_champ_state_estimation",
        default_value="false",
        description="Enable CHAMP state estimation and EKF stack instead of raw Gazebo odom",
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/commands"},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_champ_state_estimation")),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_champ_state_estimation")),
        parameters=[
            {"base_link_frame": base_frame},
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_champ_state_estimation")),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "odom/raw"},
            {"odom0_config": [True, True, True, True, True, True, False, False, False, False, False, False, False, False, False]},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # Go2 static frame connection (map -> odom)
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        condition=IfCondition(LaunchConfiguration("static_map_tf")),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
    )
    
    # Go2 URDF connection (base_footprint -> base_link)  
    base_footprint_to_base_link_tf_node = Node(
        package='tf2_ros',
        name='base_footprint_to_base_link_tf_node',
        executable='static_transform_publisher',
        condition=IfCondition(LaunchConfiguration("use_champ_state_estimation")),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        # parameters=[{"use_sim_time": use_sim_time}]
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch', 'foxglove_bridge_launch.xml'
    )
    
    foxglove_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(foxglove_launch),
        condition=IfCondition(LaunchConfiguration("foxglove")),
        launch_arguments={
            'topic_whitelist': "['^/tf$', '^/tf_static$', '^/joint_states$', '^/imu/data$', '^/odom$', '^/scan$', '^/cmd_vel$', '^/map$', '^/map_metadata$', '^/robot_description$', '^/parameter_events$', '^/rosout$']",
            'client_topic_whitelist': "['^/clicked_point$', '^/initialpose$', '^/move_base_simple/goal$']",
            'capabilities': '[clientPublish,assets]',
            'ignore_unresponsive_param_nodes': 'true',
        }.items(),
    )
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                unitree_go2_description,
                'worlds',
                'default.sdf'
            ]), ' -r -s']  # -r = start unpaused, -s = server only (no GUI, headless)
        }.items(),
    )
    
    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )
    
    # Bridge ROS 2 topics to Gazebo Sim
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Gazebo to ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/velodyne_points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # D455 RGBD camera bridges
            '/d455/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )
    
    # Spawn controllers shortly after the robot is created so /cmd_vel starts working promptly.
    spawner_joint_states = ExecuteProcess(
        cmd=[
            "/opt/ros/humble/lib/controller_manager/spawner",
            "joint_states_controller",
            "--controller-manager-timeout",
            "120",
        ],
        output='screen',
    )

    controller_spawner_js = TimerAction(
        period=2.5,
        actions=[spawner_joint_states]
    )

    spawner_joint_group_effort = ExecuteProcess(
        cmd=[
            "/opt/ros/humble/lib/controller_manager/spawner",
            "joint_group_effort_controller",
            "--controller-manager-timeout",
            "120",
        ],
        output='screen',
    )

    controller_spawner_effort = TimerAction(
        period=3.2,
        actions=[spawner_joint_group_effort]
    )

    # The controller spawner can linger as a node even after the controller is
    # active, so starting the gait node strictly on process exit is unreliable.
    quadruped_controller_start = TimerAction(
        period=4.5,
        actions=[quadruped_controller_node],
    )
    
    # Shell script to manually check controller status 
    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_foxglove,
            declare_static_map_tf,
            declare_use_champ_state_estimation,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path, 
            
            # Gazebo and robot nodes first
            gz_sim,
            robot_description_publisher,
            robot_state_publisher_node,
            odom_tf_broadcaster_node,
            
            # Wait for robot_description to be published before spawning
            TimerAction(
                period=2.0,
                actions=[gazebo_spawn_robot],
            ),
            gazebo_bridge,
            
            # CHAMP controller nodes
            quadruped_controller_start,
            state_estimator_node,
            
            # EKF nodes for localization
            base_to_footprint_ekf,
            footprint_to_odom_ekf,
            
            # TF publishers for frame connections
            map_to_odom_tf_node,
            base_footprint_to_base_link_tf_node,
            
            # Controller spawners that handle the complete lifecycle
            controller_spawner_js,
            controller_spawner_effort,
            # Visualization (only if rviz flag is set)
            rviz2,
            foxglove_bridge,
        ]
    )
