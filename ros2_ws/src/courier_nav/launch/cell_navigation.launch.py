"""
Launch file for cell-to-cell courier robot navigation.

This launch file starts:
- Gazebo simulation with ROS bridge
- World spawner (5x5 grid with obstacles)
- AprilTag localizer for position corrections
- Behavior Tree mission controller

The robot navigates using:
- BFS pathfinding algorithm
- LIDAR obstacle detection
- AprilTag localization
- Cell-by-cell centering and rotation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    courier_description_dir = get_package_share_directory('courier_description')
    
    # Include simulation launch (Gazebo + bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(courier_description_dir, 'launch', 'sim.launch.py')
        )
    )
    
    # Spawn robot in Gazebo (delayed for Gazebo to start)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/empty/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'sdf_filename: "/home/ubuntu/ros2_ws/robot.sdf", name: "courier_robot", pose: {position: {x: 0.5, y: 0.5, z: 0.1}}'
                ],
                output='screen'
            )
        ]
    )
    
    # World spawner - spawn grid with obstacles (delayed after robot spawns)
    world_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='courier_nav',
                executable='spawner',
                name='world_spawner',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # AprilTag localizer (delayed after world is spawned)
    apriltag_localizer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='courier_nav',
                executable='apriltag_localizer',
                name='apriltag_localizer',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Behavior Tree mission controller (delayed to allow sensors to init)
    mission_controller = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='courier_nav',
                executable='mission_controller',
                name='mission_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Static transforms for TF tree
    static_tf_map = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            )
        ]
    )
    
    static_tf_base = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_footprint_tf',
                arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_footprint']
            )
        ]
    )
    
    static_tf_lidar = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_tf',
                arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link']
            )
        ]
    )
    
    static_tf_camera = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_tf',
                arguments=['0.15', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
            )
        ]
    )
    
    return LaunchDescription([
        sim_launch,
        spawn_robot,
        static_tf_map,
        static_tf_base,
        static_tf_lidar,
        static_tf_camera,
        world_spawner,
        apriltag_localizer,
        mission_controller,
    ])
