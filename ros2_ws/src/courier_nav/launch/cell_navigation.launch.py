"""
Launch file per navigazione cell-to-cell
Include:
- Gazebo simulation
- World spawner (griglia migliorata)
- Cell-to-cell controller (no Nav2)
- AprilTag localizer
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
                    '--req', 'sdf_filename: "/root/ros2_ws/robot.sdf", name: "courier_robot", pose: {position: {x: 0.5, y: 0.5, z: 0.1}}'
                ],
                output='screen'
            )
        ]
    )
    
    # World spawner - spawn griglia migliorata (delayed after robot spawns)
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
        period=20.0,
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
    
    # Cell-to-Cell navigation controller (delayed to allow sensors to init)
    navigation_controller = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='courier_nav',
                executable='courier_controller',
                name='courier_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Static transform: map -> odom (identity, since we use odom as reference)
    static_tf = TimerAction(
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
    
    return LaunchDescription([
        sim_launch,
        spawn_robot,
        static_tf,
        world_spawner,
        apriltag_localizer,
        navigation_controller,
    ])
