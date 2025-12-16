from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    
    # Avvia il controllore PID dopo 5 secondi (per dare tempo a Gazebo di inizializzare)
    pid_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='courier_control',
                executable='pid_controller',
                name='pid_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Avvia il mission controller con behavior tree
    mission_controller = TimerAction(
        period=5.0,
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
    
    return LaunchDescription([
        pid_controller,
        mission_controller
    ])
