import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    courier_nav_dir = get_package_share_directory('courier_nav')
    courier_description_dir = get_package_share_directory('courier_description')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    # Include simulation launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(courier_description_dir, 'launch', 'sim.launch.py')
        )
    )
    
    # Include Nav2 bringup (delayed to allow Gazebo to start)
    nav2_launch = TimerAction(
        period=8.0,  # Wait for Gazebo and robot to spawn
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(courier_nav_dir, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # Robot State Publisher (needed for TF tree)
    # This reads URDF and publishes robot_description and TF
    robot_state_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': open(
                        os.path.join(courier_description_dir, 'urdf', 'courier.urdf.xacro'),
                        'r'
                    ).read() if os.path.exists(
                        os.path.join(courier_description_dir, 'urdf', 'courier.urdf.xacro')
                    ) else ''
                }]
            )
        ]
    )
    
    # Initial pose publisher for AMCL (set robot's starting position)
    initial_pose_pub = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        sim_launch,
        robot_state_publisher,
        nav2_launch,
        initial_pose_pub,
    ])
