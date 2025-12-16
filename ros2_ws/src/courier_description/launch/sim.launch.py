import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
 
def generate_launch_description():

    pkg_name = 'courier_description'

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
 
    # 1. Processa l'URDF (Xacro)

    # Nota: Usiamo il path assoluto per semplicità di sviluppo immediato

    xacro_file = os.path.join(os.environ['HOME'], 'ros2_ws/src', pkg_name, 'urdf', 'courier.urdf.xacro')

    doc = xacro.process_file(xacro_file)

    robot_desc = doc.toxml()
 
    # 2. Avvia Gazebo Harmonic (Mondo vuoto)

    # L'argomento -r fa partire la simulazione subito (run)

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

        ),

        launch_arguments={'gz_args': '-r empty.sdf'}.items(),

    )
 
    # 3. Spawna il Robot in Gazebo (centro della cella verde START)

    spawn = Node(

        package='ros_gz_sim',

        executable='create',

        arguments=[

            '-name', 'courier_robot',

            '-string', robot_desc,

            '-x', '0.5', '-y', '0.5', '-z', '0.1'  # Row 0, Col 0

        ],

        output='screen'

    )
 
    # 4. Robot State Publisher (Pubblica le trasformazioni TF del robot)

    robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='both',

        parameters=[{'robot_description': robot_desc}],

    )
 
    # 5. Bridge ROS 2 <-> Gazebo (Fondamentale!)

    # Collega i topic di Gazebo ai topic standard ROS 2

    bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[

            # Comando Motori: ROS -> Gazebo

            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # Odometria: Gazebo -> ROS

            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # Lidar: Gazebo -> ROS

            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

            # Trasformazioni TF

            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'

        ],

        output='screen'

    )
 
    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        spawn,

        bridge

    ])
 
