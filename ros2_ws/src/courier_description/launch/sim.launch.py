import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_name = 'courier_description'

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Path al workspace - usa variabile d'ambiente se disponibile, altrimenti cerca
    if 'COLCON_PREFIX_PATH' in os.environ:
        ws_path = os.path.dirname(os.environ['COLCON_PREFIX_PATH'].split(':')[0])
    else:
        # Fallback: assumi che siamo in install/
        ws_path = os.path.expanduser('~/Scaricati/courier_robot-main/ros2_ws')
    
    robot_sdf_path = os.path.join(ws_path, 'robot.sdf')
    
    # Verifica che il file esista
    if not os.path.exists(robot_sdf_path):
        raise FileNotFoundError(f"Robot SDF non trovato: {robot_sdf_path}")
    
    # 1. Leggi il contenuto del file SDF
    with open(robot_sdf_path, 'r') as f:
        robot_desc = f.read()
 
    # 2. Avvia Gazebo Harmonic (Mondo vuoto)

    # L'argomento -r fa partire la simulazione subito (run)

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

        ),

        launch_arguments={'gz_args': '-r empty.sdf'}.items(),

    )
 
    # 3. Robot spawning è gestito esternamente nello script start_simulation.sh
 
    # 4. Robot State Publisher - NON NECESSARIO per SDF puro (Gazebo gestisce tutto)
 
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
        bridge
    ])
 
