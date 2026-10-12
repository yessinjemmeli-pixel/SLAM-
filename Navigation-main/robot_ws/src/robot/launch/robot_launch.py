from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_node_with_terminal(package_name: str, node_name: str):
    return Node(package=package_name,
                executable=node_name,
                name=node_name,
                output='screen',
                
                emulate_tty=True)


def generate_launch_description():

    # -- Port série du Lidar
    # Exemple : ros2 launch robot robot_launch.py serial_port :=/dev/ttyUSB1
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Port série du RPLidar C1',
    )

    # -- Port série du Arduino
    arduino_serial_port_arg = DeclareLaunchArgument(
        'arduino_serial_port',
        default_value='/dev/ttyACM0',
        description='Port série du Arduino (Romeo BLE)',
    )

    # -- Driver officiel Slamtec
    # Baudrate C1 : 460800 | Mode : DenseBoost
    rplidar_driver = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate' : 460800,
            'scan_mode': 'DenseBoost',
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    serial_node = Node(package="robot",
                       executable="serial",
                       name="serial",
                       output='screen',
                       parameters=[{
                           "serial_port": LaunchConfiguration('arduino_serial_port'),
                       }],
                       emulate_tty=True)

    # Liste des noms des scripts python à lancer dans le package robot
    robot_nodes_to_launch = ['control', 'lidar', 'encoders', 'automatic', 'localization']
    #simulation_nodes_to_launch = ["driver"]

    ld = LaunchDescription()

    ld.add_action(lidar_serial_port_arg)
    ld.add_action(rplidar_driver)

    ld.add_action(arduino_serial_port_arg)
    ld.add_action(serial_node)

    # Ouverture des différents terminaux de type xterm
    for node_name in robot_nodes_to_launch:
        ld.add_action(launch_node_with_terminal("robot", node_name))

    #for node_name in simulation_nodes_to_launch:
    #    ld.add_action(launch_node_with_terminal("simulation", node_name))

    return ld
