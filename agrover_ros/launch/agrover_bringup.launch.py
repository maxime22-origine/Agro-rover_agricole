"""
AGROVER MR 25.26 — Launch file principal
Lance : agrover_node + rplidar + twist_mux
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('agrover_ros')

    # ── Arguments ─────────────────────────────────────────────
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Lancer RViz2')

    serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Port série OpenCR')

    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Port série RPLIDAR A1')

    # ── Nœud principal AGROVER ─────────────────────────────────
    agrover_node = Node(
        package='agrover_ros',
        executable='agrover_node',
        name='agrover_node',
        parameters=[
            os.path.join(pkg, 'config', 'params.yaml'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ],
        output='screen'
    )

    # ── RPLIDAR A1 ─────────────────────────────────────────────
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port'     : LaunchConfiguration('lidar_port'),
            'serial_baudrate' : 115200,
            'frame_id'        : 'laser_link',
            'inverted'        : False,
            'angle_compensate': True,
            'scan_mode'       : 'Standard'
        }],
        output='screen'
    )

    # ── twist_mux ──────────────────────────────────────────────
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen'
    )

    # ── Robot State Publisher (URDF/TF) ───────────────────────
    # Utiliser Command + xacro pour traiter le fichier .xacro correctement
    urdf_path = os.path.join(pkg, 'urdf', 'agrover.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    # ── RViz2 (optionnel) ──────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        use_rviz,
        serial_port,
        lidar_port,
        robot_state_publisher,
        agrover_node,
        rplidar_node,
        twist_mux_node,
        rviz_node,
    ])
