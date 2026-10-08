"""
AGROVER MR 25.26 — Launch SLAM
Lance : bringup + slam_toolbox (mode cartographie)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg       = get_package_share_directory('agrover_ros')
    slam_pkg  = get_package_share_directory('slam_toolbox')

    # Lancer le bringup de base
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'agrover_bringup.launch.py')
        )
    )

    # SLAM Toolbox — mode cartographie asynchrone
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': False}
        ],
        output='screen'
    )

    return LaunchDescription([
        bringup,
        slam_node,
    ])
