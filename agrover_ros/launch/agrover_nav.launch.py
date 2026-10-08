"""
AGROVER MR 25.26 — Launch Navigation Autonome
Lance : bringup + AMCL + Nav2
Prérequis : avoir une carte enregistrée avec le SLAM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg      = get_package_share_directory('agrover_ros')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    map_file = DeclareLaunchArgument(
        'map',
        default_value='/home/rover/maps/agrover_map.yaml',
        description='Chemin vers la carte YAML'
    )

    # Bringup de base
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'agrover_bringup.launch.py')
        )
    )

    # Nav2 complet
    # Nav2 publie par défaut sur /cmd_vel → on remplace vers /cmd_vel/nav
    # pour que twist_mux arbitre correctement avec la téléopération manuelle
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map'              : LaunchConfiguration('map'),
            'use_sim_time'     : 'false',
            'params_file'      : os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'autostart'        : 'true',
            'cmd_vel_topic'    : '/cmd_vel/nav',
        }.items()
    )

    return LaunchDescription([
        map_file,
        bringup,
        nav2,
    ])
