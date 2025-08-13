# Datei: simulation.lauch.py
# Beschreibung: Simulations Launch datei und Transport-Bridge
#               Parametrierung
# Autor: Chukwunonso Bob-Anyeji
# Date: 18.07.2025
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    pkg_depthcam_sim = get_package_share_directory('depthcam_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sim_sdf_path = os.path.join(
        pkg_depthcam_sim,
        'world',
        'simulation.sdf'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': sim_sdf_path,
        }.items(),
    )

    # RQt
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/depthcam'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_depthcam_sim, 'rviz', 'simulation.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/depthcam@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/depthcam/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),
        bridge,
        rviz,
        rqt
    ])
