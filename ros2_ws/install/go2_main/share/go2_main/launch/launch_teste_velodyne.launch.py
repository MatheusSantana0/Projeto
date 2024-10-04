from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    

    #Caminho para o arquivo de configuração contendo os parâmetros para o slam
    slam_toolbox_param_file = os.path.join(
        get_package_share_directory('go2_main'),
        'config',
        'mapper_params_online_async.yaml'
    )


    #Caminho para o launch do slam_toolbox, com a passagem de alguns parâmetros
    pkg_slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': slam_toolbox_param_file,
            'use_sim_time': 'false',
        }.items()
    )

        
    pkg_velodyner_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_driver'),
                'launch', 'velodyne_driver_node-VLP16-launch.py'
            )
        )
    )

    pkg_velodyner_laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_laserscan'),
                'launch', 'velodyne_laserscan_node-launch.py'
            )
        )
    )

    pkg_velodyner_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_pointcloud'),
                'launch', 'velodyne_transform_node-VLP16-launch.py'
            )
        )
    )



    
    return LaunchDescription([
        pkg_slam_toolbox_launch,
        pkg_velodyner_driver_launch,
        pkg_velodyner_pointcloud_launch,
        pkg_velodyner_laser_scan_launch,
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2'
        ),
    ])
