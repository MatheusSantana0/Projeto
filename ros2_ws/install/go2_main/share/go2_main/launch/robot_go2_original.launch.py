from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    #Caminho do pacote da descrição do robô
    pkg_robot_description = get_package_share_directory('go2_description')

    #Caminho para o arquivo de configuração contendo os parâmetros para o slam
    slam_toolbox_param_file = os.path.join(
        get_package_share_directory('go2_main'),
        'config',
        'mapper_params_online_async.yaml'
    )

    #Caminho para o launch da descrição do robô, o qual ativa o robot_state_publisher e o rviz
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'robot.launch.py')
        )
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

    
    return LaunchDescription([
        robot_description_launch,
        Node(
            package="go2_main",
            executable="broadcast_tf",
            output="screen"
        ),
        Node(
            package="go2_main",
            executable="valores_juntas",
            output="screen"
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[
                ('cloud_in', 'utlidar/cloud_deskewed'),
                ('scan', 'scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'max_height': 0.5,
                'angle_increment': 0.00087,
            }],
            output="screen",
        ),
        pkg_slam_toolbox_launch
    ])
