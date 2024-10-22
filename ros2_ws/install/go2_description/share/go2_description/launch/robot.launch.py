import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros.descriptions

# Launch para os componentes da descrição do robô
def generate_launch_description():

    # Nome do arquivo que consta o urdf com a descrição do robô
    urdf_file = 'go2_description.urdf'
 
    # Nome do pacote em que está localizada a descrição do robô
    package_description = "go2_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher
    # É um pacote nativo do ROS responsável por enviar dados para o tópico do tf. O seu nó que leva o mesmo nome, recebe como parâmetro o urdf,
    # informando todas os elos e juntas e as suas conexões. Além disso, esse nó está inscrito no /joint_states, dessa forma os valores das juntas,
    # seja em um ambiente de simulação ou em um ambiente real, devem ser direcionados para esse tópico. 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False, 
            'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ', robot_desc_path]),value_type=str)}],
        output="screen"
    )

    # Configuração RVIZ
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'v3_go2.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config_dir])

    # Criação e retorno do objeto de descrição do launch
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node,
        ]
    )