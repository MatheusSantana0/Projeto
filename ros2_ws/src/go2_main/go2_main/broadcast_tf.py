#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped


class BroadcastTF(Node):
    """Classe responsável pela publicação dos valores atuais de odometria como forma de atualizar a rede do TF,
    ou seja, as relações entre os sietams de coordenadas existentes no robô. Dessa forma, permite visualizar o 
    deslocamento do robô ao longo do rviz"""

    def __init__(self):
        super().__init__('broadcast_tf')

        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_link"

        self.subscriber = self.create_subscription(
            PoseStamped,
            '/utlidar/robot_pose',
            self.odom_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        # O objeto 'TransformBroadcaster' é um nó do ROS que publica mensagens TF
        self.br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Broadcaster TF inicializado!")

    def odom_callback(self, msg):
        """
        Função de callback, em que recebe os dados provenientes da odometria.
        
        Args:
            msg: mensagem proveniente do tópico /utlidar/robot_pose no formato PoseStamped
        """

        self.cam_bot_odom = msg
        self.broadcast_new_tf()

    def broadcast_new_tf(self):
        """
        Esta função publica uma nova mensagem TF para toda a rede do TF.
        """

        # Captura dos valores atuais de odometria
        position = self.cam_bot_odom.pose.position
        orientation = self.cam_bot_odom.pose.orientation

        # Definição da hora atual no cabeçalho da mensagem TF
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        # Aquisição dos valores de odometria atual do robô para translação
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z

        # Aquisição dos valores de odometria atual do robô para orientação
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        # Envio da mensagem TF para todos
        self.br.sendTransform(self.transform_stamped)


def main(args=None):
    rclpy.init()
    objeto_tf = BroadcastTF()
    rclpy.spin(objeto_tf)

if __name__ == '__main__':
    main()