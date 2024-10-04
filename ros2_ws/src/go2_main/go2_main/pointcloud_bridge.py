#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudBridge(Node):
    def __init__(self):
        super().__init__('pointcloud_bridge')

        # Assina o tópico /utlidar/cloud_deskewed
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.listener_callback,
            10
        )

        # Publica no tópico /velodyne_points
        self.publisher_ = self.create_publisher(PointCloud2, '/velodyne_points', 10)

    def listener_callback(self, msg):
        # Processa ou altera a mensagem, se necessário
        self.get_logger().info('Receiving PointCloud2 data from /utlidar/cloud_deskewed')

        # Publica a mensagem no tópico /velodyne_points
        self.publisher_.publish(msg)
        self.get_logger().info('Published PointCloud2 data to /velodyne_points')


def main(args=None):
    rclpy.init(args=args)

    # Cria e roda o nó
    pointcloud_bridge = PointCloudBridge()

    try:
        rclpy.spin(pointcloud_bridge)
    except KeyboardInterrupt:
        pass

    # Destrói o nó e encerra a execução
    pointcloud_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
