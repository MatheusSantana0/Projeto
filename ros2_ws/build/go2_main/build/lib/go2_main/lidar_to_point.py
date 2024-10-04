import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')

        # Subscriber para o tópico de point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/input_point_cloud',
            self.listener_callback,
            10
        )
        # Publisher para o tópico de point cloud processada
        self.publisher = self.create_publisher(PointCloud2, '/output_point_cloud', 10)

    def listener_callback(self, msg):
        # Convertendo a PointCloud2 para numpy
        pc_data = pc2.point_cloud2_to_xyz_array(msg)
        
        # Convertendo para Open3D PointCloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(pc_data)
        
        # Aumentando a densidade da point cloud
        o3d_pc = self.increase_point_cloud_density(o3d_pc)
        
        # Convertendo de volta para numpy
        increased_points = np.asarray(o3d_pc.points)

        # Convertendo numpy para PointCloud2
        header = msg.header
        new_pc = pc2.create_cloud_xyz32(header, increased_points)

        # Publicando a nova point cloud
        self.publisher.publish(new_pc)

    def increase_point_cloud_density(self, point_cloud):
        # Aumentando a quantidade de pontos (exemplo simples: duplicando pontos)
        # Em um caso real, você usaria técnicas mais sofisticadas
        points = np.asarray(point_cloud.points)
        new_points = np.vstack([points, points * 1.1])  # Exemplo de duplicação simples
        new_point_cloud = o3d.geometry.PointCloud()
        new_point_cloud.points = o3d.utility.Vector3dVector(new_points)
        return new_point_cloud

def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = PointCloudProcessor()
    rclpy.spin(point_cloud_processor)
    point_cloud_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
