import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # Impede que o garbage collector elimine a assinatura

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f'Recebido: linear.x={linear_x}, angular.z={angular_z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
