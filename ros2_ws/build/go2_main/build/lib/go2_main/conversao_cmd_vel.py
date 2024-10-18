import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
from rclpy.qos import QoSProfile

class ConversaoCmdVel(Node):
    """Classe responsável pela tradução de comandos de velocidade. Leitura dos comandos de velocidade fornecidos 
    pelo pacote nav2 através do tópico cmd_vel, e consequente publicação desses dados no formato lido pelo tópico
    /api/sport/request próprio do Go2"""

    def __init__(self):
        super().__init__("conversao_cmd_vel")

        self.subscriber = self.create_subscription(
            Twist,                          # tipo da mensagem
            "/cmd_vel",                     # tópico
            self.listener_callback,         # callback
            QoSProfile(depth=10)            # qos profile
        )

        self.publisher = self.create_publisher(
            Request,                        # tipo da mensagem
            "/api/sport/request",           # tópico
            10)                             # qos profile

    def listener_callback(self, msg):
        """
        Função destinada a leitura e envio dos comandos de velocidade. 
        
        Args:
            msg: mensagem proveniente do tópico cmd_vel no formato Twist
        """

        msg_pub = Request()
        msg_pub.header.identity.id = 19             # Aparentemente número aleatório
        msg_pub.header.identity.api_id = 1008       # const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;

        msg_pub.parameter = "{\"x\": " + str(msg.linear.x) + ", \"y\": " + str(msg.linear.y) + ", \"z\": " + str(msg.angular.z) + "}"
        
        self.publisher.publish(msg_pub)     

def main(args=None):
    rclpy.init(args=args)
    conversao_cmd_vel = ConversaoCmdVel()
    rclpy.spin(conversao_cmd_vel)
    conversao_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
