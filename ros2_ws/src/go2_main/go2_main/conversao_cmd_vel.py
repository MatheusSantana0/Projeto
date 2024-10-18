import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from unitree_api.msg import Request
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy



class ConversaoCmdVel(Node):
    """Classe responsável pela tradução de comandos de velocidade. Leitura dos comandos de velocidade fornecidos pelo pacote nav2 através do tópico cmd_vel, e 
    consequente publicação desses dados no formato lido pelo tópico /api/sport/request próprio do Go2 """

    def __init__(self):
        super().__init__("conversao_cmd_vel")

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            QoSProfile(depth=10)
        )

        self.publisher = self.create_publisher(Request, "/api/sport/request",10)
        self.timer = self.create_timer(0.05, self.timer_callback) #Publicação de dados a uma frequência de 20 hz

    def listener_callback(self, msg):
        """
        Função destinada a leitura dos pontos recebidos (x, y e z de cada uma das patas) e associação a variável correspondente
        
        Args:
            SportModeState: mensagem com pontos
        """
        
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
    
    def timer_callback(self):
        """
        Função destinada a montar a mensagem do tipo JointState e publicá-la no tóico correspondente.
        """

        msg = Request()
        msg.header.identity.id = 18
        msg.header.identity.api_id = 1008

        if self.linear_x > 0.0 or self.linear_y > 0.0 or self.angular_z != 0.0:
            msg.parameter = "{\"x\": " + str(self.linear_x) + ", \"y\": " + str(self.linear_y) + ", \"z\": " + str(self.angular_z) + "}"
        
        self.publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    conversao_cmd_vel = ConversaoCmdVel()
    rclpy.spin(conversao_cmd_vel)
    conversao_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
