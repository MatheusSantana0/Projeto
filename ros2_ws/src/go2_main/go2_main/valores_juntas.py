import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
from go2_main.calculos import get_robot_joints


class ValoresJuntas(Node):
    """Classe responsável pela leitura dos pontos das patas fornecidos pelo tópico/sportmodestate presente no robô,
    conversão dos pontos em valores de juntas e consequente transmissão para o tópico /joint_states (tópico no qual
    o robot_state_publisher lê diretamente os valores das juntas)"""

    def __init__(self):
        super().__init__("valores_juntas")

        self.subscriber = self.create_subscription(
            SportModeState,
            "/sportmodestate",
            self.listener_callback,
            QoSProfile(depth=10)
        )

        self.publisher = self.create_publisher(JointState, "/joint_states",10)
        self.timer = self.create_timer(0.05, self.timer_callback) #Publicação de dados a uma frequência de 20 hz

    def listener_callback(self, msg):
        """
        Função destinada a leitura dos pontos recebidos (x, y e z de cada uma das patas) e associação a variável correspondente
        
        Args:
            SportModeState: mensagem com pontos
        """

        self.FR_x = float(msg.foot_position_body[0])
        self.FR_y = float(msg.foot_position_body[1])
        self.FR_z = float(msg.foot_position_body[2])

        self.FL_x = float(msg.foot_position_body[3])
        self.FL_y = float(msg.foot_position_body[4])
        self.FL_z = float(msg.foot_position_body[5])

        self.RR_x = float(msg.foot_position_body[6])
        self.RR_y = float(msg.foot_position_body[7])
        self.RR_z = float(msg.foot_position_body[8])

        self.RL_x = float(msg.foot_position_body[9])
        self.RL_y = float(msg.foot_position_body[10])
        self.RL_z = float(msg.foot_position_body[11])
    
    def timer_callback(self):
        """
        Função destinada a montar a mensagem do tipo JointState e publicá-la no tópico correspondente.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_robot_joints(
            [self.FL_x, self.FL_y, self.FL_z],
            0
        )

        FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_robot_joints(
            [self.FR_x, self.FR_y, self.FR_z],
            1
        )

        RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_robot_joints(
            [self.RL_x, self.RL_y, self.RL_z],
            2
        )

        RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_robot_joints(
            [self.RR_x, self.RR_y, self.RR_z],
            3
        )

        msg.name = [
            'FL_hip_joint','FL_thigh_joint','FL_calf_joint',
            'FR_hip_joint','FR_thigh_joint','FR_calf_joint',
            'RL_hip_joint','RL_thigh_joint','RL_calf_joint',
            'RR_hip_joint','RR_thigh_joint','RR_calf_joint'
        ]
        
        msg.position = [
            FL_hip_joint, FL_thigh_joint, FL_calf_joint,
            FR_hip_joint, FR_thigh_joint, FR_calf_joint,
            RL_hip_joint, RL_thigh_joint, RL_calf_joint,
            RR_hip_joint, RR_thigh_joint, RR_calf_joint,
        ]
        
        self.publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    valores_das_juntas = ValoresJuntas()
    rclpy.spin(valores_das_juntas)
    valores_das_juntas.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
