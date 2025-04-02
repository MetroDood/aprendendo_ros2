import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, LaserScan
from math import sqrt, pi, exp, cos, sin

def gaussian(x, mean, sigma):
    return (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mean) ** 2) / (2 * sigma ** 2))

class extendedKalman(Node):

    def __init__(self):
        super().__init__('kalman')
        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.publisher = self.create_publisher(Float64, '/position_x', qos_profile)

        # # Initialize pose and motion parameters
        # self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        # self.wheel_radius = 0.033
        # self.wheel_dist = 0.178
        # self.measurement[None, None]
        # self.last_measurement = [None, None]
        # self.distance = [0, 0]
        # self.map[1,3]

        # self.sigma_laser = 0.01
        # self.sigma_movement = 0.002
        # self.sigma_odometria = 0.175

        # self.at_door = False
        # self.porta = 0
        # Constantes
        self.raio = 0.033
        self.distancia_rodas = 0.178
   
        # Variáveis
        self.posicao = [0.0, 0.0, 0.0] # x, y, theta
        self.medidas = [None, None] # esq, dir
        self.ultimas_medidas = [None, None] # esq, dir
        self.distancias = [0, 0]
        self.mapa = [1.0, 3.0] # posição central das duas “portas” existentes
   
        # Definindo uma estimativa dos possíveis erros
        self.sigma_odometria = 0.2 # rad
        self.sigma_lidar = 0.175 # meters
        self.sigma_movimento = 0.002 # m

        # info da porta
        self.na_porta = False
        self.porta = 0

        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0


    def run(self):
        rclpy.spin(self)

    def scan_callback(self, msg):
        range = np.array(msg.ranges[72:108])
        self.update()

    def joint_callback(self, msg):
        self.left_wheel_velocity = msg.velocity[0]
        self.right_wheel_velocity = msg.velocity[1]
        self.update()

    def update(self):

    def __del__(self):
        self.get_logger().info('Terminating Node.')

def main(args=None):
    rclpy.init(args=args)
    node = extendedKalman()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()