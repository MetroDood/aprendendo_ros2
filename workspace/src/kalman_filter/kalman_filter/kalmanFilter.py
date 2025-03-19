import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, LaserScan
from math import sqrt, pi, exp, cos, sin

def gaussian(x, mean, sigma):
    return (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mean) ** 2) / (2 * sigma ** 2))

class kalmanFilter(Node):

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
        if(self.ultimas_medidas[0] == None or
           self.ultimas_medidas[1] == None):
            self.ultimas_medidas[0] = msg_joints.position[0]
            self.ultimas_medidas[1] = msg_joints.position[1]      
            return

        self.medidas[0] = self.left_wheel_velocity
        self.medidas[1] = self.right_wheel_velocity

        diff = self.medidas[0] - self.ultimas_medidas[0] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[0] = diff * self.raio + random.random()*0.002 # determina distância percorrida em metros e adiciona um pequeno erro
        self.ultimas_medidas[0] = self.medidas[0]

        diff = self.medidas[1] - self.ultimas_medidas[1] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[1] = diff * self.raio + random.random()*0.002 # determina distância percorrida em metros + pequeno erro
        self.ultimas_medidas[1] = self.medidas[1]

        # ## cálculo da dist linear e angular percorrida no timestep
        deltaS = (self.distancias[0] + self.distancias[1]) / 2.0
        deltaTheta = (self.distancias[1] - self.distancias[0]) / self.distancia_rodas
        self.posicao[2] = (self.posicao[2] + deltaTheta) % 6.28

        deltaSx = deltaS * numpy.cos(self.posicao[2])
        deltaSy = deltaS * numpy.sin(self.posicao[2])

        # atualização acumulativa da posição x e y
        self.posicao[0] = self.posicao[0] + deltaSx # atualiza x
        self.posicao[1] = self.posicao[1] + deltaSy # atualiza y

        # print("Postura:", self.posicao)
        self.pub_x(self.posicao[0])
        # if self.latest_joint is not None:
        #     left_encoder = self.latest_joint[0]
        #     right_encoder = self.latest_joint[1]

        #     distance = [0, 0]
        #     for i in range(2):
        #         diff = (left_encoder if i == 0 else right_encoder) - self.last_measurement[i]
        #         distance[i] = diff * self.wheel_radius + np.random.normal(0, self.sigma_movement)
        #         self.last_measurement[i] = left_encoder if i == 0 else right_encoder


        #     deltaS = (distance[0] + distance[1]) / 2.0
        #     deltaTheta = (distance[1] - distance[0]) / self.wheel_dist
        #     self.pose[2] = (self.pose[2] + deltaTheta) % (2 * pi)

        #     deltaSx = deltaS * cos(self.pose[2])
        #     deltaSy = deltaS * sin(self.pose[2])

        #     self.pose[0] += deltaSx
        #     self.pose[1] += deltaSy 

        #     # Publish X position
        #     msg = Float64()
        #     msg.data = self.pose[0]
        #     self.publisher.publish(msg)

    def __del__(self):
        self.get_logger().info('Terminating Node.')

def main(args=None):
    rclpy.init(args=args)
    node = kalmanFilter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()