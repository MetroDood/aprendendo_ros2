import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import numpy as np
import math
import random
import tf_transformations
from math import sqrt, pi, exp, cos, sin

def gaussian(x, mean, sigma):
    return (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mean) ** 2) / (2 * sigma ** 2))

class extendedKalman(Node):

    def __init__(self):
        super().__init__('kalman')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        
        self.publisher_posicao = self.create_publisher(Pose2D, '/posicao', qos_profile)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        #dados do robô
        self.raio = 0.033
        self.distancia_rodas = 0.178
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta

        self.v = 0.2  # m/s
        self.raio = 1.0  # m
        self.w = self.v / self.raio
        self.dt = 0.1  # s

        #incerteza
        self.sigma_x = 0.004
        self.sigma_y = 0.004  
        self.sigma_z = 0.004  
        self.sigma_th = math.radians(0.01)
        self.sigma_v = 0.0005  
        self.sigma_w = math.radians(0.01)  

        
        self.sigma_z_x = 0.005 
        self.sigma_z_y = 0.005  

        #Definição de raio de giro do robô e dt

        self.v = 0.5
        self.raio = 2
        self.w = self.v / self.raio
        self.dt = 0.1

    def odom_callback(self, msg):
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        _, _, self.yaw = tf_transformations.euler_from_quaternion([x, y, z, w])

        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

    def update(self):

        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.w
        self.publisher_cmd_vel.publish(twist)

        self.sigma_xRand = self.sigma_x * random.gauss(0, 0.3)
        self.sigma_yRand = self.sigma_y * random.gauss(0, 0.3)
        self.sigma_thRand = self.sigma_th * random.gauss(0, 0.3)
        self.sigma_vRand = self.sigma_v * random.gauss(0, 0.2)
        self.sigma_wRand = self.sigma_w * random.gauss(0, 0.1)

        Pv = self.pose.copy()
        Pv[0] = (Pv[0] + self.sigma_xRand) + (self.v + self.sigma_vRand) * math.cos(Pv[2] + self.sigma_thRand) * self.dt
        Pv[1] = (Pv[1] + self.sigma_yRand) + (self.v + self.sigma_vRand) * math.sin(Pv[2] + self.sigma_thRand) * self.dt
        Pv[2] = (Pv[2] + self.sigma_thRand) + (self.w + self.sigma_wRand) * self.dt
        self.pose = Pv

        C = np.array([[1, 0, 0], [0, 1, 0]])
        R = np.array([[self.sigma_z_x**2, 0], [0, self.sigma_z_y**2]])
        incerteza = np.dot(np.sqrt(R), np.random.randn(2, 1)).flatten()
        y = np.dot(C, Pv[:3]) + incerteza

        # Estimativa com EKF
        Pe = self.pose.copy()
        Pe[0] += self.v * math.cos(Pe[2]) * self.dt
        Pe[1] += self.v * math.sin(Pe[2]) * self.dt
        Pe[2] += self.w * self.dt

        Q = np.array([[self.sigma_x**2, 0, 0],
                      [0, self.sigma_y**2, 0],
                      [0, 0, self.sigma_th**2]])

        M = np.array([[self.sigma_v**2, 0],
                      [0, self.sigma_w**2]])

        F = np.array([[1, 0, -self.v * math.sin(Pe[2]) * self.dt],
                      [0, 1, self.v * math.cos(Pe[2]) * self.dt],
                      [0, 0, 1]])

        G = np.array([[math.cos(Pe[2]) * self.dt, 0],
                      [math.sin(Pe[2]) * self.dt, 0],
                      [0, self.dt]])

        H = C
        z = np.dot(H, Pe)
        K = np.dot(P, np.dot(H.T, np.linalg.pinv(np.dot(H, np.dot(P, H.T)) + R)))
        Pe = Pe + np.dot(K, (y - z))
        P = np.dot((np.eye(Q.shape[0]) - np.dot(K, H)), P)

        self.pose = Pe
        self.publicar_posicao()

    def publicar_posicao(self):
        msg = Pose2D()
        msg.x = self.pose[0]
        msg.y = self.pose[1]
        msg.theta = self.pose[2]
        self.publisher_posicao.publish(msg)
        self.get_logger().info(f'x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {math.degrees(msg.theta):.2f}°')

    def __del__(self):
        self.get_logger().info('Terminate Node')


def main(args=None):
    rclpy.init(args=args)
    node = extendedKalman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
