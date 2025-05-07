import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
import random
import tf_transformations
from tf_transformations import quaternion_from_euler


class ExtendedKalmanFilter(Node):

    def __init__(self):
        super().__init__('kalman')
        qos_profile = QoSProfile(depth=10)

        # Publishers e subscribers
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.publisher_pose = self.create_publisher(Pose, '/pose', qos_profile)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Timer para controle de movimento e update do filtro
        self.dt = 0.1
        self.create_timer(self.dt, self.send_velocity)
        self.create_timer(self.dt, self.update)

        # Estado inicial do robô
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.v = 0.5  # m/s
        self.raio = 2.0  # m
        self.w = self.v / self.raio

        # Inicialização das incertezas
        self.sigma_x = 0.004
        self.sigma_y = 0.004
        self.sigma_th = math.radians(0.01)
        self.sigma_v = 0.0005
        self.sigma_w = math.radians(0.01)
        self.sigma_z_x = 0.005
        self.sigma_z_y = 0.005

        self.P = np.eye(3) * 0.01
        self.received_odom = False

    def send_velocity(self):
        vel = Twist()
        vel.linear.x = self.v
        vel.angular.z = self.w
        self.publisher_cmd_vel.publish(vel)

    def odom_callback(self, msg):
        # Armazena a posição para ser usada na medição simulada
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.received_odom = True

    def update(self):
        if not self.received_odom:
            return

        # Simula a leitura com ruído (y)
        Pv = self.pose.copy()
        Pv[0] += self.v * math.cos(Pv[2]) * self.dt + self.sigma_x * random.gauss(0, 0.3)
        Pv[1] += self.v * math.sin(Pv[2]) * self.dt + self.sigma_y * random.gauss(0, 0.3)
        Pv[2] += self.w * self.dt + self.sigma_th * random.gauss(0, 0.3)

        # Simula a incerteza de medição
        C = np.array([[1, 0, 0], [0, 1, 0]])
        R = np.array([[self.sigma_z_x**2, 0], [0, self.sigma_z_y**2]])
        incerteza = np.dot(np.sqrt(R), np.random.randn(2, 1)).flatten()
        y = np.dot(C, Pv[:3]) + incerteza

        # Predição
        Pe = self.pose.copy()
        Pe[0] += self.v * math.cos(Pe[2]) * self.dt
        Pe[1] += self.v * math.sin(Pe[2]) * self.dt
        Pe[2] += self.w * self.dt

        # Covariâncias e matrizes do EKF
        Q = np.array([
            [self.sigma_x**2, 0, 0],
            [0, self.sigma_y**2, 0],
            [0, 0, self.sigma_th**2]
        ])

        F = np.array([
            [1, 0, -self.v * math.sin(Pe[2]) * self.dt],
            [0, 1,  self.v * math.cos(Pe[2]) * self.dt],
            [0, 0, 1]
        ])

        H = C
        z = np.dot(H, Pe)
        S = np.dot(H, np.dot(self.P, H.T)) + R
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))

        Pe = Pe + np.dot(K, (y - z))
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.P)

        self.pose = Pe
        self.publish_position()

    def publish_position(self):
        msg = Pose()
        msg.position.x = self.pose[0]
        msg.position.y = self.pose[1]

        quat = quaternion_from_euler(0, 0, self.pose[2])
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]

        self.publisher_pose.publish(msg)
        self.get_logger().info(f'x: {msg.position.x:.2f}, y: {msg.position.y:.2f}, theta: {math.degrees(self.pose[2]):.2f}°')

    def __del__(self):
        self.get_logger().info('Terminate Node')


def main(args=None):
    rclpy.init(args=args)
    node = ExtendedKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()