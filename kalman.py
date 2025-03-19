import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, LaserScan
from math import sqrt, pi, exp, cos, sin

def gaussian(x, mean, sigma):
    return (1 / (sigma * sqrt(2 * pi))) * exp(-((x - mean) ** 2) / (2 * sigma ** 2))

class KalmanFilter(Node):

    def __init__(self):
        super().__init__('kalman')
        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.publisher = self.create_publisher(Float64, '/position_x', qos_profile)

        # Initialize pose and motion parameters
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.wheel_radius = 0.033
        self.wheel_dist = 0.178
        self.last_measurements = [0.0, 0.0]

        self.sigma_laser = 0.01
        self.sigma_movement = 0.002

        self.latest_laser = None
        self.latest_joint = None

    def run(self):
        rclpy.spin(self)

    def scan_callback(self, msg):
        range_values = np.array(msg.ranges[250:290])
        valid_range = range_values[np.isfinite(range_values)]
        if valid_range.size > 0:
            laser_measurement = np.mean(valid_range)
            self.get_logger().info(f'Laser measurement: {laser_measurement}')
            self.latest_laser = laser_measurement
            self.update()

    def joint_callback(self, msg):
        if len(msg.velocity) < 2:
            return  # Safety check if data is incomplete

        left_wheel_velocity = msg.velocity[0]
        right_wheel_velocity = msg.velocity[1]
        self.get_logger().info(f'Joint velocities: left={left_wheel_velocity}, right={right_wheel_velocity}')
        self.latest_joint = (left_wheel_velocity, right_wheel_velocity)
        self.update()

    def update(self):
        if self.latest_joint is not None:
            left_encoder = self.latest_joint[0]
            right_encoder = self.latest_joint[1]

            # Calculate distances from encoder values
            distancias = [0, 0]
            for i in range(2):
                diff = (left_encoder if i == 0 else right_encoder) - self.last_measurements[i]
                distancias[i] = diff * self.wheel_radius + np.random.normal(0, self.sigma_movement)
                self.last_measurements[i] = left_encoder if i == 0 else right_encoder

            # Compute movement updates
            deltaS = (distancias[0] + distancias[1]) / 2.0
            deltaTheta = (distancias[1] - distancias[0]) / self.wheel_dist
            self.pose[2] = (self.pose[2] + deltaTheta) % (2 * pi)

            deltaSx = deltaS * cos(self.pose[2])
            deltaSy = deltaS * sin(self.pose[2])

            self.pose[0] += deltaSx
            self.pose[1] += deltaSy  # Update y as well

            # Publish X position
            msg = Float64()
            msg.data = self.pose[0]
            self.publisher.publish(msg)
            self.get_logger().info(f'Published X Position: {self.pose[0]}')

    def __del__(self):
        self.get_logger().info('Terminating Node.')

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()