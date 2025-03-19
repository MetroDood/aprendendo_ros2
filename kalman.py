import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, LaserScan

def gaussian(x,mean,sigma):
        return (1/(sigma*sqrt(2*pi)))exp(-((x-mean)2)/(2*sigma*2))

class kalmanFilter(Node):

    
    def _init_(self):
        super()._init_('kalman')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.publisher = self.create_publisher(Float64, '/postition_x', qos_profile)
        
        #define initial states and fundamental variables - PLACEHOLDER
        Apose = [0,0,0] 
        wheel_radius = 0.033
        wheel_dist = 0.178
        measure = [0,0]
        prev_measure = [0,0]
        dist = [0,0]
        mapa = [2,4,6]

        #define possible measurement errors - PLACEHOLDER
        sigma_laser = 0.01
        sigma_movement = 0.002


    def run(self):
        control = 0
        cont = 0
        porta = 0 

        pass
        

    def scan_callback(self, msg):
        self.range[240]

    def joint_callback(self, msg):
        self.get


    #destrutor do Nó
    def _del_(self):
        self.get_logger().info('Terminating Node.')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = kalmanFilter()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if _name_ == '_main_':
    main()