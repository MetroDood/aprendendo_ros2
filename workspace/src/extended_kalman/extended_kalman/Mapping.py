#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
import tf_transformations
from bresenham import bresenham
import numpy as np
import math
import time


class Mapping(Node):

    def __init__(self):
        super().__init__('grid_mapping')

        # ---- Parâmetros do mapa ----
        self.resolution = 0.05             # [m/célula]
        self.width      = 800              # nº de células eixo X  -> 800*0.05 = 40 m
        self.height     = 800              # nº de células eixo Y
        self.origin_x   = -self.width  * self.resolution / 2.0   # centro em (0,0)
        self.origin_y   = -self.height * self.resolution / 2.0

        # -1 = desconhecido, 0 = livre, 100 = ocupado
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)

        # ---- Pose do robô ----
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.pose_ok = False

        q_profile_sensor = QoSProfile(depth=10,
                                      reliability=QoSReliabilityPolicy.BEST_EFFORT)
        q_profile_odom   = QoSProfile(depth=50)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            q_profile_odom)

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            q_profile_sensor)

        self.pub_map = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)

        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info('Nó de mapeamento inicializado.')

    def odom_cb(self, msg: Odometry):
        """Guarda posição (x,y) e orientação (yaw) do robô."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.pose_ok = True

    def scan_cb(self, msg: LaserScan):
        """Processa cada feixe do laser e atualiza o mapa."""
        if not self.pose_ok:
            return  

        robot_cell = self.world_to_map(self.x, self.y)
        if robot_cell is None:
            return  

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue
            if r > msg.range_max or r < msg.range_min:
                angle += msg.angle_increment
                continue

            global_angle = angle + self.yaw

            end_x = self.x + r * math.cos(global_angle)
            end_y = self.y + r * math.sin(global_angle)

            end_cell = self.world_to_map(end_x, end_y)
            if end_cell is None:
                angle += msg.angle_increment
                continue

            line = list(bresenham(robot_cell[0], robot_cell[1],
                                  end_cell[0], end_cell[1]))

            # Todas exceto a última são livres
            for cx, cy in line[:-1]:
                if self.in_bounds(cx, cy):
                    self.grid[cy, cx] = 0  # livre

            # A última é ocupada
            ox, oy = line[-1]
            if self.in_bounds(ox, oy):
                self.grid[oy, ox] = 100  # obstáculo

            angle += msg.angle_increment

    # ----------------------------------------------------------
    # FUNÇÕES DE APOIO
    # ----------------------------------------------------------
    def world_to_map(self, wx: float, wy: float):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        if self.in_bounds(mx, my):
            return (mx, my)
        return None

    def in_bounds(self, mx: int, my: int) -> bool:
        return 0 <= mx < self.width and 0 <= my < self.height

    # ----------------------------------------------------------
    # PUBLICAÇÃO DO MAPA
    # ----------------------------------------------------------
    def publish_map(self):
        """Envia OccupancyGrid para o tópico /map."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        info = MapMetaData()
        info.resolution = self.resolution
        info.width      = self.width
        info.height     = self.height
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        info.map_load_time = msg.header.stamp

        msg.info = info
        msg.data = self.grid.flatten().tolist()

        self.pub_map.publish(msg)

    # ----------------------------------------------------------
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Mapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()