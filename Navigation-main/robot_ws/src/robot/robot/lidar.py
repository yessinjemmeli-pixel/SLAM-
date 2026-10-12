# https://github.com/Fypur/Navigation/wiki/Lidar

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from msgs.msg import Lidar
from robot.robot_config import LIDAR_MIN_DIST, LIDAR_MAX_RANGE_M, LIDAR_OBSTACLE_RANGE

# Peut-être pas très utile mais on définit la QoS adaptée aux capteurs : best-effort, depth = 1
SENSOR_QOS = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar')

        # -- Paramètres déclarés --
        self.declare_parameter('min_distance', LIDAR_MIN_DIST) # en dessous le bruit est trop proche
        self.declare_parameter('max_distance', LIDAR_MAX_RANGE_M) # portée du Lidar dans notre cas pas necessairement tres grande
        self.declare_parameter('obstacle_range', LIDAR_OBSTACLE_RANGE) # seuil de detection

        self.min_dist = float(self.get_parameter('min_distance').value or 0.05)
        self.max_dist = float(self.get_parameter('max_distance').value or 6.0)
        self.obs_range = float(self.get_parameter('obstacle_range').value or 2.0)

        # -- Abonnnement au driver officiel
        # -- Le driver rplidar_ros publie sur le /scan en sensor_msgs/LaserScan

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            SENSOR_QOS,
        )

        # -- Publication --
        # -- Topic principal : tous les points valides (format utilisé pour notre robot)

        self.pub_lidar = self.create_publisher(Lidar, '/robot/lidar', 10)

        # Utilisé directement par l'algorithme de plannification et pour la détection d'obstacles
        # Les obstacles sont tous les points à une distance inférieure à self.obs_range
        self.pub_obstacles = self.create_publisher(Lidar, '/robot/lidar_obstacles', 10)

        self.get_logger().info(
            f"""Lidar Node launched, min={self.min_dist}m, max={self.max_dist}m, seuil_obstacle={self.obs_range}m"""
        )

    # -- Callback principal --

    def scan_callback(self, scan: LaserScan):
        """
        Reçoit un LaserScan complet du driver C1 et publie deux messages Lidar:
            - /robot/lidar : tous les points valides
            - /robot/lidar_obstacles : points à moins de obstacle_range metres
        """
        angles_all = []
        dists_all = []
        angles_obs = []
        dists_obs = []

        angle = 0.0

        for r in scan.ranges:
            # On filtre les valeurs invalides
            if math.isfinite(r):
                # on garde les angles en radians ici
                angles_all.append(angle)
                dists_all.append(r)

                if r <= self.obs_range:
                    angles_obs.append(angle)
                    dists_obs.append(r)

            angle += scan.angle_increment


        # Publication sur topic principal
        msg_all = Lidar()
        msg_all.angles = angles_all
        msg_all.distances = dists_all
        self.pub_lidar.publish(msg_all)

        # Publication topic obstacles (TOUJOURS publier, même si vide)
        #if angles_obs:
        msg_obs = Lidar()
        msg_obs.angles = angles_obs
        msg_obs.distances = dists_obs
        self.pub_obstacles.publish(msg_obs)

        self.get_logger().debug(f"Scan reçu : {len(dists_all)} pts valides, {len(dists_obs)} obstacles (<{self.obs_range}m)")
        # self.get_logger().debug(f"angle_max={scan.angle_max} rad, angle_min={scan.angle_min} rad, angle_inc={scan.angle_increment} rad")

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()