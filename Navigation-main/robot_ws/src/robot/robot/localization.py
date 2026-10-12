import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
import math, time
import numpy as np
import open3d as o3d
from robot.robot_config import (WHEEL_RADIUS, LX, LY, LIDAR_MIN_DIST, LIDAR_MAX_RANGE_M, LOC_DIST_THRESHOLD,
                                LOC_ANGLE_THRESHOLD, angle_wrap, ICP_MAX_CORRESPOND_DIST, ICP_MAX_JUMP_DIST,
                                ICP_MAX_JUMP_ANGLE, ICP_VOXEL_SIZE, ICP_FITNESS_THRESHOLD, ICP_INLIER_RMSE_THRESHOLD,
                                ICP_MAP_UPDATE_FITNESS, ICP_MIN_POINTS, ENCODER_NOISE_THRESHOLD)

class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')

        # Position fusionnée (publiée)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # Position odométrique brute (pour debug / monitorer la dérive)
        self.odom_x, self.odom_y, self.odom_theta = 0.0, 0.0, 0.0

        self.last_time = time.time()
        self.ref_pcd   = None          # Dernier scan de référence (frame globale)

        self.last_ref_x = 0.0
        self.last_ref_y = 0.0
        self.last_ref_theta = 0.0

        # Seuils pour déclencher une mise à jour de la carte
        self.dist_threshold = LOC_DIST_THRESHOLD
        self.angle_threshold = LOC_ANGLE_THRESHOLD

        self.create_subscription(RPMs,  '/robot/encoders', self.encoders_callback, 1)
        self.create_subscription(Lidar, '/robot/lidar', self.lidar_callback, 1)
        self.create_subscription(Empty, '/robot/reset_pos', self.reset, 1)
        self.pub_pos = self.create_publisher(Pose2D, '/robot/pos', 10)
        self.get_logger().info("Noeud Localisation démarré (open3d ICP)")

    #  ENCODEURS

    def encoders_callback(self, msg: RPMs):

        to_rad = (2 * math.pi) / 60.0
        w_fl = msg.front_left_rpm  * to_rad
        w_fr = msg.front_right_rpm * to_rad
        w_rl = msg.back_left_rpm   * to_rad
        w_rr = msg.back_right_rpm  * to_rad

        if max(abs(w_fl), abs(w_fr), abs(w_rl), abs(w_rr)) < ENCODER_NOISE_THRESHOLD:
            return

        # Cinématique inverse
        vx = (WHEEL_RADIUS / 4.0) * ( w_fl + w_fr + w_rl + w_rr)
        vy = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-w_fl + w_fr - w_rl + w_rr)

        now = time.time()
        #dt  = now - self.last_time
        #self.last_time = now
        dt = max(1e-4, min(now - self.last_time, 1.0))
        self.last_time = now

        # Incrément dans le repère du robot
        # On calcule delta une seule fois et on l'applique aux DEUX positions
        # L'odométrie et la position fusionnée partagent le même mouvement relatif
        # seule leur origine absolue diffère (l'ICP corrige la fusionnée)

        '''cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * cos_t - vy * sin_t) * dt
        self.y += (vx * sin_t + vy * cos_t) * dt
        self.theta  = angle_wrap(self.theta + wz * dt)
        self._publish()'''

        def apply_increment(x, y, theta):
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            new_x = x + (vx * cos_t - vy * sin_t) * dt
            new_y = y + (vx * sin_t + vy * cos_t) * dt
            new_theta = angle_wrap(theta + wz * dt)
            return new_x, new_y, new_theta

        # Position odométrique brute (intégration pure, sans correction ICP)
        self.odom_x, self.odom_y, self.odom_theta = apply_increment(
            self.odom_x, self.odom_y, self.odom_theta
        )

        # Position fusionnée : on applique le même déplacement relatif
        # (l'ICP ne touche que self.x/y/theta, jamais odom_*)
        self.x, self.y, self.theta = apply_increment(
            self.x, self.y, self.theta
        )

        self._publish()

    #  LIDAR  –  correction via open3d

    def lidar_callback(self, msg: Lidar):
        # On utilise la pose FUSIONNÉE pour projeter le scan en coordonnées globales
        curr_pcd = self._scan_to_global_pcd(msg)
        if len(curr_pcd.points) < ICP_MIN_POINTS:
            return

        # Premier scan : on l'enregistre comme référence, pas de correction possible
        if self.ref_pcd is None or len(self.ref_pcd.points) < 30:
            self.ref_pcd = curr_pcd
            return

        # ICP : aligner le scan courant sur le scan de référence
        # init = identité car les encoders ont déjà pré-aligné les deux nuages

        #curr_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        #self.ref_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))

        result = o3d.pipelines.registration.registration_icp(
            source=curr_pcd,
            target=self.ref_pcd,
            max_correspondence_distance=ICP_MAX_CORRESPOND_DIST,       # Max d'écart accepté
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            #o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )

        # Vérification de la qualité du recalage
        # fitness = fraction de points matchés ; rmse = erreur résiduelle
        if result.fitness < ICP_FITNESS_THRESHOLD or result.inlier_rmse > ICP_INLIER_RMSE_THRESHOLD:
            self.get_logger().warn(
                f"ICP peu fiable (fitness={result.fitness:.2f}, rmse={result.inlier_rmse:.3f}) — scan ignoré"
            )
            self.ref_pcd = curr_pcd   # On repart quand même de ce scan
            return

        T = result.transformation   # Matrice 4×4 : correction dans le repère global

        # On calcule l'ampleur de la correction demandée par l'ICP
        #corr_dist = math.hypot(T[0, 3], T[1, 3])
        #corr_angle = abs(math.atan2(T[1, 0], T[0, 0]))

        # Si l'ICP demande un saut de plus de 15 cm ou de plus de 20 degrés d'un coup
        #if corr_dist > 0.20 or corr_angle > 0.35:
        #self.get_logger().warn(
        #f"Rejet ICP : Correction trop brutale (dist={corr_dist:.2f}m, angle={math.degrees(corr_angle):.1f}°)"
        #)
        #return # On fait confiance aux encodeurs pour ce tick, on ignore l'ICP

        # Appliquer la correction T à la pose du robot
        # T * [x, y, 0, 1]^T  :  position corrigée
        pos_h  = np.array([self.x, self.y, 0.0, 1.0])
        corr   = T @ pos_h
        new_x     = float(corr[0])
        new_y     = float(corr[1])
        new_theta = angle_wrap(self.theta + math.atan2(T[1, 0], T[0, 0]))
        #self.x     = float(corr[0])
        #self.y     = float(corr[1])
        #self.theta = self._wrap(self.theta + math.atan2(T[1, 0], T[0, 0]))

        jump_dist = math.hypot(new_x - self.x, new_y - self.y)
        jump_angle = abs(math.atan2(T[1, 0], T[0, 0]))

        # Tolérance de 15 cm et ~20 degrés max d'un coup valeurs ad hoc
        # à ajuster selon le bruit des encodeurs et la fréquence des scans
        if jump_dist > ICP_MAX_JUMP_DIST or jump_angle > ICP_MAX_JUMP_ANGLE:
            self.get_logger().warn(
                f"Rejet ICP : Saut trop violent (dist={jump_dist:.2f}m, angle={math.degrees(jump_angle):.1f}°)"
            )
            return

        # Correction appliquée UNIQUEMENT sur la position fusionnée
        # self.odom_* n'est JAMAIS touché par l'ICP
        # On peut donc à tout moment calculer la dérive
        self.x = new_x
        self.y = new_y
        self.theta = new_theta
        self._publish()

        self.get_logger().debug(
            f"ICP OK  fitness={result.fitness:.2f}  "
            f"fused=({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.1f}°)  "
            f"drift=({self.x - self.odom_x:.3f}, {self.y - self.odom_y:.3f})"
        )

        # Mettre à jour le scan de référence avec la pose corrigée
        #self.ref_pcd = self._scan_to_global_pcd(msg)
        #self._publish()

        # Mise à jour de la carte
        dx = self.x - self.last_ref_x
        dy = self.y - self.last_ref_y
        dtheta = abs(angle_wrap(self.theta - self.last_ref_theta))
        dist = math.hypot(dx, dy)

        # Si le robot a parcouru dist_threshold OU a tourné de angle_threshold
        # depuis la dernière mise à jour, ET que l'ICP était de bonne qualité, on met à jour la carte
        if (dist > self.dist_threshold or dtheta > self.angle_threshold) and result.fitness > ICP_MAP_UPDATE_FITNESS:
            nouveau_scan = self._scan_to_global_pcd(msg)

            # On fusionne l'ancien nuage avec le nouveau
            self.ref_pcd += nouveau_scan

            # On filtre le nuage fusionné.
            # Sans ça, le nuage devient gigantesque, l'ICP va ramer.
            # Un "voxel" de taille B cm garde un seul point par cube de BxBxB cm.
            self.ref_pcd = self.ref_pcd.voxel_down_sample(voxel_size=ICP_VOXEL_SIZE)

            # On mémorise la position de cette mise à jour
            self.last_ref_x = self.x
            self.last_ref_y = self.y
            self.last_ref_theta = self.theta

            self.get_logger().info(f"CARTE MISE À JOUR ! Taille du nuage: {len(self.ref_pcd.points)} points")

        self._publish()

    #  UTILITAIRES
    def _scan_to_global_pcd(self, msg: Lidar) -> o3d.geometry.PointCloud:
        """
        Convertit un scan Lidar (angles, distances) en nuage de points 3D
        dans le repère global, en utilisant la pose courante (self.x/y/theta).
        Z = 0
        """
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        pts = []
        for a, d in zip(msg.angles, msg.distances):
            if math.isfinite(d) and LIDAR_MIN_DIST < d < (LIDAR_MAX_RANGE_M - 0.05):
                lx = d * math.cos(a)
                ly = d * math.sin(a)
                pts.append([
                    self.x + lx * cos_t - ly * sin_t,
                    self.y + lx * sin_t + ly * cos_t,
                    0.0
                ])
        pcd = o3d.geometry.PointCloud()
        if pts:
            pcd.points = o3d.utility.Vector3dVector(np.array(pts, dtype=np.float64))
        return pcd

    def _publish(self):
        out = Pose2D()
        out.x, out.y, out.theta = float(self.x), float(self.y), float(self.theta)
        self.pub_pos.publish(out)

    def reset(self, msg: Empty):
        # Position fusionnée (publiée)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # Position odométrique brute (pour debug / monitorer la dérive)
        self.odom_x, self.odom_y, self.odom_theta = 0.0, 0.0, 0.0

        self.last_time = time.time()
        self.ref_pcd = None  # Dernier scan de référence (frame globale)

        self.last_ref_x = 0.0
        self.last_ref_y = 0.0
        self.last_ref_theta = 0.0
    #@staticmethod
    #def angle_wrap(a: float) -> float:
    #return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
