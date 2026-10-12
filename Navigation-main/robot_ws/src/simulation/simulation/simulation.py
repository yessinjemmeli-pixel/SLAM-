import rclpy
import math
import pygame
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
import pymunk
from time import time
from geometry_msgs.msg import Pose2D

# --- CONSTANTES ---
FPS = 30
FREQ_PUB = 0.1  # 10 Hz pour les capteurs simulés
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
PIXELS_PER_METER = 100.0

# Paramètres du robot physique
WHEEL_RADIUS = 0.04
LX = 0.15
LY = 0.15


class SimulationNode(Node):
    """
    Ce nœud ne gère que la simualtion.
    """
    def __init__(self):
        super().__init__("driver")

        # Publishers
        self.pub_lidar = self.create_publisher(Lidar, '/robot/lidar', 10)
        self.pub_lidar_obstacles = self.create_publisher(Lidar, '/robot/lidar_obstacles', 10)
        self.pub_encoders = self.create_publisher(RPMs, '/robot/encoders', 10)

        # Abonnements
        self.create_subscription(Pose2D, "/robot/pos", self.pos_callback, 10)
        self.create_subscription(RPMs, "/robot/command", self.cmd_callback, 10)

        # Commandes et état
        self.target_rpms = [0.0, 0.0, 0.0, 0.0]  # FL, FR, BR, BL
        self.current_rpms = [0.0, 0.0, 0.0, 0.0]

        # Position réelle (Ground Truth) et estimée
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_theta = 0.0

        self.estim_x = 0.0
        self.estim_y = 0.0
        self.estim_theta = 0.0

        self.last_update = time()

        # Espace physique Pymunk EN MÈTRES (Origine 0,0 au centre)
        self.space = pymunk.Space()
        self.walls = self.create_map_in_meters()
        self.robot_body, self.robot_shape = self.create_robot_in_meters()
        self.hit_points = [] # Points du lidar

        self.create_timer(FREQ_PUB, self.publish_sensors)
        self.get_logger().info("Simulation démarrée")

    def create_map_in_meters(self):
        static_body = self.space.static_body

        # Murs (Origine au centre)
        # Largeur: 12m (-6 à 6), Hauteur: 8m (-4 à 4)
        walls_segments = [
            # Murs extérieurs
            pymunk.Segment(static_body, (-3.5, 3.5), (3.5, 3.5), 0.03),
            pymunk.Segment(static_body, (3.5, 3.5), (3.5, -3.5), 0.03),
            pymunk.Segment(static_body, (3.5, -3.5), (-3.5, -3.5), 0.03),
            pymunk.Segment(static_body, (-3.5, -3.5), (-3.5, 3.5), 0.03),


            # Obstacles intérieurs
            #pymunk.Segment(static_body, (-3.5, 2.0), (-1.5, 0.0), 0.05),
            pymunk.Segment(static_body, (2.0, 2.5), (2.0, -1.5), 0.05),
            #pymunk.Segment(static_body, (-2.0, -2.5), (0.5, -2.5), 0.05)
        ]

        for wall in walls_segments:
            wall.elasticity = 0.5
            wall.friction = 0.5
            wall.filter = pymunk.ShapeFilter(categories=0b10)

        self.space.add(*walls_segments)
        return walls_segments

    def create_robot_in_meters(self):
        mass = 1.0
        # Taille du robot: 0.3 mètres
        size = (0.3, 0.3)
        moment = pymunk.moment_for_box(mass, size)
        body = pymunk.Body(mass, moment)
        body.position = (0.0, 0.0) # Départ au centre (0,0)

        shape = pymunk.Poly.create_box(body, size, radius=0.01)
        shape.elasticity = 0.0
        shape.friction = 0.0
        shape.filter = pymunk.ShapeFilter(categories=0b01, mask=0b10)

        self.space.add(body, shape)
        return body, shape

    def pos_callback(self, msg: Pose2D):
        self.estim_x = msg.x
        self.estim_y = msg.y
        self.estim_theta = msg.theta

    def cmd_callback(self, msg: RPMs):
        self.target_rpms = [
            msg.front_left_rpm, msg.front_right_rpm,
            msg.back_right_rpm, msg.back_left_rpm
        ]

    def update_physics(self):
        current_time = time()
        dt = current_time - self.last_update
        self.last_update = current_time
        if dt <= 0: return

        # 1. Inertie (Ce que les moteurs essaient de faire)
        for i in range(4):
            self.current_rpms[i] += (self.target_rpms[i] - self.current_rpms[i]) * 10.0 * dt

        # 2. Cinématique Directe (m/s) -> La vitesse voulue par le robot
        coef = (2 * math.pi) / 60.0
        w_fl = self.current_rpms[0] * coef
        w_fr = self.current_rpms[1] * coef
        w_rr = self.current_rpms[2] * coef
        w_bl = self.current_rpms[3] * coef

        vx_cmd = (WHEEL_RADIUS / 4.0) * ( w_fl + w_fr + w_bl + w_rr)
        vy_cmd = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr + w_bl - w_rr)
        w_cmd  = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-w_fl + w_fr - w_bl + w_rr)

        # Vitesses globales appliquées à Pymunk
        v_gx_cmd = vx_cmd * math.cos(self.gt_theta) - vy_cmd * math.sin(self.gt_theta)
        v_gy_cmd = vx_cmd * math.sin(self.gt_theta) + vy_cmd * math.cos(self.gt_theta)

        self.robot_body.velocity = (v_gx_cmd, v_gy_cmd)
        self.robot_body.angular_velocity = w_cmd

        # --- SAUVEGARDE AVANT PHYSIQUE ---
        old_x = self.robot_body.position.x
        old_y = self.robot_body.position.y
        old_theta = self.robot_body.angle

        # 3. Résolution des collisions
        self.space.step(dt)

        # --- LECTURE APRES PHYSIQUE ---
        self.gt_x = self.robot_body.position.x
        self.gt_y = self.robot_body.position.y
        self.gt_theta = self.robot_body.angle

        # Normalisation de l'angle
        while self.gt_theta > math.pi: self.gt_theta -= 2 * math.pi
        while self.gt_theta < -math.pi: self.gt_theta += 2 * math.pi

        # 4. CINÉMATIQUE INVERSE (Calcul de la réalité)
        # Vitesses globales réelles (bridées par les murs)
        real_v_gx = (self.gt_x - old_x) / dt
        real_v_gy = (self.gt_y - old_y) / dt

        # On utilise l'angle non normalisé pour la différence pour éviter les sauts
        real_w = (self.robot_body.angle - old_theta) / dt

        # Conversion des vitesses globales vers le repère local du robot
        real_vx = real_v_gx * math.cos(old_theta) + real_v_gy * math.sin(old_theta)
        real_vy = -real_v_gx * math.sin(old_theta) + real_v_gy * math.cos(old_theta)

        # Calcul des RPMs réels des 4 roues (Inverse d'un châssis Mecanum)
        k = LX + LY
        r = WHEEL_RADIUS
        coef_inv = 60.0 / (2 * math.pi)

        real_w_fl = (1.0 / r) * (real_vx - real_vy - k * real_w)
        real_w_fr = (1.0 / r) * (real_vx + real_vy + k * real_w)
        real_w_bl = (1.0 / r) * (real_vx + real_vy - k * real_w)
        real_w_rr = (1.0 / r) * (real_vx - real_vy + k * real_w)

        # 5. On écrase les RPMs par la réalité
        # Si le robot tape un mur, real_vx = 0, donc les encodeurs vaudront 0 !
        self.current_rpms[0] = real_w_fl * coef_inv
        self.current_rpms[1] = real_w_fr * coef_inv
        self.current_rpms[2] = real_w_rr * coef_inv
        self.current_rpms[3] = real_w_bl * coef_inv

    def publish_sensors(self):
        # --- ENCODERS ---
        rpm_msg = RPMs()
        rpm_msg.front_left_rpm = float(self.current_rpms[0])
        rpm_msg.front_right_rpm = float(self.current_rpms[1])
        rpm_msg.back_right_rpm = float(self.current_rpms[2])
        rpm_msg.back_left_rpm = float(self.current_rpms[3])
        self.pub_encoders.publish(rpm_msg)

        # --- LIDAR (Raycasting pur en mètres) ---
        lidar_msg = Lidar()
        obstacle_msg = Lidar()

        robot_pos = self.robot_body.position
        max_range_m = 6.0
        lidar_filter = pymunk.ShapeFilter(mask=0b10)
        D0_INFLUENCE = 2.0

        self.hit_points = []

        for i in range(360):
            local_angle = math.radians(i)
            # Angle global standard en maths
            global_angle = self.gt_theta + local_angle
            end_point = robot_pos + pymunk.Vec2d(max_range_m, 0).rotated(global_angle)

            query = self.space.segment_query_first(robot_pos, end_point, 0.0, lidar_filter)

            if query:
                dist_m = robot_pos.get_distance(query.point) # Distance le long du rayon
                self.hit_points.append(query.point)
            else:
                dist_m = max_range_m
                self.hit_points.append(end_point)

            lidar_msg.angles.append(float(local_angle))
            lidar_msg.distances.append(float(dist_m))

            if dist_m <= D0_INFLUENCE:
                obstacle_msg.angles.append(float(local_angle))
                obstacle_msg.distances.append(float(dist_m))

        self.pub_lidar.publish(lidar_msg)
        if len(obstacle_msg.distances) > 0:
            self.pub_lidar_obstacles.publish(obstacle_msg)


class Renderer:
    """
    Gère exclusivement Pygame. 
    """
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Simulateur Robot")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

    def pt_to_pixels(self, x_m, y_m):
        """Convertit les mètres (0,0 au centre, Y vers le haut) en pixels (0,0 en haut à gauche, Y vers le bas)"""
        pg_x = SCREEN_WIDTH / 2 + x_m * PIXELS_PER_METER
        pg_y = SCREEN_HEIGHT / 2 - y_m * PIXELS_PER_METER
        return pg_x, pg_y

    def draw(self, sim_node):
        self.screen.fill((15, 15, 20))

        # Dessiner les murs
        for wall in sim_node.walls:
            p1 = self.pt_to_pixels(wall.a.x, wall.a.y)
            p2 = self.pt_to_pixels(wall.b.x, wall.b.y)
            pygame.draw.line(self.screen, (200, 200, 200), p1, p2, max(2, int(wall.radius * 2 * PIXELS_PER_METER)))

        # Récupérer positions
        pg_x, pg_y = self.pt_to_pixels(sim_node.gt_x, sim_node.gt_y)
        pg_estim_x, pg_estim_y = self.pt_to_pixels(sim_node.estim_x, sim_node.estim_y)

        # Dessiner Lidar
        for point in sim_node.hit_points:
            pt_px = self.pt_to_pixels(point.x, point.y)
            pygame.draw.line(self.screen, (0, 50, 0), (pg_x, pg_y), pt_px, 1)
            pygame.draw.circle(self.screen, (0, 255, 0), (int(pt_px[0]), int(pt_px[1])), 2)

        # Dessiner le robot
        robot_px_size = int(0.3 * PIXELS_PER_METER) # 80 px
        robot_rect = pygame.Surface((robot_px_size, robot_px_size), pygame.SRCALPHA)
        pygame.draw.rect(robot_rect, (255, 255, 255), (0, 0, robot_px_size, robot_px_size), border_radius=10)

        # Ligne d'orientation
        pygame.draw.line(robot_rect, (255, 0, 0), (robot_px_size/2, robot_px_size/2), (robot_px_size, robot_px_size/2), 5)

        # Pygame tourne dans le sens trigonométrique, on passe l'angle de la simulation tel quel
        rotated_rect = pygame.transform.rotate(robot_rect, math.degrees(sim_node.gt_theta))
        rect = rotated_rect.get_rect(center=(pg_x, pg_y))
        self.screen.blit(rotated_rect, rect.topleft)

        # Position estimée
        pygame.draw.circle(self.screen, (0, 100, 255), (int(pg_estim_x), int(pg_estim_y)), 15, width=2)

        # Textes
        pos_text = self.font.render(
            f"Vraie Position: ({sim_node.gt_x:.2f}m, {sim_node.gt_y:.2f}m) θ: {math.degrees(sim_node.gt_theta):.1f}°",
            True, (255, 255, 255)
        )
        rpm_text = self.font.render(
            f"Encodeurs simulés: FL:{sim_node.current_rpms[0]:.0f} FR:{sim_node.current_rpms[1]:.0f} BR:{sim_node.current_rpms[2]:.0f} BL:{sim_node.current_rpms[3]:.0f}",
            True, (0, 255, 0)
        )

        self.screen.blit(pos_text, (10, 10))
        self.screen.blit(rpm_text, (10, 35))

        pygame.display.flip()
        self.clock.tick(FPS)


def main():
    rclpy.init()

    sim_node = SimulationNode()
    renderer = Renderer()

    running = True
    while running and rclpy.ok():
        rclpy.spin_once(sim_node, timeout_sec=0.0)

        # Evénements Pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Mise à jour physique
        sim_node.update_physics()

        # Affichage
        renderer.draw(sim_node)

    pygame.quit()
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
