"""Affiche en direct la grille d'occupation (/robot/map) et la trajectoire (/robot/pos).

Lancer avec :  ros2 run visualization map
Gris = inconnu, blanc = libre, noir = occupe. Le triangle rouge est le robot,
la ligne bleue sa trajectoire.
"""
import math
import os
import time

import numpy as np
import rclpy
import dearpygui.dearpygui as dpg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

from robot.steady_node import SteadyNode
from robot.occupancy_grid import grid_to_gray, write_png

# La ligne 0 du tableau de la grille est le y le plus petit, et ImPlot affiche la
# ligne 0 d'une texture en bas de la zone : pas de retournement necessaire.
# Si la carte apparait en miroir vertical sur votre machine, passez cette constante a True.
FLIP_VERTICAL = False

MAX_TRAIL = 5000


class MapVisualizer(SteadyNode):

    def __init__(self):
        super().__init__("map_visualizer")

        self.last_grid = None       # (data2d int8 [h, w], resolution, origin_x, origin_y)
        self.tex_tag = None
        self.tex_size = None
        self.trail_x, self.trail_y = [], []

        dpg.create_context()
        dpg.create_viewport(title='Occupancy grid', width=900, height=900)
        dpg.add_texture_registry(tag="map_tex_registry")

        with dpg.window(label="Carte", tag="map_window", width=880, height=860):
            with dpg.group(horizontal=True):
                dpg.add_button(label="Sauvegarder PNG", callback=self.save_png)
                dpg.add_button(label="Recentrer", callback=self.fit)
                self.status = dpg.add_text("En attente de /robot/map ...")
            with dpg.plot(label="Carte", width=-1, height=-1, equal_aspects=True):
                self.x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="x (m)")
                self.y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="y (m)")
                # Placeholder 1x1 : remplace par la vraie texture a la 1ere carte recue
                self._make_texture(np.full((1, 1), 205, dtype=np.uint8))
                self.img = dpg.add_image_series(self.tex_tag, bounds_min=[0, 0], bounds_max=[0.05, 0.05],
                                                parent=self.y_axis)
                self.trail = dpg.add_line_series([], [], parent=self.y_axis, label="trajectoire")
                self.robot = dpg.add_scatter_series([0.0], [0.0], parent=self.y_axis, label="robot")

        dpg.setup_dearpygui()
        dpg.show_viewport()

        map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/robot/map', self.map_callback, map_qos)
        self.create_subscription(Pose2D, '/robot/pos', self.pos_callback, 10)
        self.get_logger().info("Map visualizer lance (topics /robot/map et /robot/pos)")

    # ------------------------------------------------------------ texture

    def _make_texture(self, gray):
        """Cree une texture dynamique RGBA a partir d'une image uint8 (ligne 0 = bas de la carte)."""
        h, w = gray.shape
        if FLIP_VERTICAL:
            gray = np.flipud(gray)
        g = gray.astype(np.float32) / 255.0
        rgba = np.stack([g, g, g, np.ones_like(g)], axis=-1).ravel().tolist()
        tag = dpg.add_dynamic_texture(w, h, rgba, parent="map_tex_registry")
        old = self.tex_tag
        self.tex_tag, self.tex_size = tag, (w, h)
        return old

    def _grid_to_display(self, data2d):
        # grid_to_gray retourne verticalement (pour une image fichier) : on annule ici
        return np.flipud(grid_to_gray(data2d))

    # ----------------------------------------------------------- callbacks

    def map_callback(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        self.last_grid = (data, res, ox, oy)

        gray = self._display_gray(data)
        if self.tex_size == (w, h):
            g = gray if not FLIP_VERTICAL else np.flipud(gray)
            g = g.astype(np.float32) / 255.0
            dpg.set_value(self.tex_tag, np.stack([g, g, g, np.ones_like(g)], axis=-1).ravel().tolist())
        else:
            old = self._make_texture(gray)
            dpg.configure_item(self.img, texture_tag=self.tex_tag)
            if old is not None:
                dpg.delete_item(old)
        dpg.configure_item(self.img, bounds_min=[ox, oy], bounds_max=[ox + w * res, oy + h * res])
        known = int(np.count_nonzero(data >= 0))
        dpg.set_value(self.status, f"{w}x{h} cellules, {res * 100:.0f} cm/cellule, "
                                   f"{known * res * res:.1f} m2 explores")

    def _display_gray(self, data2d):
        return self._grid_to_display(data2d)

    def pos_callback(self, msg: Pose2D):
        self.trail_x.append(msg.x)
        self.trail_y.append(msg.y)
        if len(self.trail_x) > MAX_TRAIL:
            del self.trail_x[0], self.trail_y[0]
        dpg.set_value(self.trail, [self.trail_x, self.trail_y])
        dpg.set_value(self.robot, [[msg.x], [msg.y]])

    # -------------------------------------------------------------- boutons

    def fit(self, *_):
        dpg.fit_axis_data(self.x_axis)
        dpg.fit_axis_data(self.y_axis)

    def save_png(self, *_):
        if self.last_grid is None:
            dpg.set_value(self.status, "Rien a sauvegarder pour l'instant")
            return
        data = self.last_grid[0]
        folder = os.path.expanduser('~/robot_maps')
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, "map_gui_%s.png" % time.strftime("%Y%m%d_%H%M%S"))
        write_png(path, grid_to_gray(data), scale=4)
        dpg.set_value(self.status, f"Sauvegarde : {path}")
        self.get_logger().info(f"Carte sauvegardee : {path}")

    def destroy_node(self):
        if dpg.is_dearpygui_running():
            dpg.destroy_context()
        super().destroy_node()


def main():
    rclpy.init()
    node = MapVisualizer()
    while dpg.is_dearpygui_running() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        dpg.render_dearpygui_frame()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
