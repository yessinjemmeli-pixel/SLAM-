import rclpy
import math
import dearpygui.dearpygui as dpg
from robot.steady_node import SteadyNode
from msgs.msg import Lidar

#this was written using AI, use it to look at what the lidar sees in real time

class LidarVisualizer(SteadyNode):

    def __init__(self):
        super().__init__("lidar_visualizer")

        # --- DearPyGui Setup ---
        dpg.create_context()
        dpg.create_viewport(title='lidar_visualization', width=800, height=800)
        dpg.setup_dearpygui()

        # Create the window and graph on initialization
        with dpg.window(label="Lidar Window", width=800, height=800, no_collapse=True) as self.window:
            # equal_aspects=True ensures the 2D map doesn't stretch and distort the distances
            with dpg.plot(label="Lidar 2D Map", height=-1, width=-1, equal_aspects=True):
                self.x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="X Axis")
                self.y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Y Axis")

                # Create an empty scatter series to hold our points
                self.scatter_series = dpg.add_scatter_series([], [], parent=self.y_axis)

        dpg.show_viewport()

        # --- ROS Setup ---
        self.create_subscription(Lidar, '/robot/lidar', self.lidar_callback, 10)
        self.get_logger().info("Lidar visualizer node successfully launched")

    def lidar_callback(self, msg: Lidar):
        xs = []
        ys = []

        # Convert polar coordinates to cartesian coordinates around (0, 0)
        for angle, distance in zip(msg.angles, msg.distances):
            # Note: math.cos and math.sin expect radians.
            # If your msg.angles are in degrees, wrap it in math.radians(angle)
            x = distance * math.cos(angle)
            y = -distance * math.sin(angle) # Negate y to flip the graph so that 0° is facing "up" on the screen
            xs.append(x)
            ys.append(y)

        # dpg.configure_item entirely replaces the old data with the new arrays,
        # perfectly fulfilling the requirement to clear the graph before drawing new points.
        dpg.configure_item(self.scatter_series, x=xs, y=ys)

    def destroy_node(self):
        # Destroy the GUI context when the node is destroyed
        if dpg.is_dearpygui_running():
            dpg.destroy_context()
        super().destroy_node()


def main():
    rclpy.init()
    node = LidarVisualizer()

    # Custom render loop: alternate between checking ROS messages and drawing the GUI frame
    while dpg.is_dearpygui_running() and rclpy.ok():
        # timeout_sec=0 ensures it doesn't block the UI if there are no ROS messages
        rclpy.spin_once(node, timeout_sec=0)
        dpg.render_dearpygui_frame()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
