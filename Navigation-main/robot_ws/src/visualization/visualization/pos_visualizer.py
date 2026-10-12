import rclpy
import math
import dearpygui.dearpygui as dpg
from robot.steady_node import SteadyNode
from geometry_msgs.msg import Pose2D
import numpy as np

#this was written using AI, use it to look at what the lidar sees in real time

class PosVisualizer(SteadyNode):

    def __init__(self):
        super().__init__("pos_visualizer")

        # --- DearPyGui Setup ---
        dpg.create_context()
        dpg.create_viewport(title='Position Visualization', width=400, height=450)

        with dpg.window(label="Position", width=400, height=450) as self.window_id:
            with dpg.plot(label="Position Plot", width=-1, height=400, equal_aspects=True):
                dpg.add_plot_axis(dpg.mvXAxis, label="x")
                self.y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="y")
                self.pos_series = dpg.add_scatter_series(x=[0.0], y=[0.0], parent=self.y_axis)

            self.theta_text = dpg.add_text("Theta: 0.0")

        dpg.setup_dearpygui()
        dpg.show_viewport()

        self.create_subscription(Pose2D, '/robot/pos', self.pos_callback, 10)

        self.get_logger().info("Position visualizer node successfully launched")

    def pos_callback(self, msg: Pose2D):
        dpg.set_value(self.pos_series, [[msg.x], [msg.y]])
        dpg.set_value(self.theta_text, f"Theta: {msg.theta:.2f}")

    def destroy_node(self):
        # Destroy the GUI context when the node is destroyed
        if dpg.is_dearpygui_running():
            dpg.destroy_context()
        super().destroy_node()


def main():
    rclpy.init()
    node = PosVisualizer()

    # Custom render loop: alternate between checking ROS messages and drawing the GUI frame
    while dpg.is_dearpygui_running() and rclpy.ok():
        # timeout_sec=0 ensures it doesn't block the UI if there are no ROS messages
        rclpy.spin_once(node, timeout_sec=0)
        dpg.render_dearpygui_frame()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
