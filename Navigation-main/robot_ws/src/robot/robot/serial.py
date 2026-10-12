import struct
import rclpy
from robot.steady_node import SteadyNode
from robot.serial_utils import open_serial_port
from serial.serialutil import SerialException
from enum import Enum
from msgs.msg import WheelSpeeds
from robot.robot_config import BAUDRATE, ARDUINO_CONNECT_PERIOD

#This uses robust serial, all explanations can be found on github
#https://github.com/araffin/arduino-robust-serial


class Order(Enum):
    """
    Pre-defined orders that are sent over to the arduino
    """

    HELLO = 5
    ALREADY_CONNECTED = 6
    WHEELSPEEDS = 7


class Serial(SteadyNode):

    def __init__(self):
        super().__init__("serial")

        self.declare_parameter("serial_port", value="/dev/ttyUSB0")

        try:
            serial_port: str = self.get_parameter("serial_port").get_parameter_value().string_value
            self.serial_file = open_serial_port(serial_port=serial_port, baudrate=BAUDRATE, timeout=None)
        except SerialException as e:
            self.get_logger().error(f"Could not find the arduino serial port. Full error: {e}")
            exit()

        self.create_subscription(WheelSpeeds, "/robot/wheels", self.send_wheel_speeds, 10)

        self.get_logger().info("Serial node successfully launched")

        self.connected_to_arduino = False
        self.connect_timer = self.create_timer(ARDUINO_CONNECT_PERIOD, self.connect_to_arduino)

    def send_wheel_speeds(self, msg: WheelSpeeds):
        """Sends the wheel speeds in the WheelSpeeds msg over to the arduino"""

        if not (self.connected_to_arduino):
            return

        self.write_order(Order.WHEELSPEEDS)
        self.write_i16(msg.front_left_wheel_speed)
        self.write_i16(msg.front_right_wheel_speed)
        self.write_i16(msg.back_right_wheel_speed)
        self.write_i16(msg.back_left_wheel_speed)

        self.get_logger().debug(
            f"Sent wheel speeds message with speeds {msg.front_left_wheel_speed}, {msg.front_right_wheel_speed}, {msg.back_right_wheel_speed}, {msg.back_left_wheel_speed}"
        )

    def connect_to_arduino(self):
        self.get_logger().info("Connect to arduino function launched")
        if self.connected_to_arduino:
            self.connect_timer.cancel()
            self.get_logger().info("Cancelled timer");
            return

        self.get_logger().info("Waiting for arduino...")
        self.write_order(Order.HELLO)

        if(not self.serial_file.in_waiting):
            self.get_logger().info("No bytes received")
            return

        bytes_array = bytearray(self.serial_file.read(1))

        if not bytes_array:
            self.get_logger().info("Invalid bytes array")
            return

        if bytes_array[0] in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            self.connected_to_arduino = True
            self.connect_timer.cancel()
            self.get_logger().info("Connected successfully to Arduino")


    def write_i8(self, value: int):
        if -128 <= value <= 127:
            self.serial_file.write(struct.pack('<b', value))
        else:
            print("Value error:{}".format(value))

    def write_i16(self, value):
        self.serial_file.write(struct.pack('<h', value))

    def write_order(self, order: Order):
        self.write_i8(order.value)


def main():
    rclpy.init()
    node = Serial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
