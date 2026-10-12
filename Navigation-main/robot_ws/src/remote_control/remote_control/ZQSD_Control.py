import rclpy
from rclpy.node import Node
from msgs.msg import RPMs, WheelSpeeds
from rclpy.clock import Clock, ClockType
import pygame
from enum import IntEnum
from robot.robot_config import DEFAULT_RPM

USE_RPM = True
DEFAULT_PWM = 240

class ZQSD_Control(Node):

    class WheelDir(IntEnum):
        BACK = -1
        STOP = 0
        FORWARD = 1

    def __init__(self):
        super().__init__("ZQSD_control")
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)  #IMPORTANT FOR THIS TO WORK ON WSL2

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)
        self.pub_pwm = self.create_publisher(WheelSpeeds, "/robot/raw_command", 10)
        self.create_timer(0.1, lambda: self.check_input(USE_RPM))

        pygame.init()
        self.window = pygame.display.set_mode((300, 300))
        self.clock = pygame.time.Clock()

        self.get_logger().info("Remote control ZQSD node launched")

    def send_pwm(self, front_left: int, front_right: int, back_right: int, back_left: int):
        msg = WheelSpeeds()
        msg.front_left_wheel_speed = front_left
        msg.front_right_wheel_speed = front_right
        msg.back_right_wheel_speed = back_right
        msg.back_left_wheel_speed = back_left
        self.pub_pwm.publish(msg)

    def send_rpm(self, front_left: float, front_right: float, back_right: float, back_left: float):
        msg = RPMs()
        msg.front_left_rpm = front_left
        msg.front_right_rpm = front_right
        msg.back_right_rpm = back_right
        msg.back_left_rpm = back_left
        self.pub_cmd.publish(msg)

    def set_wheel_dirs(self, is_rpm: bool, front_left: WheelDir, front_right: WheelDir, back_right: WheelDir, back_left: WheelDir):
        def transform_wheel_dir(wheel_dir, power: float | int):
            if wheel_dir == self.WheelDir.BACK:
                return -power
            if wheel_dir == self.WheelDir.FORWARD:
                return power
            return 0.0

        def to_rpm(wheel_dir):
            return transform_wheel_dir(wheel_dir, DEFAULT_RPM)
        def to_pwm(wheel_dir):
            return int(transform_wheel_dir(wheel_dir, DEFAULT_PWM))

        if is_rpm:
            self.send_rpm(to_rpm(front_left), to_rpm(front_right), to_rpm(back_right), to_rpm(back_left))
        else:
            self.send_pwm(to_pwm(front_left), to_pwm(front_right), to_pwm(back_right), to_pwm(back_left))
        
        self.get_logger().info(f"Sent direction in {"rpm" if is_rpm else "pwm"}: {front_left.value, front_right.value, back_right.value, back_left.value}")


    def check_input(self, in_rpm: bool):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            
        f = self.WheelDir.FORWARD
        b = self.WheelDir.BACK
        s = self.WheelDir.STOP

        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]:
            self.set_wheel_dirs(in_rpm, f, f, f, f)
        elif keys[pygame.K_s]:
            self.set_wheel_dirs(in_rpm, b, b, b, b)
        elif keys[pygame.K_q]:
            self.set_wheel_dirs(in_rpm, b, f, b, f)
        elif keys[pygame.K_d]:
            self.set_wheel_dirs(in_rpm, f, b, f, b)
        elif keys[pygame.K_a]:
            self.set_wheel_dirs(in_rpm, b, f, f, b)
        elif keys[pygame.K_e]:
            self.set_wheel_dirs(in_rpm, f, b, b, f)
        else:
            self.set_wheel_dirs(in_rpm, s, s, s, s)

def main():
    rclpy.init()
    try:
        rclpy.spin(ZQSD_Control())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
