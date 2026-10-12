# all of this code is documented at https://github.com/Fypur/Navigation/wiki/Encoders

import rclpy
import rclpy.logging
from robot.steady_node import SteadyNode
from enum import IntEnum
import time
from msgs.msg import RPMs
import collections
from robot.robot_config import FRONT_LEFT_FLIPPED, FRONT_RIGHT_FLIPPED, BACK_RIGHT_FLIPPED, BACK_LEFT_FLIPPED, RPMS_UPDATING_PERIOD

# This node uses pigpio instead of GPIO. This is so that the callback when the B (or A) signal is falling we have a callback in about
# 1μs instead of 1ms. This is important because if the callback isn't made fast enough, the reading of the A (or B) signal isn't correct
# We need it to be correct in order to know if the wheel is spinning forwards or backwards
try:
    import pigpio
except ImportError:
    rclpy.logging.get_logger("encoders").error(
        "Couldn't import pigpio. Run: pip install pigpio and make sure pigpiod is running (sudo pigpiod).")
    exit()


class PinMap(IntEnum):
    ENCODER_FRONT_LEFT_A = 17
    ENCODER_FRONT_LEFT_B = 27
    ENCODER_FRONT_RIGHT_A = 20
    ENCODER_FRONT_RIGHT_B = 21
    ENCODER_BACK_RIGHT_A = 19
    ENCODER_BACK_RIGHT_B = 26
    ENCODER_BACK_LEFT_A = 2
    ENCODER_BACK_LEFT_B = 3

# With pigpio we trigger on both edges of A (instead of one edge of B),
# giving 2x the pulses
PULSES_PER_REV = 234.3 * 2


class Encoders(SteadyNode):

    class EncoderSignalPin:

        def __init__(self, pi: pigpio.pi, a_pin: int, b_pin: int, reversed: bool, logger) -> None:
            self.pi = pi
            self.a_pin = a_pin
            self.b_pin = b_pin
            self.reversed_motor = reversed
            self.sliding_average_window = 0.3
            self.logger = logger

            self.positive_encoder_pulse_timestamps = collections.deque()
            self.negative_encoder_pulse_timestamps = collections.deque()
            self.rpm = 0.0

            pi.set_mode(a_pin, pigpio.INPUT)
            pi.set_mode(b_pin, pigpio.INPUT)
            pi.set_pull_up_down(a_pin, pigpio.PUD_UP)
            pi.set_pull_up_down(b_pin, pigpio.PUD_UP)

            # Seed B's level before attaching callbacks
            self.b_level = pi.read(b_pin)

            # B callback just keeps b_level current.
            # Since we fire on every edge, b_level is always accurate by the
            # time on_a fires — no need to read the pin live
            # This is kinda eh logic but it works
            self.callback_b = pi.callback(b_pin, pigpio.EITHER_EDGE, self.on_b)
            self.callback_a = pi.callback(a_pin, pigpio.EITHER_EDGE, self.on_a)

            logger.info(f"Pin setup for pins a={a_pin} b={b_pin}")

        def on_b(self, gpio, level, tick):
            self.b_level = level

        def on_a(self, gpio, level, tick):
            # Determine direction from the quadrature state transition.
            # b_level is always current: if B changed, on_b already fired.
            #   A rising  + B low  → forward
            #   A rising  + B high → backward
            #   A falling + B high → forward
            #   A falling + B low  → backward
            if level == 1:
                forward = self.b_level == 0
            else:
                forward = self.b_level == 1

            t = time.time()
            if forward:
                self.positive_encoder_pulse_timestamps.append(t)
            else:
                self.negative_encoder_pulse_timestamps.append(t)

        def update_rpm(self):
            """
            Updates the RPM of the wheel associated with this encoder and returns it.
            The RPM is positive when spinning forwards, and negative when spinning backwards.
            RPM is calculated using a sliding average (moyenne glissante)
            """
            current_time = time.time()

            def remove_old_timestamps(encoder_pulse_timestamps: collections.deque):
                while encoder_pulse_timestamps and \
                      current_time - encoder_pulse_timestamps[0] > self.sliding_average_window:
                    encoder_pulse_timestamps.popleft()

            remove_old_timestamps(self.positive_encoder_pulse_timestamps)
            remove_old_timestamps(self.negative_encoder_pulse_timestamps)

            pulse_count = len(self.positive_encoder_pulse_timestamps) - len(self.negative_encoder_pulse_timestamps)
            self.rpm = 60 * pulse_count / (self.sliding_average_window * PULSES_PER_REV)

            if not self.reversed_motor:
                return self.rpm
            else:
                return -self.rpm

        def cancel(self):
            self.callback_a.cancel()
            self.callback_b.cancel()

    def __init__(self):
        super().__init__("encoders")

        def is_raspberry_pi():
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    return 'Raspberry Pi' in f.read()
            except FileNotFoundError:
                return False

        if not is_raspberry_pi():
            self.get_logger().error("This node can only run a raspberry pi !")
            exit()

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpiod — is it running? (sudo pigpiod)")
            exit()

        logger = self.get_logger()
        self.encoderSignalPins = [
            self.EncoderSignalPin(self.pi, PinMap.ENCODER_FRONT_LEFT_A, PinMap.ENCODER_FRONT_LEFT_B, FRONT_LEFT_FLIPPED, logger),
            self.EncoderSignalPin(self.pi, PinMap.ENCODER_FRONT_RIGHT_A, PinMap.ENCODER_FRONT_RIGHT_B, FRONT_RIGHT_FLIPPED, logger),
            self.EncoderSignalPin(self.pi, PinMap.ENCODER_BACK_RIGHT_A, PinMap.ENCODER_BACK_RIGHT_B, BACK_RIGHT_FLIPPED, logger),
            self.EncoderSignalPin(self.pi, PinMap.ENCODER_BACK_LEFT_A, PinMap.ENCODER_BACK_LEFT_B, BACK_LEFT_FLIPPED, logger),
        ]

        self.pub = self.create_publisher(RPMs, "/robot/encoders", 10)
        self.create_timer(RPMS_UPDATING_PERIOD, self.send_RPMs)
        self.get_logger().info("Encoders node launched")

    def send_RPMs(self):
        msg = RPMs()
        msg.front_left_rpm = self.encoderSignalPins[0].update_rpm()
        msg.front_right_rpm = self.encoderSignalPins[1].update_rpm()
        msg.back_right_rpm = self.encoderSignalPins[2].update_rpm()
        msg.back_left_rpm = self.encoderSignalPins[3].update_rpm()

        self.get_logger().debug(
            f"measured RPMs: {msg.front_left_rpm:.1f}, {msg.front_right_rpm:.1f}, {msg.back_right_rpm:.1f}, {msg.back_left_rpm:.1f}"
        )
        self.pub.publish(msg)

    def destroy_node(self):
        for enc in self.encoderSignalPins:
            enc.cancel()
        self.pi.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = Encoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
