import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import RPMs, AsservParamChange
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from msgs.msg import RPMs, AsservParamChange, WheelSpeeds
from robot.robot_config import DEFAULT_RPM, HELP_MESSAGE
import readline #this is what makes the command line feel good (using arrow keys etc)


class Console(SteadyNode):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "setrpm"

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)
        self.pub_raw_cmd = self.create_publisher(WheelSpeeds, "/robot/raw_command", 10)
        self.pub_asserv_param = self.create_publisher(AsservParamChange, "robot/asserv_params", 10)
        self.pub_goal = self.create_publisher(Point, "/robot/automatic_goal", 10)
        self.pub_enable_auto = self.create_publisher(Bool, "/robot/enable_auto", 10)
        self.pub_pid_request = self.create_publisher(Empty, "/robot/pid_print_request", 10)
        self.pub_reset_pos = self.create_publisher(Empty, "/robot/reset_pos", 10)
        self.create_subscription(String, "/robot/pid_print_values", self.pid_print_callback, 10)

        # set up autocomplete in readline
        COMMANDS = ["setrpm", "setpwm", "stop", "printpid", "help", "setkp", "setki", "setkd", "automatic", "manual", "resetpos"]
        def completer(text, state):
            matches = [c for c in COMMANDS if c.startswith(text)]
            return matches[state] if state < len(matches) else None

        readline.set_completer(completer)
        readline.parse_and_bind("tab: complete")

        self.get_logger().info("Type \"help\" to get help on what commands exist and how to use them.")

    # ---------------- UI ----------------
    def show_menu(self):
        print("\n==========================\n")
        #log += "Commands: setrpm | stop | setkp | setki | setkd | help \n"
        #log += f"Last : {self.last_cmd}\n"
        #log += "==========================\n"
        # self.get_logger().info(log)

    def input_loop(self):
        while rclpy.ok():
            self.show_menu()
            cmd = input("> ")

            if cmd == "":
                cmd = self.last_cmd
            else:
                self.last_cmd = cmd

            try:
                self.process(cmd)
            except:
                self.get_logger().error("Couldn't parse given command. Check if your arguments have the right type !")

            timeout = 0
            if cmd == "printpid":
                timeout = 1 # we'll wait 1 sec for the answer

            # Receives messages with no waiting time unless we're expecting a message
            rclpy.spin_once(self, timeout_sec=timeout)

    def process(self, cmd: str):

        split_cmd = cmd.strip().split(" ")
        command_name = split_cmd[0]

        def get_float_arg(arg_index: int):
            return float(split_cmd[arg_index])

        def get_int_arg(arg_index: int):
            return int(split_cmd[arg_index])

        if command_name == "manual":
            msg = Bool()
            msg.data = False
            self.pub_enable_auto.publish(msg)
            self.get_logger().info("Manual mode activated.")
        elif command_name == "automatic":
            if len(split_cmd) != 3:
                self.get_logger().error("The automatic command takes exactly 2 arguments (x and y).")
                return

            # automatic.py s'active
            msg_enable = Bool()
            msg_enable.data = True
            self.pub_enable_auto.publish(msg_enable)

            # On envoie la cible
            goal_msg = Point()
            goal_msg.x = get_float_arg(1)
            goal_msg.y = get_float_arg(2)
            goal_msg.z = 0.0
            self.pub_goal.publish(goal_msg)
        elif command_name == "setrpm":
            msg = Bool()
            msg.data = False
            self.pub_enable_auto.publish(msg)

            m = RPMs()
            if len(split_cmd) == 1:
                m.front_left_rpm = DEFAULT_RPM
                m.front_right_rpm = DEFAULT_RPM
                m.back_right_rpm = DEFAULT_RPM
                m.back_left_rpm = DEFAULT_RPM
            elif len(split_cmd) == 2:
                m.front_left_rpm = get_float_arg(1)
                m.front_right_rpm = get_float_arg(1)
                m.back_right_rpm = get_float_arg(1)
                m.back_left_rpm = get_float_arg(1)
            elif len(split_cmd) == 5:
                m.front_left_rpm = get_float_arg(1)
                m.front_right_rpm = get_float_arg(2)
                m.back_right_rpm = get_float_arg(3)
                m.back_left_rpm = get_float_arg(4)
            else:
                self.get_logger().error(f"The setrpm command either takes none, one or four arguments")

            self.pub_cmd.publish(m)
        elif command_name == "setpwm":
            m = WheelSpeeds()
            if len(split_cmd) == 2:
                m.front_left_wheel_speed = get_int_arg(1)
                m.front_right_wheel_speed = get_int_arg(1)
                m.back_right_wheel_speed = get_int_arg(1)
                m.back_left_wheel_speed = get_int_arg(1)
            elif len(split_cmd) == 5:
                m.front_left_wheel_speed = get_int_arg(1)
                m.front_right_wheel_speed = get_int_arg(2)
                m.back_right_wheel_speed = get_int_arg(3)
                m.back_left_wheel_speed = get_int_arg(4)
            else:
                self.get_logger().error(f"The setpwm command either takes one or four arguments")

            self.pub_raw_cmd.publish(m)
        elif command_name == "stop":
            msg = Bool()
            msg.data = False
            self.pub_enable_auto.publish(msg)

            m = RPMs()
            m.front_left_rpm = 0.
            m.front_right_rpm = 0.
            m.back_right_rpm = 0.
            m.back_left_rpm = 0.

            self.pub_cmd.publish(m)
        elif command_name == "printpid":
            self.pub_pid_request.publish(Empty())
        elif command_name == "resetpos":
            self.pub_reset_pos.publish(Empty())
        elif command_name == "setkp" or command_name == "setki" or command_name == "setkd":
            if len(split_cmd) != 2 and len(split_cmd) != 3:
                self.get_logger().error(f"set... commands takes 2 or 3 arguments")
                return

            param_id = ""
            if command_name == "setkp":
                param_id = "kp"
            elif command_name == "setki":
                param_id = "ki"
            elif command_name == "setkd":
                param_id = "kd"

            new_value = 0.
            if len(split_cmd) == 3:
                new_value = get_float_arg(2)
            else:
                new_value = get_float_arg(1)

            def send_asserv_param_message(wheel_id: str):
                m = AsservParamChange()
                m.wheel_id = wheel_id
                m.param_id = param_id
                if wheel_id != "frontleft" and wheel_id != "frontright"and wheel_id != "backright" and wheel_id != "backleft":
                    self.get_logger().error(f"Wrong wheel id used ! Use frontleft, frontright, backright or backleft")
                    return

                m.new_value = new_value

                self.pub_asserv_param.publish(m)

            if len(split_cmd) == 3:
                send_asserv_param_message(split_cmd[1])
            else:
                send_asserv_param_message("frontleft")
                send_asserv_param_message("frontright")
                send_asserv_param_message("backleft")
                send_asserv_param_message("backright")

        elif command_name == "help":
            self.get_logger().info(HELP_MESSAGE)
        else:
            self.get_logger().error(f"Unknown command \"{command_name}\"")
            return

    def pid_print_callback(self, msg: String):
        self.get_logger().info(msg.data)


def main():
    rclpy.init()
    node = Console()
    try:
        # This is technically bad since it blocks the main thread
        # But I had issues with the previous version with some desyncs
        # Console shouldn't really be receiving data anyways so i'd much
        # rather leave it like this. Feel free to change it though
        # Update : The console now receives messages but only in between commands !
        # There is still some desyncs unfortunately tho
        node.input_loop()
    except KeyboardInterrupt:
        print("\n")
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
