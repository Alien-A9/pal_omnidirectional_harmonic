#!/usr/bin/python3
import math
import re
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ANSI_REGEX = re.compile(r'\x1b\[[0-9;]*m')

CSI = "\033["
RESET = f"{CSI}0m"
BOLD = f"{CSI}1m"
DIM = f"{CSI}2m"
CYAN = f"{CSI}36m"
GREEN = f"{CSI}32m"
YELLOW = f"{CSI}33m"
RED = f"{CSI}31m"
MAGENTA = f"{CSI}35m"
CLEAR_LINE = f"{CSI}2K\r"


def pad_ansi(text: str, width: int) -> str:
    """Pad string containing ANSI escape codes to an exact visual width."""
    vis_len = len(ANSI_REGEX.sub('', text))
    return text + (' ' * max(0, width - vis_len))


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OmniControllerNode(Node):
    def __init__(self):
        super().__init__('omni_controller')
        self.declare_parameter('ui_mode', 'simple')  # 'simple' or 'box'
        self. declare_parameter('show_odom', True)

        self.ui_mode = self.get_parameter('ui_mode').get_parameter_value().string_value
        self.show_odom = self.get_parameter('show_odom').get_parameter_value().bool_value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_deg = 0.0
        self.meas_vx = 0.0
        self.meas_vy = 0.0
        self.meas_wz = 0.0
        self.odom_received = False

        self.r = 0.0762
        self.k_geom = (0.488 + 0.44635) / 2.0

        self.timer = self.create_timer(0.05, self.publish_twist)

    def odom_callback(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.yaw_deg = math.degrees(quaternion_to_yaw(msg.pose.pose.orientation))
        self.meas_vx = msg.twist.twist.linear.x
        self.meas_vy = msg.twist.twist.linear.y
        self.meas_wz = msg.twist.twist.angular.z
        self.odom_received = True

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.angular.z = float(self.wz)
        self.pub.publish(msg)

    def set_velocity(self, vx: float, vy: float, wz: float):
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.publish_twist()

    def compute_wheel_omegas(self):
        fl = (self.vx - self.vy - self.k_geom * self.wz) / self.r
        rr = (self.vx - self.vy + self.k_geom * self.wz) / self.r
        fr = (self.vx + self.vy + self.k_geom * self.wz) / self.r
        rl = (self.vx + self.vy - self.k_geom * self.wz) / self.r
        return fl, fr, rl, rr


def render_ui(node: OmniControllerNode, input_buf: str, status_msg: str):
    fl, fr, rl, rr = node.compute_wheel_omegas()

    cmd_str = (
        f"{GREEN}Cmd Twist {RESET}: "
        f"vx={BOLD}{node.vx:+6.2f}{RESET} m/s   "
        f"vy={BOLD}{node.vy:+6.2f}{RESET} m/s   "
        f"wz={BOLD}{node.wz:+6.2f}{RESET} rad/s"
    )
    whl_str = (
        f"{MAGENTA}Wheels ω  {RESET}: "
        f"FL={fl:+6.2f} | FR={fr:+6.2f} | RL={rl:+6.2f} | RR={rr:+6.2f} rad/s"
    )

    if not node.show_odom:
        odom_str = f"{DIM}Odometry  : [Hidden - press 'o' + Enter to show]{RESET}"
    elif node.odom_received:
        odom_str = (
            f"{CYAN}Odom Pose {RESET}: "
            f"x={BOLD}{node.pos_x:+7.3f}{RESET} m   "
            f"y={BOLD}{node.pos_y:+7.3f}{RESET} m   "
            f"yaw={BOLD}{node.yaw_deg:+7.2f}°{RESET}"
        )
    else:
        odom_str = f"{CYAN}Odom Pose {RESET}: {YELLOW}Waiting for /odom...{RESET}"

    help_str = f"{DIM}Keys: <vx vy wz> | 's': Stop | 'o': Odom | 'u': UI ({node.ui_mode}) | 'q': Quit{RESET}"

    if node.ui_mode == 'box':
        W = 68
        lines = [
            f"{CYAN}┌{'─' * W}┐{RESET}",
            f"{CYAN}│{RESET} {pad_ansi(f'{BOLD}PAL OMNI CONTROLLER{RESET}', W - 1)}{CYAN}│{RESET}",
            f"{CYAN}├{'─' * W}┤{RESET}",
            f"{CYAN}│{RESET} {pad_ansi(cmd_str, W - 1)}{CYAN}│{RESET}",
            f"{CYAN}│{RESET} {pad_ansi(whl_str, W - 1)}{CYAN}│{RESET}",
            f"{CYAN}│{RESET} {pad_ansi(odom_str, W - 1)}{CYAN}│{RESET}",
            f"{CYAN}├{'─' * W}┤{RESET}",
            f"{CYAN}│{RESET} {pad_ansi(help_str, W - 1)}{CYAN}│{RESET}",
            f"{CYAN}└{'─' * W}┘{RESET}",
            f" {YELLOW}Status{RESET}  : {status_msg}",
            f" {BOLD}vx vy wz > {RESET}{input_buf}",
        ]
    else:
        sep = f"{DIM}{'─' * 68}{RESET}"
        lines = [
            f"{BOLD}{CYAN}PAL OMNI CONTROLLER{RESET}  {help_str}",
            sep,
            f"  {cmd_str}",
            f"  {whl_str}",
            f"  {odom_str}",
            sep,
            f"  {YELLOW}Status{RESET}   : {status_msg}",
            f"  {BOLD}vx vy wz > {RESET}{input_buf}",
        ]

    # \033[H moves to top-left; \033[J clears any leftover lines below the prompt
    # while leaving the cursor right after `input_buf`
    output = f"{CSI}H" + "\n".join(f"{CLEAR_LINE}{line}" for line in lines) + f"{CSI}J"
    sys.stdout.write(output)
    sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = OmniControllerNode()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    input_buf = ""
    status_msg = f"{GREEN}Ready.{RESET}"

    sys.stdout.write(f"{CSI}2J{CSI}H")
    sys.stdout.flush()

    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            # Single-threaded spin_once prevents the C++ thread abort on exit
            rclpy.spin_once(node, timeout_sec=0.02)

            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                ch = sys.stdin.read(1)
                if ch in ('\n', '\r'):
                    cmd = input_buf.strip().lower()
                    input_buf = ""

                    if cmd in ('q', 'quit', 'exit'):
                        node.set_velocity(0.0, 0.0, 0.0)
                        break
                    elif cmd in ('s', 'stop', '0', ''):
                        node.set_velocity(0.0, 0.0, 0.0)
                        status_msg = f"{YELLOW}Stopped (0.0, 0.0, 0.0).{RESET}"
                    elif cmd in ('o', 'odom'):
                        node.show_odom = not node.show_odom
                        status_msg = f"{CYAN}Odom display: {'ON' if node.show_odom else 'OFF'}{RESET}"
                    elif cmd in ('u', 'ui'):
                        node.ui_mode = 'box' if node.ui_mode == 'simple' else 'simple'
                        status_msg = f"{CYAN}Switched UI to '{node.ui_mode}' mode.{RESET}"
                    else:
                        parts = cmd.replace(',', ' ').split()
                        try:
                            vx = float(parts[0])
                            vy = float(parts[1]) if len(parts) > 1 else 0.0
                            wz = float(parts[2]) if len(parts) > 2 else 0.0
                            node.set_velocity(vx, vy, wz)
                            status_msg = f"{GREEN}Sent: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}{RESET}"
                        except ValueError:
                            status_msg = f"{RED}Invalid! Enter numbers: vx vy wz{RESET}"

                elif ch in ('\x7f', '\x08'):
                    input_buf = input_buf[:-1]
                elif ch == '\x03':
                    node.set_velocity(0.0, 0.0, 0.0)
                    break
                elif ch.isprintable():
                    input_buf += ch

            render_ui(node, input_buf, status_msg)

    except KeyboardInterrupt:
        node.set_velocity(0.0, 0.0, 0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        sys.stdout.write(f"\n{CLEAR_LINE}{GREEN}Clean exit. Robot halted.{RESET}\n")
        sys.stdout.flush()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()