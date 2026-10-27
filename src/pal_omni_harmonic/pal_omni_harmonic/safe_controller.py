#!/usr/bin/env python3
import math
import re
import select
import sys
import termios
import tty
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

try:
    from sim_bringup.msg import Obstacle
    USE_SIM_BRINGUP_MSG = True
except ImportError:
    USE_SIM_BRINGUP_MSG = False

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


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def solve_cbf_qp_box(u_nom: np.ndarray, a: np.ndarray, b: float, v_max: float) -> np.ndarray:
    """
    Exact closed-form solver for the 2D box-constrained CBF-QP (Eq. 10):
        u* = argmin_{u} ||u - u_nom||^2
        s.t. 2(x - o)^T u + alpha * h(x) >= 0
             -v_max <= vx <= v_max
             -v_max <= vy <= v_max
    """
    u_box = np.clip(u_nom, -v_max, v_max)
    if np.dot(a, u_box) + b >= 0.0:
        return u_box

    candidates = []
    a_norm_sq = float(np.dot(a, a))

    # 1. Unconstrained projection onto active CBF hyperplane: a^T u + b = 0
    if a_norm_sq > 1e-12:
        u_proj = u_nom - ((np.dot(a, u_nom) + b) / a_norm_sq) * a
        if np.all(np.abs(u_proj) <= v_max + 1e-9):
            candidates.append(np.clip(u_proj, -v_max, v_max))

    # 2. Active box edges: u_x = +-v_max
    for ux_f in (-v_max, v_max):
        uy_opt = float(np.clip(u_nom[1], -v_max, v_max))
        if a[0] * ux_f + a[1] * uy_opt + b >= -1e-9:
            candidates.append(np.array([ux_f, uy_opt]))
        elif abs(a[1]) > 1e-9:
            uy_c = -(b + a[0] * ux_f) / a[1]
            if -v_max - 1e-9 <= uy_c <= v_max + 1e-9:
                candidates.append(np.array([ux_f, float(np.clip(uy_c, -v_max, v_max))]))

    # 3. Active box edges: u_y = +-v_max
    for uy_f in (-v_max, v_max):
        ux_opt = float(np.clip(u_nom[0], -v_max, v_max))
        if a[0] * ux_opt + a[1] * uy_f + b >= -1e-9:
            candidates.append(np.array([ux_opt, uy_f]))
        elif abs(a[0]) > 1e-9:
            ux_c = -(b + a[1] * uy_f) / a[0]
            if -v_max - 1e-9 <= ux_c <= v_max + 1e-9:
                candidates.append(np.array([float(np.clip(ux_c, -v_max, v_max)), uy_f]))

    # 4. Box corners
    for ux_v in (-v_max, v_max):
        for uy_v in (-v_max, v_max):
            if a[0] * ux_v + a[1] * uy_v + b >= -1e-9:
                candidates.append(np.array([ux_v, uy_v]))

    if candidates:
        return min(candidates, key=lambda u: float(np.sum((u - u_nom) ** 2)))

    if a_norm_sq > 1e-12:
        scale = v_max / max(abs(a[0]), abs(a[1]), 1e-6)
        return np.clip(a * scale, -v_max, v_max)
    return np.zeros(2)


class SafeController(Node):
    def __init__(self, init_goal=None):
        super().__init__('safe_controller')

        # Target goal state
        if init_goal is not None:
            self.x1_d, self.x2_d = float(init_goal[0]), float(init_goal[1])
            self.has_goal = True
            self.goal_reached = False
        else:
            self.x1_d, self.x2_d = 0.0, 0.0
            self.has_goal = False
            self.goal_reached = True

        # Robot state (from /odom)
        self.x1 = None
        self.x2 = None
        self.yaw = 0.0
        self.yaw_ref = None

        # Obstacle state (from /obstacle)
        self.ox = None
        self.oy = None
        self.r = None

        # ---------------- Tuned PID & CBF Parameters ----------------
        # Tuned for 34kg PAL Omni-Base with v_max = 0.5 m/s at 20 Hz
        self.Kp = 1.35        # Crisp proportional response
        self.Ki = 0.08        # Eliminates steady-state friction offset near goal
        self.Kd = 0.22        # Damps approach velocity to prevent overshoot
        self.i_max = 0.25     # Tight anti-windup clamp
        self.k_yaw = 1.2      # Heading lock gain to prevent passive yaw drift

        # CBF-QP Parameters
        self.r_robot = 0.44   # PAL omni_base circumradius (m)
        self.delta = 0.08     # Extra safety margin (m)
        self.alpha = 1.0      # Smooth barrier decay rate (lower = earlier, wider curve)
        self.v_max = 0.5      # Max velocity per axis (m/s)
        self.goal_tol = 0.025 # Convergence tolerance (2.5 cm)

        # Internal PID states
        self.err_int = np.zeros(2)
        self.prev_x = None
        self.d_err_filt = np.zeros(2)  # Low-pass filtered derivative

        # Live telemetry values
        self.dist_goal = 0.0
        self.h_val = 0.0
        self.u_nom = np.zeros(2)
        self.u_star = np.zeros(2)
        self.cbf_active = False

        # History logs for plotting
        self.t_elapsed = 0.0
        self.log_t = []
        self.log_x1 = []
        self.log_x2 = []
        self.log_h = []
        self.log_unom_x = []
        self.log_unom_y = []
        self.log_ustar_x = []
        self.log_ustar_y = []
        self.goals_history = []
        if self.has_goal:
            self.goals_history.append((self.x1_d, self.x2_d))

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        obstacle_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if USE_SIM_BRINGUP_MSG:
            self.create_subscription(Obstacle, '/obstacle', self.obstacle_sim_cb, obstacle_qos)
        else:
            self.create_subscription(Point, '/obstacle', self.obstacle_fortress_cb, obstacle_qos)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20 Hz control timer
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

    def set_new_goal(self, gx: float, gy: float):
        self.x1_d = float(gx)
        self.x2_d = float(gy)
        self.has_goal = True
        self.goal_reached = False
        self.err_int = np.zeros(2)      # Reset integral on new setpoint
        self.d_err_filt = np.zeros(2)   # Avoid derivative kick on setpoint step
        self.goals_history.append((self.x1_d, self.x2_d))

    def clear_logs(self):
        self.t_elapsed = 0.0
        self.log_t.clear()
        self.log_x1.clear()
        self.log_x2.clear()
        self.log_h.clear()
        self.log_unom_x.clear()
        self.log_unom_y.clear()
        self.log_ustar_x.clear()
        self.log_ustar_y.clear()
        self.goals_history = [(self.x1_d, self.x2_d)] if self.has_goal else []

    def stop_robot(self):
        self.goal_reached = True
        self.u_nom = np.zeros(2)
        self.u_star = np.zeros(2)
        self.cmd_pub.publish(Twist())

    def odom_callback(self, msg: Odometry):
        self.x1 = msg.pose.pose.position.x
        self.x2 = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.yaw_ref is None:
            self.yaw_ref = self.yaw

    def obstacle_sim_cb(self, msg):
        self.ox = msg.center.x
        self.oy = msg.center.y
        self.r = msg.radius

    def obstacle_fortress_cb(self, msg: Point):
        self.ox = msg.x
        self.oy = msg.y
        self.r = msg.z

    def control_loop(self):
        if self.x1 is None:
            self.cmd_pub.publish(Twist())
            return
        if self.r is None:
            return

        x = np.array([self.x1, self.x2])
        o = np.array([self.ox, self.oy])
        rel_obs = x - o
        dist_obs = float(np.linalg.norm(rel_obs))
        r_s = self.r + self.r_robot + self.delta
        self.h_val = float(np.dot(rel_obs, rel_obs) - (r_s ** 2))

        if not self.has_goal or self.goal_reached:
            self.prev_x = x.copy()
            return

        x_d = np.array([self.x1_d, self.x2_d])
        err = x_d - x
        self.dist_goal = float(np.linalg.norm(err))

        # Check convergence
        if self.dist_goal <= self.goal_tol:
            self.stop_robot()
            return

        # ----------------------------------------------------------------
        # 1. Tuned Nominal PID Controller (with derivative on measurement)
        # ----------------------------------------------------------------
        # Derivative on measurement (-dx/dt) prevents derivative kick when changing x_d dynamically
        if self.prev_x is None:
            raw_d_err = np.zeros(2)
        else:
            raw_d_err = -(x - self.prev_x) / self.dt
        self.prev_x = x.copy()

        # Exponential low-pass filter on derivative (tau ~ 0.1s)
        self.d_err_filt = 0.6 * self.d_err_filt + 0.4 * raw_d_err

        # Conditional integration: only accumulate Ki near the goal (< 0.6 m) when CBF is not saturating
        if self.dist_goal < 0.6 and not self.cbf_active:
            self.err_int = np.clip(self.err_int + err * self.dt, -self.i_max, self.i_max)
        elif self.cbf_active:
            self.err_int *= 0.85

        u_nom = self.Kp * err + self.Ki * self.err_int + self.Kd * self.d_err_filt

        # Smooth tangent symmetry-breaker when heading dead-on into obstacle center
        if dist_obs < r_s + 0.65 and np.linalg.norm(u_nom) > 1e-4:
            cos_ang = np.dot(rel_obs, u_nom) / (dist_obs * np.linalg.norm(u_nom))
            if cos_ang < -0.985:
                tangent = np.array([-rel_obs[1], rel_obs[0]]) / max(dist_obs, 1e-6)
                u_nom = u_nom + 0.22 * tangent

        # ----------------------------------------------------------------
        # 2. Safety Filter (CBF-QP) -> u_star = [vx_global, vy_global]
        # ----------------------------------------------------------------
        a = 2.0 * rel_obs
        b = self.alpha * self.h_val

        u_star = solve_cbf_qp_box(u_nom, a, b, self.v_max)
        self.cbf_active = bool(np.linalg.norm(u_star - np.clip(u_nom, -self.v_max, self.v_max)) > 1e-3)

        self.u_nom = u_nom
        self.u_star = u_star

        # ----------------------------------------------------------------
        # 3. Rotate Global Velocity u_star to Robot Body Frame + Yaw Lock
        # ----------------------------------------------------------------
        c_yaw = math.cos(self.yaw)
        s_yaw = math.sin(self.yaw)
        vx_body = float(c_yaw * u_star[0] + s_yaw * u_star[1])
        vy_body = float(-s_yaw * u_star[0] + c_yaw * u_star[1])

        cmd = Twist()
        cmd.linear.x = vx_body
        cmd.linear.y = vy_body
        if self.yaw_ref is not None:
            yaw_err = wrap_angle(self.yaw_ref - self.yaw)
            cmd.angular.z = float(np.clip(self.k_yaw * yaw_err, -0.6, 0.6))
        self.cmd_pub.publish(cmd)

        # Log telemetry for plots
        self.log_t.append(self.t_elapsed)
        self.log_x1.append(self.x1)
        self.log_x2.append(self.x2)
        self.log_h.append(self.h_val)
        self.log_unom_x.append(float(u_nom[0]))
        self.log_unom_y.append(float(u_nom[1]))
        self.log_ustar_x.append(float(u_star[0]))
        self.log_ustar_y.append(float(u_star[1]))
        self.t_elapsed += self.dt

    def plot_results(self, save_path: str = 'cbf_results.png') -> str:
        if not self.log_t:
            return "No trajectory data logged yet!"

        try:
            import matplotlib.pyplot as plt
        except ImportError as e:
            return f"Matplotlib import error: {e} (run: pip install 'numpy<2')"

        r_s = self.r + self.r_robot + self.delta
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

        # 1. Trajectory + Obstacle + Safe Boundary
        ax1 = axes[0]
        ax1.add_patch(plt.Circle((self.ox, self.oy), self.r, color='crimson', alpha=0.65, label=f'Obstacle (r={self.r:.2f}m)'))
        ax1.add_patch(plt.Circle((self.ox, self.oy), r_s, color='crimson', fill=False, linestyle='--', linewidth=1.8, label=f'Safe Radius $r_s$ ({r_s:.2f}m)'))
        ax1.plot(self.log_x1, self.log_x2, 'b-', linewidth=2.2, label='Robot Trajectory $x(t)$')
        ax1.plot(self.log_x1[0], self.log_x2[0], 'go', markersize=8, label='Start')
        for idx, (gx, gy) in enumerate(self.goals_history):
            lbl = 'Goal $x_d$' if idx == len(self.goals_history) - 1 else None
            ax1.plot(gx, gy, 'r*', markersize=12, label=lbl)
        ax1.set_title('2D Trajectory & Obstacle Avoidance')
        ax1.set_xlabel('$x_1$ (m)')
        ax1.set_ylabel('$x_2$ (m)')
        ax1.axis('equal')
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='best')

        # 2. Barrier function h(x(t))
        ax2 = axes[1]
        ax2.plot(self.log_t, self.log_h, color='darkorange', linewidth=2.0, label='$h(x(t))$')
        ax2.axhline(0.0, color='black', linestyle='--', linewidth=1.2, label='$h(x) = 0$ boundary')
        ax2.set_title('Barrier Function $h(x(t))$ Over Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('$h(x)$ ($m^2$)')
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc='best')

        # 3. Nominal vs Safe Control Inputs
        ax3 = axes[2]
        ax3.plot(self.log_t, self.log_unom_x, 'b--', alpha=0.55, label='$u_{nom,x}$')
        ax3.plot(self.log_t, self.log_unom_y, 'g--', alpha=0.55, label='$u_{nom,y}$')
        ax3.plot(self.log_t, self.log_ustar_x, 'b-', linewidth=1.8, label='$u^*_x$')
        ax3.plot(self.log_t, self.log_ustar_y, 'g-', linewidth=1.8, label='$u^*_y$')
        ax3.axhline(self.v_max, color='gray', linestyle=':', linewidth=1.0, label=f'$\\pm v_{{max}}$ ({self.v_max} m/s)')
        ax3.axhline(-self.v_max, color='gray', linestyle=':', linewidth=1.0)
        ax3.set_title('Nominal $u_{nom}(t)$ vs. Safe Input $u^*(t)$')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.grid(True, linestyle=':', alpha=0.7)
        ax3.legend(loc='best', fontsize=8)

        plt.tight_layout()
        plt.savefig(save_path, dpi=200)
        plt.show(block=False)
        plt.pause(0.1)
        return f"Saved plots to '{save_path}' ({len(self.log_t)} samples)."


def render_ui(node: SafeController, input_buf: str, status_msg: str):
    sep = f"{DIM}{'─' * 76}{RESET}"

    if node.x1 is None:
        odom_str = f"{YELLOW}Waiting for /odom...{RESET}"
    else:
        odom_str = (
            f"x1={BOLD}{node.x1:+6.3f}{RESET}m  "
            f"x2={BOLD}{node.x2:+6.3f}{RESET}m  "
            f"yaw={BOLD}{math.degrees(node.yaw):+6.1f}°{RESET}"
        )

    if node.r is None:
        obs_str = f"{YELLOW}Waiting for /obstacle (run spawn_obstacle)...{RESET}"
    else:
        r_s = node.r + node.r_robot + node.delta
        cbf_badge = f"{RED}[CBF INTERVENING]{RESET}" if node.cbf_active else f"{GREEN}[NOMINAL SAFE]{RESET}"
        obs_str = (
            f"center=({node.ox:+.2f}, {node.oy:+.2f})  "
            f"r={node.r:.2f}m (r_s={r_s:.2f}m)  "
            f"h(x)={BOLD}{node.h_val:+6.3f}{RESET} {cbf_badge}"
        )

    if not node.has_goal:
        state_str = f"{YELLOW}IDLE (Enter target 'x1_d x2_d' below){RESET}"
    elif node.goal_reached:
        state_str = f"{GREEN}HOLDING AT GOAL ({node.x1_d:+.2f}, {node.x2_d:+.2f}) | Err: {node.dist_goal:.3f}m{RESET}"
    else:
        state_str = f"{CYAN}NAVIGATING -> ({node.x1_d:+.2f}, {node.x2_d:+.2f}) | Dist: {BOLD}{node.dist_goal:.3f}m{RESET}"

    lines = [
        f"{BOLD}{CYAN}PAL OMNI SAFE CONTROLLER (PID + CBF-QP){RESET}   "
        f"{DIM}[t={node.t_elapsed:5.1f}s | Samples={len(node.log_t)}]{RESET}",
        sep,
        f"  {CYAN}State     {RESET}: {state_str}",
        f"  {CYAN}Robot Pose{RESET}: {odom_str}",
        f"  {MAGENTA}Obstacle  {RESET}: {obs_str}",
        f"  {GREEN}Control u {RESET}: "
        f"u_nom=({node.u_nom[0]:+5.2f}, {node.u_nom[1]:+5.2f}) m/s  |  "
        f"u*=(vx:{BOLD}{node.u_star[0]:+5.2f}{RESET}, vy:{BOLD}{node.u_star[1]:+5.2f}{RESET}) m/s",
        f"  {DIM}Tuning    : Kp={node.Kp:.2f} Ki={node.Ki:.2f} Kd={node.Kd:.2f} | alpha={node.alpha:.2f} | v_max={node.v_max:.2f}{RESET}",
        sep,
        f"  {DIM}Commands  : <x1_d x2_d> Set Goal | 's': Stop | 'p': Plot | 'c': Clear Log | 'g Kp Ki Kd' | 'a alpha' | 'q': Quit{RESET}",
        f"  {YELLOW}Status    {RESET}: {status_msg}",
        f"  {BOLD}Target x1_d x2_d > {RESET}{input_buf}",
    ]

    output = f"{CSI}H" + "\n".join(f"{CLEAR_LINE}{line}" for line in lines) + f"{CSI}J"
    sys.stdout.write(output)
    sys.stdout.flush()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    init_goal = (float(args[0]), float(args[1])) if len(args) >= 2 else None

    rclpy.init()
    node = SafeController(init_goal)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    input_buf = ""
    status_msg = (
        f"{GREEN}Navigating to ({init_goal[0]:.2f}, {init_goal[1]:.2f}){RESET}"
        if init_goal else f"{YELLOW}Ready. Type 'x1_d x2_d' (e.g. 3.0 3.0) and press Enter.{RESET}"
    )

    sys.stdout.write(f"{CSI}2J{CSI}H")
    sys.stdout.flush()

    prev_reached = node.goal_reached

    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            # Notify when goal is newly reached
            if node.has_goal and node.goal_reached and not prev_reached:
                status_msg = (
                    f"{GREEN}Goal ({node.x1_d:.2f}, {node.x2_d:.2f}) reached! "
                    f"Enter new 'x1_d x2_d', or press 'p' to plot.{RESET}"
                )
            prev_reached = node.goal_reached

            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                ch = sys.stdin.read(1)
                if ch in ('\n', '\r'):
                    cmd = input_buf.strip()
                    input_buf = ""
                    low = cmd.lower()

                    if low in ('q', 'quit', 'exit'):
                        node.stop_robot()
                        break
                    elif low in ('s', 'stop'):
                        node.stop_robot()
                        status_msg = f"{YELLOW}Robot stopped. Waiting for new goal.{RESET}"
                    elif low in ('p', 'plot'):
                        msg = node.plot_results()
                        status_msg = f"{CYAN}{msg}{RESET}"
                    elif low in ('c', 'clear'):
                        node.clear_logs()
                        status_msg = f"{CYAN}Trajectory logs cleared.{RESET}"
                    elif low.startswith('g '):
                        parts = low[2:].replace(',', ' ').split()
                        try:
                            node.Kp = float(parts[0])
                            if len(parts) > 1:
                                node.Ki = float(parts[1])
                            if len(parts) > 2:
                                node.Kd = float(parts[2])
                            status_msg = f"{GREEN}Updated PID: Kp={node.Kp:.2f}, Ki={node.Ki:.2f}, Kd={node.Kd:.2f}{RESET}"
                        except ValueError:
                            status_msg = f"{RED}Usage: g <Kp> <Ki> <Kd>{RESET}"
                    elif low.startswith('a '):
                        try:
                            node.alpha = float(low[2:].strip())
                            status_msg = f"{GREEN}Updated CBF alpha={node.alpha:.2f}{RESET}"
                        except ValueError:
                            status_msg = f"{RED}Usage: a <alpha>{RESET}"
                    elif low != "":
                        parts = low.replace(',', ' ').split()
                        try:
                            gx, gy = float(parts[0]), float(parts[1])
                            node.set_new_goal(gx, gy)
                            prev_reached = False
                            status_msg = f"{GREEN}New Goal Set -> ({gx:+.2f}, {gy:+.2f}){RESET}"
                        except (ValueError, IndexError):
                            status_msg = f"{RED}Invalid! Enter two numbers: x1_d x2_d (e.g. 3.0 2.5){RESET}"

                elif ch in ('\x7f', '\x08'):
                    input_buf = input_buf[:-1]
                elif ch == '\x03':
                    node.stop_robot()
                    break
                elif ch.isprintable():
                    input_buf += ch

            render_ui(node, input_buf, status_msg)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        sys.stdout.write(f"\n{CLEAR_LINE}{GREEN}Shutting down safe_controller.{RESET}\n")
        sys.stdout.flush()
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        if node.log_t:
            print(node.plot_results())
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()