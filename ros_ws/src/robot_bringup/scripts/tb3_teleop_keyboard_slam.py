#!/usr/bin/env python3
# 基于 turtlebot3_teleop/script/teleop_keyboard.py（BSD），默认线/角速度比例更适合建图。
# 官方 Waffle：角速度上限 1.82 rad/s、步进 0.1，线速度步进仅 0.01，易“转太快、走太慢”。
#
# 可选环境变量（浮点数）：
#   TB3_TELEOP_MAX_LIN_VEL   线速度上限 |m/s|（默认：Burger 0.22 / Waffle&Pi 0.26）
#   TB3_TELEOP_MAX_ANG_VEL   角速度上限 |rad/s|（默认 0.65，低于官方 1.82）
#   TB3_TELEOP_LIN_STEP      每次 w/x 调整量（默认 0.025）
#   TB3_TELEOP_ANG_STEP      每次 a/d 调整量（默认 0.05）

import os
import select
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

SLAM_DEFAULT_MAX_ANG_VEL = 0.65
SLAM_DEFAULT_LIN_STEP = 0.025
SLAM_DEFAULT_ANG_STEP = 0.05


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return float(raw)


TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

if TURTLEBOT3_MODEL == "burger":
    MAX_LIN_VEL = _float_env("TB3_TELEOP_MAX_LIN_VEL", BURGER_MAX_LIN_VEL)
else:
    MAX_LIN_VEL = _float_env("TB3_TELEOP_MAX_LIN_VEL", WAFFLE_MAX_LIN_VEL)

MAX_ANG_VEL = _float_env("TB3_TELEOP_MAX_ANG_VEL", SLAM_DEFAULT_MAX_ANG_VEL)
if TURTLEBOT3_MODEL == "burger":
    MAX_ANG_VEL = min(MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
else:
    MAX_ANG_VEL = min(MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

LIN_VEL_STEP_SIZE = _float_env("TB3_TELEOP_LIN_STEP", SLAM_DEFAULT_LIN_STEP)
ANG_VEL_STEP_SIZE = _float_env("TB3_TELEOP_ANG_STEP", SLAM_DEFAULT_ANG_STEP)

msg = f"""
Control TurtleBot3（建图友好默认线/角比例，见 tb3_teleop_keyboard_slam.py 顶部说明）
---------------------------
        w
   a    s    d
        x

w/x : 线速度 ±{LIN_VEL_STEP_SIZE:g} m/s（上限 ±{MAX_LIN_VEL:g}）
a/d : 角速度 ±{ANG_VEL_STEP_SIZE:g} rad/s（上限 ±{MAX_ANG_VEL:g}）

space / s : 停止
CTRL-C : 退出
"""


def get_key(settings):
    if os.name == "nt":
        return msvcrt.getch().decode("utf-8")
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print(
        "currently:\tlinear velocity {}\t angular velocity {} ".format(
            target_linear_velocity,
            target_angular_velocity,
        )
    )


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        return min(input_vel, output_vel + slop)
    if input_vel < output_vel:
        return max(input_vel, output_vel - slop)
    return input_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        return low_bound
    if input_vel > high_bound:
        return high_bound
    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node("tb3_teleop_keyboard_slam")
    pub = node.create_publisher(Twist, "cmd_vel", qos)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    status = 0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == "w":
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity + LIN_VEL_STEP_SIZE
                )
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == "x":
                target_linear_velocity = check_linear_limit_velocity(
                    target_linear_velocity - LIN_VEL_STEP_SIZE
                )
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == "a":
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity + ANG_VEL_STEP_SIZE
                )
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == "d":
                target_angular_velocity = check_angular_limit_velocity(
                    target_angular_velocity - ANG_VEL_STEP_SIZE
                )
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key in (" ", "s"):
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == "\x03":
                break
            else:
                pass

            if status == 20:
                print(msg)
                status = 0

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                LIN_VEL_STEP_SIZE / 2.0,
            )
            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                ANG_VEL_STEP_SIZE / 2.0,
            )

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.angular.z = control_angular_velocity
            pub.publish(twist)

    finally:
        twist = Twist()
        pub.publish(twist)
        if os.name != "nt" and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
