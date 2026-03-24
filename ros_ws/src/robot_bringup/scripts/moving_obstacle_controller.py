#!/usr/bin/env python3
import argparse
import math
import shutil
import subprocess
import sys
import time


def compute_xy(args, t: float):
    omega = 2.0 * math.pi * args.frequency

    if args.trajectory == "circle":
        x = args.center_x + args.amplitude_x * math.cos(omega * t)
        y = args.center_y + args.amplitude_y * math.sin(omega * t)
        return x, y

    if args.trajectory == "figure8":
        x = args.center_x + args.amplitude_x * math.sin(omega * t)
        y = args.center_y + args.amplitude_y * math.sin(2.0 * omega * t)
        return x, y

    if args.trajectory == "lissajous":
        x = args.center_x + args.amplitude_x * math.sin(
            args.lissajous_ax * omega * t + args.lissajous_phase
        )
        y = args.center_y + args.amplitude_y * math.sin(args.lissajous_ay * omega * t)
        return x, y

    if args.trajectory == "patrol":
        perimeter = 2.0 * (args.amplitude_x + args.amplitude_y)
        if perimeter <= 1e-6:
            return args.center_x, args.center_y

        speed = max(args.patrol_speed, 1e-6)
        phase_dist = (speed * t) % perimeter

        x_min = args.center_x - args.amplitude_x / 2.0
        x_max = args.center_x + args.amplitude_x / 2.0
        y_min = args.center_y - args.amplitude_y / 2.0
        y_max = args.center_y + args.amplitude_y / 2.0

        edge1 = args.amplitude_x
        edge2 = edge1 + args.amplitude_y
        edge3 = edge2 + args.amplitude_x

        if phase_dist < edge1:
            return x_min + phase_dist, y_min
        if phase_dist < edge2:
            return x_max, y_min + (phase_dist - edge1)
        if phase_dist < edge3:
            return x_max - (phase_dist - edge2), y_max
        return x_min, y_max - (phase_dist - edge3)

    x = args.center_x + args.amplitude_x * math.sin(omega * t)
    y = args.center_y
    return x, y


def run_gz_mode(args):
    if shutil.which("gz") is None:
        print("ERROR: gz command not found in PATH", file=sys.stderr)
        return 1

    print(
        f"Moving obstacle [{args.entity_name}] using gz model fallback "
        f"(trajectory={args.trajectory}, rate_hz={args.rate_hz})"
    )
    start_time = time.time()
    sleep_dt = max(1.0 / args.rate_hz, 0.01)

    while True:
        t = time.time() - start_time
        x, y = compute_xy(args, t)
        cmd = [
            "gz",
            "model",
            "-m",
            args.entity_name,
            "-x",
            str(float(x)),
            "-y",
            str(float(y)),
            "-z",
            str(float(args.z)),
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ]
        subprocess.run(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
        )
        time.sleep(sleep_dt)


def run_ros_mode(args):
    import rclpy
    from rclpy.node import Node
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState

    class RosObstacleMover(Node):
        def __init__(self):
            super().__init__("tb3_moving_obstacle_controller")
            self.start_time = time.time()
            self.client = self.create_client(SetEntityState, args.service_name)
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service {args.service_name} ...")
            self.timer = self.create_timer(1.0 / args.rate_hz, self.on_timer)
            self.get_logger().info(
                f"Moving obstacle [{args.entity_name}] using {args.service_name} "
                f"(trajectory={args.trajectory}, rate_hz={args.rate_hz})"
            )

        def on_timer(self):
            t = time.time() - self.start_time
            x, y = compute_xy(args, t)

            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = args.entity_name
            request.state.pose.position.x = float(x)
            request.state.pose.position.y = float(y)
            request.state.pose.position.z = float(args.z)
            request.state.pose.orientation.w = 1.0
            request.state.twist.linear.x = 0.0
            request.state.twist.linear.y = 0.0
            request.state.twist.linear.z = 0.0
            request.state.twist.angular.x = 0.0
            request.state.twist.angular.y = 0.0
            request.state.twist.angular.z = 0.0
            request.state.reference_frame = "world"
            self.client.call_async(request)

    rclpy.init()
    node = RosObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["ros", "gz"], default="ros")
    parser.add_argument("--entity-name", required=True)
    parser.add_argument("--service-name", default="")
    parser.add_argument("--center-x", type=float, required=True)
    parser.add_argument("--center-y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--amplitude-x", type=float, required=True)
    parser.add_argument("--amplitude-y", type=float, default=0.8)
    parser.add_argument("--frequency", type=float, required=True)
    parser.add_argument("--rate-hz", type=float, required=True)
    parser.add_argument(
        "--trajectory",
        choices=["line", "circle", "figure8", "lissajous", "patrol"],
        default="line",
    )
    parser.add_argument("--lissajous-ax", type=float, default=3.0)
    parser.add_argument("--lissajous-ay", type=float, default=2.0)
    parser.add_argument("--lissajous-phase", type=float, default=1.57079632679)
    parser.add_argument("--patrol-speed", type=float, default=0.6)
    args = parser.parse_args()

    if args.mode == "gz":
        raise SystemExit(run_gz_mode(args))

    if not args.service_name:
        print("ERROR: --service-name is required in ros mode", file=sys.stderr)
        raise SystemExit(2)

    run_ros_mode(args)


if __name__ == "__main__":
    main()
