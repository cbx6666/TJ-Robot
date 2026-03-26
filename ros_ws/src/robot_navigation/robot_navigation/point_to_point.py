import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# 将数值限制在给定区间内，避免速度指令过大。
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# 将角度归一化到 [-pi, pi]，方便比较航向误差。
def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# 从平面运动下的四元数中提取 yaw 航向角。
def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class PointToPointController(Node):
    # 初始化 ROS 节点、控制参数、订阅器、发布器和控制定时器。
    def __init__(self) -> None:
        super().__init__("point_to_point_controller")

        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("use_goal_yaw", False)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("linear_speed_limit", 0.22)
        self.declare_parameter("angular_speed_limit", 1.2)
        self.declare_parameter("linear_kp", 0.8)
        self.declare_parameter("angular_kp", 2.5)
        self.declare_parameter("drive_heading_kp", 1.5)
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("heading_tolerance", 0.08)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.use_goal_yaw = bool(self.get_parameter("use_goal_yaw").value)
        self.linear_speed_limit = float(self.get_parameter("linear_speed_limit").value)
        self.angular_speed_limit = float(self.get_parameter("angular_speed_limit").value)
        self.linear_kp = float(self.get_parameter("linear_kp").value)
        self.angular_kp = float(self.get_parameter("angular_kp").value)
        self.drive_heading_kp = float(self.get_parameter("drive_heading_kp").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.current_pose: Optional[Pose2D] = None
        self.completed = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.on_timer)

        self.get_logger().info(
            "Point-to-point controller started "
            f"(goal=({self.goal_x:.2f}, {self.goal_y:.2f}), "
            f"use_goal_yaw={self.use_goal_yaw}, goal_yaw={self.goal_yaw:.2f})"
        )

    # 接收里程计数据并更新机器人当前位置与朝向。
    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=normalize_angle(yaw_from_quaternion(orientation.z, orientation.w)),
        )

    # 定时执行闭环控制：先转向目标，再前进，必要时对齐终点朝向。
    def on_timer(self) -> None:
        if self.current_pose is None or self.completed:
            return

        command = Twist()

        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        distance = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(heading_to_goal - self.current_pose.yaw)

        if distance > self.position_tolerance:
            if abs(heading_error) > 0.35:
                command.angular.z = clamp(
                    self.angular_kp * heading_error,
                    -self.angular_speed_limit,
                    self.angular_speed_limit,
                )
            else:
                command.linear.x = clamp(
                    self.linear_kp * distance,
                    0.0,
                    self.linear_speed_limit,
                )
                command.angular.z = clamp(
                    self.drive_heading_kp * heading_error,
                    -self.angular_speed_limit,
                    self.angular_speed_limit,
                )
        elif self.use_goal_yaw:
            final_heading_error = normalize_angle(self.goal_yaw - self.current_pose.yaw)
            if abs(final_heading_error) > self.heading_tolerance:
                command.angular.z = clamp(
                    self.angular_kp * final_heading_error,
                    -self.angular_speed_limit,
                    self.angular_speed_limit,
                )
            else:
                self.finish()
                return
        else:
            self.finish()
            return

        self.cmd_pub.publish(command)

    # 在到达目标后停止控制、发送零速度并关闭节点。
    def finish(self) -> None:
        if self.completed:
            return

        self.completed = True
        self.timer.cancel()
        self.publish_stop()
        self.get_logger().info("Goal reached, robot stopped.")
        rclpy.shutdown()

    # 发布零速度，确保机器人停车。
    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    # 节点销毁前再次发送停止指令，避免退出时残留运动。
    def destroy_node(self) -> bool:
        self.publish_stop()
        return super().destroy_node()


# 初始化 ROS、创建控制节点并保持运行直到结束。
def main() -> None:
    rclpy.init()
    node = PointToPointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
