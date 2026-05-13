#!/usr/bin/env python3
"""Nav2 waypoint patrol entrypoint for the indoor map."""

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


# 仿真脚本默认把 TurtleBot3 生成在地图坐标 (0, 0, 0) 附近。
# 如果真实底盘或自定义 world 的初始位姿不同，请先在 RViz 使用 2D Pose Estimate，
# 或把这里改成实测初始位姿，确保 AMCL 能建立 map -> odom。
INITIAL_POSE = Waypoint("初始位姿", 0.0, 0.0, 0.0)


# 这些点按 robot_bringup/maps/map.yaml 的坐标系人工预置，目标是让机器人绕主要室内区域走一遍。
# 后续如果地图更新，优先在 RViz 里读取可通行区域坐标并同步调整这里。
WAYPOINTS = [
    Waypoint("门口区域", -7.5, 0.4, 0.0),
    Waypoint("客厅区域", -4.5, -3.3, -1.57),
    Waypoint("中部走廊区域", -0.5, 0.5, 0.0),
    Waypoint("北侧房间区域", 0.5, 3.4, 1.57),
    Waypoint("右侧公共区域", 5.5, 2.2, 0.0),
    Waypoint("右侧房间区域", 6.5, -2.4, -1.57),
    Waypoint("末端房间区域", 4.5, -4.4, 3.14),
    Waypoint("回到走廊区域", 0.0, 0.4, 3.14),
]


def make_pose(navigator: BasicNavigator, waypoint: Waypoint) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0

    # 仅绕 Z 轴旋转：yaw -> quaternion，避免额外依赖 tf_transformations。
    half_yaw = waypoint.yaw * 0.5
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


def describe_result(result: TaskResult) -> str:
    if result == TaskResult.SUCCEEDED:
        return "SUCCEEDED"
    if result == TaskResult.CANCELED:
        return "CANCELED"
    if result == TaskResult.FAILED:
        return "FAILED"
    return f"UNKNOWN({result})"


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    navigator.get_logger().info("发布 AMCL 初始位姿，等待 Nav2 lifecycle 节点进入 active...")
    navigator.setInitialPose(make_pose(navigator, INITIAL_POSE))
    navigator.waitUntilNav2Active(localizer="amcl")
    navigator.get_logger().info("Nav2 已就绪，开始室内自动巡航。")

    try:
        for index, waypoint in enumerate(WAYPOINTS, start=1):
            navigator.get_logger().info(
                f"前往第 {index}/{len(WAYPOINTS)} 个 waypoint：{waypoint.name} "
                f"(x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f})"
            )

            navigator.goToPose(make_pose(navigator, waypoint))
            last_feedback_log_time = 0.0

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                now = time.monotonic()
                if feedback is not None and now - last_feedback_log_time > 5.0:
                    last_feedback_log_time = now
                    navigator.get_logger().info(
                        f"{waypoint.name} 剩余距离约 {feedback.distance_remaining:.2f} m"
                    )

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(f"已到达：{waypoint.name}")
            else:
                # 不直接退出，便于动态障碍短时干扰后继续尝试后续区域。
                navigator.get_logger().warn(
                    f"{waypoint.name} 导航结果：{describe_result(result)}，继续尝试下一个点。"
                )

        navigator.get_logger().info("Patrol finished")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
