# pyright: reportMissingImports=false
"""终端周期性提示当前 YOLO 检测框数量（经类别过滤）。

用法（仿真 + YOLO 已跑）：
  ros2 run human_yolo_seg yolo_object_watch
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class YoloObjectWatch(Node):
    def __init__(self) -> None:
        super().__init__("yolo_object_watch")
        self.declare_parameter("stats_prefix", "/yolo_objects")
        prefix = self.get_parameter("stats_prefix").get_parameter_value().string_value.rstrip("/")
        self._prefix = prefix
        self._count = 0
        self._conf = 0.0
        self.create_subscription(Int32, f"{prefix}/detection_count", self._on_count, 10)
        self.create_subscription(Float32, f"{prefix}/detection_max_conf", self._on_conf, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"监听 {prefix}/detection_count 与 detection_max_conf（与 yolo_object_seg 节点一致）"
        )

    def _on_count(self, msg: Int32) -> None:
        self._count = int(msg.data)

    def _on_conf(self, msg: Float32) -> None:
        self._conf = float(msg.data)

    def _tick(self) -> None:
        if self._count > 0:
            self.get_logger().info(
                f"[检测到目标] 过滤后框数量={self._count}，最高置信度={self._conf:.2f} "
                f"（RViz: {self._prefix}/annotated_image）"
            )
        else:
            self.get_logger().info(
                "[未检出] 当前画面内无达到阈值的所选类别（调角度/光照或略降 conf_threshold）"
            )


def main() -> None:
    rclpy.init()
    node = YoloObjectWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
