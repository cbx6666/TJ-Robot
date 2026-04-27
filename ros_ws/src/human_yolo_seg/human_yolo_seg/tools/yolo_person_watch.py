# pyright: reportMissingImports=false
"""终端里持续提示「当前是否检出 COCO person」——给做 Gazebo 人物模型的同事用。

用法（仿真 + YOLO 已跑）：
  ros2 run human_yolo_seg yolo_person_watch

默认订阅 /human_yolo/person_count 与 person_max_conf（与 yolo_person_seg_node 发布前缀一致）。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class YoloPersonWatch(Node):
    def __init__(self) -> None:
        super().__init__("yolo_person_watch")
        self.declare_parameter("stats_prefix", "/human_yolo")
        prefix = self.get_parameter("stats_prefix").get_parameter_value().string_value.rstrip("/")
        self._count = 0
        self._conf = 0.0
        self.create_subscription(Int32, f"{prefix}/person_count", self._on_count, 10)
        self.create_subscription(Float32, f"{prefix}/person_max_conf", self._on_conf, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"监听 {prefix}/person_count 与 person_max_conf；"
            "有人且 conf 足够时下面会周期性提示（与 RViz /annotated_image 一致）"
        )

    def _on_count(self, msg: Int32) -> None:
        self._count = int(msg.data)

    def _on_conf(self, msg: Float32) -> None:
        self._conf = float(msg.data)

    def _tick(self) -> None:
        if self._count > 0:
            self.get_logger().info(
                f"[人物模型可识别] 检出 person 数量={self._count}，最高置信度={self._conf:.2f} "
                "（COCO person=0；可在 RViz 看 /human_yolo/annotated_image）"
            )
        else:
            self.get_logger().info(
                "[未检出] 当前相机画面内无达到阈值的 person（调角度/距离/光照，或略降 conf_threshold）"
            )


def main() -> None:
    rclpy.init()
    node = YoloPersonWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
