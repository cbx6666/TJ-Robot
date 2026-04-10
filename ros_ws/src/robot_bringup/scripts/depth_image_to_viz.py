#!/usr/bin/env python3
"""将仿真深度图 (32FC1 / 16UC1) 转为 mono8，供 RViz Image 显示。

RViz 对浮点深度编码支持很差，常显示为 no image；Gazebo 里看到的深度预览
是仿真器自己的渲染，不等于本节点订阅的话题一定相同。"""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class DepthImageToViz(Node):
    def __init__(self) -> None:
        super().__init__("depth_image_to_viz")
        self._last_bad_enc_log = 0.0
        self.declare_parameter("in_topic", "/tb3_depth_only/depth/image_raw")
        self.declare_parameter("out_topic", "/camera/depth/viz_mono8")
        # 与 TB3 激光/深度常用量程上限一致（Gazebo clip far 约 3.5 m）
        self.declare_parameter("max_range_m", 3.5)
        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self._max_range = float(self.get_parameter("max_range_m").value)
        qos = _sensor_qos()
        self._pub = self.create_publisher(Image, out_topic, qos)
        self.create_subscription(Image, in_topic, self._cb, qos)
        self.get_logger().info(f"{in_topic} -> {out_topic} (mono8 for RViz)")

    def _cb(self, msg: Image) -> None:
        h, w = msg.height, msg.width
        if h <= 0 or w <= 0:
            return
        enc = msg.encoding
        try:
            if enc == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
                valid = np.isfinite(arr) & (arr > 0.0)
                d = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
                d = np.clip(d, 0.0, self._max_range)
                out = (d / self._max_range * 255.0).astype(np.uint8)
                out[~valid] = 0
            elif enc == "16UC1":
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
                valid = arr > 0
                # 常见为毫米
                d = arr.astype(np.float32) * 0.001
                d = np.clip(d, 0.0, self._max_range)
                out = (d / self._max_range * 255.0).astype(np.uint8)
                out[~valid] = 0
            else:
                now = time.monotonic()
                if now - self._last_bad_enc_log > 10.0:
                    self.get_logger().warning(
                        f"跳过不支持的 depth encoding: {enc}（仅支持 32FC1 / 16UC1）",
                    )
                    self._last_bad_enc_log = now
                return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"深度解码失败: {exc}")
            return

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = h
        out_msg.width = w
        out_msg.encoding = "mono8"
        out_msg.is_bigendian = 0
        out_msg.step = w
        out_msg.data = out.tobytes()
        self._pub.publish(out_msg)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = DepthImageToViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
