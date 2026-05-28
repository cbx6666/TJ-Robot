"""独立「模拟语音」窗口（tkinter）。RViz 面板未编译时的后备；发布 /interaction/speech_text。"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_PRESETS = (
    "进行房间巡检",
    "帮我拿个水杯",
    "帮我拿个花瓶",
    "停止",
)


class SimSpeechGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_speech_gui")
        self.declare_parameter("speech_text_topic", "/interaction/speech_text")
        topic = str(self.get_parameter("speech_text_topic").value)
        self._pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"[sim_speech_gui] 发布 {topic}（tkinter 窗口）")
        self._root = tk.Tk()
        self._root.title("TJ-Robot 模拟语音")
        self._build_ui()
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.create_timer(0.05, self._tk_pump)

    def _build_ui(self) -> None:
        frm = ttk.Frame(self._root, padding=8)
        frm.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            frm,
            text="调试模式：选择或输入句子，点击发送 → LLM",
            wraplength=360,
        ).grid(row=0, column=0, columnspan=2, pady=(0, 8))
        self._var = tk.StringVar(value=_PRESETS[0])
        combo = ttk.Combobox(frm, textvariable=self._var, values=_PRESETS, width=42)
        combo.grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Label(frm, text="自定义:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self._custom = ttk.Entry(frm, width=44)
        self._custom.grid(row=3, column=0, columnspan=2, sticky="ew")
        ttk.Button(frm, text="发送 → LLM", command=self._send).grid(
            row=4, column=0, columnspan=2, pady=12
        )

    def _send(self) -> None:
        custom = self._custom.get().strip()
        text = custom if custom else self._var.get().strip()
        if not text:
            self.get_logger().warning("[sim_speech_gui] 句子为空")
            return
        self._pub.publish(String(data=text))
        self.get_logger().info(f"[sim_speech_gui] 已发送: {text!r}")

    def _tk_pump(self) -> None:
        try:
            self._root.update()
        except tk.TclError:
            pass

    def _on_close(self) -> None:
        self.get_logger().info("[sim_speech_gui] 窗口关闭")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = SimSpeechGuiNode()
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
