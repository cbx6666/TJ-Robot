#!/usr/bin/env python3
"""在官方 turtlebot3_burger 的 model.sdf 上增加与 waffle 等价的 RGB 相机（/camera/image_raw 等）。

用于 burger + YOLO + 激光建图，车体仍为 burger，仅多一个 camera_rgb_frame（与 URDF fragment 位姿一致）。
"""
from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _rgb_link_and_joint() -> tuple[ET.Element, ET.Element]:
    link_xml = """
<link name="camera_rgb_frame">
  <collision name="collision">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>0.01 0.01 0.01</size>
      </box>
    </geometry>
  </collision>
  <visual name="visual">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>0.01 0.01 0.01</size>
      </box>
    </geometry>
  </visual>
  <inertial>
    <pose>0 0 0 0 0 0</pose>
    <mass>0.01</mass>
    <inertia>
      <ixx>1e-6</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>1e-6</iyy>
      <iyz>0</iyz>
      <izz>1e-6</izz>
    </inertia>
  </inertial>
  <sensor name="camera_rgb_sensor" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="camera_rgb">
      <horizontal_fov>1.085595</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>12.0</far>
      </clip>
    </camera>
    <plugin name="camera_rgb_controller" filename="libgazebo_ros_camera.so">
      <ros></ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_rgb_optical_frame</frame_name>
    </plugin>
  </sensor>
</link>
""".strip()
    joint_xml = """
<joint name="camera_rgb_joint" type="fixed">
  <pose>0.064 -0.065 0.094 0 0 0</pose>
  <parent>base_link</parent>
  <child>camera_rgb_frame</child>
</joint>
""".strip()
    return ET.fromstring(link_xml), ET.fromstring(joint_xml)


def main() -> int:
    p = argparse.ArgumentParser(description="为 burger 生成带 RGB 的 model.sdf")
    p.add_argument("src_sdf", type=Path, help="系统 turtlebot3_burger/model.sdf")
    p.add_argument("dst_sdf", type=Path, help="输出路径")
    args = p.parse_args()

    if not args.src_sdf.is_file():
        print(f"ERROR: 未找到 {args.src_sdf}", file=sys.stderr)
        return 1

    tree = ET.parse(args.src_sdf)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        print("ERROR: SDF 根下无 <model>", file=sys.stderr)
        return 1

    for link in model.findall("link"):
        if link.get("name") == "camera_rgb_frame":
            print("NOTE: 已有 camera_rgb_frame，直接复制源 SDF", file=sys.stderr)
            args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.src_sdf, args.dst_sdf)
            return 0

    base_idx = None
    for i, child in enumerate(model):
        if child.tag == "link" and child.get("name") == "base_link":
            base_idx = i
            break
    if base_idx is None:
        print("ERROR: SDF 中未找到 link base_link", file=sys.stderr)
        return 1

    rgb_link, rgb_joint = _rgb_link_and_joint()
    model.insert(base_idx + 1, rgb_link)
    model.append(rgb_joint)

    args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.dst_sdf, encoding="utf-8", xml_declaration=False)
    print(f"Wrote {args.dst_sdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
