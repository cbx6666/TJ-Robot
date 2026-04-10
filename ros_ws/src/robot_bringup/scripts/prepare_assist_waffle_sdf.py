#!/usr/bin/env python3
"""在官方 turtlebot3_waffle(.pi) 的 model.sdf 中插入 camera_depth_frame + 深度相机插件。

assist 模式不再用「合并 URDF + spawn_entity -file urdf」生成整机：URDF 里 package:// 网格在 Gazebo Classic
中常解析失败（看不见车体），且易与官方插件叠加导致异常。本脚本复制系统 model.sdf（model:// 网格），
仅在已有 camera_rgb_frame 上增加与 waffle_depth_gazebo_fragment 等价的深度传感器，话题仍为
  /tb3_depth_only/depth/image_raw 等。
"""
from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# 与 TurtleBot3 LDS（/scan）量程上限一致，见 waffle_sim_plugins_gazebo_fragment.xml
TB3_LASER_RANGE_MAX_M = 3.5


def _parse_fragment() -> tuple[ET.Element, ET.Element]:
    link_xml = f"""
<link name="camera_depth_frame">
  <sensor name="tb3_depth_sensor" type="depth">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>false</visualize>
    <camera name="tb3_depth_cam">
      <horizontal_fov>1.085595</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>{TB3_LASER_RANGE_MAX_M}</far>
      </clip>
    </camera>
    <plugin name="tb3_depth_only_plugin" filename="libgazebo_ros_camera.so">
      <ros></ros>
      <camera_name>tb3_depth_only</camera_name>
      <frame_name>camera_depth_optical_frame</frame_name>
    </plugin>
  </sensor>
</link>
""".strip()
    joint_xml = """
<joint name="camera_depth_joint" type="fixed">
  <parent>camera_rgb_frame</parent>
  <child>camera_depth_frame</child>
</joint>
""".strip()
    return ET.fromstring(link_xml), ET.fromstring(joint_xml)


def main() -> int:
    p = argparse.ArgumentParser(description="为 assist 模式生成带深度传感器的 waffle SDF")
    p.add_argument("src_sdf", type=Path, help="系统 turtlebot3_waffle 或 waffle_pi 的 model.sdf")
    p.add_argument("dst_sdf", type=Path, help="输出路径（如 LOG_DIR 下）")
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
        if link.get("name") == "camera_depth_frame":
            print("NOTE: 已有 camera_depth_frame，直接复制源 SDF", file=sys.stderr)
            args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.src_sdf, args.dst_sdf)
            return 0

    rgb_idx = None
    for i, child in enumerate(model):
        if child.tag == "link" and child.get("name") == "camera_rgb_frame":
            rgb_idx = i
            break
    if rgb_idx is None:
        print("ERROR: SDF 中未找到 link camera_rgb_frame，无法挂深度（仅支持官方 waffle/waffle_pi SDF）", file=sys.stderr)
        return 1

    joint_idx = None
    for i, child in enumerate(model):
        if child.tag == "joint" and child.get("name") == "camera_rgb_joint":
            joint_idx = i
            break

    depth_link, depth_joint = _parse_fragment()
    model.insert(rgb_idx + 1, depth_link)
    if joint_idx is not None:
        # 在 camera_rgb_frame 后插入了 1 个 link，原 camera_rgb_joint 下标 +1；深度关节紧接其后 → insert(joint_idx+2)
        model.insert(joint_idx + 2, depth_joint)
    else:
        model.append(depth_joint)

    args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
    # spawn_entity（lxml）对「带 encoding 声明的 str」会报错；不写 XML 声明即可，Gazebo 不需要
    tree.write(args.dst_sdf, encoding="utf-8", xml_declaration=False)
    print(f"Wrote {args.dst_sdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
