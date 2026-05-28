#!/usr/bin/env python3
"""在官方 turtlebot3_waffle(.pi) 的 model.sdf 上配置 RGB-D。

两种模式（``TB3_SIM_UNIFIED_RGBD`` / ``--unified-rgbd``）：

1. **一体 RGB-D（推荐）**：把 ``camera_rgb_frame`` 上官方 RGB 传感器改为 ``type="depth"``，
   ``libgazebo_ros_camera`` 发布 ``/camera/image_raw``、``/camera/camera_info`` 与 ``/camera/depth/*``（同光心、同像素）。
2. **双相机（旧）**：保留官方 RGB，另插 ``camera_depth_frame`` + ``/tb3_depth_only/*``。
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TB3_DEPTH_DEFAULT_CLIP_FAR_M = 12.0
TB3_UNIFIED_CAMERA_NAME = "camera"
TB3_UNIFIED_FRAME_NAME = "camera_rgb_optical_frame"


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        v = int(raw)
        return v if v > 0 else default
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


def _depth_update_rate_hz() -> float:
    raw = os.environ.get("TB3_CAMERA_UPDATE_RATE", "8").strip()
    try:
        v = float(raw)
        return v if v > 0.0 else 8.0
    except ValueError:
        return 8.0


def _depth_far_clip_m() -> float:
    raw = os.environ.get("TB3_DEPTH_CLIP_FAR_M", "").strip()
    if not raw:
        return float(TB3_DEPTH_DEFAULT_CLIP_FAR_M)
    try:
        v = float(raw)
        return v if v > 0.11 else float(TB3_DEPTH_DEFAULT_CLIP_FAR_M)
    except ValueError:
        return float(TB3_DEPTH_DEFAULT_CLIP_FAR_M)


def _parse_fragment() -> tuple[ET.Element, ET.Element]:
    far_m = _depth_far_clip_m()
    rate_hz = _depth_update_rate_hz()
    dw = _env_int("TB3_SIM_DEPTH_WIDTH", 640)
    dh = _env_int("TB3_SIM_DEPTH_HEIGHT", 480)
    link_xml = f"""
<link name="camera_depth_frame">
  <sensor name="tb3_depth_sensor" type="depth">
    <always_on>true</always_on>
    <update_rate>{rate_hz:g}</update_rate>
    <visualize>false</visualize>
    <camera name="tb3_depth_cam">
      <horizontal_fov>1.085595</horizontal_fov>
      <image>
        <width>{dw}</width>
        <height>{dh}</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>{far_m}</far>
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


def _remove_tb3_depth_mount(model: ET.Element) -> int:
    removed = 0
    for link in list(model.findall("link")):
        if link.get("name") == "camera_depth_frame":
            model.remove(link)
            removed += 1
    for joint in list(model.findall("joint")):
        if joint.get("name") == "camera_depth_joint":
            model.remove(joint)
            removed += 1
    return removed


def _find_child(parent: ET.Element, tag: str, name: str | None = None) -> ET.Element | None:
    for child in parent:
        if child.tag != tag:
            continue
        if name is None or child.get("name") == name:
            return child
    return None


def _set_or_create(parent: ET.Element, tag: str, text: str) -> ET.Element:
    el = parent.find(tag)
    if el is None:
        el = ET.SubElement(parent, tag)
    el.text = text
    return el


def _convert_rgb_sensor_to_unified_rgbd(
    link: ET.Element,
    *,
    far_m: float,
    rate_hz: float,
    width: int,
    height: int,
) -> bool:
    """将 camera_rgb_frame 上 type=camera 改为 type=depth（一体 RGB-D）。"""
    sensor = None
    for cand in link.findall("sensor"):
        stype = (cand.get("type") or "").strip()
        if stype in ("camera", "depth"):
            sensor = cand
            break
    if sensor is None:
        return False

    sensor.set("type", "depth")
    sensor.set("name", sensor.get("name") or "camera")

    for tag in ("noise", "visualize"):
        for node in list(sensor.findall(tag)):
            sensor.remove(node)

    _set_or_create(sensor, "always_on", "true")
    _set_or_create(sensor, "update_rate", f"{rate_hz:g}")
    _set_or_create(sensor, "visualize", "false")

    cam = _find_child(sensor, "camera")
    if cam is None:
        cam = ET.SubElement(sensor, "camera", {"name": "camera_rgbd"})
    hfov = cam.find("horizontal_fov")
    if hfov is None or not (hfov.text or "").strip():
        _set_or_create(cam, "horizontal_fov", "1.02974")

    img = cam.find("image")
    if img is None:
        img = ET.SubElement(cam, "image")
    _set_or_create(img, "width", str(width))
    _set_or_create(img, "height", str(height))
    _set_or_create(img, "format", "R8G8B8")

    clip = cam.find("clip")
    if clip is None:
        clip = ET.SubElement(cam, "clip")
    _set_or_create(clip, "near", "0.1")
    _set_or_create(clip, "far", f"{far_m:g}")

    plugin = None
    for cand in sensor.findall("plugin"):
        fn = cand.get("filename") or ""
        if "libgazebo_ros_camera" in fn:
            plugin = cand
            break
    if plugin is None:
        plugin = ET.SubElement(
            sensor,
            "plugin",
            {"name": "camera_rgbd_plugin", "filename": "libgazebo_ros_camera.so"},
        )
    ros_el = plugin.find("ros")
    if ros_el is None:
        ros_el = ET.SubElement(plugin, "ros")
    for child in list(ros_el):
        ros_el.remove(child)

    _set_or_create(plugin, "camera_name", TB3_UNIFIED_CAMERA_NAME)
    _set_or_create(plugin, "frame_name", TB3_UNIFIED_FRAME_NAME)
    return True


def _apply_unified_rgbd(model: ET.Element) -> bool:
    rgb_link = None
    for link in model.findall("link"):
        if link.get("name") == "camera_rgb_frame":
            rgb_link = link
            break
    if rgb_link is None:
        print("ERROR: SDF 中未找到 link camera_rgb_frame", file=sys.stderr)
        return False

    removed = _remove_tb3_depth_mount(model)
    if removed:
        print(f"NOTE: 已移除旧的双相机挂载（links/joints removed={removed}）", file=sys.stderr)

    ok = _convert_rgb_sensor_to_unified_rgbd(
        rgb_link,
        far_m=_depth_far_clip_m(),
        rate_hz=_depth_update_rate_hz(),
        width=_env_int("TB3_SIM_DEPTH_WIDTH", 640),
        height=_env_int("TB3_SIM_DEPTH_HEIGHT", 480),
    )
    if not ok:
        print("ERROR: camera_rgb_frame 上未找到可转换的 camera/depth 传感器", file=sys.stderr)
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description="为 assist 模式配置 waffle RGB 或一体 RGB-D")
    p.add_argument("src_sdf", type=Path, help="系统 turtlebot3_waffle 或 waffle_pi 的 model.sdf")
    p.add_argument("dst_sdf", type=Path, help="输出路径（如 LOG_DIR 下）")
    p.add_argument(
        "--unified-rgbd",
        action="store_true",
        help="一体 RGB-D：/camera/image_raw + /camera/depth/*（同传感器）",
    )
    args = p.parse_args()
    unified = args.unified_rgbd or _env_bool("TB3_SIM_UNIFIED_RGBD", False)

    if not args.src_sdf.is_file():
        print(f"ERROR: 未找到 {args.src_sdf}", file=sys.stderr)
        return 1

    tree = ET.parse(args.src_sdf)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        print("ERROR: SDF 根下无 <model>", file=sys.stderr)
        return 1

    if unified:
        if not _apply_unified_rgbd(model):
            return 1
        args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
        tree.write(args.dst_sdf, encoding="utf-8", xml_declaration=False)
        print(
            f"Wrote unified RGB-D {args.dst_sdf} "
            f"(topics /{TB3_UNIFIED_CAMERA_NAME}/rgb/*, /{TB3_UNIFIED_CAMERA_NAME}/depth/*)"
        )
        return 0

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
        print("ERROR: SDF 中未找到 link camera_rgb_frame，无法挂深度", file=sys.stderr)
        return 1

    joint_idx = None
    for i, child in enumerate(model):
        if child.tag == "joint" and child.get("name") == "camera_rgb_joint":
            joint_idx = i
            break

    depth_link, depth_joint = _parse_fragment()
    model.insert(rgb_idx + 1, depth_link)
    if joint_idx is not None:
        model.insert(joint_idx + 2, depth_joint)
    else:
        model.append(depth_joint)

    args.dst_sdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.dst_sdf, encoding="utf-8", xml_declaration=False)
    print(f"Wrote dual-camera depth mount {args.dst_sdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
