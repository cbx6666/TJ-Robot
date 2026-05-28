"""室内服务机器人相机安装高度（Gazebo waffle + URDF TF 对齐）。

TB3 waffle 的 Gazebo 画面由 camera_rgb_frame 的 **link 级 pose** 决定（官方约 0.069 -0.047 0.107），
只改 camera_joint 而不改 link pose → 画面仍约 0.1 m（像脚底）。只改 link pose 为 0.89 且再抬高
camera_joint → 仿真与 URDF 双重抬高，YOLO 地图点偏远。

策略（waffle SDF）：
- camera_joint / camera_rgb_joint 保持官方位姿（0.094 + 0.013）
- link pose 的 Z = 目标光心高度 - 0.107（勿把 0.89 直接写入 link，否则 Gazebo 光心≈1.0 m）
URDF：camera_link Z = 目标光心高度 - 0.013（与上式同一 optical_z_target_m()）
YOLO：depth_range_to_optical_z 修正俯视时的射线距离（Gazebo 16UC1 时；32FC1 可关）
"""
from __future__ import annotations

import os
import re

TB3_CAMERA_RGB_Z_OFFICIAL_M = 0.094
TB3_CAMERA_RGB_Z_DEFAULT_SERVICE_M = 0.89
TB3_SDF_CAMERA_RGB_LINK_X = 0.069
TB3_SDF_CAMERA_RGB_LINK_Y = -0.047
TB3_SDF_CAMERA_RGB_LINK_Z_OFFICIAL = 0.107
TB3_URDF_RGB_JOINT_Z_OFFSET_M = 0.013


def camera_rgb_z_m() -> float:
    """目标光心高度（m，相对 base_link），Gazebo 与 URDF 共用。"""
    raw = os.environ.get("TB3_CAMERA_RGB_Z_M", "").strip()
    if not raw:
        return TB3_CAMERA_RGB_Z_DEFAULT_SERVICE_M
    try:
        v = float(raw)
        if v < TB3_CAMERA_RGB_Z_OFFICIAL_M:
            return TB3_CAMERA_RGB_Z_OFFICIAL_M
        if v > 1.5:
            return 1.5
        return v
    except ValueError:
        return TB3_CAMERA_RGB_Z_DEFAULT_SERVICE_M


def waffle_gazebo_joint_chain_z_m() -> float:
    """waffle SDF：camera_joint.z + camera_rgb_joint.z（保持官方不动）。"""
    return TB3_CAMERA_RGB_Z_OFFICIAL_M + TB3_URDF_RGB_JOINT_Z_OFFSET_M


def sdf_camera_rgb_link_pose_z_m(z_m: float | None = None) -> float:
    """Gazebo camera_rgb_frame link/inertial pose.z，使光心总高度 = camera_rgb_z_m()。"""
    return camera_rgb_z_m() if z_m is None else z_m - waffle_gazebo_joint_chain_z_m()


def urdf_camera_link_joint_z_m(z_m: float | None = None) -> float:
    """URDF base_link→camera_link（无 SDF 式 link pose 0.107 时用）。"""
    z = camera_rgb_z_m() if z_m is None else z_m
    return max(TB3_CAMERA_RGB_Z_OFFICIAL_M, z - TB3_URDF_RGB_JOINT_Z_OFFSET_M)


def _format_z(z: float) -> str:
    return f"{z:.4f}".rstrip("0").rstrip(".")


def _patch_sdf_joint_pose_z(block: str, z_str: str) -> tuple[str, int]:
    pose_rx = re.compile(
        r"(<pose>)\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*(</pose>)"
    )
    pm = pose_rx.search(block)
    if not pm:
        return block, 0
    new_block = (
        block[: pm.start()]
        + f"{pm.group(1)}{pm.group(2)} {pm.group(3)} {z_str} "
        f"{pm.group(5)} {pm.group(6)} {pm.group(7)}{pm.group(8)}"
        + block[pm.end() :]
    )
    return new_block, 1


def _sdf_joint_named(sdf_text: str, joint_name: str) -> re.Match[str] | None:
    return re.search(
        rf'(<joint\b[^>]*\bname="{re.escape(joint_name)}"[^>]*>)([\s\S]*?)(</joint>)',
        sdf_text,
        flags=re.MULTILINE,
    )


def _sdf_joint_parent_child(sdf_text: str, parent: str, child: str) -> re.Match[str] | None:
    rx = re.compile(
        rf"(<joint\b[^>]*>)([\s\S]*?)(</joint>)",
        flags=re.MULTILINE,
    )
    for m in rx.finditer(sdf_text):
        inner = m.group(2)
        if re.search(rf"<parent>\s*{re.escape(parent)}\s*</parent>", inner) and re.search(
            rf"<child>\s*{re.escape(child)}\s*</child>", inner
        ):
            return m
    return None


def _patch_sdf_link_pose_xyz(
    sdf_text: str,
    link_name: str,
    x: float,
    y: float,
    z: float,
) -> tuple[str, int]:
    """改 link/inertial pose（Gazebo 相机画面主要看这里）。"""
    link_rx = re.compile(
        rf'(<link\s+name="{re.escape(link_name)}"[^>]*>)([\s\S]*?)(</link>)',
        flags=re.MULTILINE,
    )
    m = link_rx.search(sdf_text)
    if not m:
        return sdf_text, 0
    inner = m.group(2)
    pose_str = f"{x} {y} {_format_z(z)} 0 0 0"
    count = 0
    new_inner = inner
    for pattern in (
        r"(<inertial>\s*)<pose>[^<]+</pose>",
        r"(</inertial>\s*)<pose>[^<]+</pose>",
    ):
        new_inner, n = re.subn(
            pattern,
            rf"\1<pose>{pose_str}</pose>",
            new_inner,
            count=1,
        )
        count += n
    if new_inner == inner:
        return sdf_text, 0
    out = sdf_text[: m.start()] + m.group(1) + new_inner + m.group(3) + sdf_text[m.end() :]
    return out, count


def _restore_sdf_camera_joint_official(sdf_text: str) -> tuple[str, int]:
    z_str = _format_z(TB3_CAMERA_RGB_Z_OFFICIAL_M)
    m = _sdf_joint_named(sdf_text, "camera_joint") or _sdf_joint_parent_child(
        sdf_text, "base_link", "camera_link"
    )
    if not m:
        return sdf_text, 0
    inner, n = _patch_sdf_joint_pose_z(m.group(2), z_str)
    if not n:
        return sdf_text, 0
    out = sdf_text[: m.start()] + m.group(1) + inner + m.group(3) + sdf_text[m.end() :]
    return out, n


def patch_sdf_camera_rgb_joint_z(sdf_text: str, z_m: float | None = None) -> tuple[str, int]:
    z_target = z_m if z_m is not None else camera_rgb_z_m()
    out = sdf_text
    count = 0

    if _sdf_joint_named(out, "camera_joint") or _sdf_joint_parent_child(out, "base_link", "camera_link"):
        out, n = _restore_sdf_camera_joint_official(out)
        count += n
        link_z = sdf_camera_rgb_link_pose_z_m(z_target)
        out, n2 = _patch_sdf_link_pose_xyz(
            out,
            "camera_rgb_frame",
            TB3_SDF_CAMERA_RGB_LINK_X,
            TB3_SDF_CAMERA_RGB_LINK_Y,
            link_z,
        )
        count += n2
        return out, count

    m = _sdf_joint_named(out, "camera_rgb_joint") or _sdf_joint_parent_child(
        out, "base_link", "camera_rgb_frame"
    )
    if m:
        inner, n = _patch_sdf_joint_pose_z(m.group(2), _format_z(z_target))
        if n:
            out = out[: m.start()] + m.group(1) + inner + m.group(3) + out[m.end() :]
            count += n
    return out, count


def patch_sdf_depth_joint_on_rgb_frame(sdf_text: str) -> tuple[str, int]:
    joint_rx = re.compile(
        r"<joint\s+name=\"camera_depth_joint\"[^>]*>[\s\S]*?</joint>",
        flags=re.MULTILINE,
    )
    replacement = """<joint name="camera_depth_joint" type="fixed">
  <parent>camera_rgb_frame</parent>
  <child>camera_depth_frame</child>
</joint>"""
    out, n = joint_rx.subn(replacement, sdf_text, count=1)
    return out, n


def patch_sdf_camera_mount(sdf_text: str, z_m: float | None = None) -> tuple[str, int]:
    out, n = patch_sdf_camera_rgb_joint_z(sdf_text, z_m)
    out, n2 = patch_sdf_depth_joint_on_rgb_frame(out)
    return out, n + n2


def _replace_origin_xyz_z(joint_inner: str, z_str: str) -> tuple[str, int]:
    om = re.search(r"<origin\b([^>]*)/>", joint_inner)
    if not om:
        return joint_inner, 0
    attrs = om.group(1)
    xm = re.search(r'xyz="([^"]+)"', attrs)
    if not xm:
        return joint_inner, 0
    xyz = xm.group(1).split()
    if len(xyz) < 3:
        return joint_inner, 0
    xyz[2] = z_str
    new_attrs = attrs[: xm.start(1)] + " ".join(xyz) + attrs[xm.end(1) :]
    new_origin = f"<origin{new_attrs}/>"
    new_inner = joint_inner[: om.start()] + new_origin + joint_inner[om.end() :]
    return new_inner, 1


def _patch_urdf_joint_by_parent_child(
    urdf_text: str, parent: str, child: str, z_str: str
) -> tuple[str, int]:
    count = 0
    out = urdf_text
    joint_rx = re.compile(r"(<joint\b[^>]*>)([\s\S]*?)(</joint>)", flags=re.MULTILINE)
    offset = 0
    for m in joint_rx.finditer(urdf_text):
        inner = m.group(2)
        if not re.search(rf'<parent\s+link="{re.escape(parent)}"\s*/>', inner):
            continue
        if not re.search(rf'<child\s+link="{re.escape(child)}"\s*/>', inner):
            continue
        new_inner, n = _replace_origin_xyz_z(inner, z_str)
        if not n:
            continue
        count += n
        start = m.start(2) + offset
        end = m.end(2) + offset
        out = out[:start] + new_inner + out[end:]
        offset += len(new_inner) - len(inner)
    return out, count


def patch_urdf_depth_joint_on_rgb_frame(urdf_text: str) -> tuple[str, int]:
    joint_rx = re.compile(
        r"<joint\s+name=\"camera_depth_joint\"[^>]*>[\s\S]*?</joint>",
        flags=re.MULTILINE,
    )
    replacement = """  <joint name="camera_depth_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="camera_rgb_frame"/>
    <child link="camera_depth_frame"/>
  </joint>"""
    out, n = joint_rx.subn(replacement, urdf_text, count=1)
    return out, n


def patch_urdf_camera_rgb_joint_z(urdf_text: str, z_m: float | None = None) -> tuple[str, int]:
    z_target = z_m if z_m is not None else camera_rgb_z_m()
    z_joint = _format_z(urdf_camera_link_joint_z_m(z_target))
    count = 0
    out = urdf_text
    out, n = _patch_urdf_joint_by_parent_child(out, "base_link", "camera_link", z_joint)
    count += n
    if not n:
        out, n = _patch_urdf_joint_by_parent_child(
            out, "base_link", "camera_rgb_frame", _format_z(z_target)
        )
        count += n
    out, n2 = patch_urdf_depth_joint_on_rgb_frame(out)
    count += n2
    return out, count


def patch_urdf_camera_mount(urdf_text: str, z_m: float | None = None) -> tuple[str, int]:
    """抬高 RGB + 将 depth 关节对齐到 camera_rgb_frame（与 Gazebo assist 深度插件一致）。

    waffle 官方 URDF 常带 RealSense 的 camera_depth 外参；若不覆盖，depth_image_proc
    RegisterNode 会按错误基线重投影，注册深度整幅为 NaN。
    """
    out, n = patch_urdf_camera_rgb_joint_z(urdf_text, z_m)
    out, n2 = patch_urdf_depth_joint_on_rgb_frame(out)
    return out, n + n2
