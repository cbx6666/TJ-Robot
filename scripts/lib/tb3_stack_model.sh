# shellcheck shell=bash
# 由 scripts/tb3_stack.sh source；依赖: SCRIPT_DIR、TB3_LOG_DIR、TURTLEBOT3_MODEL、ros2。

expand_robot_urdf() {
  local input_file="$1"
  local output_file="$2"
  local frag="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/urdf/waffle_depth_camera_links.urdf.xml"
  local xraw="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_xacro_raw.urdf"
  local need_depth_links=0
  if [[ "${TURTLEBOT3_MODEL}" == "waffle" || "${TURTLEBOT3_MODEL}" == "waffle_pi" ]]; then
    need_depth_links=1
  fi
  if [[ -f "${output_file}" && "${output_file}" -nt "${input_file}" ]]; then
    if [[ "${need_depth_links}" != "1" || ! -f "${frag}" ]]; then
      return 0
    fi
    if [[ "${output_file}" -nt "${frag}" ]]; then
      return 0
    fi
  fi
  if ! ros2 run xacro xacro "${input_file}" >"${xraw}" 2>"${TB3_LOG_DIR}/xacro_tb3.log"; then
    echo "ERROR: xacro 展开失败: ${input_file}" >&2
    echo "       详情见: ${TB3_LOG_DIR}/xacro_tb3.log" >&2
    return 1
  fi
  if [[ "${need_depth_links}" == "1" && -f "${frag}" ]]; then
    if ! python3 - "${xraw}" "${frag}" "${output_file}" <<'PYAPPEND'
import re
import sys
from pathlib import Path

xacro_out = Path(sys.argv[1])
frag_path = Path(sys.argv[2])
out_path = Path(sys.argv[3])
base = xacro_out.read_text(encoding="utf-8")
tok = "</robot>"
if tok not in base:
    print("ERROR: xacro URDF 缺少 </robot>", file=sys.stderr)
    raise SystemExit(1)


def _already_has_camera_depth_mount(urdf: str) -> bool:
    # Humble turtlebot3_description/waffle 常为 RealSense 树，已包含 camera_depth_*。
    # 旧版正则在部分 joint 排版下会漏检，导致与 fragment 二次拼装、link 重名。
    if re.search(r'<link\s+name\s*=\s*"camera_depth_frame"', urdf):
        return True
    if re.search(r"<link\s+name\s*=\s*'camera_depth_frame'", urdf):
        return True
    return False


frag = frag_path.read_text(encoding="utf-8").strip()
if _already_has_camera_depth_mount(base):
    print(
        "NOTE: URDF already has camera_depth_frame; skipping waffle_depth_camera_links fragment",
        file=sys.stderr,
    )
    frag = ""

if frag:
    head, _sep, _tail = base.rpartition(tok)
    merged = head.rstrip() + "\n" + frag + "\n" + tok + "\n"
else:
    merged = base

out_path.write_text(merged, encoding="utf-8")
PYAPPEND
    then
      echo "ERROR: 拼接 depth_camera_links URDF 失败" >&2
      return 1
    fi
  else
    cp -f "${xraw}" "${output_file}"
  fi
}

prepare_model_with_camera_rate() {
  local input_file="$1"
  local output_file="$2"
  local camera_rate="$3"
  local enable_camera="$4"
  local camera_always_on="$5"
  local patch_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/patch_tb3_gazebo_camera_sensors.py"
  if [[ ! -f "${patch_py}" ]]; then
    echo "WARNING: missing ${patch_py}" >&2
    return 1
  fi
  python3 "${patch_py}" \
    --input "${input_file}" \
    --output "${output_file}" \
    --rate "${camera_rate}" \
    --enable "${enable_camera}" \
    --always-on "${camera_always_on}" \
    --rgb-width "${TB3_SIM_CAMERA_WIDTH:-640}" \
    --rgb-height "${TB3_SIM_CAMERA_HEIGHT:-480}" \
    --depth-width "${TB3_SIM_DEPTH_WIDTH:-640}" \
    --depth-height "${TB3_SIM_DEPTH_HEIGHT:-480}" \
    --kinds camera
}
