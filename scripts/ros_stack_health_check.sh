#!/usr/bin/env bash
# ROS 2 栈体检：节点重复（图 vs 真实进程）、关键话题发布者数量、TF、可选扩展。
# 用法：仿真/Nav2 跑着时在「已 source 与工作区同一 ROS_DOMAIN_ID」的终端执行：
#   bash scripts/ros_stack_health_check.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available

echo "========== 环境 =========="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>=0}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
echo

echo "========== 节点：图里同名是否「多行」（多为发现/显示重复，需对照进程）=========="
tmp_nodes="$(mktemp)"
if ! ros2 node list 2>"${tmp_nodes}.err" >"${tmp_nodes}"; then
  echo "ros2 node list 失败（是否在跑 roscore/是否有权限？）"
  cat "${tmp_nodes}.err" 2>/dev/null || true
  exit 1
fi
if grep -q "share an exact name" "${tmp_nodes}.err" 2>/dev/null; then
  echo "(ros2 CLI 警告: graph 中存在同名节点或发现异常 — 往下对照进程计数)"
fi
TOTAL_LINES="$(wc -l <"${tmp_nodes}" | tr -d ' ')"
UNIQ_NAMES="$(sort -u "${tmp_nodes}" | wc -l | tr -d ' ')"
echo "node list 总行数: ${TOTAL_LINES}"
echo "去重后名字数:   ${UNIQ_NAMES}"
echo
echo "同名出现 >1 次（图上重复行，按次数排序前 40）:"
sort "${tmp_nodes}" | uniq -c | sort -rn | awk '$1>1 {print}' | head -40 || true
CNT_DUP_SORT="$(sort "${tmp_nodes}" | uniq -c | awk '$1>1 {c++} END{print c+0}')"
echo "至少有重复显示的名字种类数: ${CNT_DUP_SORT}"
echo

echo "========== 进程：关键节点是否真的多开（应多 ≤1，listener 可无 pgrep）=========="
for needle in \
  "ros2.*launch.*navigation" \
  "_ros2_daemon" \
  "gzserver" \
  "gzclient" \
  "rviz2" \
  "yolo_object_seg" \
  "robot_state_publisher" \
  "publish_robot_description" \
  "amcl" \
  "map_server" \
  "planner_server" \
  "controller_server"; do
  n="$(pgrep -af "${needle}" 2>/dev/null | wc -l | tr -d ' ')"
  printf "%-42s pgrep -af 命中行数: %s\n" "${needle}" "${n}"
done
echo "(说明: navigation launch 常为 1 条父进程树；若 gzserver/amcl/map_server 等为 2+ 才是真重复)"

echo
echo "========== 匿名 TF listener 节点个数（正常现象，每个 C++ TransformListener 一个）=========="
grep -c '^/transform_listener_impl_' "${tmp_nodes}" 2>/dev/null | awk '{print "transform_listener_impl_* 节点数(图中):",$1}' || true

echo
echo "========== 关键话题：发布者 / 订阅者数量 =========="
for t in \
  /tf \
  /tf_static \
  /clock \
  /map \
  /scan \
  /odom \
  /camera/image_raw \
  /camera/camera_info \
  /yolo_objects/annotated_image \
  /yolo_objects/target_objects_marker \
  /cmd_vel_nav; do
  echo "--- ${t} ---"
  ros2 topic info "${t}" 2>&1 || echo "(topic 不存在或未连接)"
done

echo
echo "========== （可选）多发布者扫描：仅列举 Publisher count>=2 =========="
WARN_TOPICS="$(mktemp)"
while IFS= read -r topic; do
  [[ -z "${topic}" ]] && continue
  info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
  pc="$(echo "${info}" | sed -n 's/^Publisher count: //p')"
  [[ -z "${pc}" ]] && continue
  if [[ "${pc}" =~ ^[0-9]+$ ]] && [[ "${pc}" -ge 2 ]]; then
    echo "${topic}" >>"${WARN_TOPICS}"
  fi
done < <(ros2 topic list 2>/dev/null)
if [[ -s "${WARN_TOPICS}" ]]; then
  echo "下列话题 Publisher>=2（可能冗余或正常现象如 /tf 多源）:"
  while IFS= read -r topic; do
    echo "  ${topic}"
    ros2 topic info "${topic}" 2>&1 | sed -n '1,25p'
  done <"${WARN_TOPICS}"
else
  echo "未发现 Publisher count>=2（或 topic 过少）。"
fi
rm -f "${WARN_TOPICS}"

echo
echo "========== 最近 Nav2 / YOLO 日志末尾（如需全量请看 data/logs/simulation）=========="
LOG="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/simulation}"
tail -25 "${LOG}/nav2.launch.log" 2>/dev/null || echo "(无 ${LOG}/nav2.launch.log)"
echo
tail -20 "${LOG}/yolo_object_seg.log" 2>/dev/null || echo "(无 ${LOG}/yolo_object_seg.log)"

rm -f "${tmp_nodes}" "${tmp_nodes}.err" 2>/dev/null || true
echo
echo "========== 完成 =========="
echo "解读: 图上 /amcl 多行但 pgrep amcl≈1 多为 DDS/graph 列表重复;"
echo "      transform_listener_impl_* 多来自 Nav2+RViz+各 costmap/YOLO，多数正常。"
