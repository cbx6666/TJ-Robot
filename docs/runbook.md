# 运行手册

## 环境准备

```bash
cd <repo>
bash scripts/setup_env.sh
```

YOLO 依赖需要使用和 ROS2 相同的 Python：

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -r ros_ws/src/human_yolo_seg/requirements.txt
```

## build

```bash
bash scripts/build.sh
```

等价手动命令：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 启动仿真

```bash
bash scripts/run_simulation.sh
```

默认是 `TB3_STACK_MODE=laser` 且 `TB3_ASSIST_SCAN_FILTER=0`。

## baseline 建图

```bash
bash scripts/run_baseline_mapping.sh
```

保存地图：

```bash
bash scripts/save_map.sh baseline_raw
```

输出到 `data/maps/baseline_raw`。

## semantic mapping

```bash
bash scripts/run_semantic_mapping.sh
```

保存 strip 前地图：

```bash
bash scripts/save_map.sh semantic_pre_strip
```

人物区域默认来自：

```text
~/.ros/tj_person_strip_regions.yaml
```

## strip 后处理

```bash
bash scripts/strip_map.sh \
  data/maps/semantic_pre_strip/map_xxx.yaml \
  ~/.ros/tj_person_strip_regions.yaml
```

输出到 `data/maps/semantic_post_strip`。

## 启动导航

先保证仿真、TF、`/map`、`/scan` 正常，再运行：

```bash
bash scripts/run_navigation.sh
```

若需切换 Nav2 参数 profile（如更保守避障）：

```bash
export TJ_NAV2_PROFILE_PARAMS=ros_ws/src/robot_bringup/config/nav2/profiles/conservative.yaml
bash scripts/run_navigation.sh
```

## 启动完整系统

```bash
bash scripts/run_full_system.sh
```

当前完整系统会启动仿真、SLAM、YOLO/person pipeline 和 task manager。ASR/NLU/TTS 仍是预留。

## 清理输出

```bash
bash scripts/clean_outputs.sh
```

只清理 `data/maps/*`、`data/logs`、`data/results` 下的生成文件，保留 `.gitkeep`。

## 停止仿真

```bash
bash scripts/tb3_stack.sh stop
```
