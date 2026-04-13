# TJ-Robot — 室内服务机器人仿真

基于 **ROS 2 Humble**、**Gazebo**、**TurtleBot3** 的室内仿真：激光建图、YOLO 人体相关处理、覆盖巡航等。

## 默认怎么跑

- **`scripts/tb3_stack.sh` 未设环境变量时**：默认 **`TB3_STACK_MODE=laser`**、**`TB3_ASSIST_SCAN_FILTER=1`**。在已编译 `human_yolo_seg` 且模型为 **burger** 时，仍会为 YOLO **注入仿真 RGB** 并尝试启动人物链。若要做**对照实验「纯激光、完全无 YOLO」**，必须显式 **`TB3_ASSIST_SCAN_FILTER=0`**（见下表）。
- **`TB3_STACK_MODE=assist`** 且未设置 `TURTLEBOT3_MODEL` 时，脚本将 **`TURTLEBOT3_MODEL` 设为 burger**（RGB + 激光，无仿真深度）。
- 需要深度与 `rgbd_to_scan` 时显式设置 **`TURTLEBOT3_MODEL=waffle`**（或 `waffle_pi`），并可配合 **`TB3_ASSIST_RGBD_BRIDGE=1`**。

主入口脚本：**`scripts/tb3_stack.sh`**（Gazebo、机器人、SLAM、RViz、可选 YOLO 人物链）。

### 人物与激光（`TB3_ASSIST_SCAN_FILTER=1` 且已编译 `human_yolo_seg`）

| `TB3_PERSON_SLAM_MODE` | SLAM 订阅 | 说明 |
|------------------------|-----------|------|
| **`mark_then_strip`**（默认） | **`/scan`** | **YOLO + person_strip_recorder + 方位角 Marker**；`map` 上人向累积点云 **`/human_yolo/person_laser_map_cloud`**；区域文件 **`~/.ros/tj_person_strip_regions.yaml`**。建图保存后可用 **`ros2 run human_yolo_seg strip_saved_map_person_free <地图.yaml> <regions.yaml>`** 将人物区域刷为空闲。整帧激光投 `map` 上色（`scan_map_colored_cloud`）默认**关闭**，需要时设 **`TB3_ENABLE_SCAN_MAP_COLORED=1`** 或 launch **`enable_scan_map_colored:=true`**。 |
| **`filtered`** | **`/scan_filtered`** | **`scan_person_filter`** 滤人后再建图；无 strip 累积点云；要看 `map` 上人向整帧上色请开 **`TB3_ENABLE_SCAN_MAP_COLORED=1`**。 |

**人物方位角**（`/human_yolo/person_azimuth_ranges`）：`tb3_stack` 会向 launch 传入 **`TB3_PERSON_AZIMUTH_MODE`**，默认 **`linear_fov`**（图像比例 × 水平视场，与激光角约定一致）；精细几何法可设 **`tf_geometry`**。详见 **`human_yolo_seg`** launch 参数。

巡航节点 **`coverage_patrol`** 仍用 **原始 `/scan`** 做近距离避障（不把人在近距离里滤掉）。

## 功能概览

- TurtleBot3 仿真、动态障碍物
- **slam_toolbox** 建图
- **human_yolo_seg**：YOLO-Seg、人物方位角、strip / 滤扫；**可选**整帧 map 彩色点云（默认关）
- **robot_navigation**：`coverage_patrol`、`point_to_point`；另有 Nav2 版覆盖（见包内 README）

## 仓库结构

```text
docs/                    代码与功能总索引（新人按图索骥）
scripts/                 环境安装、tb3_stack 一键栈
任务单/                  阶段任务、实验报告与原理说明
saved_maps/              可自行存放导出的地图（大文件默认 .gitignore）
ros_ws/src/
  robot_bringup/         launch、world、地图、配置（见包内 README）
  robot_navigation/      运动与覆盖巡航（节点说明见 robot_navigation/README.md）
  human_yolo_seg/        YOLO 与人物–激光链；`models/*.pt` 权重需自备、不入库（见包内 README）
  robot_interfaces/      预留
  robot_tasks/           预留
```

**功能 → 源码路径** 总表：**[docs/代码与功能索引.md](docs/代码与功能索引.md)**

## 快速开始

将 `<REPO>` 换成你的仓库根目录。

```bash
cd <REPO>
bash scripts/setup_env.sh    # 首次或缺依赖时

cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
cd ..
```

**YOLO 权重（仓库不包含）**：若走人物链，需自行准备 Ultralytics 分割权重（如 `yolo26n-seg.pt`），放到 `ros_ws/src/human_yolo_seg/models/`，或通过节点参数 `model_path` 指向本机绝对路径；默认文件名见 [human_yolo_seg/README.md](ros_ws/src/human_yolo_seg/README.md)。

**YOLO 的 Python 依赖（`setup_env.sh` 不会装）**：`human_yolo_seg` 需要 **与当前 `ros2` 相同的 `python3`** 安装依赖，否则 YOLO 节点起不来、RViz 里永远没有 `/human_yolo/annotated_image`。在已 `source /opt/ros/humble/setup.bash` 的终端执行：

```bash
python3 -m pip install -r ros_ws/src/human_yolo_seg/requirements.txt
```

（NumPy 版本与 `cv_bridge` 兼容性见该文件内注释；勿用另一个 Python 环境装完就算。）

**RViz 里看不到 YOLO 画面时**：默认配置 `robot_bringup/config/test1.rviz` 已启用 **Image → `/human_yolo/annotated_image`**。若你用的是自己保存过的旧 RViz 配置，到左侧 **Displays** 勾选 **「YOLO (/human_yolo/annotated_image)」**。另请确认未设 **`TB3_NO_GUI=1` / `TB3_HEADLESS=1`**（无界面模式不启 RViz），且 **`TB3_ASSIST_SCAN_FILTER=1`**（默认即为 1）并已 **`colcon build` 出 `human_yolo_seg`**。仍无发布者时在仓库根执行 **`bash scripts/tb3_stack.sh check`**，或 **`bash scripts/tb3_stack.sh logs yolo`** 看 `yolo_person_seg.log`。

```bash
TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

另一终端（在 `ros_ws` 下已 `source install/setup.bash`）：

```bash
ros2 run robot_navigation coverage_patrol
```

停止栈：`bash scripts/tb3_stack.sh stop`

## 常用模式（环境变量）

在仓库根目录执行；切换前建议先 **`stop`**。

| 目的 | 示例 |
|------|------|
| 仅激光基线，**无 YOLO**（须关 filter） | `TB3_STACK_MODE=laser TB3_ASSIST_SCAN_FILTER=0 bash scripts/tb3_stack.sh start` |
| assist + YOLO（默认 mark_then_strip） | `TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start` |
| 滤人激光建图 | 同上，并 `export TB3_PERSON_SLAM_MODE=filtered` |
| 人物方位角用几何法（TF+地面） | 另设 `export TB3_PERSON_AZIMUTH_MODE=tf_geometry`（默认由脚本为 `linear_fov`） |
| Waffle + 深度桥 | 再加 `TURTLEBOT3_MODEL=waffle TB3_ASSIST_RGBD_BRIDGE=1` |

更多环境变量与日志：**[scripts/README.md](scripts/README.md)**  
工作区与包说明：**[ros_ws/README.md](ros_ws/README.md)**  
覆盖巡航、存图、Nav2：**[ros_ws/src/robot_navigation/robot_navigation/README.md](ros_ws/src/robot_navigation/robot_navigation/README.md)**  
对照建图实验（思路与步骤）：**[任务单/实验报告_AI语义识别剔除动态障碍物建图对照.md](任务单/实验报告_AI语义识别剔除动态障碍物建图对照.md)**  
体系原理（建图、导航、识人、去人）：**[任务单/实验原理_建图导航与语义去人.md](任务单/实验原理_建图导航与语义去人.md)**  
代码路径总索引：**[docs/代码与功能索引.md](docs/代码与功能索引.md)**

修改 `ros_ws/src` 后需重新 **`colcon build`**（或按需 `--packages-select`）。
