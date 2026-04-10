# 实验报告：AI 语义识别辅助剔除动态障碍物建图可行性验证

**实验平台**：TJ-Robot 仓库 · ROS 2 Humble · Gazebo · TurtleBot3（Burger + 仿真 RGB）· slam_toolbox · YOLO-Seg（`human_yolo_seg`）

**文档说明**：本报告对齐本仓库**已实现的实验链路**（2D 激光建图 + 可选 YOLO/RGB 人物语义），与「仅激光基线 vs YOLO+RGB 方案」的对照思路一致，便于组内归档与答辩引用。

**原理详解（建图 / 导航 / 识人 / 去人）**：见同目录 **[实验原理_建图导航与语义去人.md](./实验原理_建图导航与语义去人.md)**。  
**功能对应源码路径总表**：见仓库 **[docs/代码与功能索引.md](../docs/代码与功能索引.md)**。

---

## 1. 实验目的与思路

### 1.1 问题背景

室内服务机器人在有人走动等**动态环境**下用激光建图时，人体会被激光反复命中，在占据栅格中形成**伪障碍、拖尾或鬼影**，降低地图可用性。传统纯激光方法无法区分「静态墙」与「动态人」。

### 1.2 核心思路（本仓库实现路径）

在**同一仿真场景、同一 SLAM 配置、同一行驶路径**下做对照：

| 组别 | 思路概要 |
|------|-----------|
| **对照组（仅激光）** | 关闭 YOLO 人物链，SLAM 直接使用原始 `/scan`，动态人完全进入地图，作为**基线**。 |
| **实验组（YOLO + RGB）** | 前向 RGB 上运行 YOLO-Seg 检人，估计人物在**激光系下的方位角区间**，建图时记录「人向」信息；建图结束后对保存的地图做**语义后处理**，将人物对应区域刷为空闲（或采用「滤扫建图」旧模式，见 §3.2）。 |

从而验证命题：

> **在动态行人干扰下，引入 AI 语义（人体）信息后，最终地图中由人引起的错占据是否减轻、是否具备工程可行性**（含实时性、同步、漏检等边界）。

### 1.3 与「滤扫实时剔除」的区分

本仓库支持两种与语义结合的方式（实验可二选一或都做）：

- **默认推荐（mark_then_strip）**：建图仍用**原始 `/scan`**（与对照组 SLAM 输入形式一致），同时 `person_strip_recorder` 根据角域与激光在 map 上累积人物区域 YAML，**保存地图后**运行 `strip_saved_map_person_free` 清障。对照实验时与基线**公平对比「同一 SLAM 前端」**，差异在**后处理是否利用语义**。
- **filtered 模式**：`scan_person_filter` 实时掩膜 `/scan` 中人向读数，SLAM 订阅 `/scan_filtered`。与基线**前端不一致**，适合作为「实时滤人」补充实验，需在报告中单独说明变量。

**本报告步骤以「mark_then_strip + 事后 strip」为主流程撰写**（与 `tb3_stack` 默认一致）。

---

## 2. 实验环境与控制变量

### 2.1 软件与硬件（仿真）

- Ubuntu + ROS 2 Humble；本仓库 `ros_ws` 已 `colcon build`（含 `human_yolo_seg`、`robot_bringup`）。
- Gazebo 世界：如 `robot_bringup/world/test1.world`（与 `tb3_stack` 默认一致即可）。
- 机器人：**TurtleBot3 Burger + 仿真 RGB**（`TB3_STACK_MODE=assist`），激光 `/scan`，图像 `/camera/image_raw`。
- 动态干扰：世界中**移动障碍物/行人脚本**保持开启，且两组实验使用**相同轨迹与速度参数**。

### 2.2 应尽量固定的量（控制变量）

- 世界文件、机器人初始位姿（或统一复位流程）。
- 动态障碍物的运动模式（周期、幅度、速度）。
- **遥控或巡航路径**：《标准行进协议》——建议固定：走闭合回路、速度上限、总时长或总路程。
- slam_toolbox 参数文件（如 `mapper_params_online_async_full_scan.yaml`），两组除「是否启用 YOLO 链」外尽量一致。
- YOLO 权重、置信度阈值、`imgsz`（实验组内固定）。

### 2.3 自变量与因变量

| 类型 | 内容 |
|------|------|
| **自变量** | 是否启用 YOLO+RGB 人物链（`TB3_ASSIST_SCAN_FILTER`）；可选是否执行 `strip_saved_map_person_free`。 |
| **因变量（建议记录）** | 保存地图中人物常出现区域的占据情况（定性截图 / 可选栅格统计）；动态人离开后的残留鬼影；建图全程是否丢定位；实验组 strip 前后地图对比；YOLO 延迟与角域是否稳定（日志或 `person_azimuth_debug`）。 |

---

## 3. 实验步骤

### 3.1 实验前准备

1. 安装依赖并完成编译：
   ```bash
   cd <仓库>/ros_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select human_yolo_seg robot_bringup
   source install/setup.bash
   ```
2. YOLO 权重放置于包内 `models/` 或 `model_path` 指向绝对路径；确认与 `ros2` 同环境的 Python 已安装 `ultralytics`、`numpy<2` 等（见 `human_yolo_seg/requirements.txt`）。
3. 清空或备份上次人物区域文件，避免串实验：
   - 默认：`~/.ros/tj_person_strip_regions.yaml`（实验组会写入）。

### 3.2 对照组：仅激光建图

1. 启动栈（**关闭 YOLO 人物链**，激光模式即可）：
   ```bash
   cd <仓库>
   TB3_STACK_MODE=laser TB3_ASSIST_SCAN_FILTER=0 bash scripts/tb3_stack.sh start
   ```
2. 按《标准行进协议》遥控或运行巡航，使机器人**多次经过动态人活动区域**，持续时间与实验组相同。
3. 在 RViz 中观察 `/map`，保存地图（如 `map_saver`），命名建议带前缀，例如：`map_baseline_only_laser.yaml` + `.pgm`。
4. 记录：截图或录屏、保存 bag（可选）、备注异常（丢定位、错位等）。
5. `bash scripts/tb3_stack.sh stop`。

### 3.3 实验组：YOLO + RGB（mark_then_strip + 事后清障）

1. 启动栈（**开启 assist + YOLO**，默认 `TB3_PERSON_SLAM_MODE=mark_then_strip`）：
   ```bash
   cd <仓库>
   TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
   ```
2. 确认话题：`/human_yolo/annotated_image`、`/human_yolo/person_azimuth_ranges` 有数据；RViz 可开 `/human_yolo/person_azimuth_markers` 与 `/human_yolo/person_laser_map_cloud`（黄点云为人向累积，需按包说明设置 PointCloud2 着色）。
3. 使用**与对照组相同**的行进协议与时长建图；SLAM 仍订阅 **`/scan`**。
4. 建图结束后保存地图：`map_yolo_rgb_before_strip.yaml` + `.pgm`。
5. 对保存地图执行语义清障（路径按你实际文件调整）：
   ```bash
   source <仓库>/ros_ws/install/setup.bash
   ros2 run human_yolo_seg strip_saved_map_person_free \
     map_yolo_rgb_before_strip.yaml \
     ~/.ros/tj_person_strip_regions.yaml
   ```
   按工具说明确认输出文件名（一般为带后缀或覆盖策略的**新 yaml**），得到 **`map_yolo_rgb_after_strip`**。
6. 记录：strip 前后地图对比截图、`tj_person_strip_regions.yaml` 中区域数量、YOLO 日志中角域是否合理。
7. `bash scripts/tb3_stack.sh stop`。

### 3.4 可选补充实验：filtered 实时滤扫建图

若需验证「建图输入即为滤后人扫」：

```bash
export TB3_PERSON_SLAM_MODE=filtered
TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

此时 SLAM 使用 **`/scan_filtered`** 与对照组**不一致**，报告中须单列一小节说明，不宜与「同 SLAM 前端」的 mark_then_strip 结论混为一条。

### 3.5 重复次数

- 建议每组至少 **3 次**独立重复（重新启动仿真、清空地图与 regions），减少随机性；时间允许可做 5 次。

---

## 4. 结果记录与对比方法（建议表格）

| 试验序号 | 组别 | 地图文件 | 人活动区占据（定性：轻/中/重） | 鬼影/拖尾 | 定位异常 | 备注 |
|----------|------|----------|--------------------------------|-----------|----------|------|
| 1 | 对照 | … | | | | |
| 1 | 实验（strip 前） | … | | | | |
| 1 | 实验（strip 后） | … | | | | |

可选定量：在已知动态人活动 ROI 内统计占据栅格比例（需统一 ROI 多边形），或使用 `annotate_saved_map_person_overlay` 生成叠加图辅助展示（见 `human_yolo_seg` 说明）。

---

## 5. 风险点与讨论要点（写入结论节）

1. **时间同步**：角域来自 RGB 时间戳，与 `/scan` 需在 `sync_max_scan_image_delay_sec` 内对齐；否则会出现「角域与人位置错位」。
2. **漏检 / 误检**：YOLO 未检出则该段无 strip；误检可能错误清除静态障碍。
3. **linear_fov vs tf_geometry**：人物方位角有粗略与几何两种模式，实验组内应固定一种并在报告中写明。
4. **mark_then_strip 语义**：地图**保存瞬间**人仍站在某格，该格仍可能被占据；strip 依赖**累积的 regions**，与「人是否被激光持续看到」有关。
5. **实时性**：YOLO 帧率、GPU/CPU 负载影响角域更新频率。

---

## 6. 结论模板（实验后填写）

- **可行性**：在本文实验条件下，YOLO+RGB + 事后 strip（和/或 filtered）是否减轻动态人引起的地图错占据：**是 / 部分 / 否**。
- **主要证据**：对照图、strip 前后对比、重复实验一致性。
- **主要限制**：________  
- **后续工作**（可选）：换 `tf_geometry`、调同步与 hold、与 Nav2 联调、或向课程主线 RGB-D + Cartographer 方案迁移时的差异说明。

---

## 7. 参考命令与文档索引

| 内容 | 位置 |
|------|------|
| 一键栈与环境变量 | 仓库根目录 `README.md`、`scripts/README.md` |
| 人物 SLAM 模式说明 | 根目录 `README.md` 表格 |
| 覆盖巡航与存图 | `ros_ws/src/robot_navigation/robot_navigation/README.md` |
| strip 工具帮助 | `ros2 run human_yolo_seg strip_saved_map_person_free --help` |

---

*文档版本：与 TJ-Robot 仓库 `tb3_stack` / `human_yolo_seg` 当前默认行为对齐；若 launch 或参数变更，请同步修订 §3。*
