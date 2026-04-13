# 实验报告：AI 语义识别辅助剔除动态障碍物建图可行性验证

**实验平台**：TJ-Robot 仓库 · ROS 2 Humble · Gazebo · TurtleBot3（Burger + 仿真 RGB）· slam_toolbox · YOLO-Seg（`human_yolo_seg`）

**技术主线**：在动态环境下，以 **前向 RGB + YOLO-Seg 人物方位角记录 + slam_toolbox 语义后处理清障** 作为核心方案，通过**对照实验验证**相对基线能否提升建图鲁棒性。

**原理详解（建图 / 导航 / 识人 / 去人）**：见同目录 **[实验原理_建图导航与语义去人.md](./实验原理_建图导航与语义去人.md)**。
**功能对应源码路径总表**：见仓库 **[docs/代码与功能索引.md](../docs/代码与功能索引.md)**。

---

## 1. 实验目的与思路

### 1.1 问题背景

室内服务机器人在动态环境（如有人走动）下采用激光建图时，人体等动态物体会被激光持续重复扫描，进而在占据栅格中形成伪障碍、拖尾与鬼影，严重降低地图可用性。传统纯激光方法无法有效区分静态墙体等固定障碍物与行人等动态目标，而在基于RGB-D的建图过程中，动态物体的持续观测信息会被SLAM前端误判为静态环境特征参与位姿匹配与地图更新，随着多帧数据不断累积，栅格地图中会逐渐出现大量错误占据区域与冗余鬼影，造成地图结构失真，不仅影响定位稳定性，还会降低后续导航决策的可靠性。

### 1.2 核心思路（本仓库实现路径）

在**同一仿真场景、同一 SLAM 配置、同一行驶路径**下做对照：

| 组别                           | 思路概要                                                                                                                                                            |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **对照组（仅激光）**     | 关闭 YOLO 人物链，SLAM 直接使用原始 `/scan`，动态人完全进入地图，作为**基线**。                                                                             |
| **实验组（YOLO + RGB）** | 前向 RGB 上运行 YOLO-Seg 检人，估计人物在**激光系下的方位角区间**；建图阶段仅记录人物区域，不改写 SLAM 输入；建图结束后再对保存的地图做**语义后处理**，将人物对应区域刷为空闲。 |

从而验证命题：

> **在动态行人干扰下，引入 AI 语义（人体）信息后，最终地图中由人引起的错占据是否减轻、是否具备工程可行性**（含实时性、同步、漏检等边界）。

### 1.3 mark_then_strip 路线

其核心做法是建图阶段保持 SLAM 前端输入为原始激光 scan，不改变与基线组的一致性；同时并行记录“人物角域与激光投影”得到人物区域，待地图保存后再执行语义清障。这样可以把实验变量控制在“是否使用语义后处理”这一点上，保证对照公平、结论可归因。相比之下，filtered 更像补充验证方案：它在建图前端直接改用过滤后的激光输入，变量发生变化，因此需要单列说明，通常不作为与基线同口径比较的主结论来源。

---

## 2. 实验环境与控制变量

### 2.1 软件与硬件

- Ubuntu + ROS 2 Humble
- Gazebo 世界： `robot_bringup/world/test1.world`
- 机器人：**TurtleBot3 Burger + 仿真 RGB**（`TB3_STACK_MODE=assist`），激光 `/scan`，图像 `/camera/image_raw`。
- 动态干扰：世界中**移动障碍物/行人脚本**保持开启，且两组实验使用**相同轨迹与速度参数。**

### 2.2 控制变量

- 世界文件、机器人初始位姿
- 动态障碍物的运动模式（周期、幅度、速度）
- 遥控或巡航路径：线路固定
- slam_toolbox 参数文件，两组除是否启用 YOLO 链外一致。
- YOLO 权重、置信度阈值、`imgsz`（实验组内固定）。

### 2.3 自变量与因变量

| 类型             | 内容                                                                                                                                                            |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **自变量** | 是否启用 YOLO+RGB 人物链（`TB3_ASSIST_SCAN_FILTER`）；是否采用 `mark_then_strip` 或 `filtered`；是否执行 `strip_saved_map_person_free`。 |
| **因变量** | 保存地图中人物常出现区域的占据情况（定性截图 / 可选栅格统计）；动态人离开后的残留鬼影；建图全程是否丢定位；实验组 strip 前后地图对比；YOLO 延迟与角域是否稳定。 |

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
2. YOLO 权重放置于包内 `models/` 或 `model_path` 指向绝对路径；确认与 `ros2` 同环境的 Python 已安装 `ultralytics`、`numpy<2` 等（见 [human_yolo_seg/requirements.txt](../ros_ws/src/human_yolo_seg/requirements.txt)）。
3. 清空或备份上次人物区域文件，避免串实验：
   - 默认：`~/.ros/tj_person_strip_regions.yaml`（实验组写入）。
4. 若需要核对启动状态，先执行一次：
   ```bash
   bash scripts/tb3_stack.sh check
   ```

### 3.2 对照组：仅激光建图

1. 启动栈（**关闭 YOLO 人物链**，保证只跑激光基线）：
   ```bash
   cd <仓库>
   TB3_STACK_MODE=laser TB3_ASSIST_SCAN_FILTER=0 bash scripts/tb3_stack.sh start
   ```
2. 按《标准行进协议》遥控或运行巡航，使机器人**多次经过动态人活动区域**，持续时间与实验组相同。
3. 在 RViz 中观察 `/map`，待地图基本收敛后保存地图。仓库里与巡航节点一致的做法是用 `map_saver_cli`：
   ```bash
   source <仓库>/ros_ws/install/setup.bash
   ros2 run nav2_map_server map_saver_cli -f map_baseline_only_laser
   ```
   生成的文件为 `map_baseline_only_laser.yaml` 和 `map_baseline_only_laser.pgm`。
4. 记录：截图或录屏、保存 bag（可选）、备注异常（丢定位、错位等）。
5. `bash scripts/tb3_stack.sh stop`。

### 3.3 实验组：YOLO + RGB（mark_then_strip + 事后清障）

1. 启动栈（**开启 assist + YOLO**，默认就是 `mark_then_strip` 路线；如需显式指定可先 `export TB3_PERSON_SLAM_MODE=mark_then_strip`）：

   ```bash
   cd <仓库>
   TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
   ```
2. 确认关键话题有数据：`/human_yolo/annotated_image`、`/human_yolo/person_azimuth_ranges`、`/human_yolo/person_azimuth_markers`，以及建图过程中持续写入的 `/human_yolo/person_laser_map_cloud`。RViz 里重点看 `/map`、`/human_yolo/annotated_image` 和 `/human_yolo/person_azimuth_markers`；`/human_yolo/person_laser_map_cloud` 需要按 PointCloud2 的 `RGB8` 着色方式显示。
3. 使用**与对照组相同**的行进协议与时长建图；`mark_then_strip` 路线下 SLAM 仍订阅 **`/scan`**，不改动建图前端。
4. 建图结束后先保存原始地图：
   ```bash
   source <仓库>/ros_ws/install/setup.bash
   ros2 run nav2_map_server map_saver_cli -f map_yolo_rgb_before_strip
   ```
   生成 `map_yolo_rgb_before_strip.yaml` 和 `map_yolo_rgb_before_strip.pgm`。
5. 再对保存地图执行语义清障：

   ```bash
   source <仓库>/ros_ws/install/setup.bash
   ros2 run human_yolo_seg strip_saved_map_person_free \
     map_yolo_rgb_before_strip.yaml \
     ~/.ros/tj_person_strip_regions.yaml
   ```
   默认会输出 `map_yolo_rgb_before_strip_person_free.yaml` 和 `map_yolo_rgb_before_strip_person_free.pgm`。
6. 记录：strip 前后地图对比截图、`tj_person_strip_regions.yaml` 中区域数量、YOLO 日志中角域是否合理。若人物区域投影整体上下偏一格，可再试 `strip_saved_map_person_free --no-flip-y` 对齐。
7. `bash scripts/tb3_stack.sh stop`。

## 4. 风险点与讨论要点

本实验中影响“动态人物剔除完整性”和“建图一致性”的主要风险来自感知视场与导航采样策略两方面，且二者在代码链路中存在耦合。

1. 感知视场不对称导致人物区域记录不完整当前方案使用前置单目 RGB（约 135° 水平视场）生成人物角域 `/human_yolo/person_azimuth_ranges`，而激光输入为 360°。在 `mark_then_strip` 路径中，`person_strip_recorder_node` 仅会对“被视觉检测到的人物角域”对应的激光束做 map 系累积标记；处于相机盲区（侧后方）的人物，即使被激光观测到，也不会进入 `~/.ros/tj_person_strip_regions.yaml`，从而在事后 `strip_saved_map_person_free` 清障时无法完全擦除。这解释了实验中“人物残留占据”主要出现在机器人后侧与转弯过渡段的现象。
2. 时空配准误差会放大误删/漏删风险人物角域来自图像推理链路，激光来自独立传感器链路。当前实现通过 `hold_seconds` 与 `sync_max_scan_image_delay_sec` 做时间窗匹配；当机器人线速度偏高、角速度变化较大时，图像时刻对应的人物方位与激光采样时刻发生偏移，易出现“人物角域与激光束错配”。同时，实验中为降低复杂度主要使用检测框角域而非逐像素掩膜投影，也会在边界处引入额外几何误差。上述误差会共同导致动态人物未完全剔除，或少量静态结构被误标记。
3. 覆盖巡航强度不足导致地图质量波动本次建图采用自研 `coverage_patrol` 单轮覆盖策略，能够保证空间被扫过一次，但对 15m×15m 大场地而言，单轮覆盖对回环约束与局部漂移抑制不足。尤其在场地中部“远离墙体、几何约束弱”的区域，若速度上限（如 `linear_speed_limit`）偏高，会放大位姿累计误差，造成地图错位。错位不仅影响地图边界一致性，也会把人物标记点投到偏移位置，进而影响 strip 阶段的清障有效性。

## 5. 结论

本实验验证了“YOLO 语义感知 + 激光角域映射 + 事后 strip 清障”路线在静态人物去除上的有效性，但在动态人物场景下仍受时空配准误差与建图漂移影响，表现为局部残留与少量误删并存。

1. 核心结论

   - 在 `mark_then_strip` 流程下，人物角域由 `/human_yolo/person_azimuth_ranges` 提供，`person_strip_recorder_node` 将角域内激光点投影到 map 并累积到区域文件，再由 `strip_saved_map_person_free` 统一清障。该链路能够稳定去除多数静态人物占据。
   - 对动态人物，清除效果受时间同步和空间几何精度共同约束，无法保证每一帧都被完整记录，因此会出现“部分动态残留”。
2. 误差来源与代码对应

   - 时间误差：当前系统使用约 0.5 s 级别匹配窗口（`sync_max_scan_image_delay_sec`）与短时保持机制（`hold_seconds`）对齐图像检测与激光扫描。机器人运动较快或转向频繁时，人物在两传感器采样时刻的相对方位变化会增大，导致角域与激光束错配。
   - 空间误差：虽然检测模型支持分割，但本实验角域计算主要使用检测框近似，未进行逐像素掩膜几何约束；叠加建图阶段位姿错位，会使人物区域在 map 上出现投影偏差，进而引发“动态漏删”与“静态结构轻微误删”。
3. 对实验结果的解释

   - “静态人物可被完全剔除”与链路设计一致：静态目标在多帧中重复出现，角域-激光融合后的区域累积更充分，strip 阶段可形成连续清障覆盖。
   - “动态人物未完全剔除”主要来自短时可见性不足、角域同步误差与巡航建图漂移叠加，而非单一模块失效。
4. 后续优化方向

   - 采用更严格的时序对齐与参数标定（`hold_seconds`、`sync_max_scan_image_delay_sec` 联合调优）。
   - 由检测框角域升级为掩膜边界角域或射线级融合，降低空间近似误差。
   - 降低建图速度并增加重复覆盖轮次，抑制 map 漂移对人物区域投影的二次放大。
   - 增设量化指标（动态人物残留面积、误删面积、重复实验重合度）以支撑结论可复现。

---

## 6. 参考命令与文档索引

| 内容               | 位置                                                           |
| ------------------ | -------------------------------------------------------------- |
| 一键栈与环境变量   | 仓库根目录 `README.md`、`scripts/README.md`                |
| 人物 SLAM 模式说明 | 根目录 `README.md` 表格                                      |
| 覆盖巡航与存图     | `ros_ws/src/robot_navigation/robot_navigation/README.md`     |
| strip 工具帮助     | `ros2 run human_yolo_seg strip_saved_map_person_free --help` |
