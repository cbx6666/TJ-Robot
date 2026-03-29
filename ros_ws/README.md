# ROS 2 Workspace

开发代码统一放在 `ros_ws/src` 下。

目录说明：

- `robot_bringup`: 系统启动入口、launch、参数文件
- `robot_navigation`: 导航相关节点和配置
- `robot_tasks`: 任务逻辑与调度
- `robot_interfaces`: 自定义 `msg/srv/action`（当前为**占位包**，不跑 `rosidl`；模板见 `msg/NodeStatus.msg`。若在 **WSL + `/mnt/e/...` 含中文路径** 下启用 `rosidl_generate_interfaces` 会触发路径解析错误，需把工作区放到 **仅 ASCII 路径**（如 `~/tj_ws`）后再按 `CMakeLists.txt` 内注释恢复生成。）
- `human_yolo_seg`: **YOLO 实例分割（默认 COCO、仅 person）**，订阅 RGB + `CameraInfo`，发布 `/human_yolo/annotated_image`、**`/human_yolo/person_azimuth_ranges`**（**`sensor_msgs/JointState`**：`header` 与 **RGB 图像**同源时间戳，`position` 为 `[lo,hi,...]` 弧度）；**`scan_person_filter_node`** 用 **ROS 时间**做 **`hold_seconds`**，并校验 **激光 `scan.header.stamp` 与上述图像时间戳**之差（默认 ≤ **0.15 s** 才掩膜），减轻「上一帧人」套到「当前激光」的时空偏差。仍见 `mapper_params_online_async_scan_filtered.yaml`。

构建：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
```

如果提示 `colcon: command not found`，先执行：

```bash
bash ../scripts/setup_env.sh
```

加载：

```bash
cd ros_ws
source install/setup.bash
```

## YOLO-Seg 人体验证（人物模型未到前可先配环境）

依赖 **与运行 `ros2` 同一 Python**（可先 `which python3`、`ros2 doctor --report` 核对）。

**NumPy 须为 1.x**：Humble 自带的 **`cv_bridge`** 与 **NumPy 2.x** 不兼容。**OpenCV 4.12 及以上** 的 pip 包会**强制装回 NumPy 2.x**，因此 YOLO 环境必须固定 **OpenCV 4.10.0.84**（与 NumPy 1.26 兼容；见 `requirements.txt`）。

```bash
pip uninstall opencv-python opencv-python-headless -y 2>/dev/null || true
pip install 'numpy>=1.23,<2' 'opencv-python-headless==4.10.0.84'
pip install -r src/human_yolo_seg/requirements.txt
```

**权重文件（默认 `yolo26n-seg.pt`）**：将本机下载的 `yolo26n-seg.pt` 放到  
`ros_ws/src/human_yolo_seg/models/yolo26n-seg.pt`  
然后 `colcon build --packages-select human_yolo_seg`，节点会从 `install/.../share/human_yolo_seg/models/` 加载。也可用 launch 参数 `model_path:=/你的绝对路径/yolo26n-seg.pt` 跳过安装目录。

若 Ultralytics 版本过旧无法加载 YOLO26 权重：`pip install -U ultralytics torch torchvision`（与 `ros2` 同一 `python3`）。**不要**单独 `pip install -U opencv-python-headless`，以免升到 4.13+ 再次要求 NumPy 2。

安装完成后若 pip 提示 **`ultralytics requires opencv-python … which is not installed`**：可忽略。本项目使用 **`opencv-python-headless`**（同样提供 `cv2`，且无 GUI 依赖），与 Ultralytics 声明的包名不同，pip 会误报冲突，**不要**再装 `opencv-python`（会与 headless 冲突）。

**GPU 加速（NVIDIA）**：节点默认参数 **`device:=auto`**——若当前 **Linux 环境**里 `python3 -c "import torch; print(torch.cuda.is_available())"` 为 `True`，则自动用 **`cuda:0`** 推理；否则用 **CPU**。Windows 下 `nvidia-smi` 正常 **不代表** WSL 里同一 Python 已能 CUDA，需在 **跑 ROS 的那个 Linux/WSL** 里安装带 CUDA 的 PyTorch，例如（CUDA 12.4 示例，与 `torch` 官网索引一致即可）：

```bash
pip uninstall torch torchvision -y
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
python3 -c "import torch; print(torch.cuda.is_available(), torch.version.cuda)"
```

显式指定：`ros2 launch ... device:=cuda:0` 或 `device:=cpu`。一键栈：`YOLO_DEVICE=cuda:0 bash scripts/tb3_stack.sh start`（默认已是 `YOLO_DEVICE=auto`）。

**仿真已跑、有 `/camera/image_raw` 时**（Waffle + Gazebo 常见）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch human_yolo_seg yolo_person_seg.launch.py use_sim_time:=true
```

若 RGB 话题不同：

```bash
ros2 launch human_yolo_seg yolo_person_seg.launch.py use_sim_time:=true image_topic:=/你的/rgb
```

**RViz**：Add → Image → Topic `/human_yolo/annotated_image`，**Reliability** 选 **Best effort**（若需）。人物进场景后应能看到 **person 分割叠加**；检不出时调低 `conf_threshold` 或换更大模型 `model_path:=yolo11s-seg.pt`。

**建模验收（人物是否被识别）**：节点会发布 **`/human_yolo/person_present`**（bool）、**`person_count`**、**`person_max_conf`**；另可 **`ros2 topic echo /human_yolo/person_azimuth_ranges`** 看 **`JointState`**（注意 **`header.stamp` 应对齐相机图像**）。另可执行 **`ros2 run human_yolo_seg yolo_person_watch`** 看每秒中文提示。更完整说明见 **`src/human_yolo_seg/models/README.txt`**。

**时间同步说明**：几何上 TF 查询已按 **图像 `header.stamp`**（见 `person_azimuth.py`）。掩膜节点参数可调：**`sync_max_scan_image_delay_sec`**（激光与 RGB 时间允许偏差）、**`hold_seconds`**（在仿真下随 **`use_sim_time`** 走同一时钟）。若驱动把 **`/scan` 或图像打成 0 时间戳**，可设 **`skip_time_sync_if_zero_stamp:=true`**（默认）仅依赖 hold，或修好驱动时间戳。兼容旧版无戳角域：filter **`azimuth_msg_type:=float64_multiarray`**（不做 scan–图像同步）。

**独立跑过滤链**（不通过 `tb3_stack` 时）：

```bash
ros2 run human_yolo_seg scan_person_filter_node --ros-args -p use_sim_time:=true
```

## 建图（一键栈：激光为主；默认 RGB 掩膜、RGB-D 桥可选）

`tb3_stack.sh start` 使用 **`slam_laser.launch.py`**。若已 build **`human_yolo_seg`** 且 **`TB3_ASSIST_SCAN_FILTER=1`（默认）**，SLAM 参数为 **`mapper_params_online_async_scan_filtered.yaml`**，订阅 **`/scan_filtered`**（人物方向上的读数被置为 nan）；否则订阅原始 **`/scan`**。**`rgbd_to_scan`（`/scan_rgbd`）默认不启动**，仅在 **`TB3_STACK_MODE=assist`** 且 **`TB3_ASSIST_RGBD_BRIDGE=1`** 时开启。

| 环境变量 | 说明 |
|----------|------|
| `TB3_STACK_MODE=laser`（**默认**） | **burger**；若 **`TB3_ASSIST_SCAN_FILTER=1`** 且已 build **`human_yolo_seg`**，一键栈会**自动**为 burger 挂 **RGB**（`merge_tb3_burger_rgb_urdf` + `prepare_burger_rgb_camera_sdf`），与纯激光实验同车型。 |
| `TB3_STACK_MODE=assist` | 未指定模型时默认 **waffle**（SDF 可注入深度）；显式 **`TURTLEBOT3_MODEL=burger`** 时为 **burger + 仅 RGB**（无深度注入）。**`TB3_ASSIST_RGBD_BRIDGE=1`** 仅对 waffle 类有意义。 |
| `TB3_ASSIST_SCAN_FILTER=0|1` | 默认 **1**：YOLO + `scan_person_filter` +（若找到 yaml）SLAM 用 **`/scan_filtered`**。 |
| `TB3_ASSIST_RGBD_BRIDGE=0|1` | 默认 **0**：assist 下也不跑深度→假激光，省算力。 |
| `RGBD_DEPTH_IMAGE_TOPIC` / `RGBD_DEPTH_CAMERA_INFO_TOPIC` | 仅 **`TB3_ASSIST_RGBD_BRIDGE=1`** 时传给 `rgbd_to_scan.launch.py`。 |
| `YOLO_IMAGE_TOPIC` / `YOLO_CAMERA_INFO_TOPIC` | RGB 掩膜链订阅的话题（默认 `/camera/image_raw` 与 `/camera/camera_info`）。 |
| （已弃用）`SLAM_SENSOR=rgbd` | 等价于 **`TB3_STACK_MODE=assist`**，**不是**把 SLAM 切到 RGB-D；地图仍来自 `/scan`。 |

**assist 实现说明**：深度由 **`robot_bringup/scripts/prepare_assist_waffle_sdf.py`** 写入临时 SDF；`merge_tb3_waffle_depth_urdf.py` 仍保留，供你在 **非 tb3_stack** 场景下手动合并 URDF 时使用。

**另：单独做 RGB-D 为主传感器的 SLAM 实验**（与一键栈无关，需自行 launch）：

```bash
source install/setup.bash
ros2 launch robot_bringup slam_rgbd.launch.py
# 或 Cartographer 后端：
ros2 launch robot_bringup slam_rgbd_cartographer.launch.py
```

参数与话题见 `launch/rgbd_to_scan.launch.py`。

RViz：默认一键栈建图看 **`/scan_filtered`**（若启用掩膜）或 **`/scan`**；assist 且 **`TB3_ASSIST_RGBD_BRIDGE=1`** 或手动跑 `slam_rgbd` 时可再看 **`/scan_rgbd`**。

**Map 一直 WARN、栅格不显示**：`slam_toolbox` 只发 **`/map`**，不发 **`/map_updates`**。请用 **`test1.rviz`**（已去掉 `map_updates`），或自己在 Map 显示里清空 **Update Topic**。**`/scan_filtered`** 发布为 **Reliable**，便于 slam_toolbox 订阅。

**Global Status: Frame [map] does not exist、激光也不显示**：不要把 **Fixed Frame** 设成 **`map`** 直到 **slam 已发布 `map` TF**（启动后前几秒或建图失败时 `map` 可能不存在）。仓库 **`test1.rviz` 默认 Fixed Frame 为 `odom`**，激光与里程计可立即显示；等 **`ros2 run tf2_ros tf2_echo map odom`** 有输出后，再在 RViz 里把 Fixed Frame 改成 **`map`** 看栅格更直观。

**`/scan` 有数据但 RViz 里看不见激光点**：Gazebo 里 **`intensities` 常为全 0**，不要用 **Intensity** 着色且上下界为 0；**`test1.rviz`** 用 **FlatColor** 固定 **红色**（255,0,0）。**LaserScan 订 `/scan` 用 Best Effort** 与 Gazebo 一致。

**仅激光建图正常、RGB+YOLO（`/scan_filtered`）不正常**：掩膜曾用 **`nan`**，**slam_toolbox / Karto** 对 `nan` 的处理可能异常。`scan_person_filter_node` 默认 **`masked_range_mode:=inf`**（与 TB3 激光里 `.inf` 无回波一致）；若需原语义可设 **`nan`** 或 **`range_max`**。

**「按方位角剔除人」是否太简？** 2D LDS 扫的是**近似水平面**的一条线；地图里能体现「地面」多半是**同一平面上的几何**（墙、障碍物边缘）或**反射/杂散**，不是相机那种「看见地板纹理」。本方案把人在**地面 z≈0** 与**相机射线**求交，再投影到**激光系水平角**——等价于假设人脚附近与激光**同一高度层**相交，**不**区分垂直方向上人与地板谁更近；角域过宽会多挡、TF/地面模型不准会偏。更精细需要 3D、多线激光或深度与人对齐，属于后续扩展。