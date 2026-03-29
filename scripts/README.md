# Scripts

这个目录负责环境安装、基础仿真拉起、运行检查、RViz 启动和日志查看。

## 脚本职责

- `setup_env.sh`
  安装 ROS 2 Humble、Gazebo、TurtleBot3、SLAM、Nav2、`colcon`，并安装自定义 ROS Python 节点需要的依赖，例如 `rclpy`、`geometry_msgs`、`nav_msgs`。
- `bootstrap.sh`
  一键执行环境安装、基础运行栈启动和健康检查。
- `tb3_stack.sh`
  负责日常运行控制，包括启动、停止、检查、单独打开 RViz 和查看日志。

## 推荐使用方式

### 1. 首次配置环境

```bash
bash scripts/setup_env.sh
```

这一步会自动：

- 安装 ROS 2、Gazebo、TurtleBot3、SLAM、Nav2、`colcon`
- 安装 ROS Python 依赖：`rclpy`、`geometry_msgs`、`nav_msgs`
- 把 `source /opt/ros/humble/setup.bash` 写入 `~/.bashrc`
- 把 `TURTLEBOT3_MODEL=burger` 写入 `~/.bashrc`
- 检查 `ros2`、`rviz2`、`gzserver`、`colcon`
- 检查关键 ROS 包是否可发现
- 检查 Python 能否导入 `rclpy`、`geometry_msgs`、`nav_msgs`

### 2. 一键完成安装、启动、检查

```bash
bash scripts/bootstrap.sh
```

适合第一次把整套基础环境拉起来。

### 3. 日常启动基础运行栈

```bash
bash scripts/tb3_stack.sh start
```

这一步会自动：

- 清理旧进程
- 启动 `gzserver`
- 展开 TurtleBot3 URDF
- 启动 `robot_state_publisher`
- 持续发布 `/robot_description`
- 生成 TurtleBot3 机器人
- 生成动态障碍物并启动其控制器
- 启动建图：**始终激光**（`slam_laser.launch.py`）。若已 `colcon build human_yolo_seg` 且 **`TB3_ASSIST_SCAN_FILTER=1`（默认）**：在 SLAM 前启动 **`scan_person_filter_node` + YOLO**，SLAM 订阅 **`/scan_filtered`**。**`TB3_ASSIST_SCAN_FILTER=0`** 时 SLAM 订阅 **`/scan`**。默认 **`TB3_STACK_MODE=laser`**（**burger** + 官方 `model.sdf`）；此时若 **`TB3_ASSIST_SCAN_FILTER=1` 且已 build `human_yolo_seg`**，会自动为 **burger 合并 URDF + 生成带 RGB 的 SDF**（与 waffle 相机同位姿），保证 **纯激光与 RGB+YOLO 辅助** 用**同一车型**。**`TB3_STACK_MODE=assist`**：未指定模型时默认 **waffle**（可注入深度）；若显式 **`TURTLEBOT3_MODEL=burger`**，则为 **burger + 仅 RGB**（无深度 SDF）。**`rgbd_to_scan` 默认关闭**，需 **`TB3_ASSIST_RGBD_BRIDGE=1`**。RViz 默认显示 **`/human_yolo/annotated_image`**，不再显示深度图。详见 `ros_ws/README`。日志含 `merge_burger_rgb.log`、`prepare_burger_rgb_sdf.log`（burger+RGB 时）。
- 生成或加载 RViz 配置
- 启动 `gzclient` 和 `rviz2`

启动前请在 `ros_ws` 下执行过 `colcon build`，以便加载 `robot_bringup`。

```bash
bash scripts/tb3_stack.sh start
TB3_STACK_MODE=assist bash scripts/tb3_stack.sh start   # 默认 waffle；深度桥另设 TB3_ASSIST_RGBD_BRIDGE=1
TURTLEBOT3_MODEL=burger TB3_STACK_MODE=assist bash scripts/tb3_stack.sh start   # assist 但车体仍为 burger（仅 RGB）
TB3_ASSIST_RGBD_BRIDGE=1 TB3_STACK_MODE=assist bash scripts/tb3_stack.sh start   # waffle assist 下再开 depth→/scan_rgbd
TB3_ASSIST_SCAN_FILTER=0 bash scripts/tb3_stack.sh start   # 关闭 YOLO/scan 过滤，SLAM 只用 /scan
```

默认机器人出生点：

- `x=-2.0`
- `y=-1.2`
- `z=0.1`
- `yaw=0.0`

如果需要修改机器人出生点：

```bash
ROBOT_START_X=-2.0 \
ROBOT_START_Y=-1.6 \
ROBOT_START_Z=0.1 \
ROBOT_START_YAW=0.0 \
bash scripts/tb3_stack.sh start
```

无界面运行时可以设置：

```bash
TB3_NO_GUI=1 bash scripts/tb3_stack.sh start
```

### 4. 日常检查

```bash
bash scripts/tb3_stack.sh check
```

`check` 校验 **`/scan`** 等与激光建图相关话题。若 **`TB3_ASSIST_SCAN_FILTER=1`（默认）**，会检查 **`/human_yolo/*`**、`/scan_filtered` 发布者（失败不退出，仅 WARN）。若 **`TB3_STACK_MODE=assist`** 且 **`TB3_ASSIST_RGBD_BRIDGE=1`**，再检查深度与 **`/scan_rgbd`**。

检查内容包括：

- `/clock`
- `/scan`（一键栈固定激光建图）
- `/odom`
- `/tf`
- `/tf_static`
- `/robot_description`
- `/map`
- `/map_metadata`
- `map -> odom` TF

### 5. 键盘遥控机器人

先用 `tb3_stack.sh start` 拉起仿真和 SLAM，再开一个新终端执行：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

常用按键：

- `w` 前进，`x` 后退，`a` / `d` 左转 / 右转
- `s` 或空格停止
- `q`、`e`、`z`、`c` 调整线速度和角速度

### 6. 仅重新打开 RViz

```bash
bash scripts/tb3_stack.sh rviz
```

这个命令会：

- 使用更稳的 WSL 软件渲染参数
- 自动加载当前 RViz 配置
- 默认显示 `TF`、`LaserScan`、`Odometry`、`Map`、`RobotModel`

### 7. 停止运行栈

```bash
bash scripts/tb3_stack.sh stop
```

会一起结束 `gzserver`、`gzclient`、`rviz2`、`slam_toolbox`、动态障碍控制器等相关进程。

## 日志

日志目录固定在：

```bash
/tmp/tb3_stack
```

查看可用日志：

```bash
bash scripts/tb3_stack.sh logs all
```

查看 Gazebo 服务端：

```bash
bash scripts/tb3_stack.sh logs gzserver
```

查看 Gazebo 客户端：

```bash
bash scripts/tb3_stack.sh logs gzclient
```

查看 RViz：

```bash
bash scripts/tb3_stack.sh logs rviz
```

查看 `robot_state_publisher`：

```bash
bash scripts/tb3_stack.sh logs rsp
```

查看 SLAM 日志（slam_toolbox 或 Cartographer）：

```bash
bash scripts/tb3_stack.sh logs slam
```

查看 `/robot_description` 发布器：

```bash
bash scripts/tb3_stack.sh logs robot_description
```

查看机器人生成日志：

```bash
bash scripts/tb3_stack.sh logs spawn
```

查看动态障碍日志：

```bash
bash scripts/tb3_stack.sh logs obstacle
```

RGB 掩膜 / assist 深度：

```bash
bash scripts/tb3_stack.sh logs scan_filter
bash scripts/tb3_stack.sh logs yolo
bash scripts/tb3_stack.sh logs rgbd
```

## 常用环境变量

- `TB3_STACK_MODE=laser|assist`（`assist` 可配合 `TURTLEBOT3_MODEL=burger` 保持 burger 车体 + RGB）
- `TB3_ASSIST_SCAN_FILTER=0|1`（默认 1：YOLO + `/scan_filtered` 建图，需已 build `human_yolo_seg`）
- `TB3_ASSIST_RGBD_BRIDGE=0|1`（默认 0；仅 assist 时起 `rgbd_to_scan`）
- `YOLO_CAMERA_INFO_TOPIC=/camera/camera_info`
- `YOLO_DEVICE=auto|cpu|cuda:0`（默认 `auto` 有 CUDA 则用 GPU，见 `ros_ws/README`）
- `TB3_NO_GUI=1`
- `RVIZ_CONFIG_FILE=<path>`
- `WORLD_FILE=<path>`
- `MAP_PGM_FILE=<path>`
- `MAP_YAML_FILE=<path>`
- `ROBOT_START_X=<value>`
- `ROBOT_START_Y=<value>`
- `ROBOT_START_Z=<value>`
- `ROBOT_START_YAW=<value>`
- `OBSTACLE_TRAJECTORY=[line|circle|figure8|lissajous|patrol]`
- `OBSTACLE_FALLBACK_RATE_HZ=<value>`
- `TB3_WAIT_POLL_SEC`（默认 `0.5`）：`tb3_stack.sh` 里等待 Gazebo 服务、`/tf_static`、`/map` 时每次探测的间隔；原为 1s，略缩短可少等几秒（仍受下面因素影响）

### 启动偏慢时先看这些

1. **WSL2 + 工程在 `/mnt/e/...`（Windows 盘）**  
   跨文件系统访问 NTFS/exFAT 时 **元数据与小文件 I/O 明显慢于 Linux 家目录（ext4）**。`colcon build`、`python3` 读大量小文件、Gazebo 读 `WORLD_FILE` 与模型若路径在 E 盘，都会拖慢。建议：把 **`ros_ws` 克隆到 `~/tj_ws` 一类纯 Linux 路径** 再编译运行；至少把 **日志与临时 URDF** 留在默认 **`TB3_LOG_DIR=/tmp/tb3_stack`**（已在 `/tmp`，不跟盘符走）。

2. **脚本里固定的等待链**（无法完全省掉）  
   `wait_for_service /spawn_entity`（最长约 20s）、`wait_for_topic /tf_static`（15s）、动态障碍的 `set_entity_state`（20s）、`wait_for_topic /map`（25s）。每一步都会周期性执行 **`ros2 service/topic list`**，每次子进程 + DDS 初始化在 WSL 上也不便宜。已把轮询从 **1s 改为默认 0.5s**（`TB3_WAIT_POLL_SEC`），就绪后能更快进入下一步。

3. **Gazebo + 软件渲染**  
   默认 `LIBGL_ALWAYS_SOFTWARE=1`（WSL 兼容）会拖慢 **gzserver** 首帧与加载。

4. **YOLO + PyTorch**  
   若 `TB3_ASSIST_SCAN_FILTER=1`，每次启动会加载权重、首次推理冷启动；**CPU** 上尤其明显。不需要时可 `TB3_ASSIST_SCAN_FILTER=0` 对比启动时间。

5. **杀毒 / Windows Defender**  
   实时扫描挂载盘上的文件有时会放大 WSL 侧延迟。

## 手动检查命令

如果不想用 `tb3_stack.sh check`，也可以手动执行：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map odom
```

## 常见问题

- 没有 `/spawn_entity`
  检查 `gzserver` 是否正常启动并加载了 `gazebo_ros_factory`
- 没有 `/tf_static`
  检查 `robot_state_publisher` 是否启动成功
- 没有 `/robot_description`
  检查 `/tmp/tb3_stack/robot_description.log`
- 没有 `map`
  先确认 `/scan`、`/odom`、`/tf_static` 正常，再让机器人移动起来
- 动态障碍不动
  先看 `bash scripts/tb3_stack.sh logs obstacle`
- 机器人起点离动态障碍太近
  启动时通过 `ROBOT_START_X/Y/Z/YAW` 调整出生点
