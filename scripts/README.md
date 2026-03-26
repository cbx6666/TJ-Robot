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
- 启动 `slam_toolbox`
- 生成或加载 RViz 配置
- 启动 `gzclient` 和 `rviz2`

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

检查内容包括：

- `/clock`
- `/scan`
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

查看 `slam_toolbox`：

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

## 常用环境变量

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
