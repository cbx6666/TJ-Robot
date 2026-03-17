# Scripts

这个目录负责项目的基础环境安装、基础运行栈启动、健康检查、RViz 启动和日志查看。

## 脚本职责

- `setup_env.sh`
  负责安装基础依赖，并把 ROS 环境和 `TURTLEBOT3_MODEL=burger` 写入 `~/.bashrc`。
  安装完成后会自动检查核心命令和关键 ROS 包。

- `bootstrap.sh`
  负责一键串联完整流程。
  它会依次执行：
  1. `setup_env.sh`
  2. `tb3_stack.sh start`
  3. `tb3_stack.sh check`

- `tb3_stack.sh`
  负责日常运行控制，包括启动、停止、检查、打开 RViz、查看日志。

## 推荐使用方式

### 1. 第一次配置环境

```bash
bash scripts/setup_env.sh
```

这一步会自动：

- 安装 ROS 2、Gazebo、TurtleBot3、SLAM、Nav2、colcon
- 把 ROS 环境写入 `~/.bashrc`
- 把 `TURTLEBOT3_MODEL=burger` 写入 `~/.bashrc`
- 检查 `ros2`、`rviz2`、`gzserver`、`colcon`
- 检查关键 ROS 包是否可发现

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
- 生成 Burger 机器人
- 启动 `slam_toolbox`
- 生成 RViz 配置 `/tmp/tb3_stack/tb3_auto.rviz`

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

### 5. 打开 RViz

```bash
bash scripts/tb3_stack.sh rviz
```

这个命令会自动：

- 使用 WSL 更稳的软件渲染参数
- 自动加载生成的 RViz 配置
- 默认显示 `TF / LaserScan / Odometry / Map / RobotModel`

### 6. 停止运行栈

```bash
bash scripts/tb3_stack.sh stop
```

## 日志

日志目录固定在：

```bash
/tmp/tb3_stack
```

查看可用日志：

```bash
bash scripts/tb3_stack.sh logs all
```

查看 Gazebo：

```bash
bash scripts/tb3_stack.sh logs gzserver
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

查看实体生成日志：

```bash
bash scripts/tb3_stack.sh logs spawn
```

## 手动检查命令

如果你不想用 `tb3_stack.sh check`，也可以手动执行：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map odom
```

## 常见问题

- 没有 `/spawn_entity`
  检查 `gzserver` 是否带了 `libgazebo_ros_factory.so`

- 没有 `/tf_static`
  检查 `robot_state_publisher` 是否启动成功

- 没有 `/robot_description`
  检查 `/tmp/tb3_stack/robot_description.log`

- 没有 `map`
  先确认 `/scan`、`/odom`、`/tf_static` 正常，再让机器人转几秒

- RViz 里地图显示异常
  这通常是 WSL 图形兼容问题，不代表后端建图失败。优先用 `tb3_stack.sh check` 和 `tf2_echo map odom` 判断系统状态。
