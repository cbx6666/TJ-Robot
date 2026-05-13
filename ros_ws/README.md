# ros_ws

## 1. 工作区定位

`ros_ws/` 是本项目的 ROS 2 工作区。  
所有需要通过 `colcon build` 构建、通过 `ros2 run` 或 `ros2 launch` 调用的 ROS 包，都统一维护在 `ros_ws/src/` 中。

## 2. 目录说明

```text
ros_ws/
├── src/        ROS 2 包源码
├── build/      colcon 构建产物
├── install/    安装空间
└── log/        构建日志
```

其中：

- `src/` 是唯一需要长期维护的源码目录
- `build/`、`install/`、`log/` 均为生成产物

## 3. 构建方式

推荐在工作区目录执行以下命令：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只构建部分包，可执行：

```bash
colcon build --packages-select robot_bringup robot_navigation
```

## 4. 运行关系

工作区中的 ROS 包通常通过仓库顶层 `scripts/` 调用，而不是在 `ros_ws/` 中手工维护长命令。职责分层如下：

- `ros_ws/src/*`：定义节点、launch、参数、资源
- `scripts/*`：封装标准启动入口

例如：

- `scripts/run_simulation.sh`：默认仿真入口
- `scripts/run_mapping.sh`：建图入口
- `scripts/run_nav2.sh`：Nav2 与自动巡视统一入口

## 5. 维护注意事项

- 不要把 `install/` 下文件当作主源码修改
- 每次修改 ROS 包源码后，都应重新构建并重新 source 环境
- 若运行行为与源码不一致，应优先检查工作区是否重新构建
