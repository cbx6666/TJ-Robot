# ROS 2 Workspace

开发代码统一放在 `ros_ws/src` 下。

目录说明：

- `robot_bringup`: 系统启动入口、launch、参数文件
- `robot_navigation`: 导航相关节点和配置
- `robot_tasks`: 任务逻辑与调度
- `robot_interfaces`: 自定义 `msg/srv/action`

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