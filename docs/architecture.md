# 架构说明

当前仓库采用 **ROS 2 工作空间单源架构**：所有运行代码以 `ros_ws/src` 为唯一真源。

## 顶层结构

```text
ros_ws/
  src/                   # ROS2 包源码（唯一真源）
scripts/                 # 构建与运行脚本
data/                    # maps/logs/results 产物
docs/                    # 设计与实验文档
```

## ros_ws/src 包职责

- `robot_bringup`：系统编排（launch/world/map/config/scripts）。
- `robot_navigation`：导航与覆盖逻辑。
- `human_yolo_seg`：YOLO 人物链路、激光过滤、地图后处理工具。
- `robot_tasks`：任务管理节点。
- `robot_interfaces`：接口定义与扩展入口。

## 开发约定

- 新功能只在 `ros_ws/src` 对应包内实现。
- `scripts/` 只做编排，不写核心业务逻辑。
