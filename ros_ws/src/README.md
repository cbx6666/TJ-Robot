# ros_ws/src

## 1. 目录定位

`ros_ws/src/` 是本项目全部 ROS 2 功能包的源码根目录。  
该目录仅存放 ROS 包源码，不存放工作区级脚本，也不存放构建产物。

## 2. 当前包划分

### 2.1 系统与仿真

- `robot_bringup`
  - 系统级 launch
  - Gazebo world
  - 地图资源
  - 模型与仿真辅助文件

### 2.2 导航

- `robot_navigation`
  - Nav2 launch
  - Nav2 参数
  - 基于静态地图的导航入口

### 2.3 感知

- `human_yolo_seg`
  - YOLO 检测节点
  - 感知 launch
  - 感知辅助工具

### 2.4 任务与交互

- `robot_tasks`
  - 任务管理节点与任务层逻辑
- `robot_interaction`
  - 语音输入与上层交互接口
- `robot_manipulation`
  - mock 操作执行接口
- `robot_interfaces`
  - 跨包消息、服务与动作接口定义

## 3. 包内组织建议

ROS 包内部建议按职责组织：

- `launch/`：启动编排
- `config/`：参数与 YAML 配置
- `msg/`、`srv/`、`action/`：接口定义
- `<package_name>/nodes/`：可运行节点
- `<package_name>/utils/`：公共工具
- `<package_name>/tools/`：离线工具或开发辅助脚本
- `resource/`：`ament_python` 包索引文件

## 4. 职责边界

- `robot_bringup` 负责系统资源与系统级启动
- `robot_navigation` 负责导航，不直接承载 Gazebo 世界和地图资源本体
- 感知、任务、交互、操作相关能力应尽量保持包级独立

## 5. 开发注意事项

- 修改任何包源码后，都需要重新构建工作区
- 若只是调整启动方式，优先修改 `launch/` 或顶层 `scripts/`
- 若新增长期功能，应优先考虑独立包，而不是继续向已有大包堆积
