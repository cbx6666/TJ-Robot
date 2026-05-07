# robot_bringup

## 1. 包定位

`robot_bringup` 是系统级资源与启动编排包。  
该包负责维护仿真环境、地图资源、模型资源以及系统级 launch，不负责承载导航算法本体。

## 2. 主要职责

- Gazebo world 文件
- 机器人地图资源
- 机器人模型、URDF、SDF 相关资源
- 系统级 launch 文件
- 仿真辅助脚本

## 3. 目录说明

```text
robot_bringup/
├── config/     配置文件与 RViz 配置
├── launch/     系统级 launch
├── maps/       静态地图资源
├── models/     Gazebo 模型资源
├── scripts/    包内辅助脚本
├── urdf/       机器人描述与片段
├── world/      Gazebo 世界文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 4. 与其他包的边界

- `robot_bringup` 负责系统资源和系统入口
- `robot_navigation` 负责 Nav2 参数与导航 launch
- 感知、任务、交互、操作等功能不应把主实现继续堆入本包

## 5. 当前常用资源

- 默认地图：`maps/map.yaml`
- 默认世界：`world/small_house.world`
- 常用 RViz 配置：`config/test1.rviz`

## 6. 使用说明

该包通常不单独手工启动，而是被顶层脚本或其他 launch 引用，例如：

- `scripts/run_simulation.sh`
- `scripts/run_mapping.sh`
- `scripts/run_nav2.sh`

## 7. 维护建议

- 与仿真直接相关的资源留在本包
- 若某项逻辑已经演化为独立功能域，应迁移到专门功能包而不是继续扩大 bringup 范围
