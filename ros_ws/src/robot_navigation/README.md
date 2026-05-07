# robot_navigation

## 1. 包定位

`robot_navigation` 是导航模块包，负责基于已有静态地图启动 Nav2。  
该包不维护世界、地图本体和 Gazebo 模型，而是专注于导航参数、导航 launch 和导航相关集成配置。

## 2. 主要职责

- Nav2 启动入口
- AMCL 定位相关参数
- planner / controller / behavior / waypoint 等 Nav2 参数
- 导航模块内部 launch 封装

## 3. 当前结构

```text
robot_navigation/
├── config/     Nav2 参数
├── launch/     Nav2 launch
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 4. 默认资源引用

- 默认地图：`robot_bringup/maps/map.yaml`
- 默认参数：`robot_navigation/config/nav2_params.yaml`

## 5. 目标流程

该包服务的最小可运行闭环如下：

1. Gazebo 启动
2. 机器人生成
3. 静态地图加载
4. AMCL 定位
5. RViz 中设置初始位姿
6. 发送导航目标
7. 机器人执行导航

## 6. 启动方式

推荐通过顶层脚本统一启动：

```bash
bash scripts/run_nav2.sh
```

如果需要直接验证 launch，可在工作区环境中执行：

```bash
ros2 launch robot_navigation navigation.launch.py
```

## 7. 维护建议

- 将导航参数和导航启动逻辑集中维护在本包
- 不在本包内重复复制地图文件
- 若后续接入动态避障、行为树扩展或任务级导航逻辑，应优先在本包周边扩展，而不是回写到 bringup
