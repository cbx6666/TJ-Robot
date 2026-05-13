# scripts

## 1. 目录定位

`scripts/` 用于保存仓库级 shell 脚本。  
这些脚本负责环境准备、工作区构建和系统启动编排，不承载长期维护的核心业务逻辑。

## 2. 脚本分类

### 2.1 环境与构建

- `setup_env.sh`
  - 安装 ROS 2 Humble、Gazebo、TurtleBot3 及常用依赖
- `build.sh`
  - 编译 `ros_ws/`
- `clean_outputs.sh`
  - 清理输出目录
- `common.sh`
  - 共享路径与环境辅助函数

### 2.2 仿真主栈

- `tb3_stack.sh`
  - TurtleBot3 仿真主栈脚本
  - 负责 Gazebo、机器人生成、GUI 以及可选功能链路的启动与停止

### 2.3 运行入口

- `run_simulation.sh`
  - 启动默认仿真链路
- `run_mapping.sh`
  - 启动建图链路
- `run_nav2.sh`
  - 统一启动基于静态地图的 Nav2 导航链路
  - 默认启动 Gazebo 底盘、Nav2、单个 RViz 和自动巡视节点
- `run_voice_demo.sh`
  - 启动语音交互演示链路
- `run_search_task.sh`
  - 启动任务演示链路
- `run_full_system.sh`
  - 启动完整系统链路
- `bootstrap.sh`
  - 环境准备与快速自检入口

## 3. 常用命令

### 3.1 编译

```bash
bash scripts/build.sh
```

### 3.2 启动默认仿真

```bash
bash scripts/run_simulation.sh
```

### 3.3 启动建图

```bash
bash scripts/run_mapping.sh
```

### 3.4 启动 Nav2

```bash
bash scripts/run_nav2.sh
```

常用开关：

```bash
PLANNER_TYPE=dijkstra bash scripts/run_nav2.sh  # 切换 Dijkstra
TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh      # 只开 Nav2，手动点目标
TB3_ENABLE_RVIZ=0 bash scripts/run_nav2.sh      # 后台自动巡视，不开 RViz
```

`run_nav2.sh` 中 RViz 只由 Nav2 launch 启动；底盘栈里的 RViz 会被关闭，避免双 RViz。
Nav2 launch 会先启动 `map_server + AMCL`，再延迟启动 planner/controller，避免导航节点在 `map -> odom` 还没建立时卡住。
运行中的导航优先用 `Ctrl+C` 停止；异常残留时再执行 `bash scripts/tb3_stack.sh stop`。

### 3.5 停止仿真栈

```bash
bash scripts/tb3_stack.sh stop
```

## 4. 使用原则

- 不在 `scripts/` 中沉淀节点级算法实现
- 不在 `scripts/` 中长期维护复杂 Python 业务逻辑
- 所有标准运行入口应优先统一到这里，避免每次手工拼装 ROS 命令
