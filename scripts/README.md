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
  - 启动基于静态地图的 Nav2 导航链路
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

### 3.5 停止仿真栈

```bash
bash scripts/tb3_stack.sh stop
```

## 4. 使用原则

- 不在 `scripts/` 中沉淀节点级算法实现
- 不在 `scripts/` 中长期维护复杂 Python 业务逻辑
- 所有标准运行入口应优先统一到这里，避免每次手工拼装 ROS 命令
