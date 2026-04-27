# scripts

脚本层只负责环境准备、构建、启动、保存、后处理和清理，不承载核心算法逻辑。

## 基础脚本

| 文件 | 作用 |
|---|---|
| `setup_env.sh` | 安装 ROS 2 Humble、Gazebo、TurtleBot3、SLAM Toolbox、Nav2 等系统依赖。 |
| `build.sh` | 编译 `ros_ws`。 |
| `common.sh` | 新脚本共享的项目根目录、ROS source、输出目录准备函数。 |
| `tb3_stack.sh` | 兼容主栈，负责当前稳定的 Gazebo/TurtleBot3/SLAM/RViz/YOLO 编排。 |

## 运行入口

| 文件 | 作用 |
|---|---|
| `run_simulation.sh` | 启动基础仿真。 |
| `run_baseline_mapping.sh` | 启动普通激光 SLAM baseline。 |
| `run_semantic_mapping.sh` | 启动 YOLO/person region 语义辅助建图。 |
| `run_navigation.sh` | 启动 Nav2 wrapper。 |
| `run_voice_demo.sh` | 启动语音交互预留入口。 |
| `run_search_task.sh` | 启动屋内搜索预留入口。 |
| `run_full_system.sh` | 启动完整系统预留入口。 |

## 数据脚本

| 文件 | 作用 |
|---|---|
| `save_map.sh` | 调用 `nav2_map_server map_saver_cli` 保存 `/map` 到 `data/maps/<kind>`。 |
| `strip_map.sh` | 调用 `strip_saved_map_person_free` 输出 after_strip 地图。 |
| `clean_outputs.sh` | 清理 `data/maps/*`、`data/logs`、`data/results` 下的生成文件。 |

## 常用命令

```bash
bash scripts/build.sh
bash scripts/run_baseline_mapping.sh
bash scripts/save_map.sh raw

bash scripts/run_semantic_mapping.sh
bash scripts/save_map.sh before_strip
bash scripts/strip_map.sh data/maps/before_strip/map_xxx.yaml ~/.ros/tj_person_strip_regions.yaml
```

停止仿真：

```bash
bash scripts/tb3_stack.sh stop
```
