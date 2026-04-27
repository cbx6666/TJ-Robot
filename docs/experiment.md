# 实验设计

## baseline

目标：建立不使用 YOLO/person strip 的普通激光 SLAM 基线。

流程：

```text
Gazebo/TurtleBot3
-> SLAM Toolbox
-> map_saver
-> data/maps/raw
-> data/logs/baseline_mapping
-> data/results
```

运行：

```bash
bash scripts/run_baseline_mapping.sh
bash scripts/save_map.sh raw
```

## semantic mapping

目标：保留完整 SLAM 过程，同时记录人物区域，建图后通过 strip 去除动态人物造成的占用。

流程：

```text
Gazebo/TurtleBot3
-> SLAM Toolbox
-> YOLO/person detection
-> person region accumulation
-> before_strip map
-> strip post-processing
-> after_strip map
-> logs/results
```

运行：

```bash
bash scripts/run_semantic_mapping.sh
bash scripts/save_map.sh before_strip
bash scripts/strip_map.sh data/maps/before_strip/map_xxx.yaml ~/.ros/tj_person_strip_regions.yaml
```

## before_strip

`before_strip` 是语义实验的原始 SLAM 地图，保存到：

```text
data/maps/before_strip
```

它应和实验参数、YOLO 模型、person region 文件一起记录。

## after_strip

`after_strip` 是后处理输出，保存到：

```text
data/maps/after_strip
```

输出包括 `.yaml` 和 `.pgm`。后续可加入 strip report。

## 参数冻结

每次实验应记录：

- `config/experiment.yaml`
- `config/slam.yaml`
- `config/yolo.yaml`
- `config/topics.yaml`
- 当前 git commit
- 启动脚本环境变量

## 日志记录

默认日志根目录：

```text
data/logs
```

建议按实验分目录：

- `data/logs/baseline_mapping`
- `data/logs/semantic_mapping`
- `data/logs/navigation`
- `data/logs/search`

## 输出结果

建议结果写入：

```text
data/results/<experiment_name>/
```

包括地图质量、导航成功率、搜索成功率、运行时间、失败原因。

## 可复现性

一个实验记录至少包含：

- 运行脚本。
- 配置快照。
- 输入地图或 world。
- 输出地图。
- person regions。
- 日志。
- 指标 JSON/CSV。
