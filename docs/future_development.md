# 未来开发路线

## ASR

在 `robot_perception/speech/asr_node.py` 接入语音识别，输出 `/interaction/speech_text`。

## NLU 与交互

扩展 `robot_interaction/nlu/intent_parser.py`，支持导航、搜索、建图、巡航、停止等 intent。

## TTS

在 `robot_interaction/tts/tts_node.py` 接入任务确认、失败提示和搜索结果播报。

## 地图语义与后处理

- `robot_mapping/semantic/`：人物区域、动态对象、语义标记。
- `robot_mapping/strip/`：person-free 地图清理和 occupancy map refinement。

## 屋内搜索

目标流程：

```text
task input
-> room_navigation
-> candidate search point
-> YOLO object_detector
-> found/not found
-> next point
-> result
```

## 真实机器人迁移

替换 `robot_perception/lidar`、`robot_perception/vision` 和 `robot_navigation` 的仿真适配，复用上层 `robot_mapping`、`robot_tasks`、`robot_interaction` 和 `robot_experiments`。
