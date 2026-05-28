# robot_rviz_plugins

RViz2 调试面板。

## Sim Speech（模拟语音）

- 在 RViz 中：**Panels → Add Panel → robot_rviz_plugins / SimSpeech**（`test1.rviz` 已默认加入）。
- 预设：房间巡检、拿水杯、拿花瓶、停止；可编辑自定义句后点 **发送 → LLM**。
- 发布话题：`/interaction/speech_text`（与 `llm_router` 订阅一致）。

构建：

```bash
cd ros_ws && colcon build --packages-select robot_rviz_plugins
source install/setup.bash
```

未构建时 RViz 可能提示找不到面板；可改用 `enable_sim_speech_gui:=true` 启动 tkinter 后备窗口。
