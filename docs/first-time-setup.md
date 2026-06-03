# 首次安装与环境准备

面向在本机第一次克隆仓库、需要搭好 ROS 2、构建工作区，以及可选配置 LLM / 本地语音的开发者。

---

## 1. 系统与终端

- **推荐环境**：**Ubuntu 22.04**（与 ROS 2 **Humble** 对应）。在 **WSL2** 中安装相同版本 Ubuntu 亦可；仓库路径在 WSL 侧一般为 `/mnt/<盘符>/...`。
- 顶层 `scripts/*.sh` 为 **Bash**。在 Windows 上请用 **WSL / Git Bash / MSYS** 执行；纯 PowerShell 不直接执行这些脚本。
- 本仓库脚本默认 ROS 安装路径为 **`/opt/ros/humble/setup.bash`**（可通过环境变量 `ROS_SETUP_BASH` 覆盖）。

---

## 2. ROS 2、Gazebo 与常用依赖（需管理员）

在未配置过 ROS 2 apt 源的机器上，可使用仓库脚本安装常用包（**需 sudo**，耗时较长）：

```bash
sudo bash scripts/setup_env.sh
```

该脚本会添加 `packages.ros.org` 源，并安装 `ros-humble-desktop`、`gazebo`、`python3-colcon-common-extensions`、`turtlebot3_gazebo`、`slam_toolbox` 等。若你已按官方文档安装好 **Humble** 及上述组件，可跳过此步。

---

## 3. 克隆后的第一次构建

在**仓库根目录**执行：

```bash
bash scripts/build.sh
```

`build.sh` 在 `ros_ws/` 下执行 `colcon build --symlink-install`。完成后请在新终端中加载工作区（若未通过脚本自动 source）：

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
```

说明：`scripts/run_*.sh` 会通过 `scripts/common.sh` 自动加载 ROS 与已构建的 `install`；若你**自行**运行 `ros2 launch` / `ros2 run` 且未先执行这些脚本，则需要手动执行上述两条 `source`。

---

## 4. LLM 配置（语音全链路 / `run_full_system` 常用）

**勿将 API Key 提交到 Git。** 仓库根目录已忽略 `local_llm.env`。

```bash
cp local_llm.env.example local_llm.env
# 编辑 local_llm.env，填入 TJ_LLM_API_KEY；按需修改 TJ_LLM_API_URL、TJ_LLM_MODEL
```

`bash scripts/run_full_system.sh` 会在启动前自动 `source` 同目录下的 `local_llm.env`（若文件存在）。在 Windows PowerShell 下仅使用 `ros2 launch` 时，请参考 `local_llm.env.example` 中的环境变量示例。

---

## 5. 本地语音识别（可选）

当 `voice_gateway` 使用 `asr_backend:=whisper_mic` 或 `whisper_file` 时，需要 Python 依赖：

```bash
pip install -r ros_ws/src/robot_interaction/requirements-voice.txt
```

在 Linux / WSL 下使用麦克风（`whisper_mic`）还需系统库 **PortAudio**：

```bash
sudo apt-get update && sudo apt-get install -y portaudio19-dev
```

节点默认 **`whisper_device=auto`**（有 NVIDIA 则用 **cuda**）、**`whisper_compute_type=int8`**，与 `voice_gateway` 一致。首次运行前建议预拉取并热身：

```bash
python3 scripts/prep_faster_whisper.py --hf-endpoint https://hf-mirror.com
# 默认即 --device auto --compute-type int8；无 GPU 时会自动 cpu+float32
```

在项目根目录执行；更多说明见 `scripts/prep_faster_whisper.py` 顶部文档字符串。

### WSL 麦克风（`whisper_mic`）

WSL **没有**独立声卡，要靠 **WSLg** 把 Windows 麦克风经 PulseAudio 转给 Linux。新开终端若未设置 `PULSE_SERVER`，`sounddevice` 常会报「未枚举到任何输入设备」——**不是**每次都要手打一长串命令，建议 **一次性** 写入 `~/.bashrc`：

```bash
# TJ-Robot WSL 麦克风（按实际路径保留其一即可）
[ -e /mnt/wslg/runtime-dir/pulse/native ] && export PULSE_SERVER=unix:/mnt/wslg/runtime-dir/pulse/native
```

`bash scripts/run_full_system_real_mic.sh` / `run_voice_llm_only.sh` 会自动 `source scripts/lib/wsl_pulse_env.sh`。

仍无设备时请检查：Windows **设置 → 隐私 → 麦克风** 已允许「适用于 Linux 的 Windows 子系统」；WSL 为 **WSL2 + 带 GUI**（WSLg）；不要用纯 SSH 进 WSL 跑语音。

### WSL 麦克风 / Pulse 僵死（`import sounddevice` 卡住、`Pa_Initialize` 无返回）

WSLg 把 Windows 麦克风经 **PulseAudio** 转给 Linux；该服务在 **整个 WSL 会话** 里只有一份。若 `voice_gateway` 被强杀、或只执行 `tb3_stack stop` 而未停语音，Pulse 可能进入僵死状态：此时在 WSL 里 `python3 -c "import sounddevice"` 会一直卡住，`pkill` / `unset PULSE_SERVER` **也无效**。

**可靠恢复**（与「关闭所有 WSL 窗口并重启 Cursor」相同）：

```powershell
# Windows PowerShell（管理员更稳妥）
wsl --shutdown
```

然后重新打开 Cursor / WSL，再执行 `bash scripts/run_voice_llm_only.sh` 验证。

**预防：**

```bash
bash scripts/kill_simulation_stack.sh   # 含语音栈 + 仿真，不要只 tb3_stack stop
```

环境噪导致频繁误「切段」时，可提高 RMS 门限（默认 **0.02**，最短有效语音 **0.38s**）：

```bash
export TJ_MIC_SPEECH_RMS_THRESHOLD=0.03
bash scripts/run_full_system_real_mic.sh
```

语音异常时可查看 `data/logs/full_system/voice_gateway_health.jsonl` 与 `task_pipeline.launch.log`。

**无麦克风联调全链路**（默认已开启，RViz 模拟语音）：

```bash
bash scripts/run_full_system.sh
# RViz 面板 Sim Speech → 发送 → LLM
```

旧 mock 定时语句：`export TJ_SIM_SPEECH_UI=0 TJ_FULL_SYSTEM_ASR=mock` 后 `enable_mock_voice:=true`。

或对已有 wav：`asr_backend:=whisper_file`，向 `/interaction/transcribe_wav_path` 发绝对路径。

### 全链路 `scripts/run_full_system.sh` 约定

- **相机 / YOLO 话题**：与 `run_simulation.sh` 相同，由 `scripts/lib/tb3_sim_assist_env.sh` 导出；一体 RGB-D 默认 `YOLO_IMAGE_TOPIC=/camera/image_raw`（非 `/camera/rgb/*`），深度 `/camera/depth/image_raw`。
- **全屋巡检**：语音 `start_room_patrol` → `/task/events` 的 `patrol_start` → **`patrol_waypoints`（NavigationManager）**：地图覆盖航点、卡死脱困、失败走廊黑名单（与 `run_nav2.sh` 同源，非旧 `patrol_smallhouse`）。
- **RViz**：与仿真栈共用 `ros_ws/src/robot_bringup/config/test1.rviz`（含 YOLO 图像/目标点/Marker 等），并在其中合并 **Nav2 的 `GoalTool`（2D Nav Goal）** 与 **「Navigation 2」** 侧栏，便于发 `NavigateToPose`，无需再切换到 `nav2_default_view.rviz`。
- **语音识别**：若启动参数里未写 `asr_backend:=...`，脚本会注入 `asr_backend:=${TJ_FULL_SYSTEM_ASR:-whisper_mic}`（默认麦克风）。无麦克风或不想装依赖时，可 `export TJ_FULL_SYSTEM_ASR=mock` 并传入 `enable_mock_voice:=true`，或自行传 `asr_backend:=whisper_file`。

---

## 下一步

构建与环境就绪后，回到仓库根目录 [README.md](../README.md) 中的「快速运行」启动仿真或全链路。
