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

节点默认 Whisper 规模为 **base**（参数 `whisper_model_size`）。首次运行前建议预拉取权重；**国内镜像一行示例**（与常见 GPU + int8 用法一致，可按机器改为 `cpu` + `float32`）：

```bash
python3 scripts/prep_faster_whisper.py --hf-endpoint https://hf-mirror.com --size base --device cuda --compute-type int8
```

在项目根目录执行；更多说明见 `scripts/prep_faster_whisper.py` 顶部文档字符串。

---

## 下一步

构建与环境就绪后，回到仓库根目录 [README.md](../README.md) 中的「快速运行」启动仿真或全链路。
