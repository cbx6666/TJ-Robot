from __future__ import annotations

import os
import queue
import re
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _resolve_existing_wav_path(raw: str) -> Path | None:
    """解析 wav 绝对路径；在 Linux/WSL 下把 ``E:/...`` 映射为 ``/mnt/e/...``。"""
    s = raw.strip().replace("\\", "/")
    if not s:
        return None
    candidates: list[Path] = [Path(s)]
    if sys.platform != "win32":
        m = re.match(r"^([a-zA-Z]):/(.*)$", s)
        if m:
            drive, rest = m.group(1).lower(), m.group(2)
            candidates.append(Path(f"/mnt/{drive}/{rest}"))
    for p in candidates:
        if p.is_file():
            return p
    return None


class VoiceGatewayNode(Node):
    """语音入口：mock 周期文本、本地 faster-whisper 文件/麦克风转写、或预留 HTTP ASR。

    - mock：定时发布测试句（默认关闭；联调可开 enable_mock_input）。
    - whisper_file：订阅 ``transcribe_wav_path_topic``，对 16kHz+ 单声道 wav 路径做转写。
    - whisper_mic：本机默认麦克风 16kHz 单声道流式采集，静音切段后转写（需 sounddevice + faster-whisper）。
    - 简体倾向：参数 ``whisper_initial_prompt`` 默认简体提示语，传给 Whisper ``initial_prompt``（与 ``mic_whisper_language:=zh`` 配合）。
    - voice_api：若 ``voice_api_url`` 非空，当前仅占位日志（可自行接云 ASR）。
    """

    def __init__(self) -> None:
        super().__init__("voice_gateway")
        self.declare_parameter("output_topic", "/interaction/speech_text")
        self.declare_parameter("enable_mock_input", False)
        self.declare_parameter("mock_interval_sec", 30.0)
        self.declare_parameter("mock_text", "开始巡检房间")
        self.declare_parameter("voice_api_url", "")
        self.declare_parameter("voice_api_key_env", "VOICE_API_KEY")
        self.declare_parameter("asr_backend", "mock")
        self.declare_parameter("speech_text_log_path", "")
        self.declare_parameter("transcribe_wav_path_topic", "/interaction/transcribe_wav_path")
        self.declare_parameter("whisper_model_size", "base")
        self.declare_parameter("whisper_device", "auto")
        self.declare_parameter("whisper_compute_type", "int8")
        self.declare_parameter("whisper_download_root", "")
        self.declare_parameter(
            "whisper_initial_prompt",
            "请用简体中文输出，勿用繁体。",
        )
        self.declare_parameter("mic_sample_rate", 16000)
        self.declare_parameter("mic_device_index", -1)
        self.declare_parameter("mic_device_name", "")
        self.declare_parameter("mic_speech_rms_threshold", 0.01)
        self.declare_parameter("mic_silence_sec", 0.65)
        self.declare_parameter("mic_min_speech_sec", 0.28)
        self.declare_parameter("mic_block_ms", 32)
        self.declare_parameter(
            "mic_debug_rms",
            False,
        )
        self.declare_parameter("mic_whisper_language", "zh")
        self.declare_parameter("mic_whisper_vad_filter", False)
        self.declare_parameter("mic_input_gain", 5.0)
        self.declare_parameter("mic_whisper_no_speech_threshold", 0.42)
        self.declare_parameter("mic_preload_whisper", False)
        self.declare_parameter("mic_force_cpu_on_wsl", False)

        self._output_topic = str(self.get_parameter("output_topic").value)
        self._speech_log_path = str(self.get_parameter("speech_text_log_path").value).strip()
        self._pub = self.create_publisher(String, self._output_topic, 10)
        if self._speech_log_path:
            self.get_logger().info(f"[voice_gateway] 识别文本将追加写入文件: {self._speech_log_path}")
        self._api_url = str(self.get_parameter("voice_api_url").value).strip()
        api_key_env = str(self.get_parameter("voice_api_key_env").value).strip()
        self._api_key = os.environ.get(api_key_env, "")
        self._asr_backend = str(self.get_parameter("asr_backend").value).strip().lower()
        self._whisper_model: Any = None
        self._whisper_lock = threading.Lock()
        self._mic_stop: threading.Event | None = None
        self._mic_thread: threading.Thread | None = None
        self._mic_tx_thread: threading.Thread | None = None
        self._mic_segment_q: queue.Queue[tuple[Any, float]] | None = None
        self._mic_result_q: queue.Queue[tuple[str, str]] = queue.Queue()
        self._mic_sr = int(self.get_parameter("mic_sample_rate").value) or 16000
        name = str(self.get_parameter("mic_device_name").value).strip()
        idx = int(self.get_parameter("mic_device_index").value)
        self._mic_device: int | str | None
        if name:
            self._mic_device = name
        elif idx >= 0:
            self._mic_device = idx
        else:
            self._mic_device = None
        self._mic_rms_thresh = float(self.get_parameter("mic_speech_rms_threshold").value)
        self._mic_silence_sec = max(float(self.get_parameter("mic_silence_sec").value), 0.2)
        self._mic_min_speech_sec = max(float(self.get_parameter("mic_min_speech_sec").value), 0.12)
        self._mic_block_ms = max(int(self.get_parameter("mic_block_ms").value), 16)
        self._mic_debug_rms = bool(self.get_parameter("mic_debug_rms").value)
        lang_raw = str(self.get_parameter("mic_whisper_language").value).strip()
        self._mic_whisper_lang = lang_raw if lang_raw else None
        self._mic_whisper_vad = bool(self.get_parameter("mic_whisper_vad_filter").value)
        self._mic_input_gain = max(float(self.get_parameter("mic_input_gain").value), 0.1)
        self._mic_no_speech_thr = float(self.get_parameter("mic_whisper_no_speech_threshold").value)
        self._mic_preload_whisper = bool(self.get_parameter("mic_preload_whisper").value)
        self._mic_force_cpu_on_wsl = bool(self.get_parameter("mic_force_cpu_on_wsl").value)
        self._mic_dbg_last_log = 0.0
        self._whisper_download_root = str(self.get_parameter("whisper_download_root").value).strip()
        self._whisper_initial_prompt = str(self.get_parameter("whisper_initial_prompt").value).strip()

        if self._api_url:
            self.get_logger().info(
                f"[voice_gateway] voice_api_url={self._api_url} key_present={bool(self._api_key)} "
                "(HTTP ASR 占位：未实现 multipart 上传)"
            )

        if self._asr_backend == "whisper_file":
            tp = str(self.get_parameter("transcribe_wav_path_topic").value)
            self.create_subscription(String, tp, self._on_wav_path, 10)
            self.get_logger().info(
                f"[voice_gateway] ASR=whisper_file，订阅 wav 路径: {tp} -> 发布识别: {self._output_topic} "
                f"(需 pip install faster-whisper，见 requirements-voice.txt)"
            )
        elif self._asr_backend == "whisper_mic":
            self._mic_stop = threading.Event()
            self._mic_segment_q = queue.Queue(maxsize=6)
            self._mic_tx_thread = threading.Thread(
                target=self._mic_transcribe_worker_loop, daemon=True, name="whisper-mic-tx"
            )
            self._mic_tx_thread.start()
            self._mic_thread = threading.Thread(target=self._mic_whisper_loop, daemon=True)
            self._mic_thread.start()
            self.create_timer(0.05, self._flush_mic_transcript_queue)
            self.get_logger().info(
                f"[voice_gateway] ASR=whisper_mic，麦克风实时转写 -> {self._output_topic} "
                f"(需 pip install -r requirements-voice.txt；device={self._mic_device!r} "
                f"index={idx} name={name!r} "
                f"rms>={self._mic_rms_thresh} 静音>{self._mic_silence_sec}s 切段)"
            )
        elif self._asr_backend == "voice_api" and self._api_url:
            self.get_logger().warning("[voice_gateway] asr_backend=voice_api 尚未接具体协议，请扩展实现")
        elif self._asr_backend not in ("mock", "whisper_file", "whisper_mic", "voice_api", "none"):
            self.get_logger().warning(f"[voice_gateway] 未知 asr_backend={self._asr_backend!r}，回退 mock")

        enable_mock = bool(self.get_parameter("enable_mock_input").value)
        self._mock_text = str(self.get_parameter("mock_text").value)
        interval = max(float(self.get_parameter("mock_interval_sec").value), 5.0)
        if self._asr_backend == "none":
            enable_mock = False
        if enable_mock and self._asr_backend in ("mock", "voice_api", "whisper_file", "whisper_mic", "none"):
            self.create_timer(interval, self._publish_mock_text)
            self.get_logger().info(
                f"[voice_gateway] mock 定时发布已开: 每 {interval}s -> {self._output_topic} 文本={self._mock_text!r}"
            )
        else:
            self.get_logger().info(f"[voice_gateway] mock 关闭；输出话题 {self._output_topic}")
        if self._asr_backend == "mock" and not enable_mock:
            tp = str(self.get_parameter("transcribe_wav_path_topic").value)
            self.get_logger().info(
                f"[voice_gateway] 当前 ASR=mock 且无定时 mock；不会产生识别结果。"
                f" 本地转写可用 asr_backend=whisper_file（发 wav 路径到 {tp}）"
                f" 或 asr_backend=whisper_mic（本机麦克风）。"
            )

    def destroy_node(self) -> bool:
        if self._mic_stop is not None:
            self._mic_stop.set()
        if self._mic_thread is not None:
            self._mic_thread.join(timeout=6.0)
            self._mic_thread = None
        if self._mic_tx_thread is not None:
            self._mic_tx_thread.join(timeout=120.0)
            self._mic_tx_thread = None
        return super().destroy_node()

    def _flush_mic_transcript_queue(self) -> None:
        try:
            while True:
                text, meta = self._mic_result_q.get_nowait()
                self._emit_speech_text(text, source="whisper_mic", meta=meta)
        except queue.Empty:
            pass

    def _mic_transcribe_worker_loop(self) -> None:
        """单线程串行执行 Whisper，避免多 daemon 线程争 CUDA；队列在模型加载前即开始消费。"""
        self.get_logger().info(
            "[voice_gateway] whisper_mic 转写 worker 启动（串行），等待切段入队…"
            "（默认不预加载模型，避免 WSL 下 CUDA 在后台线程卡死；可设 mic_preload_whisper:=true）"
        )
        if self._mic_preload_whisper:
            self.get_logger().info("[voice_gateway] whisper_mic 预加载 Whisper（mic_preload_whisper:=true）…")
            try:
                self._ensure_whisper()
            except Exception as ex:
                self.get_logger().error(
                    f"[voice_gateway] whisper_mic 预加载 Whisper 异常: {ex}\n{traceback.format_exc()}"
                )
        q = self._mic_segment_q
        if q is None:
            return
        while not self._mic_stop.is_set():
            try:
                item = q.get(timeout=0.25)
            except queue.Empty:
                continue
            audio, thresh = item
            self._mic_transcribe_segment(audio, thresh)
        self.get_logger().info("[voice_gateway] whisper_mic 转写 worker 已退出")

    def _mic_transcribe_segment(self, audio: Any, thresh: float) -> None:
        """在独立线程中转写，避免阻塞麦克风采集（否则会 overflow 且永远无结果）。"""
        import numpy as np

        try:
            n = int(np.asarray(audio).shape[0]) if hasattr(audio, "shape") else 0
            dur = n / max(self._mic_sr, 1)
            self.get_logger().info(
                f"[voice_gateway] whisper_mic 后台转写开始: 样本数={n} 约{dur:.2f}s（Whisper 推理中，请稍候）"
            )
            model = self._ensure_whisper()
            if model is None:
                self.get_logger().error("[voice_gateway] whisper_mic 转写中止: Whisper 模型未就绪")
                return
            a = np.ascontiguousarray(np.asarray(audio, dtype=np.float32))
            peak = float(np.max(np.abs(a))) if a.size else 0.0
            a = np.clip(a * self._mic_input_gain, -1.0, 1.0)
            peak_g = float(np.max(np.abs(a))) if a.size else 0.0
            self.get_logger().info(
                f"[voice_gateway] whisper_mic 送识别: 原峰值={peak:.6f} 增益后峰值={peak_g:.5f} "
                f"x{self._mic_input_gain:.2f} 时长={a.shape[0] / max(self._mic_sr, 1):.2f}s"
            )
            kw: dict[str, Any] = {
                "beam_size": 5,
                "vad_filter": self._mic_whisper_vad,
                "no_speech_threshold": self._mic_no_speech_thr,
            }
            if self._mic_whisper_lang:
                kw["language"] = self._mic_whisper_lang
            if self._whisper_initial_prompt:
                kw["initial_prompt"] = self._whisper_initial_prompt
            t0 = time.monotonic()
            with self._whisper_lock:
                segments, _info = model.transcribe(a, **kw)
                text = "".join(s.text for s in segments).strip()
            dt = time.monotonic() - t0
            if text:
                preview = (text[:48] + "…") if len(text) > 48 else text
                self.get_logger().info(
                    f"[voice_gateway] whisper_mic 转写完成: 用时={dt:.2f}s 字数={len(text)} 预览={preview!r}"
                )
                self._mic_result_q.put((text, f"{dt:.2f}s, rms_thresh={thresh}"))
            else:
                self.get_logger().info(
                    f"[voice_gateway] whisper_mic 转写完成: 无有效文本（Whisper 已跑完，用时={dt:.2f}s，字数=0）。"
                    "可试: 说话更长更响、-p mic_input_gain:=10、-p mic_whisper_no_speech_threshold:=0.32"
                )
        except Exception as ex:
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 转写失败: {ex}\n{traceback.format_exc()}"
            )

    def _mic_whisper_loop(self) -> None:
        try:
            import numpy as np
            import sounddevice as sd
        except ImportError as e:
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 缺少依赖: {e}；请 pip install -r requirements-voice.txt"
            )
            return
        except OSError as e:
            self.get_logger().error(
                "[voice_gateway] sounddevice 需要系统库 PortAudio（与 pip 无关）。"
                "Ubuntu / Debian / WSL: sudo apt-get update && sudo apt-get install -y portaudio19-dev "
                f"安装后重试。原始错误: {e}"
            )
            return

        def _default_input_index() -> int | None:
            try:
                pair = sd.default.device
                if pair is None:
                    return None
                di = pair[0] if isinstance(pair, (list, tuple)) else pair
                if di is None:
                    return None
                i = int(di)
                return i if i >= 0 else None
            except Exception:
                return None

        def _log_input_devices() -> None:
            try:
                found: list[str] = []
                for i, d in enumerate(sd.query_devices()):
                    if int(d.get("max_input_channels", 0) or 0) > 0:
                        found.append(f"[{i}] {d.get('name', '')}")
                if found:
                    self.get_logger().info("[voice_gateway] 当前枚举到的输入设备: " + "; ".join(found))
                else:
                    self.get_logger().warning("[voice_gateway] 未枚举到任何输入通道>0 的音频设备。")
            except Exception as ex:
                self.get_logger().warning(f"[voice_gateway] 枚举音频设备失败: {ex}")

        use_device: int | str
        if self._mic_device is not None:
            use_device = self._mic_device
        else:
            di = _default_input_index()
            if di is None:
                _log_input_devices()
                self.get_logger().error(
                    "[voice_gateway] whisper_mic 无法打开麦克风：无有效默认输入设备（PortAudio 常为 device=-1）。"
                    "WSL 内通常没有转发笔记本麦克风，请在 Windows 原生终端运行本节点，或在 WSL 配置好 PulseAudio/管道后"
                    "执行: python3 -c \"import sounddevice as sd; print(sd.query_devices())\"，再用 -p mic_device_index:=<数字> 指定设备（-1 为默认）。"
                )
                return
            use_device = di
            self.get_logger().info(f"[voice_gateway] whisper_mic 使用默认输入设备 index={use_device}")

        sr = self._mic_sr
        block = max(int(sr * self._mic_block_ms / 1000.0), 128)
        silence_samples_need = int(sr * self._mic_silence_sec)
        min_samples = int(sr * self._mic_min_speech_sec)
        thresh = self._mic_rms_thresh

        buf: list[Any] = []
        silence_run = 0
        had_voice = False

        try:
            stream_kw: dict[str, Any] = {
                "samplerate": sr,
                "channels": 1,
                "dtype": "float32",
                "blocksize": block,
                "device": use_device,
            }
            with sd.InputStream(**stream_kw) as stream:
                self.get_logger().info("[voice_gateway] whisper_mic 麦克风已打开，请说话（说完停顿约 0.6s 触发识别）")
                while self._mic_stop and not self._mic_stop.is_set():
                    data, overflowed = stream.read(block)
                    if overflowed:
                        self.get_logger().warning("[voice_gateway] whisper_mic 输入溢出，可能丢字")
                    chunk = np.asarray(data, dtype=np.float32).reshape(-1)
                    rms = float(np.sqrt(np.mean(chunk * chunk))) if chunk.size else 0.0
                    if self._mic_debug_rms:
                        bs_dbg = sum(x.size for x in buf)
                        if had_voice or bs_dbg > 0:
                            now = time.monotonic()
                            if now - self._mic_dbg_last_log >= 0.5:
                                self._mic_dbg_last_log = now
                                self.get_logger().info(
                                    f"[voice_gateway] whisper_mic dbg rms={rms:.5f} thr={thresh} "
                                    f"had_voice={had_voice} buf_samples={bs_dbg}"
                                )
                    loud = rms >= thresh
                    if loud:
                        had_voice = True
                        silence_run = 0
                        buf.append(chunk)
                    else:
                        if had_voice:
                            buf.append(chunk)
                            silence_run += int(chunk.size)
                            if silence_run >= silence_samples_need:
                                audio = np.concatenate(buf).astype(np.float32, copy=False)
                                buf.clear()
                                had_voice = False
                                silence_run = 0
                                if audio.size < min_samples:
                                    continue
                                self.get_logger().info(
                                    f"[voice_gateway] whisper_mic 切段 {audio.size / sr:.2f}s，已入队转写…"
                                )
                                q = self._mic_segment_q
                                if q is not None:
                                    try:
                                        q.put_nowait((audio, thresh))
                                    except queue.Full:
                                        self.get_logger().warning(
                                            "[voice_gateway] whisper_mic 转写队列满，丢弃本段（请等上一段转写完成）"
                                        )
        except Exception as e:
            _log_input_devices()
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 麦克风线程退出: {e}。"
                "若含 device -1：多为无可用麦克风；WSL 建议换 Windows 运行或配置音频后设置 mic_device_index / mic_device_name。"
            )

    def _append_speech_log_file(self, text: str) -> None:
        if not self._speech_log_path:
            return
        line = f"{datetime.now().isoformat(timespec='seconds')}\t{text}\n"
        try:
            Path(self._speech_log_path).parent.mkdir(parents=True, exist_ok=True)
            with open(self._speech_log_path, "a", encoding="utf-8") as f:
                f.write(line)
        except OSError as e:
            self.get_logger().warning(f"[voice_gateway] 写入识别文本日志失败: {e}")

    def _emit_speech_text(self, text: str, *, source: str, meta: str = "") -> None:
        t = (text or "").strip()
        if not t:
            return
        self._pub.publish(String(data=t))
        extra = f" ({meta})" if meta else ""
        self.get_logger().info(f"[voice_gateway] 识别文本[{source}]{extra} -> {self._output_topic}: {t!r}")
        self._append_speech_log_file(t)

    def _ensure_whisper(self) -> Any:
        if self._whisper_model is not None:
            return self._whisper_model
        try:
            from faster_whisper import WhisperModel
        except ImportError as e:
            self.get_logger().error(
                f"[voice_gateway] 未安装 faster-whisper: {e}；请 pip install -r requirements-voice.txt"
            )
            return None
        with self._whisper_lock:
            if self._whisper_model is not None:
                return self._whisper_model
            size = str(self.get_parameter("whisper_model_size").value).strip() or "base"
            dev = str(self.get_parameter("whisper_device").value).strip() or "auto"
            ctype = str(self.get_parameter("whisper_compute_type").value).strip() or "int8"
            if dev == "auto":
                try:
                    import torch

                    dev = "cuda" if torch.cuda.is_available() else "cpu"
                except ImportError:
                    dev = "cpu"
            if (
                self._asr_backend == "whisper_mic"
                and self._mic_force_cpu_on_wsl
                and dev == "cuda"
            ):
                try:
                    with open("/proc/version", encoding="utf-8") as vf:
                        is_wsl = "microsoft" in vf.read().lower()
                except OSError:
                    is_wsl = False
                if is_wsl:
                    self.get_logger().warning(
                        "[voice_gateway] mic_force_cpu_on_wsl:=true 且检测到 WSL："
                        "为避免部分环境下 CUDA 在后台线程初始化失败，已改用 CPU。"
                        "若你确认 GPU 可用，请使用默认 mic_force_cpu_on_wsl:=false 并设 whisper_device:=cuda。"
                    )
                    dev = "cpu"
            elif self._asr_backend == "whisper_mic" and dev == "cuda":
                try:
                    with open("/proc/version", encoding="utf-8") as vf:
                        is_wsl = "microsoft" in vf.read().lower()
                except OSError:
                    is_wsl = False
                if is_wsl:
                    self.get_logger().info(
                        "[voice_gateway] WSL 下使用 CUDA 加载 Whisper；若长时间无「模型就绪」，"
                        "多为首次下载权重或驱动问题，可先运行 scripts/prep_faster_whisper.py，"
                        "或临时 -p mic_force_cpu_on_wsl:=true -p whisper_device:=cpu。"
                    )
            # CPU + int8 在部分 WSL/ctranslate2 上会导致 WhisperModel() 永不返回，见不到「模型就绪」
            if dev == "cpu" and ctype.lower() in ("int8", "int8_float32", "int8_bfloat16"):
                self.get_logger().warning(
                    f"[voice_gateway] device=cpu 时将 compute_type 从 {ctype} 改为 float32，"
                    "避免 WhisperModel 构造卡死；若需省内存可手动试 whisper_compute_type:=default"
                )
                ctype = "float32"
            try:
                from huggingface_hub.constants import HF_HUB_CACHE

                self.get_logger().info(f"[voice_gateway] HuggingFace 默认缓存目录: {HF_HUB_CACHE}")
            except Exception:
                pass
            if self._whisper_download_root:
                self.get_logger().info(
                    f"[voice_gateway] whisper_download_root={self._whisper_download_root!r}（权重将放于此）"
                )
            self.get_logger().info(f"[voice_gateway] 加载 WhisperModel size={size} device={dev} compute={ctype}")
            self.get_logger().info(
                "[voice_gateway] 正在构造 WhisperModel…（权重与 prep 时相同，Hub 上无单独的「GPU int8 包」；"
                "若已 prep 仍长时间无输出，多为本机 device/compute 初始化——WSL+CUDA 常见卡住，"
                "可另终端用相同参数运行 scripts/prep_faster_whisper.py 热身，或 whisper_device:=cpu）"
            )
            t0 = time.monotonic()
            try:
                if self._whisper_download_root:
                    try:
                        self._whisper_model = WhisperModel(
                            size, device=dev, compute_type=ctype, download_root=self._whisper_download_root
                        )
                    except TypeError:
                        self._whisper_model = WhisperModel(size, device=dev, compute_type=ctype)
                else:
                    self._whisper_model = WhisperModel(size, device=dev, compute_type=ctype)
            except Exception as ex:
                self.get_logger().error(
                    f"[voice_gateway] WhisperModel 构造失败: {ex}\n{traceback.format_exc()}"
                )
                return None
            self.get_logger().info(f"[voice_gateway] Whisper 模型就绪，耗时 {time.monotonic() - t0:.2f}s")
            return self._whisper_model

    def _on_wav_path(self, msg: String) -> None:
        path = (msg.data or "").strip()
        if not path:
            return
        p = _resolve_existing_wav_path(path)
        if p is None:
            self.get_logger().warning(
                f"[voice_gateway] 转写跳过：文件不存在 {path!r}"
                f"（在 WSL 下请改用 /mnt/e/... 或确保文件已生成）"
            )
            return
        model = self._ensure_whisper()
        if model is None:
            return
        try:
            t0 = time.monotonic()
            tw_kw: dict[str, Any] = {"beam_size": 5, "vad_filter": True}
            if self._mic_whisper_lang:
                tw_kw["language"] = self._mic_whisper_lang
            if self._whisper_initial_prompt:
                tw_kw["initial_prompt"] = self._whisper_initial_prompt
            with self._whisper_lock:
                segments, _info = model.transcribe(str(p.resolve()), **tw_kw)
            text = "".join(s.text for s in segments).strip()
            dt = time.monotonic() - t0
            if not text:
                self.get_logger().warning(f"[voice_gateway] 转写结果为空: {path} ({dt:.2f}s)")
                return
            self._emit_speech_text(text, source="whisper_file", meta=f"{dt:.2f}s, wav={path}")
        except Exception as e:
            self.get_logger().error(f"[voice_gateway] 转写失败: {e}")

    def _publish_mock_text(self) -> None:
        self._emit_speech_text(self._mock_text, source="mock_timer")


def main() -> None:
    rclpy.init()
    node = VoiceGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
