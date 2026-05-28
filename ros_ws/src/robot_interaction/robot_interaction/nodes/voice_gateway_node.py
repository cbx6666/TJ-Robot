from __future__ import annotations

import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _is_wsl() -> bool:
    try:
        with open("/proc/version", encoding="utf-8") as vf:
            return "microsoft" in vf.read().lower()
    except OSError:
        return False


def _wsl_pulse_socket_candidates() -> list[str]:
    cands = [
        "/mnt/wslg/runtime-dir/pulse/native",
        "/mnt/wslg/PulseServer",
        "/mnt/wslg/pulse/native",
    ]
    xdg = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if xdg:
        cands.append(os.path.join(xdg, "pulse", "native"))
    return [p for p in cands if os.path.exists(p)]


def _pulse_server_alive() -> bool:
    if not os.environ.get("PULSE_SERVER"):
        return False
    import subprocess

    try:
        subprocess.run(
            ["pactl", "info"],
            capture_output=True,
            timeout=4,
            check=True,
        )
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def _apply_wsl_pulse_env() -> str | None:
    """WSLg：设置可用的 PULSE_SERVER；套接字存在但 Pulse 未响应时不强绑（避免枚举为空）。"""
    if not _is_wsl():
        return os.environ.get("PULSE_SERVER")
    for cand in _wsl_pulse_socket_candidates():
        os.environ["PULSE_SERVER"] = f"unix:{cand}"
        if _pulse_server_alive():
            return os.environ["PULSE_SERVER"]
    cur = os.environ.get("PULSE_SERVER", "").strip()
    if cur and _pulse_server_alive():
        return cur
    os.environ.pop("PULSE_SERVER", None)
    return None


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
    - 中文：``mic_whisper_language:=zh``；可选 ``whisper_initial_prompt``（默认空，避免静音时幻听提示语）。
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
        self.declare_parameter("whisper_initial_prompt", "")
        self.declare_parameter("mic_sample_rate", 16000)
        self.declare_parameter("mic_device_index", -1)
        self.declare_parameter("mic_device_name", "")
        self.declare_parameter("mic_speech_rms_threshold", 0.02)
        self.declare_parameter("mic_silence_sec", 0.65)
        self.declare_parameter("mic_min_speech_sec", 0.38)
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
        self.declare_parameter("whisper_preload_at_startup", True)
        self.declare_parameter("mic_open_retry_sec", 120.0)
        self.declare_parameter("mic_force_cpu_on_wsl", False)
        self.declare_parameter("mic_health_log_path", "")
        self.declare_parameter("mic_health_interval_sec", 5.0)
        self.declare_parameter("mic_stall_alert_sec", 4.0)

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
        self._mic_stream: Any = None
        self._mic_stream_lock = threading.Lock()
        self._mic_health_lock = threading.Lock()
        self._mic_last_read_mono = 0.0
        self._mic_last_rms = 0.0
        self._mic_peak_rms = 0.0
        self._mic_read_count = 0
        self._mic_overflow_count = 0
        self._mic_segment_count = 0
        self._mic_stream_open = False
        self._mic_thread_alive = False
        self._mic_stall_incident_sent = False
        self._mic_health_watchdog_thread: threading.Thread | None = None
        self._mic_health_log_path = str(self.get_parameter("mic_health_log_path").value).strip()
        if not self._mic_health_log_path:
            self._mic_health_log_path = os.environ.get("TJ_VOICE_HEALTH_LOG", "").strip()
        self._mic_health_interval_sec = max(
            float(self.get_parameter("mic_health_interval_sec").value), 2.0
        )
        self._mic_stall_alert_sec = max(float(self.get_parameter("mic_stall_alert_sec").value), 1.5)
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
        self._whisper_preload_at_startup = bool(self.get_parameter("whisper_preload_at_startup").value)
        self._mic_open_retry_sec = max(float(self.get_parameter("mic_open_retry_sec").value), 10.0)
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
            self.get_logger().info(
                f"[voice_gateway] ASR=whisper_mic，本机麦克风实时转写 -> {self._output_topic} "
                f"(说完停顿约 {self._mic_silence_sec}s 触发识别；device={self._mic_device!r} "
                f"index={idx} name={name!r}，未指定时自动选 Pulse/默认输入；"
                f"rms>={self._mic_rms_thresh})"
            )
            self._mic_stop = threading.Event()
            self._mic_segment_q = queue.Queue(maxsize=6)
            # 与单独 interaction 一致：先开麦/入队，Whisper 可后台加载；切段在模型未就绪时会等 _ensure_whisper
            self._mic_tx_thread = threading.Thread(
                target=self._mic_transcribe_worker_loop, daemon=True, name="whisper-mic-tx"
            )
            self._mic_tx_thread.start()
            self._mic_thread = threading.Thread(
                target=self._mic_whisper_loop, daemon=True, name="whisper-mic-capture"
            )
            self._mic_thread.start()
            self._mic_health_watchdog_thread = threading.Thread(
                target=self._mic_health_watchdog_loop,
                daemon=True,
                name="whisper-mic-health",
            )
            self._mic_health_watchdog_thread.start()
            if self._mic_health_log_path:
                self.get_logger().info(
                    f"[voice_gateway] 健康诊断 JSONL: {self._mic_health_log_path} "
                    f"(间隔 {self._mic_health_interval_sec}s；read 停滞 >{self._mic_stall_alert_sec}s 写 incident)"
                )
            self.create_timer(0.05, self._flush_mic_transcript_queue)
            if self._whisper_preload_at_startup:

                def _bg_preload_whisper() -> None:
                    try:
                        self._ensure_whisper()
                    except Exception as ex:
                        self.get_logger().error(
                            f"[voice_gateway] 后台预加载 Whisper 异常: {ex}\n{traceback.format_exc()}"
                        )

                self.get_logger().info(
                    "[voice_gateway] 麦克风与转写 worker 已启动；Whisper 在后台加载"
                    "（模型未就绪时切段会先入队，就绪后再转写，与单独 launch 行为一致）…"
                )
                threading.Thread(target=_bg_preload_whisper, daemon=True, name="whisper-preload").start()
            else:
                self.get_logger().info(
                    "[voice_gateway] whisper_preload_at_startup:=false：首段语音触发加载 Whisper，"
                    "此前切段将阻塞在转写 worker 直至模型就绪"
                )
        elif self._asr_backend == "whisper_file" and self._whisper_preload_at_startup:
            self.get_logger().info(
                "[voice_gateway] 主线程预加载 Whisper（whisper_file）…"
            )
            self._ensure_whisper()
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

    @staticmethod
    def _utc_iso() -> str:
        return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")

    def _probe_pulse_ok(self) -> bool | None:
        if not os.environ.get("PULSE_SERVER"):
            return None
        try:
            subprocess.run(
                ["pactl", "info"],
                capture_output=True,
                timeout=2,
                check=True,
            )
            return True
        except (OSError, subprocess.SubprocessError):
            return False

    def _mic_note_read(self, rms: float) -> None:
        with self._mic_health_lock:
            self._mic_last_read_mono = time.monotonic()
            self._mic_last_rms = rms
            self._mic_peak_rms = max(self._mic_peak_rms, rms)
            self._mic_read_count += 1

    def _mic_note_overflow(self) -> None:
        with self._mic_health_lock:
            self._mic_overflow_count += 1

    def _mic_note_segment(self) -> None:
        with self._mic_health_lock:
            self._mic_segment_count += 1

    def _mic_set_stream_open(self, open_: bool) -> None:
        with self._mic_health_lock:
            self._mic_stream_open = open_
            if open_:
                self._mic_peak_rms = 0.0

    def _mic_set_thread_alive(self, alive: bool) -> None:
        with self._mic_health_lock:
            self._mic_thread_alive = alive

    def _collect_mic_health_snapshot(self, *, event: str = "periodic") -> dict[str, Any]:
        now = time.monotonic()
        with self._mic_health_lock:
            last_read = self._mic_last_read_mono
            snap = {
                "ts": self._utc_iso(),
                "event": event,
                "stream_open": self._mic_stream_open,
                "mic_thread_alive": self._mic_thread_alive,
                "read_count": self._mic_read_count,
                "overflow_count": self._mic_overflow_count,
                "segment_count": self._mic_segment_count,
                "last_rms": round(self._mic_last_rms, 6),
                "peak_rms": round(self._mic_peak_rms, 6),
                "rms_threshold": self._mic_rms_thresh,
                "read_stall_sec": round(max(0.0, now - last_read), 3) if last_read > 0 else -1.0,
                "pulse_server": os.environ.get("PULSE_SERVER"),
                "pulse_ok": self._probe_pulse_ok(),
                "whisper_ready": self._whisper_model is not None,
                "segment_queue_size": (
                    self._mic_segment_q.qsize() if self._mic_segment_q is not None else -1
                ),
            }
        return snap

    def _append_mic_health_jsonl(self, snap: dict[str, Any]) -> None:
        if not self._mic_health_log_path:
            return
        path = Path(self._mic_health_log_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(snap, ensure_ascii=False) + "\n")
        except OSError as ex:
            self.get_logger().warning(f"[voice_health] 无法写入 {path}: {ex}")

    def _dump_mic_incident(self, snap: dict[str, Any], reason: str) -> None:
        base = Path(self._mic_health_log_path).parent if self._mic_health_log_path else Path("/tmp")
        inc_dir = base / "voice_health_incidents"
        inc_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = inc_dir / f"incident_{stamp}.txt"
        lines = [
            f"===== voice mic incident {self._utc_iso()} =====",
            f"reason: {reason}",
            json.dumps(snap, ensure_ascii=False, indent=2),
            "",
            "===== pgrep (gz / yolo / voice) =====",
        ]
        for pat in ("gzserver", "gzclient", "yolo_object", "voice_gateway", "whisper"):
            try:
                out = subprocess.run(
                    ["pgrep", "-af", pat],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                lines.append(f"--- {pat} ---")
                lines.append(out.stdout.strip() or "(none)")
            except (OSError, subprocess.SubprocessError) as ex:
                lines.append(f"--- {pat}: {ex}")
        try:
            out = subprocess.run(["free", "-h"], capture_output=True, text=True, timeout=2)
            lines.extend(["", "===== free -h =====", out.stdout])
        except (OSError, subprocess.SubprocessError):
            pass
        if shutil.which("nvidia-smi"):
            try:
                out = subprocess.run(
                    ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total", "--format=csv,noheader"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                lines.extend(["", "===== nvidia-smi =====", out.stdout])
            except (OSError, subprocess.SubprocessError):
                pass
        try:
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            self.get_logger().error(f"[voice_health] 已写入故障快照: {path}")
        except OSError as ex:
            self.get_logger().error(f"[voice_health] 无法写入快照 {path}: {ex}")

    def _mic_health_watchdog_loop(self) -> None:
        while self._mic_stop is not None and not self._mic_stop.is_set():
            time.sleep(self._mic_health_interval_sec)
            snap = self._collect_mic_health_snapshot(event="periodic")
            self._append_mic_health_jsonl(snap)
            stall = float(snap.get("read_stall_sec", -1))
            if (
                snap.get("stream_open")
                and snap.get("mic_thread_alive")
                and stall >= self._mic_stall_alert_sec
            ):
                if not self._mic_stall_incident_sent:
                    self._mic_stall_incident_sent = True
                    msg = (
                        f"[voice_health] 麦克风 read 停滞 {stall:.1f}s（>{self._mic_stall_alert_sec}s）"
                        f" peak_rms={snap.get('peak_rms')} thr={snap.get('rms_threshold')}"
                        f" pulse_ok={snap.get('pulse_ok')} gz 可能已拖死 Pulse；见 incident 文件"
                    )
                    self.get_logger().error(msg)
                    self._dump_mic_incident(snap, reason="mic_read_stall")
            else:
                self._mic_stall_incident_sent = False

    def _release_mic_stream(self) -> None:
        with self._mic_stream_lock:
            stream = self._mic_stream
            self._mic_stream = None
        if stream is None:
            return
        for meth in ("abort", "stop", "close"):
            try:
                fn = getattr(stream, meth, None)
                if callable(fn):
                    fn()
            except Exception:
                pass

    def destroy_node(self) -> bool:
        if self._mic_stop is not None:
            self._mic_stop.set()
        self._release_mic_stream()
        if self._mic_thread is not None:
            self._mic_thread.join(timeout=6.0)
            self._mic_thread = None
        if self._mic_tx_thread is not None:
            self._mic_tx_thread.join(timeout=120.0)
            self._mic_tx_thread = None
        if self._mic_health_watchdog_thread is not None:
            self._mic_health_watchdog_thread.join(timeout=3.0)
            self._mic_health_watchdog_thread = None
        return super().destroy_node()

    def _flush_mic_transcript_queue(self) -> None:
        try:
            while True:
                text, meta = self._mic_result_q.get_nowait()
                self._emit_speech_text(text, source="whisper_mic", meta=meta)
        except queue.Empty:
            pass

    def _mic_transcribe_worker_loop(self) -> None:
        """单线程串行执行 Whisper，避免多 daemon 线程争 CUDA；模型未就绪时段落会在取队列后等待加载。"""
        self.get_logger().info(
            "[voice_gateway] whisper_mic 转写 worker 启动（串行），等待麦克风切段入队…"
        )
        if self._mic_preload_whisper and self._whisper_model is None and not self._whisper_preload_at_startup:
            self.get_logger().info("[voice_gateway] mic_preload_whisper:=true，worker 内加载 Whisper…")
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
                segments, info = model.transcribe(a, **kw)
                seg_list = list(segments)
            text = self._postprocess_whisper_text(
                "".join(s.text for s in seg_list).strip(), seg_list, info=info
            )
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
                    "可试: 说话更长更响、-p mic_input_gain:=10、-p mic_whisper_no_speech_threshold:=0.32；"
                    "若曾设 whisper_initial_prompt 请改回空字符串"
                )
        except Exception as ex:
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 转写失败: {ex}\n{traceback.format_exc()}"
            )

    @staticmethod
    def _mic_list_inputs(sd_mod: Any) -> list[tuple[int, str]]:
        out: list[tuple[int, str]] = []
        try:
            for i, d in enumerate(sd_mod.query_devices()):
                if int(d.get("max_input_channels", 0) or 0) > 0:
                    out.append((i, str(d.get("name", ""))))
        except Exception:
            pass
        return out

    @staticmethod
    def _mic_rank_input_indices(inputs: list[tuple[int, str]], *, wsl: bool) -> list[int]:
        """WSL 常见 [0]=pulse、[1]=default；PortAudio 默认常为 1，但 pulse 更不易 Timeout。"""

        def _prio(item: tuple[int, str]) -> tuple[int, int]:
            i, name = item
            n = name.lower()
            if wsl and "pulse" in n:
                return (0, i)
            if wsl and n.strip() == "default":
                return (30, i)
            if wsl and "default" in n:
                return (25, i)
            return (10, i)

        return [i for i, _ in sorted(inputs, key=_prio)]

    def _mic_log_input_devices(self, sd_mod: Any) -> list[tuple[int, str]]:
        inputs = self._mic_list_inputs(sd_mod)
        if inputs:
            self.get_logger().info(
                "[voice_gateway] 输入设备: "
                + "; ".join(f"[{i}] {name}" for i, name in inputs)
            )
        else:
            self.get_logger().warning("[voice_gateway] 未枚举到任何输入通道>0 的音频设备。")
        return inputs

    def _mic_input_device_candidates(self, sd_mod: Any) -> list[int | str]:
        """按优先级返回待尝试的输入设备；指定 index 不存在时回退到自动列表。"""
        pulse = _apply_wsl_pulse_env()
        self.get_logger().info(
            f"[voice_gateway] 音频环境 PULSE_SERVER={os.environ.get('PULSE_SERVER', '<未设置>')}"
            + ("（已校验可用）" if pulse else "（PortAudio 默认路由）")
        )

        wsl = _is_wsl()
        max_attempts = 8 if wsl else 2
        for attempt in range(max_attempts):
            inputs = self._mic_list_inputs(sd_mod)
            if inputs:
                if attempt == 0:
                    self._mic_log_input_devices(sd_mod)
                ranked = self._mic_rank_input_indices(inputs, wsl=wsl)
                if wsl and ranked:
                    pa_default: int | None = None
                    try:
                        pair = sd_mod.default.device
                        di = pair[0] if isinstance(pair, (list, tuple)) else pair
                        if di is not None and int(di) >= 0:
                            pa_default = int(di)
                    except Exception:
                        pa_default = None
                    if pa_default is not None and pa_default != ranked[0]:
                        self.get_logger().info(
                            f"[voice_gateway] WSL 优先 pulse 设备 index={ranked[0]} "
                            f"（PortAudio 默认输入为 index={pa_default}，full_system 下 default 易 Pulse Timeout）"
                        )
                preferred = self._mic_device
                if preferred is not None:
                    if isinstance(preferred, int):
                        if preferred in ranked:
                            rest = [i for i in ranked if i != preferred]
                            return [preferred, *rest]
                        self.get_logger().warning(
                            f"[voice_gateway] mic_device_index={preferred} 当前不可用"
                            f"（PortAudio 尚未枚举到该设备），改用自动列表并继续重试"
                        )
                    else:
                        return [preferred, *ranked]
                return ranked
            if attempt == 0:
                self._mic_log_input_devices(sd_mod)
            had_pulse = bool(os.environ.get("PULSE_SERVER"))
            if had_pulse and not _pulse_server_alive():
                self.get_logger().warning(
                    "[voice_gateway] PULSE_SERVER 已设但 pactl 无响应，清除后重试"
                )
                os.environ.pop("PULSE_SERVER", None)
            elif attempt == 2 and not had_pulse:
                _apply_wsl_pulse_env()
            if attempt + 1 < max_attempts:
                time.sleep(2.0)
        self._mic_log_input_devices(sd_mod)
        return []

    def _mic_whisper_loop(self) -> None:
        try:
            import numpy as np
            import sounddevice as sd
        except ImportError as e:
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 缺少依赖: {e}；请 pip install -r requirements-voice.txt"
            )
            self._mic_set_thread_alive(False)
            return
        except OSError as e:
            self.get_logger().error(
                "[voice_gateway] sounddevice 需要系统库 PortAudio（与 pip 无关）。"
                "Ubuntu / Debian / WSL: sudo apt-get update && sudo apt-get install -y portaudio19-dev "
                f"安装后重试。原始错误: {e}"
            )
            self._mic_set_thread_alive(False)
            return

        self._mic_set_thread_alive(True)
        sr = self._mic_sr
        block = max(int(sr * self._mic_block_ms / 1000.0), 128)
        silence_samples_need = int(sr * self._mic_silence_sec)
        min_samples = int(sr * self._mic_min_speech_sec)
        thresh = self._mic_rms_thresh

        buf: list[Any] = []
        silence_run = 0
        had_voice = False

        stream: Any = None
        use_device: int | str | None = None
        stream_kw_base: dict[str, Any] = {
            "samplerate": sr,
            "channels": 1,
            "dtype": "float32",
            "blocksize": block,
        }
        deadline = time.monotonic() + self._mic_open_retry_sec
        attempt_round = 0
        while stream is None and time.monotonic() < deadline:
            if self._mic_stop and self._mic_stop.is_set():
                return
            candidates = self._mic_input_device_candidates(sd)
            if not candidates:
                attempt_round += 1
                if attempt_round == 1 or attempt_round % 5 == 0:
                    self.get_logger().warning(
                        f"[voice_gateway] 尚无输入设备，{self._mic_open_retry_sec:.0f}s 内重试…"
                        "（与 Whisper 加载并行；勿在另一终端运行 check_mic_devices.sh）"
                    )
                time.sleep(2.0)
                continue
            for dev in candidates:
                try:
                    self.get_logger().info(
                        f"[voice_gateway] whisper_mic 尝试打开输入设备 "
                        f"{'index=' + str(dev) if isinstance(dev, int) else 'name=' + repr(dev)}"
                    )
                    stream = sd.InputStream(**stream_kw_base, device=dev)
                    stream.start()
                    with self._mic_stream_lock:
                        self._mic_stream = stream
                    use_device = dev
                    break
                except Exception as ex:
                    self.get_logger().warning(
                        f"[voice_gateway] 打开设备 {dev!r} 失败: {ex}；尝试下一个候选"
                    )
                    if stream is not None:
                        try:
                            stream.close()
                        except Exception:
                            pass
                        stream = None
            if stream is None:
                attempt_round += 1
                time.sleep(2.0)
        if stream is None or use_device is None:
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 在 {self._mic_open_retry_sec:.0f}s 内仍无法打开麦克风。"
                "请先停 full_system 后运行: bash scripts/check_mic_devices.sh"
            )
            self._mic_set_thread_alive(False)
            return

        self._mic_set_stream_open(True)
        self._mic_note_read(0.0)
        self._append_mic_health_jsonl(self._collect_mic_health_snapshot(event="stream_open"))

        self.get_logger().info(
            f"[voice_gateway] whisper_mic 已打开输入 "
            f"{'index=' + str(use_device) if isinstance(use_device, int) else 'name=' + repr(use_device)}，"
            "请说话（说完停顿约 0.6s 触发识别）"
        )

        try:
            while self._mic_stop and not self._mic_stop.is_set():
                data, overflowed = stream.read(block)
                if overflowed:
                    self._mic_note_overflow()
                    self.get_logger().warning("[voice_gateway] whisper_mic 输入溢出，可能丢字")
                chunk = np.asarray(data, dtype=np.float32).reshape(-1)
                rms = float(np.sqrt(np.mean(chunk * chunk))) if chunk.size else 0.0
                self._mic_note_read(rms)
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
                            self._mic_note_segment()
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
            self.get_logger().error(
                f"[voice_gateway] whisper_mic 麦克风线程退出: {e}。"
                "WSL 可试 -p mic_device_index:=0；若 Pulse Timeout 多为 default 设备，勿用 index=1。"
            )
        finally:
            self._mic_set_stream_open(False)
            self._append_mic_health_jsonl(self._collect_mic_health_snapshot(event="stream_close"))
            with self._mic_stream_lock:
                if self._mic_stream is stream:
                    self._mic_stream = None
            if stream is not None:
                try:
                    stream.stop()
                    stream.close()
                except Exception:
                    pass
            self._mic_set_thread_alive(False)

    _PROMPT_ECHO_FRAGMENTS = (
        "请用简体中文",
        "勿用繁体",
        "简体中文输出",
    )

    def _postprocess_whisper_text(self, text: str, segments: list[Any], *, info: Any = None) -> str:
        """去掉静音幻听的 initial_prompt 回声及明显无语音段。"""
        t = (text or "").strip()
        if not t:
            return ""
        prompt = self._whisper_initial_prompt
        if prompt:

            def _norm(s: str) -> str:
                return re.sub(r"[\s，。、；：\"'「」！？,.;:!?\-]+", "", s)

            pn, tn = _norm(prompt), _norm(t)
            if pn and (pn in tn or tn in pn):
                return ""
        if len(t) <= 36:
            for frag in self._PROMPT_ECHO_FRAGMENTS:
                if frag in t:
                    return ""
        if segments:
            nsp = [float(getattr(s, "no_speech_prob", 0.0) or 0.0) for s in segments]
            if nsp and min(nsp) > 0.72 and len(t) < 48:
                return ""
            logprobs = [getattr(s, "avg_logprob", None) for s in segments]
            logprobs = [float(x) for x in logprobs if x is not None]
            if logprobs and max(logprobs) < -0.95 and len(t) < 24:
                return ""
        _ = info
        return t

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
            return self._whisper_model  # 同进程内复用，不会重复构造
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
                    prep_hint = (
                        f"python3 scripts/prep_faster_whisper.py --size {size} "
                        f"--device {dev} --compute-type {ctype}"
                    )
                    self.get_logger().info(
                        "[voice_gateway] WSL 下使用 CUDA 加载 Whisper；若长时间无「模型就绪」，"
                        f"可在另一终端用与节点相同参数热身: {prep_hint}"
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
            prep_cmd = (
                f"python3 scripts/prep_faster_whisper.py --size {size} "
                f"--device {dev} --compute-type {ctype}"
            )
            self.get_logger().info(
                "[voice_gateway] 正在构造 WhisperModel…（权重与 prep 相同；Hub 上无单独「GPU int8 包」，"
                f"int8 指 CTranslate2 在 {dev} 上的量化推理）"
            )
            self.get_logger().info(
                f"[voice_gateway] 若长时间无「模型就绪」，可在另一终端先热身（参数须一致）: {prep_cmd}"
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
            elapsed = time.monotonic() - t0
            self.get_logger().info(
                f"[voice_gateway] Whisper 模型就绪，耗时 {elapsed:.2f}s"
                "（此后本进程内转写不再重复加载；仅重启 launch/节点会再花同样时间初始化 GPU）"
            )
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
                segments, info = model.transcribe(str(p.resolve()), **tw_kw)
                seg_list = list(segments)
            text = self._postprocess_whisper_text(
                "".join(s.text for s in seg_list).strip(), seg_list, info=info
            )
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
