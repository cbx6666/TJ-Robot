#!/usr/bin/env python3
"""预下载或校验 faster-whisper 权重（与 voice_gateway 首次加载一致）。

WSL 若报 Network unreachable，可任选其一：
  1) 使用镜像：  export HF_ENDPOINT=https://hf-mirror.com
     python3 scripts/prep_faster_whisper.py ...
  2) 在 Windows 浏览器能上网的机器上下载仓库，拷进 WSL 目录后：
     python3 scripts/prep_faster_whisper.py --local-model /path/to/faster-whisper-base
  3) 节点里把 whisper_model_size 设为同一本地目录（不要用 Hub 别名如 base，要用路径字符串）。

官方模型页（base 示例，含「Files and versions」可打包下载）:
  https://huggingface.co/Systran/faster-whisper-base

国内镜像站（同仓库）:
  https://hf-mirror.com/Systran/faster-whisper-base

说明: Hub 上只有一套 CTranslate2 权重（如 Systran/faster-whisper-base），与 int8/float32 或 cpu/cuda 无关；
若节点用 cuda+int8，建议用相同参数 prep 一次，便于区分「下载慢」与「GPU 初始化卡住」（WSL 常见后者）。

用法（默认与 voice_gateway 一致：whisper_device=auto → 有 CUDA 则用 cuda，whisper_compute_type=int8）:
  python3 scripts/prep_faster_whisper.py
  python3 scripts/prep_faster_whisper.py --size base --device cuda --compute-type int8
  python3 scripts/prep_faster_whisper.py --hf-endpoint https://hf-mirror.com
  python3 scripts/prep_faster_whisper.py --local-model /mnt/e/models/faster-whisper-base
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


def _resolve_device_compute(device: str, compute_type: str) -> tuple[str, str]:
    """与 voice_gateway_node._ensure_whisper 相同的 device/compute 解析（不含 mic_force_cpu_on_wsl）。"""
    dev = (device or "auto").strip().lower()
    ctype = (compute_type or "int8").strip()
    if dev == "auto":
        try:
            import torch

            dev = "cuda" if torch.cuda.is_available() else "cpu"
        except ImportError:
            dev = "cpu"
    if dev == "cpu" and ctype.lower() in ("int8", "int8_float32", "int8_bfloat16"):
        print(
            "[prep] device=cpu：compute_type 自动改为 float32（与 voice_gateway 一致，避免 WhisperModel 卡死）",
            flush=True,
        )
        ctype = "float32"
    return dev, ctype


def _hub_repo_for_size(size: str) -> str:
    """HuggingFace 上的仓库名（faster-whisper 约定）。"""
    if size.startswith("/") or size.startswith("."):
        return ""
    return f"Systran/faster-whisper-{size}"


def _print_network_help(size: str) -> None:
    repo = _hub_repo_for_size(size)
    print("", file=sys.stderr)
    print("[prep] 网络不可达时的可选办法：", file=sys.stderr, flush=True)
    print("  (1) 终端先执行: export HF_ENDPOINT=https://hf-mirror.com", file=sys.stderr, flush=True)
    print("      再重新运行本脚本。", file=sys.stderr, flush=True)
    if repo:
        print(f"  (2) 浏览器打开（需能访问外网或镜像）:", file=sys.stderr, flush=True)
        print(f"      https://huggingface.co/{repo}", file=sys.stderr, flush=True)
        print(f"      镜像: https://hf-mirror.com/{repo}", file=sys.stderr, flush=True)
        print(
            f"      下载完整仓库（含 model.bin 等）解压到某目录，例如 /mnt/e/models/faster-whisper-{size} ，",
            file=sys.stderr,
            flush=True,
        )
        print(
            f"      然后: python3 scripts/prep_faster_whisper.py --local-model /mnt/e/models/faster-whisper-{size}",
            file=sys.stderr,
            flush=True,
        )
        print(
            "  (3) 在 Windows 下用浏览器/git 下载后，通过 /mnt/e/... 拷到 WSL，路径同上。",
            file=sys.stderr,
            flush=True,
        )
        print(
            "  (4) ros2 运行时参数 whisper_model_size 填同一绝对路径字符串（与 --local-model 一致）。",
            file=sys.stderr,
            flush=True,
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size", default="base", help="Hub 上的名字，如 tiny/base/small；或配合 --local-model 可忽略")
    ap.add_argument(
        "--device",
        default="auto",
        help="auto（有 CUDA 用 cuda，否则 cpu）| cuda | cpu；与 voice_gateway whisper_device 一致",
    )
    ap.add_argument(
        "--compute-type",
        default="int8",
        help="与 voice_gateway whisper_compute_type 一致；仅 cpu 时会自动降为 float32",
    )
    ap.add_argument(
        "--download-root",
        default="",
        help="可选；与 voice_gateway 的 whisper_download_root 一致",
    )
    ap.add_argument(
        "--hf-endpoint",
        default="",
        help="例如 https://hf-mirror.com ，会写入环境变量 HF_ENDPOINT 后再下载",
    )
    ap.add_argument(
        "--local-model",
        default="",
        help="已下载到本地的 CTranslate2 模型目录（内含 model.bin），不经 Hub",
    )
    ap.add_argument(
        "--print-urls",
        action="store_true",
        help="只打印 base 对应的官方/镜像下载页后退出",
    )
    args = ap.parse_args()

    if args.print_urls:
        print("base 官方: https://huggingface.co/Systran/faster-whisper-base")
        print("base 镜像: https://hf-mirror.com/Systran/faster-whisper-base")
        return 0

    ep = (args.hf_endpoint or "").strip().rstrip("/")
    if ep:
        os.environ["HF_ENDPOINT"] = ep
        print(f"[prep] 已设置 HF_ENDPOINT={ep!r}", flush=True)

    os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "0")

    try:
        from huggingface_hub.constants import HF_HUB_CACHE

        print(f"[prep] HuggingFace 默认缓存: {HF_HUB_CACHE}", flush=True)
    except Exception:
        pass

    dr = (args.download_root or "").strip()
    lm = (args.local_model or "").strip()
    dev, ctype = _resolve_device_compute(args.device, args.compute_type)

    try:
        from faster_whisper import WhisperModel
    except ImportError as e:
        print(f"[prep] 未安装 faster-whisper: {e}", file=sys.stderr, flush=True)
        print("[prep] 请执行: pip install -r ros_ws/src/robot_interaction/requirements-voice.txt", file=sys.stderr)
        return 1

    if lm:
        p = Path(lm).expanduser().resolve()
        if not p.is_dir():
            print(f"[prep] --local-model 不是目录: {p}", file=sys.stderr, flush=True)
            return 1
        print(f"[prep] 从本地目录加载（不经网络）: {p}", flush=True)
        t0 = time.monotonic()
        try:
            WhisperModel(str(p), device=dev, compute_type=ctype)
        except Exception as e:
            print(f"[prep] 失败: {e}", file=sys.stderr, flush=True)
            return 1
        print(f"[prep] 完成，耗时 {time.monotonic() - t0:.1f}s", flush=True)
        return 0

    print(
        f"[prep] 开始从 Hub 拉取: size={args.size!r} device={dev!r} compute_type={ctype!r}"
        + (f" download_root={dr!r}" if dr else ""),
        flush=True,
    )
    t0 = time.monotonic()
    try:
        if dr:
            try:
                WhisperModel(args.size, device=dev, compute_type=ctype, download_root=dr)
            except TypeError:
                WhisperModel(args.size, device=dev, compute_type=ctype)
        else:
            WhisperModel(args.size, device=dev, compute_type=ctype)
    except Exception as e:
        print(f"[prep] 失败: {e}", file=sys.stderr, flush=True)
        _print_network_help(args.size)
        return 1

    print(f"[prep] 完成，耗时 {time.monotonic() - t0:.1f}s（若已缓存则主要为本地校验）", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
