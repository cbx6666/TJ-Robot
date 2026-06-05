from __future__ import annotations

import json
import os
import re
import unicodedata
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_FETCH_LABEL_ALIASES: tuple[tuple[str, str], ...] = (
    ("水杯", "cup"),
    ("茶杯", "cup"),
    ("杯子", "cup"),
    ("杯", "cup"),
    ("可乐罐", "bottle"),
    ("可乐", "bottle"),
    ("瓶子", "bottle"),
    ("瓶", "bottle"),
    ("花瓶", "vase"),
    ("椅子", "chair"),
    ("书本", "book"),
    ("书", "book"),
    ("手机", "cell phone"),
)


def _load_text_file(path: str) -> str:
    p = Path(path)
    if not p.is_file():
        return ""
    return p.read_text(encoding="utf-8").strip()


def _normalize_api_key_for_http(raw: str) -> str:
    """去除 .env / 复制粘贴里常见的不可见字符与 Unicode 空白，便于通过 http.client 的 latin-1 头编码。"""
    s = (raw or "").strip().lstrip("\ufeff")
    s = unicodedata.normalize("NFKC", s)
    s = s.replace("\r", "").replace("\n", "")
    s = re.sub(r"[\u200b-\u200f\u202a-\u202e\u2060\u2062\u2063\ufeff]", "", s)
    s = re.sub(r"[\u00a0\u1680\u2000-\u200a\u202f\u205f\u3000]+", "", s)
    return s.strip()


def _api_key_for_bearer_header(raw: str) -> tuple[str, str | None]:
    """返回 (key, tag)。tag 为 None 表示可用；为 stripped_non_ascii 表示已剥除非 ASCII；为 empty/其它字符串表示失败且 key 为空。"""
    key = _normalize_api_key_for_http(raw)
    if not key:
        return "", "empty"

    def _latin1_ok(s: str) -> bool:
        try:
            s.encode("latin-1")
            return True
        except UnicodeEncodeError:
            return False

    if _latin1_ok(f"Bearer {key}"):
        return key, None

    ascii_key = "".join(c for c in key if ord(c) < 128)
    if ascii_key and _latin1_ok(f"Bearer {ascii_key}"):
        return ascii_key, "stripped_non_ascii" if ascii_key != key else None

    hdr = f"Bearer {key}"
    for i, ch in enumerate(hdr):
        try:
            ch.encode("latin-1")
        except UnicodeEncodeError:
            return "", f"char_at_bearer_offset_{i}=U+{ord(ch):04X}_name={unicodedata.name(ch, '?')}"
    return "", "latin1"


def _extract_json_object(text: str) -> dict[str, Any] | None:
    """从模型输出中尽量解析单个 JSON 对象。"""
    text = (text or "").strip()
    if not text:
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    m = re.search(r"\{[\s\S]*\}", text)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except json.JSONDecodeError:
        return None


def _turn_direction_sign(user_text: str) -> int | None:
    t = (user_text or "").strip()
    if not t:
        return None
    low = t.lower()
    if re.search(r"向右|右转|往右|朝右", t):
        return -1
    if re.search(r"向左|左转|往左|朝左", t):
        return 1
    if "逆时针" in t:
        return 1
    if "顺时针" in t:
        return -1
    if re.search(r"\b(turn\s+)?right\b", low) or re.search(r"\bright\b", low):
        return -1
    if re.search(r"\b(turn\s+)?left\b", low) or re.search(r"\bleft\b", low):
        return 1
    if re.search(r"\bccw\b", low):
        return 1
    if re.search(r"\bcw\b", low) or "clockwise" in low:
        return -1
    return None


def _parse_turn_magnitude_deg(user_text: str) -> float:
    t = (user_text or "").strip()
    m = re.search(r"(-?\d+(?:\.\d+)?)\s*度", t)
    if m:
        return abs(float(m.group(1)))
    if "180" in t:
        return 180.0
    if "90" in t:
        return 90.0
    return 0.0


def _fix_turn_intent(obj: dict[str, Any], user_text: str) -> dict[str, Any]:
    """LLM/离线输出后统一：左转为正 relative_yaw_deg，右转为负。"""
    if str(obj.get("command", "")).strip() != "navigate_to_pose":
        return obj
    args = obj.get("args") if isinstance(obj.get("args"), dict) else {}
    args = dict(args)
    text = str(args.get("user_request_zh") or user_text or "").strip()
    if text:
        args["user_request_zh"] = text
    yaw_only = bool(args.get("yaw_only") or args.get("rotate_in_place"))
    rel_raw = args.get("relative_yaw_deg", args.get("yaw_delta_deg"))
    try:
        rel = float(rel_raw) if rel_raw is not None and rel_raw != "" else None
    except (TypeError, ValueError):
        rel = None
    if rel is None and not yaw_only and not any(k in text for k in ("转", "转身", "旋转")):
        return {**obj, "args": args}

    sign_hint = _turn_direction_sign(text)
    mag = abs(rel) if rel is not None else _parse_turn_magnitude_deg(text)
    if mag <= 0.0:
        mag = _parse_turn_magnitude_deg(text)
    if mag <= 0.0:
        return {**obj, "args": args}

    if sign_hint is not None:
        signed = float(sign_hint) * mag
    elif rel is not None:
        signed = float(rel)
    else:
        signed = mag

    args["yaw_only"] = True
    args["relative_yaw_deg"] = signed
    args.pop("yaw_delta_deg", None)
    return {**obj, "args": args}


def _canonical_fetch_label(raw: str, user_text: str = "") -> str:
    text = (raw or "").strip()
    low = text.lower().replace("_", " ")
    direct = {
        "cup": "cup",
        "mug": "cup",
        "water cup": "cup",
        "bottle": "bottle",
        "beer": "bottle",
        "coke": "bottle",
        "coke can": "bottle",
        "vase": "vase",
        "chair": "chair",
        "book": "book",
        "cell phone": "cell phone",
        "phone": "cell phone",
    }
    if low in direct:
        return direct[low]
    probe = f"{text} {user_text}".strip()
    for zh, label in _FETCH_LABEL_ALIASES:
        if zh in probe:
            return label
    return text or "object"


def _normalize_fetch_intent(obj: dict[str, Any], user_text: str) -> dict[str, Any]:
    if str(obj.get("command", "")).strip() != "fetch_object":
        return obj
    args = obj.get("args") if isinstance(obj.get("args"), dict) else {}
    args = dict(args)
    raw_label = ""
    for key in ("object_label", "label", "object", "target"):
        v = args.get(key)
        if v is not None and str(v).strip():
            raw_label = str(v).strip()
            break
    args["object_label"] = _canonical_fetch_label(raw_label, user_text)
    if user_text:
        args["user_request_zh"] = user_text
    return {**obj, "args": args}


def _offline_plan(user_text: str) -> dict[str, Any]:
    t = (user_text or "").strip()
    low = t.lower()
    if not t:
        return {
            "version": 1,
            "command": "noop",
            "args": {},
            "rationale_zh": "空输入",
            "confidence": 0.0,
        }
    if any(k in t for k in ("停", "取消", "别动", "不要动", "停止")) or "stop" in low:
        return {
            "version": 1,
            "command": "stop",
            "args": {},
            "rationale_zh": "检测到停止/取消类指令",
            "confidence": 0.7,
        }
    if any(k in t for k in ("巡检", "巡视", "巡逻", "检查房间", "看看房间")):
        return {
            "version": 1,
            "command": "start_room_patrol",
            "args": {"patrol_scope": "room_default"},
            "rationale_zh": "离线规则：巡检房间",
            "confidence": 0.65,
        }
    if any(k in t for k in ("拿", "取", "帮我拿", "抓", "递给我")):
        label = "object"
        for w, en in (
            ("椅子", "chair"),
            ("杯子", "cup"),
            ("水杯", "cup"),
            ("茶杯", "cup"),
            ("花瓶", "vase"),
            ("瓶子", "bottle"),
            ("可乐", "bottle"),
            ("书", "book"),
            ("手机", "cell phone"),
            ("杯", "cup"),
        ):
            if w in t:
                label = en
                break
        return {
            "version": 1,
            "command": "fetch_object",
            "args": {"object_label": label, "user_request_zh": t},
            "rationale_zh": "离线规则：取物语义",
            "confidence": 0.55,
        }
    if any(k in t for k in ("转", "转身", "旋转", "面向")):
        mag = _parse_turn_magnitude_deg(t)
        sign_hint = _turn_direction_sign(t)
        delta = float(sign_hint) * mag if sign_hint is not None else mag
        return _fix_turn_intent(
            {
                "version": 1,
                "command": "navigate_to_pose",
                "args": {
                    "frame_id": "map",
                    "yaw_only": True,
                    "relative_yaw_deg": delta,
                    "user_request_zh": t,
                },
                "rationale_zh": "离线规则：原地相对转向（左正右负）",
                "confidence": 0.6,
            },
            t,
        )
    if any(k in t for k in ("去", "到", "导航", "走过去")):
        return {
            "version": 1,
            "command": "navigate_to_pose",
            "args": {"frame_id": "map", "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "user_request_zh": t},
            "rationale_zh": "离线规则：导航但未解析坐标，使用占位 0,0（请用大模型或明确坐标）",
            "confidence": 0.35,
        }
    return {
        "version": 1,
        "command": "noop",
        "args": {"user_request_zh": t},
        "rationale_zh": "离线规则：未匹配到明确指令",
        "confidence": 0.2,
    }


def _resolve_chat_completions_url(base: str) -> str:
    u = base.strip().rstrip("/")
    if u.endswith("/chat/completions"):
        return u
    if u.endswith("/v1"):
        return f"{u}/chat/completions"
    return f"{u}/v1/chat/completions"


def _validate_cmd(obj: dict[str, Any]) -> dict[str, Any]:
    allowed = {"navigate_to_pose", "start_room_patrol", "fetch_object", "stop", "noop"}
    cmd = str(obj.get("command", "noop")).strip()
    if cmd not in allowed:
        cmd = "noop"
    out = {
        "version": int(obj.get("version", 1) or 1),
        "command": cmd,
        "args": obj.get("args") if isinstance(obj.get("args"), dict) else {},
        "rationale_zh": str(obj.get("rationale_zh", "")),
        "confidence": float(obj.get("confidence", 0.5) or 0.0),
    }
    return out


class LlmRouterNode(Node):
    """订阅语音识别文本，调用 OpenAI 兼容 Chat Completions（可选），发布结构化 JSON 到 ``parsed_intent_topic``。"""

    def __init__(self) -> None:
        super().__init__("llm_router")
        self.declare_parameter("speech_text_topic", "/interaction/speech_text")
        self.declare_parameter("parsed_intent_topic", "/interaction/parsed_intent")
        self.declare_parameter("task_goal_topic", "/task/goal_text")
        self.declare_parameter("system_prompt_file", "")
        self.declare_parameter("llm_api_url", "")
        self.declare_parameter("llm_api_key_env", "TJ_LLM_API_KEY")
        self.declare_parameter("llm_model", "gpt-4o-mini")
        self.declare_parameter("llm_timeout_sec", 60.0)
        self.declare_parameter("publish_task_goal_from_fetch", False)

        speech_topic = str(self.get_parameter("speech_text_topic").value)
        parsed_topic = str(self.get_parameter("parsed_intent_topic").value)
        self._task_goal_topic = str(self.get_parameter("task_goal_topic").value)
        self._prompt_file = str(self.get_parameter("system_prompt_file").value).strip()
        self._llm_url = str(self.get_parameter("llm_api_url").value).strip()
        if not self._llm_url:
            self._llm_url = os.environ.get("TJ_LLM_API_URL", "").strip()
        self._key_env = str(self.get_parameter("llm_api_key_env").value).strip() or "TJ_LLM_API_KEY"
        self._model = str(self.get_parameter("llm_model").value).strip() or "gpt-4o-mini"
        env_model = os.environ.get("TJ_LLM_MODEL", "").strip()
        if env_model:
            self._model = env_model
        self._timeout = max(float(self.get_parameter("llm_timeout_sec").value), 5.0)
        self._pub_fetch = bool(self.get_parameter("publish_task_goal_from_fetch").value)

        self._system_prompt = _load_text_file(self._prompt_file) if self._prompt_file else ""
        if not self._system_prompt:
            self._system_prompt = (
                "你是机器人任务规划器。只输出一个 JSON，字段 version, command, args, rationale_zh, confidence。"
                "command 取值 navigate_to_pose|start_room_patrol|fetch_object|stop|noop。"
            )

        self._pub_intent = self.create_publisher(String, parsed_topic, 10)
        self._pub_goal = self.create_publisher(String, self._task_goal_topic, 10)
        self.create_subscription(String, speech_topic, self._on_speech, 10)

        key_present = bool(os.environ.get(self._key_env, "").strip())
        self.get_logger().info(
            f"[llm_router] speech={speech_topic} parsed={parsed_topic} task_goal={self._task_goal_topic} "
            f"llm_url={'set' if self._llm_url else 'empty(offline)'} "
            f"(参数 llm_api_url 或环境变量 TJ_LLM_API_URL；模型可用 llm_model 或 TJ_LLM_MODEL) "
            f"model={self._model!r} key_env={self._key_env} key_present={key_present}"
        )

    def _on_speech(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"[llm_router] user_text={text}")
        self.get_logger().info(f"[llm_router] 收到语音文本: {text!r}")
        # 在订阅回调内同步调用，避免非 executor 线程调用 publish（rclpy 非线程安全）。
        api_key = os.environ.get(self._key_env, "").strip()
        if self._llm_url and api_key:
            obj = self._call_openai_compatible(text, api_key)
        else:
            obj = _offline_plan(text)
        if obj is None:
            obj = _offline_plan(text)
        obj = _validate_cmd(obj)
        obj = _fix_turn_intent(obj, text)
        obj = _normalize_fetch_intent(obj, text)
        payload = json.dumps(obj, ensure_ascii=False)
        self._pub_intent.publish(String(data=payload))
        self.get_logger().info(f"[llm_router] parsed_intent: {payload}")
        args = obj.get("args") if isinstance(obj.get("args"), dict) else {}
        self.get_logger().info(
            f"[llm_router] parsed_intent={obj.get('command', 'noop')} "
            f"object_label={args.get('object_label', '')}"
        )
        if self._pub_fetch and obj.get("command") == "fetch_object":
            args = obj.get("args") or {}
            label = str(args.get("object_label", "object"))
            goal = f"TASK_GOAL:拿取{label}"
            self._pub_goal.publish(String(data=goal))
            self.get_logger().info(f"[llm_router] -> task_goal: {goal!r}")

    def _call_openai_compatible(self, user_text: str, api_key: str) -> dict[str, Any] | None:
        url = _resolve_chat_completions_url(self._llm_url)
        key_clean, key_err = _api_key_for_bearer_header(api_key)
        if key_err == "empty":
            self.get_logger().error("[llm_router] TJ_LLM_API_KEY 为空，跳过 HTTP 调用。")
            return None
        if key_err == "stripped_non_ascii":
            self.get_logger().warning(
                "[llm_router] TJ_LLM_API_KEY 内混有非 ASCII 字符（全角/特殊字母/不可见字符），已自动剥除后使用；"
                "请检查 local_llm.env 是否在编辑器里被误插入 Unicode。"
            )
        elif key_err is not None:
            self.get_logger().error(
                "[llm_router] TJ_LLM_API_KEY 无法用于 HTTP Authorization（须为 Latin-1 可编码，一般为纯 ASCII）。"
                f"诊断: {key_err}。请检查环境变量 {self._key_env!r} 是否与 local_llm.env 一致、文件是否为 UTF-8 无 BOM、"
                "以及是否在系统/IDE 里另设了同名变量覆盖了正确 Key。"
            )
            return None

        auth_header = f"Bearer {key_clean}"

        body = {
            "model": self._model,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": user_text},
            ],
            "temperature": 0.2,
        }
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            headers={
                "Content-Type": "application/json",
                "Authorization": auth_header,
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                raw = resp.read().decode("utf-8")
            j = json.loads(raw)
            choices = j.get("choices") or []
            if not choices:
                self.get_logger().warning("[llm_router] API 返回无 choices")
                return None
            content = (((choices[0] or {}).get("message") or {}).get("content")) or ""
            return _extract_json_object(str(content))
        except urllib.error.HTTPError as e:
            try:
                detail = e.read().decode("utf-8", errors="replace")[:500]
            except Exception:
                detail = str(e)
            self.get_logger().error(f"[llm_router] HTTPError: {e.code} {detail}")
            return None
        except Exception as e:
            self.get_logger().error(f"[llm_router] 请求失败: {e}")
            return None


def main() -> None:
    rclpy.init()
    node = LlmRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
