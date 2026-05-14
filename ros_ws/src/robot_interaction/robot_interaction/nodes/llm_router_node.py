from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _load_text_file(path: str) -> str:
    p = Path(path)
    if not p.is_file():
        return ""
    return p.read_text(encoding="utf-8").strip()


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
        for w in ("椅子", "杯", "瓶子", "书", "手机"):
            if w in t:
                label = {"椅子": "chair", "杯": "cup", "瓶子": "bottle", "书": "book", "手机": "cell phone"}.get(
                    w, "object"
                )
                break
        return {
            "version": 1,
            "command": "fetch_object",
            "args": {"object_label": label, "user_request_zh": t},
            "rationale_zh": "离线规则：取物语义",
            "confidence": 0.55,
        }
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
        self.declare_parameter("publish_task_goal_from_fetch", True)

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
        payload = json.dumps(obj, ensure_ascii=False)
        self._pub_intent.publish(String(data=payload))
        self.get_logger().info(f"[llm_router] parsed_intent: {payload}")
        if self._pub_fetch and obj.get("command") == "fetch_object":
            args = obj.get("args") or {}
            label = str(args.get("object_label", "object"))
            goal = f"TASK_GOAL:拿取{label}"
            self._pub_goal.publish(String(data=goal))
            self.get_logger().info(f"[llm_router] -> task_goal: {goal!r}")

    def _call_openai_compatible(self, user_text: str, api_key: str) -> dict[str, Any] | None:
        url = _resolve_chat_completions_url(self._llm_url)

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
                "Authorization": f"Bearer {api_key}",
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
