"""机器人联动协议定义。

阶段 1 协议约定：
1. 一行一条消息
2. UTF-8 编码
3. 使用 `TYPE;KEY=VALUE;KEY=VALUE` 格式
4. 当前只支持 `TASK=CIRCLE`
"""

from __future__ import annotations

from dataclasses import dataclass

from exceptions import ProtocolError
from result_types import CircleFitResult

SUPPORTED_TASKS = {"CIRCLE"}


@dataclass(slots=True)
class RobotMessage:
    """解析后的机器人消息。"""

    message_type: str
    req: str
    fields: dict[str, str]
    raw: str

    def get(self, key: str, default: str | None = None) -> str | None:
        """按 key 获取字段值。"""
        return self.fields.get(key.upper(), default)

    def get_required(self, key: str) -> str:
        """获取一个必填字段。"""
        value = self.get(key)
        if value is None or value == "":
            raise ProtocolError(f"缺少字段 {key}")
        return value

    def get_int(self, key: str) -> int:
        """把字段解析成整数。"""
        value = self.get_required(key)
        try:
            return int(value)
        except ValueError as exc:
            raise ProtocolError(f"字段 {key} 不是有效整数: {value}") from exc

    def get_float(self, key: str) -> float:
        """把字段解析成浮点数。"""
        value = self.get_required(key)
        try:
            return float(value)
        except ValueError as exc:
            raise ProtocolError(f"字段 {key} 不是有效数字: {value}") from exc


def parse_message(raw_message: str) -> RobotMessage:
    """把原始文本解析为机器人消息。"""
    cleaned = raw_message.strip()
    if not cleaned:
        raise ProtocolError("收到空消息")

    parts = [part.strip() for part in cleaned.split(";") if part.strip()]
    if not parts:
        raise ProtocolError("消息内容为空")

    message_type = parts[0].upper()
    fields: dict[str, str] = {}
    for segment in parts[1:]:
        if "=" not in segment:
            raise ProtocolError(f"字段格式不正确: {segment}")
        key, value = segment.split("=", 1)
        normalized_key = key.strip().upper()
        normalized_value = value.strip()
        if not normalized_key:
            raise ProtocolError("字段名不能为空")
        fields[normalized_key] = normalized_value

    message = RobotMessage(
        message_type=message_type,
        req=fields.get("REQ", "0"),
        fields=fields,
        raw=cleaned,
    )
    _validate_message(message)
    return message


def build_ack(req: str, state: str, **extra_fields: str | int | float) -> str:
    """构造 ACK 消息。"""
    ordered_fields: list[tuple[str, str | int | float]] = [("REQ", req), ("STATE", state)]
    ordered_fields.extend((key.upper(), value) for key, value in extra_fields.items())
    return _serialize_message("ACK", ordered_fields)


def build_error(req: str, code: str, message: str, **extra_fields: str | int | float) -> str:
    """构造 ERROR 消息。"""
    ordered_fields: list[tuple[str, str | int | float]] = [
        ("REQ", req),
        ("CODE", code),
        ("MSG", message),
    ]
    ordered_fields.extend((key.upper(), value) for key, value in extra_fields.items())
    return _serialize_message("ERROR", ordered_fields)


def build_circle_result(req: str, result: CircleFitResult, frame: str) -> str:
    """构造拟合圆结果消息。"""
    return _serialize_message(
        "RESULT",
        [
            ("REQ", req),
            ("STATUS", "OK"),
            ("TASK", "CIRCLE"),
            ("FRAME", frame),
            ("CX", _format_number(result.center[0])),
            ("CY", _format_number(result.center[1])),
            ("CZ", _format_number(result.center[2])),
            ("D", _format_number(result.diameter)),
            ("NX", _format_number(result.normal[0])),
            ("NY", _format_number(result.normal[1])),
            ("NZ", _format_number(result.normal[2])),
        ],
    )


def _validate_message(message: RobotMessage) -> None:
    """校验消息格式和阶段 1 的协议约束。"""
    message.get_required("REQ")

    if message.message_type == "HELLO":
        message.get_required("CLIENT")
        message.get_required("VER")
        return

    if message.message_type == "START":
        task = message.get_required("TASK").upper()
        if task not in SUPPORTED_TASKS:
            raise ProtocolError(f"当前不支持的任务类型: {task}")
        points = message.get_int("POINTS")
        if points <= 0:
            raise ProtocolError("POINTS 必须大于 0")
        message.get_required("FRAME")
        return

    if message.message_type == "POINT":
        idx = message.get_int("IDX")
        if idx <= 0:
            raise ProtocolError("IDX 必须从 1 开始")
        message.get_float("X")
        message.get_float("Y")
        message.get_float("Z")
        return

    if message.message_type == "END":
        return

    raise ProtocolError(f"未知消息类型: {message.message_type}")


def _serialize_message(message_type: str, ordered_fields: list[tuple[str, str | int | float]]) -> str:
    """把消息类型和字段序列化为一行文本。"""
    segments = [message_type.upper()]
    for key, value in ordered_fields:
        segments.append(f"{key}={value}")
    return ";".join(segments)


def _format_number(value: float) -> str:
    """统一数值输出格式。"""
    return f"{value:.6f}"
