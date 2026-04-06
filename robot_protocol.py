"""机器人联动协议定义。

阶段 2 协议约定：
1. 一行一条消息
2. UTF-8 编码
3. 消息格式为 `TYPE;KEY=VALUE;KEY=VALUE`
4. 当前支持任务：
   - CIRCLE
   - LINE
   - INTERSECTION3P
"""

from __future__ import annotations

from dataclasses import dataclass

from exceptions import ProtocolError
from result_types import CircleFitResult, LineResult

Point3D = tuple[float, float, float]

SUPPORTED_TASKS = {"CIRCLE", "LINE", "INTERSECTION3P"}


@dataclass(slots=True)
class RobotMessage:
    """解析后的机器人协议消息。"""

    message_type: str
    req: str
    fields: dict[str, str]
    raw: str

    def get(self, key: str, default: str | None = None) -> str | None:
        """按 key 读取字段值。"""
        return self.fields.get(key.upper(), default)

    def get_required(self, key: str) -> str:
        """读取一个必填字段。"""
        value = self.get(key)
        if value is None or value == "":
            raise ProtocolError(f"缺少字段 {key}")
        return value

    def get_int(self, key: str) -> int:
        """把字段解析为整数。"""
        value = self.get_required(key)
        try:
            return int(value)
        except ValueError as exc:
            raise ProtocolError(f"字段 {key} 不是有效整数: {value}") from exc

    def get_float(self, key: str) -> float:
        """把字段解析为浮点数。"""
        value = self.get_required(key)
        try:
            return float(value)
        except ValueError as exc:
            raise ProtocolError(f"字段 {key} 不是有效数字: {value}") from exc


def parse_message(raw_message: str) -> RobotMessage:
    """把原始文本解析为结构化协议消息。"""
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


def build_line_result(req: str, result: LineResult, frame: str) -> str:
    """构造两面交线结果消息。"""
    return _serialize_message(
        "RESULT",
        [
            ("REQ", req),
            ("STATUS", "OK"),
            ("TASK", "LINE"),
            ("FRAME", frame),
            ("PX", _format_number(result.point[0])),
            ("PY", _format_number(result.point[1])),
            ("PZ", _format_number(result.point[2])),
            ("DX", _format_number(result.direction[0])),
            ("DY", _format_number(result.direction[1])),
            ("DZ", _format_number(result.direction[2])),
        ],
    )


def build_intersection3p_result(req: str, point: Point3D, frame: str) -> str:
    """构造三平面交点结果消息。"""
    return _serialize_message(
        "RESULT",
        [
            ("REQ", req),
            ("STATUS", "OK"),
            ("TASK", "INTERSECTION3P"),
            ("FRAME", frame),
            ("PX", _format_number(point[0])),
            ("PY", _format_number(point[1])),
            ("PZ", _format_number(point[2])),
        ],
    )


def _validate_message(message: RobotMessage) -> None:
    """校验协议消息的基础格式。"""
    message.get_required("REQ")

    if message.message_type == "HELLO":
        message.get_required("CLIENT")
        message.get_required("VER")
        return

    if message.message_type == "START":
        task = message.get_required("TASK").upper()
        if task not in SUPPORTED_TASKS:
            raise ProtocolError(f"当前不支持的任务类型: {task}")

        message.get_required("FRAME")
        if task == "CIRCLE":
            points = message.get_int("POINTS")
            if points <= 0:
                raise ProtocolError("POINTS 必须大于 0")
            return

        if task == "LINE":
            p1 = message.get_int("P1")
            p2 = message.get_int("P2")
            if p1 <= 0 or p2 <= 0:
                raise ProtocolError("LINE 任务的 P1/P2 必须大于 0")
            return

        p1 = _parse_optional_int(message, "P1", 3)
        p2 = _parse_optional_int(message, "P2", 3)
        p3 = _parse_optional_int(message, "P3", 3)
        if p1 <= 0 or p2 <= 0 or p3 <= 0:
            raise ProtocolError("INTERSECTION3P 任务的 P1/P2/P3 必须大于 0")
        return

    if message.message_type == "POINT":
        idx = message.get_int("IDX")
        if idx <= 0:
            raise ProtocolError("IDX 必须从 1 开始")
        if "GROUP" in message.fields and not message.get_required("GROUP").strip():
            raise ProtocolError("GROUP 不能为空")
        message.get_float("X")
        message.get_float("Y")
        message.get_float("Z")
        return

    if message.message_type == "END":
        return

    raise ProtocolError(f"未知消息类型: {message.message_type}")


def _serialize_message(message_type: str, ordered_fields: list[tuple[str, str | int | float]]) -> str:
    """把消息类型和字段列表序列化为一行文本。"""
    segments = [message_type.upper()]
    for key, value in ordered_fields:
        segments.append(f"{key}={value}")
    return ";".join(segments)


def _format_number(value: float) -> str:
    """统一数值输出格式。"""
    return f"{value:.6f}"


def _parse_optional_int(message: RobotMessage, key: str, default: int) -> int:
    """读取可选整数，没有时返回默认值。"""
    value = message.get(key)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise ProtocolError(f"字段 {key} 不是有效整数: {value}") from exc
