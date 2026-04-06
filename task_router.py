"""多任务路由辅助模块。

这个模块把“不同 TASK 的协议规则、点分组规则、结果格式规则”
集中到一起，避免这些分支判断全部堆到 controller 里。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from config import EXPECTED_POINT_COUNT, MIN_POINTS_FOR_CIRCLE, MIN_POINTS_FOR_PLANE
from exceptions import ProtocolError
from result_types import CircleFitResult, LineResult
from robot_protocol import (
    RobotMessage,
    build_circle_result,
    build_intersection3p_result,
    build_line_result,
)
from robot_session import RobotSession

Point3D = tuple[float, float, float]
TaskExecutionResult = CircleFitResult | LineResult | Point3D

CIRCLE_GROUP = "POINTS"
LINE_GROUPS = ("P1", "P2")
INTERSECTION3P_GROUPS = ("P1", "P2", "P3")


@dataclass(slots=True)
class TaskStartConfig:
    """启动任务时解析出的配置。"""

    task: str
    frame: str
    expected_groups: dict[str, int]
    group_order: tuple[str, ...]

    @property
    def total_expected_points(self) -> int:
        """本任务总共需要的点数。"""
        return sum(self.expected_groups.values())

    def build_ack_fields(self) -> dict[str, str | int]:
        """给 START 的 ACK 补充任务相关字段。"""
        fields: dict[str, str | int] = {
            "TASK": self.task,
            "FRAME": self.frame,
            "TOTAL": self.total_expected_points,
        }
        if self.task == "CIRCLE":
            fields["POINTS"] = self.expected_groups[CIRCLE_GROUP]
            return fields

        for group in self.group_order:
            fields[group] = self.expected_groups[group]
        return fields


def parse_start_config(message: RobotMessage) -> TaskStartConfig:
    """根据 START 消息生成任务配置。"""
    task = message.get_required("TASK").upper()
    frame = message.get_required("FRAME").upper()

    if task == "CIRCLE":
        points = message.get_int("POINTS")
        if points < MIN_POINTS_FOR_CIRCLE:
            raise ProtocolError(f"CIRCLE 至少需要 {MIN_POINTS_FOR_CIRCLE} 个点")
        return TaskStartConfig(
            task=task,
            frame=frame,
            expected_groups={CIRCLE_GROUP: points},
            group_order=(CIRCLE_GROUP,),
        )

    if task == "LINE":
        p1 = message.get_int("P1")
        p2 = message.get_int("P2")
        if p1 < MIN_POINTS_FOR_PLANE:
            raise ProtocolError(f"LINE 的 P1 至少需要 {MIN_POINTS_FOR_PLANE} 个点")
        if p2 < MIN_POINTS_FOR_PLANE:
            raise ProtocolError(f"LINE 的 P2 至少需要 {MIN_POINTS_FOR_PLANE} 个点")
        return TaskStartConfig(
            task=task,
            frame=frame,
            expected_groups={"P1": p1, "P2": p2},
            group_order=LINE_GROUPS,
        )

    if task == "INTERSECTION3P":
        p1 = _parse_optional_int(message, "P1", 3)
        p2 = _parse_optional_int(message, "P2", 3)
        p3 = _parse_optional_int(message, "P3", 3)
        if p1 < MIN_POINTS_FOR_PLANE or p2 < MIN_POINTS_FOR_PLANE or p3 < MIN_POINTS_FOR_PLANE:
            raise ProtocolError(f"INTERSECTION3P 的每组点至少需要 {MIN_POINTS_FOR_PLANE} 个点")
        if p1 + p2 + p3 != EXPECTED_POINT_COUNT:
            raise ProtocolError(
                f"INTERSECTION3P 当前固定需要 {EXPECTED_POINT_COUNT} 个点，"
                f"收到配置 P1={p1}, P2={p2}, P3={p3}"
            )
        return TaskStartConfig(
            task=task,
            frame=frame,
            expected_groups={"P1": p1, "P2": p2, "P3": p3},
            group_order=INTERSECTION3P_GROUPS,
        )

    raise ProtocolError(f"当前不支持的任务类型: {task}")


def resolve_point_group(message: RobotMessage, session: RobotSession) -> str:
    """根据当前任务决定 POINT 属于哪个分组。"""
    if session.task == "CIRCLE":
        raw_group = message.get("GROUP")
        if raw_group is None:
            return CIRCLE_GROUP
        group = raw_group.upper()
        if group != CIRCLE_GROUP:
            raise ProtocolError(f"CIRCLE 任务不支持分组 {group}，只能是 {CIRCLE_GROUP}")
        return group

    raw_group = message.get_required("GROUP")
    group = raw_group.upper()
    if group not in session.expected_groups:
        raise ProtocolError(f"未知分组 {group}，当前任务支持分组: {', '.join(session.group_order)}")
    return group


def execute_task(
    session: RobotSession,
    run_circle: Callable[[list[Point3D]], CircleFitResult],
    run_line: Callable[[list[Point3D], list[Point3D]], LineResult],
    run_intersection3p: Callable[[list[Point3D]], Point3D],
) -> TaskExecutionResult:
    """根据任务类型把点路由到不同测量流程。"""
    grouped_points = session.get_grouped_point_tuples()

    if session.task == "CIRCLE":
        return run_circle(grouped_points[CIRCLE_GROUP])

    if session.task == "LINE":
        return run_line(grouped_points["P1"], grouped_points["P2"])

    if session.task == "INTERSECTION3P":
        points = (
            grouped_points["P1"]
            + grouped_points["P2"]
            + grouped_points["P3"]
        )
        return run_intersection3p(points)

    raise ProtocolError(f"当前不支持的任务类型: {session.task}")


def build_result_message(req: str, session: RobotSession, result: TaskExecutionResult) -> str:
    """根据任务类型构造 RESULT 协议消息。"""
    if session.task == "CIRCLE":
        return build_circle_result(req, result, session.frame)  # type: ignore[arg-type]

    if session.task == "LINE":
        return build_line_result(req, result, session.frame)  # type: ignore[arg-type]

    if session.task == "INTERSECTION3P":
        return build_intersection3p_result(req, result, session.frame)  # type: ignore[arg-type]

    raise ProtocolError(f"当前不支持的任务类型: {session.task}")


def build_result_snapshot(task: str, result: TaskExecutionResult) -> dict:
    """把结果对象转成适合 UI 展示的快照。"""
    if task == "CIRCLE":
        circle = result
        return {
            "task": task,
            "fields": {
                "圆心 X": circle.center[0],
                "圆心 Y": circle.center[1],
                "圆心 Z": circle.center[2],
                "直径": circle.diameter,
                "法向量 X": circle.normal[0],
                "法向量 Y": circle.normal[1],
                "法向量 Z": circle.normal[2],
            },
        }

    if task == "LINE":
        line = result
        return {
            "task": task,
            "fields": {
                "线上点 X": line.point[0],
                "线上点 Y": line.point[1],
                "线上点 Z": line.point[2],
                "方向 X": line.direction[0],
                "方向 Y": line.direction[1],
                "方向 Z": line.direction[2],
            },
        }

    point = result
    return {
        "task": task,
        "fields": {
            "交点 X": point[0],
            "交点 Y": point[1],
            "交点 Z": point[2],
        },
    }


def summarize_result(task: str, result_snapshot: dict) -> str:
    """生成适合任务历史显示的一行摘要。"""
    fields = result_snapshot.get("fields", {})
    if task == "CIRCLE":
        return (
            f"center=({_fmt(fields.get('圆心 X'))}, "
            f"{_fmt(fields.get('圆心 Y'))}, {_fmt(fields.get('圆心 Z'))}), "
            f"D={_fmt(fields.get('直径'))}"
        )

    if task == "LINE":
        return (
            f"point=({_fmt(fields.get('线上点 X'))}, "
            f"{_fmt(fields.get('线上点 Y'))}, {_fmt(fields.get('线上点 Z'))})"
        )

    return (
        f"point=({_fmt(fields.get('交点 X'))}, "
        f"{_fmt(fields.get('交点 Y'))}, {_fmt(fields.get('交点 Z'))})"
    )


def _parse_optional_int(message: RobotMessage, key: str, default: int) -> int:
    """读取一个可选整数，没有时返回默认值。"""
    value = message.get(key)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise ProtocolError(f"字段 {key} 不是有效整数: {value}") from exc


def _fmt(value: object) -> str:
    """把结果值格式化成适合日志的一段文本。"""
    if isinstance(value, (int, float)):
        return f"{float(value):.6f}"
    return "--"
