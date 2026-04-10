"""机器人联动会话对象。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

from polyworks_robot_arm.common.exceptions import SessionStateError

Point3D = tuple[float, float, float]


@dataclass(slots=True)
class SessionPoint:
    """一次任务中的单个点。"""

    group: str
    idx: int
    x: float
    y: float
    z: float

    def as_tuple(self) -> Point3D:
        """转换成统一的三维点格式。"""
        return self.x, self.y, self.z


@dataclass(slots=True)
class RobotSession:
    """单个机器人任务的会话状态。"""

    req: str
    task: str
    expected_groups: dict[str, int]
    group_order: tuple[str, ...]
    frame: str
    client_label: str
    client_id: str = ""
    state: str = "RECEIVING"
    points: list[SessionPoint] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    last_result: dict | None = None
    last_error_code: str | None = None
    last_error_message: str | None = None

    @property
    def expected_points(self) -> int:
        """总目标点数。"""
        return sum(self.expected_groups.values())

    @property
    def received_points(self) -> int:
        """当前已接收的总点数。"""
        return len(self.points)

    @property
    def received_groups(self) -> dict[str, int]:
        """按分组统计已接收点数。"""
        counts = {group: 0 for group in self.group_order}
        for point in self.points:
            counts[point.group] += 1
        return counts

    @property
    def is_full(self) -> bool:
        """是否已收满所有分组的点。"""
        if self.received_points != self.expected_points:
            return False
        counts = self.received_groups
        return all(counts[group] == self.expected_groups[group] for group in self.group_order)

    def add_point(self, group: str, idx: int, point: Point3D) -> None:
        """向当前会话追加一个点。"""
        if self.state not in {"RECEIVING", "READY"}:
            raise SessionStateError(f"当前状态 {self.state} 不允许继续接收点")

        normalized_group = group.upper()
        if normalized_group not in self.expected_groups:
            raise SessionStateError(f"未知点分组: {normalized_group}")

        group_points = [item for item in self.points if item.group == normalized_group]
        if any(item.idx == idx for item in group_points):
            raise SessionStateError(f"分组 {normalized_group} 中点序号重复: {idx}")

        expected_idx = len(group_points) + 1
        if idx != expected_idx:
            raise SessionStateError(
                f"分组 {normalized_group} 的点序号必须连续，期望 {expected_idx}，实际 {idx}"
            )

        if len(group_points) >= self.expected_groups[normalized_group]:
            raise SessionStateError(f"分组 {normalized_group} 已收满，不能继续追加点")

        x, y, z = point
        self.points.append(
            SessionPoint(group=normalized_group, idx=idx, x=x, y=y, z=z)
        )
        self.state = "READY" if self.is_full else "RECEIVING"

    def mark_computing(self) -> None:
        """标记为计算中。"""
        self.state = "COMPUTING"

    def mark_done(self, result_snapshot: dict) -> None:
        """标记为完成。"""
        self.last_result = result_snapshot
        self.last_error_code = None
        self.last_error_message = None
        self.state = "DONE"

    def mark_error(self, code: str, message: str) -> None:
        """标记为错误。"""
        self.last_error_code = code
        self.last_error_message = message
        self.state = "ERROR"

    def get_group_point_tuples(self, group: str) -> list[Point3D]:
        """获取某个分组的点列表。"""
        normalized_group = group.upper()
        return [point.as_tuple() for point in self.points if point.group == normalized_group]

    def get_grouped_point_tuples(self) -> dict[str, list[Point3D]]:
        """获取全部分组点。"""
        return {
            group: self.get_group_point_tuples(group)
            for group in self.group_order
        }

    def describe_progress(self) -> str:
        """返回类似 `P1=3/4, P2=4/4` 的进度文本。"""
        counts = self.received_groups
        return ", ".join(
            f"{group}={counts[group]}/{self.expected_groups[group]}"
            for group in self.group_order
        )

    def to_snapshot(self) -> dict:
        """转换成适合 UI 展示的快照。"""
        received_groups = self.received_groups
        return {
            "req": self.req,
            "task": self.task,
            "expected_points": self.expected_points,
            "received_points": self.received_points,
            "expected_groups": dict(self.expected_groups),
            "received_groups": received_groups,
            "group_order": list(self.group_order),
            "progress_text": self.describe_progress(),
            "frame": self.frame,
            "client_label": self.client_label,
            "client_id": self.client_id,
            "state": self.state,
            "created_at": self.created_at,
            "points": [
                {
                    "group": point.group,
                    "idx": point.idx,
                    "x": point.x,
                    "y": point.y,
                    "z": point.z,
                }
                for point in self.points
            ],
            "last_error_code": self.last_error_code,
            "last_error_message": self.last_error_message,
            "last_result": self.last_result,
        }

