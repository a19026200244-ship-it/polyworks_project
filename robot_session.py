"""机器人联动会话对象。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

from exceptions import SessionStateError
from result_types import CircleFitResult

Point3D = tuple[float, float, float]


@dataclass(slots=True)
class SessionPoint:
    """单个会话点。"""

    idx: int
    x: float
    y: float
    z: float

    def as_tuple(self) -> Point3D:
        """转换成项目统一使用的三维点格式。"""
        return self.x, self.y, self.z


@dataclass(slots=True)
class RobotSession:
    """单个机器人测量任务会话。

    阶段 1 中一个会话只处理一件事：
    “接收一组用于拟合圆的点，并最终返回圆结果”。
    """

    req: str
    task: str
    expected_points: int
    frame: str
    client_label: str
    client_id: str = ""
    state: str = "RECEIVING"
    points: list[SessionPoint] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    last_result: CircleFitResult | None = None
    last_error_code: str | None = None
    last_error_message: str | None = None

    @property
    def received_points(self) -> int:
        """当前已经收到的点数。"""
        return len(self.points)

    @property
    def is_full(self) -> bool:
        """是否已经收满点。"""
        return self.received_points >= self.expected_points

    def add_point(self, idx: int, point: Point3D) -> None:
        """向会话中追加一个点。"""
        if self.state not in {"RECEIVING", "READY"}:
            raise SessionStateError(f"当前状态 {self.state} 不允许继续接收点")

        if any(existing.idx == idx for existing in self.points):
            raise SessionStateError(f"点序号重复: {idx}")

        expected_idx = self.received_points + 1
        if idx != expected_idx:
            raise SessionStateError(f"点序号必须连续，期望 {expected_idx}，实际 {idx}")

        if self.received_points >= self.expected_points:
            raise SessionStateError("点数已经收满，不能继续追加")

        x, y, z = point
        self.points.append(SessionPoint(idx=idx, x=x, y=y, z=z))
        self.state = "READY" if self.is_full else "RECEIVING"

    def mark_computing(self) -> None:
        """标记为计算中。"""
        self.state = "COMPUTING"

    def mark_done(self, result: CircleFitResult) -> None:
        """标记为完成。"""
        self.last_result = result
        self.last_error_code = None
        self.last_error_message = None
        self.state = "DONE"

    def mark_error(self, code: str, message: str) -> None:
        """标记为错误。"""
        self.last_error_code = code
        self.last_error_message = message
        self.state = "ERROR"

    def get_point_tuples(self) -> list[Point3D]:
        """获取用于测量计算的点列表。"""
        return [point.as_tuple() for point in self.points]

    def to_snapshot(self) -> dict:
        """转成适合界面展示的快照。"""
        return {
            "req": self.req,
            "task": self.task,
            "expected_points": self.expected_points,
            "received_points": self.received_points,
            "frame": self.frame,
            "client_label": self.client_label,
            "client_id": self.client_id,
            "state": self.state,
            "created_at": self.created_at,
            "points": [
                {
                    "idx": point.idx,
                    "x": point.x,
                    "y": point.y,
                    "z": point.z,
                }
                for point in self.points
            ],
            "last_error_code": self.last_error_code,
            "last_error_message": self.last_error_message,
            "last_result": None
            if self.last_result is None
            else {
                "center": self.last_result.center,
                "diameter": self.last_result.diameter,
                "normal": self.last_result.normal,
            },
        }
