"""标定与坐标变换相关数据结构。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

Point3D = tuple[float, float, float]
Matrix3x3 = tuple[Point3D, Point3D, Point3D]


@dataclass(slots=True)
class CalibrationPointPair:
    """一对对应点。"""

    label: str
    pw_point: Point3D
    robot_point: Point3D


@dataclass(slots=True)
class CalibrationResidual:
    """单对点的残差结果。"""

    label: str
    pw_point: Point3D
    robot_point: Point3D
    predicted_robot_point: Point3D
    error: float


@dataclass(slots=True)
class RigidTransform:
    """刚体变换。"""

    name: str
    source_frame: str
    target_frame: str
    rotation: Matrix3x3
    translation: Point3D
    point_count: int
    rms_error: float
    max_error: float
    created_at: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    notes: str = ""

    def to_dict(self) -> dict:
        """转成可序列化字典。"""
        return {
            "name": self.name,
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "rotation": [list(row) for row in self.rotation],
            "translation": list(self.translation),
            "point_count": self.point_count,
            "rms_error": self.rms_error,
            "max_error": self.max_error,
            "created_at": self.created_at,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict) -> RigidTransform:
        """从字典恢复对象。"""
        rotation_rows = data.get("rotation", [])
        translation = data.get("translation", [0.0, 0.0, 0.0])
        return cls(
            name=str(data.get("name", "")),
            source_frame=str(data.get("source_frame", "PW")),
            target_frame=str(data.get("target_frame", "ROBOT_BASE")),
            rotation=tuple(tuple(float(value) for value in row) for row in rotation_rows),  # type: ignore[arg-type]
            translation=tuple(float(value) for value in translation),  # type: ignore[arg-type]
            point_count=int(data.get("point_count", 0)),
            rms_error=float(data.get("rms_error", 0.0)),
            max_error=float(data.get("max_error", 0.0)),
            created_at=str(data.get("created_at", "")),
            notes=str(data.get("notes", "")),
        )

    def to_snapshot(self) -> dict:
        """转成适合 UI 展示的快照。"""
        return {
            "name": self.name,
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "rotation": [list(row) for row in self.rotation],
            "translation": list(self.translation),
            "point_count": self.point_count,
            "rms_error": self.rms_error,
            "max_error": self.max_error,
            "created_at": self.created_at,
            "notes": self.notes,
        }


@dataclass(slots=True)
class CalibrationSolveResult:
    """一次标定求解结果。"""

    transform: RigidTransform
    residuals: list[CalibrationResidual]

    def to_snapshot(self) -> dict:
        """转成适合 UI 展示的快照。"""
        return {
            "transform": self.transform.to_snapshot(),
            "residuals": [
                {
                    "label": residual.label,
                    "pw_point": list(residual.pw_point),
                    "robot_point": list(residual.robot_point),
                    "predicted_robot_point": list(residual.predicted_robot_point),
                    "error": residual.error,
                }
                for residual in self.residuals
            ],
        }
