"""标定与坐标变换相关数据结构。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

Point3D = tuple[float, float, float]
Matrix3x3 = tuple[Point3D, Point3D, Point3D]

# Stage 3 里几乎所有数据都会围绕这两个基础类型流动:
# Point3D 表示三维点/三维向量，Matrix3x3 表示旋转矩阵 R。
# 先记住它们，后面看标定算法和结果回传会容易很多。


@dataclass(slots=True)
class CalibrationPointPair:
    """一对对应点。"""

    # 同一个物理特征点，在两个坐标系里各有一个坐标值。
    # 这就是“点对标定”的最小数据单元。
    label: str
    pw_point: Point3D
    robot_point: Point3D


@dataclass(slots=True)
class CalibrationResidual:
    """单对点的残差结果。"""

    # 求解出变换后，会把 pw_point 预测到机器人坐标系，
    # predicted_robot_point 就是这个预测值，
    # error 则是预测值和真实 robot_point 的距离误差。
    label: str
    pw_point: Point3D
    robot_point: Point3D
    predicted_robot_point: Point3D
    error: float


@dataclass(slots=True)
class RigidTransform:
    """刚体变换。"""

    # Stage 3 的核心成果就是这个对象。
    # 它描述了一个刚体变换:
    # robot_point = R * pw_point + T
    # 后续机器人联动页如果请求 ROBOT_BASE 坐标，
    # controller 就会拿当前激活的 RigidTransform 来做坐标转换。
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

    # 给仓库层持久化使用。JSON 不支持 tuple，因此这里会转成 list。
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
    # 从本地 JSON 恢复时，再把 list 还原成 tuple，方便数学模块直接使用。
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

    # 给 UI/日志展示用的轻量快照，目的不是持久化，而是便于显示。
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

    # 一次标定求解最终会输出两部分:
    # 1. transform: 后续可复用的刚体变换
    # 2. residuals: 这批样本点的误差明细
    transform: RigidTransform
    residuals: list[CalibrationResidual]

    # UI 通过这个快照直接刷新表格和文本，不需要理解 dataclass 细节。
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
