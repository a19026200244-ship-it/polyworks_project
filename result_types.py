"""测量结果数据结构。

这个模块只负责定义项目里会返回给界面层的结果类型，
不再包含任何本地几何计算逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass

Point3D = tuple[float, float, float]


@dataclass
class CircleFitResult:
    """空间圆拟合结果。"""

    center: Point3D
    diameter: float
    normal: Point3D
    radius: float
    axis_u: Point3D
    axis_v: Point3D


@dataclass
class LineResult:
    """两平面交线结果。"""

    point: Point3D
    direction: Point3D
