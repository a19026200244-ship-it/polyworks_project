"""测量结果数据结构。

这个模块只负责定义项目里会返回给界面层的结果类型，
不再包含任何本地几何计算逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass

# 统一使用 `(x, y, z)` 表示一个三维点。
Point3D = tuple[float, float, float]


@dataclass
class CircleFitResult:
    """空间圆拟合结果。

    这个类只负责“装结果”：
    - center: 圆心
    - diameter: 直径
    - normal: 法向量
    - radius: 半径
    - axis_u / axis_v: 圆所在平面内的两个局部方向
    """

    center: Point3D
    diameter: float
    normal: Point3D
    radius: float
    axis_u: Point3D
    axis_v: Point3D


@dataclass
class LineResult:
    """两平面交线结果。

    - point: 交线上的一个参考点
    - direction: 交线方向向量
    """

    point: Point3D
    direction: Point3D
