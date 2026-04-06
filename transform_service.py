"""坐标变换服务。"""

from __future__ import annotations

from result_types import CircleFitResult, LineResult
from transform_models import Point3D, RigidTransform


def transform_point(point: Point3D, transform: RigidTransform) -> Point3D:
    """把一个点从源坐标系变换到目标坐标系。"""
    rotation = transform.rotation
    translation = transform.translation
    x, y, z = point

    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + translation[0],
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + translation[1],
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + translation[2],
    )


def transform_vector(vector: Point3D, transform: RigidTransform) -> Point3D:
    """把一个方向向量进行旋转变换。"""
    rotation = transform.rotation
    x, y, z = vector

    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z,
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z,
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z,
    )


def transform_points(points: list[Point3D], transform: RigidTransform) -> list[Point3D]:
    """批量变换点。"""
    return [transform_point(point, transform) for point in points]


def transform_circle_result(result: CircleFitResult, transform: RigidTransform) -> CircleFitResult:
    """变换拟合圆结果。"""
    return CircleFitResult(
        center=transform_point(result.center, transform),
        diameter=result.diameter,
        normal=transform_vector(result.normal, transform),
        radius=result.radius,
        axis_u=transform_vector(result.axis_u, transform),
        axis_v=transform_vector(result.axis_v, transform),
    )


def transform_line_result(result: LineResult, transform: RigidTransform) -> LineResult:
    """变换两面交线结果。"""
    return LineResult(
        point=transform_point(result.point, transform),
        direction=transform_vector(result.direction, transform),
    )


def transform_task_result(
    task: str,
    result: CircleFitResult | LineResult | Point3D,
    transform: RigidTransform,
) -> CircleFitResult | LineResult | Point3D:
    """根据任务类型变换结果对象。"""
    normalized_task = task.upper()
    if normalized_task == "CIRCLE":
        return transform_circle_result(result, transform)  # type: ignore[arg-type]
    if normalized_task == "LINE":
        return transform_line_result(result, transform)  # type: ignore[arg-type]
    return transform_point(result, transform)  # type: ignore[arg-type]
