"""坐标变换服务。"""

from __future__ import annotations

from polyworks_robot_arm.calibration.transform_models import Matrix3x3, Point3D, RigidTransform
from polyworks_robot_arm.common.exceptions import TransformError
from polyworks_robot_arm.common.result_types import CircleFitResult, LineResult

# 这个文件只做一件事: “把已经求好的变换应用到点、向量和结果上”。
# 它不负责求解 R/T，也不关心 UI、Socket、PolyWorks，
# 所以可以把它看成 Stage 3 里最纯粹的坐标转换执行层。


def transform_point(point: Point3D, transform: RigidTransform) -> Point3D:
    """把一个点从源坐标系变换到目标坐标系。"""
    rotation = transform.rotation
    translation = transform.translation
    x, y, z = point

    # 点有绝对位置，所以要经过“旋转 + 平移”。
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + translation[0],
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + translation[1],
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + translation[2],
    )


def transform_vector(vector: Point3D, transform: RigidTransform) -> Point3D:
    """把一个方向向量进行旋转变换。"""
    rotation = transform.rotation
    x, y, z = vector

    # 向量只表示“方向”，不表示“位置”，所以不能加平移 T。
    # 这一点对圆法向量、直线方向向量尤其重要。
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z,
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z,
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z,
    )


# 这是单点变换的批量版本，方便后续调试和扩展。
def transform_points(points: list[Point3D], transform: RigidTransform) -> list[Point3D]:
    """批量变换点。"""
    return [transform_point(point, transform) for point in points]


# Stage 3 当前保存的是一个“已求好的刚体变换”，例如 PW -> ROBOT_BASE。
# 当机器人发来的点已经是 ROBOT_BASE 时，想要送进 PolyWorks，就必须用它的逆变换。
def invert_transform(transform: RigidTransform) -> RigidTransform:
    """把一个刚体变换反向，得到目标坐标系 -> 源坐标系。"""
    inverse_rotation = _transpose_matrix(transform.rotation)
    tx, ty, tz = transform.translation
    inverse_translation = (
        -(
            inverse_rotation[0][0] * tx
            + inverse_rotation[0][1] * ty
            + inverse_rotation[0][2] * tz
        ),
        -(
            inverse_rotation[1][0] * tx
            + inverse_rotation[1][1] * ty
            + inverse_rotation[1][2] * tz
        ),
        -(
            inverse_rotation[2][0] * tx
            + inverse_rotation[2][1] * ty
            + inverse_rotation[2][2] * tz
        ),
    )

    notes = f"Inverse of {transform.name}"
    if transform.notes:
        notes = f"{notes}; {transform.notes}"

    return RigidTransform(
        name=f"{transform.name}__inverse",
        source_frame=transform.target_frame,
        target_frame=transform.source_frame,
        rotation=inverse_rotation,
        translation=inverse_translation,
        point_count=transform.point_count,
        rms_error=transform.rms_error,
        max_error=transform.max_error,
        created_at=transform.created_at,
        notes=notes,
    )


# controller 不应该自己去猜“该正着用变换，还是反着用变换”。
# 这里统一根据源/目标坐标系关系，返回应该使用的那个变换对象。
def resolve_transform_between_frames(
    source_frame: str,
    target_frame: str,
    base_transform: RigidTransform,
) -> RigidTransform | None:
    """根据源/目标坐标系，解析该使用正向还是逆向变换。"""
    normalized_source = source_frame.upper()
    normalized_target = target_frame.upper()
    if normalized_source == normalized_target:
        return None

    base_source = base_transform.source_frame.upper()
    base_target = base_transform.target_frame.upper()
    if normalized_source == base_source and normalized_target == base_target:
        return base_transform

    if normalized_source == base_target and normalized_target == base_source:
        return invert_transform(base_transform)

    raise TransformError(
        "当前激活变换不支持该坐标转换: "
        f"{source_frame} -> {target_frame}，"
        f"可用方向只有 {base_transform.source_frame} -> {base_transform.target_frame}"
    )


# 圆结果里有三类数据:
# 1. center 是点，要做旋转+平移
# 2. normal / axis_u / axis_v 是方向，只做旋转
# 3. diameter / radius 是标量，不受坐标变换影响
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


# 直线同理:
# point 是线上的一个位置点，需要旋转+平移；
# direction 只是方向，需要旋转。
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
    # controller 不需要知道每种结果对象内部长什么样，
    # 只需要把 task 和 result 交给这里统一分发即可。
    if normalized_task == "CIRCLE":
        return transform_circle_result(result, transform)  # type: ignore[arg-type]
    if normalized_task == "LINE":
        return transform_line_result(result, transform)  # type: ignore[arg-type]
    # 当前其余情况按三维点处理，对应三平面交点结果。
    return transform_point(result, transform)  # type: ignore[arg-type]


# 旋转矩阵的逆等于转置，这是刚体旋转里非常关键的性质。
def _transpose_matrix(matrix: Matrix3x3) -> Matrix3x3:
    """转置 3x3 旋转矩阵。"""
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )
