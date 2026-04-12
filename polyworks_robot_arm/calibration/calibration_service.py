"""标定求解服务。"""

from __future__ import annotations

from datetime import datetime

from polyworks_robot_arm.calibration.transform_models import (
    CalibrationPointPair,
    CalibrationResidual,
    CalibrationSolveResult,
    Point3D,
    RigidTransform,
)
from polyworks_robot_arm.common.config import (
    MIN_CALIBRATION_POINT_PAIRS,
    PW_FRAME_NAME,
    ROBOT_BASE_FRAME_NAME,
)
from polyworks_robot_arm.common.exceptions import CalibrationError

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    np = None
    HAS_NUMPY = False

# Stage 3 的核心算法依赖 numpy 做矩阵运算。
# 这里显式暴露 HAS_NUMPY，方便 UI 在缺少依赖时提前提示用户。


class CalibrationService:
    """通过点对求解刚体变换。"""

    def solve_point_pair_calibration(
        self,
        point_pairs: list[CalibrationPointPair],
        name: str,
        source_frame: str = PW_FRAME_NAME,
        target_frame: str = ROBOT_BASE_FRAME_NAME,
    ) -> CalibrationSolveResult:
        """利用对应点求解 PW -> ROBOT 的刚体变换。"""
        if not HAS_NUMPY:
            raise CalibrationError("缺少 numpy，请先安装后再进行标定求解")

        if len(point_pairs) < MIN_CALIBRATION_POINT_PAIRS:
            raise CalibrationError(
                f"点对标定至少需要 {MIN_CALIBRATION_POINT_PAIRS} 对点，当前只有 {len(point_pairs)} 对"
            )

        transform_name = name.strip() or datetime.now().strftime("transform_%Y%m%d_%H%M%S")
        # 这两组数组分别表示“同一批特征点”在两个坐标系下的坐标。
        # 这里最重要的前提就是: 索引必须一一对应，不能乱序。
        source_points = np.array([pair.pw_point for pair in point_pairs], dtype=float)
        target_points = np.array([pair.robot_point for pair in point_pairs], dtype=float)

        self._validate_points(source_points, target_points)

        # 第 1 步: 求两组点各自的质心。
        # 这样做是为了先把整体平移量分离出去，后面更容易求旋转。
        source_centroid = source_points.mean(axis=0)
        target_centroid = target_points.mean(axis=0)
        # 第 2 步: 把两组点都移动到各自“以质心为原点”的局部坐标系。
        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid

        # 第 3 步: 构造协方差矩阵，并通过 SVD 求最优旋转矩阵。
        # 这一步本质上是在找一个最合适的 R，让 PW 点云尽量贴近 ROBOT 点云。
        covariance = source_centered.T @ target_centered
        u_matrix, _, v_transpose = np.linalg.svd(covariance)
        rotation = v_transpose.T @ u_matrix.T

        # 如果 det(R) < 0，说明得到的是“带镜像翻转”的矩阵，
        # 不是合法的刚体旋转，所以这里要纠正回来。
        if np.linalg.det(rotation) < 0:
            v_transpose[-1, :] *= -1
            rotation = v_transpose.T @ u_matrix.T

        # 第 4 步: 在旋转确定后，平移 T 直接由两个质心关系得到。
        translation = target_centroid - rotation @ source_centroid

        # 第 5 步: 用求出来的 R/T 回算一遍所有点，得到逐点误差。
        predicted_points = (rotation @ source_points.T).T + translation
        errors = np.linalg.norm(predicted_points - target_points, axis=1)

        residuals = [
            CalibrationResidual(
                label=pair.label,
                pw_point=pair.pw_point,
                robot_point=pair.robot_point,
                predicted_robot_point=_to_point(predicted_points[index]),
                error=float(errors[index]),
            )
            for index, pair in enumerate(point_pairs)
        ]

        transform = RigidTransform(
            name=transform_name,
            source_frame=source_frame,
            target_frame=target_frame,
            # rotation 和 translation 就是后续结果回传时真正要复用的数据。
            rotation=tuple(_to_point(row) for row in rotation),  # type: ignore[arg-type]
            translation=_to_point(translation),
            point_count=len(point_pairs),
            rms_error=float(np.sqrt(np.mean(errors ** 2))),
            max_error=float(errors.max()),
        )

        return CalibrationSolveResult(transform=transform, residuals=residuals)

    @staticmethod
    def _validate_points(source_points, target_points) -> None:
        """校验点对是否足以求解稳定变换。"""
        if source_points.shape != target_points.shape:
            raise CalibrationError("PolyWorks 点和机器人点数量不一致")

        if source_points.shape[1] != 3:
            raise CalibrationError("标定点必须是三维点")

        # rank 用来判断点集是否退化。
        # 如果点几乎全落在一条线上，那么空间旋转无法稳定求解，
        # 算法虽然可能跑完，但结果会不可靠，所以这里提前拦住。
        source_rank = np.linalg.matrix_rank(source_points - source_points.mean(axis=0))
        target_rank = np.linalg.matrix_rank(target_points - target_points.mean(axis=0))
        if source_rank < 2 or target_rank < 2:
            raise CalibrationError("标定点退化，至少需要非共线的空间点对")

# 统一把 numpy 向量转换成普通 float tuple，方便 dataclass、JSON、UI 复用。
def _to_point(values) -> Point3D:
    """把 numpy 向量转成三维点。"""
    return float(values[0]), float(values[1]), float(values[2])