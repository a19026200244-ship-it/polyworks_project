"""标定求解服务。"""

from __future__ import annotations

from datetime import datetime

from config import MIN_CALIBRATION_POINT_PAIRS, PW_FRAME_NAME, ROBOT_BASE_FRAME_NAME
from exceptions import CalibrationError
from transform_models import (
    CalibrationPointPair,
    CalibrationResidual,
    CalibrationSolveResult,
    Point3D,
    RigidTransform,
)

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    np = None
    HAS_NUMPY = False


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
        source_points = np.array([pair.pw_point for pair in point_pairs], dtype=float)
        target_points = np.array([pair.robot_point for pair in point_pairs], dtype=float)

        self._validate_points(source_points, target_points)

        source_centroid = source_points.mean(axis=0)
        target_centroid = target_points.mean(axis=0)
        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid

        covariance = source_centered.T @ target_centered
        u_matrix, _, v_transpose = np.linalg.svd(covariance)
        rotation = v_transpose.T @ u_matrix.T

        if np.linalg.det(rotation) < 0:
            v_transpose[-1, :] *= -1
            rotation = v_transpose.T @ u_matrix.T

        translation = target_centroid - rotation @ source_centroid

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

        source_rank = np.linalg.matrix_rank(source_points - source_points.mean(axis=0))
        target_rank = np.linalg.matrix_rank(target_points - target_points.mean(axis=0))
        if source_rank < 2 or target_rank < 2:
            raise CalibrationError("标定点退化，至少需要非共线的空间点对")


def _to_point(values) -> Point3D:
    """把 numpy 向量转成三维点。"""
    return float(values[0]), float(values[1]), float(values[2])
