"""标定与坐标变换协调器。"""

from __future__ import annotations

import logging

from polyworks_robot_arm.calibration.calibration_service import CalibrationService
from polyworks_robot_arm.calibration.transform_models import (
    CalibrationPointPair,
    CalibrationSolveResult,
    Point3D,
    RigidTransform,
)
from polyworks_robot_arm.calibration.transform_repository import TransformRepository
from polyworks_robot_arm.calibration.transform_service import (
    resolve_transform_between_frames,
    transform_points,
    transform_task_result,
)
from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.config import PW_FRAME_NAME, ROBOT_BASE_FRAME_NAME
from polyworks_robot_arm.common.exceptions import CalibrationError, RepositoryError, TransformError
from polyworks_robot_arm.common.result_types import CircleFitResult, LineResult


class CalibrationCoordinator:
    """负责标定求解、变换仓库和坐标转换。"""

    def __init__(
        self,
        transform_repository: TransformRepository,
        calibration_service: CalibrationService,
        emit_log,
    ) -> None:
        self.logger = get_logger("controller.calibration")
        self.transform_repository = transform_repository
        self.calibration_service = calibration_service
        self._emit_log = emit_log
        self._latest_calibration_result: CalibrationSolveResult | None = None
        self._active_transform: RigidTransform | None = None
        self._load_active_transform_on_startup()

    @property
    def active_transform(self) -> RigidTransform | None:
        """当前激活的坐标变换。"""
        return self._active_transform

    def solve_point_pair_calibration(
        self,
        pw_points: list[Point3D],
        robot_points: list[Point3D],
        name: str,
        source_frame: str = PW_FRAME_NAME,
        target_frame: str = ROBOT_BASE_FRAME_NAME,
    ) -> CalibrationSolveResult:
        """根据两组对应点求解刚体变换。"""
        if len(pw_points) != len(robot_points):
            raise CalibrationError(
                f"PolyWorks 点数量({len(pw_points)})和机器人点数量({len(robot_points)})不一致"
            )

        pairs = [
            CalibrationPointPair(
                label=f"P{index}",
                pw_point=pw_point,
                robot_point=robot_point,
            )
            for index, (pw_point, robot_point) in enumerate(zip(pw_points, robot_points), start=1)
        ]

        self._emit_log(
            f"开始求解点对标定: name={name or '--'}, point_pairs={len(pairs)}, "
            f"{source_frame}->{target_frame}"
        )
        try:
            result = self.calibration_service.solve_point_pair_calibration(
                pairs,
                name=name,
                source_frame=source_frame,
                target_frame=target_frame,
            )
        except CalibrationError:
            raise
        except Exception as exc:
            self.logger.exception("标定求解失败")
            self._emit_log(f"标定求解失败: {exc}", logging.ERROR)
            raise CalibrationError(f"标定求解失败: {exc}") from exc

        self._latest_calibration_result = result
        self._emit_log(
            f"标定求解完成: name={result.transform.name}, "
            f"RMS={result.transform.rms_error:.6f}, MAX={result.transform.max_error:.6f}"
        )
        return result

    def save_transform(self, transform: RigidTransform, set_active: bool = True) -> None:
        """保存一个变换，并可选设为激活。"""
        try:
            self.transform_repository.save_transform(transform, set_active=set_active)
        except RepositoryError:
            raise
        except Exception as exc:
            self.logger.exception("保存标定变换失败")
            raise RepositoryError(f"保存标定变换失败: {exc}") from exc

        if set_active:
            self._active_transform = transform
            self._emit_log(
                f"已保存并激活变换: {transform.name} ({transform.source_frame}->{transform.target_frame})"
            )
        else:
            self._emit_log(f"已保存变换: {transform.name}")

    def load_transform(self, name: str, set_active: bool = True) -> RigidTransform:
        """读取一个保存过的变换。"""
        transform = self.transform_repository.get_transform(name)
        if transform is None:
            raise RepositoryError(f"未找到名为 {name} 的标定变换")

        if set_active:
            self.transform_repository.set_active_transform(name)
            self._active_transform = transform
            self._emit_log(
                f"已加载并激活变换: {transform.name} ({transform.source_frame}->{transform.target_frame})"
            )
        else:
            self._emit_log(f"已加载变换: {transform.name}")

        return transform

    def list_transform_names(self) -> list[str]:
        """返回全部已保存变换的名称列表。"""
        return [transform.name for transform in self.transform_repository.list_transforms()]

    def get_calibration_ui_snapshot(self) -> dict:
        """获取标定页面需要的状态快照。"""
        transforms = self.transform_repository.list_transforms()
        latest_result = (
            None
            if self._latest_calibration_result is None
            else self._latest_calibration_result.to_snapshot()
        )
        return {
            "active_transform": None if self._active_transform is None else self._active_transform.to_snapshot(),
            "available_transforms": [transform.to_snapshot() for transform in transforms],
            "latest_result": latest_result,
        }

    def convert_points_between_frames(
        self,
        points: list[Point3D],
        source_frame: str,
        target_frame: str,
    ) -> list[Point3D]:
        """批量把点从一个坐标系转换到另一个坐标系。"""
        normalized_source = source_frame.upper()
        normalized_target = target_frame.upper()
        if normalized_source == normalized_target:
            return points

        if self._active_transform is None:
            raise TransformError(
                f"当前没有激活的标定变换，无法把点从 {source_frame} 转换到 {target_frame}"
            )

        transform = resolve_transform_between_frames(
            normalized_source,
            normalized_target,
            self._active_transform,
        )
        if transform is None:
            return points

        return transform_points(points, transform)

    def convert_task_result_between_frames(
        self,
        task: str,
        result: CircleFitResult | LineResult | Point3D,
        source_frame: str,
        target_frame: str,
    ) -> CircleFitResult | LineResult | Point3D:
        """把测量结果从一个坐标系转换到另一个坐标系。"""
        normalized_source = source_frame.upper()
        normalized_target = target_frame.upper()
        if normalized_source == normalized_target:
            return result

        if self._active_transform is None:
            raise TransformError(
                f"当前没有激活的标定变换，无法把结果从 {source_frame} 转换到 {target_frame}"
            )

        transform = resolve_transform_between_frames(
            normalized_source,
            normalized_target,
            self._active_transform,
        )
        if transform is None:
            return result

        return transform_task_result(task, result, transform)

    def _load_active_transform_on_startup(self) -> None:
        """程序启动时尝试恢复上次激活的变换。"""
        try:
            self._active_transform = self.transform_repository.get_active_transform()
        except Exception as exc:
            self.logger.warning("加载激活变换失败: %s", exc)
            self._active_transform = None
