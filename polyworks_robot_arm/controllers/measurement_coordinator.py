"""手动测量相关协调器。"""

from __future__ import annotations

from collections.abc import Callable
import logging
import threading
from typing import TypeVar

from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.exceptions import MeasurementError, PolyWorksConnectionError
from polyworks_robot_arm.common.result_types import CircleFitResult, LineResult
from polyworks_robot_arm.integrations.polyworks_com import PolyWorksConnector
from polyworks_robot_arm.measurement.services import MeasurementService

Point3D = tuple[float, float, float]
MeasurementResultT = TypeVar("MeasurementResultT")


class MeasurementCoordinator:
    """负责 PolyWorks 连接和手动测量流程。"""

    def __init__(
        self,
        connector: PolyWorksConnector,
        measurement_service: MeasurementService,
        emit_log: Callable[[str, int, bool], None],
    ) -> None:
        self.logger = get_logger("controller.measurement")
        self.connector = connector
        self.measurement_service = measurement_service
        self._emit_log = emit_log
        self._main_thread_executor: Callable[[Callable[[], object]], object] | None = None

    @property
    def connected(self) -> bool:
        """当前是否已连接到 PolyWorks。"""
        return self.connector.connected

    def set_main_thread_executor(
        self,
        executor: Callable[[Callable[[], object]], object] | None,
    ) -> None:
        """注册主线程执行器，供后台线程安全调用 PolyWorks。"""
        self._main_thread_executor = executor

    def connect_polyworks(self) -> None:
        """连接 PolyWorks。"""
        self._emit_log("正在连接 PolyWorks Inspector...")
        try:
            self.connector.connect()
        except Exception as exc:
            self.logger.exception("连接 PolyWorks 失败")
            self._emit_log(f"连接 PolyWorks 失败: {exc}", logging.ERROR)
            raise PolyWorksConnectionError(f"连接 PolyWorks 失败: {exc}") from exc

        self._emit_log("成功连接到 PolyWorks Inspector")

    def disconnect_polyworks(self) -> None:
        """断开 PolyWorks 连接。"""
        if self.connector.connected:
            self.connector.disconnect()
        self._emit_log("已断开 PolyWorks 连接")

    def run_intersection_measurement(self, points: list[Point3D]) -> Point3D:
        """执行三平面交点测量。"""
        self._emit_log(f"开始执行三平面交点计算，输入点数: {len(points)}")
        try:
            result = self._execute_measurement_task(
                lambda: self.measurement_service.run_measurement(points)
            )
        except Exception as exc:
            self.logger.exception("三平面交点计算失败")
            self._emit_log(f"三平面交点计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"三平面交点计算失败: {exc}") from exc
        finally:
            self.measurement_service.cleanup_result_file()

        self._emit_log(
            f"三平面交点完成: X={result[0]:.6f}, Y={result[1]:.6f}, Z={result[2]:.6f}"
        )
        return result

    def fit_circle_from_points(self, points: list[Point3D]) -> CircleFitResult:
        """执行拟合圆测量。"""
        self._emit_log(f"开始使用 PolyWorks 拟合圆，输入点数: {len(points)}")
        try:
            result = self._execute_measurement_task(
                lambda: self.measurement_service.fit_circle_from_points(points)
            )
        except Exception as exc:
            self.logger.exception("拟合圆计算失败")
            self._emit_log(f"拟合圆计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"拟合圆计算失败: {exc}") from exc

        self._emit_log(
            "PolyWorks 拟合圆完成: "
            f"center=({result.center[0]:.6f}, {result.center[1]:.6f}, {result.center[2]:.6f}), "
            f"diameter={result.diameter:.6f}, "
            f"normal=({result.normal[0]:.6f}, {result.normal[1]:.6f}, {result.normal[2]:.6f})"
        )
        return result

    def intersect_plane_groups(
        self,
        plane1_points: list[Point3D],
        plane2_points: list[Point3D],
    ) -> LineResult:
        """执行两面交线测量。"""
        self._emit_log(
            "开始使用 PolyWorks 计算两面交线，"
            f"平面1点数: {len(plane1_points)}，平面2点数: {len(plane2_points)}"
        )
        try:
            result = self._execute_measurement_task(
                lambda: self.measurement_service.intersect_plane_groups(
                    plane1_points,
                    plane2_points,
                )
            )
        except Exception as exc:
            self.logger.exception("两面交线计算失败")
            self._emit_log(f"两面交线计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"两面交线计算失败: {exc}") from exc

        self._emit_log(
            "PolyWorks 两面交线完成: "
            f"point=({result.point[0]:.6f}, {result.point[1]:.6f}, {result.point[2]:.6f}), "
            f"direction=({result.direction[0]:.6f}, {result.direction[1]:.6f}, {result.direction[2]:.6f})"
        )
        return result

    def shutdown(self) -> None:
        """关闭测量协调器持有的资源。"""
        if self.connector.connected:
            self.connector.disconnect()
            self._emit_log("程序关闭时已断开 PolyWorks 连接")

    def _execute_measurement_task(
        self,
        task: Callable[[], MeasurementResultT],
    ) -> MeasurementResultT:
        """确保 PolyWorks 调用始终运行在创建 COM 对象的主线程。"""
        if threading.current_thread() is threading.main_thread():
            return task()

        if self._main_thread_executor is None:
            raise RuntimeError("未配置主线程执行器，后台线程不能直接调用 PolyWorks")

        return self._main_thread_executor(task)
