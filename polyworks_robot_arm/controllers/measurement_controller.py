"""应用控制器门面。

这个模块现在只保留“对外统一入口”的职责：
1. 为 UI 暴露稳定的方法名和属性
2. 统一管理日志回调与主线程执行器
3. 组合测量、标定、机器人联动三个协调器

这样可以避免把所有业务都继续堆在一个超大的 controller 文件里。
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import logging
import threading

from polyworks_robot_arm.calibration.calibration_service import CalibrationService
from polyworks_robot_arm.calibration.transform_models import Point3D, RigidTransform
from polyworks_robot_arm.calibration.transform_repository import TransformRepository
from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.result_types import CircleFitResult, LineResult
from polyworks_robot_arm.controllers.calibration_coordinator import CalibrationCoordinator
from polyworks_robot_arm.controllers.measurement_coordinator import MeasurementCoordinator
from polyworks_robot_arm.controllers.robot_link_coordinator import RobotLinkCoordinator
from polyworks_robot_arm.integrations.polyworks_com import PolyWorksConnector
from polyworks_robot_arm.measurement.services import MeasurementService


class MeasurementController:
    """统一协调 UI、测量、标定、机器人联动和日志。"""

    def __init__(
        self,
        connector: PolyWorksConnector | None = None,
        measurement_service: MeasurementService | None = None,
        transform_repository: TransformRepository | None = None,
        calibration_service: CalibrationService | None = None,
    ) -> None:
        self.logger = get_logger("controller")
        self._log_callback: Callable[[str], None] | None = None
        self._ui_log_queue: deque[str] = deque()
        self._ui_log_lock = threading.Lock()

        # 这些底层依赖仍由门面统一持有，方便 UI 和其它模块继续通过 controller 访问。
        self.connector = connector or PolyWorksConnector()
        self.measurement_service = measurement_service or MeasurementService(self.connector)
        self.transform_repository = transform_repository or TransformRepository()
        self.calibration_service = calibration_service or CalibrationService()

        # 1. 手动测量与 PolyWorks 连接
        self.measurement = MeasurementCoordinator(
            connector=self.connector,
            measurement_service=self.measurement_service,
            emit_log=self.emit_log,
        )

        # 2. 标定、变换仓库与坐标变换
        self.calibration = CalibrationCoordinator(
            transform_repository=self.transform_repository,
            calibration_service=self.calibration_service,
            emit_log=self.emit_log,
        )

        # 3. 机器人 Socket、协议、会话和任务回传
        self.robot = RobotLinkCoordinator(
            emit_log=self.emit_log,
            is_polyworks_connected=lambda: self.measurement.connected,
            get_active_transform=lambda: self.calibration.active_transform,
            run_circle=self.measurement.fit_circle_from_points,
            run_line=self.measurement.intersect_plane_groups,
            run_intersection3p=self.measurement.run_intersection_measurement,
            convert_points_between_frames=self.calibration.convert_points_between_frames,
            convert_task_result_between_frames=self.calibration.convert_task_result_between_frames,
        )

    @property
    def connected(self) -> bool:
        """当前是否已连接到 PolyWorks。"""
        return self.measurement.connected

    @property
    def active_transform(self) -> RigidTransform | None:
        """当前激活的坐标变换。"""
        return self.calibration.active_transform

    def set_log_callback(self, callback: Callable[[str], None]) -> None:
        """注册 UI 日志回调。"""
        self._log_callback = callback

    def set_main_thread_executor(
        self,
        executor: Callable[[Callable[[], object]], object] | None,
    ) -> None:
        """注册主线程执行器，供后台线程安全调用 PolyWorks。"""
        self.measurement.set_main_thread_executor(executor)

    def consume_pending_logs(self) -> list[str]:
        """获取等待显示到 UI 的日志。"""
        with self._ui_log_lock:
            messages = list(self._ui_log_queue)
            self._ui_log_queue.clear()
        return messages

    def emit_log(self, message: str, level: int = logging.INFO, notify_ui: bool = True) -> None:
        """统一输出日志。"""
        self.logger.log(level, message)
        if not notify_ui:
            return

        if self._log_callback is not None and threading.current_thread() is threading.main_thread():
            self._log_callback(message)
            return

        with self._ui_log_lock:
            self._ui_log_queue.append(message)

    # ===== 手动测量与 PolyWorks 连接 =====

    def connect_polyworks(self) -> None:
        """连接 PolyWorks。"""
        self.measurement.connect_polyworks()

    def disconnect_polyworks(self) -> None:
        """断开 PolyWorks 连接。"""
        self.measurement.disconnect_polyworks()

    def run_intersection_measurement(self, points: list[Point3D]) -> Point3D:
        """执行三平面交点测量。"""
        return self.measurement.run_intersection_measurement(points)

    def fit_circle_from_points(self, points: list[Point3D]) -> CircleFitResult:
        """执行拟合圆测量。"""
        return self.measurement.fit_circle_from_points(points)

    def intersect_plane_groups(
        self,
        plane1_points: list[Point3D],
        plane2_points: list[Point3D],
    ) -> LineResult:
        """执行两面交线测量。"""
        return self.measurement.intersect_plane_groups(plane1_points, plane2_points)

    # ===== 标定与坐标变换 =====

    def solve_point_pair_calibration(
        self,
        pw_points: list[Point3D],
        robot_points: list[Point3D],
        name: str,
        source_frame: str = "PW",
        target_frame: str = "ROBOT_BASE",
    ):
        """根据两组对应点求解刚体变换。"""
        return self.calibration.solve_point_pair_calibration(
            pw_points,
            robot_points,
            name,
            source_frame=source_frame,
            target_frame=target_frame,
        )

    def save_transform(self, transform: RigidTransform, set_active: bool = True) -> None:
        """保存一个变换，并可选设为激活。"""
        self.calibration.save_transform(transform, set_active=set_active)

    def load_transform(self, name: str, set_active: bool = True) -> RigidTransform:
        """读取一个保存过的变换。"""
        return self.calibration.load_transform(name, set_active=set_active)

    def list_transform_names(self) -> list[str]:
        """返回全部已保存变换的名称列表。"""
        return self.calibration.list_transform_names()

    def get_calibration_ui_snapshot(self) -> dict:
        """获取标定页面需要的状态快照。"""
        return self.calibration.get_calibration_ui_snapshot()

    # ===== 机器人联动 =====

    def start_robot_server(self) -> bool:
        """启动正式机器人服务端。"""
        return self.robot.start_robot_server(self)

    def stop_robot_server(self) -> None:
        """停止机器人服务端。"""
        self.robot.stop_robot_server()

    def start_simulator_listener(self) -> bool:
        """兼容阶段 0 的旧方法名。"""
        return self.start_robot_server()

    def stop_simulator_listener(self) -> None:
        """兼容阶段 0 的旧方法名。"""
        self.stop_robot_server()

    def on_robot_client_connected(self, client_label: str) -> None:
        """记录客户端连接事件。"""
        self.robot.on_robot_client_connected(client_label)

    def on_robot_client_disconnected(self, client_label: str) -> None:
        """记录客户端断开事件。"""
        self.robot.on_robot_client_disconnected(client_label)

    def handle_robot_message(self, raw_message: str, client_label: str) -> list[str]:
        """处理一条正式协议消息。"""
        return self.robot.handle_robot_message(raw_message, client_label)

    def get_robot_ui_snapshot(self) -> dict:
        """获取机器人联动页面需要的状态快照。"""
        return self.robot.get_robot_ui_snapshot()

    @staticmethod
    def format_client_address(address: tuple[str, int]) -> str:
        """格式化客户端地址。"""
        return RobotLinkCoordinator.format_client_address(address)

    def shutdown(self) -> None:
        """关闭控制器占用的资源。"""
        self.robot.stop_robot_server()
        self.measurement.shutdown()
