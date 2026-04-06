"""应用控制器。

这个模块位于 UI 层和业务层之间，负责：
1. 统一调度 PolyWorks 连接与手动测量
2. 管理机器人 TCP 服务端
3. 维护机器人会话、协议日志、任务历史
4. 管理标定变换的求解、保存、加载与激活
5. 为 UI 提供稳定的状态快照
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import logging
import threading
from typing import TypeVar

from app_logger import get_logger
from calibration_service import CalibrationService
from config import (
    PW_FRAME_NAME,
    ROBOT_BASE_FRAME_NAME,
    ROBOT_DEFAULT_FRAME,
    ROBOT_PROTOCOL_HISTORY_LIMIT,
    ROBOT_SERVER_ENABLED,
    ROBOT_SERVER_HOST,
    ROBOT_SERVER_PORT,
    ROBOT_TASK_HISTORY_LIMIT,
)
from exceptions import (
    CalibrationError,
    MeasurementError,
    PolyWorksConnectionError,
    ProtocolError,
    RepositoryError,
    RobotServerError,
    SessionStateError,
    TransformError,
)
from polyworks_com import PolyWorksConnector
from result_types import CircleFitResult, LineResult
from robot_protocol import build_ack, build_error, parse_message
from robot_server import RobotRequestHandler, RobotTCPServer
from robot_session import RobotSession
from services import MeasurementService
from task_router import (
    build_result_message,
    build_result_snapshot,
    execute_task,
    parse_start_config,
    resolve_point_group,
    summarize_result,
)
from transform_models import CalibrationPointPair, CalibrationSolveResult, Point3D, RigidTransform
from transform_repository import TransformRepository
from transform_service import transform_task_result

MeasurementResultT = TypeVar("MeasurementResultT")


class MeasurementController:
    """统一协调 UI、业务层、协议层、标定层和日志。"""

    def __init__(
        self,
        connector: PolyWorksConnector | None = None,
        measurement_service: MeasurementService | None = None,
        transform_repository: TransformRepository | None = None,
        calibration_service: CalibrationService | None = None,
    ) -> None:
        self.logger = get_logger("controller")
        self.connector = connector or PolyWorksConnector()
        self.measurement_service = measurement_service or MeasurementService(self.connector)
        self.transform_repository = transform_repository or TransformRepository()
        self.calibration_service = calibration_service or CalibrationService()

        self._log_callback: Callable[[str], None] | None = None
        self._main_thread_executor: Callable[[Callable[[], object]], object] | None = None
        self._ui_log_queue: deque[str] = deque()
        self._ui_log_lock = threading.Lock()

        self._robot_lock = threading.Lock()
        self._robot_server: RobotTCPServer | None = None
        self._robot_server_thread: threading.Thread | None = None
        self._active_client_label: str | None = None
        self._active_client_id: str = ""
        self._robot_session: RobotSession | None = None
        self._robot_protocol_history: deque[str] = deque(maxlen=ROBOT_PROTOCOL_HISTORY_LIMIT)
        self._robot_task_history: deque[str] = deque(maxlen=ROBOT_TASK_HISTORY_LIMIT)

        self._latest_calibration_result: CalibrationSolveResult | None = None
        self._active_transform: RigidTransform | None = None
        self._load_active_transform_on_startup()

    @property
    def connected(self) -> bool:
        """当前是否已连接到 PolyWorks。"""
        return self.connector.connected

    @property
    def active_transform(self) -> RigidTransform | None:
        """当前激活的坐标变换。"""
        return self._active_transform

    def set_log_callback(self, callback: Callable[[str], None]) -> None:
        """注册 UI 日志回调。"""
        self._log_callback = callback

    def set_main_thread_executor(
        self,
        executor: Callable[[Callable[[], object]], object] | None,
    ) -> None:
        """注册主线程执行器，供后台线程安全调用 PolyWorks。"""
        self._main_thread_executor = executor

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

    def connect_polyworks(self) -> None:
        """连接 PolyWorks。"""
        self.emit_log("正在连接 PolyWorks Inspector...")
        try:
            self.connector.connect()
        except Exception as exc:
            self.logger.exception("连接 PolyWorks 失败")
            self.emit_log(f"连接 PolyWorks 失败: {exc}", logging.ERROR)
            raise PolyWorksConnectionError(f"连接 PolyWorks 失败: {exc}") from exc

        self.emit_log("成功连接到 PolyWorks Inspector")

    def disconnect_polyworks(self) -> None:
        """断开 PolyWorks 连接。"""
        if self.connector.connected:
            self.connector.disconnect()
        self.emit_log("已断开 PolyWorks 连接")

    def run_intersection_measurement(self, points: list[Point3D]) -> Point3D:
        """执行三平面交点测量。"""
        self.emit_log(f"开始执行三平面交点计算，输入点数: {len(points)}")
        try:
            result = self._execute_measurement_task(
                lambda: self.measurement_service.run_measurement(points)
            )
        except Exception as exc:
            self.logger.exception("三平面交点计算失败")
            self.emit_log(f"三平面交点计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"三平面交点计算失败: {exc}") from exc
        finally:
            self.measurement_service.cleanup_result_file()

        self.emit_log(
            f"三平面交点完成: X={result[0]:.6f}, Y={result[1]:.6f}, Z={result[2]:.6f}"
        )
        return result

    def fit_circle_from_points(self, points: list[Point3D]) -> CircleFitResult:
        """执行拟合圆测量。"""
        self.emit_log(f"开始使用 PolyWorks 拟合圆，输入点数: {len(points)}")
        try:
            result = self._execute_measurement_task(
                lambda: self.measurement_service.fit_circle_from_points(points)
            )
        except Exception as exc:
            self.logger.exception("拟合圆计算失败")
            self.emit_log(f"拟合圆计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"拟合圆计算失败: {exc}") from exc

        self.emit_log(
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
        self.emit_log(
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
            self.emit_log(f"两面交线计算失败: {exc}", logging.ERROR)
            raise MeasurementError(f"两面交线计算失败: {exc}") from exc

        self.emit_log(
            "PolyWorks 两面交线完成: "
            f"point=({result.point[0]:.6f}, {result.point[1]:.6f}, {result.point[2]:.6f}), "
            f"direction=({result.direction[0]:.6f}, {result.direction[1]:.6f}, {result.direction[2]:.6f})"
        )
        return result

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

        self.emit_log(
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
            self.emit_log(f"标定求解失败: {exc}", logging.ERROR)
            raise CalibrationError(f"标定求解失败: {exc}") from exc

        self._latest_calibration_result = result
        self.emit_log(
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
            self.emit_log(
                f"已保存并激活变换: {transform.name} ({transform.source_frame}->{transform.target_frame})"
            )
        else:
            self.emit_log(f"已保存变换: {transform.name}")

    def load_transform(self, name: str, set_active: bool = True) -> RigidTransform:
        """读取一个保存过的变换。"""
        transform = self.transform_repository.get_transform(name)
        if transform is None:
            raise RepositoryError(f"未找到名为 {name} 的标定变换")

        if set_active:
            self.transform_repository.set_active_transform(name)
            self._active_transform = transform
            self.emit_log(
                f"已加载并激活变换: {transform.name} ({transform.source_frame}->{transform.target_frame})"
            )
        else:
            self.emit_log(f"已加载变换: {transform.name}")

        return transform

    def list_transform_names(self) -> list[str]:
        """返回全部已保存变换的名称列表。"""
        return [transform.name for transform in self.transform_repository.list_transforms()]

    def get_calibration_ui_snapshot(self) -> dict:
        """获取标定页面需要的状态快照。"""
        transforms = self.transform_repository.list_transforms()
        active_transform = self._active_transform
        latest_result = None if self._latest_calibration_result is None else self._latest_calibration_result.to_snapshot()

        return {
            "active_transform": None if active_transform is None else active_transform.to_snapshot(),
            "available_transforms": [transform.to_snapshot() for transform in transforms],
            "latest_result": latest_result,
        }

    def start_robot_server(self) -> bool:
        """启动正式机器人服务端。"""
        if not ROBOT_SERVER_ENABLED:
            self.emit_log("机器人服务端已在配置中关闭", logging.INFO)
            return False

        with self._robot_lock:
            if self._robot_server is not None:
                return True

            try:
                self._robot_server = RobotTCPServer(
                    (ROBOT_SERVER_HOST, ROBOT_SERVER_PORT),
                    RobotRequestHandler,
                    self,
                )
            except OSError as exc:
                self.logger.exception("启动机器人服务端失败")
                self.emit_log(f"启动机器人服务端失败: {exc}", logging.ERROR)
                self._robot_server = None
                raise RobotServerError(f"启动机器人服务端失败: {exc}") from exc

            self._robot_server_thread = threading.Thread(
                target=self._robot_server.serve_forever,
                name="robot-server",
                daemon=True,
            )
            self._robot_server_thread.start()

        self.emit_log(f"机器人服务端已启动: {ROBOT_SERVER_HOST}:{ROBOT_SERVER_PORT}")
        return True

    def stop_robot_server(self) -> None:
        """停止机器人服务端。"""
        with self._robot_lock:
            server = self._robot_server
            thread = self._robot_server_thread
            self._robot_server = None
            self._robot_server_thread = None
            self._active_client_label = None

        if server is None:
            return

        server.shutdown()
        server.server_close()
        if thread is not None:
            thread.join(timeout=1.0)

        self.emit_log("机器人服务端已停止")

    def start_simulator_listener(self) -> bool:
        """兼容阶段 0 的旧方法名。"""
        return self.start_robot_server()

    def stop_simulator_listener(self) -> None:
        """兼容阶段 0 的旧方法名。"""
        self.stop_robot_server()

    def on_robot_client_connected(self, client_label: str) -> None:
        """记录客户端连接事件。"""
        with self._robot_lock:
            self._active_client_label = client_label
        self.emit_log(f"机器人客户端已连接: {client_label}")

    def on_robot_client_disconnected(self, client_label: str) -> None:
        """记录客户端断开事件。"""
        with self._robot_lock:
            if self._active_client_label == client_label:
                self._active_client_label = None
        self.emit_log(f"机器人客户端已断开: {client_label}")

    def handle_robot_message(self, raw_message: str, client_label: str) -> list[str]:
        """处理一条正式协议消息。"""
        self._record_protocol("IN", raw_message)
        current_req = "0"

        try:
            message = parse_message(raw_message)
            current_req = message.req
            if message.message_type == "HELLO":
                responses = self._handle_hello(message)
            elif message.message_type == "START":
                responses = self._handle_start(message, client_label)
            elif message.message_type == "POINT":
                responses = self._handle_point(message)
            elif message.message_type == "END":
                responses = self._handle_end(message)
            else:  # pragma: no cover
                responses = [build_error("0", "UNKNOWN_MESSAGE", "unknown_message_type")]
        except ProtocolError as exc:
            responses = [build_error(current_req, "PROTOCOL_ERROR", "invalid_message", DETAIL=str(exc))]
            self.emit_log(f"协议错误: {exc}", logging.WARNING)
        except SessionStateError as exc:
            responses = [
                build_error(current_req, "SESSION_ERROR", "invalid_session_state", DETAIL=str(exc))
            ]
            self.emit_log(f"会话状态错误: {exc}", logging.WARNING)

        return self._record_outbound_protocol(responses)

    def get_robot_ui_snapshot(self) -> dict:
        """获取机器人联动页面需要的状态快照。"""
        with self._robot_lock:
            session_snapshot = None if self._robot_session is None else self._robot_session.to_snapshot()
            active_client = self._active_client_label
            protocol_history = list(self._robot_protocol_history)
            task_history = list(self._robot_task_history)
            server_running = self._robot_server is not None

        return {
            "server_running": server_running,
            "host": ROBOT_SERVER_HOST,
            "port": ROBOT_SERVER_PORT,
            "active_client": active_client,
            "default_frame": ROBOT_DEFAULT_FRAME,
            "active_transform": None if self._active_transform is None else self._active_transform.to_snapshot(),
            "session": session_snapshot,
            "protocol_history": protocol_history,
            "task_history": task_history,
        }

    def shutdown(self) -> None:
        """关闭控制器占用的资源。"""
        self.stop_robot_server()
        if self.connector.connected:
            self.connector.disconnect()
            self.emit_log("程序关闭时已断开 PolyWorks 连接")

    @staticmethod
    def format_client_address(address: tuple[str, int]) -> str:
        """格式化客户端地址。"""
        return f"{address[0]}:{address[1]}"

    def _handle_hello(self, message) -> list[str]:
        """处理 HELLO 消息。"""
        client_id = message.get_required("CLIENT")
        with self._robot_lock:
            self._active_client_id = client_id
        self.emit_log(f"收到客户端握手: {client_id}")
        return [build_ack(message.req, "HELLO", CLIENT=client_id)]

    def _handle_start(self, message, client_label: str) -> list[str]:
        """处理 START 消息。"""
        task_config = parse_start_config(message)

        with self._robot_lock:
            if self._robot_session is not None and self._robot_session.state in {
                "RECEIVING",
                "READY",
                "COMPUTING",
            }:
                raise SessionStateError("上一轮任务尚未结束")

            self._robot_session = RobotSession(
                req=message.req,
                task=task_config.task,
                expected_groups=task_config.expected_groups,
                group_order=task_config.group_order,
                frame=task_config.frame,
                client_label=client_label,
                client_id=self._active_client_id,
            )

        self.emit_log(
            f"机器人任务开始: REQ={message.req}, TASK={task_config.task}, "
            f"GROUPS={self._robot_session.describe_progress()}, FRAME={task_config.frame}"
        )
        return [build_ack(message.req, "RECEIVING", **task_config.build_ack_fields())]

    def _handle_point(self, message) -> list[str]:
        """处理 POINT 消息。"""
        with self._robot_lock:
            session = self._require_active_session(message.req)
            group = resolve_point_group(message, session)
            idx = message.get_int("IDX")
            point = (
                message.get_float("X"),
                message.get_float("Y"),
                message.get_float("Z"),
            )
            session.add_point(group, idx, point)
            received_points = session.received_points
            received_groups = session.received_groups
            group_count = received_groups[group]
            state = session.state
            progress_text = session.describe_progress()

        self.emit_log(
            f"机器人点已接收: REQ={message.req}, GROUP={group}, IDX={idx}, "
            f"COUNT={received_points}, PROGRESS={progress_text}"
        )
        return [
            build_ack(
                message.req,
                "POINT_ACCEPTED",
                GROUP=group,
                IDX=idx,
                COUNT=received_points,
                GROUP_COUNT=group_count,
                NEXT_STATE=state,
            )
        ]

    def _handle_end(self, message) -> list[str]:
        """处理 END 消息。"""
        with self._robot_lock:
            session = self._require_active_session(message.req)
            progress_text = session.describe_progress()
            if not session.is_full:
                session.mark_error("NOT_ENOUGH_POINTS", "need_more_points")
                self._robot_task_history.append(self._build_task_history_entry(session))
                return [
                    build_error(
                        message.req,
                        "NOT_ENOUGH_POINTS",
                        "need_more_points",
                        EXPECTED=session.expected_points,
                        RECEIVED=session.received_points,
                        DETAIL=progress_text,
                    )
                ]

            if not self.connected:
                session.mark_error("POLYWORKS_NOT_CONNECTED", "connect_polyworks_first")
                self._robot_task_history.append(self._build_task_history_entry(session))
                return [
                    build_error(
                        message.req,
                        "POLYWORKS_NOT_CONNECTED",
                        "connect_polyworks_first",
                    )
                ]

            session.mark_computing()

        responses = [build_ack(message.req, "COMPUTING", TASK=session.task)]
        try:
            raw_result = execute_task(
                session,
                self.fit_circle_from_points,
                self.intersect_plane_groups,
                self.run_intersection_measurement,
            )
            output_result = self._convert_task_result_to_requested_frame(
                session.task,
                raw_result,
                session.frame,
            )
            result_snapshot = build_result_snapshot(session.task, output_result)
            result_message = build_result_message(message.req, session, output_result)
        except TransformError as exc:
            self.logger.exception("机器人结果坐标变换失败")
            self.emit_log(f"机器人结果坐标变换失败: {exc}", logging.ERROR)
            with self._robot_lock:
                current_session = self._robot_session
                if current_session is not None and current_session.req == message.req:
                    current_session.mark_error("TRANSFORM_ERROR", str(exc))
                    self._robot_task_history.append(self._build_task_history_entry(current_session))
            responses.append(
                build_error(
                    message.req,
                    "TRANSFORM_ERROR",
                    "transform_failed",
                    TASK=session.task,
                    FRAME=session.frame,
                    DETAIL=str(exc),
                )
            )
            return responses
        except Exception as exc:
            self.logger.exception("机器人任务执行失败")
            self.emit_log(f"机器人任务执行失败: {exc}", logging.ERROR)
            with self._robot_lock:
                current_session = self._robot_session
                if current_session is not None and current_session.req == message.req:
                    current_session.mark_error("TASK_FAILED", str(exc))
                    self._robot_task_history.append(self._build_task_history_entry(current_session))
            responses.append(
                build_error(
                    message.req,
                    "TASK_FAILED",
                    "task_execution_failed",
                    TASK=session.task,
                    DETAIL=str(exc),
                )
            )
            return responses

        with self._robot_lock:
            current_session = self._robot_session
            if current_session is not None and current_session.req == message.req:
                current_session.mark_done(result_snapshot)
                self._robot_task_history.append(self._build_task_history_entry(current_session))

        responses.append(result_message)
        return responses

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

    def _convert_task_result_to_requested_frame(
        self,
        task: str,
        result: CircleFitResult | LineResult | Point3D,
        requested_frame: str,
    ) -> CircleFitResult | LineResult | Point3D:
        """把 PolyWorks 结果按请求的坐标系进行转换。"""
        normalized_frame = requested_frame.upper()
        if normalized_frame == PW_FRAME_NAME:
            return result

        if self._active_transform is None:
            raise TransformError("当前没有激活的标定变换，无法返回机器人坐标")

        if self._active_transform.source_frame.upper() != PW_FRAME_NAME:
            raise TransformError(
                f"当前激活变换源坐标系为 {self._active_transform.source_frame}，不是 {PW_FRAME_NAME}"
            )

        if normalized_frame != self._active_transform.target_frame.upper():
            raise TransformError(
                f"当前激活变换输出坐标系为 {self._active_transform.target_frame}，不能返回 {requested_frame}"
            )

        return transform_task_result(task, result, self._active_transform)

    def _require_active_session(self, req: str) -> RobotSession:
        """校验并返回当前活动会话。"""
        if self._robot_session is None:
            raise SessionStateError("当前没有活动会话")
        if self._robot_session.req != req:
            raise SessionStateError(
                f"REQ 不匹配，当前会话 REQ={self._robot_session.req}，收到 REQ={req}"
            )
        return self._robot_session

    def _record_protocol(self, direction: str, message: str) -> None:
        """记录一条协议日志。"""
        with self._robot_lock:
            self._robot_protocol_history.append(f"{direction}: {message}")

    def _record_outbound_protocol(self, responses: list[str]) -> list[str]:
        """记录发出的协议消息。"""
        for response in responses:
            self._record_protocol("OUT", response)
        return responses

    def _build_task_history_entry(self, session: RobotSession) -> str:
        """生成一条任务历史文本。"""
        base = (
            f"{session.created_at} | REQ={session.req} | TASK={session.task} | FRAME={session.frame} | "
            f"STATE={session.state} | PROGRESS={session.describe_progress()}"
        )

        if session.last_result is not None:
            summary = summarize_result(session.task, session.last_result)
            return f"{base} | RESULT={summary}"

        if session.last_error_code or session.last_error_message:
            return (
                f"{base} | ERROR={session.last_error_code or '--'}"
                f" | DETAIL={session.last_error_message or '--'}"
            )

        return base

    def _load_active_transform_on_startup(self) -> None:
        """程序启动时尝试恢复上次激活的变换。"""
        try:
            self._active_transform = self.transform_repository.get_active_transform()
        except Exception as exc:
            self.logger.warning("加载激活变换失败: %s", exc)
            self._active_transform = None
