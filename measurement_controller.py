"""应用控制器。

这个模块位于界面层和业务层之间，负责：
1. 统一调度 PolyWorks 连接与手动测量
2. 管理机器人 TCP 服务端
3. 维护机器人会话状态和协议日志
4. 给 UI 提供稳定的状态快照
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import logging
import threading

from app_logger import get_logger
from config import (
    ROBOT_DEFAULT_FRAME,
    ROBOT_PROTOCOL_HISTORY_LIMIT,
    ROBOT_SERVER_ENABLED,
    ROBOT_SERVER_HOST,
    ROBOT_SERVER_PORT,
)
from exceptions import (
    MeasurementError,
    PolyWorksConnectionError,
    ProtocolError,
    RobotServerError,
    SessionStateError,
)
from polyworks_com import PolyWorksConnector
from result_types import CircleFitResult, LineResult
from robot_protocol import build_ack, build_circle_result, build_error, parse_message
from robot_server import RobotRequestHandler, RobotTCPServer
from robot_session import RobotSession
from services import MeasurementService

Point3D = tuple[float, float, float]


class MeasurementController:
    """统一协调 UI、业务层、协议层和日志。"""

    def __init__(
        self,
        connector: PolyWorksConnector | None = None,
        measurement_service: MeasurementService | None = None,
    ) -> None:
        self.logger = get_logger("controller")
        self.connector = connector or PolyWorksConnector()
        self.measurement_service = measurement_service or MeasurementService(self.connector)

        self._log_callback: Callable[[str], None] | None = None
        self._ui_log_queue: deque[str] = deque()
        self._ui_log_lock = threading.Lock()

        self._robot_lock = threading.Lock()
        self._robot_server: RobotTCPServer | None = None
        self._robot_server_thread: threading.Thread | None = None
        self._active_client_label: str | None = None
        self._active_client_id: str = ""
        self._robot_session: RobotSession | None = None
        self._robot_protocol_history: deque[str] = deque(maxlen=ROBOT_PROTOCOL_HISTORY_LIMIT)

    @property
    def connected(self) -> bool:
        """当前是否已连接到 PolyWorks。"""
        return self.connector.connected

    def set_log_callback(self, callback: Callable[[str], None]) -> None:
        """注册 UI 日志回调。"""
        self._log_callback = callback

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
            result = self.measurement_service.run_measurement(points)
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
            result = self.measurement_service.fit_circle_from_points(points)
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
            result = self.measurement_service.intersect_plane_groups(plane1_points, plane2_points)
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

    # 兼容阶段 0 的旧接口名称。
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
            else:  # pragma: no cover - parse_message 已经保证不会走到这里
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
            server_running = self._robot_server is not None

        return {
            "server_running": server_running,
            "host": ROBOT_SERVER_HOST,
            "port": ROBOT_SERVER_PORT,
            "active_client": active_client,
            "default_frame": ROBOT_DEFAULT_FRAME,
            "session": session_snapshot,
            "protocol_history": protocol_history,
        }

    def shutdown(self) -> None:
        """关闭控制器占用的资源。"""
        self.stop_robot_server()
        if self.connector.connected:
            self.connector.disconnect()
            self.emit_log("程序关闭时已断开 PolyWorks 连接")

    @staticmethod
    def format_client_address(address: tuple[str, int]) -> str:
        """把客户端地址格式化成便于阅读的文本。"""
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
        task = message.get_required("TASK").upper()
        expected_points = message.get_int("POINTS")
        frame = message.get_required("FRAME").upper()

        with self._robot_lock:
            if self._robot_session is not None and self._robot_session.state in {
                "RECEIVING",
                "READY",
                "COMPUTING",
            }:
                raise SessionStateError("上一轮任务尚未结束")

            self._robot_session = RobotSession(
                req=message.req,
                task=task,
                expected_points=expected_points,
                frame=frame,
                client_label=client_label,
                client_id=self._active_client_id,
            )

        self.emit_log(
            f"机器人任务开始: REQ={message.req}, TASK={task}, POINTS={expected_points}, FRAME={frame}"
        )
        return [
            build_ack(
                message.req,
                "RECEIVING",
                TASK=task,
                POINTS=expected_points,
                FRAME=frame,
            )
        ]

    def _handle_point(self, message) -> list[str]:
        """处理 POINT 消息。"""
        with self._robot_lock:
            session = self._require_active_session(message.req)
            idx = message.get_int("IDX")
            point = (
                message.get_float("X"),
                message.get_float("Y"),
                message.get_float("Z"),
            )
            session.add_point(idx, point)
            received_points = session.received_points
            state = session.state

        self.emit_log(
            f"机器人点已接收: REQ={message.req}, IDX={idx}, COUNT={received_points}"
        )
        return [
            build_ack(
                message.req,
                "POINT_ACCEPTED",
                IDX=idx,
                COUNT=received_points,
                NEXT_STATE=state,
            )
        ]

    def _handle_end(self, message) -> list[str]:
        """处理 END 消息。"""
        with self._robot_lock:
            session = self._require_active_session(message.req)
            if session.received_points < session.expected_points:
                session.mark_error("NOT_ENOUGH_POINTS", "need_more_points")
                return [
                    build_error(
                        message.req,
                        "NOT_ENOUGH_POINTS",
                        "need_more_points",
                        EXPECTED=session.expected_points,
                        RECEIVED=session.received_points,
                    )
                ]
            if not self.connected:
                session.mark_error("POLYWORKS_NOT_CONNECTED", "connect_polyworks_first")
                return [
                    build_error(
                        message.req,
                        "POLYWORKS_NOT_CONNECTED",
                        "connect_polyworks_first",
                    )
                ]
            session.mark_computing()
            point_list = session.get_point_tuples()
            frame = session.frame

        responses = [build_ack(message.req, "COMPUTING")]
        try:
            result = self.fit_circle_from_points(point_list)
        except MeasurementError:
            with self._robot_lock:
                current_session = self._robot_session
                if current_session is not None and current_session.req == message.req:
                    current_session.mark_error("FIT_FAILED", "cannot_fit_circle")
            responses.append(build_error(message.req, "FIT_FAILED", "cannot_fit_circle"))
            return responses

        with self._robot_lock:
            current_session = self._robot_session
            if current_session is not None and current_session.req == message.req:
                current_session.mark_done(result)

        responses.append(build_circle_result(message.req, result, frame))
        return responses

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
