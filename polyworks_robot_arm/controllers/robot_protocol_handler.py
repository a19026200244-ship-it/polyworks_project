"""机器人协议处理器。"""

from __future__ import annotations

import logging

from polyworks_robot_arm.calibration.transform_models import Point3D
from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.config import PW_FRAME_NAME
from polyworks_robot_arm.common.exceptions import ProtocolError, SessionStateError, TransformError
from polyworks_robot_arm.robot.robot_protocol import build_ack, build_error, parse_message
from polyworks_robot_arm.robot.robot_session import RobotSession
from polyworks_robot_arm.robot.task_router import (
    build_result_message,
    build_result_snapshot,
    execute_task_from_grouped_points,
    parse_start_config,
    resolve_point_group,
    summarize_result,
)

from polyworks_robot_arm.controllers.robot_session_store import RobotSessionStore


class RobotProtocolHandler:
    """负责协议解析、会话流转和任务回传。"""

    def __init__(
        self,
        store: RobotSessionStore,
        emit_log,
        is_polyworks_connected,
        run_circle,
        run_line,
        run_intersection3p,
        convert_points_between_frames,
        convert_task_result_between_frames,
    ) -> None:
        self.logger = get_logger("controller.robot.protocol")
        self.store = store
        self._emit_log = emit_log
        self._is_polyworks_connected = is_polyworks_connected
        self._run_circle = run_circle
        self._run_line = run_line
        self._run_intersection3p = run_intersection3p
        self._convert_points_between_frames = convert_points_between_frames
        self._convert_task_result_between_frames = convert_task_result_between_frames

    def handle_robot_message(self, raw_message: str, client_label: str) -> list[str]:
        """处理一条正式协议消息。"""
        self.store.record_protocol("IN", raw_message)
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
            self._emit_log(f"协议错误: {exc}", logging.WARNING)
        except SessionStateError as exc:
            responses = [
                build_error(current_req, "SESSION_ERROR", "invalid_session_state", DETAIL=str(exc))
            ]
            self._emit_log(f"会话状态错误: {exc}", logging.WARNING)

        return self.store.record_outbound_protocol(responses)

    def on_client_connected(self, client_label: str) -> None:
        """记录客户端连接事件。"""
        with self.store.lock:
            self.store.active_client_label = client_label
        self._emit_log(f"机器人客户端已连接: {client_label}")

    def on_client_disconnected(self, client_label: str) -> None:
        """记录客户端断开事件。"""
        with self.store.lock:
            if self.store.active_client_label == client_label:
                self.store.active_client_label = None
        self._emit_log(f"机器人客户端已断开: {client_label}")

    def build_ui_snapshot(self, server_running: bool, active_transform_snapshot: dict | None) -> dict:
        """构造机器人联动页面状态快照。"""
        return self.store.build_ui_snapshot(server_running, active_transform_snapshot)

    def _handle_hello(self, message) -> list[str]:
        """处理 HELLO 消息。"""
        client_id = message.get_required("CLIENT")
        with self.store.lock:
            self.store.active_client_id = client_id
        self._emit_log(f"收到客户端握手: {client_id}")
        return [build_ack(message.req, "HELLO", CLIENT=client_id)]

    def _handle_start(self, message, client_label: str) -> list[str]:
        """处理 START 消息。"""
        task_config = parse_start_config(message)

        with self.store.lock:
            if self.store.session is not None and self.store.session.state in {
                "RECEIVING",
                "READY",
                "COMPUTING",
            }:
                raise SessionStateError("上一轮任务尚未结束")

            self.store.session = RobotSession(
                req=message.req,
                task=task_config.task,
                expected_groups=task_config.expected_groups,
                group_order=task_config.group_order,
                frame=task_config.frame,
                client_label=client_label,
                client_id=self.store.active_client_id,
            )
            progress_text = self.store.session.describe_progress()

        self._emit_log(
            f"机器人任务开始: REQ={message.req}, TASK={task_config.task}, "
            f"GROUPS={progress_text}, FRAME={task_config.frame}"
        )
        return [build_ack(message.req, "RECEIVING", **task_config.build_ack_fields())]

    def _handle_point(self, message) -> list[str]:
        """处理 POINT 消息。"""
        with self.store.lock:
            session = self.store.require_active_session(message.req)
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

        self._emit_log(
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
        with self.store.lock:
            session = self.store.require_active_session(message.req)
            progress_text = session.describe_progress()
            if not session.is_full:
                session.mark_error("NOT_ENOUGH_POINTS", "need_more_points")
                self.store.task_history.append(self._build_task_history_entry(session))
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

            if not self._is_polyworks_connected():
                session.mark_error("POLYWORKS_NOT_CONNECTED", "connect_polyworks_first")
                self.store.task_history.append(self._build_task_history_entry(session))
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
            measurement_points = self._convert_session_points_to_measurement_frame(session)
            raw_result = execute_task_from_grouped_points(
                session.task,
                measurement_points,
                self._run_circle,
                self._run_line,
                self._run_intersection3p,
            )
            output_result = self._convert_task_result_between_frames(
                session.task,
                raw_result,
                source_frame=PW_FRAME_NAME,
                target_frame=session.frame,
            )
            result_snapshot = build_result_snapshot(session.task, output_result)
            result_message = build_result_message(message.req, session, output_result)
        except TransformError as exc:
            self.logger.exception("机器人任务坐标变换失败")
            self._emit_log(f"机器人任务坐标变换失败: {exc}", logging.ERROR)
            with self.store.lock:
                current_session = self.store.session
                if current_session is not None and current_session.req == message.req:
                    current_session.mark_error("TRANSFORM_ERROR", str(exc))
                    self.store.task_history.append(self._build_task_history_entry(current_session))
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
            self._emit_log(f"机器人任务执行失败: {exc}", logging.ERROR)
            with self.store.lock:
                current_session = self.store.session
                if current_session is not None and current_session.req == message.req:
                    current_session.mark_error("TASK_FAILED", str(exc))
                    self.store.task_history.append(self._build_task_history_entry(current_session))
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

        with self.store.lock:
            current_session = self.store.session
            if current_session is not None and current_session.req == message.req:
                current_session.mark_done(result_snapshot)
                self.store.task_history.append(self._build_task_history_entry(current_session))

        responses.append(result_message)
        return responses

    def _convert_session_points_to_measurement_frame(
        self,
        session: RobotSession,
        measurement_frame: str = PW_FRAME_NAME,
    ) -> dict[str, list[Point3D]]:
        """把机器人会话中的点转换到实际测量坐标系。"""
        grouped_points = session.get_grouped_point_tuples()
        normalized_measurement_frame = measurement_frame.upper()
        if session.frame.upper() == normalized_measurement_frame:
            return grouped_points

        converted_groups = {
            group: self._convert_points_between_frames(
                points,
                source_frame=session.frame,
                target_frame=normalized_measurement_frame,
            )
            for group, points in grouped_points.items()
        }
        self._emit_log(
            f"机器人输入点坐标转换完成: REQ={session.req}, "
            f"{session.frame.upper()}->{normalized_measurement_frame}"
        )
        return converted_groups

    @staticmethod
    def _build_task_history_entry(session: RobotSession) -> str:
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
