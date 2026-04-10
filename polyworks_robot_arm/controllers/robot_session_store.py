"""机器人会话与运行期状态存储。"""

from __future__ import annotations

from collections import deque
import threading

from polyworks_robot_arm.common.config import (
    ROBOT_DEFAULT_FRAME,
    ROBOT_PROTOCOL_HISTORY_LIMIT,
    ROBOT_SERVER_HOST,
    ROBOT_SERVER_PORT,
    ROBOT_TASK_HISTORY_LIMIT,
)
from polyworks_robot_arm.common.exceptions import SessionStateError
from polyworks_robot_arm.robot.robot_session import RobotSession


class RobotSessionStore:
    """集中维护机器人联动运行期状态。"""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.active_client_label: str | None = None
        self.active_client_id: str = ""
        self.session: RobotSession | None = None
        self.protocol_history: deque[str] = deque(maxlen=ROBOT_PROTOCOL_HISTORY_LIMIT)
        self.task_history: deque[str] = deque(maxlen=ROBOT_TASK_HISTORY_LIMIT)

    def require_active_session(self, req: str) -> RobotSession:
        """校验并返回当前活动会话。"""
        if self.session is None:
            raise SessionStateError("当前没有活动会话")
        if self.session.req != req:
            raise SessionStateError(
                f"REQ 不匹配，当前会话 REQ={self.session.req}，收到 REQ={req}"
            )
        return self.session

    def record_protocol(self, direction: str, message: str) -> None:
        """记录一条协议消息。"""
        with self.lock:
            self.protocol_history.append(f"{direction}: {message}")

    def record_outbound_protocol(self, responses: list[str]) -> list[str]:
        """记录全部发出的协议响应。"""
        for response in responses:
            self.record_protocol("OUT", response)
        return responses

    def append_task_history(self, entry: str) -> None:
        """追加一条任务历史。"""
        with self.lock:
            self.task_history.append(entry)

    def clear_active_client(self) -> None:
        """清空当前活动客户端。"""
        with self.lock:
            self.active_client_label = None

    def build_ui_snapshot(self, server_running: bool, active_transform_snapshot: dict | None) -> dict:
        """构造机器人联动页面需要的状态快照。"""
        with self.lock:
            session_snapshot = None if self.session is None else self.session.to_snapshot()
            active_client = self.active_client_label
            protocol_history = list(self.protocol_history)
            task_history = list(self.task_history)

        return {
            "server_running": server_running,
            "host": ROBOT_SERVER_HOST,
            "port": ROBOT_SERVER_PORT,
            "active_client": active_client,
            "default_frame": ROBOT_DEFAULT_FRAME,
            "active_transform": active_transform_snapshot,
            "session": session_snapshot,
            "protocol_history": protocol_history,
            "task_history": task_history,
        }
