"""机器人 TCP 服务端。"""

from __future__ import annotations

import socketserver
import threading

from polyworks_robot_arm.common.config import ROBOT_SERVER_ENCODING
from polyworks_robot_arm.robot.robot_protocol import build_error


class RobotTCPServer(socketserver.ThreadingTCPServer):
    """阶段 1 使用的正式机器人服务端。"""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, handler_class, controller) -> None:
        self.controller = controller
        self._client_lock = threading.Lock()
        self._active_client: str | None = None
        super().__init__(server_address, handler_class)

    def try_register_client(self, client_label: str) -> bool:
        """尝试登记一个活动客户端。"""
        with self._client_lock:
            if self._active_client is not None and self._active_client != client_label:
                return False
            self._active_client = client_label
            return True

    def unregister_client(self, client_label: str) -> None:
        """注销活动客户端。"""
        with self._client_lock:
            if self._active_client == client_label:
                self._active_client = None

    @property
    def active_client(self) -> str | None:
        """当前活动客户端标签。"""
        with self._client_lock:
            return self._active_client


class RobotRequestHandler(socketserver.StreamRequestHandler):
    """处理单个机器人客户端连接。"""

    def handle(self) -> None:
        controller = self.server.controller
        client_label = controller.format_client_address(self.client_address)

        if not self.server.try_register_client(client_label):
            self._write_lines(
                [build_error("0", "SERVER_BUSY", "single_client_only")]
            )
            return

        controller.on_robot_client_connected(client_label)

        try:
            while True:
                raw_line = self.rfile.readline()
                if not raw_line:
                    break

                try:
                    message = raw_line.decode(ROBOT_SERVER_ENCODING).strip()
                    responses = controller.handle_robot_message(message, client_label)
                except Exception as exc:  # pragma: no cover - 防御性兜底
                    controller.logger.exception("处理机器人消息时出现未预期异常")
                    responses = [build_error("0", "INTERNAL_ERROR", str(exc))]

                self._write_lines(responses)
        finally:
            controller.on_robot_client_disconnected(client_label)
            self.server.unregister_client(client_label)

    def _write_lines(self, lines: list[str]) -> None:
        """按行回写响应。"""
        for line in lines:
            self.wfile.write(f"{line}\n".encode(ROBOT_SERVER_ENCODING))

