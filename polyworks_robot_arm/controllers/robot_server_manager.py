"""机器人 Socket 服务端生命周期管理。"""

from __future__ import annotations

import logging
import threading

from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.config import ROBOT_SERVER_ENABLED, ROBOT_SERVER_HOST, ROBOT_SERVER_PORT
from polyworks_robot_arm.common.exceptions import RobotServerError
from polyworks_robot_arm.robot.robot_server import RobotRequestHandler, RobotTCPServer


class RobotServerManager:
    """负责启动、停止和跟踪机器人服务端。"""

    def __init__(self, emit_log) -> None:
        self.logger = get_logger("controller.robot.server")
        self._emit_log = emit_log
        self._server_lock = threading.Lock()
        self._server: RobotTCPServer | None = None
        self._server_thread: threading.Thread | None = None

    @property
    def is_running(self) -> bool:
        """当前服务端是否运行中。"""
        with self._server_lock:
            return self._server is not None

    def start(self, controller_proxy) -> bool:
        """启动正式机器人服务端。"""
        if not ROBOT_SERVER_ENABLED:
            self._emit_log("机器人服务端已在配置中关闭", logging.INFO)
            return False

        with self._server_lock:
            if self._server is not None:
                return True

            try:
                self._server = RobotTCPServer(
                    (ROBOT_SERVER_HOST, ROBOT_SERVER_PORT),
                    RobotRequestHandler,
                    controller_proxy,
                )
            except OSError as exc:
                self.logger.exception("启动机器人服务端失败")
                self._emit_log(f"启动机器人服务端失败: {exc}", logging.ERROR)
                self._server = None
                raise RobotServerError(f"启动机器人服务端失败: {exc}") from exc

            self._server_thread = threading.Thread(
                target=self._server.serve_forever,
                name="robot-server",
                daemon=True,
            )
            self._server_thread.start()

        self._emit_log(f"机器人服务端已启动: {ROBOT_SERVER_HOST}:{ROBOT_SERVER_PORT}")
        return True

    def stop(self) -> None:
        """停止机器人服务端。"""
        with self._server_lock:
            server = self._server
            thread = self._server_thread
            self._server = None
            self._server_thread = None

        if server is None:
            return

        server.shutdown()
        server.server_close()
        if thread is not None:
            thread.join(timeout=1.0)

        self._emit_log("机器人服务端已停止")

    @staticmethod
    def format_client_address(address: tuple[str, int]) -> str:
        """格式化客户端地址。"""
        return f"{address[0]}:{address[1]}"
