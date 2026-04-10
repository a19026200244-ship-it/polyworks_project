"""机器人联动协调器。"""

from __future__ import annotations

from polyworks_robot_arm.calibration.transform_models import RigidTransform
from polyworks_robot_arm.controllers.robot_protocol_handler import RobotProtocolHandler
from polyworks_robot_arm.controllers.robot_server_manager import RobotServerManager
from polyworks_robot_arm.controllers.robot_session_store import RobotSessionStore


class RobotLinkCoordinator:
    """负责组合机器人联动相关子模块。"""

    def __init__(
        self,
        emit_log,
        is_polyworks_connected,
        get_active_transform,
        run_circle,
        run_line,
        run_intersection3p,
        convert_points_between_frames,
        convert_task_result_between_frames,
    ) -> None:
        self._get_active_transform = get_active_transform
        self.store = RobotSessionStore()
        self.server_manager = RobotServerManager(emit_log=emit_log)
        self.protocol_handler = RobotProtocolHandler(
            store=self.store,
            emit_log=emit_log,
            is_polyworks_connected=is_polyworks_connected,
            run_circle=run_circle,
            run_line=run_line,
            run_intersection3p=run_intersection3p,
            convert_points_between_frames=convert_points_between_frames,
            convert_task_result_between_frames=convert_task_result_between_frames,
        )

    @property
    def logger(self):
        """暴露协议处理器 logger，兼容旧调用。"""
        return self.protocol_handler.logger

    def start_robot_server(self, controller_proxy) -> bool:
        """启动正式机器人服务端。"""
        return self.server_manager.start(controller_proxy)

    def stop_robot_server(self) -> None:
        """停止机器人服务端。"""
        self.server_manager.stop()
        self.store.clear_active_client()

    def on_robot_client_connected(self, client_label: str) -> None:
        """记录客户端连接事件。"""
        self.protocol_handler.on_client_connected(client_label)

    def on_robot_client_disconnected(self, client_label: str) -> None:
        """记录客户端断开事件。"""
        self.protocol_handler.on_client_disconnected(client_label)

    def handle_robot_message(self, raw_message: str, client_label: str) -> list[str]:
        """处理一条正式协议消息。"""
        return self.protocol_handler.handle_robot_message(raw_message, client_label)

    def get_robot_ui_snapshot(self) -> dict:
        """获取机器人联动页面需要的状态快照。"""
        active_transform: RigidTransform | None = self._get_active_transform()
        active_transform_snapshot = None if active_transform is None else active_transform.to_snapshot()
        return self.protocol_handler.build_ui_snapshot(
            server_running=self.server_manager.is_running,
            active_transform_snapshot=active_transform_snapshot,
        )

    @staticmethod
    def format_client_address(address: tuple[str, int]) -> str:
        """格式化客户端地址。"""
        return RobotServerManager.format_client_address(address)
