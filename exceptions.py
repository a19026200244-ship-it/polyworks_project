"""项目统一异常定义。"""

from __future__ import annotations


class AppError(Exception):
    """项目基础异常类型。"""


class ConfigurationError(AppError):
    """配置错误。"""


class PolyWorksConnectionError(AppError):
    """PolyWorks 连接错误。"""


class MeasurementError(AppError):
    """测量流程错误。"""


class ProtocolError(AppError):
    """协议或消息格式错误。"""


class RobotSimulatorError(AppError):
    """机器人模拟器相关错误。"""


class RobotServerError(AppError):
    """机器人 TCP 服务端相关错误。"""


class SessionStateError(AppError):
    """机器人会话状态错误。"""
