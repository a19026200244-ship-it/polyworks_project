"""应用日志工具。

这个模块负责两件事：
1. 配置统一的文件日志
2. 安装全局异常钩子，尽量把未捕获异常也写进日志
"""

from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler

from config import LOG_BACKUP_COUNT, LOG_DIR, LOG_FILE, LOG_MAX_BYTES

APP_LOGGER_NAME = "polyworks_robot_arm"


def _configure_base_logger() -> logging.Logger:
    """配置项目的根日志器。"""
    logger = logging.getLogger(APP_LOGGER_NAME)
    if logger.handlers:
        return logger

    LOG_DIR.mkdir(parents=True, exist_ok=True)

    logger.setLevel(logging.INFO)
    logger.propagate = False

    formatter = logging.Formatter(
        fmt="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = RotatingFileHandler(
        LOG_FILE,
        maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    return logger


def get_logger(name: str | None = None) -> logging.Logger:
    """获取项目日志器。

    `name` 不传时返回根日志器；
    传入时返回该模块对应的子日志器。
    """
    base_logger = _configure_base_logger()
    if not name:
        return base_logger
    return logging.getLogger(f"{base_logger.name}.{name}")


def install_global_excepthook(logger: logging.Logger | None = None) -> None:
    """安装全局异常钩子，把未捕获异常写入日志。"""
    active_logger = logger or get_logger("global")

    def _handle_exception(exc_type, exc_value, exc_traceback) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        active_logger.exception(
            "捕获到未处理异常",
            exc_info=(exc_type, exc_value, exc_traceback),
        )

    sys.excepthook = _handle_exception
