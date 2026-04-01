"""机器人联动协议模拟器。

阶段 1 里它会按正式协议发送：
HELLO -> START -> POINT... -> END
"""

from __future__ import annotations

import argparse
import select
import socket
import time

from app_logger import get_logger
from config import (
    ROBOT_DEFAULT_FRAME,
    ROBOT_DEFAULT_TASK,
    ROBOT_SERVER_ENCODING,
    ROBOT_SERVER_HOST,
    ROBOT_SERVER_PORT,
    ROBOT_SERVER_REPLY_WAIT,
    ROBOT_SERVER_SOCKET_TIMEOUT,
)
from data_parser import parse_points_file
from exceptions import RobotSimulatorError

logger = get_logger("robot_simulator")


def build_protocol_messages(
    points_file: str,
    req: str,
    client_id: str,
    frame: str,
    version: str,
) -> list[str]:
    """构造一组正式协议消息。"""
    points = parse_points_file(points_file)

    messages = [
        f"HELLO;REQ=1;CLIENT={client_id};VER={version}",
        f"START;REQ={req};TASK={ROBOT_DEFAULT_TASK};POINTS={len(points)};FRAME={frame}",
    ]

    for index, (x, y, z) in enumerate(points, start=1):
        messages.append(
            f"POINT;REQ={req};IDX={index};X={x:.6f};Y={y:.6f};Z={z:.6f}"
        )

    messages.append(f"END;REQ={req}")
    return messages


def send_messages(host: str, port: int, messages: list[str], interval: float) -> None:
    """按顺序发送协议消息，并打印服务端回复。"""
    try:
        with socket.create_connection((host, port), timeout=ROBOT_SERVER_SOCKET_TIMEOUT) as client:
            client.settimeout(ROBOT_SERVER_SOCKET_TIMEOUT)
            receive_buffer = b""

            for message in messages:
                logger.info("发送消息: %s", message)
                client.sendall(f"{message}\n".encode(ROBOT_SERVER_ENCODING))
                print(f"SEND: {message}")

                replies, receive_buffer = _read_available_lines(client, receive_buffer)
                for reply in replies:
                    print(f"RECV: {reply}")

                time.sleep(interval)
    except OSError as exc:
        raise RobotSimulatorError(
            "无法连接到机器人服务端，请先启动主程序并在“机器人联动”页点击“启动 Socket 服务”"
        ) from exc


def _read_available_lines(client: socket.socket, receive_buffer: bytes) -> tuple[list[str], bytes]:
    """读取服务端当前已经返回的所有整行消息。"""
    replies: list[str] = []
    deadline = time.time() + ROBOT_SERVER_REPLY_WAIT

    while time.time() < deadline:
        readable, _, _ = select.select([client], [], [], 0.05)
        if not readable:
            continue

        chunk = client.recv(4096)
        if not chunk:
            break

        receive_buffer += chunk
        while b"\n" in receive_buffer:
            raw_line, receive_buffer = receive_buffer.split(b"\n", 1)
            cleaned = raw_line.decode(ROBOT_SERVER_ENCODING).strip()
            if cleaned:
                replies.append(cleaned)

    return replies, receive_buffer


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="按阶段 1 正式协议向服务端发送拟合圆任务")
    parser.add_argument("--host", default=ROBOT_SERVER_HOST, help="服务端主机地址")
    parser.add_argument("--port", type=int, default=ROBOT_SERVER_PORT, help="服务端端口")
    parser.add_argument(
        "--file",
        dest="points_file",
        default="数据文件/拟合圆.txt",
        help="用于拟合圆的点文件",
    )
    parser.add_argument("--req", default="1001", help="任务请求号")
    parser.add_argument("--client", dest="client_id", default="ROBOT_1", help="客户端标识")
    parser.add_argument("--frame", default=ROBOT_DEFAULT_FRAME, help="坐标系标识")
    parser.add_argument("--ver", default="1.0", help="协议版本号")
    parser.add_argument("--interval", type=float, default=0.15, help="两条消息之间的发送间隔")
    return parser.parse_args()


def main() -> None:
    """程序入口。"""
    args = parse_args()

    try:
        messages = build_protocol_messages(
            points_file=args.points_file,
            req=args.req,
            client_id=args.client_id,
            frame=args.frame,
            version=args.ver,
        )
        send_messages(args.host, args.port, messages, args.interval)
    except Exception as exc:
        logger.exception("机器人模拟器执行失败")
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
