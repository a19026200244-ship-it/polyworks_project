"""机器人联动协议模拟器。

阶段 3 说明：
- 协议里的 FRAME 表示客户端发送点和接收结果所使用的坐标系
- 示例文件默认仍按 PolyWorks 坐标保存
- 当 FRAME 和数据文件坐标系不一致时，模拟器会使用当前激活变换自动换算
"""

from __future__ import annotations

import argparse
import select
import socket
import time

from polyworks_robot_arm.calibration.transform_repository import TransformRepository
from polyworks_robot_arm.calibration.transform_service import (
    resolve_transform_between_frames,
    transform_points,
)
from polyworks_robot_arm.common.app_logger import get_logger
from polyworks_robot_arm.common.config import (
    EXPECTED_POINT_COUNT,
    PW_FRAME_NAME,
    ROBOT_DEFAULT_FRAME,
    ROBOT_DEFAULT_TASK,
    ROBOT_SERVER_ENCODING,
    ROBOT_SERVER_HOST,
    ROBOT_SERVER_PORT,
    ROBOT_SERVER_REPLY_WAIT,
    ROBOT_SERVER_SOCKET_TIMEOUT,
    SAMPLE_DATA_DIR,
)
from polyworks_robot_arm.common.data_parser import parse_points_file
from polyworks_robot_arm.common.exceptions import RobotSimulatorError, TransformError

Point3D = tuple[float, float, float]
logger = get_logger("robot_simulator")


def build_protocol_messages(
    task: str,
    req: str,
    client_id: str,
    frame: str,
    version: str,
    data_frame: str = PW_FRAME_NAME,
    points_file: str | None = None,
    p1_file: str | None = None,
    p2_file: str | None = None,
    p3_file: str | None = None,
) -> list[str]:
    """按任务类型构造一组协议消息。"""
    normalized_task = task.upper()
    client_frame = frame.upper()
    source_data_frame = data_frame.upper()
    messages = [f"HELLO;REQ=1;CLIENT={client_id};VER={version}"]

    if normalized_task == "CIRCLE":
        circle_file = points_file or str(SAMPLE_DATA_DIR / "拟合圆.txt")
        grouped_points = {"POINTS": parse_points_file(circle_file)}
        grouped_points = _convert_grouped_points_for_client_frame(
            grouped_points,
            source_frame=source_data_frame,
            target_frame=client_frame,
        )
        messages.append(
            f"START;REQ={req};TASK=CIRCLE;POINTS={len(grouped_points['POINTS'])};FRAME={client_frame}"
        )
        messages.extend(_build_point_messages(req, "CIRCLE", grouped_points))
        messages.append(f"END;REQ={req}")
        return messages

    if normalized_task == "LINE":
        plane1_file = p1_file or str(SAMPLE_DATA_DIR / "平面1.txt")
        plane2_file = p2_file or str(SAMPLE_DATA_DIR / "平面2.txt")
        grouped_points = {
            "P1": parse_points_file(plane1_file),
            "P2": parse_points_file(plane2_file),
        }
        grouped_points = _convert_grouped_points_for_client_frame(
            grouped_points,
            source_frame=source_data_frame,
            target_frame=client_frame,
        )
        messages.append(
            f"START;REQ={req};TASK=LINE;"
            f"P1={len(grouped_points['P1'])};P2={len(grouped_points['P2'])};FRAME={client_frame}"
        )
        messages.extend(_build_point_messages(req, "LINE", grouped_points))
        messages.append(f"END;REQ={req}")
        return messages

    if normalized_task == "INTERSECTION3P":
        grouped_points = _load_intersection3p_points(points_file, p1_file, p2_file, p3_file)
        grouped_points = _convert_grouped_points_for_client_frame(
            grouped_points,
            source_frame=source_data_frame,
            target_frame=client_frame,
        )
        messages.append(
            "START;"
            f"REQ={req};TASK=INTERSECTION3P;"
            f"P1={len(grouped_points['P1'])};P2={len(grouped_points['P2'])};P3={len(grouped_points['P3'])};"
            f"FRAME={client_frame}"
        )
        messages.extend(_build_point_messages(req, "INTERSECTION3P", grouped_points))
        messages.append(f"END;REQ={req}")
        return messages

    raise RobotSimulatorError(f"当前模拟器不支持任务类型: {normalized_task}")


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
    parser = argparse.ArgumentParser(description="按 Stage 3 协议向服务端发送机器人测量任务")
    parser.add_argument("--host", default=ROBOT_SERVER_HOST, help="服务端主机地址")
    parser.add_argument("--port", type=int, default=ROBOT_SERVER_PORT, help="服务端端口")
    parser.add_argument(
        "--task",
        default=ROBOT_DEFAULT_TASK,
        choices=["CIRCLE", "LINE", "INTERSECTION3P"],
        help="要回放的任务类型",
    )
    parser.add_argument(
        "--file",
        dest="points_file",
        default=None,
        help="CIRCLE 或 INTERSECTION3P 的主文件；INTERSECTION3P 可传 9 点合并文件",
    )
    parser.add_argument("--p1-file", dest="p1_file", default=None, help="分组 P1 的点文件")
    parser.add_argument("--p2-file", dest="p2_file", default=None, help="分组 P2 的点文件")
    parser.add_argument("--p3-file", dest="p3_file", default=None, help="分组 P3 的点文件")
    parser.add_argument("--req", default="1001", help="任务请求号")
    parser.add_argument("--client", dest="client_id", default="ROBOT_1", help="客户端标识")
    parser.add_argument("--frame", default=ROBOT_DEFAULT_FRAME, help="客户端发送/接收使用的坐标系标识")
    parser.add_argument("--data-frame", default=PW_FRAME_NAME, help="点文件本身使用的坐标系")
    parser.add_argument("--ver", default="1.0", help="协议版本号")
    parser.add_argument("--interval", type=float, default=0.15, help="两条消息之间的发送间隔")
    return parser.parse_args()


def main() -> None:
    """程序入口。"""
    args = parse_args()

    try:
        messages = build_protocol_messages(
            task=args.task,
            req=args.req,
            client_id=args.client_id,
            frame=args.frame,
            version=args.ver,
            data_frame=args.data_frame,
            points_file=args.points_file,
            p1_file=args.p1_file,
            p2_file=args.p2_file,
            p3_file=args.p3_file,
        )
        send_messages(args.host, args.port, messages, args.interval)
    except Exception as exc:
        logger.exception("机器人模拟器执行失败")
        raise SystemExit(str(exc)) from exc


def _build_point_messages(
    req: str,
    task: str,
    grouped_points: dict[str, list[Point3D]],
) -> list[str]:
    """按任务类型生成 POINT 消息。"""
    messages: list[str] = []
    if task == "CIRCLE":
        for index, (x, y, z) in enumerate(grouped_points["POINTS"], start=1):
            messages.append(
                f"POINT;REQ={req};IDX={index};X={x:.6f};Y={y:.6f};Z={z:.6f}"
            )
        return messages

    for group, points in grouped_points.items():
        for index, (x, y, z) in enumerate(points, start=1):
            messages.append(
                f"POINT;REQ={req};GROUP={group};IDX={index};X={x:.6f};Y={y:.6f};Z={z:.6f}"
            )
    return messages


def _load_intersection3p_points(
    points_file: str | None,
    p1_file: str | None,
    p2_file: str | None,
    p3_file: str | None,
) -> dict[str, list[Point3D]]:
    """加载 INTERSECTION3P 任务的分组点。"""
    if p1_file or p2_file or p3_file:
        if not (p1_file and p2_file and p3_file):
            raise RobotSimulatorError("INTERSECTION3P 若使用分组文件，必须同时提供 --p1-file --p2-file --p3-file")
        return {
            "P1": parse_points_file(p1_file),
            "P2": parse_points_file(p2_file),
            "P3": parse_points_file(p3_file),
        }

    merged_file = points_file or str(SAMPLE_DATA_DIR / "平面交点.txt")
    points = parse_points_file(merged_file)
    if len(points) != EXPECTED_POINT_COUNT:
        raise RobotSimulatorError(
            f"INTERSECTION3P 的合并文件必须包含 {EXPECTED_POINT_COUNT} 个点，实际为 {len(points)}"
        )

    return {
        "P1": points[0:3],
        "P2": points[3:6],
        "P3": points[6:9],
    }


def _convert_grouped_points_for_client_frame(
    grouped_points: dict[str, list[Point3D]],
    source_frame: str,
    target_frame: str,
) -> dict[str, list[Point3D]]:
    """把示例文件点转换成客户端准备发送的坐标系。"""
    normalized_source = source_frame.upper()
    normalized_target = target_frame.upper()
    if normalized_source == normalized_target:
        return grouped_points

    transform = _load_transform_for_simulator(normalized_source, normalized_target)
    return {
        group: transform_points(points, transform)
        for group, points in grouped_points.items()
    }


def _load_transform_for_simulator(source_frame: str, target_frame: str):
    """为模拟器加载一条可用的坐标变换。"""
    repository = TransformRepository()
    active_transform = repository.get_active_transform()
    if active_transform is None:
        raise RobotSimulatorError(
            f"当前没有激活的标定变换，模拟器无法把点从 {source_frame} 转换到 {target_frame}"
        )

    try:
        transform = resolve_transform_between_frames(source_frame, target_frame, active_transform)
    except TransformError as exc:
        raise RobotSimulatorError(f"模拟器坐标转换失败: {exc}") from exc

    if transform is None:
        raise RobotSimulatorError("模拟器未能解析出有效的坐标变换")
    return transform


if __name__ == "__main__":
    main()
