"""测量业务流程。

这个模块把“很多底层 PolyWorks 命令”组织成几套可直接调用的功能：
1. 三平面交点
2. 多点拟合圆
3. 两面交线

和 `polyworks_com.py` 的分工不同：
- `polyworks_com.py` 负责“怎么发命令”
- `services.py` 负责“按什么步骤完成一个业务功能”
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from polyworks_robot_arm.common.config import (
    EXPECTED_POINT_COUNT,
    INTERSECTION_POINT_NAME,
    MIN_POINTS_FOR_CIRCLE,
    MIN_POINTS_FOR_PLANE,
    PLANE_NAMES,
    RESULT_FILE,
    TEMP_MACRO_FILE,
)
from polyworks_robot_arm.common.result_types import CircleFitResult, LineResult
from polyworks_robot_arm.integrations.polyworks_com import PolyWorksConnector

# 项目里统一把一个三维点表示为 `(x, y, z)`。
Point3D = tuple[float, float, float]


def validate_points(points: list[Point3D]) -> None:
    """校验“三平面交点”功能是否给了 9 个点。"""
    if len(points) != EXPECTED_POINT_COUNT:
        raise ValueError(f"需要 {EXPECTED_POINT_COUNT} 个点，实际为 {len(points)} 个")


def _validate_min_points(points: list[Point3D], minimum: int, label: str) -> None:
    """校验一个点集是否达到最小点数要求。"""
    if len(points) < minimum:
        raise ValueError(f"{label}至少需要 {minimum} 个点，实际只有 {len(points)} 个")


def _build_plane_commands(plane_names: list[str], point_names: list[str]) -> list[str]:
    """根据点名和目标平面名生成三平面交点的平面创建命令。"""
    commands: list[str] = []
    for plane_index, plane_name in enumerate(plane_names):
        start_index = plane_index * 3
        commands.append('TREEVIEW OBJECT SELECT NONE')
        for point_name in point_names[start_index:start_index + 3]:
            commands.append(f'TREEVIEW OBJECT SELECT ( "{point_name}", "On" )')
        commands.append(f'FEATURE PRIMITIVE PLANE FROM_POINTS ( "{plane_name}", )')
    return commands


def _build_intersection_commands(plane_names: list[str], intersection_name: str) -> list[str]:
    """根据平面名生成三平面交点命令。"""
    commands = ['TREEVIEW OBJECT SELECT NONE']
    for plane_name in plane_names:
        commands.append(f'TREEVIEW OBJECT SELECT ( "{plane_name}", "On" )')
    commands.append(
        f'FEATURE PRIMITIVE POINT FROM_INTERSECTION_PLANES ( "{intersection_name}", )'
    )
    return commands


def _normalize(vector: Point3D) -> Point3D:
    """把一个三维向量归一化。"""
    x, y, z = vector
    length = (x * x + y * y + z * z) ** 0.5
    if length == 0:
        raise ValueError("方向向量长度为 0")
    return x / length, y / length, z / length


def _build_axes_from_normal(normal: Point3D) -> tuple[Point3D, Point3D]:
    """根据法向量生成圆所在平面内的两个正交方向。"""
    nx, ny, nz = _normalize(normal)
    reference = (1.0, 0.0, 0.0) if abs(nx) < 0.9 else (0.0, 1.0, 0.0)

    ux = ny * reference[2] - nz * reference[1]
    uy = nz * reference[0] - nx * reference[2]
    uz = nx * reference[1] - ny * reference[0]
    axis_u = _normalize((ux, uy, uz))

    vx = ny * axis_u[2] - nz * axis_u[1]
    vy = nz * axis_u[0] - nx * axis_u[2]
    vz = nx * axis_u[1] - ny * axis_u[0]
    axis_v = _normalize((vx, vy, vz))

    return axis_u, axis_v


def _read_numbers_from_file(result_file: Path, expected_count: int) -> list[float]:
    """读取 PolyWorks 宏导出的逗号分隔结果。"""
    if not result_file.exists():
        raise RuntimeError(f"未找到结果文件: {result_file}")

    content = result_file.read_text(encoding='utf-8').strip()
    parts = [float(value.strip()) for value in content.split(',') if value.strip()]
    if len(parts) < expected_count:
        raise RuntimeError(f"结果文件格式不正确: {content}")

    return parts[:expected_count]


class MeasurementService:
    """组织机械臂测量流程。"""

    def __init__(
        self,
        connector: PolyWorksConnector,
        result_file: Path = RESULT_FILE,
        temp_macro_file: Path = TEMP_MACRO_FILE,
    ) -> None:
        self.connector = connector
        self.result_file = Path(result_file)
        self.temp_macro_file = Path(temp_macro_file)
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.temp_macro_file.parent.mkdir(parents=True, exist_ok=True)

    def _require_connected(self) -> None:
        """要求当前必须已经连接到 PolyWorks。"""
        if not self.connector.connected:
            raise RuntimeError("请先连接 PolyWorks，再执行该功能")

    def _make_unique_token(self) -> str:
        """生成唯一后缀，避免本次创建的对象和旧对象重名。"""
        return datetime.now().strftime('%Y%m%d_%H%M%S_%f')

    def _make_temp_paths(self, tag: str) -> tuple[Path, Path]:
        """为一次宏执行生成独立的中间文件路径。"""
        token = self._make_unique_token()
        result_file = self.result_file.with_name(
            f'{self.result_file.stem}_{tag}_{token}{self.result_file.suffix}'
        )
        macro_file = self.temp_macro_file.with_name(
            f'{self.temp_macro_file.stem}_{tag}_{token}{self.temp_macro_file.suffix}'
        )
        result_file.parent.mkdir(parents=True, exist_ok=True)
        macro_file.parent.mkdir(parents=True, exist_ok=True)
        return result_file, macro_file

    def _run_macro_and_read_numbers(
        self,
        tag: str,
        body_lines: list[str],
        expected_count: int,
        value_expression: str,
    ) -> list[float]:
        """执行一个临时宏，并把数值结果读回来。"""
        result_file, macro_file = self._make_temp_paths(tag)

        macro_lines = ['VERSION "6.0"', *body_lines]
        macro_lines.append(f'DATA_FILE CREATE ( "{result_file.as_posix()}" )')
        macro_lines.append(f'DATA_FILE APPEND ( "{result_file.as_posix()}", "{value_expression}" )')
        macro_content = '\n'.join(macro_lines)

        # 统一保留中间文件，方便后续排查“命令发了什么、PolyWorks 导出了什么”。
        macro_file.write_text(macro_content, encoding='utf-16')
        self.connector.execute_macro(macro_file)
        return _read_numbers_from_file(result_file, expected_count)

    def _create_feature_points(self, points: list[Point3D], prefix: str) -> list[str]:
        """把输入坐标创建成 PolyWorks 点特征。"""
        point_names: list[str] = []
        for index, point in enumerate(points, start=1):
            point_name = f'{prefix}_点{index}'
            self.connector.create_point(point, point_name)
            point_names.append(point_name)
        return point_names

    def _select_objects(self, object_names: list[str]) -> None:
        """按名字批量选择对象。"""
        self.connector.select_none()
        for object_name in object_names:
            self.connector.select_object(object_name)

    def _read_point_feature(self, feature_name: str) -> Point3D:
        """读取点特征的坐标。"""
        numbers = self._run_macro_and_read_numbers(
            tag='point',
            body_lines=[
                'DECLARE pt_x',
                'DECLARE pt_y',
                'DECLARE pt_z',
                'TREEVIEW OBJECT SELECT NONE',
                # 这里不能只按对象名读取，否则容易读到错误对象或默认值。
                # 改成和圆/直线一致，先取对应 feature 的 nominal primitive 再读坐标。
                f'TREEVIEW PRIMITIVE SELECT FROM_SELECTED_FEATURES NOMINAL ( "{feature_name}" )',
                'TREEVIEW PRIMITIVE POINT PROPERTIES POINT GET ( pt_x, pt_y, pt_z )',
            ],
            expected_count=3,
            value_expression='$pt_x,$pt_y,$pt_z',
        )
        return numbers[0], numbers[1], numbers[2]

    def _read_circle_feature(self, feature_name: str) -> CircleFitResult:
        """读取拟合圆的圆心、法向量和半径。"""
        numbers = self._run_macro_and_read_numbers(
            tag='circle',
            body_lines=[
                'DECLARE center_x',
                'DECLARE center_y',
                'DECLARE center_z',
                'DECLARE axis_i',
                'DECLARE axis_j',
                'DECLARE axis_k',
                'DECLARE radius',
                'TREEVIEW OBJECT SELECT NONE',
                f'TREEVIEW PRIMITIVE SELECT FROM_SELECTED_FEATURES NOMINAL ( "{feature_name}" )',
                'TREEVIEW PRIMITIVE CIRCLE PROPERTIES CENTER GET ( center_x, center_y, center_z )',
                'TREEVIEW PRIMITIVE CIRCLE PROPERTIES AXIS_ORIENTATION GET ( axis_i, axis_j, axis_k )',
                'TREEVIEW PRIMITIVE CIRCLE PROPERTIES RADIUS GET ( radius )',
            ],
            expected_count=7,
            value_expression='$center_x,$center_y,$center_z,$radius,$axis_i,$axis_j,$axis_k',
        )

        center = (numbers[0], numbers[1], numbers[2])
        radius = numbers[3]
        normal = (numbers[4], numbers[5], numbers[6])
        axis_u, axis_v = _build_axes_from_normal(normal)

        return CircleFitResult(
            center=center,
            diameter=radius * 2.0,
            normal=normal,
            radius=radius,
            axis_u=axis_u,
            axis_v=axis_v,
        )

    def _read_line_feature(self, feature_name: str) -> LineResult:
        """读取交线上的一点和方向向量。"""
        numbers = self._run_macro_and_read_numbers(
            tag='line',
            body_lines=[
                'DECLARE origin_x',
                'DECLARE origin_y',
                'DECLARE origin_z',
                'DECLARE dir_i',
                'DECLARE dir_j',
                'DECLARE dir_k',
                'TREEVIEW OBJECT SELECT NONE',
                f'TREEVIEW PRIMITIVE SELECT FROM_SELECTED_FEATURES NOMINAL ( "{feature_name}" )',
                'TREEVIEW PRIMITIVE LINE PROPERTIES ORIGIN GET ( origin_x, origin_y, origin_z )',
                'TREEVIEW PRIMITIVE LINE PROPERTIES ORIENTATION GET ( dir_i, dir_j, dir_k )',
            ],
            expected_count=6,
            value_expression='$origin_x,$origin_y,$origin_z,$dir_i,$dir_j,$dir_k',
        )

        return LineResult(
            point=(numbers[0], numbers[1], numbers[2]),
            direction=(numbers[3], numbers[4], numbers[5]),
        )

    def run_measurement(self, points: list[Point3D]) -> Point3D:
        """执行“三平面交点”流程。"""
        self._require_connected()
        validate_points(points)

        # 三平面交点之前一直使用固定名称，容易和 PolyWorks 工程里的旧对象重名。
        # 这里统一改成带时间戳的唯一名称，避免选到历史对象而导致结果读错。
        token = self._make_unique_token()
        point_names = self._create_feature_points(points, f'交点输入_{token}')
        plane_names = [f'{plane_name}_{token}' for plane_name in PLANE_NAMES]
        intersection_name = f'{INTERSECTION_POINT_NAME}_{token}'

        self.connector.execute_commands(_build_plane_commands(plane_names, point_names))
        self.connector.execute_commands(_build_intersection_commands(plane_names, intersection_name))
        return self._read_point_feature(intersection_name)

    def cleanup_result_file(self) -> None:
        """保留兼容接口。当前中间结果文件默认保留，不再自动删除。"""
        return

    def fit_circle_from_points(self, points: list[Point3D]) -> CircleFitResult:
        """通过 PolyWorks 拟合圆。"""
        self._require_connected()
        _validate_min_points(points, MIN_POINTS_FOR_CIRCLE, '拟合圆')

        token = self._make_unique_token()
        point_prefix = f'拟合圆输入_{token}'
        circle_name = f'拟合圆_{token}'

        point_names = self._create_feature_points(points, point_prefix)
        self._select_objects(point_names)
        self.connector.execute_command(
            f'FEATURE PRIMITIVE CIRCLE FROM_CENTER_POINTS ( "{circle_name}", )'
        )

        return self._read_circle_feature(circle_name)

    def intersect_plane_groups(
        self,
        plane1_points: list[Point3D],
        plane2_points: list[Point3D],
    ) -> LineResult:
        """通过 PolyWorks 拟合两个平面并计算交线。"""
        self._require_connected()
        _validate_min_points(plane1_points, MIN_POINTS_FOR_PLANE, '平面 1')
        _validate_min_points(plane2_points, MIN_POINTS_FOR_PLANE, '平面 2')

        token = self._make_unique_token()
        plane1_point_names = self._create_feature_points(plane1_points, f'交线平面1输入_{token}')
        plane2_point_names = self._create_feature_points(plane2_points, f'交线平面2输入_{token}')
        plane1_name = f'交线平面1_{token}'
        plane2_name = f'交线平面2_{token}'
        line_name = f'交线_{token}'

        self._select_objects(plane1_point_names)
        self.connector.execute_command(f'FEATURE PRIMITIVE PLANE FROM_POINTS ( "{plane1_name}", )')

        self._select_objects(plane2_point_names)
        self.connector.execute_command(f'FEATURE PRIMITIVE PLANE FROM_POINTS ( "{plane2_name}", )')

        self._select_objects([plane1_name, plane2_name])
        self.connector.execute_command(
            f'FEATURE PRIMITIVE LINE FROM_INTERSECTION_PLANES ( "{line_name}", )'
        )

        return self._read_line_feature(line_name)
