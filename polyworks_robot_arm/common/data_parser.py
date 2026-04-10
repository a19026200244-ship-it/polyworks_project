"""坐标文件解析。

职责尽量保持单一：
1. 负责把 txt 文件里的点坐标读出来
2. 负责把界面输入框里的点坐标文本解析出来

不处理界面，也不处理 PolyWorks 通信。
"""

from pathlib import Path

# 项目里统一把一个三维点写成 `(x, y, z)`。
# 这种写法叫“类型别名”，主要是为了让代码更好读。
Point3D = tuple[float, float, float]


def _parse_line_to_point(line: str, line_number: int) -> Point3D:
    """把一行文本解析成一个三维点。"""
    # 允许用户用多种分隔方式输入，减少手动整理格式的负担。
    # 例如：
    # 1,2,3
    # 1 2 3
    # 1;2;3
    # 最后都会被统一处理成一样的结果。
    normalized = line.replace(';', ',').replace('\t', ',').replace(' ', ',')
    parts = [item.strip() for item in normalized.split(',') if item.strip()]
    if len(parts) != 3:
        raise ValueError(f"第 {line_number} 行格式不正确，应为 x,y,z，实际内容: {line}")

    try:
        # 文本输入最终都要转成浮点数，后续计算才能直接使用。
        return tuple(float(value) for value in parts)
    except ValueError as exc:
        raise ValueError(f"第 {line_number} 行包含无法转换的数字: {line}") from exc


def parse_points_file(filepath: str | Path) -> list[Point3D]:
    """解析坐标文件，返回 [(x, y, z), ...]。

    这个函数专门给“从 txt 文件读取点坐标”的场景使用。
    """
    points: list[Point3D] = []
    # 统一转成 Path 对象，后面读文件会更方便。
    path = Path(filepath)

    with path.open('r', encoding='utf-8') as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            # 支持空行和注释行，方便你在 txt 里写说明。
            if not line or line.startswith('#'):
                continue
            points.append(_parse_line_to_point(line, line_number))

    return points


def parse_points_text(text: str) -> list[Point3D]:
    """解析文本框中的多行点坐标。

    这个函数给界面里的 QTextEdit 输入框使用。
    """
    points: list[Point3D] = []
    # `splitlines()` 会把文本框里的多行内容拆成一行一行，
    # 这样就能复用和读文件几乎一样的解析逻辑。
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        points.append(_parse_line_to_point(line, line_number))
    return points
