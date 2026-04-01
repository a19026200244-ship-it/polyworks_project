"""应用配置。

这个文件只放“不会频繁变化”的固定参数，
这样其他模块就不用到处写死常量。
"""

from pathlib import Path

# 窗口显示配置
# 这些参数只影响界面外观，不参与测量计算。
WINDOW_TITLE = "PolyWorks 机械臂测量接口"
WINDOW_MIN_WIDTH = 860
WINDOW_MIN_HEIGHT = 900
APP_FONT_FAMILY = "Microsoft YaHei"
APP_FONT_SIZE = 9

# 功能 A / B 输入时，至少需要 3 个点。
# 三点可以唯一确定一个平面，也能给出一个基础圆。
MIN_POINTS_FOR_PLANE = 3
MIN_POINTS_FOR_CIRCLE = 3

# 原三平面交点功能固定要求 9 个点。
# 当前界面约定：
# 顶部 3 个点 -> 平面 1
# 正面 3 个点 -> 平面 2
# 侧面 3 个点 -> 平面 3
EXPECTED_POINT_COUNT = 9
POINT_GROUPS = ["顶部"] * 3 + ["正面"] * 3 + ["侧面"] * 3
POINT_GROUP_COLORS = {
    "顶部": (230, 245, 255),
    "正面": (230, 255, 230),
    "侧面": (255, 245, 230),
}

# PolyWorks 导出过程中会临时用到的文件。
# Python 会生成宏文件让 PolyWorks 执行，
# 再把结果暂存到文本文件里读回。
RESULT_FILE = Path("D:/polyworks_temp_result.txt")
TEMP_MACRO_FILE = Path("D:/pw_get_point.pwmacro")

# PolyWorks 中创建对象时统一使用的名称。
# 集中写在这里，后面如果要改命名规则，不用全项目到处找。
POINT_NAME_TEMPLATE = "点 {index}"
PLANE_NAMES = ["平面 1", "平面 2", "平面 3"]
INTERSECTION_POINT_NAME = "点 10"
