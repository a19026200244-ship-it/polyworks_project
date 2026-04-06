"""应用配置。

这个文件只放“不会频繁变化”的固定参数，
这样其他模块就不用到处写死常量。
"""

from pathlib import Path

# 这个文件适合集中保存“全局固定参数”。
# 如果你以后想改窗口标题、最少点数要求、临时文件路径，
# 一般都应该先来这里修改。

# 项目根目录。
# 后面日志、数据库、示例文件等路径都尽量基于它来拼接，
# 这样项目移动到别的电脑后更容易保持一致。
APP_BASE_DIR = Path(__file__).resolve().parent

# 运行期目录配置。
LOG_DIR = APP_BASE_DIR / "logs"
LOG_FILE = LOG_DIR / "app.log"
LOG_MAX_BYTES = 1_048_576
LOG_BACKUP_COUNT = 3

# 为后续数据库功能预留默认路径。
DATA_DIR = APP_BASE_DIR / "runtime_data"
DATABASE_FILE = DATA_DIR / "measurements.sqlite3"
TRANSFORM_REPOSITORY_FILE = DATA_DIR / "transforms.json"

# 机器人联动相关配置。
# 阶段 1 先只支持：
# - 单客户端
# - 单任务 CIRCLE
# - 一行一条消息的文本协议
ROBOT_SERVER_HOST = "127.0.0.1"
ROBOT_SERVER_PORT = 18765
ROBOT_SERVER_ENCODING = "utf-8"
ROBOT_SERVER_BUFFER_SIZE = 4096
ROBOT_SERVER_SOCKET_TIMEOUT = 3.0
ROBOT_SERVER_REPLY_WAIT = 0.2
ROBOT_SERVER_ENABLED = True
ROBOT_UI_POLL_INTERVAL_MS = 300
ROBOT_PROTOCOL_HISTORY_LIMIT = 100
ROBOT_TASK_HISTORY_LIMIT = 50
ROBOT_MAIN_THREAD_TASK_TIMEOUT = 30.0
ROBOT_DEFAULT_TASK = "CIRCLE"
ROBOT_DEFAULT_FRAME = "PW"
ROBOT_DEFAULT_POINT_COUNT = 8
PW_FRAME_NAME = "PW"
ROBOT_BASE_FRAME_NAME = "ROBOT_BASE"

# 保留阶段 0 配置别名，兼容之前已经写好的代码和测试命令。
SIMULATOR_HOST = ROBOT_SERVER_HOST
SIMULATOR_PORT = ROBOT_SERVER_PORT
SIMULATOR_ENCODING = ROBOT_SERVER_ENCODING
SIMULATOR_BUFFER_SIZE = ROBOT_SERVER_BUFFER_SIZE
SIMULATOR_SOCKET_TIMEOUT = ROBOT_SERVER_SOCKET_TIMEOUT
SIMULATOR_LISTENER_ENABLED = ROBOT_SERVER_ENABLED

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
MIN_CALIBRATION_POINT_PAIRS = 4

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
#
# 注意：这里是“默认模板路径”。
# 实际运行时，services.py 会继续在文件名里加时间戳，
# 生成真正的一次性临时文件，避免多次运行时重名。
RESULT_FILE = Path("D:/polyworks_temp_result.txt")
TEMP_MACRO_FILE = Path("D:/pw_get_point.pwmacro")

# PolyWorks 中创建对象时统一使用的名称。
# 集中写在这里，后面如果要改命名规则，不用全项目到处找。
POINT_NAME_TEMPLATE = "点 {index}"
PLANE_NAMES = ["平面 1", "平面 2", "平面 3"]
INTERSECTION_POINT_NAME = "点 10"
