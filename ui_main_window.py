"""主窗口界面。

界面层只关心三件事：
1. 把按钮、表格、日志这些控件摆出来
2. 接收用户操作
3. 调用业务层并显示结果

可以把它理解成“界面调度中心”：
- 它自己不做复杂几何计算
- 它主要负责组织流程、更新状态、展示结果
"""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import threading

from PySide6.QtCore import QObject, QTimer, Qt, Signal, Slot
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from app_logger import get_logger
from calibration_service import HAS_NUMPY
from config import (
    EXPECTED_POINT_COUNT,
    POINT_GROUP_COLORS,
    POINT_GROUPS,
    ROBOT_MAIN_THREAD_TASK_TIMEOUT,
    WINDOW_MIN_HEIGHT,
    WINDOW_MIN_WIDTH,
    WINDOW_TITLE,
)
from data_parser import parse_points_file, parse_points_text
from exceptions import AppError
from measurement_controller import MeasurementController
from polyworks_com import HAS_COM
from services import validate_points
from ui_calibration_tab import CalibrationTab
from ui_robot_link_tab import RobotLinkTab

# 这个文件适合从“用户操作流程”的角度去理解：
# 用户点按钮 -> 对应的 `_on_xxx()` 被触发 -> 调用业务层 -> 更新界面结果。


class _MainThreadTask:
    """保存一次主线程任务的执行结果。"""

    def __init__(self, callback: Callable[[], object]) -> None:
        self.callback = callback
        self.done = threading.Event()
        self.result: object | None = None
        self.error: Exception | None = None


class _MainThreadExecutor(QObject):
    """把后台线程提交的任务切回 Qt 主线程执行。"""

    execute_requested = Signal(object)

    def __init__(self, timeout_seconds: float, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._timeout_seconds = timeout_seconds
        self.execute_requested.connect(self._run_task, Qt.QueuedConnection)

    def submit(self, callback: Callable[[], object]) -> object:
        """后台线程会阻塞等待，直到主线程执行完成并返回结果。"""
        if threading.current_thread() is threading.main_thread():
            return callback()

        task = _MainThreadTask(callback)
        self.execute_requested.emit(task)

        if not task.done.wait(self._timeout_seconds):
            raise TimeoutError(f"主线程测量任务超时，等待超过 {self._timeout_seconds:.1f} 秒")

        if task.error is not None:
            raise task.error

        return task.result

    @Slot(object)
    def _run_task(self, task: _MainThreadTask) -> None:
        try:
            task.result = task.callback()
        except Exception as exc:
            task.error = exc
        finally:
            task.done.set()


class MainWindow(QMainWindow):
    """程序主窗口。

    窗口里包含三个核心功能页：
    1. 三平面交点
    2. 多点拟合圆
    3. 两面交线
    """

    def __init__(self) -> None:
        super().__init__()
        self.logger = get_logger("ui")
        # controller 负责在 UI、业务层、日志和阶段 0 基础设施之间做调度。
        self.controller = MeasurementController()
        self._main_thread_executor = _MainThreadExecutor(ROBOT_MAIN_THREAD_TASK_TIMEOUT, self)
        self.controller.set_log_callback(self._append_log)
        self.controller.set_main_thread_executor(self._main_thread_executor.submit)
        # 为了尽量少改动现有 UI 逻辑，这里保留一个连接器别名。
        self.connector = self.controller.connector
        # 这里保存“三平面交点”功能加载得到的 9 个点。
        self.points: list[tuple[float, float, float]] = []
        self._init_ui()

    def _init_ui(self) -> None:
        """创建主界面布局和控件。"""
        # 这一段只负责“把界面搭出来”，并不做实际测量。
        self.setWindowTitle(WINDOW_TITLE)
        self.setMinimumSize(WINDOW_MIN_WIDTH, WINDOW_MIN_HEIGHT)

        # 主窗口里通常会先放一个 central widget，
        # 再把布局和控件放进这个 central widget。
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 最外层采用纵向布局：
        # 上面是连接区，中间是标签页，下面是日志区。
        root_layout = QVBoxLayout(central_widget)
        root_layout.setSpacing(10)
        root_layout.setContentsMargins(15, 15, 15, 15)

        root_layout.addWidget(self._build_connection_group())

        # 三个标签页分别承载三个不同的测量功能。
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_intersection_tab(), '三平面交点')
        self.tabs.addTab(self._build_circle_tab(), '拟合圆')
        self.tabs.addTab(self._build_line_tab(), '两面交线')
        self.robot_link_tab = RobotLinkTab(self.controller)
        self.tabs.addTab(self.robot_link_tab, '机器人联动')
        self.calibration_tab = CalibrationTab(self.controller)
        self.tabs.addTab(self.calibration_tab, '标定与变换')
        root_layout.addWidget(self.tabs)

        root_layout.addWidget(self._build_log_group())

        self.statusBar().showMessage('就绪')
        self._log('程序启动完成')
        if not HAS_COM:
            self._log('警告: 未检测到 comtypes，请先运行 pip install comtypes')
        if not HAS_NUMPY:
            self._log('警告: 未检测到 numpy，标定与变换功能暂不可用')
        self._update_action_buttons()

        self._log_timer = QTimer(self)
        self._log_timer.timeout.connect(self._flush_controller_logs)
        self._log_timer.start(300)

    def _build_connection_group(self) -> QGroupBox:
        """创建 PolyWorks 连接区域。"""
        group = QGroupBox('PolyWorks 连接')
        layout = QHBoxLayout(group)

        self.lbl_status = QLabel('● 未连接')
        self.lbl_status.setStyleSheet('color: red; font-weight: bold;')
        layout.addWidget(self.lbl_status)
        layout.addStretch()

        # `clicked.connect(...)` 表示按钮点击后自动调用某个函数。
        self.btn_connect = QPushButton('连接 PolyWorks')
        self.btn_connect.setFixedWidth(150)
        self.btn_connect.clicked.connect(self._on_connect)
        layout.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton('断开连接')
        self.btn_disconnect.setFixedWidth(100)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        layout.addWidget(self.btn_disconnect)

        return group

    def _build_intersection_tab(self) -> QWidget:
        """创建“三平面交点”页面。"""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(self._build_file_group())
        layout.addWidget(self._build_points_group())
        layout.addWidget(self._build_intersection_button())
        layout.addWidget(self._build_intersection_result_group())
        return page

    def _build_circle_tab(self) -> QWidget:
        """创建“拟合圆”页面。"""
        page = QWidget()
        layout = QVBoxLayout(page)

        input_group = QGroupBox('输入圆上的点（至少 3 个）')
        input_layout = QVBoxLayout(input_group)
        input_layout.addWidget(QLabel('每行一个点，格式支持: x,y,z 或 x y z'))

        # QTextEdit 适合输入多行文本，所以这里用它来粘贴多组点坐标。
        self.circle_text = QTextEdit()
        self.circle_text.setPlaceholderText('示例:\n10,0,0\n0,10,0\n-10,0,0\n0,-10,0')
        self.circle_text.setMaximumHeight(220)
        input_layout.addWidget(self.circle_text)

        # 这一行专门放“加载点”和“开始计算”两个操作按钮。
        circle_btn_row = QHBoxLayout()
        btn_load_circle = QPushButton('从文件加载点')
        btn_load_circle.clicked.connect(lambda: self._load_points_to_text_edit(self.circle_text))
        circle_btn_row.addWidget(btn_load_circle)
        circle_btn_row.addStretch()

        self.btn_circle = QPushButton('计算拟合圆')
        self.btn_circle.setEnabled(False)
        self.btn_circle.clicked.connect(self._on_fit_circle)
        circle_btn_row.addWidget(self.btn_circle)
        input_layout.addLayout(circle_btn_row)
        layout.addWidget(input_group)

        result_group = QGroupBox('拟合圆结果')
        result_layout = QGridLayout(result_group)
        self.circle_result_labels = self._create_value_labels(
            result_layout,
            ['圆心 X', '圆心 Y', '圆心 Z', '直径', '法向量 X', '法向量 Y', '法向量 Z'],
        )
        layout.addWidget(result_group)
        layout.addStretch()
        return page

    def _build_line_tab(self) -> QWidget:
        """创建“两面交线”页面。"""
        page = QWidget()
        layout = QVBoxLayout(page)

        input_group = QGroupBox('输入两个平面的点')
        input_layout = QHBoxLayout(input_group)

        # 左右两列分别输入两个平面的点集。
        plane1_group = QVBoxLayout()
        plane1_group.addWidget(QLabel('平面 1 点集（至少 3 个）'))
        self.line_plane1_text = QTextEdit()
        self.line_plane1_text.setPlaceholderText('示例:\n0,0,0\n10,0,0\n0,10,0')
        plane1_group.addWidget(self.line_plane1_text)
        btn_load_plane1 = QPushButton('加载平面 1 文件')
        btn_load_plane1.clicked.connect(lambda: self._load_points_to_text_edit(self.line_plane1_text))
        plane1_group.addWidget(btn_load_plane1)
        input_layout.addLayout(plane1_group)

        plane2_group = QVBoxLayout()
        plane2_group.addWidget(QLabel('平面 2 点集（至少 3 个）'))
        self.line_plane2_text = QTextEdit()
        self.line_plane2_text.setPlaceholderText('示例:\n0,0,0\n0,10,0\n0,0,10')
        plane2_group.addWidget(self.line_plane2_text)
        btn_load_plane2 = QPushButton('加载平面 2 文件')
        btn_load_plane2.clicked.connect(lambda: self._load_points_to_text_edit(self.line_plane2_text))
        plane2_group.addWidget(btn_load_plane2)
        input_layout.addLayout(plane2_group)

        layout.addWidget(input_group)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.btn_line = QPushButton('计算两面交线')
        self.btn_line.setEnabled(False)
        self.btn_line.clicked.connect(self._on_intersect_planes)
        btn_row.addWidget(self.btn_line)
        layout.addLayout(btn_row)

        result_group = QGroupBox('两面交线结果')
        result_layout = QGridLayout(result_group)
        self.line_result_labels = self._create_value_labels(
            result_layout,
            ['线上点 X', '线上点 Y', '线上点 Z', '方向 X', '方向 Y', '方向 Z'],
        )
        layout.addWidget(result_group)
        layout.addStretch()
        return page

    def _build_file_group(self) -> QGroupBox:
        """创建三平面交点功能的文件选择区域。"""
        group = QGroupBox('文件选择')
        layout = QHBoxLayout(group)
        layout.addWidget(QLabel('坐标文件:'))

        self.lbl_file = QLabel('未选择')
        self.lbl_file.setStyleSheet('color: gray;')
        layout.addWidget(self.lbl_file, 1)

        btn_load = QPushButton('加载坐标文件')
        btn_load.clicked.connect(self._on_load_file)
        layout.addWidget(btn_load)

        return group

    def _build_points_group(self) -> QGroupBox:
        """创建三平面交点功能的点表格区域。"""
        group = QGroupBox('测量点坐标（3 组 x 3 点 = 9 个点）')
        layout = QVBoxLayout(group)

        # 这里固定创建 9 行，因为三平面交点功能固定需要 9 个点。
        self.table = QTableWidget(EXPECTED_POINT_COUNT, 4)
        self.table.setHorizontalHeaderLabels(['点名', 'X', 'Y', 'Z'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setMaximumHeight(280)
        layout.addWidget(self.table)

        # 这里强调 9 个点的分组顺序，避免导入时顺序放错。
        info = QLabel('顶部: 点 1-3  |  正面: 点 4-6  |  侧面: 点 7-9')
        info.setStyleSheet('color: #666; font-size: 11px;')
        info.setAlignment(Qt.AlignCenter)
        layout.addWidget(info)

        return group

    def _build_intersection_button(self) -> QPushButton:
        """创建三平面交点的执行按钮。"""
        self.btn_exec = QPushButton('执行计算（建立平面并求交点）')
        self.btn_exec.setFixedHeight(45)
        self.btn_exec.setStyleSheet(
            'QPushButton { background-color: #0078d4; color: white; '
            'font-size: 14px; font-weight: bold; border: none; border-radius: 5px; }'
            'QPushButton:hover { background-color: #106ebe; }'
            'QPushButton:pressed { background-color: #005a9e; }'
            'QPushButton:disabled { background-color: #ccc; color: #888; }'
        )
        self.btn_exec.setEnabled(False)
        self.btn_exec.clicked.connect(self._on_execute)
        return self.btn_exec

    def _build_intersection_result_group(self) -> QGroupBox:
        """创建三平面交点结果显示区域。"""
        group = QGroupBox('计算结果 - 三平面交点')
        layout = QVBoxLayout(group)
        coords_layout = QHBoxLayout()

        self.result_labels: dict[str, QLabel] = {}
        for axis in ('X', 'Y', 'Z'):
            frame = QFrame()
            frame.setFrameStyle(QFrame.StyledPanel)
            frame.setStyleSheet('background-color: #f5f5f5; border-radius: 5px; padding: 10px;')

            frame_layout = QVBoxLayout(frame)
            axis_label = QLabel(axis)
            axis_label.setAlignment(Qt.AlignCenter)
            axis_label.setStyleSheet('font-size: 14px; font-weight: bold; color: #0078d4;')
            frame_layout.addWidget(axis_label)

            value_label = QLabel('--')
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet('font-size: 18px; font-weight: bold;')
            frame_layout.addWidget(value_label)

            self.result_labels[axis] = value_label
            coords_layout.addWidget(frame)

        layout.addLayout(coords_layout)
        return group

    def _build_log_group(self) -> QGroupBox:
        """创建日志显示区域。"""
        group = QGroupBox('运行日志')
        layout = QVBoxLayout(group)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(160)
        self.log_text.setStyleSheet('font-family: Consolas, monospace; font-size: 11px;')
        layout.addWidget(self.log_text)

        return group

    def _create_value_labels(self, layout: QGridLayout, titles: list[str]) -> dict[str, QLabel]:
        """批量创建结果显示标签。

        拟合圆和两面交线的结果展示形式很像，
        所以抽出这个公共方法，减少重复代码。
        """
        labels: dict[str, QLabel] = {}
        for index, title in enumerate(titles):
            row = index // 2
            column = (index % 2) * 2
            title_label = QLabel(f'{title}:')
            value_label = QLabel('--')
            value_label.setStyleSheet('font-weight: bold; color: #0078d4;')
            layout.addWidget(title_label, row, column)
            layout.addWidget(value_label, row, column + 1)
            labels[title] = value_label
        return labels

    def _append_log(self, message: str) -> None:
        """只向界面日志区域追加一条消息。"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{timestamp}] {message}')
        # 让日志尽快刷新到界面上，用户反馈会更及时。
        QApplication.processEvents()

    def _log(self, message: str) -> None:
        """同时写入文件日志和界面日志。"""
        self.logger.info(message)
        self._append_log(message)

    def _flush_controller_logs(self) -> None:
        """把后台线程积累的日志安全刷新到界面。"""
        for message in self.controller.consume_pending_logs():
            self._append_log(message)

    def _update_action_buttons(self) -> None:
        """根据连接状态刷新三个功能按钮。

        统一把“按钮能不能点”的判断放在这里管理：
        - 三平面交点：要求已连接，并且已经加载 9 个点
        - 拟合圆：要求已连接
        - 两面交线：要求已连接
        """
        self.btn_exec.setEnabled(self.controller.connected and len(self.points) == EXPECTED_POINT_COUNT)
        self.btn_circle.setEnabled(self.controller.connected)
        self.btn_line.setEnabled(self.controller.connected)

    def _set_connection_status(self, connected: bool) -> None:
        """统一更新连接状态显示。"""
        # 这里只处理“显示状态”，不负责真正去连接或断开。
        if connected:
            self.lbl_status.setText('● 已连接')
            self.lbl_status.setStyleSheet('color: green; font-weight: bold;')
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.statusBar().showMessage('已连接')
        else:
            self.lbl_status.setText('● 未连接')
            self.lbl_status.setStyleSheet('color: red; font-weight: bold;')
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.statusBar().showMessage('已断开')

    def _clear_result_labels(self, labels: dict[str, QLabel], text: str) -> None:
        """把一组结果标签统一清空或改成同一个提示文本。"""
        for label in labels.values():
            label.setText(text)

    def _fill_points_table(self) -> None:
        """把已经读取的 9 个点显示到表格里。

        不同平面会使用不同背景色，便于用户快速识别点分组。
        """
        for row_index, (x, y, z) in enumerate(self.points):
            # 根据点属于哪一组平面，给整行加不同背景色。
            group_name = POINT_GROUPS[row_index]
            background = QColor(*POINT_GROUP_COLORS[group_name])
            row_data = [
                f'点 {row_index + 1} ({group_name})',
                f'{x:.4f}',
                f'{y:.4f}',
                f'{z:.4f}',
            ]

            for column_index, text in enumerate(row_data):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                item.setBackground(background)
                self.table.setItem(row_index, column_index, item)

    def _load_points_to_text_edit(self, editor: QTextEdit) -> None:
        """把文件中的点坐标加载到某个文本框里。

        这个方法给“拟合圆”和“两面交线”两个页面复用。
        读到文件后，会转成统一的 `x,y,z` 文本格式写入文本框。
        """
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            '选择坐标文件',
            '',
            '文本文件 (*.txt);;所有文件 (*)',
        )
        if not filepath:
            # 用户取消选择文件时，直接返回即可。
            return

        try:
            points = parse_points_file(filepath)
            # 统一转换成 `x,y,z` 的文本格式，方便后面继续解析。
            lines = [f'{x},{y},{z}' for x, y, z in points]
            editor.setPlainText('\n'.join(lines))
            self._log(f'已加载文本点集: {filepath}')
        except Exception as exc:
            self._log(f'加载文本点集失败: {exc}')
            QMessageBox.warning(self, '加载失败', str(exc))

    def _on_connect(self) -> None:
        """处理“连接 PolyWorks”按钮。"""
        try:
            self.controller.connect_polyworks()
            self._set_connection_status(True)
            self._update_action_buttons()
        except AppError as exc:
            QMessageBox.critical(self, '连接失败', str(exc))

    def _on_disconnect(self) -> None:
        """处理“断开连接”按钮。"""
        self.controller.disconnect_polyworks()
        self._set_connection_status(False)
        self._update_action_buttons()

    def _on_load_file(self) -> None:
        """处理三平面交点页面里的文件加载动作。"""
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            '选择坐标文件',
            '',
            '文本文件 (*.txt);;所有文件 (*)',
        )
        if not filepath:
            return

        try:
            points = parse_points_file(filepath)
            validate_points(points)
            self.points = points
            self.lbl_file.setText(filepath)
            self.lbl_file.setStyleSheet('color: black;')
            self._fill_points_table()
            self._update_action_buttons()
            self._log(f'已加载 {len(self.points)} 个坐标点: {filepath}')
        except Exception as exc:
            self._log(f'加载失败: {exc}')
            QMessageBox.warning(self, '加载失败', str(exc))

    def _on_execute(self) -> None:
        """执行三平面交点计算。

        这个入口方法负责：
        1. 检查是否已连接 PolyWorks
        2. 检查是否已加载 9 个点
        3. 调用业务层执行计算
        4. 把结果显示到界面上
        """
        if not self.controller.connected:
            QMessageBox.warning(self, '未连接', '请先连接 PolyWorks')
            return

        if len(self.points) != EXPECTED_POINT_COUNT:
            QMessageBox.warning(self, '数据不足', '请先加载包含 9 个点的坐标文件')
            return

        # 先给用户一个明确反馈：程序已经开始算了。
        self._clear_result_labels(self.result_labels, '计算中...')
        self.btn_exec.setEnabled(False)
        # 把鼠标改成忙碌状态，避免用户误以为程序没反应。
        QApplication.setOverrideCursor(Qt.WaitCursor)

        try:
            x, y, z = self.controller.run_intersection_measurement(self.points)
            self.result_labels['X'].setText(f'{x:.6f}')
            self.result_labels['Y'].setText(f'{y:.6f}')
            self.result_labels['Z'].setText(f'{z:.6f}')
            self.statusBar().showMessage('计算完成')
        except AppError as exc:
            self._clear_result_labels(self.result_labels, '错误')
            self.statusBar().showMessage('执行出错')
            QMessageBox.critical(self, '执行出错', str(exc))
        finally:
            # 无论成功还是失败，都要恢复鼠标状态。
            QApplication.restoreOverrideCursor()
            self._update_action_buttons()

    def _on_fit_circle(self) -> None:
        """执行多点拟合圆。

        界面层只负责收集点文本和显示结果，
        真正的几何构造与计算由业务层调用 PolyWorks 完成。
        """
        if not self.controller.connected:
            QMessageBox.warning(self, '未连接', '请先连接 PolyWorks')
            return

        self._clear_result_labels(self.circle_result_labels, '计算中...')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            # 先把文本框里的内容解析成点列表。
            points = parse_points_text(self.circle_text.toPlainText())
            result = self.controller.fit_circle_from_points(points)

            # 最后把业务层返回的结果对象显示回界面。
            self.circle_result_labels['圆心 X'].setText(f'{result.center[0]:.6f}')
            self.circle_result_labels['圆心 Y'].setText(f'{result.center[1]:.6f}')
            self.circle_result_labels['圆心 Z'].setText(f'{result.center[2]:.6f}')
            self.circle_result_labels['直径'].setText(f'{result.diameter:.6f}')
            self.circle_result_labels['法向量 X'].setText(f'{result.normal[0]:.6f}')
            self.circle_result_labels['法向量 Y'].setText(f'{result.normal[1]:.6f}')
            self.circle_result_labels['法向量 Z'].setText(f'{result.normal[2]:.6f}')
            self.statusBar().showMessage('拟合圆完成')
        except AppError as exc:
            self._clear_result_labels(self.circle_result_labels, '错误')
            QMessageBox.critical(self, '拟合圆出错', str(exc))
        finally:
            QApplication.restoreOverrideCursor()

    def _on_intersect_planes(self) -> None:
        """执行两面交线计算。

        用户输入两组点后，这里会调用业务层在 PolyWorks 中：
        1. 分别构造两个最佳拟合平面
        2. 再求两个平面的交线
        """
        if not self.controller.connected:
            QMessageBox.warning(self, '未连接', '请先连接 PolyWorks')
            return

        self._clear_result_labels(self.line_result_labels, '计算中...')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            # 先分别解析两个文本框里的点。
            plane1_points = parse_points_text(self.line_plane1_text.toPlainText())
            plane2_points = parse_points_text(self.line_plane2_text.toPlainText())
            result = self.controller.intersect_plane_groups(plane1_points, plane2_points)

            # 然后把结果对象拆开，显示到标签里。
            self.line_result_labels['线上点 X'].setText(f'{result.point[0]:.6f}')
            self.line_result_labels['线上点 Y'].setText(f'{result.point[1]:.6f}')
            self.line_result_labels['线上点 Z'].setText(f'{result.point[2]:.6f}')
            self.line_result_labels['方向 X'].setText(f'{result.direction[0]:.6f}')
            self.line_result_labels['方向 Y'].setText(f'{result.direction[1]:.6f}')
            self.line_result_labels['方向 Z'].setText(f'{result.direction[2]:.6f}')
            self.statusBar().showMessage('两面交线完成')
        except AppError as exc:
            self._clear_result_labels(self.line_result_labels, '错误')
            QMessageBox.critical(self, '两面交线出错', str(exc))
        finally:
            QApplication.restoreOverrideCursor()

    def closeEvent(self, event) -> None:
        """窗口关闭时，顺手断开 PolyWorks 连接。"""
        # Qt 在窗口关闭时会自动调用这个函数。
        self.controller.shutdown()
        super().closeEvent(event)

