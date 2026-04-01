"""机器人联动页面。"""

from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from config import ROBOT_UI_POLL_INTERVAL_MS
from exceptions import AppError
from measurement_controller import MeasurementController


class RobotLinkTab(QWidget):
    """阶段 1 机器人联动页面。"""

    def __init__(self, controller: MeasurementController) -> None:
        super().__init__()
        self.controller = controller
        self._last_protocol_text = ""
        self._init_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh_view)
        self._timer.start(ROBOT_UI_POLL_INTERVAL_MS)
        self.refresh_view()

    def _init_ui(self) -> None:
        root_layout = QVBoxLayout(self)
        root_layout.setSpacing(10)

        root_layout.addWidget(self._build_server_group())
        root_layout.addWidget(self._build_status_group())
        root_layout.addWidget(self._build_points_group())
        root_layout.addWidget(self._build_result_group())
        root_layout.addWidget(self._build_protocol_group())

    def _build_server_group(self) -> QGroupBox:
        group = QGroupBox("机器人联动服务")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("服务地址:"))
        self.lbl_server_address = QLabel("--")
        layout.addWidget(self.lbl_server_address)

        layout.addSpacing(20)
        layout.addWidget(QLabel("服务状态:"))
        self.lbl_server_state = QLabel("未启动")
        layout.addWidget(self.lbl_server_state)
        layout.addStretch()

        self.btn_start_server = QPushButton("启动 Socket 服务")
        self.btn_start_server.clicked.connect(self._on_start_server)
        layout.addWidget(self.btn_start_server)

        self.btn_stop_server = QPushButton("停止 Socket 服务")
        self.btn_stop_server.clicked.connect(self._on_stop_server)
        layout.addWidget(self.btn_stop_server)

        return group

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("当前任务状态")
        layout = QGridLayout(group)

        self.status_labels = self._create_value_labels(
            layout,
            [
                "PolyWorks 连接",
                "客户端",
                "REQ",
                "任务",
                "坐标系",
                "会话状态",
                "目标点数",
                "已收点数",
            ],
        )
        return group

    def _build_points_group(self) -> QGroupBox:
        group = QGroupBox("实时接收点")
        layout = QVBoxLayout(group)

        self.points_table = QTableWidget(0, 4)
        self.points_table.setHorizontalHeaderLabels(["IDX", "X", "Y", "Z"])
        self.points_table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.points_table)
        return group

    def _build_result_group(self) -> QGroupBox:
        group = QGroupBox("拟合圆结果")
        layout = QGridLayout(group)

        self.result_labels = self._create_value_labels(
            layout,
            ["圆心 X", "圆心 Y", "圆心 Z", "直径", "法向量 X", "法向量 Y", "法向量 Z"],
        )
        return group

    def _build_protocol_group(self) -> QGroupBox:
        group = QGroupBox("协议日志")
        layout = QVBoxLayout(group)

        self.protocol_text = QTextEdit()
        self.protocol_text.setReadOnly(True)
        self.protocol_text.setMaximumHeight(220)
        layout.addWidget(self.protocol_text)

        return group

    def _create_value_labels(self, layout: QGridLayout, titles: list[str]) -> dict[str, QLabel]:
        labels: dict[str, QLabel] = {}
        for index, title in enumerate(titles):
            row = index // 2
            column = (index % 2) * 2
            title_label = QLabel(f"{title}:")
            value_label = QLabel("--")
            layout.addWidget(title_label, row, column)
            layout.addWidget(value_label, row, column + 1)
            labels[title] = value_label
        return labels

    def _on_start_server(self) -> None:
        try:
            self.controller.start_robot_server()
            self.refresh_view()
        except AppError as exc:
            QMessageBox.critical(self, "启动失败", str(exc))

    def _on_stop_server(self) -> None:
        self.controller.stop_robot_server()
        self.refresh_view()

    def refresh_view(self) -> None:
        """刷新页面显示。"""
        snapshot = self.controller.get_robot_ui_snapshot()
        session = snapshot["session"]

        self.lbl_server_address.setText(f"{snapshot['host']}:{snapshot['port']}")
        self.lbl_server_state.setText("运行中" if snapshot["server_running"] else "未启动")
        self.btn_start_server.setEnabled(not snapshot["server_running"])
        self.btn_stop_server.setEnabled(snapshot["server_running"])

        self.status_labels["PolyWorks 连接"].setText("已连接" if self.controller.connected else "未连接")
        self.status_labels["客户端"].setText(snapshot["active_client"] or "--")

        if session is None:
            self.status_labels["REQ"].setText("--")
            self.status_labels["任务"].setText("--")
            self.status_labels["坐标系"].setText("--")
            self.status_labels["会话状态"].setText("--")
            self.status_labels["目标点数"].setText("--")
            self.status_labels["已收点数"].setText("--")
            self._fill_points_table([])
            self._clear_result_labels()
        else:
            self.status_labels["REQ"].setText(str(session["req"]))
            self.status_labels["任务"].setText(str(session["task"]))
            self.status_labels["坐标系"].setText(str(session["frame"]))
            self.status_labels["会话状态"].setText(str(session["state"]))
            self.status_labels["目标点数"].setText(str(session["expected_points"]))
            self.status_labels["已收点数"].setText(str(session["received_points"]))
            self._fill_points_table(session["points"])
            self._fill_result(session.get("last_result"))

        protocol_text = "\n".join(snapshot["protocol_history"])
        if protocol_text != self._last_protocol_text:
            self.protocol_text.setPlainText(protocol_text)
            self._last_protocol_text = protocol_text

    def _fill_points_table(self, points: list[dict]) -> None:
        self.points_table.setRowCount(len(points))
        for row, point in enumerate(points):
            values = [
                str(point["idx"]),
                f"{point['x']:.6f}",
                f"{point['y']:.6f}",
                f"{point['z']:.6f}",
            ]
            for column, text in enumerate(values):
                self.points_table.setItem(row, column, QTableWidgetItem(text))

    def _fill_result(self, result: dict | None) -> None:
        if result is None:
            self._clear_result_labels()
            return

        center = result["center"]
        normal = result["normal"]
        self.result_labels["圆心 X"].setText(f"{center[0]:.6f}")
        self.result_labels["圆心 Y"].setText(f"{center[1]:.6f}")
        self.result_labels["圆心 Z"].setText(f"{center[2]:.6f}")
        self.result_labels["直径"].setText(f"{result['diameter']:.6f}")
        self.result_labels["法向量 X"].setText(f"{normal[0]:.6f}")
        self.result_labels["法向量 Y"].setText(f"{normal[1]:.6f}")
        self.result_labels["法向量 Z"].setText(f"{normal[2]:.6f}")

    def _clear_result_labels(self) -> None:
        for label in self.result_labels.values():
            label.setText("--")
