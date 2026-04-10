"""标定与变换页面。"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from polyworks_robot_arm.calibration.calibration_service import HAS_NUMPY
from polyworks_robot_arm.calibration.transform_models import CalibrationSolveResult, Point3D
from polyworks_robot_arm.common.config import PW_FRAME_NAME, ROBOT_BASE_FRAME_NAME
from polyworks_robot_arm.common.data_parser import parse_points_file
from polyworks_robot_arm.common.exceptions import AppError
from polyworks_robot_arm.controllers.measurement_controller import MeasurementController

# 这个页面是 Stage 3 的“人工操作入口”。
# 你可以把它理解成一条很直的流程:
# 1. 读入两组对应点
# 2. 在界面上按索引自动配对
# 3. 点击求解得到刚体变换
# 4. 查看误差
# 5. 保存并设为当前生效变换


class CalibrationTab(QWidget):
    """阶段 3 标定与变换页面。"""

    def __init__(self, controller: MeasurementController) -> None:
        super().__init__()
        self.controller = controller
        # 这两个列表分别缓存“从文件加载进来的两组点”。
        self.pw_points: list[Point3D] = []
        self.robot_points: list[Point3D] = []
        # latest_result 保存最近一次求解结果，供“保存变换”按钮复用。
        self.latest_result: CalibrationSolveResult | None = None
        self._init_ui()
        self.refresh_repository_view()

    def _init_ui(self) -> None:
        # 页面布局顺序基本就是推荐的人工使用顺序。
        root_layout = QVBoxLayout(self)
        root_layout.setSpacing(10)

        root_layout.addWidget(self._build_transform_selector_group())
        root_layout.addWidget(self._build_input_group())
        root_layout.addWidget(self._build_pair_group())
        root_layout.addWidget(self._build_action_group())
        root_layout.addWidget(self._build_residual_group())

    def _build_transform_selector_group(self) -> QGroupBox:
        group = QGroupBox("当前变换选择区")
        layout = QVBoxLayout(group)

        top_row = QHBoxLayout()
        top_row.addWidget(QLabel("已保存变换:"))
        self.cmb_transforms = QComboBox()
        top_row.addWidget(self.cmb_transforms, 1)

        self.btn_refresh_transforms = QPushButton("刷新列表")
        self.btn_refresh_transforms.clicked.connect(self.refresh_repository_view)
        top_row.addWidget(self.btn_refresh_transforms)

        self.btn_load_transform = QPushButton("加载并设为当前")
        self.btn_load_transform.clicked.connect(self._on_load_transform)
        top_row.addWidget(self.btn_load_transform)
        layout.addLayout(top_row)

        self.lbl_active_transform = QLabel("当前激活变换: --")
        layout.addWidget(self.lbl_active_transform)

        self.active_transform_text = QTextEdit()
        self.active_transform_text.setReadOnly(True)
        self.active_transform_text.setMaximumHeight(120)
        layout.addWidget(self.active_transform_text)

        return group

    def _build_input_group(self) -> QGroupBox:
        group = QGroupBox("标定点输入")
        layout = QHBoxLayout(group)

        pw_group = QVBoxLayout()
        pw_header = QHBoxLayout()
        pw_header.addWidget(QLabel("PolyWorks 点表"))
        pw_header.addStretch()
        self.btn_load_pw = QPushButton("加载 PW 点文件")
        self.btn_load_pw.clicked.connect(self._on_load_pw_points)
        pw_header.addWidget(self.btn_load_pw)
        pw_group.addLayout(pw_header)
        self.tbl_pw_points = self._create_point_table()
        pw_group.addWidget(self.tbl_pw_points)
        self.lbl_pw_count = QLabel("点数: 0")
        pw_group.addWidget(self.lbl_pw_count)
        layout.addLayout(pw_group)

        robot_group = QVBoxLayout()
        robot_header = QHBoxLayout()
        robot_header.addWidget(QLabel("机器人点表"))
        robot_header.addStretch()
        self.btn_load_robot = QPushButton("加载机器人点文件")
        self.btn_load_robot.clicked.connect(self._on_load_robot_points)
        robot_header.addWidget(self.btn_load_robot)
        robot_group.addLayout(robot_header)
        self.tbl_robot_points = self._create_point_table()
        robot_group.addWidget(self.tbl_robot_points)
        self.lbl_robot_count = QLabel("点数: 0")
        robot_group.addWidget(self.lbl_robot_count)
        layout.addLayout(robot_group)

        return group

    def _build_pair_group(self) -> QGroupBox:
        group = QGroupBox("配对点列表")
        layout = QVBoxLayout(group)

        self.lbl_pair_status = QLabel("请先分别加载 PolyWorks 点和机器人点")
        layout.addWidget(self.lbl_pair_status)

        self.tbl_pairs = QTableWidget(0, 8)
        self.tbl_pairs.setHorizontalHeaderLabels(
            ["PAIR", "PW X", "PW Y", "PW Z", "ROBOT X", "ROBOT Y", "ROBOT Z", "残差"]
        )
        self.tbl_pairs.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_pairs.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.tbl_pairs)
        return group

    def _build_action_group(self) -> QGroupBox:
        group = QGroupBox("求解与保存")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("变换名称:"), 0, 0)
        self.edit_transform_name = QLineEdit()
        self.edit_transform_name.setPlaceholderText("例如: PW_to_ROBOT_BASE_01")
        layout.addWidget(self.edit_transform_name, 0, 1)

        layout.addWidget(QLabel("源坐标系:"), 1, 0)
        self.lbl_source_frame = QLabel(PW_FRAME_NAME)
        layout.addWidget(self.lbl_source_frame, 1, 1)

        layout.addWidget(QLabel("目标坐标系:"), 1, 2)
        self.lbl_target_frame = QLabel(ROBOT_BASE_FRAME_NAME)
        layout.addWidget(self.lbl_target_frame, 1, 3)

        self.btn_solve = QPushButton("求解刚体变换")
        self.btn_solve.clicked.connect(self._on_solve)
        layout.addWidget(self.btn_solve, 2, 0, 1, 2)

        self.btn_save_transform = QPushButton("保存并设为当前")
        self.btn_save_transform.setEnabled(False)
        self.btn_save_transform.clicked.connect(self._on_save_transform)
        layout.addWidget(self.btn_save_transform, 2, 2, 1, 2)

        if not HAS_NUMPY:
            warning = QLabel("警告: 未检测到 numpy，当前无法进行标定求解")
            layout.addWidget(warning, 3, 0, 1, 4)

        return group

    def _build_residual_group(self) -> QGroupBox:
        group = QGroupBox("残差显示区")
        layout = QVBoxLayout(group)

        metrics_layout = QGridLayout()
        self.metric_labels = self._create_value_labels(
            metrics_layout,
            ["求解名称", "点对数量", "RMS 误差", "最大残差"],
        )
        layout.addLayout(metrics_layout)

        self.residual_text = QTextEdit()
        self.residual_text.setReadOnly(True)
        self.residual_text.setMaximumHeight(180)
        layout.addWidget(self.residual_text)

        return group

    @staticmethod
    def _create_point_table() -> QTableWidget:
        table = QTableWidget(0, 3)
        table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        return table

    @staticmethod
    def _create_value_labels(layout: QGridLayout, titles: list[str]) -> dict[str, QLabel]:
        labels: dict[str, QLabel] = {}
        for index, title in enumerate(titles):
            row = index // 2
            column = (index % 2) * 2
            layout.addWidget(QLabel(f"{title}: "), row, column)
            value_label = QLabel("--")
            layout.addWidget(value_label, row, column + 1)
            labels[title] = value_label
        return labels

    def _on_load_pw_points(self) -> None:
        # 重新加载数据后，旧的求解结果已经不再可信，所以要清空 latest_result。
        self.pw_points = self._load_points_from_file("选择 PolyWorks 点文件")
        self._fill_point_table(self.tbl_pw_points, self.pw_points)
        self.lbl_pw_count.setText(f"点数: {len(self.pw_points)}")
        self.latest_result = None
        self.btn_save_transform.setEnabled(False)
        self._update_pair_table()

    def _on_load_robot_points(self) -> None:
        # 机器人点的处理逻辑和 PW 点是对称的。
        self.robot_points = self._load_points_from_file("选择机器人点文件")
        self._fill_point_table(self.tbl_robot_points, self.robot_points)
        self.lbl_robot_count.setText(f"点数: {len(self.robot_points)}")
        self.latest_result = None
        self.btn_save_transform.setEnabled(False)
        self._update_pair_table()

    def _on_solve(self) -> None:
        # UI 层先做最基础的前置校验，避免把明显错误的数据送进算法层。
        if not HAS_NUMPY:
            QMessageBox.warning(self, "缺少依赖", "未检测到 numpy，无法进行标定求解")
            return

        if len(self.pw_points) != len(self.robot_points):
            QMessageBox.warning(self, "点数不一致", "PolyWorks 点和机器人点数量必须一致")
            return

        if not self.pw_points:
            QMessageBox.warning(self, "数据不足", "请先加载两组标定点")
            return

        transform_name = self.edit_transform_name.text().strip()
        # 真正的标定算法不在 UI 里，而是交给 controller 统一调度。
        try:
            result = self.controller.solve_point_pair_calibration(
                self.pw_points,
                self.robot_points,
                transform_name,
                source_frame=self.lbl_source_frame.text().strip(),
                target_frame=self.lbl_target_frame.text().strip(),
            )
        except AppError as exc:
            QMessageBox.critical(self, "求解失败", str(exc))
            return

        self.latest_result = result
        # 只有成功求解后，保存按钮才允许点击。
        self.btn_save_transform.setEnabled(True)
        self._fill_result_summary(result)
        self._update_pair_table(result)
        self.edit_transform_name.setText(result.transform.name)

    def _on_save_transform(self) -> None:
        if self.latest_result is None:
            QMessageBox.warning(self, "尚未求解", "请先完成一次标定求解")
            return

        try:
            # 保存时顺手设为 active，后续机器人结果回传就能直接套用它。
            self.controller.save_transform(self.latest_result.transform, set_active=True)
        except AppError as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return

        self.refresh_repository_view()
        QMessageBox.information(self, "保存成功", "标定变换已保存并设为当前激活变换")

    def _on_load_transform(self) -> None:
        transform_name = self.cmb_transforms.currentText().strip()
        if not transform_name:
            QMessageBox.warning(self, "未选择", "请先选择一个已保存变换")
            return

        try:
            # “加载并设为当前”代表告诉 controller:
            # 之后如果机器人请求 ROBOT_BASE 坐标，就优先用这个变换回传。
            transform = self.controller.load_transform(transform_name, set_active=True)
        except AppError as exc:
            QMessageBox.critical(self, "加载失败", str(exc))
            return

        self._fill_active_transform(transform.to_snapshot())
        self.refresh_repository_view()

    def refresh_repository_view(self) -> None:
        """刷新保存库和当前激活变换显示。"""
        try:
            # 页面不直接碰仓库，而是统一通过 controller 获取快照。
            snapshot = self.controller.get_calibration_ui_snapshot()
        except AppError as exc:
            QMessageBox.critical(self, "读取失败", str(exc))
            return

        transforms = snapshot["available_transforms"]
        active_transform = snapshot["active_transform"]

        self.cmb_transforms.blockSignals(True)
        current_name = self.cmb_transforms.currentText().strip()
        self.cmb_transforms.clear()
        self.cmb_transforms.addItems([item["name"] for item in transforms])
        if current_name:
            index = self.cmb_transforms.findText(current_name)
            if index >= 0:
                self.cmb_transforms.setCurrentIndex(index)
        self.cmb_transforms.blockSignals(False)

        self._fill_active_transform(active_transform)

    def _load_points_from_file(self, title: str) -> list[Point3D]:
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            title,
            "",
            "文本文件 (*.txt);;所有文件 (*)",
        )
        if not filepath:
            return []
        return parse_points_file(filepath)

    @staticmethod
    def _fill_point_table(table: QTableWidget, points: list[Point3D]) -> None:
        table.setRowCount(len(points))
        for row, (x, y, z) in enumerate(points):
            for column, value in enumerate((x, y, z)):
                table.setItem(row, column, QTableWidgetItem(f"{value:.6f}"))

    def _update_pair_table(self, result: CalibrationSolveResult | None = None) -> None:
        residual_map = {}
        if result is not None:
            # 按 label 建索引，方便逐行显示每一对点的误差。
            residual_map = {item.label: item for item in result.residuals}

        pair_count = max(len(self.pw_points), len(self.robot_points))
        self.tbl_pairs.setRowCount(pair_count)

        for row in range(pair_count):
            label = f"P{row + 1}"
            # 当前版本的配对规则很简单: 按索引一一对应。
            # 所以文件里第 1 行 PW 点默认配第 1 行 ROBOT 点。
            pw_point = self.pw_points[row] if row < len(self.pw_points) else None
            robot_point = self.robot_points[row] if row < len(self.robot_points) else None

            values = [
                label,
                self._fmt_value(None if pw_point is None else pw_point[0]),
                self._fmt_value(None if pw_point is None else pw_point[1]),
                self._fmt_value(None if pw_point is None else pw_point[2]),
                self._fmt_value(None if robot_point is None else robot_point[0]),
                self._fmt_value(None if robot_point is None else robot_point[1]),
                self._fmt_value(None if robot_point is None else robot_point[2]),
                self._fmt_value(residual_map.get(label).error if label in residual_map else None),
            ]

            for column, text in enumerate(values):
                self.tbl_pairs.setItem(row, column, QTableWidgetItem(text))

        if not self.pw_points and not self.robot_points:
            self.lbl_pair_status.setText("请先分别加载 PolyWorks 点和机器人点")
        elif len(self.pw_points) != len(self.robot_points):
            self.lbl_pair_status.setText(
                f"当前两组点数量不一致：PW={len(self.pw_points)}，ROBOT={len(self.robot_points)}"
            )
        else:
            # 明确告诉用户现在采用的是“按索引自动配对”。
            self.lbl_pair_status.setText(f"当前按索引自动配对，共 {len(self.pw_points)} 对点")

    def _fill_result_summary(self, result: CalibrationSolveResult) -> None:
        transform = result.transform
        self.metric_labels["求解名称"].setText(transform.name)
        self.metric_labels["点对数量"].setText(str(transform.point_count))
        self.metric_labels["RMS 误差"].setText(f"{transform.rms_error:.6f}")
        self.metric_labels["最大残差"].setText(f"{transform.max_error:.6f}")

        matrix_lines = [
            f"源坐标系: {transform.source_frame}",
            f"目标坐标系: {transform.target_frame}",
            "旋转矩阵 R:",
        ]
        # 这里把 R/T 明文显示出来，是为了让数学结果和界面结果能对上。
        for row in transform.rotation:
            matrix_lines.append(
                f"[ {row[0]: .6f}, {row[1]: .6f}, {row[2]: .6f} ]"
            )
        matrix_lines.append(
            "平移向量 T: "
            f"[ {transform.translation[0]: .6f}, {transform.translation[1]: .6f}, {transform.translation[2]: .6f} ]"
        )
        self.residual_text.setPlainText("\n".join(matrix_lines))

    def _fill_active_transform(self, transform_snapshot: dict | None) -> None:
        if transform_snapshot is None:
            self.lbl_active_transform.setText("当前激活变换: --")
            self.active_transform_text.setPlainText("--")
            return

        # active transform 是“当前默认生效”的变换。
        # 机器人联动页请求 ROBOT_BASE 结果时，controller 会优先使用它。
        self.lbl_active_transform.setText(
            "当前激活变换: "
            f"{transform_snapshot['name']} ({transform_snapshot['source_frame']} -> {transform_snapshot['target_frame']})"
        )
        rotation = transform_snapshot["rotation"]
        translation = transform_snapshot["translation"]
        lines = [
            f"创建时间: {transform_snapshot['created_at']}",
            f"点对数量: {transform_snapshot['point_count']}",
            f"RMS: {transform_snapshot['rms_error']:.6f}",
            f"MAX: {transform_snapshot['max_error']:.6f}",
            "R:",
        ]
        for row in rotation:
            lines.append(f"[ {row[0]: .6f}, {row[1]: .6f}, {row[2]: .6f} ]")
        lines.append(
            f"T: [ {translation[0]: .6f}, {translation[1]: .6f}, {translation[2]: .6f} ]"
        )
        self.active_transform_text.setPlainText("\n".join(lines))

    @staticmethod
    def _fmt_value(value: float | None) -> str:
        return "--" if value is None else f"{value:.6f}"

