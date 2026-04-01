"""应用启动入口。

这个文件尽量保持很薄：
只负责启动 Qt 应用并打开主窗口。
"""

import sys

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

from config import APP_FONT_FAMILY, APP_FONT_SIZE
from ui_main_window import MainWindow


def main() -> None:
    """启动桌面程序。

    这里不放业务逻辑，只做 Qt 应用初始化。
    这样主入口会非常清晰，后续排查启动问题也更方便。
    """
    # QApplication 是所有 Qt 桌面程序的入口对象。
    app = QApplication(sys.argv)

    # 统一设置整个程序的界面风格和字体。
    app.setStyle("Fusion")
    app.setFont(QFont(APP_FONT_FAMILY, APP_FONT_SIZE))

    # 创建并显示主窗口。
    window = MainWindow()
    window.show()

    # 进入 Qt 事件循环，直到用户关闭窗口。
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

