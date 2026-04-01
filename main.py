"""应用启动入口。

这个文件尽量保持很薄：
只负责启动 Qt 应用并打开主窗口。
"""

import sys

from app_logger import get_logger, install_global_excepthook
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

from config import APP_FONT_FAMILY, APP_FONT_SIZE
from ui_main_window import MainWindow

# 新手可以把这个文件看成“程序起点”：
# Python 启动后，会先来到这里，
# 再由这里创建界面窗口并进入 Qt 的事件循环。


def main() -> None:
    """启动桌面程序。

    这里不放业务逻辑，只做 Qt 应用初始化。
    这样主入口会非常清晰，后续排查启动问题也更方便。
    """
    logger = get_logger("main")
    install_global_excepthook(logger)

    # QApplication 是所有 Qt 桌面程序的入口对象。
    # 没有它，窗口、按钮、输入框这些控件都没有运行环境。
    app = QApplication(sys.argv)

    # 统一设置整个程序的界面风格和字体。
    app.setStyle("Fusion")
    app.setFont(QFont(APP_FONT_FAMILY, APP_FONT_SIZE))

    # 创建并显示主窗口。
    # 后面你看到的 3 个功能页，都是从 MainWindow 展开的。
    window = MainWindow()
    window.show()

    # 进入 Qt 事件循环，直到用户关闭窗口。
    # 所有按钮点击、界面刷新、日志更新，都是在这里持续处理的。
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

