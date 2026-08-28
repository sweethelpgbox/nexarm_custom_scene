import sys
import os
import traceback

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import Qt
from nexarm_qt.styles import DARK_THEME

def excepthook(exc_type, exc_value, exc_traceback):
    error_msg = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
    print("Uncaught exception:", error_msg)
    try:
        with open("crash_log.txt", "w") as f:
            f.write(error_msg)
    except:
        pass
    if QApplication.instance():
        QMessageBox.critical(None, "Application Crash",
                             f"An unexpected error occurred:\n{exc_value}\n\nSee crash_log.txt for details.")
    sys.exit(1)

sys.excepthook = excepthook

from nexarm_qt.ui.main_window import MainWindow

if __name__ == "__main__":
    # ── 高 DPI 自动缩放 ──
    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    # 关键：让 Qt 使用精确缩放因子（如 1.5），而不是取整到 2.0
    os.environ["QT_SCALE_FACTOR_ROUNDING_POLICY"] = "PassThrough"

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_THEME)

    try:
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        excepthook(type(e), e, e.__traceback__)
