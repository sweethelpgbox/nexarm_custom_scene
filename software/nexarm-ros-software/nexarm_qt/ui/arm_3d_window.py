"""Standalone fullscreen 3D Arm Visualization Window."""

import os
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt

from nexarm_qt.ui.arm_3d_widget import Arm3DWidget
from nexarm_qt.translations import STRINGS


class Arm3DWindow(QMainWindow):
    """Independent window for 3D arm visualization.
    
    Hides instead of closing to preserve loaded STL meshes.
    """

    def __init__(self, comm_manager, lang='zh', parent=None):
        super().__init__(parent)
        self.comm_manager = comm_manager
        self.lang = lang
        # Do NOT use WA_DeleteOnClose — we hide instead of destroy
        self.setWindowTitle(STRINGS[lang].get("title_3d_view", "3D 机械臂视图"))
        self.setStyleSheet("background-color: #1e1e28;")

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)

        # Top bar with close button
        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(10, 5, 10, 0)
        top_bar.addStretch()

        btn_close = QPushButton(STRINGS[lang].get("btn_close_3d", "关闭 3D 视图"))
        self.btn_close = btn_close
        btn_close.setStyleSheet(
            "background-color: #3C3F52; color: white; padding: 6px 15px; "
            "font-size: 10pt; font-weight: bold; border: 1px solid #5A5D6E; "
            "border-radius: 4px; min-height: 32px;")
        btn_close.setCursor(Qt.PointingHandCursor)
        btn_close.clicked.connect(self.hide)  # hide, not close
        top_bar.addWidget(btn_close)
        layout.addLayout(top_bar)

        # 3D Widget
        self.arm_widget = Arm3DWidget(comm_manager, lang)
        layout.addWidget(self.arm_widget)

    def closeEvent(self, event):
        """Override close to hide instead of destroy."""
        event.ignore()
        self.hide()

    def update_language(self, lang):
        self.lang = lang
        self.setWindowTitle(STRINGS[lang].get("title_3d_view", "3D 机械臂视图"))
        if hasattr(self, 'btn_close'):
            self.btn_close.setText(STRINGS[lang].get("btn_close_3d", "关闭 3D 视图"))
        self.arm_widget.update_language(lang)
