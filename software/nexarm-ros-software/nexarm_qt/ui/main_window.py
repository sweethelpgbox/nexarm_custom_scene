from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFrame, QListWidget, QStackedWidget
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSettings, QTimer, QSize
import os
from nexarm_qt.comm_manager import CommManager
from nexarm_qt.constants import CMD_GET_CUR_COORDS
from nexarm_qt.ui.log_widget import LogWidget
from nexarm_qt.ui.servo_tab import ServoTab
from nexarm_qt.ui.coord_tab import CoordTab

from nexarm_qt.ui.peripheral_tab import PeripheralTab
from nexarm_qt.ui.system_tab import SystemTab
from nexarm_qt.ui.teach_tab import TeachTab
from nexarm_qt.translations import STRINGS

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.lang = 'zh'
        self.settings = QSettings("NexArm", "NexArmQt")
        self.setWindowTitle(STRINGS[self.lang]["window_title"])
        self.setMinimumSize(800, 600)
        # 窗口大小 = 屏幕可用区域的 90%，自适应任何分辨率和 DPI
        from PyQt5.QtWidgets import QDesktopWidget
        avail = QDesktopWidget().availableGeometry()
        w = int(avail.width() * 0.90)
        h = int(avail.height() * 0.90)
        self.resize(w, h)
        import sys
        if hasattr(sys, '_MEIPASS'):
            icon_candidates = [
                os.path.join(sys._MEIPASS, "nexarm_icon.ico"),
                os.path.join(sys._MEIPASS, "nexarm_icon.png"),
            ]
        else:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            icon_candidates = [
                os.path.join(base_dir, "nexarm_icon.ico"),
                os.path.join(base_dir, "nexarm_icon.png"),
            ]
        icon_path = next((p for p in icon_candidates if os.path.exists(p)), "")
        if icon_path:
            self.setWindowIcon(QIcon(icon_path))

        # DPI 缩放因子：以 96 DPI 为基准
        self._dpi_scale = self.logicalDpiX() / 96.0
        
        self.comm_manager = CommManager()
        self.setup_ui()
        self.connect_signals()
        
        self.refresh_ports()
        QTimer.singleShot(0, self.connect_internal)

    def _s(self, px):
        return int(px)

    def _navigation_labels(self):
        return [
            STRINGS[self.lang]["tab_servo"],
            STRINGS[self.lang]["tab_coord"],
            STRINGS[self.lang]["tab_peripheral"],
            STRINGS[self.lang]["tab_system"],
            STRINGS[self.lang].get("tab_teach", "示教编辑"),
        ]

    def update_language(self):
        self.setWindowTitle(STRINGS[self.lang]["window_title"])
        for row, label in enumerate(self._navigation_labels()):
            self.nav_list.item(row).setText(label)
        self.header_lbl.setText(self.nav_list.currentItem().text())
        
        self.update_connection_status(self.comm_manager.is_connected, "")
        
        self.lbl_reception_owner.setText(STRINGS[self.lang]["reception_owner"])
        self.btn_lang.setText("English" if self.lang == 'zh' else "中文")
        
        self.tab_servo.update_language(self.lang)
        self.tab_coord.update_language(self.lang)
        self.tab_peripheral.update_language(self.lang)
        self.tab_system.update_language(self.lang)
        self.tab_teach.update_language(self.lang)
        self.log_widget.update_language(self.lang)

    def toggle_language(self):
        self.lang = 'en' if self.lang == 'zh' else 'zh'
        self.btn_lang.setText("English" if self.lang == 'zh' else "中文")
        self.update_language()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # --- LEFT SIDEBAR ---
        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(self._s(248))
        sidebar_layout = QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.setSpacing(0)

        # Logo & Title Section (Increased Top Margin)
        title_container = QWidget()
        title_layout = QVBoxLayout(title_container)
        title_layout.setContentsMargins(0, 6, 0, 2)

        logo_label = QLabel()
        import sys, os
        if hasattr(sys, '_MEIPASS'):
            logo_candidates = [
                os.path.join(sys._MEIPASS, "nexarm_icon.ico"),
                os.path.join(sys._MEIPASS, "nexarm_icon.png"),
            ]
        else:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            logo_candidates = [
                os.path.join(base_dir, "nexarm_icon.ico"),
                os.path.join(base_dir, "nexarm_icon.png"),
            ]
        icon_path2 = next((p for p in logo_candidates if os.path.exists(p)), "")
        logo_pix = QIcon(icon_path2).pixmap(48, 48) if icon_path2 else QIcon().pixmap(48, 48)
        logo_label.setPixmap(logo_pix)
        logo_label.setAlignment(Qt.AlignCenter)
        logo_label.setFixedHeight(self._s(52))
        title_layout.addWidget(logo_label)
        sidebar_layout.addWidget(title_container)

        # Navigation List
        self.nav_list = QListWidget()
        self.nav_list.setObjectName("NavList")
        self.nav_list.addItems(self._navigation_labels())
        self.nav_list.setSpacing(10)
        self.nav_list.setCurrentRow(0)
        self.nav_list.currentRowChanged.connect(self.on_nav_changed)
        self.nav_list.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.nav_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        for row in range(self.nav_list.count()):
            self.nav_list.item(row).setSizeHint(QSize(0, self._s(48)))
        min_nav_height = self.nav_list.count() * self._s(48) + max(0, self.nav_list.count() - 1) * self._s(10) + self._s(24)
        self.nav_list.setMinimumHeight(min_nav_height)
        sidebar_layout.addWidget(self.nav_list, 1)

        # Bottom footer: passive status only, not a control panel.
        self.sidebar_footer = QFrame()
        self.sidebar_footer.setObjectName("SidebarFooter")
        self.sidebar_footer.setStyleSheet("""
            QFrame#SidebarFooter {
                background-color: #1A1B2A;
                border-top: 1px solid rgba(255, 255, 255, 0.06);
            }
        """)
        footer_layout = QVBoxLayout(self.sidebar_footer)
        footer_layout.setContentsMargins(20, 12, 20, 12)
        footer_layout.setSpacing(10)

        status_row = QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(8)
        self.lbl_reception_owner = QLabel(STRINGS[self.lang]["reception_owner"])
        self.lbl_reception_owner.setStyleSheet("color: #90A4AE; font-size: 9pt; font-weight: 700;")
        status_row.addWidget(self.lbl_reception_owner)
        self.status_dot = QLabel()
        self.status_dot.setFixedSize(self._s(9), self._s(9))
        self.status_dot.setStyleSheet("background-color: #EF5350; border-radius: 4px;")
        status_row.addWidget(self.status_dot)
        self.lbl_status = QLabel(STRINGS[self.lang]["reception_idle"])
        self.lbl_status.setProperty("class", "status-error")
        self.lbl_status.setStyleSheet("QLabel { font-weight: 600; color: #90A4AE; font-size: 9pt; }")
        status_row.addWidget(self.lbl_status)
        status_row.addStretch()
        footer_layout.addLayout(status_row)

        self.available_ports = []

        # Language Toggle
        lang_row = QHBoxLayout()
        lang_row.setContentsMargins(0, 0, 0, 0)
        self.btn_lang = QPushButton("English" if self.lang == 'zh' else "中文")
        self.btn_lang.setFixedHeight(self._s(35))
        self.btn_lang.setStyleSheet("""
            QPushButton {
                background-color: #343645;
                color: #FFFFFF;
                border: 1px solid rgba(255,255,255,0.1);
                border-radius: 4px;
                font-size: 9pt;
                padding: 0 10px;
            }
            QPushButton:hover {
                border-color: #FA8F01;
            }
        """)
        self.btn_lang.clicked.connect(self.toggle_language)
        lang_row.addWidget(self.btn_lang)
        footer_layout.addLayout(lang_row)
        sidebar_layout.addWidget(self.sidebar_footer)

        main_layout.addWidget(self.sidebar)

        # --- MAIN CONTENT AREA ---
        content_container = QWidget()
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        self.header_lbl = QLabel(STRINGS[self.lang]["tab_servo"])
        self.header_lbl.setProperty("class", "title-text")
        self.header_lbl.setStyleSheet("QLabel { padding: 12px 15px; background-color: #1E1F31; font-weight: bold; font-size: 14pt; }")
        content_layout.addWidget(self.header_lbl)

        self.stack = QStackedWidget()
        self.tab_servo = ServoTab(self.comm_manager, self.lang)
        self.tab_coord = CoordTab(self.comm_manager, self.lang)
        self.tab_peripheral = PeripheralTab(self.comm_manager, self.lang)
        self.tab_system = SystemTab(self.comm_manager, self.lang)
        self.tab_teach = TeachTab(self.comm_manager, self.lang)
        
        self.stack.addWidget(self.tab_servo)
        self.stack.addWidget(self.tab_coord)
        self.stack.addWidget(self.tab_peripheral)
        self.stack.addWidget(self.tab_system)
        self.stack.addWidget(self.tab_teach)
        content_layout.addWidget(self.stack, 1)

        self.log_widget = LogWidget(self.comm_manager, self.lang)
        self.log_widget.setMaximumHeight(self._s(150))
        self.content_layout = content_layout
        
        # Initial state: we start on tab 0, so put it in tab_servo
        self.tab_servo.left_pane_layout.addWidget(self.log_widget)

        main_layout.addWidget(content_container, 1)

    def on_nav_changed(self, index):
        self.stack.setCurrentIndex(index)
        text = self.nav_list.item(index).text()
        self.header_lbl.setText(text)

        # 切换到动作组页面时，同步当前舵机位置到滑杆
        if index == 0 and hasattr(self, 'tab_servo'):
            self.tab_servo.sync_current_servo_positions()

        # 切换到坐标页面时，自动获取当前坐标
        if index == 1 and hasattr(self, 'tab_coord'):
            self.comm_manager.send_sys(CMD_GET_CUR_COORDS)

        if hasattr(self, 'log_widget'):
            if index == 0:
                if hasattr(self.tab_servo, 'left_pane_layout'):
                    self.log_widget.setMinimumHeight(self._s(100))
                    self.log_widget.setMaximumHeight(self._s(150))
                    self.log_widget.show()
                    self.tab_servo.left_pane_layout.addWidget(self.log_widget)
            else:
                if hasattr(self, 'content_layout'):
                    self.log_widget.setMinimumHeight(self._s(100))
                    self.log_widget.setMaximumHeight(self._s(150))
                    self.log_widget.show()
                    self.content_layout.addWidget(self.log_widget)

    def connect_signals(self):
        self.comm_manager.connection_status_changed.connect(self.update_connection_status)

    def refresh_ports(self):
        self.available_ports = self.comm_manager.get_available_ports()

    def connect_internal(self):
        if hasattr(self, "log_widget"):
            self.log_widget.append_text_log(STRINGS[self.lang]["ros_reception_log"])
        if self.connect_ros_bridge():
            return
        self.connect_serial_direct()

    def connect_ros_bridge(self):
        try:
            self.comm_manager.connect_ros()
            return bool(self.comm_manager.is_connected and self.comm_manager.connection_type == 'ros')
        except Exception as e:
            self.update_connection_status(False, str(e))
            return False

    def connect_serial_direct(self):
        self.refresh_ports()
        port = self.available_ports[0] if self.available_ports else ""
        if not port:
            self.update_connection_status(False, "No serial port found")
            return False
        try:
            self.comm_manager.connect(port, 1000000)
            return bool(self.comm_manager.is_connected and self.comm_manager.connection_type == 'serial')
        except Exception as e:
            self.update_connection_status(False, str(e))
            return False

    def update_connection_status(self, is_connected, message):
        if is_connected:
            self.lbl_status.setText(STRINGS[self.lang]["reception_takeover"])
            self.status_dot.setStyleSheet("background-color: #43A047; border-radius: 4px;")
            self.lbl_status.setStyleSheet("QLabel { font-weight: bold; color: #4CAF50; font-size: 9pt; }")
        else:
            self.lbl_status.setText(STRINGS[self.lang]["reception_idle"])
            self.status_dot.setStyleSheet("background-color: #EF5350; border-radius: 4px;")
            self.lbl_status.setStyleSheet("QLabel { font-weight: 600; color: #90A4AE; font-size: 9pt; }")
        
        self.lbl_status.style().unpolish(self.lbl_status)
        self.lbl_status.style().polish(self.lbl_status)
            
        if message and message != "Disconnected":
            if is_connected:
                self.log_widget.append_text_log(f"Info: {message}")
            else:
                self.log_widget.append_text_log(f"Connection Error: {message}")

    def closeEvent(self, event):
        if self.comm_manager.is_connected:
            self.comm_manager.disconnect()
        event.accept()
