from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QSplitter, QGroupBox, QPushButton, QFileDialog, QMessageBox, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5 import uic
import os
from nexarm_qt.translations import STRINGS
from nexarm_qt.styles import S

class LogWidget(QWidget):
    def __init__(self, comm_manager, lang='zh'):
        super().__init__()
        self.comm_manager = comm_manager
        self.lang = lang
        
        # UI Loading Logic
        self.ui_loaded = False
        ui_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'ui_files', 'log_widget.ui')
        if os.path.exists(ui_path):
            try:
                uic.loadUi(ui_path, self)
                self.ui_loaded = True
                self.setup_ui_from_file()
            except Exception as e:
                print(f"Failed to load Log UI: {e}")
        
        if not self.ui_loaded:
            self.setup_ui_manual()
            
        self.connect_signals()

    def update_language(self, lang):
        self.lang = lang
        if hasattr(self, 'lbl_hex_title'): self.lbl_hex_title.setText(STRINGS[lang]["grp_hex_log"])
        if hasattr(self, 'lbl_txt_title'): self.lbl_txt_title.setText(STRINGS[lang]["grp_txt_log"])
        if hasattr(self, 'btn_clear_hex'): self.btn_clear_hex.setText(STRINGS[lang].get("btn_clear_log", "一键清除"))
        if hasattr(self, 'btn_export_hex'): self.btn_export_hex.setText(STRINGS[lang].get("btn_export_log", "一键导出"))
        if hasattr(self, 'btn_clear_txt'): self.btn_clear_txt.setText(STRINGS[lang].get("btn_clear_log", "一键清除"))
        if hasattr(self, 'btn_export_txt'): self.btn_export_txt.setText(STRINGS[lang].get("btn_export_log", "一键导出"))

    def setup_ui_from_file(self):
        # We expect the user to have named widgets: txt_hex, txt_log
        pass

    def setup_ui_manual(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        splitter = QSplitter(Qt.Horizontal)
        
        # Hex Log (using QWidget to bypass QGroupBox top margin)
        self.grp_hex = QWidget()
        layout_hex = QVBoxLayout(self.grp_hex)
        layout_hex.setContentsMargins(0, 0, 0, 0)
        
        toolbar_hex = QHBoxLayout()
        toolbar_hex.setSpacing(10)
        self.lbl_hex_title = QLabel(STRINGS[self.lang]["grp_hex_log"])
        self.lbl_hex_title.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 11pt; border: none;")
        self.btn_clear_hex = QPushButton(STRINGS[self.lang].get("btn_clear_log", "一键清除"))
        self.btn_export_hex = QPushButton(STRINGS[self.lang].get("btn_export_log", "一键导出"))
        self.btn_clear_hex.setFixedSize(S(75), S(24))
        self.btn_export_hex.setFixedSize(S(75), S(24))
        self.btn_clear_hex.setStyleSheet("font-size: 9pt; min-height: 16pt; padding: 2px;")
        self.btn_export_hex.setStyleSheet("font-size: 9pt; min-height: 16pt; padding: 2px;")
        self.btn_clear_hex.clicked.connect(self.clear_hex_log)
        self.btn_export_hex.clicked.connect(self.export_hex_log)
        
        toolbar_hex.addWidget(self.lbl_hex_title)
        toolbar_hex.addStretch()
        toolbar_hex.addWidget(self.btn_clear_hex)
        toolbar_hex.addWidget(self.btn_export_hex)
        layout_hex.addLayout(toolbar_hex)
        
        self.txt_hex = QTextEdit()
        self.txt_hex.setFont(QFont("Consolas", 9))
        self.txt_hex.setReadOnly(True)
        layout_hex.addWidget(self.txt_hex)
        
        # Text Log
        self.grp_txt = QWidget()
        layout_txt = QVBoxLayout(self.grp_txt)
        layout_txt.setContentsMargins(0, 0, 0, 0)
        
        toolbar_txt = QHBoxLayout()
        toolbar_txt.setSpacing(10)
        self.lbl_txt_title = QLabel(STRINGS[self.lang]["grp_txt_log"])
        self.lbl_txt_title.setStyleSheet("color: #FA8F01; font-weight: bold; font-size: 11pt; border: none;")
        self.btn_clear_txt = QPushButton(STRINGS[self.lang].get("btn_clear_log", "一键清除"))
        self.btn_export_txt = QPushButton(STRINGS[self.lang].get("btn_export_log", "一键导出"))
        self.btn_clear_txt.setFixedSize(S(75), S(24))
        self.btn_export_txt.setFixedSize(S(75), S(24))
        self.btn_clear_txt.setStyleSheet("font-size: 9pt; min-height: 16pt; padding: 2px;")
        self.btn_export_txt.setStyleSheet("font-size: 9pt; min-height: 16pt; padding: 2px;")
        self.btn_clear_txt.clicked.connect(self.clear_text_log)
        self.btn_export_txt.clicked.connect(self.export_text_log)
        
        toolbar_txt.addWidget(self.lbl_txt_title)
        toolbar_txt.addStretch()
        toolbar_txt.addWidget(self.btn_clear_txt)
        toolbar_txt.addWidget(self.btn_export_txt)
        layout_txt.addLayout(toolbar_txt)
        
        self.txt_log = QTextEdit()
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setReadOnly(True)
        layout_txt.addWidget(self.txt_log)

        splitter.addWidget(self.grp_hex)
        splitter.addWidget(self.grp_txt)
        splitter.setHandleWidth(20)  # wider gap between HEX and TXT panels
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)
        self.setMinimumHeight(100)

    def connect_signals(self):
        self.comm_manager.hex_log_received.connect(self.append_hex_log)
        self.comm_manager.log_message_received.connect(self.append_text_log)

    MAX_LOG_LINES = 500  # 最多保留 500 行

    def append_hex_log(self, prefix, data):
        if hasattr(self, 'txt_hex'):
            hex_str = " ".join([f"{b:02X}" for b in data])
            self.txt_hex.append(f"[{prefix}] {hex_str}")
            # 限制行数，防止内存爆炸
            doc = self.txt_hex.document()
            if doc.blockCount() > self.MAX_LOG_LINES:
                cursor = self.txt_hex.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.blockCount() - self.MAX_LOG_LINES)
                cursor.removeSelectedText()
                cursor.deleteChar()  # 删除多余换行

    def append_text_log(self, text):
        if hasattr(self, 'txt_log'):
            self.txt_log.append(text)
            doc = self.txt_log.document()
            if doc.blockCount() > self.MAX_LOG_LINES:
                cursor = self.txt_log.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.blockCount() - self.MAX_LOG_LINES)
                cursor.removeSelectedText()
                cursor.deleteChar()

    def clear_hex_log(self):
        if hasattr(self, 'txt_hex'):
            self.txt_hex.clear()

    def export_hex_log(self):
        if not hasattr(self, 'txt_hex'):
            return
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Export Hex Log", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            try:
                with open(file_name, 'w', encoding='utf-8') as f:
                    f.write(self.txt_hex.toPlainText())
                QMessageBox.information(self, STRINGS[self.lang].get("msg_save_success", "Saved"), f"HEX log saved to {file_name}")
            except Exception as e:
                QMessageBox.critical(self, STRINGS[self.lang].get("msg_error", "Error"), f"Failed to save: {str(e)}")

    def clear_text_log(self):
        if hasattr(self, 'txt_log'):
            self.txt_log.clear()

    def export_text_log(self):
        if not hasattr(self, 'txt_log'):
            return
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Export Text Log", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            try:
                with open(file_name, 'w', encoding='utf-8') as f:
                    f.write(self.txt_log.toPlainText())
                QMessageBox.information(self, STRINGS[self.lang].get("msg_save_success", "Saved"), f"Text log saved to {file_name}")
            except Exception as e:
                QMessageBox.critical(self, STRINGS[self.lang].get("msg_error", "Error"), f"Failed to save: {str(e)}")
