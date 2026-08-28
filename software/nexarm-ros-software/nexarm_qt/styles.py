DARK_THEME = """
/*
主背景: #1E1F31
侧边栏背景: #1E1F31
面板背景: #343645
主要色调: #FA8F01
次要文本: #B0BEC5
设计基准: 1600x900
*/

QMainWindow, QWidget {
    background-color: #1E1F31;
    color: #FFFFFF;
    font-family: "Microsoft YaHei", "PingFang SC", "Segoe UI", sans-serif;
    font-size: 10pt;
}

QLabel {
    color: #FFFFFF;
    font-size: 10pt;
    background-color: transparent;
}

.title-text {
    font-size: 13pt;
    font-weight: bold;
    color: #FFFFFF;
}

.body-text {
    font-size: 10pt;
    font-weight: normal;
    color: #FFFFFF;
}

.aux-text {
    font-size: 9pt;
    font-weight: 500;
    color: #B0BEC5;
}

.secondary-text {
    color: #B0BEC5;
    font-size: 9pt;
}

.disabled-text {
    color: #78909C;
    font-size: 9pt;
}

QFrame#Sidebar {
    background-color: #1E1F31;
    border-right: 1px solid rgba(255, 255, 255, 0.05);
}

QListWidget#NavList {
    background-color: transparent;
    border: none;
    outline: none;
    margin-top: 10px;
}

QListWidget#NavList::item {
    padding: 12px 20px;
    color: #B0BEC5;
    border-left: 4px solid transparent;
    font-size: 11pt;
}

QListWidget#NavList::item:selected {
    background-color: #343645;
    color: #FFFFFF;
    border-left: 4px solid #FA8F01;
}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {
    background-color: #343645;
    border: 1px solid #4A4D5E;
    border-radius: 4px;
    color: #FFFFFF;
    padding: 4px 8px;
    min-height: 20pt;
    font-size: 10pt;
    selection-background-color: #FA8F01;
    selection-color: #FFFFFF;
}

QComboBox {
    margin-top: 3px;
    padding-top: 2px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 2px solid #FA8F01;
    border-style: solid;
    background-color: #3A3C4D;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid #4A4D5E;
    background-color: #3C3F52;
    border-top-right-radius: 4px;
    margin: 1px;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border-left: 1px solid #4A4D5E;
    background-color: #3C3F52;
    border-bottom-right-radius: 4px;
    margin: 1px;
}

QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
    width: 0; height: 0;
    border-left: 3px solid transparent;
    border-right: 3px solid transparent;
    border-bottom: 4px solid #B0BEC5;
}

QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
    width: 0; height: 0;
    border-left: 3px solid transparent;
    border-right: 3px solid transparent;
    border-top: 4px solid #B0BEC5;
}

QComboBox {
    padding-right: 20px;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid #4A4D5E;
    background-color: transparent;
}

QComboBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid #B0BEC5;
    width: 0; height: 0;
}

QComboBox QAbstractItemView {
    background-color: #252735 !important;
    color: #FFFFFF !important;
    border: 1px solid #4A4D5E;
    selection-background-color: #4A4D5E !important;
    selection-color: #FFFFFF !important;
    outline: none;
}

QListView {
    background-color: #252735;
    color: #FFFFFF;
    border: none;
    selection-background-color: #4A4D5E;
}

QListView::item {
    padding: 8px;
    background-color: #252735;
    color: #FFFFFF;
}

QListView::item:selected {
    background-color: #4A4D5E;
    color: #FFFFFF;
}

QPushButton {
    background-color: #3C3F52;
    border: 1px solid #5A5D6E;
    border-radius: 4px;
    color: white;
    padding: 6px 14px;
    font-family: "Microsoft YaHei";
    font-size: 10pt;
    font-weight: bold;
    min-height: 24pt;
}

QPushButton:hover {
    background-color: #4A4D5E;
    border-color: #FA8F01;
}

QPushButton:pressed {
    background-color: #2D2F3F;
    border-color: #E68200;
}

QPushButton[type="primary"], #btn_home, #btn_read, #btn_send, #btn_get {
    background-color: #3C3F52;
    border: 1px solid #5A5D6E;
    color: #FFFFFF;
}

QPushButton[type="primary"]:hover, #btn_home:hover, #btn_read:hover, #btn_send:hover, #btn_get:hover {
    background-color: #4A4D5E;
    border-color: #FA8F01;
}

QPushButton[type="dir"] {
    background-color: #3C3F52;
    font-weight: bold;
    min-width: 90px;
    min-height: 50px;
    border-radius: 10px;
    border: 1px solid #5A5D6E;
}

QPushButton[type="dir"]:hover {
    background-color: #FA8F01;
    border: 1px solid rgba(255,255,255,0.2);
}

QGroupBox {
    background-color: #1E1F31;
    border: none;
    border-top: 1px solid #343645;
    margin-top: 25px;
    padding-top: 12px;
    font-size: 11pt;
    font-weight: bold;
    color: #FFFFFF;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 0px;
    padding: 0 5px;
    color: #FA8F01;
}

#ServoControlBox {
    background-color: transparent;
    border: none;
    border-radius: 0px;
}

.servo-spin {
    background-color: transparent !important;
    border: none !important;
    color: #FFFFFF;
    font-size: 10pt;
    font-weight: bold;
    padding: 2px;
}

.servo-label {
    background-color: transparent !important;
    border: none !important;
    color: #B0BEC5;
    font-size: 9pt;
    font-weight: bold;
}

QSlider {
    border: none;
    outline: none;
}

QSlider::groove:horizontal {
    height: 6px;
    background: rgba(255, 255, 255, 0.1);
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background: #FFFFFF;
    border: 2px solid rgba(255, 255, 255, 0.8);
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 8px;
}

QSlider::handle:horizontal:hover {
    background: #FFFFFF;
    border: 2px solid #FFFFFF;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 9px;
}

QSlider::sub-page:horizontal {
    background: #57596E;
    border-radius: 3px;
}
"""

def dpi_scale():
    return 1.0

def S(px):
    return int(px)
