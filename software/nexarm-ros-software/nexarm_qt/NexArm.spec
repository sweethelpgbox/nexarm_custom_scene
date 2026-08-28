# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['main.py'],
    pathex=['..'],
    binaries=[],
    datas=[
        ('0.842.png', '.'),
        ('0.842_old.png', '.'),
        ('nexarm_icon.png', '.'),
        ('coord_axes.png', '.'),
        ('ui/nexarm.urdf', 'ui'),
        ('STL', 'STL'),
    ],
    hiddenimports=['PyQt5.sip', 'pyqtgraph', 'pyqtgraph.opengl', 'OpenGL', 'OpenGL.GL', 'OpenGL.GLU',
                   'nexarm_qt', 'nexarm_qt.comm_manager', 'nexarm_qt.constants', 'nexarm_qt.styles',
                   'nexarm_qt.translations', 'nexarm_qt.ui', 'nexarm_qt.ui.main_window',
                   'nexarm_qt.ui.servo_tab', 'nexarm_qt.ui.coord_tab', 'nexarm_qt.ui.peripheral_tab',
                   'nexarm_qt.ui.system_tab', 'nexarm_qt.ui.teach_tab', 'nexarm_qt.ui.log_widget',
                   'nexarm_qt.ui.arm_3d_widget',
                   'nexarm_qt.ui.arm_3d_window',
                   'nexarm_qt.ui.dpad_widget', 'nexarm_qt.ui.urdf_parser'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='NexArm',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['nexarm_icon.ico'],
)
