# Protocol Constants — 与 Global.h 保持一致
CMD_SYS_HEAD = 0xFF

# ── 系统 / 基础指令 ──────────────────────────────────────
CMD_FIRMWARE_VERSION_CHECK  = 1
CMD_CHECK_BAT_LEVEL_CHECK   = 2
CMD_ACTION_GROUP_RUN        = 3
CMD_ACTION_GROUP_STOP       = 4
CMD_ACTION_GROUP_DOWNLOAD   = 5
CMD_FKINE_RESULT_GET        = 6
CMD_IKINE_RESULT_GET        = 7
CMD_COORDINATE_SET          = 8
CMD_BUZZER_SET              = 9
CMD_OLED_SET                = 10
CMD_GET_CUR_COORDS          = 11
CMD_OLED_ICON               = 12
CMD_SET_SINGLE_MOTOR        = 13
CMD_STOP_ALL_MOTOR          = 14
CMD_SET_MOTOR_SPEED         = 15

# ── 外设 ─────────────────────────────────────────────────
CMD_CONVEYOR_SET            = 16
CMD_STEPPER_RESET           = 17
CMD_STEPPER_DIV             = 18
CMD_STEPPER_RUN             = 19

CMD_BUTTON_EVENT            = 22
CMD_ACTION_GROUP_ERASE      = 23

# ── ESP-NOW / 通信 ───────────────────────────────────────
CMD_SET_ESPNOW_CHANNEL      = 30
CMD_SET_GLOBAL_ACC          = 31
CMD_ESPNOW_SYNC_CTRL        = 33

# ── 底盘 ─────────────────────────────────────────────────
CMD_MECANUM_CONTROL          = 34
CMD_TANK_CONTROL             = 35
CMD_SET_PEER_MAC             = 36

# ── AI 玩法 ──────────────────────────────────────────────
CMD_COLOR_TRACK              = 40
CMD_FACE_TRACK               = 41
CMD_SELF_LEARN_TRACK         = 42
CMD_APRILTAG_TRACK           = 43
CMD_APRILTAG_GRAB            = 44
CMD_APRILTAG_SET_OFFSET      = 45
CMD_COLOR_GRAB               = 46
CMD_LLM_CONTROL              = 47
CMD_GARBAGE_GRAB             = 48
CMD_CALIBRATION              = 49

# ── 机械臂运动 ───────────────────────────────────────────
CMD_ARM_MOVE_INC             = 50
CMD_ARM_SERVO_SINGLE         = 51

# ── 舵机配置 ─────────────────────────────────────────────
CMD_SET_SERVO_ID             = 52
CMD_SET_SERVO_MODE           = 53
CMD_ARM_RESET                = 54
CMD_READ_ALL_SERVOS          = 55
CMD_SET_MOVE_ACC             = 56
CMD_SET_POS_OFFSET           = 57
CMD_GET_POS_OFFSET           = 58
CMD_SET_PID_PARAM            = 59
CMD_GET_PID_PARAM            = 60
CMD_SET_TORQUE               = 61
CMD_SET_BT_MODE              = 62
CMD_SET_KINEMATICS_PARAM     = 63
CMD_GET_KINEMATICS_PARAM     = 64
CMD_GET_REAL_JOINT_ANGLES    = 65
CMD_GET_REAL_TCP_POSE        = 66

CMD_LEROBOT_MODE             = 68
CMD_PC_SYNC_TEACH            = 69

# ── 舵机高级 (CMD 70~80) ─────────────────────────────────
CMD_SYNC_WRITE_SERVOS        = 70
CMD_SERVO_READ_OVERLOAD      = 71
CMD_SERVO_WRITE_OVERLOAD     = 72
CMD_SERVO_READ_BAUD          = 73
CMD_SERVO_WRITE_BAUD         = 74
CMD_SERVO_READ_MAX_TORQUE    = 75
CMD_SERVO_WRITE_MAX_TORQUE   = 76
CMD_SERVO_READ_ANGLE_LIMIT   = 77
CMD_SERVO_WRITE_ANGLE_LIMIT  = 78
CMD_SET_COORD_LIMITS         = 79
CMD_GET_COORD_LIMITS         = 80
CMD_SERVO_CALI_POS           = 88
CMD_SET_INTERP_MODE          = 89
CMD_FIRMWARE_UPDATE          = 90  # App: 进入Bootloader升级模式
CMD_FW_START                 = 91  # Bootloader: 擦除App区
CMD_FW_DATA                  = 92  # Bootloader: 写入数据包
CMD_FW_END                   = 93  # Bootloader: 完成升级，重启
CMD_FW_QUERY                 = 94  # Bootloader: 查询状态

# ── 示教编辑 (CMD 120~135) ───────────────────────────────
CMD_ACTION_EDIT_ENTER        = 120
CMD_ACTION_EDIT_EXIT         = 121
CMD_ACTION_EDIT_START        = 122
CMD_ACTION_EDIT_STOP         = 123
CMD_ACTION_EDIT_PLAY         = 124
CMD_ACTION_EDIT_PLAY_STOP    = 125
CMD_ACTION_EDIT_CLEAR        = 126
CMD_ACTION_EDIT_QUERY        = 127
CMD_SYNC_TEACH_ENTER         = 128
CMD_SYNC_TEACH_EXIT          = 129
CMD_SYNC_TEACH_REC_START     = 130
CMD_SYNC_TEACH_REC_STOP      = 131
CMD_SYNC_TEACH_PLAY          = 132
CMD_SYNC_TEACH_PLAY_STOP     = 133
CMD_SYNC_TEACH_CLEAR         = 134
CMD_SYNC_TEACH_QUERY         = 135

CMD_GESTURE_TRACK            = 81
CMD_MOVE_INC                 = 82
CMD_SET_PS3_MAC              = 83
CMD_FACTORY_RESET            = 84
CMD_SET_CHASSIS_CONFIG       = 85
CMD_GET_CHASSIS_CONFIG       = 86
CMD_SCAN_WIFI_CHANNELS       = 87

# ── 旧别名兼容（供已有代码引用） ─────────────────────────
CMD_MOTOR_STOP       = CMD_STOP_ALL_MOTOR
CMD_MECANUM_RUN      = CMD_MECANUM_CONTROL
CMD_TANK_RUN         = CMD_TANK_CONTROL
CMD_ESPNOW_SET_CHANNEL = CMD_SET_ESPNOW_CHANNEL
CMD_ESPNOW_SET_ACC   = CMD_SET_GLOBAL_ACC
CMD_ESPNOW_SET_ROLE  = CMD_ESPNOW_SYNC_CTRL  # 旧代码兼容，实际映射到 33
CMD_ESPNOW_ENABLE    = CMD_ESPNOW_SYNC_CTRL  # 开启/关闭 ESP-NOW
CMD_ESPNOW_SYNC      = CMD_ESPNOW_SYNC_CTRL  # 同步控制
CMD_ESPNOW_SET_MAC   = CMD_SET_PEER_MAC       # 设置目标 MAC
CMD_ESPNOW_SCAN      = CMD_SCAN_WIFI_CHANNELS # 扫描
CMD_ESPNOW_SCAN_CHANNEL = CMD_SCAN_WIFI_CHANNELS
CMD_STEPPER_SET_DIV  = CMD_STEPPER_DIV
CMD_PS3_SET_MAC      = CMD_SET_PS3_MAC
CMD_SYNC_MOVE_SERVOS = CMD_SYNC_WRITE_SERVOS
CMD_READ_OVERLOAD    = CMD_SERVO_READ_OVERLOAD
CMD_SET_OVERLOAD     = CMD_SERVO_WRITE_OVERLOAD
CMD_READ_BAUD        = CMD_SERVO_READ_BAUD
CMD_SET_BAUD         = CMD_SERVO_WRITE_BAUD
CMD_READ_MAX_TORQUE  = CMD_SERVO_READ_MAX_TORQUE
CMD_SET_MAX_TORQUE   = CMD_SERVO_WRITE_MAX_TORQUE
CMD_READ_ANGLE_LIMIT = CMD_SERVO_READ_ANGLE_LIMIT
CMD_SET_ANGLE_LIMIT  = CMD_SERVO_WRITE_ANGLE_LIMIT

# ── 板载 ID ──────────────────────────────────────────────
AT32_SYS_ID = 0x5A

# ── 舵机总线协议 ─────────────────────────────────────────
SERVO_CMD_READ  = 2
SERVO_CMD_WRITE = 3

# Servo Registers
SERVO_REG_TORQUE      = 40
SERVO_REG_ACC         = 41
SERVO_REG_GOAL_POS    = 42
SERVO_REG_PRESENT_POS = 56
