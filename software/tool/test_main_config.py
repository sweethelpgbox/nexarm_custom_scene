import os
import sys
from pathlib import Path

import yaml


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool import main  # noqa: E402


def test_parse_exports_supports_current_typerc_names():
    text = """
export CAMERA_TYPE=aurora
export MACHINE_TYPE=NexArm
export ASR_LANGUAGE=Chinese
export MICROPHONE_TYPE=WonderEchoPro
export CALIBRATION_CURRENT_SCENE=scene_3
"""
    exports = main.parse_exports(text)

    assert main.export_value(exports, "CAMERA_TYPE", "DEPTH_CAMERA_TYPE", default="None") == "aurora"
    assert main.export_value(exports, "MICROPHONE_TYPE", "MIC_TYPE", default="xf") == "WonderEchoPro"
    assert main.export_value(exports, "LIDAR_TYPE", default="None") == "None"
    assert main.export_value(exports, "CALIBRATION_CURRENT_SCENE", default="scene_0") == "scene_3"


def test_replace_or_append_export_preserves_existing_lines():
    text = "export CAMERA_TYPE=aurora\nexport MACHINE_TYPE=NexArm\n"

    updated = main.replace_or_append_export(text, "CALIBRATION_CURRENT_SCENE", "scene_2")
    updated = main.replace_or_append_export(updated, "CAMERA_TYPE", "usb_cam")

    assert "export CAMERA_TYPE=usb_cam\n" in updated
    assert "export MACHINE_TYPE=NexArm\n" in updated
    assert "export CALIBRATION_CURRENT_SCENE=scene_2\n" in updated


def test_write_scene_yaml_sets_current_scene(tmp_path):
    scene_path = tmp_path / "calibration_scene.yaml"
    scene_path.write_text(
        yaml.safe_dump({"current_scene": "scene_0", "scenes": {"scene_0": {}, "scene_4": {}}}),
        encoding="utf-8",
    )

    main.write_scene_yaml_current_scene(str(scene_path), "scene_4")

    data = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    assert data["current_scene"] == "scene_4"
    assert "scene_0" in data["scenes"]


def test_network_labels_do_not_crash_when_psutil_denied(monkeypatch):
    window = main.MainWindow.__new__(main.MainWindow)
    window.chinese = True

    def raise_permission_error():
        raise PermissionError("denied")

    monkeypatch.setattr(main.psutil, "net_if_addrs", raise_permission_error)

    assert window.get_wlan() == "未检测到无线网卡"
    assert window.get_eth() == "未连接有线网络"


def test_typerc_path_prefers_ros2_workspace_file(tmp_path, monkeypatch):
    home = tmp_path / "home" / "ubuntu"
    tool_path = home / "gai" / "software" / "tool"
    tool_path.mkdir(parents=True)
    gai_typerc = home / "gai" / ".typerc"
    ros2_typerc = home / "ros2_ws" / ".typerc"
    gai_typerc.parent.mkdir(parents=True, exist_ok=True)
    ros2_typerc.parent.mkdir(parents=True, exist_ok=True)
    gai_typerc.write_text("export CALIBRATION_CURRENT_SCENE=scene_4\n", encoding="utf-8")
    ros2_typerc.write_text("export CALIBRATION_CURRENT_SCENE=scene_5\n", encoding="utf-8")

    monkeypatch.setattr(main, "ROS2_WS_TYPERC_PATH", str(ros2_typerc))

    assert main.typerc_path_for_tool(str(tool_path)) == str(ros2_typerc)


def test_write_text_file_uses_local_ros_path_when_docker_is_not_running(tmp_path, monkeypatch):
    ros2_dir = tmp_path / "home" / "ubuntu" / "ros2_ws"
    typerc = ros2_dir / ".typerc"

    monkeypatch.setattr(main, "DOCKER_TARGET_PREFIXES", (str(ros2_dir) + os.sep,))
    monkeypatch.setattr(main, "docker_container_running", lambda container=None: False)

    main.write_text_file(str(typerc), "export CALIBRATION_CURRENT_SCENE=scene_2\n")

    assert typerc.read_text(encoding="utf-8") == "export CALIBRATION_CURRENT_SCENE=scene_2\n"


def test_parse_wifi_scan_output_deduplicates_and_keeps_strongest_signal():
    text = "Home:35:WPA2\nRobot:80:WPA1 WPA2\nHome:70:WPA2\n:hidden:WPA2\n"

    networks = main.parse_wifi_scan_output(text)

    assert networks == [
        {"ssid": "Robot", "signal": 80, "security": "WPA1 WPA2"},
        {"ssid": "Home", "signal": 70, "security": "WPA2"},
    ]


def test_parse_wifi_scan_output_supports_escaped_colons():
    networks = main.parse_wifi_scan_output("Lab\\:A:55:WPA2\n")

    assert networks == [{"ssid": "Lab:A", "signal": 55, "security": "WPA2"}]


def test_render_wifi_config_quotes_values_safely():
    config = main.render_wifi_config("Hiwonder", "pa'ss")

    assert "WIFI_MODE = 2" in config
    assert "WIFI_AP_PASSWORD" in config
    assert "# WIFI_STA_SSID = 'your_wifi_name'" in config
    assert "WIFI_STA_SSID = 'Hiwonder'" in config
    assert 'WIFI_STA_PASSWORD = "pa\'ss"' in config


def test_read_wifi_config_extracts_sta_values(tmp_path):
    path = tmp_path / "wifi_conf.py"
    path.write_text(
        'WIFI_MODE = 2\nWIFI_STA_SSID = "Hiwonder"\nWIFI_STA_PASSWORD = "hiwonder"\n',
        encoding="utf-8",
    )

    assert main.read_wifi_config(str(path)) == {
        "WIFI_MODE": 2,
        "WIFI_STA_SSID": "Hiwonder",
        "WIFI_STA_PASSWORD": "hiwonder",
    }


def test_validate_wifi_ssid_rejects_empty_value():
    assert main.validate_wifi_ssid("  ") is None
    assert main.validate_wifi_ssid(" Robot ") == "Robot"


def test_normalize_machine_type_accepts_old_misspelling():
    assert main.normalize_machine_type("NexArm_Meacuam") == "NexArm_Mecanum"
    assert main.normalize_machine_type("NexArm_Track") == "NexArm_Track"


def test_wifi_config_path_uses_external_toolbox_file():
    assert main.wifi_config_path() == "/home/pi/hiwonder-toolbox/wifi_conf.py"


def test_reset_service_thread_does_not_restart_wifi_by_default():
    thread = main.ResetServiceThread()

    assert thread.restart_wifi is False


def test_write_wifi_config_file_creates_rendered_config(tmp_path):
    path = tmp_path / "wifi_conf.py"

    main.write_wifi_config_file(str(path), "Robot", "12345678")

    assert path.read_text(encoding="utf-8") == main.render_wifi_config("Robot", "12345678")


def test_write_wifi_config_file_preserves_existing_config(tmp_path):
    path = tmp_path / "wifi_conf.py"
    path.write_text(
        "#!/usr/bin/python3\n"
        "#coding:utf8\n\n"
        "WIFI_MODE = 1                              #wifi的工作模式， 1为AP模式， 2为STA模式\n"
        "#WIFI_AP_SSID = 'WN-Robot'                  #AP模式下的SSID。字符和数字构成,需要以 HW- 开头，否则app功能无法使用\n"
        "WIFI_AP_PASSWORD = 'hiwonder'        #AP模式下的WIFI密码,字符和数字构成\n"
        "# WIFI_STA_SSID = 'your_wifi_name'            #STA模式下的SSID\n"
        "# WIFI_STA_PASSWORD = 'your_wifi_password'    #STA模式下的WIFI密码 \n"
        "WIFI_STA_SSID = 'Old'            #STA模式下的SSID\n"
        "WIFI_STA_PASSWORD = 'oldpass'    #STA模式下的WIFI密码 \n",
        encoding="utf-8",
    )

    main.write_wifi_config_file(str(path), "Robot", "12345678")

    config = path.read_text(encoding="utf-8")
    assert "WIFI_MODE = 2                              #wifi的工作模式， 1为AP模式， 2为STA模式" in config
    assert "# WIFI_STA_SSID = 'your_wifi_name'" in config
    assert "WIFI_STA_SSID = 'Robot'            #STA模式下的SSID" in config
    assert "WIFI_STA_PASSWORD = '12345678'    #STA模式下的WIFI密码" in config
    assert "WIFI_AP_PASSWORD = 'hiwonder'        #AP模式下的WIFI密码,字符和数字构成" in config


def test_parse_iw_interface_type_detects_ap_mode():
    text = """Interface wlan0
\tifindex 2
\tssid HW-25064557
\ttype AP
\tchannel 1 (2412 MHz)
"""

    assert main.parse_iw_interface_type(text) == "AP"
    assert main.wifi_scan_requires_ap_cycle(text) is True


def test_active_wifi_connection_from_nmcli_output():
    text = "HW-25064557:802-11-wireless:wlan0\nWired connection 1:802-3-ethernet:eth0\n"

    assert main.active_wifi_connection_from_nmcli_output(text) == "HW-25064557"
