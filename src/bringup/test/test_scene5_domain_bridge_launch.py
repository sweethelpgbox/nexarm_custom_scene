from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"


def test_scene5_domain_bridge_launches_only_on_a_arm():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "if scene_env.is_scene5 and scene_env.arm_role == 'A':" in text
    assert "SCENE5_BRIDGE_A_DOMAIN_ID" in text
    assert "SCENE5_BRIDGE_B_DOMAIN_ID" in text
    assert "scene5 两个 ROS_DOMAIN_ID 只在 A 机桥接图像和传送带控制" in text


def test_scene5_domain_bridge_launch_does_not_start_b_side_bridge():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "scene5 两台机器都需要启动 TCP bridge" not in text
    assert "if scene_env.is_scene5:" not in text
    assert "additional_env={'SCENE5_ARM_ROLE': scene_env.arm_role}" not in text


def test_scene5_b_still_skips_web_frontend_nodes():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "is_scene5_b = scene_env.is_scene5 and scene_env.arm_role == 'B'" in text
    assert (
        "extra_nodes = [] if is_scene5_b else "
        "[web_video_server_node, rosbridge_websocket_launch]"
    ) in text
