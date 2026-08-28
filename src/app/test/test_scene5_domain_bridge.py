import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_scene5_domain_bridge_uses_only_required_topics():
    from app import scene5_tcp_bridge

    assert scene5_tcp_bridge.B_IMAGE_TOPIC == (
        "/arm_b/waste_classification_motor_depth/result_image/compressed"
    )
    assert scene5_tcp_bridge.B_IMAGE_MSG_TYPE.__name__ == "CompressedImage"
    assert scene5_tcp_bridge.B_CONVEYOR_TOPIC == "/arm_b/ros_robot_controller/conveyor/set"
    assert not hasattr(scene5_tcp_bridge, "B_SERVICE_SPECS")


def test_scene5_domain_bridge_defaults_to_a_side_bridge():
    from app import scene5_tcp_bridge

    config = scene5_tcp_bridge.load_bridge_config({})

    assert config.arm_role == "A"
    assert config.a_domain_id == scene5_tcp_bridge.DEFAULT_A_DOMAIN_ID
    assert config.b_domain_id == scene5_tcp_bridge.DEFAULT_B_DOMAIN_ID
    assert config.run_domain_bridge is True


def test_scene5_domain_bridge_b_role_stays_idle():
    from app import scene5_tcp_bridge

    config = scene5_tcp_bridge.load_bridge_config(
        {
            "SCENE5_ARM_ROLE": "B",
            "ROS_DOMAIN_ID": "79",
            "SCENE5_BRIDGE_A_DOMAIN_ID": "78",
        }
    )

    assert config.arm_role == "B"
    assert config.a_domain_id == 78
    assert config.b_domain_id == 79
    assert config.run_domain_bridge is False


def test_scene5_domain_bridge_reads_explicit_domain_ids():
    from app import scene5_tcp_bridge

    config = scene5_tcp_bridge.load_bridge_config(
        {
            "SCENE5_ARM_ROLE": "A",
            "ROS_DOMAIN_ID": "91",
            "SCENE5_BRIDGE_A_DOMAIN_ID": "92",
            "SCENE5_BRIDGE_B_DOMAIN_ID": "93",
        }
    )

    assert config.arm_role == "A"
    assert config.a_domain_id == 92
    assert config.b_domain_id == 93
    assert config.run_domain_bridge is True


def test_scene5_domain_bridge_rejects_same_domain_ids():
    from app import scene5_tcp_bridge

    bridge = scene5_tcp_bridge.Scene5DomainBridge(
        scene5_tcp_bridge.BridgeConfig(
            arm_role="A",
            a_domain_id=78,
            b_domain_id=78,
            run_domain_bridge=True,
        )
    )

    try:
        bridge.setup()
    except RuntimeError as exc:
        assert "different ROS_DOMAIN_ID" in str(exc)
    else:
        raise AssertionError("bridge accepted identical ROS_DOMAIN_ID values")


def test_scene5_domain_bridge_does_not_use_tcp_or_jpeg_transport():
    bridge = Path(__file__).resolve().parents[1] / "app" / "scene5_tcp_bridge.py"
    text = bridge.read_text(encoding="utf-8")

    assert "socket" not in text
    assert "cv2.imencode" not in text
    assert "SCENE5_BRIDGE_JPEG_QUALITY" not in text
    assert "BridgeServerNode" not in text
    assert "BridgeClientNode" not in text
