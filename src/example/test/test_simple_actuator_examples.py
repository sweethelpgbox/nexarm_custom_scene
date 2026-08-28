from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
EXAMPLE_SRC = ROOT / "src" / "example"


def test_simple_examples_wait_for_current_controller_ready_service():
    simple_dir = EXAMPLE_SRC / "example" / "simple" / "include"

    for filename in ("bus_servo_node.py", "buzzer_node.py"):
        text = (simple_dir / filename).read_text(encoding="utf-8")

        assert "/ros_robot_controller/init_finish" not in text
        assert "/controller_manager/init_finish" in text
        assert "wait_for_service" in text


def test_bus_servo_example_matches_current_message_schema():
    text = (EXAMPLE_SRC / "example" / "simple" / "include" / "bus_servo_node.py").read_text(encoding="utf-8")

    assert "msg.duration" not in text


def test_example_setup_installs_simple_namespace_packages():
    text = (EXAMPLE_SRC / "setup.py").read_text(encoding="utf-8")

    assert "find_namespace_packages" in text
    assert "packages=find_namespace_packages" in text
