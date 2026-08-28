from pathlib import Path


REPO_QT_ROOT = Path(__file__).resolve().parents[3] / "nexarm-ros-software/nexarm_qt"
ROOT = REPO_QT_ROOT if REPO_QT_ROOT.exists() else Path("/home/ubuntu/nexarm-ros-software/nexarm_qt")


def test_external_nexarm_qt_offers_scene4_and_clears_default_scene_key():
    main_window = ROOT / "ui" / "main_window.py"
    text = main_window.read_text(encoding="utf-8")

    assert '("scene_4", "Scene 4")' in text
    assert "CALIBRATION_DEFAULT_SCENE" in text
    assert "CALIBRATION_CURRENT_SCENE" in text
    assert 'remove_export(text, "CALIBRATION_DEFAULT_SCENE")' in text
    assert 'append_unset_before_echo(text, "CALIBRATION_DEFAULT_SCENE")' in text


def test_external_environment_tab_shows_scene4_home_and_current_scene():
    environment_tab = ROOT / "ui" / "environment_tab.py"
    text = environment_tab.read_text(encoding="utf-8")

    assert '"CALIBRATION_CURRENT_SCENE"' in text
    assert '"CALIBRATION_SCENE4_HOME_X"' in text
    assert '"CALIBRATION_SCENE4_HOME_Y"' in text
    assert '"CALIBRATION_SCENE4_HOME_Z"' in text
    assert '"CALIBRATION_SCENE4_HOME_PITCH"' in text
