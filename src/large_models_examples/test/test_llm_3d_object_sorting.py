import os

os.environ.setdefault("ASR_LANGUAGE", "Chinese")
os.environ.setdefault("ASR_MODE", "offline")

from large_models_examples.llm_3d_object_sorting import parse_object_sorting_objects


def test_parse_object_sorting_objects_accepts_action_list():
    actions = ["object_sorting('cylinder')"]

    assert parse_object_sorting_objects(actions) == ["cylinder"]


def test_parse_object_sorting_objects_accepts_action_string():
    action = "object_sorting('cylinder', 'cuboid')"

    assert parse_object_sorting_objects(action) == ["cylinder", "cuboid"]
