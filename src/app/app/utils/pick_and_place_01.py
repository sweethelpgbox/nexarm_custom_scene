#!/usr/bin/python3
# coding=utf8
"""Compatibility wrapper for the ArmCoords-only pick_and_place implementation."""

try:
    from app.utils.pick_and_place import (  # noqa: F401
        CLAW_FULL_CLOSE,
        CLAW_GRAB,
        CLAW_OPEN,
        interrupt,
        load_scene_home_pose,
        pick,
        pick_without_back,
        place,
        pulse_to_claw,
        publish_arm,
    )
except ImportError:
    from .pick_and_place import (  # noqa: F401
        CLAW_FULL_CLOSE,
        CLAW_GRAB,
        CLAW_OPEN,
        interrupt,
        load_scene_home_pose,
        pick,
        pick_without_back,
        place,
        pulse_to_claw,
        publish_arm,
    )
