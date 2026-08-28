import os
import re


SCENE5_ID = "scene_5"
DEFAULT_TYPERC_PATH = "/home/ubuntu/ros2_ws/.typerc"
DEFAULT_CONTROLLER_PREFIX = "/ros_robot_controller"
ARM_NAMESPACES = {
    "A": "/arm_a",
    "B": "/arm_b",
}
EXPORT_RE = re.compile(r"^\s*export\s+([A-Za-z_][A-Za-z0-9_]*)=(.*)\s*$")


def clean_export_value(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        value = value[1:-1]
    return value


def parse_exports(text):
    exports = {}
    for line in str(text or "").splitlines():
        match = EXPORT_RE.match(line)
        if match:
            exports[match.group(1)] = clean_export_value(match.group(2))
    return exports


def read_typerc(path=None):
    path = path or os.environ.get("CALIBRATION_TYPERC_PATH", DEFAULT_TYPERC_PATH)
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception:
        return ""


def normalize_scene5_arm_role(role):
    role = str(role or "A").strip().upper()
    return role if role in ARM_NAMESPACES else "A"


def first_value(*values, default=""):
    for value in values:
        if value not in (None, ""):
            return str(value)
    return default


def ros_name(prefix, suffix=""):
    prefix = str(prefix or "").strip().strip("/")
    suffix = str(suffix or "").strip().strip("/")
    if prefix and suffix:
        return f"/{prefix}/{suffix}"
    if prefix:
        return f"/{prefix}"
    if suffix:
        return f"/{suffix}"
    return "/"


class SceneEnvironment:
    def __init__(self, scene_id, arm_role):
        self.scene_id = str(scene_id or "")
        self.arm_role = normalize_scene5_arm_role(arm_role)

    @property
    def is_scene5(self):
        return self.scene_id == SCENE5_ID

    @property
    def role_namespace(self):
        return ARM_NAMESPACES[self.arm_role] if self.is_scene5 else ""

    @property
    def role_namespace_name(self):
        return self.role_namespace.strip("/")

    @property
    def controller_prefix(self):
        if not self.is_scene5:
            return DEFAULT_CONTROLLER_PREFIX
        return ros_name(self.role_namespace, "ros_robot_controller")

    @property
    def calibration_controller_prefix(self):
        if not self.is_scene5:
            return DEFAULT_CONTROLLER_PREFIX
        return "/arm_a/ros_robot_controller"

    @property
    def calibration_allowed(self):
        return not self.is_scene5 or self.arm_role == "A"

    @property
    def calibration_enabled_launch_value(self):
        return "true" if self.calibration_allowed else "false"

    @property
    def camera_name(self):
        if not self.is_scene5:
            return "depth_cam"
        return ros_name(self.role_namespace, "depth_cam").strip("/")

    @property
    def camera_prefix(self):
        if not self.is_scene5:
            return "/depth_cam"
        return ros_name(self.role_namespace, "depth_cam")

    def topic(self, suffix):
        if self.is_scene5:
            return ros_name(self.role_namespace, suffix)
        return ros_name("", suffix)

    def camera_topic(self, suffix):
        return ros_name(self.camera_prefix, suffix)

    def controller_topic(self, suffix):
        return ros_name(self.controller_prefix, suffix)

    def yolo_namespace(self, name="yolo"):
        if self.is_scene5:
            return ros_name(self.role_namespace, name)
        return str(name or "yolo").strip("/")


def load_scene_environment(text=None, environ=None):
    environ = os.environ if environ is None else environ
    exports = parse_exports(read_typerc() if text is None else text)
    scene_id = first_value(
        environ.get("CALIBRATION_CURRENT_SCENE"),
        environ.get("CALIBRATION_DEFAULT_SCENE"),
        environ.get("SCENE"),
        exports.get("CALIBRATION_CURRENT_SCENE"),
        exports.get("CALIBRATION_DEFAULT_SCENE"),
        exports.get("SCENE"),
        default="",
    )
    arm_role = first_value(
        environ.get("SCENE5_ARM_ROLE"),
        exports.get("SCENE5_ARM_ROLE"),
        default="A",
    )
    return SceneEnvironment(scene_id, arm_role)
