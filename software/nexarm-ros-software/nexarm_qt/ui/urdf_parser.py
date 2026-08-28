"""Lightweight URDF parser for NexArm — extracts joint chain and STL paths."""

import xml.etree.ElementTree as ET
import os
import numpy as np


def _parse_vec(elem, attr, default=(0, 0, 0)):
    node = elem.find(attr)
    if node is None:
        return list(default)
    text = node.get("xyz") or node.get("rpy")
    if text is None:
        # try xyz first, then rpy
        text = node.get("xyz", "0 0 0")
    return [float(v) for v in text.split()]


def parse_urdf(urdf_path, stl_dir=None):
    """Parse URDF and return (links, joints) for the arm chain.

    Returns
    -------
    links : dict  {link_name: {"stl_path": str|None, "origin_xyz": list, "origin_rpy": list}}
    joints : list of dict, ordered from base to tip
        Each dict: name, type, parent, child, axis, origin_xyz, origin_rpy, lower, upper
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    if stl_dir is None:
        # Try common locations for STL files
        base = os.path.dirname(os.path.dirname(urdf_path))
        import sys
        candidates = [
            os.path.join(base, "STL"),
            os.path.join(base, "meshes"),
            os.path.join(base, "..", "STL"),
            os.path.join(base, "..", "meshes"),
            # nexarm_description package layout
            os.path.join(base, "..", "..", "nexarm_description", "nexarm_description", "meshes"),
        ]
        # PyInstaller bundled path
        if hasattr(sys, '_MEIPASS'):
            candidates.insert(0, os.path.join(sys._MEIPASS, "STL"))
        for c in candidates:
            c = os.path.abspath(c)
            if os.path.isdir(c):
                stl_dir = c
                break
        if stl_dir is None:
            stl_dir = os.path.join(base, "STL")  # fallback
    stl_dir = os.path.abspath(stl_dir)

    # --- Parse links ---
    links = {}
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        stl_path = None
        vis_origin_xyz = [0, 0, 0]
        vis_origin_rpy = [0, 0, 0]

        visual = link_elem.find("visual")
        if visual is not None:
            origin = visual.find("origin")
            if origin is not None:
                vis_origin_xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                vis_origin_rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                fn = mesh.get("filename", "")
                # filename may be like "package://xxx/meshes/link1.STL" or just "link1.STL"
                basename = os.path.basename(fn)
                candidate = os.path.join(stl_dir, basename)
                if os.path.exists(candidate):
                    stl_path = candidate

        links[name] = {
            "stl_path": stl_path,
            "origin_xyz": vis_origin_xyz,
            "origin_rpy": vis_origin_rpy,
        }

    # --- Parse joints ---
    all_joints = []
    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name")
        jtype = joint_elem.get("type")

        origin = joint_elem.find("origin")
        origin_xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
        origin_rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]

        parent = joint_elem.find("parent").get("link")
        child = joint_elem.find("child").get("link")

        axis_elem = joint_elem.find("axis")
        axis = [float(v) for v in axis_elem.get("xyz", "0 0 1").split()] if axis_elem is not None else [0, 0, 1]

        limit_elem = joint_elem.find("limit")
        lower = float(limit_elem.get("lower", "0")) if limit_elem is not None else 0
        upper = float(limit_elem.get("upper", "0")) if limit_elem is not None else 0

        all_joints.append({
            "name": name,
            "type": jtype,
            "parent": parent,
            "child": child,
            "axis": axis,
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "lower": lower,
            "upper": upper,
        })

    # Order joints from base to tip by walking the kinematic chain
    child_map = {j["parent"]: j for j in all_joints if j["type"] in ("revolute", "continuous")}
    ordered = []
    current = "base_link"
    while current in child_map:
        j = child_map[current]
        ordered.append(j)
        current = j["child"]

    return links, ordered


def rpy_to_matrix(rpy):
    """Roll-pitch-yaw (XYZ extrinsic) to 3x3 rotation matrix."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axis_angle_matrix(axis, angle):
    """Rotation matrix for rotation about `axis` by `angle` radians."""
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return np.eye(3)
    axis = axis / norm
    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])


def make_transform(xyz, rpy):
    """Create 4x4 homogeneous transform from xyz translation and rpy rotation."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def joint_transform(joint, angle):
    """4x4 transform for a joint at given angle (radians)."""
    T_origin = make_transform(joint["origin_xyz"], joint["origin_rpy"])
    R = np.eye(4)
    R[:3, :3] = axis_angle_matrix(joint["axis"], angle)
    return T_origin @ R
