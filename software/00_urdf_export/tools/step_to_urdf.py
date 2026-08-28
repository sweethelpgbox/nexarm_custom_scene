#!/usr/bin/env python
"""Convert a STEP file to a display URDF using OpenCascade/OCP.

This script intentionally creates a single-link URDF. STEP files normally do
not encode robot joints; a useful articulated URDF still needs link grouping
and joint frames from CAD or manual measurements.
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _add_local_deps(repo_root: Path) -> None:
    deps = repo_root / ".codex_deps"
    if deps.exists():
        sys.path.insert(0, str(deps))


def _load_step(step_path: Path):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader

    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {step_path}")

    reader.TransferRoots()
    return reader.OneShape()


def _mesh_shape(shape, linear_deflection: float, angular_deflection: float) -> None:
    from OCP.BRepMesh import BRepMesh_IncrementalMesh

    mesher = BRepMesh_IncrementalMesh(
        shape,
        linear_deflection,
        False,
        angular_deflection,
        True,
    )
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError("OpenCascade meshing failed")


def _write_stl(shape, stl_path: Path, ascii_stl: bool) -> None:
    from OCP.StlAPI import StlAPI_Writer

    stl_path.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = ascii_stl
    ok = writer.Write(shape, str(stl_path))
    if not ok:
        raise RuntimeError(f"Failed to write STL file: {stl_path}")


def _shape_bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        return None
    return box.Get()


def _indent(elem: ET.Element, level: int = 0) -> None:
    space = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = space + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = space
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = space


def _add_origin(parent: ET.Element, xyz: str = "0 0 0", rpy: str = "0 0 0") -> ET.Element:
    return ET.SubElement(parent, "origin", {"xyz": xyz, "rpy": rpy})


def _add_mesh_geometry(parent: ET.Element, mesh_filename: str, scale: str) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": mesh_filename, "scale": scale})


def _write_single_link_urdf(
    urdf_path: Path,
    robot_name: str,
    link_name: str,
    mesh_filename: str,
    mesh_scale: str,
    color_rgba: str,
) -> None:
    urdf_path.parent.mkdir(parents=True, exist_ok=True)

    robot = ET.Element("robot", {"name": robot_name})
    robot.append(ET.Comment(" Generated from STEP as one visual/collision link. "))
    robot.append(ET.Comment(" Add articulated joints manually after splitting the CAD into per-link meshes. "))

    link = ET.SubElement(robot, "link", {"name": link_name})

    inertial = ET.SubElement(link, "inertial")
    _add_origin(inertial)
    ET.SubElement(inertial, "mass", {"value": "1.0"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "1e-3",
            "ixy": "0",
            "ixz": "0",
            "iyy": "1e-3",
            "iyz": "0",
            "izz": "1e-3",
        },
    )

    visual = ET.SubElement(link, "visual")
    _add_origin(visual)
    _add_mesh_geometry(visual, mesh_filename, mesh_scale)
    material = ET.SubElement(visual, "material", {"name": "nexarm_light"})
    ET.SubElement(material, "color", {"rgba": color_rgba})

    collision = ET.SubElement(link, "collision")
    _add_origin(collision)
    _add_mesh_geometry(collision, mesh_filename, mesh_scale)

    _indent(robot)
    tree = ET.ElementTree(robot)
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", default="00.stp", help="Input STEP/STP file")
    parser.add_argument(
        "--mesh",
        default="nexarm_description/meshes/00_full.stl",
        help="Output STL mesh path",
    )
    parser.add_argument(
        "--urdf",
        default="nexarm_description/urdf/nexarm_from_step.urdf",
        help="Output URDF path",
    )
    parser.add_argument("--robot-name", default="nexarm_from_step")
    parser.add_argument("--link-name", default="base_link")
    parser.add_argument(
        "--mesh-filename",
        default="package://nexarm_description/meshes/00_full.stl",
        help="Mesh filename written into the URDF",
    )
    parser.add_argument(
        "--mesh-scale",
        default="0.001 0.001 0.001",
        help="URDF mesh scale. 00.stp declares millimetres, so default converts mm to metres.",
    )
    parser.add_argument(
        "--linear-deflection",
        type=float,
        default=0.5,
        help="Meshing linear deflection in STEP units. Smaller is finer and slower.",
    )
    parser.add_argument(
        "--angular-deflection",
        type=float,
        default=0.5,
        help="Meshing angular deflection in radians.",
    )
    parser.add_argument("--ascii-stl", action="store_true", help="Write ASCII STL instead of binary")
    parser.add_argument("--skip-mesh", action="store_true", help="Only write URDF")
    parser.add_argument("--color", default="0.79216 0.81961 0.93333 1")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path.cwd()
    _add_local_deps(repo_root)

    step_path = Path(args.step)
    mesh_path = Path(args.mesh)
    urdf_path = Path(args.urdf)

    if not step_path.exists():
        raise FileNotFoundError(step_path)

    if not args.skip_mesh:
        shape = _load_step(step_path)
        bounds = _shape_bounds(shape)
        if bounds is not None:
            print("STEP bounds:", bounds)
        _mesh_shape(shape, args.linear_deflection, args.angular_deflection)
        _write_stl(shape, mesh_path, args.ascii_stl)
        print(f"Wrote mesh: {mesh_path}")

    _write_single_link_urdf(
        urdf_path=urdf_path,
        robot_name=args.robot_name,
        link_name=args.link_name,
        mesh_filename=args.mesh_filename,
        mesh_scale=args.mesh_scale,
        color_rgba=args.color,
    )
    print(f"Wrote URDF: {urdf_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
