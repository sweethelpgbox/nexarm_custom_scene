# model_00_description

This folder was generated from `00.stp`.

Contents:

- `urdf/00.urdf`: single-link URDF for display.
- `meshes/00_full.stl`: STL mesh exported from the STEP file.
- `tools/step_to_urdf.py`: local conversion script using OpenCascade/OCP.

The original STEP file declares millimetres, so the URDF mesh uses:

```xml
scale="0.001 0.001 0.001"
```

This is a display model. The STEP file does not include robot joint semantics,
so articulated joints need separate link meshes and manually defined joint
frames.
