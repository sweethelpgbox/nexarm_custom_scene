# Color Plate Sorting Design

## Goal

Add a new sorting node to `openclaw_controller` that can:

- detect colored objects on the table
- pick only enabled target colors
- detect color-matched trays dynamically instead of using fixed place coordinates
- verify tray color again before placing

## Scope

The node lives inside `openclaw_controller` and follows the service model already used by `app/object_sorting.py`:

- `~/enter`
- `~/exit`
- `~/enable_sorting`
- `~/set_target`

It also publishes an annotated debug image and exposes a simple status service.

## Architecture

The implementation is split into two layers:

1. `color_plate_sorting_utils.py`
   Pure functions for color-region detection, tray matching, and target selection. This is the main test surface.

2. `color_plate_sorting.py`
   ROS2 node that subscribes to RGB-D data, manages state, performs pick/place sequencing, and uses the utility layer for perception decisions.

## Perception Rules

- Objects are detected by color within a workspace ROI.
- Trays are detected by color using separate contour thresholds from objects.
- Placement uses `item_color == tray_color`.
- A tray is valid only if it is seen in consecutive frames with stable center and area.
- Placement is aborted if the tray cannot be revalidated before release.

## Motion Rules

- Pick pose is computed from the selected object center and local depth.
- Place pose is computed from the matched tray center and local depth plus configurable Z offset.
- The node reuses the RGB-D projection and kinematics flow already present in `claw_track_and_grab.py`.

## Deliverables

- new utility module
- new sorting node
- package entry point
- launch file
- config file
- focused utility tests

## Constraints

- Keep changes scoped to `openclaw_controller`.
- Reuse existing color tables and ROS patterns where practical.
- Prefer utility-level tests over full ROS integration tests for this first implementation.
