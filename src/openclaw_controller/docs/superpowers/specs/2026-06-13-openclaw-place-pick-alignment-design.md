# OpenClaw Place Pick Alignment Design

## Goal

Make the dynamic placement path in `openclaw_object_transport.py` use the same coordinate and gripper-yaw calculation chain as the working pick path.

## Current Problem

The pick path computes a world position with `get_object_world_position()`, computes yaw with `calculate_pick_grasp_yaw()`, applies kinematics calibration, then calls `pick_and_place.pick()`.

The place path currently diverges after detecting the color placement area:

- It references `intrinsic` inside `transport_thread()` without defining it locally.
- It can reference `image_seq` in the right-side branch before assignment.
- It duplicates left/right placement code.
- It applies extra left/right axis swaps and sign changes to the detected placement position.

Those differences mean the attempted pick-style placement calculation falls back to old logic and can produce inconsistent placement coordinates.

## Design

Keep the existing target classification, left/right observation moves, color-area detection, and `pick_and_place.place()` call. Replace only the placement target preparation.

Add a small helper on `ObjectSortingNode`:

`resolve_place_pose(place_target, target_info)`

The helper will:

1. Read `self.intrinsic`, `self.extristric`, and `self.white_area_center`.
2. Convert `place_target[2]` from image pixel coordinates to world coordinates via `get_object_world_position()`.
3. Compute `yaw_place` by calling `calculate_pick_grasp_yaw()` with the placement target and available target info.
4. Fall back to `calculate_place_grasp_yaw()` only if `calculate_pick_grasp_yaw()` returns `None`; the fallback must also return a roll angle in degrees for `pick_and_place.place()`.
5. Apply `apply_kinematics_calibration()` to the world position.
6. Return `(final_place_position, yaw_place)`.

`transport_thread()` will use one shared placement branch:

1. Record `image_seq` immediately after pick succeeds.
2. Move to the left observation pose for red, green, and blue targets; otherwise move to the right observation pose.
3. Detect the requested color placement area with `get_color_place_position()`.
4. Resolve the final placement pose with `resolve_place_pose()`.
5. Call `pick_and_place.place(final_place_position, 80, yaw_place, 200, self.arm_pub)`.

The left/right axis swap blocks will be removed. The final placement coordinate will come from the same camera/world/kinematics chain used by picking.

## Testing

Use the repository's existing text-contract style tests because the ROS node imports hardware and ROS dependencies that are not safe to import in ordinary pytest.

Add focused assertions that:

- `openclaw_object_transport.py` defines `resolve_place_pose()`.
- `transport_thread()` captures `image_seq` before the left/right branch.
- `transport_thread()` no longer references a bare local `intrinsic`.
- The old left/right coordinate swap blocks are absent.
- The final place call still uses `pick_and_place.place(final_place_position, 80, yaw_place, 200, self.arm_pub)`.
