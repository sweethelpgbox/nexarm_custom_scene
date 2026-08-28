# OpenClaw Place Pick Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the OpenClaw dynamic placement coordinate and yaw calculation with the existing working pick logic.

**Architecture:** Keep the existing ROS node and movement flow. Add one helper that resolves placement pixel detections through the same world-coordinate and yaw chain used by picking, then simplify `transport_thread()` to call that helper from a single shared placement path.

**Tech Stack:** Python, ROS2 `rclpy`, OpenCV, existing `app.utils.pick_and_place`, pytest text-contract tests

---

### Task 1: Contract Test

**Files:**
- Modify: `/home/ubuntu/ros2_ws/src/openclaw_controller/test/test_object_sorting_contract.py`

- [ ] **Step 1: Write the failing test**

Add a test that reads `/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/openclaw_object_transport.py` and asserts:

```python
def test_openclaw_object_transport_place_uses_pick_coordinate_chain():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/openclaw_object_transport.py"
    )
    text = node_file.read_text(encoding="utf-8")
    transport_thread = text.split("    def transport_thread(self):", 1)[1].split(
        "    def main(self):", 1
    )[0]

    assert "def resolve_place_pose(self, place_target, target_info):" in text
    assert "position, projection_matrix = self.get_object_world_position(" in text
    assert "result = self.calculate_pick_grasp_yaw(position, place_target, target_info, intrinsic, projection_matrix)" in text
    assert "final_place_position = self.apply_kinematics_calibration(position)" in text
    assert "return utils.normalize_gripper_roll_deg(yaw)" in text
    assert "1500 + int(angle / 180 * 2000)" not in text
    assert "with self.lock:" in transport_thread
    assert "image_seq = self.image_seq" in transport_thread
    assert "self.resolve_place_pose(place_target, [place_target])" in transport_thread
    assert "intrinsic," not in transport_thread
    assert "-pos_place[1]" not in text
    assert "pos_place[1]," not in text
    assert "pick_and_place.place(final_place_position, 80, yaw_place, 200, self.arm_pub)" in text
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/ubuntu/ros2_ws/src/openclaw_controller
python3 -m pytest test/test_object_sorting_contract.py::test_openclaw_object_transport_place_uses_pick_coordinate_chain -q
```

Expected: FAIL because `resolve_place_pose()` does not exist and old coordinate swaps still exist.

### Task 2: Implementation

**Files:**
- Modify: `/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/openclaw_object_transport.py`

- [ ] **Step 1: Add placement resolver helper**

Add this method near the yaw helpers:

```python
    def resolve_place_pose(self, place_target, target_info):
        intrinsic = self.intrinsic
        if intrinsic is None or self.extristric is None or self.white_area_center is None:
            raise RuntimeError("missing calibration data for place pose")

        position, projection_matrix = self.get_object_world_position(
            place_target[2], intrinsic, self.extristric, self.white_area_center
        )
        position[2] = 0.04

        result = self.calculate_pick_grasp_yaw(position, place_target, target_info, intrinsic, projection_matrix)
        if result is not None:
            yaw_place = utils.normalize_gripper_roll_deg(result[0])
        else:
        yaw_place = self.calculate_place_grasp_yaw(position)

        final_place_position = self.apply_kinematics_calibration(position)
        return final_place_position, yaw_place
```

Update `calculate_place_grasp_yaw()` so the fallback returns a normalized roll angle in degrees, not a servo pulse value:

```python
    def calculate_place_grasp_yaw(self, position):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        yaw2 = yaw + 90 if yaw < 0 else yaw - 90
        if abs(yaw2) < abs(yaw):
            yaw = yaw2
        return utils.normalize_gripper_roll_deg(yaw)
```

- [ ] **Step 2: Simplify `transport_thread()` placement branch**

Replace the duplicated left/right placement preparation with a shared flow:

```python
                    with self.lock:
                        image_seq = self.image_seq

                    if target_name in ('red', 'green', 'blue'):
                        self.go_left()
                    else:
                        self.go_right()
                    time.sleep(2)

                    place_result = self.get_color_place_position(target_name, image_seq)
                    if place_result is None:
                        self.get_logger().error(f'No {target_name} place area detected, cancel place')
                        self.go_home(False)
                        self.target = None
                        self.start_transport = False
                        continue

                    pos_place, place_target = place_result
                    self.get_logger().info(f'\033[1;34m{"Place Area":<30}: {place_target[0]}, center: {place_target[2]}\033[0m')
                    cv2.circle(self.latest_image, place_target[2], 5, (255, 0, 255), -1)
                    self.get_logger().info(f'\033[1;34m{"pos_place (Detected)":<30}: {pos_place}\033[0m')

                    final_place_position, yaw_place = self.resolve_place_pose(place_target, [place_target])
                    self.get_logger().info(f'\033[1;34m{"Place Position (Final)":<30}: {final_place_position}, Yaw: {yaw_place}\033[0m')

                    finish = pick_and_place.place(final_place_position, 80, yaw_place, 200, self.arm_pub)
                    self.go_home(not finish)
```

- [ ] **Step 3: Run focused test**

Run:

```bash
cd /home/ubuntu/ros2_ws/src/openclaw_controller
python3 -m pytest test/test_object_sorting_contract.py::test_openclaw_object_transport_place_uses_pick_coordinate_chain -q
```

Expected: PASS.

- [ ] **Step 4: Run contract test file**

Run:

```bash
cd /home/ubuntu/ros2_ws/src/openclaw_controller
python3 -m pytest test/test_object_sorting_contract.py -q
```

Expected: existing text-contract tests pass or reveal unrelated pre-existing contract drift.
