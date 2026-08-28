# Color Plate Sorting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new `openclaw_controller` sorting node that dynamically finds color trays and revalidates tray color before placing.

**Architecture:** Build tested pure perception helpers first, then compose them into a ROS2 node that mirrors the existing sorting service interface and reuses the current RGB-D pick/place flow.

**Tech Stack:** Python, ROS2 `rclpy`, OpenCV, existing `openclaw_controller` helpers, pytest

---

### Task 1: Detection Utility Tests

**Files:**
- Create: `test/test_color_plate_sorting_utils.py`

- [ ] **Step 1: Write failing tests**
- [ ] **Step 2: Run the tests and verify the expected failures**
- [ ] **Step 3: Implement the minimal utility code**
- [ ] **Step 4: Re-run the utility tests**

### Task 2: Utility Implementation

**Files:**
- Create: `openclaw_controller/color_plate_sorting_utils.py`
- Modify: `openclaw_controller/scene_task_utils.py`

- [ ] **Step 1: Add reusable color-region detection helpers**
- [ ] **Step 2: Add tray matching and stable-selection helpers**
- [ ] **Step 3: Re-run focused tests**

### Task 3: Sorting Node

**Files:**
- Create: `openclaw_controller/color_plate_sorting.py`

- [ ] **Step 1: Add ROS services and subscriptions**
- [ ] **Step 2: Add perception loop and target/tray selection**
- [ ] **Step 3: Add pick/place workflow with tray revalidation**
- [ ] **Step 4: Run focused tests plus import smoke checks**

### Task 4: Package Wiring

**Files:**
- Create: `config/color_plate_sorting.yaml`
- Create: `launch/color_plate_sorting.launch.py`
- Modify: `setup.py`

- [ ] **Step 1: Add runtime config**
- [ ] **Step 2: Add launch file**
- [ ] **Step 3: Add console entry point**

### Task 5: Verification

**Files:**
- Verify: `test/test_color_plate_sorting_utils.py`

- [ ] **Step 1: Run focused pytest**
- [ ] **Step 2: Run a Python import smoke check for the new node**
- [ ] **Step 3: Report actual status and residual risks**
