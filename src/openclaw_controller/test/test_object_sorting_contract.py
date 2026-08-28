from pathlib import Path


def test_openclaw_object_sorting_exists_and_removes_tag_detection():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    assert node_file.exists()

    text = node_file.read_text(encoding="utf-8")
    assert "dt_apriltags" not in text
    assert "Detector(" not in text
    assert "tag1" not in text
    assert "tag2" not in text
    assert "tag3" not in text


def test_openclaw_object_sorting_launch_targets_local_package():
    launch_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/launch/object_sorting.launch.py"
    )
    text = launch_file.read_text(encoding="utf-8")
    assert "package='openclaw_controller'" in text
    assert "executable='object_sorting'" in text
    assert "executable='object_placement'" in text


def test_openclaw_object_sorting_reuses_default_place_targets():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")
    assert "place_position = DEFAULT_SCENE_PLACE_TARGETS.copy()" in text
    assert "self.place_position.get(target_key, DEFAULT_SCENE_PLACE_TARGETS.get(target_key))" not in text


def test_openclaw_object_sorting_stands_by_without_enter_exit_services():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "~/enter" not in text
    assert "~/exit" not in text
    assert "self.enter_srv_callback(" not in text
    assert "self.exit_srv_callback(" not in text
    assert "self.prepare_sorting()" in text
    assert "self.start_sorting()" not in text


def test_openclaw_object_sorting_set_target_controls_sorting_state():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.enable_sorting = False" in text
    assert "if any(self.target_labels.values()):" in text
    assert "pick_and_place.interrupt(False)" in text
    assert "pick_and_place.interrupt(True)" in text


def test_openclaw_object_sorting_accepts_yellow_without_key_error():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert '"yellow": False' in text
    assert "self.target_labels.get(target[0], False)" in text


def test_openclaw_object_sorting_reports_missing_target_block_message():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "from std_msgs.msg import String" in text
    assert "self.missing_target_pub = self.create_publisher(String, '~/missing_target_color', 10)" in text
    assert "self.create_service(Trigger, '~/get_missing_target_color', self.get_missing_target_color_callback)" in text
    assert "def format_missing_target_message(self, missing_target_colors):" in text
    assert "def publish_missing_target_message(self, missing_target_colors):" in text
    assert "missing_target_colors = self.get_missing_target_colors(detected_colors)" in text
    assert "self.publish_missing_target_message(missing_target_colors)" in text
    assert "缺少" in text
    assert "色块" in text


def test_openclaw_object_sorting_publishes_place_request_after_pick():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.place_request_pub = self.create_publisher(String, '~/place_request', 10)" in text
    assert "def publish_place_request(self, target_color):" in text
    assert "requested_color = self.placed_color" in text
    assert "self.try_publish_place_request(requested_color)" in text
    assert "threading.Thread(target=self.wait_for_place_status, args=(requested_color,), daemon=True).start()" in text
    assert "pick_and_place.place(" not in text


def test_openclaw_object_sorting_gates_place_requests_while_in_flight():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.place_request_in_flight = False" in text
    assert "def try_publish_place_request(self, target_color):" in text
    assert "if self.place_request_in_flight:" in text
    assert "self.place_request_in_flight = True" in text
    assert "self.try_publish_place_request(self.placed_color)" in text
    assert "self.try_publish_place_request(requested_color)" in text


def test_openclaw_object_sorting_accepts_placed_color_service():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "from interfaces.srv import SetStringBool, SetString" in text
    assert "self.placed_color = ''" in text
    assert "self.set_placed_color_srv = self.create_service(SetString, '~/set_placed_color', self.set_placed_color_srv_callback)" in text
    assert "def set_placed_color_srv_callback(self, request, response):" in text
    assert "placed_color = request.data.strip()" in text
    assert "if placed_color not in self.target_labels:" in text
    assert "self.placed_color = placed_color" in text
    assert "response.success = True" in text


def test_openclaw_object_placement_program_exists_and_places_by_color():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    setup_file = Path("/home/ubuntu/ros2_ws/src/openclaw_controller/setup.py")

    assert node_file.exists()
    text = node_file.read_text(encoding="utf-8")
    setup_text = setup_file.read_text(encoding="utf-8")

    assert "RIGHT_BOX_OBSERVE_POSE = (0.0, -200.0, 200.0, -90.0, 0.0, 0.0, 1000)" in text
    assert "DEFAULT_SCENE_PLACE_TARGETS = {" in text
    assert "1: [0.087, 0.133, 0.015]" in text
    assert "2: [0.017, 0.133, 0.015]" in text
    assert "3: [-0.053, 0.133, 0.015]" in text
    assert "COLOR_FALLBACK_PLACE_INDEX = {'red': 1, 'green': 2, 'blue': 3}" in text
    assert "self.create_subscription(String, '/object_sorting/place_request', self.place_request_callback, 10)" in text
    assert "self.result_publisher = self.create_publisher(Image, '~/image_result', 1)" in text
    assert "def publish_result_image(self, bgr_image, target_info=None):" in text
    assert "self.result_publisher.publish(self.bridge.cv2_to_imgmsg(draw_image, \"bgr8\"))" in text
    assert "self.publish_result_image(self.latest_bgr_image, target_info)" in text
    assert "def detect_color_box_center(self, target_color):" in text
    assert "def resolve_place_index_from_center(self, center):" in text
    assert "if 100 <= x <= 200:" in text
    assert "if 201 <= x <= 300:" in text
    assert "if 301 <= x <= 400:" in text
    assert "def resolve_place_index_for_target_color(self, target_color):" in text
    assert "center = self.detect_color_box_center(target_color)" in text
    assert "place_index = self.resolve_place_index_from_center(center)" in text
    assert "return COLOR_FALLBACK_PLACE_INDEX[target_color]" in text
    assert "def place_request_callback(self, msg):" in text
    assert "place_index = self.resolve_place_index_for_target_color(target_color)" in text
    assert "place_position = copy.deepcopy(self.place_position[place_index])" in text
    assert "from servo_controller_msgs.msg import ServosPosition" in text
    assert "from kinematics.kinematics_control import set_pose_target" in text
    assert "from kinematics_msgs.srv import SetRobotPose" in text
    assert "from servo_controller.bus_servo_control import set_servo_position" in text
    assert "self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)" in text
    assert "self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')" in text
    assert "def send_request(self, client, msg):" in text
    assert "def place_with_kinematics(self, place_position):" in text
    assert "msg = set_pose_target(position, self.place_pitch, [-90.0, 90.0], 1.0)" in text
    assert "set_servo_position(self.joints_pub, 2.0, ((6, servo_data[0]),))" in text
    assert "set_servo_position(self.joints_pub, 1.0, ((1, 200),))" in text
    assert "self.place_with_kinematics(place_position)" in text
    assert "pick_and_place.place(" not in text
    assert "def get_object_world_position(" not in text
    assert "def apply_kinematics_calibration(" not in text
    assert "common.pixels_to_world" not in text
    assert "'object_placement = openclaw_controller.object_placement:main'" in setup_text


def test_openclaw_object_placement_waits_for_requested_color_and_places_by_slot():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.color_sub = self.create_subscription(String, '/object_sorting/place_request', self.get_color_callback, 1)" in text
    assert "self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)" in text
    assert "if not self.color or self.color not in self.lab_data['color_range_list']:" in text
    assert "return target_list, result_image" in text
    assert "def get_scene5_place_position(self, center_x, center_y):" in text
    assert "if center_x < 160:" in text
    assert "if center_y < 240:" in text
    assert "place_pos = self.get_scene5_place_position(center_x, center_y)" in text
    assert "self.submit_place_position(place_pos)" in text


def test_openclaw_object_placement_places_left_request_without_image_processing():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "'left': [30, 210.0, 200.0]" in text
    assert "LEFT_PLACE_POSITION" not in text
    assert "if color == 'left':" in text
    assert "self.color = color" in text
    assert "place_pos = PLACE_POSITION['left']" in text
    assert "self.set_place_status(True, f\"found:{self.color}\")" in text
    assert "self.submit_place_position(place_pos)" in text
    assert "threading.Thread(target=self.place_worker, daemon=True).start()" in text
    assert "threading.Thread(target=self.move" not in text
    assert "def submit_place_position(self, position):" in text
    assert "def place_worker(self):" in text

    left_branch = text.split("if color == 'left':", 1)[1].split(
        "if color not in self.lab_data['color_range_list']:", 1
    )[0]
    assert "image_processing(" not in left_branch
    assert "self.lab_data['color_range_list']" not in left_branch


def test_openclaw_object_placement_draws_yellow_horizontal_reference_lines():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "def draw_reference_lines(self, image):" in text
    assert "cv2.line(image, (0, 100), (img_w - 1, 100), (255, 255, 0), 2)" in text
    assert "cv2.line(image, (0, 300), (img_w - 1, 300), (255, 255, 0), 2)" in text
    assert "self.draw_reference_lines(result_image)" in text


def test_openclaw_object_placement_passes_place_position_as_one_thread_argument():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "def move(self, position):" in text
    assert "self.place_position_queue = queue.Queue(maxsize=1)" in text
    assert "threading.Thread(target=self.place_worker, daemon=True).start()" in text
    assert "self.submit_place_position(place_pos)" in text
    assert "threading.Thread(target=self.move, args=(place_pos), daemon=True).start()" not in text


def test_openclaw_object_placement_exposes_status_service_when_requested_plate_missing():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.place_status_srv = self.create_service(Trigger, '~/place_status', self.get_place_status_callback)" in text
    assert "self.no_plate_reported = False" in text
    assert "self.place_status = 'idle'" in text
    assert "self.place_status_success = False" in text
    assert "def set_place_status(self, success, text):" in text
    assert "def get_place_status_callback(self, request, response):" in text
    assert "response.success = self.place_status_success" in text
    assert "response.message = self.place_status" in text
    assert "self.no_plate_reported = False" in text
    assert "if self.color and self.start_count and not self.placing and not target_list and not self.no_plate_reported:" in text
    assert "self.set_place_status(False, f\"not_found:{self.color}\")" in text
    assert "self.no_plate_reported = True" in text
    assert "create_publisher(String, '~/place_status'" not in text


def test_openclaw_object_placement_does_not_report_missing_after_place_starts():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_placement.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.placing = False" in text
    assert "self.placing = True" in text
    assert "if target_list and self.running and not self.placing:" in text
    assert "if self.color and self.start_count and not self.placing and not target_list and not self.no_plate_reported:" in text
    assert "placed_color = self.color" in text
    assert "self.set_place_status(True, f\"placed:{placed_color}\")" in text
    assert "String(data=f\"{self.color} is no found\")" not in text


def test_openclaw_object_sorting_polls_place_status_service():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")

    assert "self.place_status_client = self.create_client(Trigger, '/object_placement/place_status')" in text
    assert "self.get_place_sub = self.create_subscription(String, 'object_placement/place_status'" not in text
    assert "def query_place_status(self):" in text
    assert "def wait_for_place_status(self, requested_color):" in text
    assert "response.success" in text
    assert "status.startswith(\"placed:\")" in text
    assert "status.startswith(\"found:\")" in text
    assert "status.startswith(\"not_found:\")" in text
    assert "place target not found" in text


def test_openclaw_object_sorting_waits_in_place_when_plate_not_found():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/object_sorting.py"
    )
    text = node_file.read_text(encoding="utf-8")
    not_found_branch = text.split('elif status.startswith("not_found:"):', 1)[1].split(
        'elif status.startswith("found:"):', 1
    )[0]

    assert "waiting" in not_found_branch
    assert "self.go_home(False)" not in not_found_branch
    assert "self.enter = False" in not_found_branch


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
    assert (
        "result = self.calculate_pick_grasp_yaw(position, place_target, target_info, intrinsic, projection_matrix)"
        in text
    )
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


def test_openclaw_object_transport_ignores_targets_without_pick_yaw():
    node_file = Path(
        "/home/ubuntu/ros2_ws/src/openclaw_controller/openclaw_controller/openclaw_object_transport.py"
    )
    text = node_file.read_text(encoding="utf-8")
    main_loop = text.split("    def main(self):", 1)[1].split(
        "    def enter_srv_callback(self, request, response):", 1
    )[0]

    assert "if self.last_position is not None and self.target is not None and result is not None:" in main_loop
    guarded_block = main_loop.split(
        "if self.last_position is not None and self.target is not None and result is not None:", 1
    )[1].split("self.last_position = position", 1)[0]
    assert "yaw = utils.normalize_gripper_roll_deg(result[0])" in guarded_block
