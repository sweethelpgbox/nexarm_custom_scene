from example.rgbd_function.include.grasp_height import side_grasp_z_from_top


def test_side_grasp_z_uses_mid_height_for_tall_object():
    assert side_grasp_z_from_top(0.06, object_height_m=0.05) == 0.035


def test_side_grasp_z_keeps_low_object_above_table():
    assert side_grasp_z_from_top(0.012, object_height_m=0.012) == 0.008
