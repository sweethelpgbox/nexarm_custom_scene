from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "example"
    / "motor"
    / "waste_classification_motor_depth.py"
)


def test_scene5_compressed_preview_uses_best_effort_qos():
    text = SOURCE.read_text(encoding="utf-8")

    assert "SCENE5_PREVIEW_QOS = QoSProfile(" in text
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in text
    assert "history=HistoryPolicy.KEEP_LAST" in text
    assert "depth=1" in text
    start = text.index("self.result_image_compressed_pub = self.create_publisher(")
    block = text[start:text.index("self.bridge = CvBridge()", start)]
    assert "CompressedImage" in block
    assert "'~/result_image/compressed'" in block
    assert "SCENE5_PREVIEW_QOS" in block
