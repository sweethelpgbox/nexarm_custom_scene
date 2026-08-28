from openvino.runtime import Core
import numpy as np
import cv2
import time
import os

MODEL_NAME = "yolov8s"
CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
    "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
]

# 颜色映射
np.random.seed(42)
colors = np.random.randint(0, 255, size=(len(CLASSES), 3), dtype="uint8")

def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = f'{CLASSES[class_id]} ({confidence:.2f})'
    color = [int(c) for c in colors[class_id]]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def infer_frame(frame):
    [height, width, _] = frame.shape
    length = max(height, width)
    image = np.zeros((length, length, 3), np.uint8)
    image[0:height, 0:width] = frame
    scale = length / 640

    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    outputs = ir.infer(blob)[output_node]
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]

    boxes, scores, class_ids = [], [], []
    for i in range(rows):
        classes_scores = outputs[0][i][4:]
        _, maxScore, _, (x, maxClassIndex) = cv2.minMaxLoc(classes_scores)
        if maxScore >= 0.25:
            box = [
                outputs[0][i][0] - 0.5 * outputs[0][i][2],
                outputs[0][i][1] - 0.5 * outputs[0][i][3],
                outputs[0][i][2],
                outputs[0][i][3]
            ]
            boxes.append(box)
            scores.append(maxScore)
            class_ids.append(maxClassIndex)

    result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45)

    for i in range(len(result_boxes)):
        index = result_boxes[i][0] if isinstance(result_boxes[i], (list, tuple, np.ndarray)) else result_boxes[i]
        box = boxes[index]
        draw_bounding_box(frame, class_ids[index], scores[index],
                          round(box[0] * scale), round(box[1] * scale),
                          round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))
    return frame

# OpenVINO初始化
core = Core()
net = core.compile_model(f"{MODEL_NAME}.xml", device_name="AUTO")
output_node = net.outputs[0]
ir = net.create_infer_request()

# -------------------- 图片识别部分 --------------------
image_path = "mouse.png"  # 你可以换成任意图片路径
if os.path.isfile(image_path):
    frame = cv2.imread(image_path)
    start = time.time()
    result = infer_frame(frame)
    end = time.time()
    print(f"Inference Time: {(end - start):.2f}s")
    cv2.imshow("YOLOv8 OpenVINO - Image", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print(f"Image file '{image_path}' not found.")

# -------------------- 视频识别部分 --------------------
# cap = cv2.VideoCapture("store-aisle-detection.mp4")
# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         break
#     start = time.time()
#     result = infer_frame(frame)
#     end = time.time()
#     fps_label = "Throughput: %.2f FPS" % (1 / (end - start))
#     cv2.putText(result, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
#     cv2.imshow("YOLOv8 OpenVINO - Video", result)
#     if cv2.waitKey(1) > -1:
#         break

# cap.release()
# cv2.destroyAllWindows()
