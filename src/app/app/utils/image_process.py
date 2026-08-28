import cv2
import math
import numpy as np

range_rgb = {
    'red': (255, 0, 50),
    'blue': (0, 50, 255),
    'green': (0, 255, 50),
    'black': (0, 0, 0),
    'white': (255, 255, 255)
}

class CropLayer(object):
    def __init__(self, params, blobs):
        self.startX = 0
        self.startY = 0
        self.endX = 0
        self.endY = 0

    def getMemoryShapes(self, inputs):
        (inputShape, targetShape) = (inputs[0], inputs[1])
        (batchSize, numChannels) = (inputShape[0], inputShape[1])
        (H, W) = (targetShape[2], targetShape[3])

        self.startX = int((inputShape[3] - targetShape[3]) / 2)
        self.startY = int((inputShape[2] - targetShape[2]) / 2)
        self.endX = self.startX + W
        self.endY = self.startY + H

        return [[batchSize, numChannels, H, W]]

    def forward(self, inputs):
        return [inputs[0][:, :, self.startY:self.endY,
                          self.startX:self.endX]]

class Colors:
    def __init__(self):
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    import random
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

def show_faces(detect_img, result_img, boxes, landmarks, bbox_color=(0, 255, 0), ll_color=(0, 0, 255)):
    detect_size = detect_img.shape[:2]
    show_size = result_img.shape[:2]
    for bb, ll in zip(boxes, landmarks):
        p1 = point_remapped(bb[:2], detect_size, show_size, data_type=int)
        p2 = point_remapped(bb[2:4], detect_size, show_size, data_type=int)
        cv2.rectangle(result_img, p1, p2, bbox_color, 2)
        for i, p in enumerate(ll):
            x, y = point_remapped(p, detect_size, show_size, data_type=int)
            cv2.circle(result_img, (x, y), 2, ll_color, 2)
    return result_img

def mp_face_location(results, img):
    h, w, c, = img.shape
    boxes = []
    keypoints = []
    if results.detections:
        for detection in results.detections:
            x_min = detection.location_data.relative_bounding_box.xmin
            y_min = detection.location_data.relative_bounding_box.ymin
            width = detection.location_data.relative_bounding_box.width
            height = detection.location_data.relative_bounding_box.height
            x_min, y_min = max(x_min * w, 0), max(y_min * h, 0)
            x_max, y_max = min(x_min + width * w, w), min(y_min + height * h, h)
            boxes.append((x_min, y_min, x_max, y_max))
            relative_keypoints = detection.location_data.relative_keypoints
            keypoints.append([(point.x * w, point.y * h) for point in relative_keypoints])
    return boxes, keypoints

def draw_tags(image, tags, corners_color=(0, 125, 255), center_color=(0, 255, 0)):
    for tag in tags:
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        cv2.putText(image, "%d"%tag.tag_id, (int(center[0] - (7 * len("%d"%tag.tag_id))), int(center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if corners_color is not None:
            for p in corners:
                cv2.circle(image, tuple(p.tolist()), 5, corners_color, -1)
        if center_color is not None:
            cv2.circle(image, tuple(center.tolist()), 8, center_color, -1)
    return image

def get_area_max_contour(contours, threshold=50):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > threshold:
                area_max_contour = c
    return area_max_contour,  contour_area_max

def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])

class GetObjectSurface:
    def __init__(self, canny_threshold1=100, canny_threshold2=200):
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2

    def adaptive_threshold(self, gray_image):
        return cv2.adaptiveThreshold(
            gray_image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            15, 3
        )

    def canny_proc(self, bgr_image):
        mean_val = np.mean(bgr_image)
        if mean_val < 60:
            low_thr = 15
        elif mean_val < 100:
            low_thr = 20
        else:
            low_thr = 25
        high_thr = low_thr * 3
        mask = cv2.Canny(bgr_image, low_thr, high_thr, apertureSize=3, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        return mask

    def gamma_correction(self, image, gamma=1.5):
        lookUpTable = np.array([((i / 255.0) ** (1.0 / gamma)) * 255
                                for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(image, lookUpTable)

    def get_top_surface(self, rgb_image):
        image_gray_check = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        mean_val = np.mean(image_gray_check)
        if mean_val < 60:
            gamma_val = 1.8
        elif mean_val < 100:
            gamma_val = 1.5
        else:
            gamma_val = 1.2

        bright_img = self.gamma_correction(rgb_image, gamma=gamma_val)
        image_scale = cv2.convertScaleAbs(bright_img, alpha=1.3, beta=10)

        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        image_gray = clahe.apply(image_gray)
        image_mb = cv2.medianBlur(image_gray, 3)
        image_gs = cv2.GaussianBlur(image_mb, (3, 3), 3)

        binary = self.adaptive_threshold(image_gs)
        mask = self.canny_proc(image_gs)

        mask1 = cv2.bitwise_and(binary, mask)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)
        return roi_image_mask
