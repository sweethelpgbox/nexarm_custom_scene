#!/usr/bin/python3
# coding=utf8
# @Author: Aiden
# @Date: 2024/12/31
import cv2
import copy
import math
import os
import numpy as np
from sdk import common
from typing import Tuple, List, Optional

# 新夹爪线性参数
# 闭合: 角度 30° -> 间距 0mm
# 张开: 角度 -60° -> 间距 51mm
# 线性关系: 夹爪间距每变化 0.5667mm，舵机转动 1度
GRIPPER_MAX_OPEN_MM = 51.0
GRIPPER_MIN_OPEN_MM = 0.0
GRIPPER_ANGLE_CLOSE = 30.0
GRIPPER_ANGLE_OPEN = -60.0
GRIPPER_MM_PER_DEG = GRIPPER_MAX_OPEN_MM / (GRIPPER_ANGLE_CLOSE - GRIPPER_ANGLE_OPEN)  # ≈0.5667


def gripper_mm_to_angle(x_mm: float) -> float:
    x_mm = max(GRIPPER_MIN_OPEN_MM, min(GRIPPER_MAX_OPEN_MM, x_mm))
    return GRIPPER_ANGLE_CLOSE - (x_mm / GRIPPER_MM_PER_DEG)


def gripper_angle_to_mm(angle: float) -> float:
    return (GRIPPER_ANGLE_CLOSE - angle) * GRIPPER_MM_PER_DEG


def get_gripper_size(angle: float, angle_zero=0) -> Tuple[float, float]:
    width_mm = gripper_angle_to_mm(angle)
    width_m = width_mm / 1000.0
    depth_m = 0.015
    return width_m, depth_m


def set_gripper_size(width: float) -> float:
    width_mm = width * 1000.0
    return gripper_mm_to_angle(width_mm)


def jetarm_roll_to_driver_roll_deg(angle_deg: float) -> float:
    """将 JetArm 上层夹爪 roll 语义转换为当前底层 Joint5 roll 语义。"""
    return float(-float(angle_deg))


def driver_roll_to_jetarm_roll_deg(angle_deg: float) -> float:
    """将当前底层 Joint5 roll 语义转换回 JetArm 上层夹爪 roll 语义。"""
    return float(-float(angle_deg))


def normalize_gripper_roll_deg(angle_deg: float, limit_deg: float = 120.0) -> float:
    """将两指夹爪等效的 180° 姿态折叠到稳定的 roll 角范围内。"""
    normalized = ((float(angle_deg) + 90.0) % 180.0) - 90.0
    return float(np.clip(normalized, -limit_deg, limit_deg))


def get_long_edge_angle(rect: Tuple[Tuple[float, float], Tuple[float, float], float]) -> float:
    """稳定计算最小外接矩形长边与图像 x 轴的夹角，规避不同 OpenCV 版本角度定义差异。"""
    box = cv2.boxPoints(rect).astype(np.float32)
    longest_vec = None
    longest_len = -1.0
    for i in range(4):
        vec = box[(i + 1) % 4] - box[i]
        length = float(np.linalg.norm(vec))
        if length > longest_len:
            longest_len = length
            longest_vec = vec
    if longest_vec is None:
        return 0.0
    angle_deg = math.degrees(math.atan2(float(longest_vec[1]), float(longest_vec[0])))
    return float(((angle_deg + 90.0) % 180.0) - 90.0)


def world_to_pixels(world_points, K, T):
    """
    Convert world coordinates to pixel coordinates. 将世界坐标点转换为像素坐标
    Args:
        world_points: List of world coordinates. 世界坐标点列表
        K: Camera intrinsic matrix 相机内参矩阵
        T: Extrinsic transformation matrix [R|t] 外参矩阵 [R|t]
    Returns:
        pixel_points: List of corresponding pixel coordinates 像素坐标点列表
    """
    pixel_points = []
    for wp in world_points:
        # Convert world coordinates to homogeneous coordinates. 将世界坐标转换为齐次坐标
        world_homo = np.append(wp, 1).reshape(4, 1)
        # Transform to camera coordinates using the extrinsic matrix. 通过外参矩阵转换到相机坐标系
        camera_point = np.dot(T, world_homo)
        # Project onto the image plane using the intrinsic matrix. 投影到像素平面
        pixel_homo = np.dot(K, camera_point[:3])
        # Normalization 归一化
        pixel = (pixel_homo / pixel_homo[2])[:2].reshape(-1)
        pixel_points.append(pixel)
    return pixel_points

def calculate_pixel_length(world_length, K, T):
    """
    Compute the corresponding pixel length for a given length in world coordinates. 计算世界坐标中的长度在像素坐标中的对应长度
    Args:
        world_length: Length in world space 世界坐标中的长度
        K: Camera intrinsic matrix 相机内参矩阵
        T: Extrinsic transformation matrix 外参矩阵
    Returns:
        pixel_length: Corresponding length in pixel space 像素坐标中的长度
    """
    # Define a starting point and direction. 定义起始点和方向
    start_point = np.array([0, 0, 0])  # starting point 起始点
    direction = np.array([0, 1, 0])  # y-direction y方向

    # Compute the endpoint. 计算终点坐标
    end_point = start_point + direction * world_length
    # Transform both endpoints to pixel coordinates. 转换两个端点到像素坐标
    pixels = world_to_pixels([start_point, end_point], K, T)
    # Calculate Euclidean distance in pixel space. 计算像素距离
    pixel_length = np.linalg.norm(pixels[1] - pixels[0])

    return int(pixel_length)

def get_plane_values(depth_image: np.ndarray, 
                    plane: Tuple[float, float, float, float],
                    intrinsic_matrix: np.ndarray) -> np.ndarray:
    """
    Compute the distance from each depth pixel to a given plane. 计算深度图像中每个点到平面的距离
    
    Args:
        depth_image: Depth image 深度图像
        plane: Plane equation parameters 平面方程参数(a,b,c,d)
        intrinsic_matrix: Camera intrinsic matrix 相机内参矩阵
        
    Returns:
        plane_values: Distance from each pixel to the plane 每个点到平面的距离
    """
    a, b, c, d = plane
    # Extract camera intrinsics 提取相机内参
    fx = intrinsic_matrix[0]
    fy = intrinsic_matrix[4]
    cx = intrinsic_matrix[2]
    cy = intrinsic_matrix[5]
    
    # Image dimensions 图像尺寸
    H, W = depth_image.shape
    
    # Generate pixel coordinate grid 生成像素坐标网格
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    
    # Calculate the camera coordinates 计算相机坐标
    z = depth_image / 1000.0  # Convert units to meters 转换为米
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Evaluate the plane equation values 计算平面方程值
    plane_values = a * x + b * y + c * z + d
    
    return plane_values

def create_roi_mask(
    depth_image: np.ndarray,
    bgr_image: np.ndarray,
    corners: np.ndarray,
    camera_info: object,
    extrinsic: np.ndarray,
    max_height: float,
    max_obj_height: float,
) -> np.ndarray:
    """
    Create a region of interest (ROI) mask. 创建感兴趣区域(ROI)的遮罩
    Args:
        depth_image: Depth image 深度图像
        bgr_image: BGR image BGR图像
        corners: corner coordinates 角点坐标
        camera_info: Camera intrinsic parameters 相机参数
        extrinsic: Camera extrinsic matrix 外参矩阵
        max_height: Maximum threshold height 最大高度
        max_obj_height: Maximum object height 物体最大高度
    Returns:
        mask: ROI mask ROI遮罩
    """
    image_height, image_width = depth_image.shape[:2]
    
    # 分解外参矩阵
    translation_vec = extrinsic[:1]
    rotation_mat = extrinsic[1:]
    corners_copy = np.array(copy.deepcopy(corners), dtype=np.float64).reshape((-1, 3))
    if len(corners_copy) >= 5:
        corner_points = corners_copy[:-1]
        center_point = corners_copy[-1:]
    elif len(corners_copy) >= 4:
        corner_points = corners_copy[:4]
        center_point = np.mean(corner_points, axis=0, keepdims=True)
    else:
        depth_image[depth_image <= 0] = max_height
        return depth_image
    
    # 投影中心点
    center_points, _ = cv2.projectPoints(
        center_point,
        np.array(rotation_mat),
        np.array(translation_vec),
        np.matrix(camera_info.k).reshape(1, -1, 3),
        np.array(camera_info.d)
    )
    center_points = np.int32(center_points).reshape(2)

    # 平面偏移后的外参
    shifted_tvec, shifted_rmat = common.extristric_plane_shift(
        np.array(translation_vec).reshape((3, 1)),
        np.array(rotation_mat),
        max_obj_height
    )
    
    # 投影其他角点
    projected_points, _ = cv2.projectPoints(
        corner_points[:4],
        np.array(shifted_rmat),
        np.array(shifted_tvec),
        np.matrix(camera_info.k).reshape(1, -1, 3),
        np.array(camera_info.d)
    )
    projected_points = np.int32(projected_points).reshape(-1, 2)
    
    # Use the projected board polygon directly. The old fixed offset ROI
    # (x+10, y-40) clips tilted boards and causes depth clicks to miss.
    margin_px = 20
    roi_poly = np.array(projected_points, dtype=np.int32)
    mask = np.zeros_like(depth_image)
    poly_mask = np.zeros(depth_image.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(poly_mask, roi_poly, 255)
    if margin_px > 0:
        kernel_size = margin_px * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        poly_mask = cv2.dilate(poly_mask, kernel, iterations=1)
    mask[poly_mask > 0] = depth_image[poly_mask > 0]

    depth_image[mask == 0] = max_height
    return depth_image

# def create_roi_mask(
#     depth_image: np.ndarray,
#     bgr_image: np.ndarray,
#     corners: np.ndarray,
#     camera_info: object,
#     extrinsic: np.ndarray,
#     max_height: float,
#     max_obj_height: float,
# ) -> np.ndarray:
#     """
#     Create a region of interest (ROI) mask. 创建感兴趣区域(ROI)的遮罩
#     Args:
#         depth_image: Depth image 深度图像
#         bgr_image: BGR image BGR图像
#         corners: corner coordinates 角点坐标
#         camera_info: Camera intrinsic parameters 相机参数
#         extrinsic: Camera extrinsic matrix 外参矩阵
#         max_height: Maximum threshold height 最大高度
#         max_obj_height: Maximum object height 物体最大高度
#     Returns:
#         mask: ROI mask ROI遮罩
#     """
#     image_height, image_width = depth_image.shape[:2]
    
#     # Decompose the extrinsic matrix 分解外参矩阵
#     translation_vec = extrinsic[:1]
#     rotation_mat = extrinsic[1:]
#     corners_copy = copy.deepcopy(corners)
    
#     # Project the central point 投影中心点
#     center_points, _ = cv2.projectPoints(
#         corners_copy[-1:],
#         np.array(rotation_mat),
#         np.array(translation_vec),
#         np.matrix(camera_info.k).reshape(1, -1, 3),
#         np.array(camera_info.d)
#     )
#     center_points = np.int32(center_points).reshape(2)

#     # Compute new extrinsic matrix after applying plane offset 计算平面偏移后的外参
#     shifted_tvec, shifted_rmat = common.extristric_plane_shift(
#         np.array(translation_vec).reshape((3, 1)),
#         np.array(rotation_mat),
#         max_obj_height
#     )
    
#     # Project other ROI corner points 投影其他角点
#     projected_points, _ = cv2.projectPoints(
#         corners_copy[:-1],
#         np.array(shifted_rmat),
#         np.array(shifted_tvec),
#         np.matrix(camera_info.k).reshape(1, -1, 3),
#         np.array(camera_info.d)
#     )
#     projected_points = np.int32(projected_points).reshape(-1, 2)
    
#     # Calculate the bounding box for the ROI 计算ROI边界
#     x_min = max(0, min(projected_points[:, 0]))
#     x_max = min(image_width, max(projected_points[:, 0]))
#     y_min = max(0, min(projected_points[:, 1]))
#     y_max = min(image_height, max(projected_points[:, 1]))
   
#     # Draw the ROI box on the BGR image 在BGR图像上绘制ROI框
#     # cv2.rectangle(bgr_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

#     # Create ROI are 创建ROI区域
#     x, y = x_min + 10, y_min - 40
#     w, h = x_max - x_min, y_max - y_min
    
#     # create mask 创建遮罩
#     mask = np.zeros_like(depth_image)
#     x2 = min(x + w, image_width)
#     y2 = max(y, 0)
#     mask[y2:y+h, x:x2] = depth_image[y2:y+h, x:x2]

#     # Zero out all regions outside the ROI in the depth image 将深度图像中对应的区域外设置为0
#     depth_image[mask == 0] = max_height

#     return depth_image

def find_depth_range(depth_image: np.ndarray, max_distance: float) -> Tuple[float, float]:
    """
    Find the minimum distance in a depth image. 查找深度图像中的最小
    Args:
        depth_image: Depth image 深度图像
    Returns:
        min_distance: Minimum distance in millimeters. 最小距离(mm)
    """
    height, width = depth_image.shape[:2]
    
    # Process depth data 处理深度数据
    depth = np.copy(depth_image).reshape(-1)
    depth[depth <= 0] = max_distance  # Set invalid values to max_distance
    
    # Find the closest point 找到最近点
    min_idx = np.argmin(depth)
    min_y, min_x = min_idx // width, min_idx % width
    min_distance = depth_image[min_y, min_x] 
    
    return min_distance

def extract_contours(
    plane_values: np.ndarray,
    filter_height: float
) -> List[np.ndarray]:
    """
    Extract contours from a depth image. 提取深度图像中的轮廓
    Args:
        plane_values: Plane value 平面值
        filter_height: Height threshold for filtering 过滤高度
    Returns:
        contours: List of extracted contours 轮廓列表
    """
    # Apply height threshold 过滤高度
    filtered_image = np.where(plane_values <= filter_height, 0, 255).astype(np.uint8)
    
    # Perform binarization and contour extraction 二值化和轮廓提取
    _, binary = cv2.threshold(filtered_image, 1, 255, cv2.THRESH_BINARY)
    # cv2.imshow(color, binary)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    return contours

def convert_depth_to_camera_coords(
    pixel_coords: Tuple[float, float, float],
    intrinsic_matrix: np.ndarray
) -> np.ndarray:
    """
    Convert depth pixel coordinates to camera coordinate system. 将深度像素坐标转换为相机坐标系
    Args:
        pixel_coords: Pixel coordinates 像素坐标 (x, y, z)
        intrinsic_matrix: Camera intrinsic matrix 相机内参矩阵
    Returns:
        camera_coords: Coordinates in the camera coordinate system 相机坐标系下的坐标
    """
    fx, fy = intrinsic_matrix[0], intrinsic_matrix[4]
    cx, cy = intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    
    return np.array([x, y, z])

def calculate_world_position(
    pixel_x: float,
    pixel_y: float,
    depth: float,
    plane: Tuple[float, float, float, float],
    endpoint: np.ndarray,
    hand2cam_tf_matrix: np.ndarray,
    intrinsic_matrix: np.ndarray
) -> np.ndarray:
    """
    Compute the position in the world coordinate system. 计算世界坐标系中的位置
    """
    camera_position = convert_depth_to_camera_coords(
        [pixel_x, pixel_y, depth / 1000],
        intrinsic_matrix
    )
    pose_end = np.matmul(
        hand2cam_tf_matrix,
        common.xyz_euler_to_mat(camera_position, (0, 0, 0))
    )
    if endpoint is None:
        endpoint = np.eye(4)
    world_position = (endpoint + pose_end)[:3, 3]
    world_position = [world_position[-1], world_position[1], world_position[0]]

    a, b, c, d = plane
    world_position[2] = (
        camera_position[0] * a +
        camera_position[1] * b +
        camera_position[2] * c + d
    )

    return world_position
