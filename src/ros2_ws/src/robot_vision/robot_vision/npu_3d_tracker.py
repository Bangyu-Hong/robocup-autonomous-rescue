"""
===============================================================================
                    YOLO + 颜色 + 形状 三合一增强版乒乓球3D追踪器
===============================================================================
版本: 5.1 (深度对齐修复版)
功能:
- 支持多球同时检测，返回最近的一个球的3D坐标
- 三重验证: YOLO目标检测 + HSV颜色过滤 + 圆形度验证
- 球面几何动态补偿，亚毫米级精度
- 深度图预处理滤波，减少噪声干扰
- 🆕 深度图黑边填充对齐 (解决640×480 vs 640×400分辨率不匹配问题)
- 多帧时间滤波，平滑输出坐标

坐标系说明:
- 输出坐标基于 camera_color_optical_frame (相机光学坐标系)
- X轴: 向右为正 (单位: 米)
- Y轴: 向下为正 (单位: 米)
- Z轴: 向前为正/深度方向 (单位: 米)
- 可通过 TF2 转换到机器人基座标系 (base_link)

作者: 哈雷酱 (傲娇大小姐工程师)
===============================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped  # 🆕 带时间戳和坐标系的点
from cv_bridge import CvBridge
import cv2
from rknnlite.api import RKNNLite
import numpy as np
import message_filters
from collections import deque


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║                                                                           ║
# ║                    ⚙️ 用户可调参数区 (所有开关和参数)                       ║
# ║                                                                           ║
# ║  修改后保存文件，重启节点即可生效                                            ║
# ║                                                                           ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          🎛️ 功能开关区                                     │
# └───────────────────────────────────────────────────────────────────────────┘

# --- 深度处理开关 ---
ENABLE_DEPTH_PREFILTER = True       # 深度图预处理滤波 (去除噪声)
ENABLE_DEPTH_JUMP_FILTER = True     # 深度跳变抑制 (防止突变)
ENABLE_SPHERICAL_COMPENSATION = True # 🆕 球面几何动态补偿 (科学家模式)

# --- 可视化开关 ---
ENABLE_VISUALIZATION = True         # 主窗口 (显示检测结果)
ENABLE_DEBUG_WINDOWS = True         # 调试窗口 (颜色掩码+深度图)

# --- 图像翻转开关 ---
FLIP_DEPTH_HORIZONTAL = True       # 水平翻转深度图 (左右相反时开启)
FLIP_COLOR_HORIZONTAL = False       # 水平翻转彩色图

# --- 深度图对齐开关 ---
# 🆕 v5.1: 解决 RGB(640×480) 和 Depth(640×400) 分辨率不匹配的问题
# 深度图和彩色图的视场角相同，但深度图上下各少40像素
# 开启此选项会给深度图加黑边，使其与彩色图尺寸一致
ENABLE_DEPTH_PADDING = True         # 深度图黑边填充对齐

# --- 坐标发布开关 ---
PUBLISH_WITH_TIMESTAMP = True       # 🆕 发布带时间戳的坐标 (用于TF转换)

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          📐 物理参数区                                     │
# └───────────────────────────────────────────────────────────────────────────┘

# --- 乒乓球物理参数 ---
BALL_DIAMETER_M = 0.040             # 乒乓球直径 (米) - 国际标准40mm
BALL_RADIUS_M = 0.020               # 乒乓球半径 (米) - 20mm

# --- 深度有效范围 ---
DEPTH_MIN_M = 0.10                  # 最小有效深度 (米)
DEPTH_MAX_M = 2.50                  # 最大有效深度 (米)
DEPTH_SCALE = 1000.0                # 深度单位转换 (Gemini输出毫米)

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          🎨 颜色检测参数区                                  │
# └───────────────────────────────────────────────────────────────────────────┘

# --- 当前球颜色 ---
BALL_COLOR = 'orange'               # 可选: 'orange', 'white', 'yellow', 'auto'

# --- 橙色乒乓球 HSV 范围 ---
ORANGE_H_MIN, ORANGE_H_MAX = 5, 25          # 色相 (0-180)
ORANGE_S_MIN, ORANGE_S_MAX = 100, 255       # 饱和度 (0-255)
ORANGE_V_MIN, ORANGE_V_MAX = 100, 255       # 亮度 (0-255)

# --- 白色乒乓球 HSV 范围 ---
WHITE_H_MIN, WHITE_H_MAX = 0, 180           # 白色不限色相
WHITE_S_MIN, WHITE_S_MAX = 0, 40            # 低饱和度
WHITE_V_MIN, WHITE_V_MAX = 180, 255         # 高亮度

# --- 黄色乒乓球 HSV 范围 ---
YELLOW_H_MIN, YELLOW_H_MAX = 20, 35
YELLOW_S_MIN, YELLOW_S_MAX = 100, 255
YELLOW_V_MIN, YELLOW_V_MAX = 100, 255

# --- 颜色检测阈值 ---
COLOR_MIN_PIXELS = 100              # 最小颜色像素数 (过滤噪点)

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          🔍 检测算法参数区                                  │
# └───────────────────────────────────────────────────────────────────────────┘

# --- YOLO 模型配置 ---
YOLO_MODEL_PATH = '/userdata/yolo_tracking/2.rknn'  # RKNN模型路径
YOLO_INPUT_SIZE = 640               # 模型输入尺寸
YOLO_CONF_THRESHOLD = 0.4           # 置信度阈值 (0.0-1.0)
YOLO_IOU_THRESHOLD = 0.45           # NMS IoU阈值 (0.0-1.0)
CLASSES = ["table_tennis_ball"]     # 类别名称

# --- 圆形检测参数 ---
CIRCULARITY_THRESHOLD = 0.6         # 圆度阈值 (0.0-1.0)
ROI_MARGIN_RATIO = 0.2              # ROI扩展比例

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          🔧 滤波器参数区                                    │
# └───────────────────────────────────────────────────────────────────────────┘

# --- 深度图预处理 ---
DEPTH_FILTER_TYPE = 'median'        # 滤波类型: 'median' 或 'bilateral'
DEPTH_FILTER_SIZE = 3               # 滤波核大小 (奇数: 3, 5, 7...)

# --- 坐标时间滤波 ---
FILTER_WINDOW_SIZE = 5              # 滤波窗口 (帧数)
FRAMES_TO_RESET = 10                # 丢失多少帧后重置滤波器

# --- 深度跳变抑制 ---
DEPTH_JUMP_THRESHOLD = 0.3          # 允许的最大跳变 (米)

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          📷 相机参数区                                      │
# └───────────────────────────────────────────────────────────────────────────┘

# --- 相机内参 (会被 camera_info 话题自动覆盖) ---
CAMERA_FX = 454.05139               # 焦距 X (像素)
CAMERA_FY = 454.05139               # 焦距 Y (像素)
CAMERA_CX = 327.56286               # 光心 X (像素)
CAMERA_CY = 244.89233               # 光心 Y (像素)

# --- 坐标系名称 ---
CAMERA_FRAME_ID = 'camera_color_optical_frame'  # 输出坐标的参考坐标系

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          📡 ROS 话题配置                                   │
# └───────────────────────────────────────────────────────────────────────────┘

TOPIC_COLOR_IMAGE = '/camera/color/image_raw'
TOPIC_DEPTH_IMAGE = '/camera/depth/image_raw'
TOPIC_CAMERA_INFO = '/camera/color/camera_info'
TOPIC_BALL_POSITION = '/target_ball_position'           # geometry_msgs/Point
TOPIC_BALL_POSITION_STAMPED = '/target_ball_position_stamped'  # 🆕 带时间戳

# ┌───────────────────────────────────────────────────────────────────────────┐
# │                          🖥️ 可视化配置                                     │
# └───────────────────────────────────────────────────────────────────────────┘

WINDOW_NAME = "Ball 3D Tracker"
DEBUG_WINDOW_COLOR = "DEBUG: Color Mask"
DEBUG_WINDOW_DEPTH = "DEBUG: Depth Map"

# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║                         参数区结束                                         ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


# 预计算颜色阈值数组
ORANGE_LOWER = np.array([ORANGE_H_MIN, ORANGE_S_MIN, ORANGE_V_MIN])
ORANGE_UPPER = np.array([ORANGE_H_MAX, ORANGE_S_MAX, ORANGE_V_MAX])
WHITE_LOWER = np.array([WHITE_H_MIN, WHITE_S_MIN, WHITE_V_MIN])
WHITE_UPPER = np.array([WHITE_H_MAX, WHITE_S_MAX, WHITE_V_MAX])
YELLOW_LOWER = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN])
YELLOW_UPPER = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX])


def filter_boxes_yolov8(inputs, conf_thres=YOLO_CONF_THRESHOLD, iou_thres=YOLO_IOU_THRESHOLD):
    """YOLOv8/v11 后处理函数"""
    out = inputs[0]
    if len(out.shape) == 3:
        out = out[0]
    if out.shape[0] < out.shape[1]:
        out = out.transpose(1, 0)

    boxes, scores, classes = [], [], []
    box_data = out[:, :4]
    class_scores = out[:, 4:]
    max_scores = np.max(class_scores, axis=1)
    class_ids = np.argmax(class_scores, axis=1)
    valid_idx = max_scores > conf_thres
    valid_boxes = box_data[valid_idx]
    valid_scores = max_scores[valid_idx]
    valid_ids = class_ids[valid_idx]

    for i in range(len(valid_boxes)):
        cx, cy, w, h = valid_boxes[i]
        boxes.append([cx - w/2, cy - h/2, cx + w/2, cy + h/2])
        scores.append(valid_scores[i])
        classes.append(valid_ids[i])

    if len(boxes) == 0:
        return [], [], []

    boxes, scores, classes = np.array(boxes), np.array(scores), np.array(classes)
    keep = []
    for c in np.unique(classes):
        idx = np.where(classes == c)[0]
        c_boxes, c_scores = boxes[idx], scores[idx]
        x1, y1, x2, y2 = c_boxes[:,0], c_boxes[:,1], c_boxes[:,2], c_boxes[:,3]
        areas = (x2-x1) * (y2-y1)
        order = c_scores.argsort()[::-1]
        while order.size > 0:
            i = order[0]
            keep.append(idx[i])
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2-xx1) * np.maximum(0, yy2-yy1)
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(ovr <= iou_thres)[0] + 1]

    return boxes[keep], classes[keep], scores[keep]


class CoordinateFilter:
    """多帧坐标滤波器 (带跳变抑制)"""

    def __init__(self, window_size=FILTER_WINDOW_SIZE):
        self.window_size = window_size
        self.x_buffer = deque(maxlen=window_size)
        self.y_buffer = deque(maxlen=window_size)
        self.z_buffer = deque(maxlen=window_size)

    def update(self, x, y, z):
        """添加新坐标并返回滤波后的结果"""
        # 深度跳变抑制
        if ENABLE_DEPTH_JUMP_FILTER and len(self.z_buffer) >= 3:
            median_z = np.median(list(self.z_buffer))
            if abs(z - median_z) > DEPTH_JUMP_THRESHOLD:
                z = median_z  # 使用历史中值

        self.x_buffer.append(x)
        self.y_buffer.append(y)
        self.z_buffer.append(z)

        return (np.median(list(self.x_buffer)),
                np.median(list(self.y_buffer)),
                np.median(list(self.z_buffer)))

    def reset(self):
        """清空缓冲区"""
        self.x_buffer.clear()
        self.y_buffer.clear()
        self.z_buffer.clear()


class NPU3DTracker(Node):
    """
    基于NPU加速的3D乒乓球追踪器

    输出坐标系: camera_color_optical_frame
    - X: 向右为正 (米)
    - Y: 向下为正 (米)
    - Z: 向前为正 (米) - 深度方向

    可通过 TF2 转换到 base_link (机器人基座标系)
    """

    def __init__(self):
        super().__init__('npu_3d_tracker')
        self.bridge = CvBridge()

        # 相机内参
        self.fx = CAMERA_FX
        self.fy = CAMERA_FY
        self.cx = CAMERA_CX
        self.cy = CAMERA_CY
        self.camera_info_received = False

        # 发布者
        from geometry_msgs.msg import Point
        self.target_pub = self.create_publisher(Point, TOPIC_BALL_POSITION, 10)

        if PUBLISH_WITH_TIMESTAMP:
            self.target_stamped_pub = self.create_publisher(
                PointStamped, TOPIC_BALL_POSITION_STAMPED, 10)

        # 加载YOLO模型
        self.rknn = RKNNLite()
        self.get_logger().info(f'🚀 加载模型: {YOLO_MODEL_PATH}')
        if self.rknn.load_rknn(YOLO_MODEL_PATH) != 0:
            self.get_logger().error('❌ 模型加载失败！')
            return
        if self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
            self.get_logger().error('❌ NPU 初始化失败！')
            return

        # 坐标滤波器
        self.coord_filter = CoordinateFilter()
        self.frames_without_detection = 0

        # 订阅话题
        self.info_sub = self.create_subscription(
            CameraInfo, TOPIC_CAMERA_INFO, self.camera_info_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, TOPIC_COLOR_IMAGE)
        depth_sub = message_filters.Subscriber(self, Image, TOPIC_DEPTH_IMAGE)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        # 打印启动信息
        self.get_logger().info('='*60)
        self.get_logger().info('✅ 乒乓球3D追踪器启动！v5.1 深度对齐修复版')
        self.get_logger().info(f'   球颜色: {BALL_COLOR}')
        self.get_logger().info(f'   最大检测距离: {DEPTH_MAX_M}m')
        self.get_logger().info(f'   球面几何补偿: {"开启" if ENABLE_SPHERICAL_COMPENSATION else "关闭"}')
        self.get_logger().info(f'   深度预处理: {DEPTH_FILTER_TYPE if ENABLE_DEPTH_PREFILTER else "关闭"}')
        self.get_logger().info(f'   🆕 深度图填充对齐: {"开启" if ENABLE_DEPTH_PADDING else "关闭"}')
        self.get_logger().info(f'   输出坐标系: {CAMERA_FRAME_ID}')
        self.get_logger().info('='*60)

    def camera_info_callback(self, msg):
        """接收相机内参"""
        if not self.camera_info_received:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'📷 内参更新: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}')

    def preprocess_depth(self, depth_img):
        """深度图预处理滤波"""
        if not ENABLE_DEPTH_PREFILTER:
            return depth_img

        if DEPTH_FILTER_TYPE == 'median':
            return cv2.medianBlur(depth_img.astype(np.uint16), DEPTH_FILTER_SIZE)
        elif DEPTH_FILTER_TYPE == 'bilateral':
            depth_float = depth_img.astype(np.float32)
            filtered = cv2.bilateralFilter(depth_float, DEPTH_FILTER_SIZE, 50, 50)
            return filtered.astype(depth_img.dtype)

        return depth_img

    def pad_depth_to_color_size(self, depth_img, target_h, target_w):
        """
        🆕 v5.1: 深度图黑边填充对齐

        问题背景:
        - RGB 传感器: 640×480
        - Depth 传感器: 640×400
        - 两者视场角相同，但深度图上下各少40像素

        解决方案:
        - 给深度图上下各填充 (target_h - depth_h) / 2 像素的黑边
        - 这样深度图和彩色图像素就能直接对应

        Args:
            depth_img: 原始深度图 (640×400)
            target_h: 目标高度 (480)
            target_w: 目标宽度 (640)

        Returns:
            填充后的深度图 (640×480)，填充区域值为0
        """
        depth_h, depth_w = depth_img.shape[:2]

        # 如果尺寸已经匹配，直接返回
        if depth_h == target_h and depth_w == target_w:
            return depth_img

        # 计算需要填充的像素数
        pad_top = (target_h - depth_h) // 2
        pad_bottom = target_h - depth_h - pad_top
        pad_left = (target_w - depth_w) // 2
        pad_right = target_w - depth_w - pad_left

        # 用0（无效深度）填充
        padded = cv2.copyMakeBorder(
            depth_img,
            pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT,
            value=0
        )

        return padded

    def refine_with_color(self, roi_bgr, ball_color=BALL_COLOR):
        """颜色检测"""
        if roi_bgr is None or roi_bgr.size == 0:
            return None

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        if ball_color == 'orange':
            mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        elif ball_color == 'white':
            mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
        elif ball_color == 'yellow':
            mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        else:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER),
                cv2.bitwise_or(
                    cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER),
                    cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
                )
            )

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        M = cv2.moments(mask)
        if M["m00"] > COLOR_MIN_PIXELS:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            conf = min(M["m00"] / 5000.0, 1.0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = max(contours, key=cv2.contourArea) if contours else None
            return cx, cy, conf, mask, contour

        return None

    def refine_with_circle(self, roi_bgr, color_mask=None):
        """圆度验证"""
        if roi_bgr is None or roi_bgr.size == 0:
            return None

        if color_mask is not None:
            binary = color_mask
        else:
            gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
            binary = cv2.GaussianBlur(gray, (5, 5), 0)
            binary = cv2.Canny(binary, 50, 150)
            binary = cv2.dilate(binary, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        if area < COLOR_MIN_PIXELS:
            return None

        perimeter = cv2.arcLength(max_contour, True)
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

        if circularity < CIRCULARITY_THRESHOLD:
            return None

        (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
        return int(cx), int(cy), int(radius), circularity, max_contour

    def get_smart_depth(self, depth_img, cx, cy, radius=10):
        """
        智能深度采样 (带球面几何动态补偿)

        球面几何补偿原理:
        - 采样点在球表面，不是球心
        - 采样点深度 = 球心深度 - √(R² - d²)
        - 所以: 球心深度 = 采样点深度 + √(R² - d²)
                        = 采样点深度 + R × √(1 - ratio²)

        其中 ratio = 采样点距球心的比例 (0.3 或 0.6)

        Returns:
            球心到相机的距离 (米)，已补偿到球心位置
        """
        h, w = depth_img.shape[:2]
        compensated_depths = []

        # 双环采样
        for r_ratio, step in [(0.3, 45), (0.6, 30)]:
            r = max(3, int(radius * r_ratio))

            # 🆕 球面几何补偿量: R × √(1 - ratio²)
            if ENABLE_SPHERICAL_COMPENSATION:
                compensation = BALL_RADIUS_M * np.sqrt(1 - r_ratio ** 2)
            else:
                compensation = BALL_RADIUS_M  # 工程模式: 固定补偿

            for angle in range(0, 360, step):
                rad = np.radians(angle)
                sx, sy = int(cx + r * np.cos(rad)), int(cy + r * np.sin(rad))
                if 0 <= sx < w and 0 <= sy < h:
                    d = depth_img[sy, sx]
                    if d > 0:
                        # 采样深度(米) + 动态补偿 = 球心深度
                        depth_m = d / DEPTH_SCALE + compensation
                        compensated_depths.append(depth_m)

        if len(compensated_depths) >= 3:
            return np.median(compensated_depths)

        # 保底 3×3 (中心采样，补偿完整球半径)
        y_min, y_max = max(0, cy - 1), min(h, cy + 2)
        x_min, x_max = max(0, cx - 1), min(w, cx + 2)
        roi = depth_img[y_min:y_max, x_min:x_max]
        valid = roi[roi > 0]
        if len(valid) > 0:
            return np.median(valid) / DEPTH_SCALE + BALL_RADIUS_M
        return None

    def pixel_to_3d(self, px, py, depth_m):
        """
        像素坐标转3D世界坐标 (针孔相机模型)

        输出坐标系: camera_color_optical_frame
        - X: (px - cx) * Z / fx  向右为正 (米)
        - Y: (py - cy) * Z / fy  向下为正 (米)
        - Z: depth_m             向前为正 (米)

        注意: depth_m 已经是球心深度 (在 get_smart_depth 中补偿过)

        Args:
            px, py: 像素坐标
            depth_m: 球心到相机的距离 (米)

        Returns:
            (x, y, z): 3D坐标 (米)，基于相机光学坐标系
        """
        z = depth_m  # 已在 get_smart_depth 中补偿，这里直接使用
        x = (px - self.cx) * z / self.fx
        y = (py - self.cy) * z / self.fy
        return x, y, z

    def letterbox_resize(self, img, size):
        """等比例缩放并填充"""
        h, w = img.shape[:2]
        scale = min(size / w, size / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (new_w, new_h))
        pad_w, pad_h = (size - new_w) // 2, (size - new_h) // 2
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[pad_h:pad_h+new_h, pad_w:pad_w+new_w] = resized
        return canvas, (scale, pad_w, pad_h)

    def letterbox_to_original(self, box, pad_info, orig_w, orig_h):
        """坐标还原"""
        scale, pad_w, pad_h = pad_info
        x1 = max(0, min(orig_w, (box[0] - pad_w) / scale))
        y1 = max(0, min(orig_h, (box[1] - pad_h) / scale))
        x2 = max(0, min(orig_w, (box[2] - pad_w) / scale))
        y2 = max(0, min(orig_h, (box[3] - pad_h) / scale))
        return x1, y1, x2, y2

    def draw_debug_panel(self, img, info):
        """绘制调试面板"""
        overlay = img.copy()
        cv2.rectangle(overlay, (5, 5), (340, 280), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)

        cv2.putText(img, "=== DEBUG PANEL v5.1 ===", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        y = 50
        for name, key, conf_key in [("YOLO", 'yolo', 'yolo_conf'),
                                     ("Color", 'color', 'color_conf'),
                                     ("Circle", 'circle', 'circularity')]:
            active = info.get(key, False)
            conf = info.get(conf_key, 0)
            color = (0, 255, 0) if active else (0, 0, 255)
            status = "ON " if active else "OFF"
            cv2.putText(img, f"{name}: {status} ({conf:.2f})", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y += 22

        cv2.putText(img, f"Balls Found: {info.get('balls_found', 0)}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        y += 22

        cv2.putText(img, f"Raw Depth:  {info.get('raw_depth', 0):.4f} m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y += 22
        cv2.putText(img, f"Filtered Z: {info.get('filtered_z', 0):.4f} m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y += 22

        cv2.putText(img, f"Pixel: ({info.get('px', 0)}, {info.get('py', 0)})", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
        y += 22

        # 🆕 显示完整3D坐标
        cv2.putText(img, f"X: {info.get('x', 0):+.4f} m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y += 22
        cv2.putText(img, f"Y: {info.get('y', 0):+.4f} m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y += 22
        cv2.putText(img, f"Z: {info.get('filtered_z', 0):.4f} m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y += 22

        fail_reason = info.get('fail_reason', '')
        if fail_reason:
            cv2.putText(img, f"FAIL: {fail_reason}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return img

    def sync_callback(self, color_msg, depth_msg):
        """主处理回调"""
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            if FLIP_COLOR_HORIZONTAL:
                cv_color = cv2.flip(cv_color, 1)
            if FLIP_DEPTH_HORIZONTAL:
                cv_depth = cv2.flip(cv_depth, 1)

            # 深度图预处理滤波
            cv_depth = self.preprocess_depth(cv_depth)

            color_h, color_w = cv_color.shape[:2]
            depth_h_orig, depth_w_orig = cv_depth.shape[:2]

            # 加上这句，看穿相机的谎言！
            self.get_logger().info(f"🔍 真实分辨率揭秘: 彩色 {color_w}x{color_h}, 深度 {depth_w_orig}x{depth_h_orig}")

            # 🆕 v5.1: 深度图黑边填充对齐
            # 如果深度图和彩色图尺寸不同，给深度图加黑边
            if ENABLE_DEPTH_PADDING and (depth_h_orig != color_h or depth_w_orig != color_w):
                cv_depth = self.pad_depth_to_color_size(cv_depth, color_h, color_w)
                # 填充后，深度图坐标和彩色图坐标直接对应，不需要缩放映射！

            # 填充后的深度图尺寸（应该和彩色图一样）
            depth_h, depth_w = cv_depth.shape[:2]

            info = {
                'yolo': False, 'yolo_conf': 0,
                'color': False, 'color_conf': 0,
                'circle': False, 'circularity': 0,
                'raw_depth': 0, 'filtered_z': 0,
                'px': 0, 'py': 0, 'x': 0, 'y': 0,
                'fail_reason': '',
                'balls_found': 0
            }

            debug_mask = np.zeros((color_h, color_w, 3), dtype=np.uint8)

            img_rgb = cv2.cvtColor(cv_color, cv2.COLOR_BGR2RGB)
            img_resized, pad_info = self.letterbox_resize(img_rgb, YOLO_INPUT_SIZE)
            outputs = self.rknn.inference(inputs=[np.expand_dims(img_resized, 0)])

            all_balls = []

            if outputs is not None:
                boxes, classes, scores = filter_boxes_yolov8(outputs)

                if len(boxes) == 0:
                    info['fail_reason'] = 'YOLO: No detection'

                for box, score in zip(boxes, scores):
                    x1, y1, x2, y2 = self.letterbox_to_original(box, pad_info, color_w, color_h)
                    x1, y1, x2, y2 = int(max(0,x1)), int(max(0,y1)), int(min(color_w,x2)), int(min(color_h,y2))

                    if x2 <= x1 or y2 <= y1:
                        continue

                    bw, bh = x2 - x1, y2 - y1
                    mx, my = int(bw * ROI_MARGIN_RATIO), int(bh * ROI_MARGIN_RATIO)
                    rx1, ry1 = max(0, x1 - mx), max(0, y1 - my)
                    rx2, ry2 = min(color_w, x2 + mx), min(color_h, y2 + my)
                    roi = cv_color[ry1:ry2, rx1:rx2]

                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    method = "YOLO"
                    conf = float(score)
                    radius = (bw + bh) / 4
                    color_contour = None
                    ball_fail_reason = ''

                    cr = self.refine_with_color(roi, BALL_COLOR)
                    if cr:
                        lcx, lcy, cc, mask, color_contour = cr

                        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        mask_bgr[:,:,0] = 0
                        mask_bgr[:,:,2] = 0
                        debug_mask[ry1:ry2, rx1:rx2] = mask_bgr

                        circ = self.refine_with_circle(roi, mask)
                        if circ:
                            ccx, ccy, r, cir, _ = circ
                            conf = float(score) * cc * cir
                            cx, cy = rx1 + ccx, ry1 + ccy
                            radius = r
                            method = "YOLO+Color+Circle"
                        else:
                            conf = float(score) * cc
                            cx, cy = rx1 + lcx, ry1 + lcy
                            method = "YOLO+Color"
                            ball_fail_reason = f'Circle: circularity < {CIRCULARITY_THRESHOLD}'
                    else:
                        ball_fail_reason = f'Color: pixels < {COLOR_MIN_PIXELS}'

                    # 🆕 v5.1: 填充对齐后，直接使用彩色图坐标访问深度图
                    # 不再需要比例缩放映射！
                    dcx = int(cx)
                    dcy = int(cy)
                    dr = int(radius)
                    depth_m = self.get_smart_depth(cv_depth, dcx, dcy, dr)

                    if depth_m is None:
                        ball_fail_reason = 'Depth: All samples invalid'
                    elif depth_m < DEPTH_MIN_M:
                        ball_fail_reason = f'Depth: {depth_m:.2f}m < MIN'
                    elif depth_m > DEPTH_MAX_M:
                        ball_fail_reason = f'Depth: {depth_m:.2f}m > MAX'
                    else:
                        all_balls.append({
                            'cx': cx, 'cy': cy, 'radius': radius,
                            'conf': conf, 'method': method,
                            'depth_m': depth_m,
                            'box': (x1, y1, x2, y2),
                            'roi': (rx1, ry1, rx2, ry2),
                            'contour': color_contour,
                            'yolo_conf': float(score),
                            'color_conf': cr[2] if cr else 0,
                            'circularity': circ[3] if cr and circ else 0
                        })

                    if not info['fail_reason'] and ball_fail_reason:
                        info['fail_reason'] = ball_fail_reason

            info['balls_found'] = len(all_balls)

            if all_balls:
                all_balls.sort(key=lambda b: b['depth_m'])
                nearest = all_balls[0]

                self.frames_without_detection = 0

                info['yolo'] = True
                info['yolo_conf'] = nearest['yolo_conf']
                info['color'] = nearest['color_conf'] > 0
                info['color_conf'] = nearest['color_conf']
                info['circle'] = nearest['circularity'] > 0
                info['circularity'] = nearest['circularity']
                info['raw_depth'] = nearest['depth_m']
                info['px'], info['py'] = nearest['cx'], nearest['cy']
                info['fail_reason'] = ''

                # 转换为3D坐标 (单位: 米)
                rx, ry, rz = self.pixel_to_3d(nearest['cx'], nearest['cy'], nearest['depth_m'])
                fx, fy, fz = self.coord_filter.update(rx, ry, rz)
                info['filtered_z'] = fz
                info['x'], info['y'] = fx, fy

                # 发布 geometry_msgs/Point
                from geometry_msgs.msg import Point
                msg = Point()
                msg.x, msg.y, msg.z = fx, fy, fz
                self.target_pub.publish(msg)

                # 🆕 发布带时间戳的坐标 (用于TF转换)
                if PUBLISH_WITH_TIMESTAMP:
                    stamped_msg = PointStamped()
                    stamped_msg.header.stamp = self.get_clock().now().to_msg()
                    stamped_msg.header.frame_id = CAMERA_FRAME_ID
                    stamped_msg.point.x = fx
                    stamped_msg.point.y = fy
                    stamped_msg.point.z = fz
                    self.target_stamped_pub.publish(stamped_msg)

                self.get_logger().info(
                    f'🎯 [{nearest["method"]}] X={fx:+.4f}m Y={fy:+.4f}m Z={fz:.4f}m '
                    f'(找到{len(all_balls)}个球)')

                if ENABLE_VISUALIZATION:
                    for i, ball in enumerate(all_balls):
                        x1, y1, x2, y2 = ball['box']
                        bcx, bcy = int(ball['cx']), int(ball['cy'])

                        if i == 0:
                            if "Circle" in ball['method']:
                                cv2.circle(cv_color, (bcx, bcy), int(ball['radius']), (0, 255, 0), 2)
                            elif "Color" in ball['method']:
                                if ball['contour'] is not None:
                                    contour_shifted = ball['contour'] + np.array([ball['roi'][0], ball['roi'][1]])
                                    cv2.drawContours(cv_color, [contour_shifted], -1, (0, 255, 0), 2)
                            else:
                                cv2.rectangle(cv_color, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            cv2.drawMarker(cv_color, (bcx, bcy), (0, 255, 0), cv2.MARKER_CROSS, 15, 2)
                            cv2.putText(cv_color, f"NEAREST Z:{ball['depth_m']:.3f}m",
                                        (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            cv2.rectangle(cv_color, (x1, y1), (x2, y2), (128, 128, 128), 1)
                            cv2.putText(cv_color, f"Z:{ball['depth_m']:.2f}m",
                                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

            else:
                self.frames_without_detection += 1
                if self.frames_without_detection > FRAMES_TO_RESET:
                    self.coord_filter.reset()

            if ENABLE_VISUALIZATION:
                cv_color = self.draw_debug_panel(cv_color, info)
                cv2.imshow(WINDOW_NAME, cv_color)

                if ENABLE_DEBUG_WINDOWS:
                    cv2.imshow(DEBUG_WINDOW_COLOR, debug_mask)

                    depth_vis = cv_depth.astype(np.float32)
                    depth_vis = np.clip(depth_vis / 3000 * 255, 0, 255).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                    # 🆕 v5.1: 填充对齐后，直接使用彩色图坐标
                    for i, ball in enumerate(all_balls):
                        dcx = int(ball['cx'])
                        dcy = int(ball['cy'])
                        color = (255, 255, 255) if i == 0 else (128, 128, 128)
                        cv2.drawMarker(depth_vis, (dcx, dcy), color, cv2.MARKER_CROSS, 15, 2)

                    cv2.imshow(DEBUG_WINDOW_DEPTH, depth_vis)

                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'❌ 异常: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = NPU3DTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'rknn'):
            node.rknn.release()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
