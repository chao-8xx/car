# -*- coding: utf-8 -*-
"""
01_crop_roi_by_scene_v7.py

用途：
- 一次只处理一个场景文件夹；
- 先找红框、裁 ROI、侧视旋转；
- 成功 ROI 与失败图片分开保存；
- 不划分 train/val/test；
- 不做数据增强；
- 不把 ROI 立即永久压缩成 64x64，只额外保存 64x64 预览。

建议分别运行四次：
1) 直道            SCENE_MODE = "straight"
2) 普通弯道        SCENE_MODE = "normal_curve"
3) 圆环/急弯左侧   SCENE_MODE = "side_left"
4) 圆环/急弯右侧   SCENE_MODE = "side_right"

源目录示例（一次只指向其中一个场景根目录）：
SOURCE_DIR/
  1枪支/
  2炸药/
  3望远镜/
  4急救包/
  5装甲车/
  6急救车/

实际小类名字只要能在 CLASS_MAP 中识别即可。
"""

import csv
import os
import cv2
import shutil
import numpy as np
from pathlib import Path


# ============================ 必改配置 ============================

SOURCE_DIR = r"E:\code\dataset_all\数据集_新圆环_7.13\左"
OUTPUT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\huafen"

# 可选：
# "straight"      直道
# "normal_curve"  普通弯道
# "side_left"     左侧急弯/圆环
# "side_right"    右侧急弯/圆环
SCENE_MODE = "side_left"

CLEAR_SCENE_OUTPUT = False

# 为安全起见，默认不移动、不删除原图。
# 失败原图会复制到 failed_original/ 中。
COPY_FAILED_ORIGINAL = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
OUTPUT_CROP_SIZE = 64
SAVE_DEBUG = True
SAVE_MODEL_INPUT_PREVIEW = True


# ============================ 类别映射 ============================

# 同时支持你当前截图中的“1枪支、2炸药……”命名，
# 也支持之前版本中不同编号顺序。
CLASS_MAP = {
    "1枪支": ("weapon", "gun"),
    "2炸药": ("weapon", "explosive"),
    "2炸弹": ("weapon", "explosive"),
    "2手榴弹": ("weapon", "explosive"),

    "3望远镜": ("supplies", "telescope"),
    "5望远镜": ("supplies", "telescope"),
    "望远镜": ("supplies", "telescope"),

    "4急救包": ("supplies", "medkit"),
    "6急救包": ("supplies", "medkit"),
    "急救包": ("supplies", "medkit"),

    "3装甲车": ("transport", "armored_car"),
    "5装甲车": ("transport", "armored_car"),
    "装甲车": ("transport", "armored_car"),

    "4急救车": ("transport", "ambulance"),
    "6急救车": ("transport", "ambulance"),
    "急救车": ("transport", "ambulance"),
    "救护车": ("transport", "ambulance"),

    "枪支": ("weapon", "gun"),
    "炸药": ("weapon", "explosive"),
    "炸弹": ("weapon", "explosive"),
    "手榴弹": ("weapon", "explosive"),
}


# ============================ 红框与裁剪参数 ============================

# 这里保留你现有 V3 的 HSV 红框检测范围，避免一次改太多。
MIN_RED_AREA_RATIO = 0.0003
MAX_RED_AREA_RATIO = 0.026

# 普通场景：红条主要横向。
MIN_ASPECT = 1.2
MAX_ASPECT = 6.5

# 侧视场景：红条可能横向或竖向，只看长边/短边。
SIDE_MIN_ELONGATION = 1.35
SIDE_MAX_ELONGATION = 8.0

MIN_Y_RATIO = 0.30
MAX_Y_RATIO = 0.85
CENTER_X_TOLERANCE = 0.36
SIDE_POSITION_LIMIT = 0.88
ASPECT_TARGET = 3.6

CROP_SIDE_SCALE = 1.05
CROP_BOTTOM_MARGIN_SCALE = 0.12
SIDE_CROP_MARKER_MARGIN_SCALE = 0.08
SIDE_CROP_SIDE_SCALE = 1.20
SIDE_CROP_MIN_SIDE = 18

# 侧视圆环/急弯的附加筛选。普通直道/普通弯道不使用这些参数。
SIDE_MIN_RED_PIXEL_RATIO = 0.08
SIDE_INNER_WHITE_MIN = 0.00
SIDE_VERTICAL_WHITE_MIN = 0.00
SIDE_MIN_RED_AREA_RATIO = 0.00010
SIDE_MAX_RED_AREA_RATIO = 0.040
SIDE_MIN_Y_RATIO = 0.16
SIDE_MAX_Y_RATIO = 0.86
SIDE_MIN_MARKER_LONG = 7
SIDE_MIN_MARKER_SHORT = 2

# 裁剪后的ROI质量过滤：宁可把可疑图放到failed，也不要污染训练集。
ROI_MIN_WHITE_RATIO = 0.10
ROI_MIN_RED_RATIO = 0.004
ROI_MAX_BLUE_RATIO = 0.62
ROI_MAX_SKIN_RATIO = 0.38

LEFT_ROTATE_CODE = cv2.ROTATE_90_CLOCKWISE
RIGHT_ROTATE_CODE = cv2.ROTATE_90_COUNTERCLOCKWISE


# ============================ 场景配置 ============================

VALID_SCENE_MODES = {"straight", "normal_curve", "side_left", "side_right"}

if SCENE_MODE not in VALID_SCENE_MODES:
    raise ValueError(f"SCENE_MODE 无效：{SCENE_MODE}")

SIDE_CONTEXT = SCENE_MODE in {"side_left", "side_right"}
SIDE_DIR = -1 if SCENE_MODE == "side_left" else (1 if SCENE_MODE == "side_right" else 0)


# ============================ 基础工具 ============================

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    idx = 1
    while True:
        candidate = path.with_name(f"{path.stem}_{idx}{path.suffix}")
        if not candidate.exists():
            return candidate
        idx += 1


def imread_cn(path: Path):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception:
        return None


def imwrite_cn(path: Path, img) -> bool:
    ensure_dir(path.parent)
    suffix = path.suffix.lower()
    if suffix not in {".jpg", ".jpeg", ".png", ".bmp"}:
        path = path.with_suffix(".jpg")
        suffix = ".jpg"
    ok, buf = cv2.imencode(suffix, img)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


def copy_failed_original(src: Path, dst: Path):
    if not COPY_FAILED_ORIGINAL:
        return
    ensure_dir(dst.parent)
    shutil.copy2(src, unique_path(dst))


def parse_class_from_path(path: Path):
    # 从离图片最近的父文件夹向上寻找小类名。
    for part in reversed(path.parts[:-1]):
        if part in CLASS_MAP:
            return CLASS_MAP[part]
        for key, value in CLASS_MAP.items():
            if key in part:
                return value
    return None


def safe_output_stem(path: Path, subclass: str) -> str:
    parent = path.parent.name
    grandparent = path.parent.parent.name if path.parent.parent else "root"
    return f"{SCENE_MODE}__{subclass}__{grandparent}_{parent}_{path.stem}"


# ============================ 红框检测 ============================

def _calc_yuv_channels(img_bgr):
    """按车端整数公式近似计算 Y/U/V，便于离线裁剪与车端红框规则对齐。"""
    b = img_bgr[:, :, 0].astype(np.int16)
    g = img_bgr[:, :, 1].astype(np.int16)
    r = img_bgr[:, :, 2].astype(np.int16)

    yy = (77 * r + 150 * g + 29 * b) >> 8
    u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8)
    v = 128 + ((128 * r - 107 * g - 21 * b) >> 8)
    return yy, u, v, b, g, r


def build_red_mask(img_bgr):
    """
    普通场景：保留原 HSV 规则，避免破坏已调好的直道/普通弯道。
    侧视场景：改用车端风格的 YUV + RGB 红色优势，并只保留严格红色色相，
    防止蓝色圆环边界被旧 HSV [125, 180] 范围误当成红色。
    """
    if not SIDE_CONTEXT:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 45, 55], dtype=np.uint8)
        upper_red1 = np.array([55, 255, 255], dtype=np.uint8)
        lower_red2 = np.array([125, 75, 55], dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        return mask

    yy, u, v, b, g, r = _calc_yuv_channels(img_bgr)

    # 宽松YUV/RGB红色优势：用于暗红、模糊、反光后的红条。
    yuv_red = (
        (yy >= 10) &
        (v >= 124) &
        ((v - u) >= 7) &
        (r >= g + 3) &
        (r >= b + 3)
    )

    # 严格红色色相，只保留色环两端，绝不再覆盖蓝紫区域。
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    value = hsv[:, :, 2]
    hsv_red = (
        (((h <= 20) | (h >= 165))) &
        (s >= 30) &
        (value >= 35) &
        (r >= g + 2) &
        (r >= b + 2)
    )

    mask = ((yuv_red | hsv_red).astype(np.uint8) * 255)

    # 小红框在远处只有少量像素，不再做开运算；只轻微闭运算连接断裂。
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    return mask


def build_white_mask(img_bgr):
    """与车端白色赛道判定接近，只供侧视候选周围白色支撑检查。"""
    yy, u, v, _, _, _ = _calc_yuv_channels(img_bgr)
    white = (
        (yy >= 80) &
        (np.abs(u - 128) <= 30) &
        (np.abs(v - 128) <= 38)
    )
    return white.astype(np.uint8) * 255


def clipped_rect(x, y, w, h, img_w, img_h):
    x0 = max(0, int(x))
    y0 = max(0, int(y))
    x1 = min(img_w, int(x + w))
    y1 = min(img_h, int(y + h))
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def mask_ratio(mask, rect):
    if rect is None:
        return 0.0
    x, y, w, h = rect
    roi = mask[y:y + h, x:x + w]
    if roi.size == 0:
        return 0.0
    return float(cv2.countNonZero(roi)) / float(max(1, w * h))


def side_white_support(white_mask, red_box):
    """
    左侧场景：朝画面中心的一侧是红框右边；
    右侧场景：朝画面中心的一侧是红框左边。
    同时要求上/下至少一侧有少量白色支撑。
    """
    img_h, img_w = white_mask.shape[:2]
    x, y, w, h = red_box
    band = max(2, min(10, max(h, w // 4)))

    top_r = clipped_rect(x - band, y - band, w + 2 * band, band, img_w, img_h)
    bottom_r = clipped_rect(x - band, y + h, w + 2 * band, band, img_w, img_h)
    left_r = clipped_rect(x - band, y, band, h, img_w, img_h)
    right_r = clipped_rect(x + w, y, band, h, img_w, img_h)

    top_white = mask_ratio(white_mask, top_r)
    bottom_white = mask_ratio(white_mask, bottom_r)
    left_white = mask_ratio(white_mask, left_r)
    right_white = mask_ratio(white_mask, right_r)

    inner_white = right_white if SIDE_DIR < 0 else left_white
    vertical_white = max(top_white, bottom_white)
    return inner_white, vertical_white, top_white, bottom_white, left_white, right_white


def build_lane_mask(img_bgr):
    h_img, w_img = img_bgr.shape[:2]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    lane_mask = ((gray > 95) & (hsv[:, :, 1] < 95)).astype(np.uint8) * 255

    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    close_width = max(31, w_img // 5)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (close_width, 5))
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    return lane_mask


def find_track_component_mask(img_bgr):
    h_img, w_img = img_bgr.shape[:2]
    lane_mask = build_lane_mask(img_bgr)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(lane_mask, 8)

    if num_labels <= 1:
        return None

    center_x = w_img // 2

    for y in range(int(h_img * 0.88), int(h_img * 0.45), -1):
        label = labels[y, center_x]
        if label > 0:
            return (labels == label).astype(np.uint8) * 255

    bottom_y1 = int(h_img * 0.55)
    bottom_y2 = int(h_img * 0.95)
    center_x1 = int(w_img * 0.25)
    center_x2 = int(w_img * 0.75)

    best_label = 0
    best_score = -1

    for label in range(1, num_labels):
        component = (labels == label)
        area = stats[label, cv2.CC_STAT_AREA]
        bottom_center_pixels = int(
            np.count_nonzero(component[bottom_y1:bottom_y2, center_x1:center_x2])
        )
        score = bottom_center_pixels * 4 + area
        if score > best_score:
            best_score = score
            best_label = label

    if best_label <= 0:
        return None

    return (labels == best_label).astype(np.uint8) * 255


def estimate_lane_bounds(img_bgr, y, search_half_height=8):
    h_img, w_img = img_bgr.shape[:2]
    y = int(np.clip(y, 0, h_img - 1))

    track_mask = find_track_component_mask(img_bgr)
    if track_mask is None:
        return None

    y1 = max(0, y - search_half_height)
    y2 = min(h_img, y + search_half_height + 1)
    roi = track_mask[y1:y2]

    col_count = np.count_nonzero(roi, axis=0)
    valid_cols = col_count >= max(1, roi.shape[0] // 5)

    if not np.any(valid_cols):
        return None

    center_x = w_img // 2

    if valid_cols[center_x]:
        left = center_x
        while left > 0 and valid_cols[left - 1]:
            left -= 1

        right = center_x
        while right < w_img - 1 and valid_cols[right + 1]:
            right += 1

        return left, right

    best = None
    start = None

    for idx, is_valid in enumerate(np.r_[valid_cols, False]):
        if is_valid and start is None:
            start = idx
        elif not is_valid and start is not None:
            end = idx - 1
            if best is None or (end - start) > (best[1] - best[0]):
                best = (start, end)
            start = None

    return best


def is_marker_inside_lane(img_bgr, red_box):
    h_img, w_img = img_bgr.shape[:2]
    x, y, w, h = red_box
    cx = x + w / 2.0
    cy = y + h / 2.0

    bounds = estimate_lane_bounds(img_bgr, cy)
    if bounds is None:
        return abs(cx - w_img / 2.0) / float(w_img) <= CENTER_X_TOLERANCE

    left, right = bounds
    lane_margin = max(5, w_img // 64)
    return left - lane_margin <= cx <= right + lane_margin


def is_side_position_reasonable(cx, w_img):
    if SIDE_DIR < 0:
        return cx <= w_img * SIDE_POSITION_LIMIT
    if SIDE_DIR > 0:
        return cx >= w_img * (1.0 - SIDE_POSITION_LIMIT)
    return True


def find_red_marker(img_bgr):
    h_img, w_img = img_bgr.shape[:2]
    img_area = h_img * w_img
    mask = build_red_mask(img_bgr)
    white_mask = build_white_mask(img_bgr) if SIDE_CONTEXT else None

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        area_ratio = area / float(max(1, img_area))

        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue

        aspect = w / float(h)
        marker_long = max(w, h)
        marker_short = min(w, h)
        elongation = marker_long / float(max(1, marker_short))

        if SIDE_CONTEXT:
            if marker_long < SIDE_MIN_MARKER_LONG or marker_short < SIDE_MIN_MARKER_SHORT:
                continue
            shape_ok = (
                SIDE_MIN_RED_AREA_RATIO <= area_ratio <= SIDE_MAX_RED_AREA_RATIO
                and SIDE_MIN_ELONGATION <= elongation <= SIDE_MAX_ELONGATION
            )
        else:
            shape_ok = (
                MIN_RED_AREA_RATIO <= area_ratio <= MAX_RED_AREA_RATIO
                and MIN_ASPECT <= aspect <= MAX_ASPECT
            )

        if not shape_ok:
            continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        y_ratio = cy / float(h_img)

        if SIDE_CONTEXT:
            if y_ratio < SIDE_MIN_Y_RATIO or y_ratio > SIDE_MAX_Y_RATIO:
                continue
        else:
            if y_ratio < MIN_Y_RATIO or y_ratio > MAX_Y_RATIO:
                continue

        if SIDE_CONTEXT:
            if not is_side_position_reasonable(cx, w_img):
                continue
        else:
            image_center_dist = abs(cx - w_img / 2.0) / float(w_img)
            if image_center_dist > CENTER_X_TOLERANCE:
                continue
            if not is_marker_inside_lane(img_bgr, (x, y, w, h)):
                continue

        rect_area = w * h
        fill_ratio = area / float(max(1, rect_area))
        min_fill_ratio = 0.10 if SIDE_CONTEXT else 0.25
        if fill_ratio < min_fill_ratio:
            continue

        side_red_ratio = fill_ratio
        inner_white = 0.0
        vertical_white = 0.0

        if SIDE_CONTEXT:
            # red mask 本身统计出的红色填充比例，避免只靠轮廓形状选中蓝白边缘。
            red_roi = mask[y:y + h, x:x + w]
            side_red_ratio = float(cv2.countNonZero(red_roi)) / float(max(1, rect_area))
            if side_red_ratio < SIDE_MIN_RED_PIXEL_RATIO:
                continue

            (
                inner_white,
                vertical_white,
                _top_white,
                _bottom_white,
                _left_white,
                _right_white,
            ) = side_white_support(white_mask, (x, y, w, h))

            # V6：白色支撑只用于排序，不再作为硬过滤。
            # 圆环红框常紧贴蓝边，硬卡白色会导致绝大多数真实目标被拒绝。

        if SIDE_CONTEXT:
            target_x = w_img * (0.35 if SIDE_DIR < 0 else 0.65)
            center_score = 1.0 - min(
                abs(cx - target_x) / max(1.0, w_img * 0.5),
                1.0,
            )
            shape_score = 1.0 - min(
                abs(elongation - ASPECT_TARGET) / ASPECT_TARGET,
                1.0,
            )
            # 侧视时优先选择“真正红 + 朝中心一侧有白色 + 上下有白色”的候选。
            center_score = (
                center_score * 0.9
                + min(side_red_ratio / 0.45, 1.0) * 2.4
                + min(inner_white / 0.50, 1.0) * 0.35
                + min(vertical_white / 0.40, 1.0) * 0.25
            )
        else:
            bounds = estimate_lane_bounds(img_bgr, cy)
            if bounds is None:
                lane_center_dist = abs(cx - w_img / 2.0) / float(w_img)
            else:
                left, right = bounds
                track_mid = (left + right) / 2.0
                lane_half_width = max((right - left) / 2.0, 1.0)
                lane_center_dist = min(
                    abs(cx - track_mid) / lane_half_width,
                    1.0,
                )

            center_score = 1.0 - lane_center_dist
            shape_score = 1.0 - min(
                abs(aspect - ASPECT_TARGET) / ASPECT_TARGET,
                1.0,
            )

        lower_score = y_ratio
        area_score = min(
            area / float(max(1.0, img_area * 0.03)),
            1.0,
        )

        score = (
            center_score * 3.0
            + shape_score * 2.0
            + lower_score
            + area_score
        )

        candidates.append((score, (x, y, w, h)))

    if not candidates:
        return None, mask

    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1], mask


# ============================ ROI 裁剪与旋转 ============================

def fit_square_box(x1, y1, side, w_img, h_img):
    side = int(max(1, min(side, w_img, h_img)))
    x1 = int(round(x1))
    y1 = int(round(y1))

    x1 = max(0, min(x1, w_img - side))
    y1 = max(0, min(y1, h_img - side))

    return x1, y1, x1 + side, y1 + side


def crop_target_above_marker(img_bgr, red_box):
    h_img, w_img = img_bgr.shape[:2]
    x, y, w, h = red_box

    side = max(round(w * CROP_SIDE_SCALE), 12)
    cx = x + w / 2.0
    marker_bottom = y + h
    bottom_margin = round(side * CROP_BOTTOM_MARGIN_SCALE)

    crop_y2 = marker_bottom + bottom_margin
    crop_y1 = crop_y2 - side
    crop_x1 = cx - side / 2.0

    x1 = int(round(crop_x1))
    y1 = int(round(crop_y1))
    x2 = x1 + side
    y2 = y1 + side

    crop_box = (x1, y1, x2, y2)

    if x1 < 0 or y1 < 0 or x2 > w_img or y2 > h_img:
        return None, crop_box

    return img_bgr[y1:y2, x1:x2].copy(), crop_box


def crop_target_side_marker(img_bgr, red_box):
    h_img, w_img = img_bgr.shape[:2]
    x, y, w, h = red_box

    marker_long = max(w, h)
    side = max(round(marker_long * SIDE_CROP_SIDE_SCALE), SIDE_CROP_MIN_SIDE)

    center_y = y + h / 2.0
    crop_y1 = center_y - side / 2.0
    marker_margin = round(side * SIDE_CROP_MARKER_MARGIN_SCALE)

    if SIDE_DIR < 0:
        crop_x2 = x + w + marker_margin
        crop_x1 = crop_x2 - side
    else:
        crop_x1 = x - marker_margin

    x1, y1, x2, y2 = fit_square_box(
        crop_x1,
        crop_y1,
        side,
        w_img,
        h_img,
    )

    crop_box = (x1, y1, x2, y2)

    if x2 <= x1 or y2 <= y1:
        return None, crop_box

    return img_bgr[y1:y2, x1:x2].copy(), crop_box


def evaluate_roi_quality(crop_img):
    """
    对最终ROI做一次轻量质量检查：
    - 白色/低饱和区域不能太少；
    - 必须仍含少量真实红色；
    - 不能几乎全是蓝色赛道；
    - 不能被手臂/皮肤区域占满。
    返回：(是否通过, 指标字典)
    """
    if crop_img is None or crop_img.size == 0:
        return False, {
            "white_ratio": 0.0,
            "red_ratio": 0.0,
            "blue_ratio": 1.0,
            "skin_ratio": 1.0,
        }

    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    white_mask = (
        (gray >= 65) &
        (hsv[:, :, 1] <= 120)
    )
    white_ratio = float(np.mean(white_mask))

    red_mask = build_red_mask(crop_img)
    red_ratio = float(cv2.countNonZero(red_mask)) / float(max(1, crop_img.shape[0] * crop_img.shape[1]))

    # OpenCV H范围为0~179，圆环蓝色主要落在约90~140。
    blue_mask = (
        (hsv[:, :, 0] >= 90) &
        (hsv[:, :, 0] <= 140) &
        (hsv[:, :, 1] >= 55)
    )
    blue_ratio = float(np.mean(blue_mask))

    ycrcb = cv2.cvtColor(crop_img, cv2.COLOR_BGR2YCrCb)
    yy = ycrcb[:, :, 0]
    cr = ycrcb[:, :, 1]
    cb = ycrcb[:, :, 2]
    skin_mask = (
        (yy >= 35) &
        (cr >= 133) & (cr <= 180) &
        (cb >= 77) & (cb <= 135)
    )
    skin_ratio = float(np.mean(skin_mask))

    ok = (
        white_ratio >= ROI_MIN_WHITE_RATIO
        and red_ratio >= ROI_MIN_RED_RATIO
        and blue_ratio <= ROI_MAX_BLUE_RATIO
        and skin_ratio <= ROI_MAX_SKIN_RATIO
    )

    return ok, {
        "white_ratio": white_ratio,
        "red_ratio": red_ratio,
        "blue_ratio": blue_ratio,
        "skin_ratio": skin_ratio,
    }


def normalize_crop_by_scene(img_bgr, red_box):
    """
    V7关键修正：
    - 普通场景：仍向红框上方裁；
    - 侧视文件夹中，红框仍是横向(w>=h)时，也向上裁，不再盲目向左/右裁；
    - 只有红框真正竖向(h>w)时，才向对应侧裁并旋转90°。
    """
    x, y, w, h = red_box

    if not SIDE_CONTEXT:
        crop_img, crop_box = crop_target_above_marker(img_bgr, red_box)
        return crop_img, crop_box, "none"

    # 圆环/急弯里红框还保持横向时，目标板仍主要位于红框上方。
    if w >= h:
        crop_img, crop_box = crop_target_above_marker(img_bgr, red_box)
        return crop_img, crop_box, "none"

    # 只有红框已经明显竖起来，才走侧向裁剪和90°旋转。
    crop_img, crop_box = crop_target_side_marker(img_bgr, red_box)
    if crop_img is None:
        return None, crop_box, "none"

    if SIDE_DIR < 0:
        crop_img = cv2.rotate(crop_img, LEFT_ROTATE_CODE)
        rotate_text = "cw90"
    else:
        crop_img = cv2.rotate(crop_img, RIGHT_ROTATE_CODE)
        rotate_text = "ccw90"

    return crop_img, crop_box, rotate_text


# ============================ 调试图 ============================

def draw_debug(
    img_bgr,
    red_box,
    crop_box,
    main_class,
    subclass,
    status,
    rotate_text,
):
    dbg = img_bgr.copy()

    if red_box is not None:
        x, y, w, h = red_box
        cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 0, 255), 2)

    if crop_box is not None:
        x1, y1, x2, y2 = crop_box
        cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)

    text1 = f"{main_class}/{subclass} {SCENE_MODE} {status}"
    text2 = f"rotate={rotate_text}"

    cv2.putText(
        dbg,
        text1,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        dbg,
        text1,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 0, 0),
        1,
    )
    cv2.putText(
        dbg,
        text2,
        (10, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        dbg,
        text2,
        (10, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (0, 0, 0),
        1,
    )

    return dbg


def make_model_input_preview(crop_img, main_class, subclass, rotate_text):
    crop64 = cv2.resize(
        crop_img,
        (OUTPUT_CROP_SIZE, OUTPUT_CROP_SIZE),
        interpolation=cv2.INTER_AREA,
    )

    scale = 3
    preview = cv2.resize(
        crop64,
        (OUTPUT_CROP_SIZE * scale, OUTPUT_CROP_SIZE * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    canvas = np.zeros(
        (preview.shape[0] + 56, preview.shape[1], 3),
        dtype=np.uint8,
    )
    canvas[:preview.shape[0]] = preview

    cv2.putText(
        canvas,
        "MODEL INPUT 64x64",
        (5, preview.shape[0] + 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (0, 255, 0),
        1,
    )
    cv2.putText(
        canvas,
        f"{main_class}/{subclass} {SCENE_MODE} {rotate_text}",
        (5, preview.shape[0] + 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.37,
        (255, 255, 255),
        1,
    )

    return crop64, canvas


# ============================ 单图处理 ============================

def process_one_image(img_path: Path, scene_output: Path, manifest_writer):
    class_info = parse_class_from_path(img_path)
    if class_info is None:
        return False, "unknown_class"

    main_class, subclass = class_info
    img = imread_cn(img_path)

    if img is None:
        return False, "read_failed"

    output_stem = safe_output_stem(img_path, subclass)
    red_box, red_mask = find_red_marker(img)

    if red_box is None:
        reason = "no_red_marker"
        failed_debug = draw_debug(
            img,
            None,
            None,
            main_class,
            subclass,
            reason,
            "none",
        )
        debug_path = unique_path(
            scene_output
            / "failed"
            / reason
            / main_class
            / subclass
            / f"{output_stem}_debug.jpg"
        )
        imwrite_cn(debug_path, failed_debug)

        mask_path = unique_path(
            scene_output / "mask_debug" / "failed" / reason / main_class / subclass / f"{output_stem}_mask.png"
        )
        imwrite_cn(mask_path, red_mask)

        copy_failed_original(
            img_path,
            scene_output
            / "failed_original"
            / reason
            / main_class
            / subclass
            / img_path.name,
        )
        return False, reason

    crop_img, crop_box, rotate_text = normalize_crop_by_scene(
        img,
        red_box,
    )

    if crop_img is None or crop_img.size == 0:
        reason = "crop_failed"
        failed_debug = draw_debug(
            img,
            red_box,
            crop_box,
            main_class,
            subclass,
            reason,
            rotate_text,
        )
        debug_path = unique_path(
            scene_output
            / "failed"
            / reason
            / main_class
            / subclass
            / f"{output_stem}_debug.jpg"
        )
        imwrite_cn(debug_path, failed_debug)

        mask_path = unique_path(
            scene_output / "mask_debug" / "failed" / reason / main_class / subclass / f"{output_stem}_mask.png"
        )
        imwrite_cn(mask_path, red_mask)

        copy_failed_original(
            img_path,
            scene_output
            / "failed_original"
            / reason
            / main_class
            / subclass
            / img_path.name,
        )
        return False, reason

    quality_ok, quality = evaluate_roi_quality(crop_img)
    if not quality_ok:
        reason = "poor_roi_quality"
        failed_debug = draw_debug(
            img,
            red_box,
            crop_box,
            main_class,
            subclass,
            (
                f"{reason} "
                f"W={quality['white_ratio']:.2f} "
                f"R={quality['red_ratio']:.3f} "
                f"B={quality['blue_ratio']:.2f} "
                f"S={quality['skin_ratio']:.2f}"
            ),
            rotate_text,
        )
        debug_path = unique_path(
            scene_output
            / "failed"
            / reason
            / main_class
            / subclass
            / f"{output_stem}_debug.jpg"
        )
        imwrite_cn(debug_path, failed_debug)

        # 保存一张可疑ROI，方便人工确认阈值，不复制整张原图。
        bad_roi_path = unique_path(
            scene_output
            / "failed_roi"
            / reason
            / main_class
            / subclass
            / f"{output_stem}_{rotate_text}.jpg"
        )
        imwrite_cn(bad_roi_path, crop_img)

        mask_path = unique_path(
            scene_output
            / "mask_debug"
            / "failed"
            / reason
            / main_class
            / subclass
            / f"{output_stem}_mask.png"
        )
        imwrite_cn(mask_path, red_mask)
        return False, reason

    # 1) 保存原尺寸 ROI：后续划分、增强都用这个。
    raw_roi_path = unique_path(
        scene_output
        / "roi_raw_success"
        / main_class
        / subclass
        / f"{output_stem}__{rotate_text}.jpg"
    )
    imwrite_cn(raw_roi_path, crop_img)

    # 2) 额外保存 64x64，仅用于检查最终模型输入。
    crop64, preview = make_model_input_preview(
        crop_img,
        main_class,
        subclass,
        rotate_text,
    )

    roi64_path = unique_path(
        scene_output
        / "roi_64_success"
        / main_class
        / subclass
        / f"{output_stem}__{rotate_text}.jpg"
    )
    imwrite_cn(roi64_path, crop64)

    if SAVE_MODEL_INPUT_PREVIEW:
        preview_path = unique_path(
            scene_output
            / "model_input_preview"
            / main_class
            / subclass
            / f"{output_stem}_preview.jpg"
        )
        imwrite_cn(preview_path, preview)

    if SAVE_DEBUG:
        debug_img = draw_debug(
            img,
            red_box,
            crop_box,
            main_class,
            subclass,
            "ok",
            rotate_text,
        )
        debug_path = unique_path(
            scene_output
            / "debug"
            / main_class
            / subclass
            / f"{output_stem}_debug.jpg"
        )
        imwrite_cn(debug_path, debug_img)

    success_mask_path = unique_path(
        scene_output / "mask_debug" / "success" / main_class / subclass / f"{output_stem}_mask.png"
    )
    imwrite_cn(success_mask_path, red_mask)

    manifest_writer.writerow([
        str(img_path),
        str(raw_roi_path),
        str(roi64_path),
        main_class,
        subclass,
        SCENE_MODE,
        SIDE_DIR,
        rotate_text,
        red_box[0],
        red_box[1],
        red_box[2],
        red_box[3],
        crop_box[0],
        crop_box[1],
        crop_box[2],
        crop_box[3],
    ])

    return True, "ok"


# ============================ 主程序 ============================

def main():
    source_dir = Path(SOURCE_DIR)
    output_root = Path(OUTPUT_ROOT)
    scene_output = output_root / SCENE_MODE

    print("=" * 64)
    print("阶段1 V7：按场景检测红框、裁 ROI、侧视旋转")
    print(f"输入目录：{source_dir}")
    print(f"场景模式：{SCENE_MODE}")
    print(f"输出目录：{scene_output}")
    print("本脚本：不划分、不增强；成功 ROI 与失败图分开保存。")
    if SIDE_CONTEXT:
        print("侧视V7：横红框向上裁、竖红框侧向裁并旋转；增加ROI质量过滤。")
    print("=" * 64)

    if not source_dir.exists():
        print(f"❌ 输入目录不存在：{source_dir}")
        return

    if CLEAR_SCENE_OUTPUT and scene_output.exists():
        shutil.rmtree(scene_output)

    ensure_dir(scene_output)

    image_paths = sorted([
        path
        for path in source_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_EXTS
    ])

    if not image_paths:
        print("❌ 没有扫描到图片。")
        return

    manifest_path = scene_output / "manifest.csv"
    ensure_dir(manifest_path.parent)

    ok_count = 0
    fail_count = 0
    reasons = {}
    class_counts = {}

    with manifest_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "source_path",
            "raw_roi_path",
            "roi64_path",
            "main_class",
            "subclass",
            "scene_mode",
            "side_dir",
            "rotate",
            "red_x",
            "red_y",
            "red_w",
            "red_h",
            "crop_x1",
            "crop_y1",
            "crop_x2",
            "crop_y2",
        ])

        for index, img_path in enumerate(image_paths, 1):
            ok, reason = process_one_image(
                img_path,
                scene_output,
                writer,
            )

            if ok:
                ok_count += 1
                class_info = parse_class_from_path(img_path)
                if class_info is not None:
                    class_counts[class_info] = class_counts.get(class_info, 0) + 1
            else:
                fail_count += 1
                reasons[reason] = reasons.get(reason, 0) + 1

            if index % 100 == 0 or index == len(image_paths):
                print(f"已处理 {index}/{len(image_paths)}")

    print("\n处理完成")
    print(f"✅ 成功：{ok_count}")
    print(f"❌ 失败：{fail_count}")

    if class_counts:
        print("\n成功裁剪分类统计：")
        for (main_class, subclass), count in sorted(class_counts.items()):
            print(f"  {main_class:10s} / {subclass:12s}: {count}")

    if reasons:
        print("\n失败原因：")
        for reason, count in sorted(reasons.items()):
            print(f"  {reason}: {count}")

    print("\n输出说明：")
    print(f"  后续划分和增强使用：{scene_output / 'roi_raw_success'}")
    print(f"  64x64结果快速检查： {scene_output / 'roi_64_success'}")
    print(f"  模型输入放大预览：  {scene_output / 'model_input_preview'}")
    print(f"  原图裁剪框检查：    {scene_output / 'debug'}")
    print(f"  红色掩码检查：      {scene_output / 'mask_debug'}")
    print(f"  失败调试图：        {scene_output / 'failed'}")
    print(f"  可疑ROI：           {scene_output / 'failed_roi'}")
    print("  默认不复制失败原图，避免重新生成一份几乎完整的数据集。")
    print(f"  处理记录：          {manifest_path}")
    print("\n下一步应先人工检查本场景 ROI，确认无误后再做 train/val/test 划分。")


if __name__ == "__main__":
    main()
