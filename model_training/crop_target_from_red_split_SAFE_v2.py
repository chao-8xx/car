# -*- coding: utf-8 -*-
"""
crop_target_from_red_split_SAFE_v2.py

功能：
1. 从原始六小类数据集中读取图片；
2. 自动检测赛道上的红色提示矩形，支持旋转矩形；
3. 根据红色矩形位置，裁剪包含目标板和红框的 ROI；
4. 自动映射成三大类：weapon / supplies / transport；
5. 自动划分 train / val / test；
6. 保存 debug 图，方便人工检查裁剪是否正确；
7. 检测失败或裁剪越界的图片会放到 failed 文件夹。

使用方法：
    python crop_target_from_red_split_SAFE_v2.py

依赖：
    pip install opencv-python numpy
"""

import os
import cv2
import random
import numpy as np
from pathlib import Path


# ============================ 路径配置 ============================

SOURCE_DIR = r"E:\code\dataset_all\数据集_8.8"
OUTPUT_DIR = r"E:\code\Dataset_Split_mytrain_new\crop_output_v222"

CLEAR_OUTPUT = False  # 不再默认清空输出目录
TRAIN_RATIO = 0.85
VAL_RATIO = 0.10
TEST_RATIO = 0.05
RANDOM_SEED = 2026
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ============================ 类别映射 ============================

CLASS_MAP = {
    "1枪支": "weapon",
    "2炸药": "weapon",
    "3装甲车": "transport",
    "4急救车": "transport",
    "5望远镜": "supplies",
    "6急救包": "supplies",

    "枪支": "weapon",
    "炸药": "weapon",
    "武器": "weapon",
    "weapon": "weapon",

    "装甲车": "transport",
    "急救车": "transport",
    "救护车": "transport",
    "交通工具": "transport",
    "载具": "transport",
    "transport": "transport",

    "望远镜": "supplies",
    "急救包": "supplies",
    "物资": "supplies",
    "supplies": "supplies",
}


# ============================ 红框与裁剪参数 ============================

MIN_RED_AREA_RATIO = 0.0003      # 红色区域最小面积比例
MAX_RED_AREA_RATIO = 0.026       # 红色区域最大面积比例
MIN_ASPECT = 1.2                 # 红框最小宽高比
MAX_ASPECT = 6.5                 # 红框最大宽高比
RECOVERY_MAX_RED_AREA_RATIO = 0.014  # 只恢复略超面积上限的赛道内红框
RECOVERY_MIN_ASPECT = 1.15       # 只恢复略低于宽高比下限的赛道内红框
RECOVERY_MIN_FILL_RATIO = 0.40   # 恢复候选必须更像实心红色矩形
MIN_Y_RATIO = 0.30               # 红框的y位置不能太靠近顶部
MAX_Y_RATIO = 0.85               # 红框的y位置不能太靠近底部
CENTER_X_TOLERANCE = 0.36        # 红框中心离图像中心的容忍度
ASPECT_TARGET = 3.6              # 车端红框评分使用的目标宽高比

CROP_SIDE_SCALE = 1.05           # 与车端 build_target_roi() 对齐
CROP_BOTTOM_MARGIN_SCALE = 0.12  # 红框下方保留一点余量，避免贴边裁掉红框
CROP_X_SHIFT_SCALE = 0.00        # 裁剪框横向偏移
CROP_Y_SHIFT_SCALE = 0.00        # 裁剪框纵向偏移

OUTPUT_CROP_SIZE = 64            # 输出裁剪图的尺寸
SAVE_DEBUG = True                # 是否保存调试图


# ============================ 工具函数 ============================

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def unique_path(path: Path) -> Path:
    """生成一个唯一文件路径，避免文件覆盖"""
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    parent = path.parent
    idx = 1
    while True:
        new_path = parent / f"{stem}_{idx}{suffix}"
        if not new_path.exists():
            return new_path
        idx += 1


def safe_name(path: Path) -> str:
    parent = path.parent.name
    return f"{parent}_{path.stem}.jpg"


def get_main_class(folder_name: str):
    if folder_name in CLASS_MAP:
        return CLASS_MAP[folder_name]
    for key, value in CLASS_MAP.items():
        if key and key in folder_name:
            return value
    return None


def collect_images(source_dir: Path):
    items = []
    for root, dirs, files in os.walk(source_dir):
        root_path = Path(root)
        label = None
        for part in root_path.parts:
            cls = get_main_class(part)
            if cls is not None:
                label = cls
                break
        if label is None:
            continue
        for file in files:
            p = root_path / file
            if p.suffix.lower() in IMAGE_EXTS:
                items.append((p, label))
    return items


def split_items(items):
    random.seed(RANDOM_SEED)
    by_class = {}
    for img_path, label in items:
        by_class.setdefault(label, []).append((img_path, label))
    split_result = {
        "train": [],
        "val": [],
        "test": [],
    }
    for label, class_items in by_class.items():
        random.shuffle(class_items)
        n = len(class_items)
        n_train = int(n * TRAIN_RATIO)
        n_val = int(n * VAL_RATIO)
        split_result["train"].extend(class_items[:n_train])
        split_result["val"].extend(class_items[n_train:n_train + n_val])
        split_result["test"].extend(class_items[n_train + n_val:])
    for split in split_result:
        random.shuffle(split_result[split])
    return split_result


# ============================ 红框检测 ============================

def build_red_mask(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 45, 55])
    upper_red1 = np.array([55, 255, 255])
    lower_red2 = np.array([125, 75, 55])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
    return mask


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
        bottom_center_pixels = int(np.count_nonzero(component[bottom_y1:bottom_y2, center_x1:center_x2]))
        score = bottom_center_pixels * 4 + area
        if score > best_score:
            best_score = score
            best_label = label

    if best_label <= 0:
        return None
    return (labels == best_label).astype(np.uint8) * 255


def estimate_lane_bounds(img_bgr, y, search_half_height=8):
    h_img, w_img = img_bgr.shape[:2]
    if h_img <= 0 or w_img <= 0:
        return None

    y = int(np.clip(y, 0, h_img - 1))
    y1 = max(0, y - search_half_height)
    y2 = min(h_img, y + search_half_height + 1)
    track_mask = find_track_component_mask(img_bgr)
    if track_mask is None:
        return None

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

    # Fallback: use the widest bright lane-like segment.
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
        # If the source image cannot expose lane edges, keep the older center
        # gate instead of rejecting the sample blindly.
        return abs(cx - w_img / 2.0) / float(w_img) <= CENTER_X_TOLERANCE

    left, right = bounds
    lane_margin = max(5, w_img // 64)
    return (cx >= left - lane_margin and cx <= right + lane_margin)


def find_red_marker(img_bgr):
    h_img, w_img = img_bgr.shape[:2]
    img_area = h_img * w_img
    mask = build_red_mask(img_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        area_ratio = area / float(img_area)
        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue
        aspect = w / float(h)

        strict_shape = (
            MIN_RED_AREA_RATIO <= area_ratio <= MAX_RED_AREA_RATIO and
            MIN_ASPECT <= aspect <= MAX_ASPECT
        )
        recovery_shape = (
            MIN_RED_AREA_RATIO <= area_ratio <= RECOVERY_MAX_RED_AREA_RATIO and
            RECOVERY_MIN_ASPECT <= aspect <= MAX_ASPECT and
            (area_ratio > MAX_RED_AREA_RATIO or aspect < MIN_ASPECT)
        )
        if not strict_shape and not recovery_shape:
            continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        y_ratio = cy / float(h_img)
        if y_ratio < MIN_Y_RATIO or y_ratio > MAX_Y_RATIO:
            continue
        image_center_dist = abs(cx - w_img / 2.0) / float(w_img)
        if image_center_dist > CENTER_X_TOLERANCE:
            continue
        rect_area = w * h
        fill_ratio = area / float(rect_area)
        min_fill_ratio = 0.25 if strict_shape else RECOVERY_MIN_FILL_RATIO
        if fill_ratio < min_fill_ratio:
            continue
        if not is_marker_inside_lane(img_bgr, (x, y, w, h)):
            continue

        bounds = estimate_lane_bounds(img_bgr, cy)
        if bounds is None:
            lane_center_dist = image_center_dist
        else:
            left, right = bounds
            track_mid = (left + right) / 2.0
            lane_half_width = max((right - left) / 2.0, 1.0)
            lane_center_dist = min(abs(cx - track_mid) / lane_half_width, 1.0)

        center_score = 1.0 - lane_center_dist
        aspect_score = 1.0 - min(abs(aspect - ASPECT_TARGET) / ASPECT_TARGET, 1.0)
        lower_score = y_ratio
        area_score = min(area / float(img_area * 0.03), 1.0)
        score = center_score * 3.0 + aspect_score * 2.0 + lower_score + area_score
        if recovery_shape:
            score -= 0.5
        candidates.append((score, (x, y, w, h)))
    if not candidates:
        return None, mask
    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1], mask


def crop_target_above_marker(img_bgr, red_box):
    h_img, w_img = img_bgr.shape[:2]
    x, y, w, h = red_box
    cx = x + w / 2.0 + CROP_X_SHIFT_SCALE * w
    side = round(w * CROP_SIDE_SCALE)
    side = max(side, 12)
    marker_bottom = y + h
    bottom_margin = round(side * CROP_BOTTOM_MARGIN_SCALE)
    crop_y2 = marker_bottom + bottom_margin + CROP_Y_SHIFT_SCALE * h
    crop_y1 = crop_y2 - side
    crop_x1 = cx - side / 2.0
    crop_x2 = cx + side / 2.0
    x1 = int(round(crop_x1))
    y1 = int(round(crop_y1))
    x2 = int(round(crop_x2))
    y2 = int(round(crop_y2))
    crop_box = (x1, y1, x2, y2)
    if x1 < 0 or y1 < 0 or x2 > w_img or y2 > h_img:
        return None, crop_box
    if x2 <= x1 or y2 <= y1:
        return None, crop_box
    return img_bgr[y1:y2, x1:x2].copy(), crop_box


def draw_debug(img_bgr, red_box=None, crop_box=None, label="", status="ok"):
    dbg = img_bgr.copy()
    if red_box is not None:
        x, y, w, h = red_box
        cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(dbg, "red marker", (x, max(0, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    if crop_box is not None:
        x1, y1, x2, y2 = crop_box
        cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(dbg, "target crop", (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    text = f"{label} | {status}"
    cv2.putText(dbg, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 2)
    cv2.putText(dbg, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 0, 0), 1)
    return dbg


def imread_cn(path: Path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_cn(path: Path, img):
    ensure_dir(path.parent)
    ext = path.suffix
    if ext.lower() not in [".jpg", ".jpeg", ".png", ".bmp"]:
        ext = ".jpg"
        path = path.with_suffix(ext)
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(str(path))
    return ok


def process_one_image(img_path: Path, label: str, split: str, output_dir: Path):
    img = imread_cn(img_path)
    if img is None:
        return False, "read_failed"

    red_box, _ = find_red_marker(img)
    out_name = safe_name(img_path)
    stem = Path(out_name).stem

    if red_box is None:
        failed_path = unique_path(output_dir / "failed" / "no_red_marker" / label / f"{stem}_debug.jpg")
        dbg = draw_debug(img, None, None, label, "no_red_marker")
        imwrite_cn(failed_path, dbg)
        return False, "no_red_marker"

    crop_img, crop_box = crop_target_above_marker(img, red_box)

    if crop_img is None:
        failed_path = unique_path(output_dir / "failed" / "crop_out_of_bounds" / label / f"{stem}_debug.jpg")
        dbg = draw_debug(img, red_box, crop_box, label, "crop_out_of_bounds")
        imwrite_cn(failed_path, dbg)
        return False, "crop_out_of_bounds"

    crop_resized = cv2.resize(crop_img, (OUTPUT_CROP_SIZE, OUTPUT_CROP_SIZE), interpolation=cv2.INTER_AREA)

    crop_path = unique_path(output_dir / split / label / f"{stem}.jpg")
    imwrite_cn(crop_path, crop_resized)

    if SAVE_DEBUG:
        debug_path = unique_path(output_dir / "debug" / split / label / f"{stem}_debug.jpg")
        dbg = draw_debug(img, red_box, crop_box, label, "ok")
        imwrite_cn(debug_path, dbg)

    return True, "ok"


def create_output_dirs(output_dir: Path):
    for split in ["train", "val", "test"]:
        for cls in ["weapon", "supplies", "transport"]:
            ensure_dir(output_dir / split / cls)
    ensure_dir(output_dir / "debug")
    ensure_dir(output_dir / "failed")


def main():
    source_dir = Path(SOURCE_DIR)
    output_dir = Path(OUTPUT_DIR)

    print("============== 安全版裁剪脚本 ==============")
    print(f"原始数据集：{source_dir}")
    print(f"输出目录：  {output_dir}")
    print("===========================================\n")

    if not source_dir.exists():
        print(f"❌ 原始数据集路径不存在：{source_dir}")
        return

    create_output_dirs(output_dir)

    print("🔍 正在扫描图片...")
    items = collect_images(source_dir)

    if not items:
        print("❌ 没有扫描到图片，请检查 SOURCE_DIR 和六小类文件夹名字。")
        return

    print(f"✅ 共扫描到 {len(items)} 张图片。")

    count_by_class = {}
    for _, label in items:
        count_by_class[label] = count_by_class.get(label, 0) + 1

    print("📌 原始类别统计：")
    for k, v in sorted(count_by_class.items()):
        print(f"  {k}: {v}")

    split_result = split_items(items)

    print("📌 划分结果：")
    for split, split_items_list in split_result.items():
        tmp = {}
        for _, label in split_items_list:
            tmp[label] = tmp.get(label, 0) + 1
        print(f"  {split}: 总数 {len(split_items_list)} | {tmp}")

    total_ok = 0
    total_fail = 0
    fail_reasons = {}

    print("\n🚀 开始裁剪...")
    for split, split_items_list in split_result.items():
        for idx, (img_path, label) in enumerate(split_items_list, 1):
            ok, reason = process_one_image(img_path, label, split, output_dir)

            if ok:
                total_ok += 1
            else:
                total_fail += 1
                fail_reasons[reason] = fail_reasons.get(reason, 0) + 1

            if idx % 50 == 0:
                print(f"  [{split}] 已处理 {idx}/{len(split_items_list)}")

    print("\n================ 处理完成 ================")
    print(f"✅ 成功裁剪：{total_ok}")
    print(f"❌ 失败数量：{total_fail}")

    if fail_reasons:
        print("失败原因统计：")
        for reason, cnt in sorted(fail_reasons.items()):
            print(f"  {reason}: {cnt}")

    print("\n请检查：")
    print(f"  debug： {output_dir / 'debug'}")
    print(f"  failed：{output_dir / 'failed'}")
    print("\n训练集：")
    print(f"  {output_dir / 'train'}")
    print("验证集：")
    print(f"  {output_dir / 'val'}")
    print("测试集：")
    print(f"  {output_dir / 'test'}")


if __name__ == "__main__":
    main()
