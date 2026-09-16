# -*- coding: utf-8 -*-
"""
05_augment_left_circle_to_20000_v1.py

左圆环增强脚本。

功能：
1. 输入已经按连续编号划分好的“圆环左” train/val/test；
2. 只增强 train；
3. val/test 仅统一缩放为 64x64，不做随机增强；
4. 保留 main_class/subclass 六小类结构；
5. 将左圆环 train 精确规划到约 20000 张，并让六小类接近平衡；
6. 所有增强先作用于原尺寸 ROI，最后再缩放为 64x64；
7. 不修改、不删除输入数据；
8. 输出 manifest、数量报告和失败文件清单。

按当前数据设置的目标：
supplies/medkit       -> 3333
supplies/telescope    -> 3333
transport/ambulance   -> 3334
transport/armored_car -> 3333
weapon/explosive      -> 3333
weapon/gun            -> 3334

train 合计：20000 张。
若存在无法读取的原图，实际数量会略低，并记录在 failed_images.csv。
"""

from __future__ import annotations

import csv
import hashlib
import random
import shutil
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np


# ============================ 必改配置 ============================

INPUT_SPLIT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\fenzu\圆环左"
OUTPUT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\圆环左"

OUTPUT_SIZE = 64
JPEG_QUALITY = 95
RANDOM_SEED = 20260713

MAX_WORKERS = 8
CLEAR_OUTPUT_DIR = False
DRY_RUN = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ============================ 左圆环目标数量 ============================

# 这里的数量包含原图。
TARGET_TRAIN_COUNTS = {
    ("supplies", "medkit"): 3333,
    ("supplies", "telescope"): 3333,
    ("transport", "ambulance"): 3334,
    ("transport", "armored_car"): 3333,
    ("weapon", "explosive"): 3333,
    ("weapon", "gun"): 3334,
}


# ============================ 中文路径读写 ============================

def imread_cn(path: Path):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception:
        return None


def imwrite_cn(path: Path, image) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)

    ok, buffer = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    )
    if ok:
        buffer.tofile(str(path.with_suffix(".jpg")))
    return bool(ok)


# ============================ 基础工具 ============================

def is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def stable_seed(path: Path, extra: int = 0) -> int:
    digest = hashlib.md5(str(path).encode("utf-8")).hexdigest()
    return RANDOM_SEED + int(digest[:8], 16) + extra * 10007


def safe_stem(path: Path) -> str:
    digest = hashlib.md5(str(path).encode("utf-8")).hexdigest()[:10]
    return f"{path.stem}__{digest}"


def resize_final(image):
    return cv2.resize(
        image,
        (OUTPUT_SIZE, OUTPUT_SIZE),
        interpolation=cv2.INTER_AREA,
    )


def clip_uint8(image):
    return np.clip(image, 0, 255).astype(np.uint8)


def class_parts(
    image_path: Path,
    input_root: Path,
) -> Tuple[str, str]:
    relative = image_path.relative_to(input_root)

    # 路径结构：
    # train/main_class/subclass/file.jpg
    if len(relative.parts) < 4:
        raise ValueError(
            f"目录应为 split/main_class/subclass：{image_path}"
        )

    return relative.parts[1], relative.parts[2]


# ============================ 增强函数 ============================

def apply_light(image, rng: random.Random):
    """
    圆环亮度覆盖较广，但不生成完全黑掉或严重过曝的图片。
    """
    level = rng.choices(
        ["light", "medium", "strong"],
        weights=[0.35, 0.50, 0.15],
        k=1,
    )[0]

    if level == "light":
        gamma = rng.uniform(0.85, 1.18)
        contrast = rng.uniform(0.92, 1.10)
        brightness = rng.uniform(-0.10, 0.10) * 255.0
    elif level == "medium":
        gamma = rng.uniform(0.72, 1.35)
        contrast = rng.uniform(0.86, 1.16)
        brightness = rng.uniform(-0.16, 0.16) * 255.0
    else:
        gamma = rng.uniform(0.65, 1.45)
        contrast = rng.uniform(0.82, 1.20)
        brightness = rng.uniform(-0.20, 0.20) * 255.0

    normalized = image.astype(np.float32) / 255.0
    result = np.power(
        np.clip(normalized, 0.0, 1.0),
        gamma,
    ) * 255.0

    return clip_uint8(
        result * contrast + brightness
    )


def motion_blur_kernel(length: int, angle_deg: float):
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0

    center = (
        length / 2.0 - 0.5,
        length / 2.0 - 0.5,
    )
    matrix = cv2.getRotationMatrix2D(
        center,
        angle_deg,
        1.0,
    )
    kernel = cv2.warpAffine(
        kernel,
        matrix,
        (length, length),
    )

    total = float(kernel.sum())
    return kernel / max(total, 1e-6)


def apply_motion_blur(image, rng: random.Random):
    """
    左圆环高速侧扫：
    3/5/7 为主，少量 9；
    斜向模糊比例高于直道。
    """
    length = rng.choices(
        [3, 5, 7, 9],
        weights=[0.18, 0.34, 0.34, 0.14],
        k=1,
    )[0]

    direction = rng.choices(
        ["near_horizontal", "moderate", "strong"],
        weights=[0.25, 0.50, 0.25],
        k=1,
    )[0]

    if direction == "near_horizontal":
        angle = rng.uniform(-12.0, 12.0)
    elif direction == "moderate":
        magnitude = rng.uniform(15.0, 40.0)
        angle = magnitude if rng.random() < 0.5 else -magnitude
    else:
        magnitude = rng.uniform(40.0, 70.0)
        angle = magnitude if rng.random() < 0.5 else -magnitude

    return cv2.filter2D(
        image,
        -1,
        motion_blur_kernel(length, angle),
    )


def apply_downscale_restore(image, rng: random.Random):
    """
    模拟距离变化和远处像素不足：
    轻度 70%~88%：25%
    中度 50%~70%：55%
    重度 38%~50%：20%
    """
    h, w = image.shape[:2]

    level = rng.choices(
        ["light", "medium", "heavy"],
        weights=[0.25, 0.55, 0.20],
        k=1,
    )[0]

    if level == "light":
        scale = rng.uniform(0.70, 0.88)
    elif level == "medium":
        scale = rng.uniform(0.50, 0.70)
    else:
        scale = rng.uniform(0.38, 0.50)

    small_w = max(8, int(round(w * scale)))
    small_h = max(8, int(round(h * scale)))

    small = cv2.resize(
        image,
        (small_w, small_h),
        interpolation=cv2.INTER_AREA,
    )

    result = cv2.resize(
        small,
        (w, h),
        interpolation=rng.choice([
            cv2.INTER_LINEAR,
            cv2.INTER_CUBIC,
        ]),
    )

    if rng.random() < 0.30:
        sigma = rng.uniform(1.0, 3.5)
        noise_rng = np.random.default_rng(
            rng.randrange(0, 2**32)
        )
        noise = noise_rng.normal(
            0.0,
            sigma,
            result.shape,
        )
        result = clip_uint8(
            result.astype(np.float32) + noise
        )

    return result


def apply_side_perspective(image, rng: random.Random):
    """
    圆环侧视透视：
    轻度 4%~7%：25%
    中度 7%~11%：55%
    重度 11%~15%：20%

    主要模拟侧面梯形，而不是四个角无规则乱变。
    """
    h, w = image.shape[:2]

    level = rng.choices(
        ["light", "medium", "heavy"],
        weights=[0.25, 0.55, 0.20],
        k=1,
    )[0]

    if level == "light":
        ratio = rng.uniform(0.04, 0.07)
    elif level == "medium":
        ratio = rng.uniform(0.07, 0.11)
    else:
        ratio = rng.uniform(0.11, 0.15)

    shift_x = ratio * w
    shift_y = ratio * h * rng.uniform(0.25, 0.65)

    source = np.float32([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1],
    ])
    target = source.copy()

    mode = rng.choices(
        [
            "left_narrow",
            "right_narrow",
            "top_narrow",
            "bottom_narrow",
        ],
        weights=[0.38, 0.38, 0.12, 0.12],
        k=1,
    )[0]

    if mode == "left_narrow":
        target[0] += [shift_x, shift_y]
        target[3] += [shift_x, -shift_y]
        target[1] += [-shift_x * 0.10, 0]
        target[2] += [-shift_x * 0.10, 0]
    elif mode == "right_narrow":
        target[1] += [-shift_x, shift_y]
        target[2] += [-shift_x, -shift_y]
        target[0] += [shift_x * 0.10, 0]
        target[3] += [shift_x * 0.10, 0]
    elif mode == "top_narrow":
        target[0] += [shift_x * 0.55, shift_y]
        target[1] += [-shift_x * 0.55, shift_y]
    else:
        target[3] += [shift_x * 0.55, -shift_y]
        target[2] += [-shift_x * 0.55, -shift_y]

    matrix = cv2.getPerspectiveTransform(
        source,
        target,
    )

    return cv2.warpPerspective(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_anisotropic_compress(image, rng: random.Random):
    """
    模拟侧视状态下的单方向压缩。
    """
    h, w = image.shape[:2]

    if rng.random() < 0.70:
        # 圆环侧视以水平方向压缩为主。
        scale_x = rng.uniform(0.62, 0.90)
        scale_y = rng.uniform(0.94, 1.04)
    else:
        scale_x = rng.uniform(0.92, 1.04)
        scale_y = rng.uniform(0.72, 0.94)

    compressed_w = max(8, int(round(w * scale_x)))
    compressed_h = max(8, int(round(h * scale_y)))

    compressed = cv2.resize(
        image,
        (compressed_w, compressed_h),
        interpolation=cv2.INTER_LINEAR,
    )

    return cv2.resize(
        compressed,
        (w, h),
        interpolation=cv2.INTER_LINEAR,
    )


def apply_small_geometry(image, rng: random.Random):
    """
    ROI 已经在裁剪阶段做过方向归一化，
    因此只做小角度旋转和小幅偏移。
    """
    h, w = image.shape[:2]

    matrix = cv2.getRotationMatrix2D(
        (w / 2.0, h / 2.0),
        rng.uniform(-7.0, 7.0),
        rng.uniform(0.94, 1.06),
    )
    matrix[0, 2] += rng.uniform(-0.07, 0.07) * w
    matrix[1, 2] += rng.uniform(-0.05, 0.05) * h

    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_defocus(image, rng: random.Random):
    kernel = rng.choice([3, 3, 5])
    return cv2.GaussianBlur(
        image,
        (kernel, kernel),
        sigmaX=rng.uniform(0.45, 1.20),
    )


def apply_white_balance(image, rng: random.Random):
    """
    轻微通道变化，模拟色温差异。
    不做大范围 Hue 变化。
    """
    factors = np.array([
        rng.uniform(0.90, 1.10),
        rng.uniform(0.90, 1.10),
        rng.uniform(0.90, 1.10),
    ], dtype=np.float32)

    result = image.astype(np.float32) * factors
    return clip_uint8(result)


def apply_mixed_side(image, rng: random.Random):
    """
    一个主要侧视退化 + 一个轻辅助增强。
    """
    primary = rng.choices(
        [
            "perspective",
            "motion",
            "distance",
            "compress",
        ],
        weights=[0.32, 0.28, 0.23, 0.17],
        k=1,
    )[0]

    if primary == "perspective":
        result = apply_side_perspective(image, rng)
    elif primary == "motion":
        result = apply_motion_blur(image, rng)
    elif primary == "distance":
        result = apply_downscale_restore(image, rng)
    else:
        result = apply_anisotropic_compress(image, rng)

    if rng.random() < 0.50:
        result = apply_light(result, rng)

    if primary != "motion" and rng.random() < 0.12:
        result = apply_defocus(result, rng)

    if rng.random() < 0.12:
        result = apply_white_balance(result, rng)

    return result


def apply_hard_but_valid(image, rng: random.Random):
    """
    困难样本只叠加两个主要退化，避免彻底不可辨认。
    """
    first = rng.choice([
        "perspective",
        "distance",
        "compress",
    ])

    if first == "perspective":
        result = apply_side_perspective(image, rng)
    elif first == "distance":
        result = apply_downscale_restore(image, rng)
    else:
        result = apply_anisotropic_compress(image, rng)

    if rng.random() < 0.60:
        result = apply_motion_blur(result, rng)
    else:
        result = apply_light(result, rng)

    return result


def make_variant(
    image,
    variant_index: int,
    rng: random.Random,
):
    """
    12种循环增强，适合少量类别需要生成较多版本。

    0  距离退化
    1  运动模糊
    2  侧视透视
    3  单方向压缩
    4  距离 + 运动模糊
    5  透视 + 运动模糊
    6  透视 + 光照
    7  距离 + 光照
    8  透视 + 单方向压缩
    9  小几何 + 光照
    10 综合侧视增强
    11 困难但可辨认增强
    """
    kind = variant_index % 12

    if kind == 0:
        return apply_downscale_restore(image, rng), "distance"

    if kind == 1:
        return apply_motion_blur(image, rng), "motion"

    if kind == 2:
        return apply_side_perspective(image, rng), "perspective"

    if kind == 3:
        return apply_anisotropic_compress(image, rng), "compress"

    if kind == 4:
        result = apply_downscale_restore(image, rng)
        result = apply_motion_blur(result, rng)
        return result, "distance_motion"

    if kind == 5:
        result = apply_side_perspective(image, rng)
        result = apply_motion_blur(result, rng)
        return result, "perspective_motion"

    if kind == 6:
        result = apply_side_perspective(image, rng)
        result = apply_light(result, rng)
        return result, "perspective_light"

    if kind == 7:
        result = apply_downscale_restore(image, rng)
        result = apply_light(result, rng)
        return result, "distance_light"

    if kind == 8:
        result = apply_side_perspective(image, rng)
        result = apply_anisotropic_compress(result, rng)
        return result, "perspective_compress"

    if kind == 9:
        result = apply_small_geometry(image, rng)
        result = apply_light(result, rng)
        return result, "geometry_light"

    if kind == 10:
        return apply_mixed_side(image, rng), "mixed_side"

    return apply_hard_but_valid(image, rng), "hard_valid"


# ============================ 精确增强配额 ============================

def build_train_quotas(
    train_images: List[Path],
    input_root: Path,
) -> Dict[Path, int]:
    """
    返回每张训练原图需要生成多少张增强图。

    例：
    某小类有260张，目标3333张：
    需要增强3073张；
    每张固定11张，再让其中213张多生成1张。
    """
    grouped: Dict[Tuple[str, str], List[Path]] = defaultdict(list)

    for image_path in train_images:
        grouped[class_parts(image_path, input_root)].append(
            image_path
        )

    quotas: Dict[Path, int] = {}

    for class_key, images in sorted(grouped.items()):
        images = sorted(images)
        current_count = len(images)

        if current_count <= 0:
            continue

        target_count = TARGET_TRAIN_COUNTS.get(
            class_key,
            current_count,
        )
        target_count = max(target_count, current_count)

        required_aug = target_count - current_count
        base_aug = required_aug // current_count
        remainder = required_aug % current_count

        rng = random.Random(
            RANDOM_SEED
            + int(
                hashlib.md5(
                    str(class_key).encode("utf-8")
                ).hexdigest()[:8],
                16,
            )
        )
        shuffled = list(images)
        rng.shuffle(shuffled)
        extra_set = set(shuffled[:remainder])

        for image_path in images:
            quotas[image_path] = (
                base_aug
                + (1 if image_path in extra_set else 0)
            )

        print(
            f"{class_key[0]}/{class_key[1]}: "
            f"原图={current_count}, "
            f"目标={target_count}, "
            f"每张增强={base_aug}或{base_aug + 1}, "
            f"多生成1张的原图数={remainder}"
        )

    return quotas


# ============================ 单图处理 ============================

def process_train_image(
    image_path: Path,
    aug_count: int,
    input_root: Path,
    output_root: Path,
):
    main_class, subclass = class_parts(
        image_path,
        input_root,
    )

    image = imread_cn(image_path)
    if image is None or image.size == 0:
        return {
            "ok": False,
            "source": str(image_path),
            "reason": "read_failed",
            "records": [],
        }

    output_dir = (
        output_root
        / "train"
        / main_class
        / subclass
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    stem = safe_stem(image_path)
    records = []

    original_path = output_dir / f"{stem}__orig.jpg"
    if not DRY_RUN:
        imwrite_cn(
            original_path,
            resize_final(image),
        )

    records.append({
        "source": str(image_path),
        "output": str(original_path),
        "split": "train",
        "main_class": main_class,
        "subclass": subclass,
        "variant": "original",
    })

    # 每张图片的增强起始位置不同，避免小类内所有图片都以完全相同顺序循环。
    offset_rng = random.Random(
        stable_seed(image_path, 777)
    )
    start_offset = offset_rng.randrange(0, 12)

    for index in range(aug_count):
        rng = random.Random(
            stable_seed(image_path, index + 1)
        )

        augmented, variant_name = make_variant(
            image,
            start_offset + index,
            rng,
        )

        output_path = (
            output_dir
            / f"{stem}__aug{index + 1:02d}_{variant_name}.jpg"
        )

        if not DRY_RUN:
            imwrite_cn(
                output_path,
                resize_final(augmented),
            )

        records.append({
            "source": str(image_path),
            "output": str(output_path),
            "split": "train",
            "main_class": main_class,
            "subclass": subclass,
            "variant": variant_name,
        })

    return {
        "ok": True,
        "source": str(image_path),
        "reason": "",
        "records": records,
    }


def process_eval_image(
    image_path: Path,
    split: str,
    input_root: Path,
    output_root: Path,
):
    main_class, subclass = class_parts(
        image_path,
        input_root,
    )

    image = imread_cn(image_path)
    if image is None or image.size == 0:
        return {
            "ok": False,
            "source": str(image_path),
            "reason": "read_failed",
            "records": [],
        }

    output_dir = (
        output_root
        / split
        / main_class
        / subclass
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    stem = safe_stem(image_path)
    output_path = output_dir / f"{stem}__orig.jpg"

    if not DRY_RUN:
        imwrite_cn(
            output_path,
            resize_final(image),
        )

    return {
        "ok": True,
        "source": str(image_path),
        "reason": "",
        "records": [{
            "source": str(image_path),
            "output": str(output_path),
            "split": split,
            "main_class": main_class,
            "subclass": subclass,
            "variant": "original_only",
        }],
    }


# ============================ 主程序 ============================

def main():
    input_root = Path(INPUT_SPLIT_ROOT)
    output_root = Path(OUTPUT_ROOT)

    print("=" * 78)
    print("左圆环增强：train 目标约 20000 张")
    print(f"输入：{input_root}")
    print(f"输出：{output_root}")
    print(f"最终尺寸：{OUTPUT_SIZE}x{OUTPUT_SIZE}")
    print(f"线程数：{MAX_WORKERS}")
    print("=" * 78)

    if not input_root.exists():
        print(f"❌ 输入目录不存在：{input_root}")
        return

    for split in ("train", "val", "test"):
        if not (input_root / split).exists():
            print(f"❌ 缺少目录：{input_root / split}")
            return

    try:
        output_root.resolve().relative_to(
            input_root.resolve()
        )
        print("❌ OUTPUT_ROOT 不能放在 INPUT_SPLIT_ROOT 内部")
        return
    except ValueError:
        pass

    if (
        CLEAR_OUTPUT_DIR
        and output_root.exists()
        and not DRY_RUN
    ):
        shutil.rmtree(output_root)

    output_root.mkdir(parents=True, exist_ok=True)

    split_images: Dict[str, List[Path]] = {}
    for split in ("train", "val", "test"):
        split_images[split] = sorted([
            path
            for path in (input_root / split).rglob("*")
            if is_image(path)
        ])

    print(
        f"输入数量：train={len(split_images['train'])}, "
        f"val={len(split_images['val'])}, "
        f"test={len(split_images['test'])}"
    )

    quotas = build_train_quotas(
        split_images["train"],
        input_root,
    )

    all_records: List[Dict[str, str]] = []
    failed_records: List[Dict[str, str]] = []
    processed = 0

    with ThreadPoolExecutor(
        max_workers=MAX_WORKERS
    ) as executor:
        futures = {}

        for image_path in split_images["train"]:
            future = executor.submit(
                process_train_image,
                image_path,
                quotas.get(image_path, 0),
                input_root,
                output_root,
            )
            futures[future] = ("train", image_path)

        for split in ("val", "test"):
            for image_path in split_images[split]:
                future = executor.submit(
                    process_eval_image,
                    image_path,
                    split,
                    input_root,
                    output_root,
                )
                futures[future] = (
                    split,
                    image_path,
                )

        total_tasks = len(futures)

        for future in as_completed(futures):
            split, image_path = futures[future]

            try:
                result = future.result()
            except Exception as error:
                result = {
                    "ok": False,
                    "source": str(image_path),
                    "reason": repr(error),
                    "records": [],
                }

            if result["ok"]:
                all_records.extend(
                    result["records"]
                )
            else:
                failed_records.append({
                    "split": split,
                    "source": result["source"],
                    "reason": result["reason"],
                })

            processed += 1

            if (
                processed % 200 == 0
                or processed == total_tasks
            ):
                print(
                    f"已处理原图 {processed}/{total_tasks}，"
                    f"已生成 {len(all_records)} 张输出"
                )

    manifest_path = (
        output_root
        / "augmentation_manifest.csv"
    )
    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "source",
                "output",
                "split",
                "main_class",
                "subclass",
                "variant",
            ],
        )
        writer.writeheader()
        writer.writerows(all_records)

    failed_path = output_root / "failed_images.csv"
    with failed_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "split",
                "source",
                "reason",
            ],
        )
        writer.writeheader()
        writer.writerows(failed_records)

    counts: Dict[Tuple[str, str, str], int] = {}
    variant_counts: Dict[str, int] = {}

    for record in all_records:
        key = (
            record["split"],
            record["main_class"],
            record["subclass"],
        )
        counts[key] = counts.get(key, 0) + 1

        variant = record["variant"]
        variant_counts[variant] = (
            variant_counts.get(variant, 0) + 1
        )

    report_lines = [
        "左圆环增强输出报告",
        "=" * 78,
        f"INPUT_SPLIT_ROOT: {INPUT_SPLIT_ROOT}",
        f"OUTPUT_ROOT: {OUTPUT_ROOT}",
        f"OUTPUT_SIZE: {OUTPUT_SIZE}",
        f"DRY_RUN: {DRY_RUN}",
        "",
        "输出数量：",
    ]

    for key in sorted(counts):
        split, main_class, subclass = key
        report_lines.append(
            f"  {split:<5} "
            f"{main_class:<10} "
            f"{subclass:<12}: {counts[key]}"
        )

    report_lines += [
        "",
        "增强类型数量：",
    ]

    for variant, count in sorted(
        variant_counts.items()
    ):
        report_lines.append(
            f"  {variant:<24}: {count}"
        )

    train_total = sum(
        count
        for (split, _, _), count in counts.items()
        if split == "train"
    )
    val_total = sum(
        count
        for (split, _, _), count in counts.items()
        if split == "val"
    )
    test_total = sum(
        count
        for (split, _, _), count in counts.items()
        if split == "test"
    )

    report_lines += [
        "",
        f"train 总数：{train_total}",
        f"val 总数：{val_total}",
        f"test 总数：{test_total}",
        f"全部输出：{len(all_records)}",
        f"失败原图：{len(failed_records)}",
        "",
        "说明：",
        "train 使用左圆环侧视增强；",
        "val/test 仅 resize 到64x64，不做随机增强。",
    ]

    report_path = (
        output_root
        / "augmentation_report.txt"
    )
    report_path.write_text(
        "\n".join(report_lines),
        encoding="utf-8",
    )

    print("\n处理完成")
    print(f"train 输出：{train_total}")
    print(f"val 输出：{val_total}")
    print(f"test 输出：{test_total}")
    print(f"失败原图：{len(failed_records)}")
    print(f"清单：{manifest_path}")
    print(f"报告：{report_path}")


if __name__ == "__main__":
    main()
