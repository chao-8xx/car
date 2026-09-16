# -*- coding: utf-8 -*-
"""
04_augment_normal_curve_shallow_balanced_v1.py

普通弯道浅增强脚本。

特点：
1. 只增强 train；
2. val/test 只统一缩放到 64x64，不做随机增强；
3. 普通弯道原始 ROI 质量一般，因此增强次数少、强度浅；
4. 保留六小类目录结构；
5. 根据当前实际数量做轻度类别平衡；
6. 所有增强先作用于原尺寸 ROI，最后再缩放到 64x64；
7. 不修改、不删除输入数据。

按当前 train 数量预计：
supplies 约 1920
transport 约 1850
weapon 约 1826
普通弯道 train 合计约 5600 张。
"""

from __future__ import annotations

import csv
import hashlib
import random
import shutil
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np


# ============================ 必改配置 ============================

INPUT_SPLIT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\fenzu\弯道"
OUTPUT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\final\弯道"

OUTPUT_SIZE = 64
JPEG_QUALITY = 95
RANDOM_SEED = 20260713

MAX_WORKERS = 8
CLEAR_OUTPUT_DIR = False
DRY_RUN = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ============================ 增强数量方案 ============================

# aug_count：每张训练原图固定生成几张增强图，不含原图本身。
# extra_probability：再额外生成1张“很轻的组合增强”的概率。
#
# 当前数据预计：
# supplies:
#   medkit     200 × (1+3) = 800
#   telescope  560 × (1+1) = 1120
#   合计约 1920
#
# transport:
#   ambulance   300 × (1+2) + 25%额外 ≈ 975
#   armored_car 250 × (1+2) + 50%额外 ≈ 875
#   合计约 1850
#
# weapon:
#   explosive 250 × (1+2) = 750
#   gun       347 × (1+2) + 10%额外 ≈ 1076
#   合计约 1826
AUGMENT_PLAN = {
    ("supplies", "medkit"): {
        "aug_count": 3,
        "extra_probability": 0.00,
    },
    ("supplies", "telescope"): {
        "aug_count": 1,
        "extra_probability": 0.00,
    },
    ("transport", "ambulance"): {
        "aug_count": 2,
        "extra_probability": 0.25,
    },
    ("transport", "armored_car"): {
        "aug_count": 2,
        "extra_probability": 0.50,
    },
    ("weapon", "explosive"): {
        "aug_count": 2,
        "extra_probability": 0.00,
    },
    ("weapon", "gun"): {
        "aug_count": 2,
        "extra_probability": 0.10,
    },
}

DEFAULT_PLAN = {
    "aug_count": 1,
    "extra_probability": 0.00,
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

    suffix = path.suffix.lower()
    if suffix not in {".jpg", ".jpeg", ".png", ".bmp"}:
        suffix = ".jpg"
        path = path.with_suffix(suffix)

    params = []
    if suffix in {".jpg", ".jpeg"}:
        params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]

    ok, buffer = cv2.imencode(suffix, image, params)
    if ok:
        buffer.tofile(str(path))
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


def infer_class_parts(
    image_path: Path,
    input_root: Path,
) -> Tuple[str, str]:
    relative = image_path.relative_to(input_root)

    # 结构应为 split/main_class/subclass/file.jpg
    if len(relative.parts) < 4:
        raise ValueError(
            f"目录结构不是 split/main_class/subclass：{image_path}"
        )

    return relative.parts[1], relative.parts[2]


# ============================ 浅增强函数 ============================

def apply_mild_light(image, rng: random.Random):
    """
    普通弯道只做浅光照变化：
    Gamma 0.85~1.18
    亮度约 ±10%
    对比度 0.92~1.08
    """
    gamma = rng.uniform(0.85, 1.18)
    contrast = rng.uniform(0.92, 1.08)
    brightness = rng.uniform(-0.10, 0.10) * 255.0

    normalized = image.astype(np.float32) / 255.0
    result = np.power(
        np.clip(normalized, 0.0, 1.0),
        gamma,
    ) * 255.0

    result = result * contrast + brightness
    return clip_uint8(result)


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
    if total <= 1e-6:
        kernel[length // 2, :] = 1.0
        total = float(kernel.sum())

    return kernel / total


def apply_mild_motion_blur(image, rng: random.Random):
    """
    浅运动模糊：
    3像素为主，少量5，极少7。
    """
    length = rng.choices(
        [3, 5, 7],
        weights=[0.65, 0.32, 0.03],
        k=1,
    )[0]

    angle_type = rng.choices(
        ["horizontal", "slant"],
        weights=[0.55, 0.45],
        k=1,
    )[0]

    if angle_type == "horizontal":
        angle = rng.uniform(-8.0, 8.0)
    else:
        magnitude = rng.uniform(12.0, 28.0)
        angle = magnitude if rng.random() < 0.5 else -magnitude

    kernel = motion_blur_kernel(length, angle)
    return cv2.filter2D(image, -1, kernel)


def apply_mild_distance(image, rng: random.Random):
    """
    普通弯道浅距离退化：
    缩小到 72%~92% 后再恢复。
    不使用直道中的 40%~55%重度范围。
    """
    h, w = image.shape[:2]
    scale = rng.uniform(0.72, 0.92)

    small_w = max(8, int(round(w * scale)))
    small_h = max(8, int(round(h * scale)))

    small = cv2.resize(
        image,
        (small_w, small_h),
        interpolation=cv2.INTER_AREA,
    )

    return cv2.resize(
        small,
        (w, h),
        interpolation=rng.choice([
            cv2.INTER_LINEAR,
            cv2.INTER_CUBIC,
        ]),
    )


def apply_mild_perspective(image, rng: random.Random):
    """
    轻度方向性透视：
    主要为 2%~5%，极少到 6%。
    """
    h, w = image.shape[:2]

    if rng.random() < 0.92:
        ratio = rng.uniform(0.02, 0.05)
    else:
        ratio = rng.uniform(0.05, 0.06)

    shift_x = ratio * w
    shift_y = ratio * h * rng.uniform(0.20, 0.50)

    src = np.float32([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1],
    ])
    dst = src.copy()

    mode = rng.choice([
        "left_narrow",
        "right_narrow",
        "top_narrow",
    ])

    if mode == "left_narrow":
        dst[0] += [shift_x, shift_y]
        dst[3] += [shift_x, -shift_y]
    elif mode == "right_narrow":
        dst[1] += [-shift_x, shift_y]
        dst[2] += [-shift_x, -shift_y]
    else:
        dst[0] += [shift_x * 0.5, shift_y]
        dst[1] += [-shift_x * 0.5, shift_y]

    matrix = cv2.getPerspectiveTransform(src, dst)

    return cv2.warpPerspective(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_very_small_geometry(image, rng: random.Random):
    """
    极轻几何变化：
    旋转 ±3°
    水平 ±3%
    垂直 ±2%
    """
    h, w = image.shape[:2]

    matrix = cv2.getRotationMatrix2D(
        (w / 2.0, h / 2.0),
        rng.uniform(-3.0, 3.0),
        rng.uniform(0.98, 1.02),
    )
    matrix[0, 2] += rng.uniform(-0.03, 0.03) * w
    matrix[1, 2] += rng.uniform(-0.02, 0.02) * h

    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_mild_extra(image, rng: random.Random):
    """
    为平衡类别而生成的额外图，强度也保持浅。
    只叠加一个主要变化和一个很轻的辅助变化。
    """
    primary = rng.choice([
        "distance",
        "motion",
        "perspective",
    ])

    if primary == "distance":
        result = apply_mild_distance(image, rng)
    elif primary == "motion":
        result = apply_mild_motion_blur(image, rng)
    else:
        result = apply_mild_perspective(image, rng)

    if rng.random() < 0.45:
        result = apply_mild_light(result, rng)

    if rng.random() < 0.12:
        result = apply_very_small_geometry(result, rng)

    return result


def select_variant_names(
    image_path: Path,
    aug_count: int,
) -> List[str]:
    """
    让同一小类中即使每张只增强1~2次，
    整体仍能覆盖 distance / motion / perspective_light。

    aug_count=1：随机选择其中一种；
    aug_count=2：随机选择其中两种；
    aug_count=3：三种全部生成。
    """
    variants = [
        "distance",
        "motion",
        "perspective_light",
    ]

    rng = random.Random(stable_seed(image_path, 700))
    rng.shuffle(variants)
    return variants[:min(aug_count, len(variants))]


def make_variant(
    image,
    variant_name: str,
    rng: random.Random,
):
    if variant_name == "distance":
        result = apply_mild_distance(image, rng)
        if rng.random() < 0.25:
            result = apply_mild_light(result, rng)
        return result

    if variant_name == "motion":
        result = apply_mild_motion_blur(image, rng)
        if rng.random() < 0.20:
            result = apply_mild_light(result, rng)
        return result

    result = apply_mild_perspective(image, rng)
    if rng.random() < 0.65:
        result = apply_mild_light(result, rng)
    return result


# ============================ 单图处理 ============================

def process_train_image(
    image_path: Path,
    input_root: Path,
    output_root: Path,
):
    main_class, subclass = infer_class_parts(
        image_path,
        input_root,
    )

    plan = AUGMENT_PLAN.get(
        (main_class, subclass),
        DEFAULT_PLAN,
    )

    image = imread_cn(image_path)
    if image is None or image.size == 0:
        return {
            "ok": False,
            "source": str(image_path),
            "reason": "read_failed",
            "records": [],
        }

    destination_dir = (
        output_root
        / "train"
        / main_class
        / subclass
    )
    destination_dir.mkdir(parents=True, exist_ok=True)

    stem = safe_stem(image_path)
    records = []

    # 原图只做最终 64x64 resize。
    original_path = destination_dir / f"{stem}__orig.jpg"
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

    aug_count = int(plan["aug_count"])
    variant_names = select_variant_names(
        image_path,
        aug_count,
    )

    for index, variant_name in enumerate(
        variant_names,
        start=1,
    ):
        rng = random.Random(
            stable_seed(image_path, index),
        )

        augmented = make_variant(
            image,
            variant_name,
            rng,
        )
        output_path = (
            destination_dir
            / f"{stem}__aug{index:02d}_{variant_name}.jpg"
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

    extra_rng = random.Random(
        stable_seed(image_path, 999),
    )

    if extra_rng.random() < float(
        plan["extra_probability"]
    ):
        extra = apply_mild_extra(
            image,
            extra_rng,
        )
        output_path = (
            destination_dir
            / f"{stem}__extra_mild.jpg"
        )

        if not DRY_RUN:
            imwrite_cn(
                output_path,
                resize_final(extra),
            )

        records.append({
            "source": str(image_path),
            "output": str(output_path),
            "split": "train",
            "main_class": main_class,
            "subclass": subclass,
            "variant": "extra_mild",
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
    main_class, subclass = infer_class_parts(
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

    destination_dir = (
        output_root
        / split
        / main_class
        / subclass
    )
    destination_dir.mkdir(parents=True, exist_ok=True)

    stem = safe_stem(image_path)
    output_path = destination_dir / f"{stem}__orig.jpg"

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

    print("=" * 76)
    print("普通弯道浅增强 + 轻度类别平衡")
    print(f"输入：{input_root}")
    print(f"输出：{output_root}")
    print(f"最终尺寸：{OUTPUT_SIZE}x{OUTPUT_SIZE}")
    print(f"线程数：{MAX_WORKERS}")
    print("=" * 76)

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

    split_images = {}
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

    tasks = []
    for split in ("train", "val", "test"):
        for image_path in split_images[split]:
            tasks.append((split, image_path))

    all_records: List[Dict[str, str]] = []
    failed_records = []
    processed = 0

    with ThreadPoolExecutor(
        max_workers=MAX_WORKERS
    ) as executor:
        futures = {}

        for split, image_path in tasks:
            if split == "train":
                future = executor.submit(
                    process_train_image,
                    image_path,
                    input_root,
                    output_root,
                )
            else:
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
    for record in all_records:
        key = (
            record["split"],
            record["main_class"],
            record["subclass"],
        )
        counts[key] = counts.get(key, 0) + 1

    report_lines = [
        "普通弯道浅增强输出报告",
        "=" * 76,
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
        f"总输出图片数：{len(all_records)}",
        f"失败原图数：{len(failed_records)}",
        "",
        "说明：",
        "train 使用浅增强；val/test 仅 resize 到64x64。",
        "普通弯道不使用强模糊、强透视和重度缩小。",
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
    print(f"总输出图片数：{len(all_records)}")
    print(f"失败原图数：{len(failed_records)}")
    print(f"清单：{manifest_path}")
    print(f"报告：{report_path}")
    print("\nval/test 只做 resize，没有随机增强。")


if __name__ == "__main__":
    main()
