# -*- coding: utf-8 -*-
"""
03_augment_straight_train_balanced_v2.py

输入：已经划分好的直道 fenzu/train、val、test。
输出：
- train：保留原图并按小类生成不同数量增强图，最后统一为64x64；
- val/test：不做随机增强，只resize为64x64；
- 保留 main_class/subclass 六小类目录结构；
- 生成 manifest 和统计报告。

依据当前数量，目标是让三大类最终都接近3.3万张：
- supplies：约33044
- transport：约33050
- weapon：约33075
- 直道训练总量约9.9万张
- V2重点加强：远距离缩小再放大、透视变化、运动模糊；
- 保留轻/中/重三个强度范围，避免所有图片都被增强得过重。
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

INPUT_SPLIT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\fenzu"
OUTPUT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\final\直道"

OUTPUT_SIZE = 64
JPEG_QUALITY = 95
RANDOM_SEED = 20260713
MAX_WORKERS = 8
CLEAR_OUTPUT_DIR = False
DRY_RUN = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# aug_count不含原图本身；extra_probability表示再额外生成一张困难增强的概率。
AUGMENT_PLAN = {
    ("supplies", "medkit"):      {"aug_count": 5, "extra_probability": 0.00},
    ("supplies", "telescope"):   {"aug_count": 4, "extra_probability": 0.00},
    ("transport", "ambulance"):  {"aug_count": 6, "extra_probability": 0.00},
    ("transport", "armored_car"): {"aug_count": 6, "extra_probability": 0.50},
    ("weapon", "explosive"):     {"aug_count": 6, "extra_probability": 0.00},
    ("weapon", "gun"):           {"aug_count": 4, "extra_probability": 0.25},
}
DEFAULT_PLAN = {"aug_count": 4, "extra_probability": 0.00}


def imread_cn(path: Path):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception:
        return None


def imwrite_cn(path: Path, image) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    )
    if ok:
        buf.tofile(str(path.with_suffix(".jpg")))
    return bool(ok)


def is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def stable_seed(path: Path, extra: int = 0) -> int:
    digest = hashlib.md5(str(path).encode("utf-8")).hexdigest()
    return RANDOM_SEED + int(digest[:8], 16) + extra * 10007


def safe_stem(path: Path) -> str:
    digest = hashlib.md5(str(path).encode("utf-8")).hexdigest()[:10]
    return f"{path.stem}__{digest}"


def resize_final(image):
    return cv2.resize(image, (OUTPUT_SIZE, OUTPUT_SIZE), interpolation=cv2.INTER_AREA)


def clip_uint8(image):
    return np.clip(image, 0, 255).astype(np.uint8)


# ============================ 增强函数 ============================

def apply_gamma_brightness(image, rng: random.Random):
    gamma = rng.uniform(0.70, 1.40)
    contrast = rng.uniform(0.85, 1.18)
    brightness = rng.uniform(-0.18, 0.18) * 255.0
    x = image.astype(np.float32) / 255.0
    x = np.power(np.clip(x, 0.0, 1.0), gamma) * 255.0
    return clip_uint8(x * contrast + brightness)


def motion_blur_kernel(length: int, angle_deg: float):
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0
    center = (length / 2.0 - 0.5, length / 2.0 - 0.5)
    matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, matrix, (length, length))
    total = float(kernel.sum())
    return kernel / max(total, 1e-6)


def apply_motion_blur(image, rng: random.Random):
    """
    高速车辆运动模糊：
    轻/中/重核长度均保留，5和7像素为主，少量9像素。
    方向同时覆盖水平、一般斜向和较大斜向。
    """
    length = rng.choices(
        [3, 5, 7, 9],
        weights=[0.25, 0.35, 0.30, 0.10],
        k=1,
    )[0]

    direction_level = rng.choices(
        ["horizontal", "moderate", "strong"],
        weights=[0.35, 0.45, 0.20],
        k=1,
    )[0]

    if direction_level == "horizontal":
        angle = rng.uniform(-8.0, 8.0)
    elif direction_level == "moderate":
        magnitude = rng.uniform(15.0, 35.0)
        angle = magnitude if rng.random() < 0.5 else -magnitude
    else:
        magnitude = rng.uniform(35.0, 60.0)
        angle = magnitude if rng.random() < 0.5 else -magnitude

    return cv2.filter2D(
        image,
        -1,
        motion_blur_kernel(length, angle),
    )


def apply_downscale_restore(image, rng: random.Random):
    """
    模拟目标距离变化和像素不足：
    轻度 75%~90%：30%
    中度 55%~75%：50%
    重度 40%~55%：20%
    """
    h, w = image.shape[:2]

    level = rng.choices(
        ["light", "medium", "heavy"],
        weights=[0.30, 0.50, 0.20],
        k=1,
    )[0]

    if level == "light":
        scale = rng.uniform(0.75, 0.90)
    elif level == "medium":
        scale = rng.uniform(0.55, 0.75)
    else:
        scale = rng.uniform(0.40, 0.55)

    sw = max(8, round(w * scale))
    sh = max(8, round(h * scale))

    small = cv2.resize(
        image,
        (sw, sh),
        interpolation=cv2.INTER_AREA,
    )

    restored = cv2.resize(
        small,
        (w, h),
        interpolation=rng.choice([
            cv2.INTER_LINEAR,
            cv2.INTER_CUBIC,
        ]),
    )

    # 远距离图像偶尔伴随轻微噪声或压缩退化。
    if rng.random() < 0.30:
        sigma = rng.uniform(1.0, 3.5)
        noise_rng = np.random.default_rng(
            rng.randrange(0, 2**32)
        )
        noise = noise_rng.normal(
            0.0,
            sigma,
            restored.shape,
        )
        restored = clip_uint8(
            restored.astype(np.float32) + noise
        )

    return restored


def apply_light_geometry(image, rng: random.Random):
    h, w = image.shape[:2]
    matrix = cv2.getRotationMatrix2D(
        (w / 2.0, h / 2.0), rng.uniform(-8.0, 8.0), rng.uniform(0.94, 1.06)
    )
    matrix[0, 2] += rng.uniform(-0.08, 0.08) * w
    matrix[1, 2] += rng.uniform(-0.06, 0.06) * h
    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_light_perspective(image, rng: random.Random):
    """
    直道目标板透视：
    轻度 3%~5%：30%
    中度 5%~8%：50%
    重度 8%~12%：20%

    主要生成左窄右宽或右窄左宽的梯形，
    不做四个角完全无规则乱动。
    """
    h, w = image.shape[:2]

    level = rng.choices(
        ["light", "medium", "heavy"],
        weights=[0.30, 0.50, 0.20],
        k=1,
    )[0]

    if level == "light":
        ratio = rng.uniform(0.03, 0.05)
    elif level == "medium":
        ratio = rng.uniform(0.05, 0.08)
    else:
        ratio = rng.uniform(0.08, 0.12)

    shift_x = ratio * w
    shift_y = ratio * h * rng.uniform(0.25, 0.65)

    src = np.float32([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1],
    ])

    mode = rng.choice([
        "left_narrow",
        "right_narrow",
        "top_narrow",
        "bottom_narrow",
    ])

    dst = src.copy()

    if mode == "left_narrow":
        dst[0] += [shift_x, shift_y]
        dst[3] += [shift_x, -shift_y]
        dst[1] += [-shift_x * 0.15, 0]
        dst[2] += [-shift_x * 0.15, 0]
    elif mode == "right_narrow":
        dst[1] += [-shift_x, shift_y]
        dst[2] += [-shift_x, -shift_y]
        dst[0] += [shift_x * 0.15, 0]
        dst[3] += [shift_x * 0.15, 0]
    elif mode == "top_narrow":
        dst[0] += [shift_x * 0.55, shift_y]
        dst[1] += [-shift_x * 0.55, shift_y]
    else:
        dst[3] += [shift_x * 0.55, -shift_y]
        dst[2] += [-shift_x * 0.55, -shift_y]

    matrix = cv2.getPerspectiveTransform(src, dst)

    return cv2.warpPerspective(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_light_defocus(image, rng: random.Random):
    """
    少量模拟失焦/震动造成的普通模糊。
    """
    kernel = rng.choice([3, 3, 5])
    return cv2.GaussianBlur(
        image,
        (kernel, kernel),
        sigmaX=rng.uniform(0.35, 1.10),
    )


def apply_small_roi_shift(image, rng: random.Random):
    """
    模拟红框裁剪位置轻微抖动：
    水平±8%，垂直±6%。
    """
    h, w = image.shape[:2]
    matrix = np.float32([
        [1.0, 0.0, rng.uniform(-0.08, 0.08) * w],
        [0.0, 1.0, rng.uniform(-0.06, 0.06) * h],
    ])

    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def apply_anisotropic_compress(image, rng: random.Random):
    h, w = image.shape[:2]
    if rng.random() < 0.5:
        sx, sy = rng.uniform(0.82, 0.98), rng.uniform(0.96, 1.04)
    else:
        sx, sy = rng.uniform(0.96, 1.04), rng.uniform(0.82, 0.98)
    nw, nh = max(8, round(w * sx)), max(8, round(h * sy))
    compressed = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return cv2.resize(compressed, (w, h), interpolation=cv2.INTER_LINEAR)


def apply_mild_combination(image, rng: random.Random):
    """
    组合增强仍然限制为：
    一个主要问题 + 一个辅助问题。
    三个主要问题出现概率最高：
    距离退化、运动模糊、透视。
    """
    primary = rng.choices(
        [
            "downscale",
            "blur",
            "perspective",
            "compress",
            "shift",
        ],
        weights=[0.30, 0.28, 0.25, 0.09, 0.08],
        k=1,
    )[0]

    if primary == "downscale":
        result = apply_downscale_restore(image, rng)
    elif primary == "blur":
        result = apply_motion_blur(image, rng)
    elif primary == "perspective":
        result = apply_light_perspective(image, rng)
    elif primary == "compress":
        result = apply_anisotropic_compress(image, rng)
    else:
        result = apply_small_roi_shift(image, rng)

    # 光照是主要辅助增强。
    if rng.random() < 0.48:
        result = apply_gamma_brightness(result, rng)

    # 少量轻微失焦，不能和重运动模糊频繁叠加。
    if primary != "blur" and rng.random() < 0.12:
        result = apply_light_defocus(result, rng)

    return result


def apply_hard_but_valid(image, rng: random.Random):
    """
    困难增强：
    必须包含距离退化；
    再从运动模糊或透视中选择一个；
    最后小概率叠加光照。
    避免同时叠加强距离+强透视+核9模糊。
    """
    result = apply_downscale_restore(image, rng)

    if rng.random() < 0.58:
        result = apply_motion_blur(result, rng)
    else:
        result = apply_light_perspective(result, rng)

    if rng.random() < 0.45:
        result = apply_gamma_brightness(result, rng)

    return result


def make_variant(image, variant_index: int, rng: random.Random):
    """
    V2固定顺序：
    1 距离退化
    2 运动模糊
    3 透视变化
    4 距离退化 + 运动模糊
    5 透视 + 光照
    6 综合随机增强

    即使某个小类每张只生成4张，也能完整覆盖三个重点问题。
    """
    kind = variant_index % 6

    if kind == 0:
        return apply_downscale_restore(image, rng), "distance"

    if kind == 1:
        return apply_motion_blur(image, rng), "motion"

    if kind == 2:
        return apply_light_perspective(image, rng), "perspective"

    if kind == 3:
        result = apply_downscale_restore(image, rng)
        result = apply_motion_blur(result, rng)
        return result, "distance_motion"

    if kind == 4:
        result = apply_light_perspective(image, rng)
        if rng.random() < 0.75:
            result = apply_gamma_brightness(result, rng)
        return result, "perspective_light"

    return apply_mild_combination(image, rng), "combined"


# ============================ 单图处理 ============================

def class_parts(image_path: Path, input_root: Path) -> Tuple[str, str]:
    rel = image_path.relative_to(input_root)
    if len(rel.parts) < 3:
        raise ValueError(f"目录应为 split/main_class/subclass：{image_path}")
    return rel.parts[1], rel.parts[2]


def process_train_image(image_path: Path, input_root: Path, output_root: Path):
    main_class, subclass = class_parts(image_path, input_root)
    plan = AUGMENT_PLAN.get((main_class, subclass), DEFAULT_PLAN)
    image = imread_cn(image_path)
    if image is None or image.size == 0:
        return False, [], str(image_path), "read_failed"

    out_dir = output_root / "train" / main_class / subclass
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = safe_stem(image_path)
    records = []

    orig_path = out_dir / f"{stem}__orig.jpg"
    if not DRY_RUN:
        imwrite_cn(orig_path, resize_final(image))
    records.append((str(image_path), str(orig_path), "train", main_class, subclass, "original"))

    for index in range(int(plan["aug_count"])):
        rng = random.Random(stable_seed(image_path, index + 1))
        augmented, variant = make_variant(image, index, rng)
        out_path = out_dir / f"{stem}__aug{index + 1:02d}_{variant}.jpg"
        if not DRY_RUN:
            imwrite_cn(out_path, resize_final(augmented))
        records.append((str(image_path), str(out_path), "train", main_class, subclass, variant))

    extra_rng = random.Random(stable_seed(image_path, 999))
    if extra_rng.random() < float(plan["extra_probability"]):
        extra = apply_hard_but_valid(image, extra_rng)
        out_path = out_dir / f"{stem}__extra_hard.jpg"
        if not DRY_RUN:
            imwrite_cn(out_path, resize_final(extra))
        records.append((str(image_path), str(out_path), "train", main_class, subclass, "extra_hard"))

    return True, records, str(image_path), ""


def process_eval_image(image_path: Path, split: str, input_root: Path, output_root: Path):
    main_class, subclass = class_parts(image_path, input_root)
    image = imread_cn(image_path)
    if image is None or image.size == 0:
        return False, [], str(image_path), "read_failed"

    out_dir = output_root / split / main_class / subclass
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = safe_stem(image_path)
    out_path = out_dir / f"{stem}__orig.jpg"
    if not DRY_RUN:
        imwrite_cn(out_path, resize_final(image))
    record = (str(image_path), str(out_path), split, main_class, subclass, "original_only")
    return True, [record], str(image_path), ""


# ============================ 主程序 ============================

def main():
    input_root = Path(INPUT_SPLIT_ROOT)
    output_root = Path(OUTPUT_ROOT)

    print("=" * 76)
    print("直道训练集增强 + 三大类数量平衡")
    print(f"输入：{input_root}")
    print(f"输出：{output_root}")
    print(f"最终尺寸：{OUTPUT_SIZE}x{OUTPUT_SIZE}")
    print("=" * 76)

    if not input_root.exists():
        print(f"❌ 输入目录不存在：{input_root}")
        return

    for split in ("train", "val", "test"):
        if not (input_root / split).exists():
            print(f"❌ 缺少目录：{input_root / split}")
            return

    try:
        output_root.resolve().relative_to(input_root.resolve())
        print("❌ OUTPUT_ROOT 不能放在 INPUT_SPLIT_ROOT 内部")
        return
    except ValueError:
        pass

    if CLEAR_OUTPUT_DIR and output_root.exists() and not DRY_RUN:
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    split_images = {
        split: sorted([p for p in (input_root / split).rglob("*") if is_image(p)])
        for split in ("train", "val", "test")
    }
    print(
        f"输入数量：train={len(split_images['train'])}, "
        f"val={len(split_images['val'])}, test={len(split_images['test'])}"
    )

    tasks = [("train", p) for p in split_images["train"]]
    tasks += [("val", p) for p in split_images["val"]]
    tasks += [("test", p) for p in split_images["test"]]

    records: List[Tuple[str, str, str, str, str, str]] = []
    failures = []

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {}
        for split, image_path in tasks:
            if split == "train":
                future = executor.submit(process_train_image, image_path, input_root, output_root)
            else:
                future = executor.submit(process_eval_image, image_path, split, input_root, output_root)
            futures[future] = (split, image_path)

        total = len(futures)
        for index, future in enumerate(as_completed(futures), 1):
            split, image_path = futures[future]
            try:
                ok, new_records, source, reason = future.result()
            except Exception as exc:
                ok, new_records, source, reason = False, [], str(image_path), repr(exc)

            if ok:
                records.extend(new_records)
            else:
                failures.append((split, source, reason))

            if index % 250 == 0 or index == total:
                print(f"已处理原图 {index}/{total}，当前输出 {len(records)} 张")

    manifest = output_root / "augmentation_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["source", "output", "split", "main_class", "subclass", "variant"])
        writer.writerows(records)

    failed_csv = output_root / "failed_images.csv"
    with failed_csv.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["split", "source", "reason"])
        writer.writerows(failures)

    counts: Dict[Tuple[str, str, str], int] = {}
    for _, _, split, main_class, subclass, _ in records:
        key = (split, main_class, subclass)
        counts[key] = counts.get(key, 0) + 1

    report_lines = [
        "直道增强输出报告 V2（重点：距离退化/运动模糊/透视）",
        "=" * 76,
        f"INPUT_SPLIT_ROOT: {INPUT_SPLIT_ROOT}",
        f"OUTPUT_ROOT: {OUTPUT_ROOT}",
        f"OUTPUT_SIZE: {OUTPUT_SIZE}",
        "",
        "输出数量：",
    ]
    for (split, main_class, subclass), count in sorted(counts.items()):
        report_lines.append(
            f"  {split:<5} {main_class:<10} {subclass:<12}: {count}"
        )
    report_lines += [
        "",
        f"总输出图片数：{len(records)}",
        f"失败原图数：{len(failures)}",
        "",
        "说明：val/test仅resize到64x64，没有随机增强。",
    ]
    report = output_root / "augmentation_report.txt"
    report.write_text("\n".join(report_lines), encoding="utf-8")

    print("\n处理完成")
    print(f"总输出图片数：{len(records)}")
    print(f"失败原图数：{len(failures)}")
    print(f"清单：{manifest}")
    print(f"报告：{report}")


if __name__ == "__main__":
    main()
