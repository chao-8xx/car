# -*- coding: utf-8 -*-
"""
03_augment_straight_train_balanced_v1.py

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
OUTPUT_ROOT = r"E:\code\Dataset_Split_mytrain_new\enhance\final"

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
    length = rng.choices([3, 5, 7], weights=[0.50, 0.40, 0.10], k=1)[0]
    angle = rng.choice([0.0, rng.uniform(-30.0, -15.0), rng.uniform(15.0, 30.0)])
    return cv2.filter2D(image, -1, motion_blur_kernel(length, angle))


def apply_downscale_restore(image, rng: random.Random):
    h, w = image.shape[:2]
    scale = rng.uniform(0.50, 0.80)
    sw, sh = max(8, round(w * scale)), max(8, round(h * scale))
    small = cv2.resize(image, (sw, sh), interpolation=cv2.INTER_AREA)
    restored = cv2.resize(
        small,
        (w, h),
        interpolation=rng.choice([cv2.INTER_LINEAR, cv2.INTER_CUBIC]),
    )
    if rng.random() < 0.35:
        sigma = rng.uniform(1.5, 4.0)
        noise_rng = np.random.default_rng(rng.randrange(0, 2**32))
        noise = noise_rng.normal(0.0, sigma, restored.shape)
        restored = clip_uint8(restored.astype(np.float32) + noise)
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
    h, w = image.shape[:2]
    shift = rng.uniform(0.03, 0.07) * min(w, h)
    src = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
    dst = src.copy()
    for i in range(4):
        dst[i] += [rng.uniform(-shift, shift), rng.uniform(-shift, shift)]
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(
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
    choice = rng.choice(["blur", "perspective", "compress", "downscale"])
    if choice == "blur":
        result = apply_motion_blur(image, rng)
    elif choice == "perspective":
        result = apply_light_perspective(image, rng)
    elif choice == "compress":
        result = apply_anisotropic_compress(image, rng)
    else:
        result = apply_downscale_restore(image, rng)
    if rng.random() < 0.65:
        result = apply_gamma_brightness(result, rng)
    return result


def apply_hard_but_valid(image, rng: random.Random):
    result = apply_downscale_restore(image, rng)
    if rng.random() < 0.65:
        result = apply_motion_blur(result, rng)
    if rng.random() < 0.65:
        result = apply_gamma_brightness(result, rng)
    return result


def make_variant(image, variant_index: int, rng: random.Random):
    kind = variant_index % 6
    if kind == 0:
        return apply_gamma_brightness(image, rng), "light"
    if kind == 1:
        return apply_motion_blur(image, rng), "motion"
    if kind == 2:
        return apply_downscale_restore(image, rng), "distance"
    if kind == 3:
        return apply_light_geometry(image, rng), "geometry"
    if kind == 4:
        return apply_light_perspective(image, rng), "perspective"
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
        "直道增强输出报告",
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
