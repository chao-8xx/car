# -*- coding: utf-8 -*-
import argparse
import csv
import random
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


SOURCE_DIR = Path(r"E:\code\dataset_all\数据集_咸鱼_5.28")
OUTPUT_DIR = Path(r"E:\code\Dataset_Split_mytrain_new\xianyu_balanced_redbox_64")

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
TRAIN_RATIO = 0.75
VAL_RATIO = 0.15
RANDOM_SEED = 2026


@dataclass(frozen=True)
class ClassConfig:
    folder_name: str
    main_class: str
    count: int = 0


@dataclass
class RedMarkerQuality:
    valid: bool
    score: float = 0.0
    angle_abs: float = 90.0
    aspect: float = 0.0
    area_ratio: float = 0.0
    box: tuple[int, int, int, int] = (0, 0, 0, 0)
    reason: str = "no_marker"


CLASS_CONFIGS = [
    ClassConfig("1枪支", "weapon"),
    ClassConfig("2手榴弹", "weapon"),
    ClassConfig("3装甲车", "transport"),
    ClassConfig("4急救车", "transport"),
    ClassConfig("5望远镜", "supplies"),
    ClassConfig("6急救包", "supplies"),
]


def imread_cn(path: Path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def list_images(folder: Path):
    return sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS)


def normalize_rect_angle(rect):
    (_, _), (rw, rh), angle = rect
    if rw <= 0 or rh <= 0:
        return 90.0, 0.0
    if rw < rh:
        angle += 90.0
        rw, rh = rh, rw
    while angle <= -90.0:
        angle += 180.0
    while angle > 90.0:
        angle -= 180.0
    angle_abs = abs(angle)
    if angle_abs > 45.0:
        angle_abs = abs(90.0 - angle_abs)
    return angle_abs, rw / max(rh, 1e-6)


def build_red_mask(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 35, 35]), np.array([55, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([125, 35, 35]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask


def detect_bottom_red_marker(img_bgr):
    if img_bgr is None or img_bgr.size == 0:
        return RedMarkerQuality(False, reason="read_failed")

    h_img, w_img = img_bgr.shape[:2]
    img_area = float(max(1, h_img * w_img))
    mask = build_red_mask(img_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = RedMarkerQuality(False, reason="no_contour")
    for cnt in contours:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue

        area_ratio = area / img_area
        if area_ratio < 0.0025 or area_ratio > 0.35:
            continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        y_ratio = cy / float(h_img)
        center_dist = abs(cx - w_img / 2.0) / float(w_img)
        fill_ratio = area / float(max(1, w * h))
        angle_abs, aspect = normalize_rect_angle(cv2.minAreaRect(cnt))

        if y_ratio < 0.38:
            continue
        if center_dist > 0.43:
            continue
        if fill_ratio < 0.25:
            continue
        if aspect < 1.05 or aspect > 8.0:
            continue

        angle_score = max(0.0, 1.0 - angle_abs / 35.0)
        center_score = max(0.0, 1.0 - center_dist / 0.43)
        y_score = min(max((y_ratio - 0.38) / 0.52, 0.0), 1.0)
        area_score = min(area_ratio / 0.08, 1.0)
        aspect_score = max(0.0, 1.0 - abs(aspect - 3.0) / 5.0)

        score = (
            angle_score * 4.0
            + center_score * 2.0
            + y_score * 1.5
            + aspect_score
            + area_score
        )
        quality = RedMarkerQuality(
            True,
            score=score,
            angle_abs=angle_abs,
            aspect=aspect,
            area_ratio=area_ratio,
            box=(x, y, w, h),
            reason="ok",
        )
        if quality.score > best.score:
            best = quality

    return best


def get_balanced_limit(configs):
    counts = [cfg.count for cfg in configs]
    if not counts or any(count <= 0 for count in counts):
        raise ValueError("all subclass counts must be positive")
    return min(counts)


def split_name(index, total):
    train_n = int(total * TRAIN_RATIO)
    val_n = int(total * VAL_RATIO)
    if index < train_n:
        return "train"
    if index < train_n + val_n:
        return "val"
    return "test"


def safe_copy_name(subclass_name: str, src: Path):
    return f"{subclass_name}_{src.stem}{src.suffix.lower()}"


def collect_ranked_images(class_dir: Path):
    rows = []
    for path in list_images(class_dir):
        img = imread_cn(path)
        quality = detect_bottom_red_marker(img)
        rows.append((path, quality))

    rows.sort(
        key=lambda item: (
            item[1].valid,
            item[1].score,
            -item[1].angle_abs,
            -item[0].stat().st_size,
            item[0].name,
        ),
        reverse=True,
    )
    return rows


def write_manifest(path: Path, manifest_rows):
    ensure_dir(path.parent)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "split",
                "main_class",
                "subclass",
                "source",
                "output",
                "score",
                "angle_abs",
                "aspect",
                "area_ratio",
                "box",
            ],
        )
        writer.writeheader()
        writer.writerows(manifest_rows)


def build_dataset(source_dir: Path, output_dir: Path, limit: int | None = None, dry_run: bool = False):
    configs = []
    for cfg in CLASS_CONFIGS:
        class_dir = source_dir / cfg.folder_name
        count = len(list_images(class_dir)) if class_dir.exists() else 0
        configs.append(ClassConfig(cfg.folder_name, cfg.main_class, count))

    balanced_limit = limit if limit is not None else get_balanced_limit(configs)
    if any(cfg.count < balanced_limit for cfg in configs):
        raise ValueError("requested limit is larger than at least one subclass")

    rng = random.Random(RANDOM_SEED)
    manifest_rows = []
    summary = []

    if not dry_run:
        ensure_dir(output_dir)

    for cfg in configs:
        ranked = collect_ranked_images(source_dir / cfg.folder_name)
        selected = ranked[:balanced_limit]
        rng.shuffle(selected)

        valid_count = sum(1 for _, q in selected if q.valid)
        summary.append((cfg.folder_name, cfg.main_class, cfg.count, balanced_limit, valid_count))

        for idx, (src, quality) in enumerate(selected):
            split = split_name(idx, balanced_limit)
            dst = output_dir / split / cfg.main_class / safe_copy_name(cfg.folder_name, src)
            if not dry_run:
                ensure_dir(dst.parent)
                shutil.copy2(src, dst)
            manifest_rows.append(
                {
                    "split": split,
                    "main_class": cfg.main_class,
                    "subclass": cfg.folder_name,
                    "source": str(src),
                    "output": str(dst),
                    "score": f"{quality.score:.6f}",
                    "angle_abs": f"{quality.angle_abs:.3f}",
                    "aspect": f"{quality.aspect:.3f}",
                    "area_ratio": f"{quality.area_ratio:.6f}",
                    "box": quality.box,
                }
            )

    if not dry_run:
        write_manifest(output_dir / "manifest.csv", manifest_rows)

    return balanced_limit, summary, manifest_rows


def parse_args():
    parser = argparse.ArgumentParser(description="Build a balanced 64x64 Xianyu red-box dataset.")
    parser.add_argument("--source", type=Path, default=SOURCE_DIR)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--limit", type=int, default=None, help="Images per six-way subclass. Defaults to min subclass count.")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    limit, summary, manifest_rows = build_dataset(args.source, args.output, args.limit, args.dry_run)

    print(f"source: {args.source}")
    print(f"output: {args.output}")
    print(f"per-subclass limit: {limit}")
    print(f"total selected: {len(manifest_rows)}")
    for subclass, main_class, count, selected, valid_count in summary:
        print(f"{subclass:8s} -> {main_class:9s} source={count:5d} selected={selected:5d} valid_marker={valid_count:5d}")
    if args.dry_run:
        print("dry-run: no files copied")
    else:
        print(f"manifest: {args.output / 'manifest.csv'}")


if __name__ == "__main__":
    main()
