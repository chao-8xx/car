 # -*- coding: utf-8 -*-
r"""
manual_crop_target_board.py

手动裁剪目标板工具：
1. 从 D:\Download\数据集_2 读取六小类图片；
2. 自动映射为三大类 weapon / supplies / transport；
3. 每张图弹出窗口，你用鼠标框选“真正的目标板区域”；
4. 保存为 64x64 到 E:\code\Dataset_Split_mytrain_new\manual_crop_dataset\all\类别；
5. 支持跳过、退出；
6. 最后会自动按 75/15/10 划分 train / val / test。

运行：
    python manual_crop_target_board.py

操作：
    鼠标拖框：框住目标板
    Enter / Space：确认保存
    c：取消当前框，重新选
    ESC：跳过当前图片

依赖：
    pip install opencv-python numpy
"""

import os
import cv2
import random
import shutil
import numpy as np
from pathlib import Path


# ============================ 路径配置 ============================

SOURCE_DIR = r"D:\Download\数据集_2"
OUTPUT_DIR = r"E:\code\Dataset_Split_mytrain_new\manual_crop_dataset"

OUTPUT_CROP_SIZE = 64

TRAIN_RATIO = 0.75
VAL_RATIO = 0.15
TEST_RATIO = 0.10
RANDOM_SEED = 2026

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# ============================ 类别映射 ============================

CLASS_MAP = {
    "1枪支": "weapon",
    "2手榴弹": "weapon",
    "3装甲车": "transport",
    "4急救车": "transport",
    "5望远镜": "supplies",
    "6急救包": "supplies",

    "枪支": "weapon",
    "手榴弹": "weapon",
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


# ============================ 基础工具 ============================

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def imread_cn(path: Path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_cn(path: Path, img):
    ensure_dir(path.parent)
    ok, buf = cv2.imencode(path.suffix, img)
    if ok:
        buf.tofile(str(path))
    return ok


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
        small_class_name = None

        for part in root_path.parts:
            cls = get_main_class(part)
            if cls is not None:
                label = cls
                small_class_name = part
                break

        if label is None:
            continue

        for file in files:
            p = root_path / file
            if p.suffix.lower() in IMAGE_EXTS:
                items.append((p, label, small_class_name or root_path.name))

    return items


def unique_path(path: Path) -> Path:
    if not path.exists():
        return path

    stem = path.stem
    suffix = path.suffix
    parent = path.parent

    idx = 1
    while True:
        p = parent / f"{stem}_{idx}{suffix}"
        if not p.exists():
            return p
        idx += 1


def safe_output_name(img_path: Path, small_class_name: str) -> str:
    return f"{small_class_name}_{img_path.stem}.jpg"


# ============================ 手动裁剪 ============================

def draw_help(img, label, index, total, path):
    show = img.copy()

    text_lines = [
        f"{index}/{total}  class={label}",
        "Drag ROI around target board. ENTER/SPACE save. ESC skip.",
        str(path)
    ]

    y = 22
    for line in text_lines:
        cv2.putText(show, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2)
        cv2.putText(show, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)
        y += 20

    return show


def manual_crop_all():
    source_dir = Path(SOURCE_DIR)
    output_dir = Path(OUTPUT_DIR)
    all_dir = output_dir / "all"

    if not source_dir.exists():
        print(f"❌ 原始路径不存在：{source_dir}")
        return

    ensure_dir(all_dir)

    items = collect_images(source_dir)
    if not items:
        print("❌ 没扫描到图片，请检查 SOURCE_DIR 和文件夹命名。")
        return

    print(f"✅ 共扫描到 {len(items)} 张图片")
    print(f"📁 裁剪保存到：{all_dir}")
    print("操作：鼠标框选目标板，Enter/Space 保存，ESC 跳过。")

    saved = 0
    skipped = 0

    for idx, (img_path, label, small_class_name) in enumerate(items, 1):
        img = imread_cn(img_path)
        if img is None:
            print(f"⚠️ 读取失败：{img_path}")
            skipped += 1
            continue

        show = draw_help(img, label, idx, len(items), img_path)

        win_name = "manual crop target board"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 900, 650)

        roi = cv2.selectROI(win_name, show, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow(win_name)

        x, y, w, h = [int(v) for v in roi]

        if w <= 2 or h <= 2:
            skipped += 1
            print(f"⏭️ 跳过：{img_path.name}")
            continue

        crop = img[y:y+h, x:x+w].copy()
        crop = cv2.resize(crop, (OUTPUT_CROP_SIZE, OUTPUT_CROP_SIZE), interpolation=cv2.INTER_AREA)

        save_name = safe_output_name(img_path, small_class_name)
        save_path = unique_path(all_dir / label / save_name)
        imwrite_cn(save_path, crop)

        saved += 1
        print(f"✅ 保存：{save_path}")

    print("\n================ 手动裁剪结束 ================")
    print(f"✅ 已保存：{saved}")
    print(f"⏭️ 已跳过：{skipped}")
    print(f"📁 全部裁剪图：{all_dir}")

    split_dataset(all_dir, output_dir)


# ============================ 划分 train / val / test ============================

def split_dataset(all_dir: Path, output_dir: Path):
    random.seed(RANDOM_SEED)

    split_root = output_dir / "split"

    if split_root.exists():
        shutil.rmtree(split_root)

    for split in ["train", "val", "test"]:
        for cls in ["weapon", "supplies", "transport"]:
            ensure_dir(split_root / split / cls)

    print("\n📦 开始划分 train / val / test ...")

    for cls in ["weapon", "supplies", "transport"]:
        cls_dir = all_dir / cls
        if not cls_dir.exists():
            continue

        imgs = []
        for p in cls_dir.iterdir():
            if p.suffix.lower() in IMAGE_EXTS:
                imgs.append(p)

        random.shuffle(imgs)

        n = len(imgs)
        n_train = int(n * TRAIN_RATIO)
        n_val = int(n * VAL_RATIO)

        split_map = {
            "train": imgs[:n_train],
            "val": imgs[n_train:n_train + n_val],
            "test": imgs[n_train + n_val:],
        }

        for split, paths in split_map.items():
            for src in paths:
                dst = split_root / split / cls / src.name
                shutil.copy2(src, dst)

        print(f"{cls}: all={n}, train={len(split_map['train'])}, val={len(split_map['val'])}, test={len(split_map['test'])}")

    print("\n✅ 划分完成：")
    print(f"训练集：{split_root / 'train'}")
    print(f"验证集：{split_root / 'val'}")
    print(f"测试集：{split_root / 'test'}")


if __name__ == "__main__":
    manual_crop_all()
