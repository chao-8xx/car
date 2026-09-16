# -*- coding: utf-8 -*-
"""
02_split_roi_by_contiguous_groups_v1.py

作用：
1. 读取已经人工清理过的 roi_raw_success；
2. 按连续编号分组，避免相邻帧被拆到 train/val/test；
3. 保留原有六小类目录结构；
4. 默认复制，不删除源图片；
5. 输出 split_manifest.csv 和 split_report.txt。

输入目录示例：
roi_raw_success/
├─ supplies/
│  ├─ telescope/
│  └─ medkit/
├─ transport/
│  ├─ armored_car/
│  └─ ambulance/
└─ weapon/
   ├─ gun/
   └─ explosive/
"""

from __future__ import annotations

import csv
import hashlib
import random
import re
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ============================ 必改配置 ============================

SOURCE_DIR = r"E:\code\Dataset_Split_mytrain_new\enhance\huafen\side_right\roi_raw_success"
OUTPUT_DIR = r"E:\code\Dataset_Split_mytrain_new\enhance\fenzu\圆环右"

# 连续多少张视为一个不可拆分组。
# 直道连续帧很多时建议 80 或 100。
GROUP_SIZE = 10

TRAIN_RATIO = 0.75
VAL_RATIO = 0.15
TEST_RATIO = 0.10

RANDOM_SEED = 20260713

# 首次运行建议 False。确认输出目录无重要文件后才设 True。
CLEAR_OUTPUT_DIR = False

# True：只统计，不复制。
DRY_RUN = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


@dataclass(frozen=True)
class ImageItem:
    path: Path
    relative_class_dir: Path
    series_prefix: str
    frame_number: Optional[int]


@dataclass
class ImageGroup:
    group_id: str
    relative_class_dir: Path
    items: List[ImageItem]


def validate_config() -> None:
    total = TRAIN_RATIO + VAL_RATIO + TEST_RATIO
    if abs(total - 1.0) > 1e-8:
        raise ValueError(f"划分比例之和必须为1，当前为 {total}")
    if GROUP_SIZE <= 0:
        raise ValueError("GROUP_SIZE 必须大于0")


def is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def extract_last_number_and_prefix(stem: str) -> Tuple[Optional[int], str]:
    """
    取文件名中最后一个较长数字串作为帧号。

    例如：
    side_left__...snapshot_19700101_003846__none
    帧号取 003846。
    """
    matches = list(re.finditer(r"\d+", stem))
    if not matches:
        return None, "__non_numeric__"

    selected = None
    for match in reversed(matches):
        if len(match.group(0)) >= 3:
            selected = match
            break
    if selected is None:
        selected = matches[-1]

    return int(selected.group(0)), stem[:selected.start()]


def class_seed(relative_class_dir: Path) -> int:
    digest = hashlib.md5(
        str(relative_class_dir).encode("utf-8")
    ).hexdigest()
    return RANDOM_SEED + int(digest[:8], 16)


def unique_destination(path: Path) -> Path:
    if not path.exists():
        return path

    index = 1
    while True:
        candidate = path.with_name(
            f"{path.stem}__dup{index}{path.suffix}"
        )
        if not candidate.exists():
            return candidate
        index += 1


def collect_items(source_dir: Path) -> Dict[Path, List[ImageItem]]:
    """
    图片所在叶子目录即为小类目录。
    会完整保留 SOURCE_DIR 下的相对目录结构。
    """
    result: Dict[Path, List[ImageItem]] = defaultdict(list)

    for path in sorted(source_dir.rglob("*")):
        if not is_image(path):
            continue

        relative_class_dir = path.parent.relative_to(source_dir)
        frame_number, series_prefix = extract_last_number_and_prefix(
            path.stem
        )

        result[relative_class_dir].append(
            ImageItem(
                path=path,
                relative_class_dir=relative_class_dir,
                series_prefix=series_prefix,
                frame_number=frame_number,
            )
        )

    return dict(result)


def build_groups(
    relative_class_dir: Path,
    items: List[ImageItem],
) -> List[ImageGroup]:
    """
    同一小类内：
    先按文件名前缀分序列；
    再按帧号排序；
    每 GROUP_SIZE 张组成一个组。
    """
    by_series: Dict[str, List[ImageItem]] = defaultdict(list)

    for item in items:
        by_series[item.series_prefix].append(item)

    groups: List[ImageGroup] = []
    block_index = 0

    for series_prefix in sorted(by_series):
        series_items = by_series[series_prefix]
        series_items.sort(
            key=lambda item: (
                item.frame_number is None,
                item.frame_number if item.frame_number is not None else 0,
                item.path.name.lower(),
            )
        )

        for start in range(0, len(series_items), GROUP_SIZE):
            block = series_items[start:start + GROUP_SIZE]
            prefix_hash = hashlib.md5(
                series_prefix.encode("utf-8")
            ).hexdigest()[:8]

            group_id = (
                f"{relative_class_dir.as_posix()}"
                f"__{prefix_hash}"
                f"__block_{block_index:04d}"
            )

            groups.append(
                ImageGroup(
                    group_id=group_id,
                    relative_class_dir=relative_class_dir,
                    items=block,
                )
            )
            block_index += 1

    return groups


def assign_groups(
    groups: List[ImageGroup],
    relative_class_dir: Path,
) -> Dict[str, List[ImageGroup]]:
    """
    组级划分。一个组不会跨集合。
    """
    rng = random.Random(class_seed(relative_class_dir))
    groups = list(groups)
    rng.shuffle(groups)

    total = sum(len(group.items) for group in groups)
    targets = {
        "train": total * TRAIN_RATIO,
        "val": total * VAL_RATIO,
        "test": total * TEST_RATIO,
    }

    assigned = {"train": [], "val": [], "test": []}
    counts = {"train": 0, "val": 0, "test": 0}

    groups.sort(key=lambda group: len(group.items), reverse=True)

    for group in groups:
        chosen = max(
            ("train", "val", "test"),
            key=lambda split: (
                targets[split] - counts[split],
                targets[split],
            ),
        )
        assigned[chosen].append(group)
        counts[chosen] += len(group.items)

    return assigned


def copy_group(
    group: ImageGroup,
    split: str,
    output_dir: Path,
    writer,
) -> int:
    count = 0

    for item in group.items:
        destination_dir = (
            output_dir
            / split
            / item.relative_class_dir
        )
        destination_dir.mkdir(parents=True, exist_ok=True)

        destination = unique_destination(
            destination_dir / item.path.name
        )

        if not DRY_RUN:
            shutil.copy2(item.path, destination)

        writer.writerow([
            group.group_id,
            split,
            str(item.relative_class_dir),
            item.series_prefix,
            "" if item.frame_number is None else item.frame_number,
            str(item.path),
            str(destination),
        ])
        count += 1

    return count


def write_report(
    output_dir: Path,
    class_stats: Dict[str, Dict[str, int]],
    group_stats: Dict[str, Dict[str, int]],
    totals: Dict[str, int],
) -> None:
    lines = [
        "连续编号分组划分报告",
        "=" * 72,
        f"SOURCE_DIR: {SOURCE_DIR}",
        f"OUTPUT_DIR: {OUTPUT_DIR}",
        f"GROUP_SIZE: {GROUP_SIZE}",
        (
            f"RATIO: train={TRAIN_RATIO:.2f}, "
            f"val={VAL_RATIO:.2f}, "
            f"test={TEST_RATIO:.2f}"
        ),
        f"RANDOM_SEED: {RANDOM_SEED}",
        "",
        "各小类图片数量：",
    ]

    for class_name in sorted(class_stats):
        stats = class_stats[class_name]
        lines.append(
            f"  {class_name:<35} "
            f"train={stats['train']:>6} "
            f"val={stats['val']:>6} "
            f"test={stats['test']:>6} "
            f"total={sum(stats.values()):>6}"
        )

    lines += ["", "各小类分组数量："]

    for class_name in sorted(group_stats):
        stats = group_stats[class_name]
        lines.append(
            f"  {class_name:<35} "
            f"train={stats['train']:>4} "
            f"val={stats['val']:>4} "
            f"test={stats['test']:>4}"
        )

    total_images = sum(totals.values())
    lines += ["", "总体图片数量："]

    for split in ("train", "val", "test"):
        ratio = totals[split] / total_images if total_images else 0.0
        lines.append(
            f"  {split:<5}: {totals[split]:>7} "
            f"({ratio * 100:6.2f}%)"
        )

    lines.append(f"  total: {total_images:>7}")

    (output_dir / "split_report.txt").write_text(
        "\n".join(lines),
        encoding="utf-8",
    )


def main() -> None:
    validate_config()

    source_dir = Path(SOURCE_DIR)
    output_dir = Path(OUTPUT_DIR)

    print("=" * 72)
    print("按连续编号分组划分 train / val / test")
    print(f"输入：{source_dir}")
    print(f"输出：{output_dir}")
    print(f"组大小：{GROUP_SIZE}")
    print(
        f"比例：{TRAIN_RATIO:.0%} / "
        f"{VAL_RATIO:.0%} / "
        f"{TEST_RATIO:.0%}"
    )
    print("=" * 72)

    if not source_dir.exists():
        print(f"❌ 输入目录不存在：{source_dir}")
        return

    try:
        output_dir.resolve().relative_to(source_dir.resolve())
        print("❌ OUTPUT_DIR 不能放在 SOURCE_DIR 内部")
        return
    except ValueError:
        pass

    if CLEAR_OUTPUT_DIR and output_dir.exists() and not DRY_RUN:
        shutil.rmtree(output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)

    by_class = collect_items(source_dir)
    if not by_class:
        print("❌ 没有扫描到图片")
        return

    manifest_path = output_dir / "split_manifest.csv"
    class_stats: Dict[str, Dict[str, int]] = {}
    group_stats: Dict[str, Dict[str, int]] = {}
    totals = {"train": 0, "val": 0, "test": 0}

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "group_id",
            "split",
            "relative_class_dir",
            "series_prefix",
            "frame_number",
            "source_path",
            "destination_path",
        ])

        for relative_class_dir in sorted(
            by_class,
            key=lambda path: path.as_posix(),
        ):
            items = by_class[relative_class_dir]
            groups = build_groups(
                relative_class_dir,
                items,
            )
            assigned = assign_groups(
                groups,
                relative_class_dir,
            )

            class_name = relative_class_dir.as_posix()
            class_stats[class_name] = {
                "train": 0,
                "val": 0,
                "test": 0,
            }
            group_stats[class_name] = {
                "train": 0,
                "val": 0,
                "test": 0,
            }

            for split in ("train", "val", "test"):
                group_stats[class_name][split] = len(
                    assigned[split]
                )

                for group in assigned[split]:
                    copied = copy_group(
                        group,
                        split,
                        output_dir,
                        writer,
                    )
                    class_stats[class_name][split] += copied
                    totals[split] += copied

            print(
                f"{class_name}: "
                f"train={class_stats[class_name]['train']}, "
                f"val={class_stats[class_name]['val']}, "
                f"test={class_stats[class_name]['test']}, "
                f"total={len(items)}, "
                f"groups={len(groups)}"
            )

    write_report(
        output_dir,
        class_stats,
        group_stats,
        totals,
    )

    total_images = sum(totals.values())

    print("\n划分完成")
    for split in ("train", "val", "test"):
        ratio = totals[split] / total_images if total_images else 0.0
        print(
            f"{split}: {totals[split]} "
            f"({ratio:.2%})"
        )

    print(f"total: {total_images}")
    print(f"\n清单：{manifest_path}")
    print(f"报告：{output_dir / 'split_report.txt'}")
    print("\n下一步：只增强 train；val/test 不做随机增强。")


if __name__ == "__main__":
    main()
