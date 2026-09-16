# -*- coding: utf-8 -*-
"""
07_merge_four_scenes_dataset_v1.py

把四套已经完成增强的数据集按 split 对应合并：

直道/train + 普通弯道/train + 左圆环/train + 右圆环/train
    -> final_all/train

val 只与 val 合并，test 只与 test 合并。
绝对不要把四套数据先混在一起再重新随机划分。

输出结构：
final_all/
├─ train/
│  ├─ supplies/
│  │  ├─ straight/medkit/...
│  │  ├─ normal_curve/medkit/...
│  │  ├─ circle_left/medkit/...
│  │  └─ circle_right/medkit/...
│  ├─ transport/
│  └─ weapon/
├─ val/
└─ test/

ImageFolder 会把 train 下第一层的
supplies / transport / weapon 识别为三类，
scene/subclass 只是类内子目录，不会产生新类别。
"""

from __future__ import annotations

import csv
import os
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Dict, Tuple


# ============================ 必改配置 ============================

SOURCE_DATASETS = {
    "straight": Path(
        r"E:\code\Dataset_Split_mytrain_new\enhance\final\直道"
    ),
    "normal_curve": Path(
        r"E:\code\Dataset_Split_mytrain_new\enhance\final\弯道"
    ),
    "circle_left": Path(
        r"E:\code\Dataset_Split_mytrain_new\enhance\final\圆环左"
    ),
    "circle_right": Path(
        r"E:\code\Dataset_Split_mytrain_new\enhance\final\圆环右"
    ),
}

OUTPUT_ROOT = Path(
    r"E:\code\Dataset_Split_mytrain_new\enhance\final_all"
)

# "copy"：普通复制，最容易理解。
# "hardlink"：同一磁盘分区时几乎不额外占空间；失败会自动退回复制。
TRANSFER_MODE = "copy"

CLEAR_OUTPUT_ROOT = False
DRY_RUN = False

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
EXPECTED_CLASSES = {"supplies", "transport", "weapon"}


# ============================ 工具 ============================

def is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def unique_path(path: Path) -> Path:
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


def transfer_file(source: Path, destination: Path) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)

    if DRY_RUN:
        return "dry_run"

    if TRANSFER_MODE == "hardlink":
        try:
            os.link(source, destination)
            return "hardlink"
        except OSError:
            shutil.copy2(source, destination)
            return "copy_fallback"

    shutil.copy2(source, destination)
    return "copy"


def parse_source_image(
    image_path: Path,
    scene_root: Path,
) -> Tuple[str, str, str]:
    """
    输入应为：
    scene_root/split/main_class/subclass/.../file.jpg
    """
    relative = image_path.relative_to(scene_root)

    if len(relative.parts) < 4:
        raise ValueError(
            f"目录结构不是 split/main_class/subclass：{image_path}"
        )

    split = relative.parts[0]
    main_class = relative.parts[1]
    subclass = relative.parts[2]

    if split not in {"train", "val", "test"}:
        raise ValueError(f"未知 split：{split}")

    if main_class not in EXPECTED_CLASSES:
        raise ValueError(
            f"未知主类别：{main_class}，文件：{image_path}"
        )

    return split, main_class, subclass


# ============================ 主程序 ============================

def main():
    print("=" * 82)
    print("合并直道、普通弯道、左圆环、右圆环")
    print("规则：train→train，val→val，test→test")
    print(f"输出：{OUTPUT_ROOT}")
    print(f"传输模式：{TRANSFER_MODE}")
    print("=" * 82)

    for scene_name, scene_root in SOURCE_DATASETS.items():
        if not scene_root.exists():
            print(f"❌ 缺少 {scene_name}：{scene_root}")
            return

        for split in ("train", "val", "test"):
            if not (scene_root / split).exists():
                print(
                    f"❌ {scene_name} 缺少目录："
                    f"{scene_root / split}"
                )
                return

    if TRANSFER_MODE not in {"copy", "hardlink"}:
        print("❌ TRANSFER_MODE 只能是 copy 或 hardlink")
        return

    if CLEAR_OUTPUT_ROOT and OUTPUT_ROOT.exists() and not DRY_RUN:
        shutil.rmtree(OUTPUT_ROOT)

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    manifest_path = OUTPUT_ROOT / "merge_manifest.csv"
    counts: Dict[Tuple[str, str, str, str], int] = defaultdict(int)
    transfer_counts: Dict[str, int] = defaultdict(int)
    failures = []

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "scene",
            "split",
            "main_class",
            "subclass",
            "source",
            "destination",
            "transfer_mode",
        ])

        for scene_name, scene_root in SOURCE_DATASETS.items():
            image_paths = sorted([
                path
                for path in scene_root.rglob("*")
                if is_image(path)
            ])

            print(
                f"\n{scene_name}：扫描到 {len(image_paths)} 张"
            )

            for index, source in enumerate(image_paths, start=1):
                try:
                    split, main_class, subclass = parse_source_image(
                        source,
                        scene_root,
                    )

                    # 保留 scene/subclass，避免不同场景同名文件覆盖。
                    destination_dir = (
                        OUTPUT_ROOT
                        / split
                        / main_class
                        / scene_name
                        / subclass
                    )
                    destination = unique_path(
                        destination_dir / source.name
                    )

                    mode_used = transfer_file(
                        source,
                        destination,
                    )

                    counts[
                        split,
                        main_class,
                        scene_name,
                        subclass,
                    ] += 1
                    transfer_counts[mode_used] += 1

                    writer.writerow([
                        scene_name,
                        split,
                        main_class,
                        subclass,
                        str(source),
                        str(destination),
                        mode_used,
                    ])

                except Exception as error:
                    failures.append(
                        (str(source), repr(error))
                    )

                if index % 5000 == 0 or index == len(image_paths):
                    print(
                        f"  已处理 {index}/{len(image_paths)}"
                    )

    report_lines = [
        "四场景数据集合并报告",
        "=" * 82,
        f"OUTPUT_ROOT: {OUTPUT_ROOT}",
        f"TRANSFER_MODE: {TRANSFER_MODE}",
        "",
        "详细数量：",
    ]

    for key in sorted(counts):
        split, main_class, scene, subclass = key
        report_lines.append(
            f"  {split:<5} {main_class:<10} "
            f"{scene:<14} {subclass:<12}: {counts[key]}"
        )

    aggregate: Dict[Tuple[str, str], int] = defaultdict(int)
    scene_aggregate: Dict[Tuple[str, str], int] = defaultdict(int)

    for (
        split,
        main_class,
        scene,
        subclass,
    ), count in counts.items():
        aggregate[split, main_class] += count
        scene_aggregate[split, scene] += count

    report_lines += ["", "按 split / 三大类汇总："]

    for key in sorted(aggregate):
        split, main_class = key
        report_lines.append(
            f"  {split:<5} {main_class:<10}: "
            f"{aggregate[key]}"
        )

    report_lines += ["", "按 split / 场景汇总："]

    for key in sorted(scene_aggregate):
        split, scene = key
        report_lines.append(
            f"  {split:<5} {scene:<14}: "
            f"{scene_aggregate[key]}"
        )

    split_totals = defaultdict(int)
    for (split, _), count in aggregate.items():
        split_totals[split] += count

    report_lines += ["", "总体数量："]
    for split in ("train", "val", "test"):
        report_lines.append(
            f"  {split:<5}: {split_totals[split]}"
        )

    report_lines += [
        "",
        "文件传输统计：",
    ]
    for mode, count in sorted(transfer_counts.items()):
        report_lines.append(f"  {mode}: {count}")

    report_lines += [
        "",
        f"失败数量：{len(failures)}",
    ]

    report_path = OUTPUT_ROOT / "merge_report.txt"
    report_path.write_text(
        "\n".join(report_lines),
        encoding="utf-8",
    )

    failure_path = OUTPUT_ROOT / "merge_failures.csv"
    with failure_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["source", "error"])
        writer.writerows(failures)

    print("\n合并完成")
    print(f"train：{split_totals['train']}")
    print(f"val：  {split_totals['val']}")
    print(f"test： {split_totals['test']}")
    print(f"失败： {len(failures)}")
    print(f"报告： {report_path}")
    print(f"清单： {manifest_path}")


if __name__ == "__main__":
    main()
