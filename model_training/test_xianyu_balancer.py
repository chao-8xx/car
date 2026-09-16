import cv2
import numpy as np

from build_balanced_xianyu_dataset import (
    ClassConfig,
    detect_bottom_red_marker,
    get_balanced_limit,
)


def make_sample(angle_degrees=0):
    img = np.full((64, 64, 3), 220, dtype=np.uint8)
    rect = ((32, 48), (34, 8), angle_degrees)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillConvexPoly(img, box, (0, 0, 210))
    return img


def test_bottom_red_marker_scores_horizontal_above_slanted():
    horizontal = detect_bottom_red_marker(make_sample(0))
    slanted = detect_bottom_red_marker(make_sample(28))

    assert horizontal.valid
    assert slanted.valid
    assert horizontal.angle_abs < 5
    assert horizontal.score > slanted.score


def test_top_red_marker_is_rejected():
    img = np.full((64, 64, 3), 220, dtype=np.uint8)
    cv2.rectangle(img, (15, 10), (49, 18), (0, 0, 210), -1)

    result = detect_bottom_red_marker(img)

    assert not result.valid


def test_lower_middle_red_marker_is_accepted():
    img = np.full((64, 64, 3), 220, dtype=np.uint8)
    cv2.rectangle(img, (18, 13), (52, 41), (0, 0, 210), -1)

    result = detect_bottom_red_marker(img)

    assert result.valid


def test_balanced_limit_uses_smallest_subclass_count():
    configs = [
        ClassConfig("1枪支", "weapon", 6316),
        ClassConfig("2手榴弹", "weapon", 7580),
        ClassConfig("3装甲车", "transport", 4274),
        ClassConfig("4急救车", "transport", 4915),
        ClassConfig("5望远镜", "supplies", 7486),
        ClassConfig("6急救包", "supplies", 4765),
    ]

    assert get_balanced_limit(configs) == 4274
