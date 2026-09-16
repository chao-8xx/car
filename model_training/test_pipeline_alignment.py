import ast
from pathlib import Path

import numpy as np
import pytest

import crop_target_from_red_split_SAFE_v2 as crop


ROOT = Path(__file__).resolve().parent


def test_crop_roi_matches_vehicle_red_marker_geometry():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    crop_img, crop_box = crop.crop_target_above_marker(img, (100, 150, 80, 24))

    assert crop_img is not None
    assert crop_box == (98, 100, 182, 184)
    assert crop_img.shape[:2] == (84, 84)


def test_red_marker_must_be_inside_detected_lane_boundaries():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:, 80:240] = (235, 235, 235)
    cv2 = pytest.importorskip("cv2")
    cv2.rectangle(img, (140, 150), (180, 160), (0, 0, 220), -1)

    red_box, _ = crop.find_red_marker(img)

    assert red_box is not None
    assert crop.is_marker_inside_lane(img, red_box)


def test_red_marker_outside_detected_lane_boundaries_is_rejected():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:, 80:240] = (235, 235, 235)
    cv2 = pytest.importorskip("cv2")
    cv2.rectangle(img, (20, 150), (60, 160), (0, 0, 220), -1)

    red_box, _ = crop.find_red_marker(img)

    assert red_box is None


def test_lower_lane_marker_scores_above_upper_distractor():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:, 80:240] = (235, 235, 235)
    cv2 = pytest.importorskip("cv2")
    cv2.rectangle(img, (130, 145), (185, 162), (0, 0, 220), -1)
    cv2.rectangle(img, (165, 82), (220, 105), (0, 0, 220), -1)

    red_box, _ = crop.find_red_marker(img)

    assert red_box is not None
    assert 125 <= red_box[0] <= 135
    assert 140 <= red_box[1] <= 150


def test_upper_off_lane_red_blob_does_not_beat_lower_track_marker():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:, 70:250] = (235, 235, 235)
    cv2 = pytest.importorskip("cv2")
    cv2.rectangle(img, (18, 86), (88, 110), (0, 0, 220), -1)
    cv2.rectangle(img, (178, 145), (222, 158), (0, 0, 220), -1)

    red_box, _ = crop.find_red_marker(img)

    assert red_box is not None
    assert 175 <= red_box[0] <= 185
    assert 140 <= red_box[1] <= 150


def test_crop_red_marker_thresholds_match_vehicle_strict_detector():
    assert crop.MIN_RED_AREA_RATIO == 0.0006
    assert crop.MAX_RED_AREA_RATIO == 0.013
    assert crop.MIN_ASPECT == 1.2
    assert crop.MAX_ASPECT == 5.2
    assert crop.MIN_Y_RATIO == 0.30
    assert crop.CROP_SIDE_SCALE == 1.05
    assert crop.CROP_BOTTOM_MARGIN_SCALE == 0.12


def test_far_small_red_marker_from_v3_failed_samples_are_detected():
    sample_dir = ROOT / "crop_output_v3" / "failed" / "no_red_marker" / "supplies"
    samples = []
    for stamp in ["001746", "001753"]:
        matches = list(sample_dir.glob(f"*{stamp}_debug.jpg"))
        if not matches:
            pytest.skip(f"failed debug sample {stamp} is not present")
        samples.extend(matches)

    for sample in samples:
        img = crop.imread_cn(sample)
        red_box, _ = crop.find_red_marker(img)

        assert red_box is not None
        assert red_box[2] >= 15
        assert red_box[3] >= 5


def test_v4_red_markers_with_flat_or_blocky_shapes_are_detected():
    sample_dir = ROOT / "crop_output_v4" / "failed" / "no_red_marker" / "supplies"
    samples = []
    for stamp in ["003224", "002217"]:
        matches = list(sample_dir.glob(f"*{stamp}_debug.jpg"))
        if not matches:
            pytest.skip(f"failed debug sample {stamp} is not present")
        samples.extend(matches)

    for sample in samples:
        img = crop.imread_cn(sample)
        red_box, _ = crop.find_red_marker(img)

        assert red_box is not None
        assert 130 <= red_box[0] <= 155
        assert 125 <= red_box[1] <= 135


def test_v7_transport_red_markers_inside_lane_are_detected():
    sample_dir = ROOT / "crop_output_v7" / "failed" / "no_red_marker" / "transport"
    samples = []
    for stamp in ["014308", "021030", "021056", "011229"]:
        matches = list(sample_dir.glob(f"*{stamp}_debug.jpg"))
        if not matches:
            pytest.skip(f"failed debug sample {stamp} is not present")
        samples.extend(matches)

    for sample in samples:
        img = crop.imread_cn(sample)
        red_box, _ = crop.find_red_marker(img)

        assert red_box is not None, sample.name
        assert 120 <= red_box[0] <= 140
        assert 120 <= red_box[1] <= 155


def test_v10_near_threshold_transport_red_markers_are_recovered():
    sample_dir = ROOT / "crop_output_v10" / "failed" / "no_red_marker" / "transport"
    samples = []
    for stamp in ["012023", "021155"]:
        matches = list(sample_dir.glob(f"*{stamp}_debug.jpg"))
        if not matches:
            pytest.skip(f"failed debug sample {stamp} is not present")
        samples.extend(matches)

    for sample in samples:
        img = crop.imread_cn(sample)
        red_box, _ = crop.find_red_marker(img)

        assert red_box is not None, sample.name
        assert crop.is_marker_inside_lane(img, red_box)
        assert 95 <= red_box[0] <= 190
        assert 115 <= red_box[1] <= 165


def test_training_uses_new_auto_crop_dataset_and_vehicle_class_order():
    tree = ast.parse((ROOT / "train_perfect.py").read_text(encoding="utf-8-sig"))
    assignments = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and isinstance(node.value, (ast.Constant, ast.List)):
                assignments[target.id] = ast.literal_eval(node.value)

    assert assignments["train_path"] == r"E:\code\Dataset_Split_mytrain_new\xianyu_balanced_redbox_64\train"
    assert assignments["val_path"] == r"E:\code\Dataset_Split_mytrain_new\xianyu_balanced_redbox_64\val"
    assert assignments["test_path"] == r"E:\code\Dataset_Split_mytrain_new\xianyu_balanced_redbox_64\test"
    assert assignments["EXPECTED_CLASS_ORDER"] == ["supplies", "transport", "weapon"]
