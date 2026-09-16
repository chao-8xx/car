import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TRAIN_FILE = ROOT / "train_perfect.py"


def parse_train_file():
    return ast.parse(TRAIN_FILE.read_text(encoding="utf-8-sig"))


def assigned_constants(tree):
    values = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name):
                try:
                    values[target.id] = ast.literal_eval(node.value)
                except Exception:
                    pass
    return values


def test_paths_include_train_val_and_test_sets():
    values = assigned_constants(parse_train_file())

    assert values["train_path"].endswith(r"xianyu_balanced_redbox_64\train")
    assert values["val_path"].endswith(r"xianyu_balanced_redbox_64\val")
    assert values["test_path"].endswith(r"xianyu_balanced_redbox_64\test")


def test_vehicle_style_augmentations_are_configured():
    source = TRAIN_FILE.read_text(encoding="utf-8-sig")

    assert "RandomAffine" in source
    assert "translate=" in source
    assert "ColorJitter" in source
    assert "GaussianBlur(kernel_size=5" in source
    assert "sigma=(0.2, 1.8)" in source
    assert "RandomJpegCompression" in source
    assert "RandomSensorNoise" in source
    assert "RandomHorizontalFlip" not in source


def test_training_loop_uses_adamw_scheduler_early_stop_and_test_eval():
    source = TRAIN_FILE.read_text(encoding="utf-8-sig")

    assert "optim.AdamW" in source
    assert "weight_decay=weight_decay" in source
    assert "ReduceLROnPlateau" in source
    assert "early_stop_patience" in source
    assert "evaluate(best_model, test_loader" in source
    assert "classification_report" in source
