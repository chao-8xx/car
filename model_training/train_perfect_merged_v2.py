# -*- coding: utf-8 -*-
"""
train_perfect_merged_low_memory_v3.py

基于用户原 train_perfect.py 的最小合理修改版：

1. 网络结构完全不变，避免影响后续车端部署和 NCNN 转换；
2. 数据路径改为四场景合并后的 final_all；
3. 离线数据已经做了大量距离、透视、运动模糊和光照增强，
   因此训练时不再叠加强运动模糊、JPEG、噪声和大仿射；
4. 在线只保留少量轻微亮度/对比度变化；
5. 使用 5 epoch warmup + cosine decay；
6. 使用轻微 label smoothing；
7. 最佳模型同时参考验证集 macro-F1，避免验证集类别数量差异影响；
8. 仍输出训练曲线、验证/测试混淆矩阵和分类报告；
9. 低内存模式：batch_size=128、num_workers=0、关闭 pin_memory。
"""

import os
import random

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    f1_score,
)
from torch.utils.data import DataLoader
from torchvision import datasets, transforms


# ============================ 配置 ============================

input_size = 64
batch_size = 128
epochs = 60

lr = 0.001
min_lr = 1e-5
weight_decay = 1e-4

warmup_epochs = 5
early_stop_patience = 12
label_smoothing = 0.05

num_workers = 0
random_seed = 20260713

dataset_root = (
    r"E:\code\Dataset_Split_mytrain_new\enhance\final_all"
)
train_path = os.path.join(dataset_root, "train")
val_path = os.path.join(dataset_root, "val")
test_path = os.path.join(dataset_root, "test")

EXPECTED_CLASS_ORDER = [
    "supplies",
    "transport",
    "weapon",
]

save_path = "best_model_merged.pth"

device = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)


# ============================ 随机种子 ============================

def seed_everything(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

    # 输入尺寸固定，benchmark 可提高卷积速度。
    torch.backends.cudnn.benchmark = True


# ============================ 网络结构：保持不变 ============================

class DWConv(nn.Module):
    def __init__(self, in_ch, out_ch, stride=1):
        super().__init__()

        self.dw = nn.Conv2d(
            in_ch,
            in_ch,
            kernel_size=3,
            stride=stride,
            padding=1,
            groups=in_ch,
            bias=False,
        )
        self.bn1 = nn.BatchNorm2d(in_ch)

        self.pw = nn.Conv2d(
            in_ch,
            out_ch,
            kernel_size=1,
            stride=1,
            padding=0,
            bias=False,
        )
        self.bn2 = nn.BatchNorm2d(out_ch)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        x = self.relu(self.bn1(self.dw(x)))
        x = self.relu(self.bn2(self.pw(x)))
        return x


class Net(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()

        self.features = nn.Sequential(
            nn.Conv2d(
                3,
                16,
                kernel_size=3,
                stride=2,
                padding=1,
                bias=False,
            ),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),

            DWConv(16, 32, stride=2),
            DWConv(32, 64, stride=2),
            DWConv(64, 128, stride=2),
        )

        self.global_pool = nn.AdaptiveAvgPool2d(1)

        self.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(128, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.global_pool(x)
        x = x.view(x.size(0), -1)
        return self.classifier(x)


# ============================ 数据变换 ============================

# 离线数据已经增强得很充分。
# 在线增强只做极轻微变化，避免“增强图再次被重度增强”。
train_transforms = transforms.Compose([
    transforms.Resize((input_size, input_size)),

    transforms.RandomApply(
        [
            transforms.ColorJitter(
                brightness=0.08,
                contrast=0.08,
                saturation=0.05,
            )
        ],
        p=0.12,
    ),

    transforms.ToTensor(),

    transforms.Normalize(
        [0.5, 0.5, 0.5],
        [0.5, 0.5, 0.5],
    ),
])

eval_transforms = transforms.Compose([
    transforms.Resize((input_size, input_size)),
    transforms.ToTensor(),
    transforms.Normalize(
        [0.5, 0.5, 0.5],
        [0.5, 0.5, 0.5],
    ),
])


# ============================ 数据加载 ============================

def build_loaders():
    for path in (train_path, val_path, test_path):
        if not os.path.isdir(path):
            raise FileNotFoundError(
                f"数据集目录不存在：{path}"
            )

    train_dataset = datasets.ImageFolder(
        train_path,
        transform=train_transforms,
    )
    val_dataset = datasets.ImageFolder(
        val_path,
        transform=eval_transforms,
    )
    test_dataset = datasets.ImageFolder(
        test_path,
        transform=eval_transforms,
    )

    class_names = train_dataset.classes

    if class_names != EXPECTED_CLASS_ORDER:
        raise RuntimeError(
            f"class order mismatch: "
            f"got {class_names}, "
            f"expected {EXPECTED_CLASS_ORDER}"
        )

    if val_dataset.class_to_idx != train_dataset.class_to_idx:
        raise RuntimeError(
            "val 类别映射与 train 不一致"
        )

    if test_dataset.class_to_idx != train_dataset.class_to_idx:
        raise RuntimeError(
            "test 类别映射与 train 不一致"
        )

    # Windows + PyCharm 下优先使用低内存、稳定配置。
    loader_kwargs = {
        "batch_size": batch_size,
        "num_workers": 0,
        "pin_memory": False,
    }

    train_loader = DataLoader(
        train_dataset,
        shuffle=True,
        **loader_kwargs,
    )
    val_loader = DataLoader(
        val_dataset,
        shuffle=False,
        **loader_kwargs,
    )
    test_loader = DataLoader(
        test_dataset,
        shuffle=False,
        **loader_kwargs,
    )

    print(f"类别映射：{train_dataset.class_to_idx}")
    print(
        f"数据量：train={len(train_dataset)}, "
        f"val={len(val_dataset)}, "
        f"test={len(test_dataset)}"
    )

    return (
        train_loader,
        val_loader,
        test_loader,
        class_names,
    )


# ============================ 评估和绘图 ============================

def evaluate(model, loader, criterion):
    model.eval()

    all_preds = []
    all_labels = []

    running_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for images, labels in loader:
            images = images.to(
                device,
                non_blocking=True,
            )
            labels = labels.to(
                device,
                non_blocking=True,
            )

            outputs = model(images)
            loss = criterion(outputs, labels)
            preds = outputs.argmax(dim=1)

            running_loss += (
                loss.item() * labels.size(0)
            )
            correct += (
                preds == labels
            ).sum().item()
            total += labels.size(0)

            all_preds.extend(
                preds.cpu().numpy()
            )
            all_labels.extend(
                labels.cpu().numpy()
            )

    avg_loss = running_loss / max(1, total)
    accuracy = 100.0 * correct / max(1, total)
    macro_f1 = 100.0 * f1_score(
        all_labels,
        all_preds,
        average="macro",
        zero_division=0,
    )

    return (
        avg_loss,
        accuracy,
        macro_f1,
        all_preds,
        all_labels,
    )


def plot_history(
    train_losses,
    train_accs,
    val_accs,
    val_macro_f1s,
):
    plt.figure(figsize=(13, 5))

    plt.subplot(1, 2, 1)
    plt.plot(
        train_losses,
        label="Train Loss",
    )
    plt.title("Training Loss")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(
        train_accs,
        label="Train Acc",
    )
    plt.plot(
        val_accs,
        label="Val Acc",
    )
    plt.plot(
        val_macro_f1s,
        label="Val Macro-F1",
    )
    plt.title("Accuracy / Macro-F1")
    plt.xlabel("Epoch")
    plt.ylabel("Percent")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(
        "training_results_merged.png",
        dpi=160,
    )
    plt.close()

    print(
        "曲线图已保存：training_results_merged.png"
    )


def plot_confusion_matrix(
    labels,
    preds,
    class_names,
    output_path,
    title,
):
    cm = confusion_matrix(
        labels,
        preds,
        labels=list(range(len(class_names))),
    )

    plt.figure(figsize=(8, 6))
    sns.heatmap(
        cm,
        annot=True,
        fmt="d",
        cmap="Blues",
        xticklabels=class_names,
        yticklabels=class_names,
    )
    plt.xlabel("Predicted Label")
    plt.ylabel("True Label")
    plt.title(title)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()


# ============================ 训练 ============================

def train():
    print(f"Using device: {device}")
    print(f"batch_size={batch_size}, num_workers=0, pin_memory=False")
    seed_everything(random_seed)

    (
        train_loader,
        val_loader,
        test_loader,
        class_names,
    ) = build_loaders()

    model = Net(
        num_classes=len(class_names)
    ).to(device)

    # 三大类训练数量已经非常接近，不需要 class weight
    # 或 WeightedRandomSampler。
    criterion = nn.CrossEntropyLoss(
        label_smoothing=label_smoothing
    )

    optimizer = optim.AdamW(
        model.parameters(),
        lr=lr,
        weight_decay=weight_decay,
    )

    warmup_scheduler = optim.lr_scheduler.LinearLR(
        optimizer,
        start_factor=0.20,
        end_factor=1.0,
        total_iters=warmup_epochs,
    )

    cosine_scheduler = (
        optim.lr_scheduler.CosineAnnealingLR(
            optimizer,
            T_max=max(1, epochs - warmup_epochs),
            eta_min=min_lr,
        )
    )

    scheduler = optim.lr_scheduler.SequentialLR(
        optimizer,
        schedulers=[
            warmup_scheduler,
            cosine_scheduler,
        ],
        milestones=[warmup_epochs],
    )

    history_train_loss = []
    history_train_acc = []
    history_val_acc = []
    history_val_macro_f1 = []

    best_macro_f1 = -1.0
    best_val_acc = 0.0
    best_epoch = 0
    no_improve_epochs = 0

    for epoch in range(epochs):
        model.train()

        running_loss = 0.0
        correct_train = 0
        total_train = 0

        for images, labels in train_loader:
            images = images.to(
                device,
                non_blocking=True,
            )
            labels = labels.to(
                device,
                non_blocking=True,
            )

            optimizer.zero_grad(
                set_to_none=True
            )

            outputs = model(images)
            loss = criterion(outputs, labels)

            loss.backward()
            optimizer.step()

            running_loss += (
                loss.item() * labels.size(0)
            )
            predicted = outputs.argmax(dim=1)

            total_train += labels.size(0)
            correct_train += (
                predicted == labels
            ).sum().item()

        epoch_loss = (
            running_loss / max(1, total_train)
        )
        epoch_acc = (
            100.0
            * correct_train
            / max(1, total_train)
        )

        (
            val_loss,
            val_acc,
            val_macro_f1,
            _,
            _,
        ) = evaluate(
            model,
            val_loader,
            criterion,
        )

        history_train_loss.append(epoch_loss)
        history_train_acc.append(epoch_acc)
        history_val_acc.append(val_acc)
        history_val_macro_f1.append(
            val_macro_f1
        )

        scheduler.step()
        current_lr = optimizer.param_groups[0]["lr"]

        print(
            f"Epoch [{epoch + 1:02d}/{epochs}] | "
            f"Loss {epoch_loss:.4f} | "
            f"Train Acc {epoch_acc:.2f}% | "
            f"Val Loss {val_loss:.4f} | "
            f"Val Acc {val_acc:.2f}% | "
            f"Val Macro-F1 {val_macro_f1:.2f}% | "
            f"LR {current_lr:.7f}"
        )

        # 验证集类别数量不完全一致，所以按 Macro-F1
        # 保存最佳模型；相同时再比较准确率。
        improved = (
            val_macro_f1 > best_macro_f1 + 1e-6
            or (
                abs(
                    val_macro_f1 - best_macro_f1
                ) <= 1e-6
                and val_acc > best_val_acc
            )
        )

        if improved:
            best_macro_f1 = val_macro_f1
            best_val_acc = val_acc
            best_epoch = epoch + 1
            no_improve_epochs = 0

            torch.save(
                model.state_dict(),
                save_path,
            )
        else:
            no_improve_epochs += 1

        if (
            no_improve_epochs
            >= early_stop_patience
        ):
            print(
                f"Early stopping at epoch "
                f"{epoch + 1}. "
                f"Best epoch={best_epoch}, "
                f"Val Acc={best_val_acc:.2f}%, "
                f"Macro-F1={best_macro_f1:.2f}%"
            )
            break

    print(
        f"\n训练结束：Best epoch={best_epoch}, "
        f"Val Acc={best_val_acc:.2f}%, "
        f"Val Macro-F1={best_macro_f1:.2f}%"
    )

    plot_history(
        history_train_loss,
        history_train_acc,
        history_val_acc,
        history_val_macro_f1,
    )

    best_model = Net(
        num_classes=len(class_names)
    ).to(device)

    state_dict = torch.load(
        save_path,
        map_location=device,
        weights_only=True,
    )
    best_model.load_state_dict(state_dict)

    (
        val_loss,
        val_acc,
        val_macro_f1,
        val_preds,
        val_labels,
    ) = evaluate(
        best_model,
        val_loader,
        criterion,
    )

    print(
        f"\nBest model val："
        f"loss={val_loss:.4f}, "
        f"acc={val_acc:.2f}%, "
        f"macro-F1={val_macro_f1:.2f}%"
    )

    plot_confusion_matrix(
        val_labels,
        val_preds,
        class_names,
        "confusion_matrix_val_merged.png",
        "Validation Confusion Matrix",
    )

    (
        test_loss,
        test_acc,
        test_macro_f1,
        test_preds,
        test_labels,
    ) = evaluate(
        best_model,
        test_loader,
        criterion,
    )

    print(
        f"\nBest model test："
        f"loss={test_loss:.4f}, "
        f"acc={test_acc:.2f}%, "
        f"macro-F1={test_macro_f1:.2f}%"
    )

    print(
        classification_report(
            test_labels,
            test_preds,
            labels=list(
                range(len(class_names))
            ),
            target_names=class_names,
            digits=4,
            zero_division=0,
        )
    )

    plot_confusion_matrix(
        test_labels,
        test_preds,
        class_names,
        "confusion_matrix_test_merged.png",
        "Test Confusion Matrix",
    )


if __name__ == "__main__":
    train()