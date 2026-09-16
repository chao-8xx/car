# 自定义模型训练 推理 对常规卷积神经网络进行简单的三层卷积
# 使用曲线图和混淆矩阵图画出最佳参数训练下的 loss/acc 图像
# 确保已按照之前的步骤准备好了数据集

import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import classification_report, confusion_matrix
import numpy as np
import os
from io import BytesIO
from PIL import Image



# =================================== 配置区域 ===========================
input_size = 64         # 输入尺寸
batch_size = 256        # 训练批次
epochs = 100             # 迭代轮数
lr = 0.001              # 学习率
weight_decay = 1e-4
early_stop_patience = 15
label_smoothing = 0.05  # 标签平滑系数；三分类建议先用0.05

# 数据集路径
train_path = r"E:\code\Dataset_Split_mytrain_new\crop_output_v20_all\train"
val_path = r"E:\code\Dataset_Split_mytrain_new\crop_output_v20_all\val"
test_path = r"E:\code\Dataset_Split_mytrain_new\crop_output_v20_all\test"
EXPECTED_CLASS_ORDER = ["supplies", "transport", "weapon"]
save_path = "best_model.pth"  # 模型权重保存路径

# CUDA GPU显卡查询
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


# =========================== 定义网络模型结构 ==============================
# 定义深度可分离卷积模块 (这是提速的终极秘密)
class DWConv(nn.Module):
    def __init__(self, in_ch, out_ch, stride=1):
        super(DWConv, self).__init__()
        # 第一步：Depthwise 卷积 (负责提取空间特征，分组计算，极度省算力)
        self.dw = nn.Conv2d(in_ch, in_ch, kernel_size=3, stride=stride,
                            padding=1, groups=in_ch, bias=False)
        self.bn1 = nn.BatchNorm2d(in_ch)

        # 第二步：Pointwise 卷积 (1x1卷积，负责融合通道特征)
        self.pw = nn.Conv2d(in_ch, out_ch, kernel_size=1, stride=1,
                            padding=0, bias=False)
        self.bn2 = nn.BatchNorm2d(out_ch)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        x = self.relu(self.bn1(self.dw(x)))
        x = self.relu(self.bn2(self.pw(x)))
        return x


# ================= 新的模型定义 (专为边缘 CPU 压榨算力设计) =================
class Net(nn.Module):
    def __init__(self, num_classes=3):
        super(Net, self).__init__()

        # 输入 3x64x64
        self.features = nn.Sequential(
            # 第一层保留标准卷积，因为输入只有3通道，DW卷积没有意义
            nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1, bias=False),  # -> 16x32x32
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),

            # 后面全部换成深度可分离卷积模块
            DWConv(16, 32, stride=2),  # -> 32x16x16
            DWConv(32, 64, stride=2),  # -> 64x8x8
            DWConv(64, 128, stride=2),  # -> 128x4x4
        )

        # 杀手锏：全局平均池化。干掉全连接层，直接把 4x4 浓缩成 1x1，彻底消灭参数！
        self.global_pool = nn.AdaptiveAvgPool2d(1)

        # 最后的分类器，只有 128x3 = 384 个参数
        self.classifier = nn.Sequential(
            nn.Dropout(0.2),  # 减少过拟合风险
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)  # 输出 shape: [batch, 128, 4, 4]
        x = self.global_pool(x)  # 输出 shape: [batch, 128, 1, 1]
        x = x.view(x.size(0), -1)  # 展平 shape: [batch, 128]
        x = self.classifier(x)  # 输出 shape: [batch, 3]
        return x


# ================= 2. 数据准备与增强 =================
# 训练集：要做数据增强（旋转、变色），让模型适应赛道环境
class RandomMotionBlur:
    def __init__(self, p=0.4, kernel_size=(3, 9)):
        self.p = p
        self.kernel_size = kernel_size

    def __call__(self, img):
        if torch.rand(1).item() >= self.p:
            return img

        k = int(torch.randint(
            self.kernel_size[0],
            self.kernel_size[1] + 1,
            (1,)
        ).item())

        # 保证奇数
        if k % 2 == 0:
            k += 1

        # 水平方向运动模糊（模拟车辆前进）
        kernel = np.zeros((k, k))
        kernel[k // 2, :] = np.ones(k) / k

        kernel = torch.tensor(
            kernel,
            dtype=torch.float32
        ).unsqueeze(0).unsqueeze(0)

        img_tensor = transforms.functional.to_tensor(img)

        img_tensor = nn.functional.conv2d(
            img_tensor.unsqueeze(0),
            kernel.repeat(3,1,1,1),
            padding=k//2,
            groups=3
        ).squeeze(0)

        img_tensor = torch.clamp(img_tensor,0,1)

        return transforms.functional.to_pil_image(img_tensor)

class RandomJpegCompression:
    def __init__(self, p=0.25, quality=(35, 85)):
        self.p = p
        self.quality = quality

    def __call__(self, img):
        if torch.rand(1).item() >= self.p:
            return img
        quality = int(torch.randint(self.quality[0], self.quality[1] + 1, (1,)).item())
        buffer = BytesIO()
        img.save(buffer, format="JPEG", quality=quality)
        buffer.seek(0)
        return Image.open(buffer).convert("RGB")


class RandomSensorNoise:
    def __init__(self, p=0.25, std=0.025):
        self.p = p
        self.std = std

    def __call__(self, tensor):
        if torch.rand(1).item() >= self.p:
            return tensor
        noise = torch.randn_like(tensor) * self.std
        return torch.clamp(tensor + noise, 0.0, 1.0)


train_transforms = transforms.Compose([
    transforms.Resize((input_size, input_size)),
    transforms.RandomAffine(
        degrees=10,
        translate=(0.05, 0.05),
        scale=(0.95, 1.05),
    ),
    transforms.ColorJitter(brightness=0.25, contrast=0.25, saturation=0.15),
    RandomMotionBlur(p=0.35),transforms.RandomApply([transforms.GaussianBlur(kernel_size=3,sigma=(0.1, 0.8))],p=0.15),
    RandomJpegCompression(p=0.25, quality=(35, 85)),
    transforms.ToTensor(),  # 转为 Tensor 并归一化到 [0,1]
    RandomSensorNoise(p=0.25, std=0.025),
    transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])  # 标准化到[-1,1]
])

# 验证集：只做 Resize，保持原样测试
val_transforms = transforms.Compose([
    transforms.Resize((input_size, input_size)),
    transforms.ToTensor(),
    transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])
])

# 加载数据
train_dataset = datasets.ImageFolder(train_path, transform=train_transforms)
val_dataset = datasets.ImageFolder(val_path, transform=val_transforms)
test_dataset = datasets.ImageFolder(test_path, transform=val_transforms)

train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)  # ImageFolder：自动从文件夹结构加载图像数据
val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)     # shuffle=True：训练时打乱数据顺序，防止模型记忆顺序
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# 获取类别名称
class_names = train_dataset.classes
if class_names != EXPECTED_CLASS_ORDER:
    raise RuntimeError(
        f"class order mismatch: got {class_names}, expected {EXPECTED_CLASS_ORDER}"
    )
print(f"类别映射: {train_dataset.class_to_idx}")


# ================= 3. 绘图功能函数 =================
def plot_history(train_losses, train_accs, val_accs):
    """绘制 Loss 和 Accuracy 曲线"""
    plt.figure(figsize=(12, 5))

    # 子图1: Loss
    plt.subplot(1, 2, 1)
    plt.plot(train_losses, label='Train Loss', color='red')
    plt.title('Training Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True)

    # 子图2: Accuracy
    plt.subplot(1, 2, 2)
    plt.plot(train_accs, label='Train Acc', color='blue')
    plt.plot(val_accs, label='Val Acc', color='green')
    plt.title('Accuracy Curve')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy (%)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('training_results.png')  # 保存图片
    print("📊 曲线图已保存为 training_results.png")
    # plt.show() # 如果你想看弹窗，可以取消注释


def evaluate(model, loader, criterion):
    model.eval()
    all_preds = []
    all_labels = []
    running_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for images, labels in loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            loss = criterion(outputs, labels)
            _, preds = torch.max(outputs, 1)

            running_loss += loss.item() * labels.size(0)
            correct += (preds == labels).sum().item()
            total += labels.size(0)
            all_preds.extend(preds.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())

    avg_loss = running_loss / max(1, total)
    acc = 100 * correct / max(1, total)
    return avg_loss, acc, all_preds, all_labels


def plot_confusion_matrix(labels, preds, output_path, title):
    """绘制混淆矩阵"""
    print(f"正在生成混淆矩阵: {output_path}")

    cm = confusion_matrix(labels, preds)

    # 画图
    plt.figure(figsize=(8, 6))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names, yticklabels=class_names)
    plt.xlabel('Predicted Label')
    plt.ylabel('True Label')
    plt.title(title)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


# ================= 4. 主训练循环 =================
def train():
    model = Net(num_classes=len(class_names)).to(device)

    # 训练阶段使用 Label Smoothing Cross Entropy。
    # 作用：防止模型对模糊、透视、目标不完整等困难样本过度自信。
    train_criterion = nn.CrossEntropyLoss(
        label_smoothing=label_smoothing
    )

    # 验证和测试仍使用普通交叉熵，使 val/test loss 更容易解释和比较。
    # 准确率本身不受这个设置影响。
    eval_criterion = nn.CrossEntropyLoss()

    print(f"Label smoothing: {label_smoothing}")

    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode="max",
        factor=0.5,
        patience=5,
    )

    # --- 用于记录数据的列表 ---
    history_train_loss = []
    history_train_acc = []
    history_val_acc = []

    best_acc = 0.0
    best_epoch = 0
    no_improve_epochs = 0

    for epoch in range(epochs):
        # --- 训练 ---
        model.train()
        running_loss = 0.0
        correct_train = 0
        total_train = 0

        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)

            optimizer.zero_grad()
            outputs = model(images)
            loss = train_criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            _, predicted = torch.max(outputs.data, 1)
            total_train += labels.size(0)
            correct_train += (predicted == labels).sum().item()

        epoch_loss = running_loss / len(train_loader)
        epoch_acc = 100 * correct_train / total_train

        # 记录训练数据
        history_train_loss.append(epoch_loss)
        history_train_acc.append(epoch_acc)

        # --- 验证 ---
        val_loss, val_acc, _, _ = evaluate(model, val_loader, eval_criterion)
        history_val_acc.append(val_acc)
        scheduler.step(val_acc)
        current_lr = optimizer.param_groups[0]["lr"]

        print(
            f"Epoch [{epoch + 1}/{epochs}] | Loss: {epoch_loss:.4f} | "
            f"Train Acc: {epoch_acc:.2f}% | Val Loss: {val_loss:.4f} | "
            f"Val Acc: {val_acc:.2f}% | LR: {current_lr:.6f}")

        if val_acc > best_acc:
            best_acc = val_acc
            best_epoch = epoch + 1
            no_improve_epochs = 0
            torch.save(model.state_dict(), save_path)
        else:
            no_improve_epochs += 1

        if no_improve_epochs >= early_stop_patience:
            print(f"Early stopping at epoch {epoch + 1}. Best epoch: {best_epoch}, best val acc: {best_acc:.2f}%")
            break

    print(f"✅ 训练结束！Best epoch: {best_epoch}, best val acc: {best_acc:.2f}%")

    # --- 可视化环节 ---
    # 1. 画 Loss 和 Acc 曲线
    plot_history(history_train_loss, history_train_acc, history_val_acc)

    # 2. 加载最好的模型来画混淆矩阵
    best_model = Net(num_classes=len(class_names)).to(device)
    best_model.load_state_dict(torch.load(save_path, map_location=device))

    val_loss, val_acc, val_preds, val_labels = evaluate(best_model, val_loader, eval_criterion)
    print(f"Best model val acc: {val_acc:.2f}% | val loss: {val_loss:.4f}")
    plot_confusion_matrix(val_labels, val_preds, "confusion_matrix_val.png", "Validation Confusion Matrix")

    test_loss, test_acc, test_preds, test_labels = evaluate(best_model, test_loader, eval_criterion)
    print(f"Best model test acc: {test_acc:.2f}% | test loss: {test_loss:.4f}")
    print(classification_report(test_labels, test_preds, target_names=class_names, digits=4))
    plot_confusion_matrix(test_labels, test_preds, "confusion_matrix_test.png", "Test Confusion Matrix")


if __name__ == '__main__':
    train()
