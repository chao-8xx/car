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
from sklearn.metrics import confusion_matrix
import numpy as np
import os



# =================================== 配置区域 ===========================
input_size = 64         # 输入尺寸
batch_size = 256        # 训练批次
epochs = 100             # 迭代轮数
lr = 0.001              # 学习率

# 数据集路径
train_path = r"E:\code\Dataset_Split_mytrain\train"
val_path = r'E:/code/Dataset_Split_mytrain/val'
save_path = "best_model.pth"  # 模型权重保存路径

# CUDA GPU显卡查询
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


# =========================== 定义网络模型结构 ==============================
# 结构：Conv -> BN -> Relu -> MaxPool -> Conv -> BN -> Relu -> MaxPool -> Conv -> BN -> Relu -> FC
class Net(nn.Module):
    def __init__(self, num_classes = 3):
        super(Net, self).__init__()

        # 第一层 提取基础特征 (输入: 3通道, 输出: 16通道)
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(16),  # BN层 批归一化 加快收敛
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 64x64 -> 32x32

            # 第2层：提取形状特征
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 32x32 -> 16x16

            # 第3层：提取高级特征
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 16x16 -> 8x8
        )
        # 全连接层 (分类器)
        # 最后的特征图大小是 64通道 * 8 * 8
        self.classifier = nn.Sequential(
            nn.Linear(64 * 8 * 8, 128),     # 将 64 * 8 * 8 （4096维） 转换为 128维的特征向量
            nn.ReLU(),
            nn.Dropout(0.5),                           # 随机丢弃50%神经元 防止过拟合
            nn.Linear(128, num_classes)      # 128维特征向量 输出3个类别
        )

    # 前向传播
    def forward(self, x):
        x = self.features(x)        # 卷积计算
        x = x.view(x.size(0), -1)   # 展平，相当于 TF 的 Flatten()
        x = self.classifier(x)      # 分类
        return x


# ================= 2. 数据准备与增强 =================
# 训练集：要做数据增强（旋转、变色），让模型适应赛道环境
train_transforms = transforms.Compose([
    transforms.Resize((input_size, input_size)),
    # transforms.RandomHorizontalFlip(),  # 随机水平翻转  无需
    transforms.RandomRotation(15),  # 随机旋转15度
    transforms.ColorJitter(brightness=0.2, contrast=0.2),  # 随机亮度对比度
    transforms.ToTensor(),  # 转为 Tensor 并归一化到 [0,1]
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

train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)  # ImageFolder：自动从文件夹结构加载图像数据
val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)     # shuffle=True：训练时打乱数据顺序，防止模型记忆顺序

# 获取类别名称
class_names = train_dataset.classes
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


def plot_confusion_matrix(model, loader):
    """加载模型并绘制混淆矩阵"""
    print("正在生成混淆矩阵...")
    model.eval()
    all_preds = []
    all_labels = []

    with torch.no_grad():
        for images, labels in loader:
            images = images.to(device)
            outputs = model(images)
            _, preds = torch.max(outputs, 1)

            all_preds.extend(preds.cpu().numpy())
            all_labels.extend(labels.numpy())

    # 计算混淆矩阵
    cm = confusion_matrix(all_labels, all_preds)

    # 画图
    plt.figure(figsize=(8, 6))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names, yticklabels=class_names)
    plt.xlabel('Predicted Label')
    plt.ylabel('True Label')
    plt.title('Confusion Matrix')
    plt.savefig('confusion_matrix.png')
    print("🟦 混淆矩阵已保存为 confusion_matrix.png")


# ================= 4. 主训练循环 =================
def train():
    model = Net(num_classes=len(class_names)).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # --- 用于记录数据的列表 ---
    history_train_loss = []
    history_train_acc = []
    history_val_acc = []

    best_acc = 0.0

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
            loss = criterion(outputs, labels)
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
        model.eval()
        correct_val = 0
        total_val = 0
        with torch.no_grad():
            for images, labels in val_loader:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                _, predicted = torch.max(outputs.data, 1)
                total_val += labels.size(0)
                correct_val += (predicted == labels).sum().item()

        val_acc = 100 * correct_val / total_val
        history_val_acc.append(val_acc)

        print(
            f"Epoch [{epoch + 1}/{epochs}] | Loss: {epoch_loss:.4f} | Train Acc: {epoch_acc:.2f}% | Val Acc: {val_acc:.2f}%")

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), save_path)

    print("✅ 训练结束！")

    # --- 可视化环节 ---
    # 1. 画 Loss 和 Acc 曲线
    plot_history(history_train_loss, history_train_acc, history_val_acc)

    # 2. 加载最好的模型来画混淆矩阵
    best_model = Net(num_classes=len(class_names)).to(device)
    best_model.load_state_dict(torch.load(save_path))
    plot_confusion_matrix(best_model, val_loader)


if __name__ == '__main__':
    train()
