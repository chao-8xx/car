import torch
import torch.nn as nn
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import os
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix

# ================= 配置区 =================
model_path = r"E:\code\Dataset_Split_mytrain\run\train_3_perfect\best_model.pth"  # 你的模型权重文件
test_dir = r"E:\code\Dataset_Split_mytrain\test"  # 你的测试图片文件夹路径 (请改成你真实的路径)
input_size = 64


# ==========================================

# 必须和训练时完全一致的网络结构
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




def run_written_test():
    if not os.path.exists(test_dir):
        print(f"❌ 找不到测试文件夹: {test_dir}，请先建好文件夹并放入图片！")
        return

    # 1. 加载模型
    device = torch.device("cpu")
    model = Net(num_classes=3).to(device)
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
        print("✅ 模型权重加载成功，准备开始笔试...")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    model.eval()  # 开启考试模式 (关闭 Dropout)

    # 2. 准备数据处理 (缩放到64x64，转为Tensor)
    transform = transforms.Compose([
        transforms.Resize((input_size, input_size)),
        transforms.ToTensor(),
        transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])  # 标准化到[-1,1]
        # 注意：如果你训练时加了 Normalize，这里也必须加上！
    ])

    # 3. 加载测试集
    test_dataset = datasets.ImageFolder(root=test_dir, transform=transform)
    test_loader = DataLoader(test_dataset, batch_size=1, shuffle=False)

    class_names = test_dataset.classes
    print(f"📌 发现的类别顺序: {class_names}")  # 这行极其重要！可以检查类别顺序是不是错了

    # 4. 逐张图片进行“考试”
    all_preds = []
    all_labels = []

    print("\n================ 📄 正在进行笔试并收集数据 ================")
    with torch.no_grad():
        for inputs, labels in test_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, predicted = torch.max(outputs, 1)

            all_preds.extend(predicted.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())

    # 计算准确率
    correct = sum(1 for x, y in zip(all_preds, all_labels) if x == y)
    total = len(all_labels)
    print(f"🏆 最终得分: {correct}/{total} ({100 * correct / total:.2f}%)")

    # ================= 🎨 绘制混淆矩阵 =================
    cm = confusion_matrix(all_labels, all_preds)

    plt.figure(figsize=(8, 6))
    # 使用 seaborn 画热力图，颜色越深代表数量越多
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names,
                yticklabels=class_names)

    plt.title(f'Confusion Matrix (Accuracy: {100 * correct / total:.2f}%)')
    plt.ylabel('True Label (真实的类别)')
    plt.xlabel('Predicted Label (模型猜的类别)')

    # 保存图片到本地
    plt.tight_layout()
    plt.savefig('confusion_matrix.png', dpi=300)
    print("✅ 混淆矩阵已生成！请查看当前目录下的 'confusion_matrix.png'")
    plt.show()

if __name__ == "__main__":
    run_written_test()