import torch
import torch.nn as nn

# ================= 配置 =================
input_size = 64
model_path = "best_model.pth"
onnx_name = "model.onnx"


# ================= 模型定义 (必须和训练时一模一样) =================
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

# ================= 执行导出 =================
def export():
    # 1. 初始化模型
    device = torch.device("cpu")  # 导出时用 CPU 即可
    model = Net(num_classes=3).to(device)

    # 2. 加载权重
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
        print(f"✅ 成功加载权重: {model_path}")
    except FileNotFoundError:
        print(f"❌ 错误: 找不到 {model_path}，请确认文件名")
        return

    model.eval()  # 切换到推理模式 (这一步极其重要！会关闭 Dropout)

    # 3. 创建虚拟输入 (1张图, 3通道, 64x64)
    dummy_input = torch.randn(1, 3, input_size, input_size).to(device)

    # 4. 导出 ONNX (修改版)
    print(f"正在导出为 {onnx_name} ...")

    # 为了防止旧文件干扰，我们换个新名字
    save_name = "model_frist.onnx"

    torch.onnx.export(model,
                      dummy_input,
                      save_name,  # 使用新文件名
                      export_params=True,  # 关键：必须显式设为 True，把权重存进去
                      input_names=['in0'],
                      output_names=['out0'],
                      opset_version=11)

    print(f"🎉 导出成功！请检查生成的 {save_name} 文件大小是否约为 2MB")
    print("------------------------------------------------")


if __name__ == "__main__":
    export()