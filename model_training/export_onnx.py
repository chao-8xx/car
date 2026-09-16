import torch
import torch.nn as nn

# ================= 配置 =================
input_size = 64
model_path = "best_model.pth"
onnx_name = "model.onnx"


# ================= 模型定义 (必须和训练时一模一样) =================
class Net(nn.Module):
    def __init__(self, num_classes=3):
        super(Net, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
        )
        self.classifier = nn.Sequential(
            nn.Linear(64 * 8 * 8, 128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
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