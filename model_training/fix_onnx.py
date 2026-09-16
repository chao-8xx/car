import onnx
import os

# 你的文件名
input_file = "model_frist.onnx"
output_file = "model_packed.onnx"

print(f"正在读取 {input_file} ...")

# 1. 加载模型 (onnx 会自动去找同目录下的 .data 文件，只要它们在一起)
if not os.path.exists(input_file):
    print("❌ 错误：找不到文件，请确认你已经运行了导出代码，并且文件就在当前目录下。")
else:
    model = onnx.load(input_file)

    # 2. 强制保存为一个单文件 (save_as_external_data 默认为 False)
    onnx.save(model, output_file)

    print(f"✅ 修复完成！")
    print(f"请检查生成的 {output_file}，它应该是一个约 2MB 的单文件。")