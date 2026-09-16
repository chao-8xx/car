# 智能车神经网络训练工程

本目录保存智能车视觉分类模型的数据处理、训练、测试和模型导出文件。

## 目录说明

- `enhance/`：数据裁剪、分组、增强和合并脚本。
- 根目录 Python 文件：训练、测试、数据集构建和 ONNX 导出脚本。
- `models/run/`：按训练批次归档的 PyTorch、ONNX 和 NCNN 模型。
- `models/int8/`：NCNN INT8 量化结果和量化表。

## 数据集边界

照片数据集、增强图片、裁剪输出和 INT8 校准图片体积较大，不进入 Git。

原始数据仍保留在本地训练目录中。`.gitignore` 已排除常见照片格式、数据集目录、缓存以及重复的智能车源码副本。

## 使用注意

部分脚本保留了原训练环境中的 Windows 绝对路径。换电脑或调整目录后，需要先修改脚本开头的数据集输入、输出和模型路径。

常用依赖包括：

- PyTorch、torchvision
- OpenCV、NumPy、Pillow
- matplotlib、seaborn、scikit-learn
- ONNX、pytest

## 部署模型

智能车运行程序加载以下通用文件名：

- `model_packed.ncnn.param`
- `model_packed.ncnn.bin`

部署时应从已确认的训练版本中选择对应文件，复制到车端程序运行目录。本目录中的多个模型版本属于训练成果归档，不代表全部都已完成实车验证。
