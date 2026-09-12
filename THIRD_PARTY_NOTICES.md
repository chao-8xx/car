# 第三方组件与来源说明

本仓库包含智能车应用代码、WUWU 外设封装以及若干预编译依赖。再分发时请同时遵守各组件自己的许可证。

## 项目代码

- `run/`：智能车视觉、状态机、控制决策和线程代码。
- `WuwuSama_Icar_Project/smartCar/`：WUWU 智能车外设封装。
- 项目代码的许可证声明见根目录 `LICENSE`。

## 主要第三方组件

| 组件 | 用途 | 目录 / 说明 |
| --- | --- | --- |
| OpenCV | 图像采集、图像处理和编码 | `WuwuSama_Icar_Project/cross_lib/opencv/` |
| ncnn | 端侧神经网络推理 | `WuwuSama_Icar_Project/cross_lib/ncnn/` |
| jsoncpp | JSON 参数读取与保存 | `WuwuSama_Icar_Project/cross_lib/json/` |
| OpenMP / gomp | 并行计算 | 由交叉工具链提供 |
| Linux / V4L2 / pthread | 设备、视频和线程接口 | 目标系统提供 |

## 模型资源

训练数据、标注、训练脚本和原始权重不在本仓库中。`model_packed.ncnn.param` 和 `model_packed.ncnn.bin` 属于端侧部署资源，应通过项目 Release 或团队指定地址获取。

如果后续发布模型，请同时说明模型来源、训练数据授权、导出工具链、类别顺序、版本和 SHA256。

## 上游许可核对

发布二进制镜像、预编译库或修改后的第三方代码前，请以对应组件随附的许可证文件和官方项目说明为准。本说明不替代第三方组件本身的许可证文本。

