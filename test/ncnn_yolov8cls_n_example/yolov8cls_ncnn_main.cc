// yolov8cls_ncnn_main.cc
// 基于 ncnn 的 yolov8-cls 目标检测模型的程序示例
// 结合图传显示实时识别的类别 置信度与帧率信息
#include "headfile.h"

// --- 配置区域 ---
// 类别名字 (必须和 metadata.yaml 对应)
static const char *class_names[] = {
    "supplies",
    "transport",
    "weapon"
};

// 简单的 Softmax 函数，把输出变成概率
void softmax(std::vector<float>& input) {
    float sum = 0.0f;
    for (float val : input) {
        sum += std::exp(val);
    }
    for (int i = 0; i < input.size(); i++) {
        input[i] = std::exp(input[i]) / sum;
    }
}

int main()
{
    // ================= 1. 初始化部分 =================
    
    // 1.1 启动图传服务器
    CameraStreamServer camera_server;
    if(camera_server.start_server(8080) < 0) {
        fprintf(stderr, "Error: Start server failed!\n");
        return -1;
    }
    // 等待服务器启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 1.2 加载 NCNN 模型
    ncnn::Net net;
    // 建议使用绝对路径，防止路径错误
    if (net.load_param("model.ncnn.param") == -1 ||
        net.load_model("model.ncnn.bin") == -1) {
        fprintf(stderr, "Error: Load model failed!\n");
        return -1;
    }
    
    // 优化：使用你的板子支持的线程数
    // net.opt.num_threads = 4; 

    // 1.3 初始化摄像头 (这里使用 OpenCV VideoCapture 方便处理)
    // 注意：原来的 camera_server_main.cc 用的是自定义 Camera 类
    // 如果那个 Camera 类能输出 cv::Mat，也可以用。
    // 为了稳妥，这里先用 OpenCV 打开，兼容性最好。
    cv::VideoCapture cap(0);
    // 降低分辨率以提高 NCNN 推理速度 (如 320x240)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 60);

    cap.set(cv::CAP_PROP_BRIGHTNESS, 100);      // 亮度 
    cap.set(cv::CAP_PROP_CONTRAST, 32);         // 对比度 
    cap.set(cv::CAP_PROP_SHARPNESS, 70);        // 清晰度
    cap.set(cv::CAP_PROP_SATURATION, 64);       // 饱和度 
    cap.set(cv::CAP_PROP_GAIN, 74);             // 增益 

    if (!cap.isOpened()) {
        fprintf(stderr, "Error: Open camera failed!\n");
        return -1;
    }

    std::cout << "Server running at http://<IP>:8080" << std::endl;

    cv::Mat frame;
    
    // FPS 计算变量
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    float current_fps = 0.0f;

    // ================= 2. 主循环 =================
    while (true)
    {
        // 2.1 获取图像
        cap >> frame;
        if (frame.empty()) break;

        // 摄像头反装 翻转图像
        cv::flip(frame, frame, -1);

        // 2.2 NCNN 推理
        // -------------------------------------------------
        int target_size = 128; // YOLOv8-cls 输入尺寸
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(
            frame.data,
            ncnn::Mat::PIXEL_BGR2RGB,
            frame.cols, frame.rows,
            target_size, target_size
        );

        const float mean_vals[3] = {0.f, 0.f, 0.f};
        const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
        in.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = net.create_extractor();
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);

        std::vector<float> scores;
        scores.resize(out.w);
        for (int j = 0; j < out.w; j++) scores[j] = out[j];
        softmax(scores);

        int max_class_idx = 0;
        float max_prob = 0.0f;
        for (int j = 0; j < scores.size(); j++) {
            if (scores[j] > max_prob) {
                max_prob = scores[j];
                max_class_idx = j;
            }
        }
        // -------------------------------------------------

        // 2.3 计算 FPS
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed >= 1000) { // 每秒更新一次 FPS
            current_fps = frame_count * 1000.0f / elapsed;
            frame_count = 0;
            last_time = now;
            // 在终端也可以打印一下监控
            // std::cout << "FPS: " << current_fps << " | Class: " << class_names[max_class_idx] << std::endl;
        }

        // 2.4 在图上绘制结果 (OSD)
        const char* label_text = class_names[max_class_idx];
        char text_buffer[256];
        
        // 绘制 FPS
        sprintf(text_buffer, "FPS: %.1f", current_fps);
        cv::putText(frame, text_buffer, cv::Point(10, 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

        // 绘制 识别结果
        sprintf(text_buffer, "%s: %.1f%%", label_text, max_prob * 100);
        // 根据置信度改变颜色：高信度绿色，低信度红色
        cv::Scalar color = (max_prob > 0.7) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(frame, text_buffer, cv::Point(10, 50), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

        // 2.5 发送给图传服务器
        // update_frame_mat 会把 cv::Mat 编码成 JPEG 并推流
        camera_server.update_frame_mat(frame);

        // 稍微休眠一下防止 CPU 占用过高 (可选，NCNN一般已经占满了)
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    camera_server.stop_server();
    return 0;
}