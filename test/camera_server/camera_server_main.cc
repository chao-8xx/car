#include "headfile.h"

/**************************************************************************
 * @brief       摄像头图传服务器例程
 * 
 * @note        Creation Time :         2025/12/19
 * @note        Author :                Blockingsys
 * @note        E-mail :                lty18455633351@163.com
 * 
 * @note        Attention :             本例程展示了如何使用摄像头图传服务器实现实时视频流传输:
 *                                      
 *                                      1. 服务器采用MJPEG格式通过HTTP协议传输视频流
 *                                      支持多客户端同时访问, 可在浏览器中实时查看
 *                                      
 *                                      2. 图传流使用低质量JPEG压缩以降低延迟
 *                                      同时保存原始帧用于高质量拍照
 *                                      
 *                                      3. 点击浏览器"拍照保存"按钮可下载原始高质量图片
 *                                      图片直接下载到客户端, 不占用开发板存储空间
 *                                      
 *                                      4. 访问方式: 在浏览器输入 http://<开发板IP>:8080
 *                                      即可查看实时画面并进行拍照操作
 *                                      
 *                                      5. 服务器在后台线程运行, 主线程负责采集摄像头
 *                                      数据并通过update_frame_mat()更新到服务器
 *                                          
 ***************************************************************************/


int main()
{
    //局部变量
    AsyncMusicPlayer music_player;
    CameraStreamServer camera_server(&music_player);
    // 1. 启动图传服务器（默认端口8080）
    if(camera_server.start_server(8080) < 0)
    {
        return -1;
    }
    
    // 等待服务器启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. 初始化摄像头（使用 0 拷贝 Camera，输出 JPEG 原始数据）
    Camera camera;
    if (camera.init(320, 240, 60) < 0) {
        return -1;
    }

    // 3. 设置摄像头参数（替代 OpenCV cap.set）
    // 注意：不同摄像头驱动支持的控制项/范围不同。
    {
    camera.set_contrast(60);  // 调整对比度到较高值，解决"灰蒙蒙"
    camera.set_sharpness(70);  // 提高锐度，解决"糊糊的"
    camera.set_brightness(10);  // 微调亮度，解决"太暗"
    camera.set_saturation(50);  // 提高饱和度，解决"发白"
    camera.set_gain(40);  // 提高增益，解决"噪点过多"
    camera.set_auto_exposure(false);  // 手动控制曝光（先关闭自动曝光）
    camera.set_exposure_absolute(50);  // 从默认156提高到300
    }

    // 4. 主循环：不断读取摄像头帧并更新到服务器（0拷贝 JPEG -> update_frame_jpeg）
    while(camera_server.is_running())
    {
        // 读取一帧：decode=false 不解码，仍然可以更新图传
        cv::Mat frame;
        if (!camera.capture_frame(frame, true)) {
            std::cerr << "Frame cannot be read" << std::endl;
            break;
        }
        // 更新到图传服务器（直接发送 JPEG 原始数据，避免 imencode）cpu占用率为30%
        camera_server.update_frame_jpeg(camera.jpeg_nowdata);

        // 翻转需要解码 翻转 再编码 cpu占用率为80%
        // cv::flip(frame, frame, -1); // 摄像头反装，翻转图像
        // camera_server.update_frame_mat(frame);
    }

    std::cout << "Program exited" << std::endl;
    return 0;
}