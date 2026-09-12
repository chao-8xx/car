#include "headfile.h"

/**************************************************************************
 * @brief       摄像头图传服务器例程 - 零拷贝模式（高性能纯图传）
 * 
 * @note        Creation Time :         2026/1/30
 * @note        Author :                Blockingsys
 * @note        E-mail :                lty18455633351@163.com
 * 
 * @note        功能说明：
 *              本例程实现基于V4L2内存映射的零拷贝高性能图传
 *              直接传输相机硬件JPEG数据，跳过所有编解码步骤
 *              
 * @note        技术特点：
 *              1. 使用 V4L2 mmap 直接访问相机DMA缓冲区
 *              2. 零拷贝技术：数据在用户空间和内核空间共享，无需拷贝
 *              3. 跳过JPEG解码和编码步骤，极大降低CPU占用
 *              4. 不能进行OpenCV图像处理（数据为JPEG字节流，非像素矩阵）
 *              
 * @note        性能指标：
 *              - 帧率: 120fps+ (320x240分辨率)
 *              - 内存占用: 约9-10MB RSS (仅3个60KB DMA缓冲区)
 *              - CPU占用: 约2% (几乎无计算开销)
 *              - 延迟: <20ms (端到端)
 *              
 * @note        数据流程：
 *              摄像头硬件 → [JPEG编码] → DMA缓冲区(内核) 
 *              ↓
 *              mmap映射(零拷贝) → 用户空间指针 → HTTP图传 → 浏览器
 *              
 * @note        技术原理：
 *              零拷贝的关键是mmap()系统调用，它将内核DMA缓冲区映射到
 *              用户空间，使得用户程序可以直接访问硬件数据，无需CPU拷贝
 *              
 *              
 * @note        重要提示：
 *              ⚠️ 零拷贝模式下不能进行图像处理！
 *              - 如果需要处理图像，必须手动解码：
 *                cv::Mat frame = cv::imdecode(camera.jpeg_nowdata, cv::IMREAD_COLOR);
 *              
 *              
 **************************************************************************/

 //-----------------------------这个千万不要删掉哇----------------------------------
static bool running = true;
static void sigint_handler(int sign) {running = false;}
static void system_init(void)
{
    //修改ctrl+c 和 kill的信号处理内容
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
}
//-----------------------------这个千万不要删掉哇----------------------------------

int main()
{
    system_init();
    
    Camera camera;
    if(camera.init(320, 240, 60) < 0) {
        std::cerr << "摄像头初始化失败" << std::endl;
        return -1;
    }

    AsyncMusicPlayer music_player;
    CameraStreamServer camera_server(&music_player);
    // 启动图传服务器（默认端口8080）
    if(camera_server.start_server(8080) < 0)
    {
        std::cerr << "图传服务器启动失败" << std::endl;
        return -1;
    }
    
    // 等待服务器启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

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

    int frame_count = 0;
    cv::Mat frame;
    cv::Mat flip_frame;
    // 4. 主循环：不断读取摄像头帧并更新到服务器（0拷贝 JPEG -> update_frame_jpeg）
    while(running && camera_server.is_running())
    {
        // 读取一帧：decode=false 不解码，使用camera_server.update_frame_jpeg时为false
        if (!camera.capture_frame(frame, true)) 
        { 
            std::cerr << "摄像头捕获失败" << std::endl;
            break;
        }
        // 更新到图传服务器（直接发送 JPEG 原始数据，避免 imencode）cpu占用率为1%
        // camera_server.update_frame_jpeg(camera.jpeg_nowdata);

        // 翻转需要解码 翻转 再编码 cpu占用率为90%
        cv::flip(frame, flip_frame, -1); // 摄像头反装，翻转图像
        
        // 3帧发送一帧 降低cpu占用率 现在cpu占用率为60%
        if(frame_count % 3 == 0) 
        camera_server.update_frame_mat(flip_frame);
        frame_count++;
    }
    return 0;
}

