//摄像头测试历程程序

#include "headfile.h"

using namespace cv;


int main()
{
    AsyncMusicPlayer music_player;
    CameraStreamServer camera_server(&music_player);
    if(camera_server.start_server(8080) < 0)
    {
        return -1;
    }
    // 等待服务器启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 视觉部分
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        std::cerr << "Camera cannot be opened" << std::endl;
        return -1;
    }

    // 设置摄像头参数
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));  
    cap.set(CAP_PROP_FRAME_WIDTH, 320);     
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CAP_PROP_FPS, 60);              

    
    cap.set(CAP_PROP_BRIGHTNESS, 100);      // 亮度 
    cap.set(CAP_PROP_CONTRAST, 32);         // 对比度 
    cap.set(CAP_PROP_SHARPNESS, 70);        // 清晰度
    cap.set(CAP_PROP_SATURATION, 64);       // 饱和度 
//  cap.set(CAP_PROP_GAIN, 74);             // 增益 
    cap.set(CAP_PROP_EXPOSURE, 3);          // 自动曝光

    // 定义帧变量
    Mat img, bin_img;

    while(camera_server.is_running())
    {
        cap >> img;
        if(img.empty()) 
        {
            std::cerr << "Frame cannot be read" << std::endl;
            break;
        }

        // 摄像头反装 翻转图像
        flip(img, img, -1);
 
        cvtColor(img, bin_img, COLOR_BGR2GRAY);
        threshold(bin_img, bin_img, 0, 255, THRESH_BINARY | THRESH_OTSU);
        

        camera_server.update_frame_mat(bin_img);
    }
    
    cap.release();
    std::cout << "Program exited" << std::endl;
    return 0;
}

