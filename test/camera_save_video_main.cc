/**************************************************************************
 * @brief       摄像头帧保存
 * 
 * @note        Creation Time :         2026/01/11
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             开启playback, 将图像原始数据缓存在内存中,
 *                                      程序结束时调用camera.playback_save();
 *                                      保存录像
 ***************************************************************************/

#include "headfile.h"

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
    //系统信号初始化
    system_init();
    
    Camera camera;
    camera.init(160, 120, 120);
    //初始化最大缓存帧
    camera.playback_init(120 * 10);
    //开始录制
    camera.playback_start();
    cv::Mat frame;
    while(running)
    {
        if(!camera.capture_frame(frame)) {
            break;
        }

    }
    //保存视频
    camera.playback_save();

    return 0;
}