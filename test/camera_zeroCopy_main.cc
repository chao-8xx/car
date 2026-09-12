#include "headfile.h"
/**************************************************************************
 * @brief       摄像头零拷贝读取
 * 
 * @note        Creation Time :         2026/01/11
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             摄像头零拷贝需要映射内存地址以及向内核申请注册
 *                                      缓冲队列,必须进行手动释放，因此修改ctrl+c和
 *                                      kill的信号用于正常关闭程序进行内存的释放，防止
 *                                      内存泄露
 ***************************************************************************/

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
    if(camera.init(320, 240, 60) < 0) {
        return 0;
    }


    cv::Mat frame;
    while(running)
    {
        if (!camera.capture_frame(frame, true)) {
            break;
        }

    }

    return 0;
}