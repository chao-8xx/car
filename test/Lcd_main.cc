#include "headfile.h"

/**************************************************************************
 * @brief       LCD FrameBuffer屏幕显示例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             给出常用显示功能，如果需要额外功能参考,
 *                                      参考Linux FrameBuffer编程
 * 
 ***************************************************************************/

//创建全局变量
LCD lcd;

int main() {  

    //务必使用前进行初始化
    lcd.lcd_init();

    //显示字符
    lcd.showChar(0, 0*8, 'a');
    //显示字符串
    lcd.showString(0, 1*8, "hello world");
    //显示整数
    lcd.showInt(0, 2*8, 100, 3);
    //显示浮点数
    lcd.showDouble(0, 3*8, 100.725, 3, 3);
    //创建一个全黑图像并显示
    lcd.showCVImage(0, 4*8, cv::Mat::zeros(30, 30, CV_8UC1), 30, 30);

    while(1) 
    {
        //while加阻塞释放cpu资源
        sleep(6);

    }
      
    return 0;
}