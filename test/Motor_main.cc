#include "headfile.h"

/**************************************************************************
 * @brief       电机定时周期控制例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             定时器线程内本身就是一个死循环，只需要创建
 *                                      执行的功能函数即可，请勿在加一层死循环，会
 *                                      导致延时函数失效，失去定时器功能!!!!!!!
 * 
 ***************************************************************************/

//创建全局变量
Motor motor;
//声明控制线程
void motor_ctrl(void *arg);

int main() {  

    //电机初始化(务必要初始化)
    motor.motor_init();

    //创建电机控制线程(500ms)执行一次 motor_ctrl 内的内容
    TimerThread motorThread(motor_ctrl, NULL, 500);
    //启动线程
    motorThread.start();

    while(1) 
    {
        //while加阻塞释放cpu资源
        sleep(6);

    }
      
    return 0;
}

void motor_ctrl(void *arg)
{
    //局部静态变量
    static int level = 0;
    static int duty = 1000;

    //设置电机1的电平和占空比
    motor.set_motor1(level, duty);
    motor.set_motor2(!level, duty);

    //一次周期翻转一次
    level = !level;
    duty = (duty <= 1000) ? 5000 : 1000;
}
