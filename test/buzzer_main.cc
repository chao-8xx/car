#include "headfile.h"

/**************************************************************************
 * @brief       无源蜂鸣器例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             这里例程我设计了一个小技巧提供给大家学习, 在创建
 *                                      定时器线程的时候提供了三个参数传入, 第一个是定时
 *                                      任务, 第二个是给这个任务传入参数, 这里在主线程里
 *                                      面创建了一个局部变量buzzer, 想要在buzzer_ctrl
 *                                      函数中使用, 可以像例程一样通关传入局部变量的地址
 *                                      从而在buzzer_ctrl函数中通过指针的方式访问主线程
 *                                      的局部变量
 ***************************************************************************/


//声明函数
void buzzer_ctrl(void *arg);

int main() 
{  
    //局部变量
    Buzzer buzzer;
    //务必使用前进行初始化
    buzzer.buzzer_init();

    //创建蜂鸣器控制线程(10ms)执行一次 buzzer_ctrl 内的内容
    TimerThread buzzer_thread(buzzer_ctrl, &buzzer, 10);
    //启动线程
    buzzer_thread.start();

    while(1) 
    {
        //阻塞让出CPU资源
        sleep(6);
    }
      
    return 0;
}

//无源蜂鸣器控制线程
void buzzer_ctrl(void *arg)
{
    Buzzer *buzzer = (Buzzer *)arg;

    static int hz = 500;
    static int dir = 1;
    
    buzzer->set_duty_freq(50, hz);

    hz += dir;
    if(hz > 1000) dir = -1;
    else if(hz < 501) dir = 1;
}