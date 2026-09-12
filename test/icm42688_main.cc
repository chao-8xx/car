#include "headfile.h"

/**************************************************************************
 * @brief       定时线程对姿态传感器读取数据 主线程显示 例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             这个例程中创建了一个10ms的定时线程, 在创建的线程中
 *                                      更新icm42688对象中的陀螺仪和加速度计数据, 在主线程
 *                                      调用LCD显示六轴数据, 此做法存在数据竞争, 需要使用
 *                                      线程同步机制保证数据安全, 库中已经封装好同步操作, 
 *                                      参考本例程的使用方法。
 * 
 ***************************************************************************/

//创建全局变量
LCD lcd;
ICM42688 icm42688;

//声明函数
void getIMU_Data(void *arg);

int main() {  

    //务必使用前进行初始化
    lcd.lcd_init();
    icm42688.icm42688_init();

    //创建陀螺仪读取线程(10ms)执行一次 getIMU_Data 内的内容
    TimerThread getIMU_thread(getIMU_Data, NULL, 10);
    //启动线程
    getIMU_thread.start();

    while(1) 
    {
        //线程同步
        icm42688.thread_syn();
        lcd.showDouble(0, 0*8, icm42688.syn_gyro_x, 5, 2);
        lcd.showDouble(0, 1*8, icm42688.syn_gyro_y, 5, 2);
        lcd.showDouble(0, 2*8, icm42688.syn_gyro_z, 5, 2);

        lcd.showDouble(0, 4*8, icm42688.syn_accel_x, 2, 4);
        lcd.showDouble(0, 5*8, icm42688.syn_accel_y, 2, 4);
        lcd.showDouble(0, 6*8, icm42688.syn_accel_z, 2, 4);

    }
      
    return 0;
}

//姿态传感器数据读取线程函数
void getIMU_Data(void *arg)
{
    icm42688.upDataAcc();
    icm42688.upDataGyro();
}

