#include "headfile.h"

/**************************************************************************
 * @brief       编码器例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             本例程展示了在两个线程中同时访问相同数据时的不同做法: 
 *                                      1.读取在本线程, 使用在本线程, 无须考虑资源竞争问题
 * 
 *                                      2.读取在其他线程, 使用在本线程, 则需要考虑资源
 *                                      竞争, 当然在wuwu库内部已经处理了这种问题, 在
 *                                      此种状态下, 调用thread_syn, 并使用对应syn_xxx
 *                                      即可保证数据安全
 *                                          
 ***************************************************************************/

//全局变量
Motor motor;
LCD lcd;

//声明函数
void get_encoder(void *arg);

int main() 
{  
    //务必使用前进行初始化
    motor.motor_init();
    // lcd.lcd_init();

    //创建编码器读取线程(10ms)执行一次 get_encoder 内的内容
    TimerThread buzzer_thread(get_encoder, NULL, 10);
    //启动线程
    buzzer_thread.start();

    while(1) 
    {
        //线程同步
        motor.thread_syn();
        // lcd.showInt(0, 0*8, motor.syn_encoder1_counts, 5);
        // lcd.showInt(0, 1*8, motor.syn_encoder2_counts, 5);

        //当无屏幕时，在终端中测试
        printf("Encoder1: %d  Encoder2: %d\r\n", motor.syn_encoder1_counts, motor.syn_encoder2_counts);
    }
      
    return 0;
}


void get_encoder(void *arg)
{
    motor.update_encoders();
    lcd.showInt(0, 3*8, motor.encoder1_counts, 5);
    lcd.showInt(0, 4*8, motor.encoder2_counts, 5);
}