#ifndef __PTHREAD_CONTROL_H
#define __PTHREAD_CONTROL_H

/******************线程函数声明******************/
void pthread_start(void);       //启动多线程

void motor_ctrl(void *arg);     //主控制线程
void key_thread(void *arg);     //按键监听线程 
void enconder_get(void *arg);   //编码器获取线程
void gyro_cal(void *arg);       //陀螺仪解算线程
void vofa_ctrl(void *arg);      //vofa线程
// void tof_thread(void *arg);     //tof测距线程
// void music_thread(void *arg);   //音乐播放线程
// void remind_ctrl(void *arg);    //提醒控制

extern bool reset_state;

#endif
