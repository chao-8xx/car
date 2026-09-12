#ifndef __BASIC_CONTROL_H
#define __BASIC_CONTROL_H
#include "headfile.h"

/*** 函数声明 ***/
//tof初始化
void tof_Init(void);
//tof获取距离
void tof_distance_get(void);

//获取偏航角速度
void yaw_gyro_get(void);
//获取所有角
void all_angle_get(void);

//获取小车行使距离
void motor_distance_get(void);
//重置小车行驶距离
void motor_distance_clear(void);

//电机使能
void motor_enable_on(void);
//电机失能
void motor_enable_off(void);
//使能电机使能控制
void motor_enable_enable_control(void);
//使能电机使能倒计时(ms)
void motor_enable_count(int ms);

//状态复位
void state_reset(void);
//强行停车
void car_stop(void);
//智能车软停止
void car_soft_stop(void);
//超速保护
void overspeed_protect(void);
//智能车起步控制
void car_start(void);

//PID各个环的误差和输出清零
void pid_ErrOut_clear(void);
//LADRC误差清除
void ladrc_Err_clear(void);

//负压PWM占空比设置
void fans_duty_set(int duty);
//负压缓启
void fans_soft_start(int target, int time);
//负压状态复位
void fans_state_reset(void);
//负压占空比保护
void fans_protect(void);

#endif

