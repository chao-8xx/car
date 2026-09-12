#ifndef __LADRC_H
#define __LADRC_H
#include "headfile.h"

typedef struct{
    
    //快速跟踪因子
    float r;    //跟踪系数，越大跟踪越快

    //采样周期，用于离散化
    float Ts;   //采样周期

    //跟踪微分器参数
    float v1;   //动态期望
    float v2;   //微分信号

    //控制器参数
    float wc;   //控制器带宽
    float w0;   //观测器带宽
    float b;    //补偿系数，b越大，输出增益越小

    //扩张状态观测器(ESO)观测值
    float z1;   //期望状态观测值（与原始期望单位一致）
    float z2;   //期望状态微分观测值（与微分信号单位一致）
    float z3;   //总扰动

    //目标值、输出限幅、上一次输出
    float target;   //目标值
    float limit;    //输出限幅
    float last_out; //上一次输出

    float integral;   //每个电机独立的积分项

} LADRC;

//LADRC函数的框架
void LADRC_Init(LADRC *ladrc,float Ts);
void LADRC_SetTarget(LADRC *ladrc, float target);
void LADRC_SetParams(LADRC *ladrc, float r, float wc, float w0, float b, float limit);  
void TD_Update(LADRC *ladrc, float target, float Ts, float r);
float fal(float e, float alpha, float delta);
void ESO_Update(LADRC *ladrc, float measure, float out,float Ts,float b);
float LADRC_Update(LADRC *ladrc, float measure);
//LADRC函数的使用
void LADRC_motor_init(void); //LADRC电机参数初始化(执行一边即可，需要在读取json数据后使用)
void LADRC_Reset(LADRC* ladrc); //LADRC清除误差
void LADRC_Rmotor(float speed);
void LADRC_Lmotor(float speed);
void LADRC_CarStart(float target, float now_value, int step, LADRC *left_speed, LADRC *right_speed);

//声明结构体
extern LADRC right_motor;
extern LADRC left_motor;

//声明变量

#endif

