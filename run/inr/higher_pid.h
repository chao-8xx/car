#ifndef __HIGHER_PID_H
#define __HIGHER_PID_H
#include "headfile.h"

//高次项PID扩展参数
typedef struct {    
    PID base;           //PID结构体定义，必须为第一个成员
    float kp_2;         //二次项Kp
    float kp_3;         //三次项Kp
} Higher_Order_PID;

class Higher{
public:
void Higher_PID_Init(PID* pid);
void Higher_PID_Cal(PID* pid,float set_value,float get_value);

private:

};

extern Higher higher;

#endif
