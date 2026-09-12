#ifndef __MADGWICK_CONTROL_H
#define __MADGWICK_CONTROL_H
#include "headfile.h"

//将陀螺仪姿态解算算法封装成函数并组合成一个类
class Madgwick_Ctrl{
public:
    void madgwickCtrl_init(void);       //陀螺仪姿态解算的初始化
    void pose_cal(void);                //姿态解算算法，需要在线程里不停地算（因为数据也是不停地在读取）
    void angle_reset(void);             //角度复位
    void yaw_angle_reset(void);         //偏航角度复位
    float pitch_get(void);              //读取俯仰角
    float roll_get(void);               //读取翻滚角
    float yaw_get(void);                //读取偏航角
    void getIMU_Data(void);             //读取陀螺仪数据（需要进线程反复读取)

private:

};

extern Madgwick_Ctrl madg;

#endif



