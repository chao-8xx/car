#ifndef __CONTROL_DECISION_H
#define __CONTROL_DECISION_H
#include "headfile.h"

#define MAX_ERR     400.0f   //定义图像最大误差

typedef enum{
    Low,        //低速
    Mid_Low,    //中低速
    Mid,        //中速
    Mid_High,   //中高速度
    High,       //高速
    Stop        //停

} Speed_state;  //速度状态

/*** 函数声明 ***/
//负压控制决策
void fans_decision(void);
//速度前瞻决策
void speed_prospect_decision(void);
//前瞻决策（根据速度状态机决定前瞻）
void prospect_decision(void);
//简单前瞻决策(纯决策前瞻)
void simple_prospect_decision(void);
//蜂鸣器控制决策
void buzzer_decision(void);
//缓慢帧跳过决策（防止因图像刷新慢而导致绕行动作滞后）
void frame_pass_decision(void);
//速率限制器
void rate_limiter(float target, float &current, float max_step, float &accumulator);
//陀螺仪检测(发车前采样，观察是否数据乱跳)
void gyroscope_check(void);

//速度->角度双环控制
void double_circle_control(float tar_angle);  //可设置目标角度
//常规完赛控制
void common_control(void);
//控制决策实现
void control_decision(void);


#endif

