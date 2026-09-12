#ifndef __ELEMENT_CONTROL_H
#define __ELEMENT_CONTROL_H
#include "headfile.h"

typedef enum{
    track_out,  //出赛道
    parallel,   //与赛道平行
    track_in,   //回赛道
    none        //空闲状态

} Pass_state;   //绕行状态枚举

typedef enum{
    left_pass,    //往左绕
    right_pass,   //往右绕
    no_pass       //不绕

} Pass_direction_state;   //绕行方向状态枚举

typedef enum{
    no_turn,    //不转
    turn_left,  //左转
    turn_right  //右转

} Turn_state;   //转弯状态


class Element_Ctrl{
public:

void curve_desion(void);        //弯道强度参数决策
void straight_control(void);    //直道
void safe_control(void);        //安全过渡状态
void curve_control(void);       //弯道
void crossing_control(void);    //十字
void ring_control(void);        //圆环
void model_control(void);       //模型
void normal_control(void);      //常规


void L_pass_control(float angle, float distance);   //左绕控制
void R_pass_control(float angle, float distance);   //右绕控制
void pass_control(Pass_direction_state *pass_dir,float angle, float distance);     //绕障控制
void dynamic_Lpass_control(float max_angle);        //动态左绕控制
void dynamic_Rpass_control(float max_angle);        //动态右绕控制
void dynamic_pass_control(Pass_direction_state *pass_dir,float angle);     //动态绕障控制


void turn_control(Turn_state *turn, float angle, float distance);   //转向控制


// void normal_parament_lock(void);    //正常参数锁定
// void straight_parament_lock(void);  //直道参数锁定
// void curve_parament_lock(void);     //弯道参数锁定
// void cross_parament_lock(void);     //十字参数锁定
// void ring_parament_lock(void);      //圆环参数锁定
// void model_parament_lock(void);     //模型参数锁定
// void parament_lock(void);           //参数锁定


float yaw_gyro_filter = 0.0f;    //角速度上次滤波后的值
float final_yaw_gyro = 0.0f;     //角速度滤波后的值


private:

float straight_new_kp = 0.0f;     //直道模糊后的参数
float straight_new_kd = 0.0f;

float safe_new_kp = 0.0f;         //安全过渡模糊后的参数
float safe_new_kd = 0.0f;

float curve_new_kp = 0.0f;        //弯道模糊后的参数
float curve_new_kd = 0.0f;

float crossing_new_kp = 0.0f;     //十字模糊后的参数
float crossing_new_kd = 0.0f;

float ring_new_kp = 0.0f;         //圆环模糊后的参数
float ring_new_kd = 0.0f;

float model_new_kp = 0.0f;        //模型模糊后的参数
float model_new_kd = 0.0f;

float new_Kp = 0.0f;              //常规控制模糊后的参数
float new_Kd = 0.0f;

float model_state_kp = 0.0f;      //模型子状态Kp

};

//类
extern Element_Ctrl element_ctrl;

//枚举(状态机)
extern Pass_state pass_state;
extern Pass_direction_state pass_direction;
extern Turn_state turn_state;

#endif
