#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#include "headfile.h"

#define num_area  7

typedef enum {
    NB = -6,  // 负大
    NM = -4,  // 负中
    NS = -2,  // 负小
    ZO = 0,   // 零
    PS = 2,   // 正小
    PM = 4,   // 正中
    PB = 6    // 正大
} FuzzySet;

extern const FuzzySet Kp_rule_list[7][7];  //kp规则表
extern const FuzzySet Kd_rule_list[7][7];   //Kd规则表

class Fuzzy{
public:

float quantization(float x, float minimum, float maximum);               //量化函数
void get_grad_membership(float error,float error_c);                     //输入 e 与 de/dt 隶属度计算函数
void get_sum_grad(void);                                                 //将输入模糊化
float get_kp(void);                                                      //计算输出增量kp对应论域值
float get_kd(void);                                                      //计算输出增量kd对应论域值
float inverse_quantization(float qvalues, float minimum, float maximum); //反区间映射函数(解模糊化) 

float PID_deltaKp(PID *pid);   //获取Kp增量
float PID_deltaKd(PID *pid);   //获取Kd增量

private:

//输入隶属度计算相关数组
int e_grad_index[2];          //误差激活的两个模糊集合索引
int ec_grad_index[2];         //误差变化率激活的两个模糊集合索引
float e_gradmembership[2];    //误差的隶属度
float ec_gradmembership[2];   //误差变化率的隶属度

//模糊集合的隶属度函数中心值（或顶点）数组
float e_membership_values[7] = {-6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f};
float ec_membership_values[7] = {-6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f};
float kp_membership_values[7] = {-6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f};
float kd_membership_values[7] = {-6.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f};

//用于存储 Kp 规则累加强度的数组（每个输出变量一个）
float KpgradSums[7];
//用于存储 Kd 规则累加强度的数组
float KdgradSums[7];

float numerator_kp;      // kp 分子（加权和）
float denominator_kp;    // kp 分母（总权重）
float numerator_kd;      // Kd 分子（加权和）
float denominator_kd;    // Kd 分母（总权重）

//为Fuzzy类增加两个成员变量,防止误差更新后而这调用误差时间不同导致偏差
float lastError_Kp = 0.0f;
float lastError_Kd = 0.0f;

//设定实际物理量的范围
const float ERR_MAX = 50.0f;        //误差最大范围
const float ERR_MIN = -50.0f;
const float EC_MAX = 10.0f;         //误差变化率最大范围
const float EC_MIN = -10.0f;
const float KP_DELTA_MAX = 0.5f;    //Kp调节幅度（增量范围）
const float KP_DELTA_MIN = -0.5f;
const float KD_DELTA_MAX = 0.2f;    //Kd调节幅度
const float KD_DELTA_MIN = -0.2f;

};

extern Fuzzy fuzzy;

#endif


