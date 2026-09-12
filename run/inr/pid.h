#ifndef  __CONTROL_PID_H
#define  __CONTROL_PID_H

// __CONTROL_PID_H

//普通PID结构体
typedef struct
{
    float kp,ki,kd;                         //三个系数
    float error,lastError,lastlastError;    //误差、上次误差、上上次误差
    float integral,maxIntegral;             //积分、积分限幅
    float output,maxOutput,minOutput;       //输出、输出限幅
} PID;

//PD+前馈控制结构体
typedef struct 
{
    float Kp;       //比例系数
    float Kd;       //微分系数
    float Kff;      //前馈系数
    float Kff_acc;  //前馈变化率系数

    float output;       //输出

    float maxOutput;    //输出上限

    float dt;           //采样周期

    //状态变量
    float error;              //本次误差
    float last_error;         //上一次的误差
    float last_target;        //上一次的目标角速度
    float last_derivative;    //上一次的微分项（用于低通滤波)
    float last_target_acc;    //上一次的目标角速度变化律（用于低通滤波）

} PD_FF;

//高次项PID扩展参数
typedef struct {    
    PID base;           //PID结构体定义，必须为第一个成员
    float kp_2;         //二次项Kp
    float kp_3;         //三次项Kp
} Higher_Order_PID;


//保证编译pid.h的时，优先编译PID结构体定义，避免编译器未识别到PID结构体后去编译其他用了PID结构体的地方，导致报错
#include "headfile.h"

/************************************普通PID************************************/
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput);
void Incremental_PID_Cal(PID *pid, float set_value, float get_value);

void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput);
void Positional_PID_Cal(PID *pid,float set_value, float get_value);

void PID_Reset(PID *pid);   //清除PID环的任何时刻误差、积分、输出

void PID_Lmotor(int target);
void PID_Rmotor(int target);
void PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed);


/************************************PD+前馈************************************/
void PD_FF_Init(PD_FF* pd, float kp, float kd, float kff, float kff_acc, float max, float ms);
void PD_FF_Reset(PD_FF* pd);
void PD_FF_Cal(PD_FF* pd, float target, float actual);


/************************************高次项PID************************************/
class Higher{
public:
    void Higher_PID_Init(PID* pid);
    void Higher_PID_Cal(PID* pid,float set_value,float get_value);

private:

};


/************************************模糊PID************************************/
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
    const float EC_MAX = 15.0f;         //误差变化率最大范围
    const float EC_MIN = -15.0f;
    const float KP_DELTA_MAX = 0.8f;    //Kp调节幅度（增量范围）
    const float KP_DELTA_MIN = -0.8f;
    const float KD_DELTA_MAX = 0.4f;    //Kd调节幅度
    const float KD_DELTA_MIN = -0.4f;

};


//声明结构体
extern PID Lmotor_PID; //左电机PID
extern PID Rmotor_PID; //右电机PID
extern PID Photo_PID;  //图像环
extern PID Angle_PID;  //角速度环
extern PD_FF Angle_PID_F;  //角速度环
extern PID Temp_PID;   //临时角度环（偏航角,避障、进圆环用）
extern PID servo_pid, makeup_pid;

//声明类
extern Higher higher_pid;
extern Fuzzy fuzzy;

#endif

