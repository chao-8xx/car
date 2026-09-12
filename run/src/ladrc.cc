#include "ladrc.h"

//定义结构体
LADRC right_motor;
LADRC left_motor;

//定义类
extern Motor motor;

extern bool is_lose;    //判断是否丢线

//初始化LADRC控制器，设置采样周期
void LADRC_Init(LADRC *ladrc,float Ts)
{
    if (ladrc == NULL || Ts <= 0) return; // 空指针和非法参数保护

    ladrc->r = 0.0f;   //跟踪系数

    //初始化需设置采样周期
    ladrc->Ts = Ts / 1000; //采样周期,Ts设置为毫秒

    ladrc->v1 = 0.0f;  //动态期望
    ladrc->v2 = 0.0f;  //微分信号

    //三个可调参数
    ladrc->wc = 0.0f;  //控制器带宽
    ladrc->w0 = 0.0f;  //观测器带宽
    ladrc->b = 0.0f;   //补偿系数

    ladrc->z1 = 0.0f;   //期望状态观测值(估计值)
    ladrc->z2 = 0.0f;   //期望状态微分观测值
    ladrc->z3 = 0.0f;   //总扰动

    ladrc->target = 0.0f;      //目标值
    ladrc->limit = 0.0f;       //输出限幅
    ladrc->last_out = 0.0f;    //上一次输出

    ladrc->integral = 0.0f;  // 积分初始化为0

}

//设置LADRC控制器目标值
void LADRC_SetTarget(LADRC *ladrc, float target)
{
    if (ladrc == NULL) return; // 空指针保护
    ladrc->target = target;
}

//设置LADRC的各个参数
//一般情况下 r=60、wc=20、w0=4*wc、b=30（实测电机增益）、limit=PWM限幅
void LADRC_SetParams(LADRC *ladrc, float r, float wc, float w0, float b, float limit)
{
    if (ladrc == NULL) return; // 空指针保护
    ladrc->r = r;
    ladrc->wc = wc;
    ladrc->w0 = w0;
    ladrc->b = b;
    ladrc->limit = limit;
}

void TD_Update(LADRC *ladrc, float target, float Ts, float r)
{
    //空指针和非法参数保护
    if (ladrc == NULL || Ts <= 0 || r < 0) return;

    //定义变量
    float h = Ts;                       //取样周期（步长）
    float x1 = ladrc->v1 - target;      //当前误差
    float x2 = ladrc->v2;               //微分信号

    //1.计算阈值
    float d = r * h;        //一级阈值
    float d0 = d * h;       //二级阈值

    //2.计算综合误差与最优加速度
    float y = x1 + h * x2;      //y为综合误差，为预测性误差，可提前调整
    //a0为最优加速度，该公式由二阶系统最速控制理论中推导
    float sqrt_arg = d*d + 8*r*fabs(y);
    float a0 = sqrt(fmax(sqrt_arg,0.0f));  //确保输入非负   

    //3.分段计算加速度a
    float sign_y = (y > 0) ? 1.0f : (y < 0) ? -1.0f : 0.0f;  // 添加符号
    float a;
    if(fabs(y) > d0)    //综合误差较大，需快速修正
    {
        a = x2 + (a0 - d)*sign_y / 2.0f;     //(a0 - d)/2.0f 为增量，由二阶系统最速控制理论推导
    }
    else                //综合误差较小，平滑收敛
    {
        a = x2 + y/h;               // y/h 线性化近似
    }

    //4.分段计算fst
    float fst;
    if(fabs(a) <= d)    //加速度在允许范围内
    {
        fst = -(r * a) / d;     //线性控制段
    }
    else
    {
        float sgn_a = (a>0) ? 1.0f : (a<0) ? -1.0f : 0.0f;  //符号函数，a>0时为1，a<0时为-1，a=0时为0
        fst = -r * sgn_a;       //饱和控制段
    }

    //状态更新(离散迭代)
    // 先计算新值
    float v1_new = ladrc->v1 + h * ladrc->v2;   // v1(t+Ts) = v1(t) + Ts * v2(t)
    float v2_new = ladrc->v2 + h * fst;         // v2(t+Ts) = v2(t) + Ts * fst(v1(t)-v0(t),v2(t),r,d)

    // 再统一更新
    ladrc->v1 = v1_new;
    ladrc->v2 = v2_new;           

}

//fal函数（非线性ESO用）
float fal(float e, float alpha, float delta)    //e表示估计值与测量值的误差，alpha表示函数幂次，delta表示函数线性区间
{
    if (delta <= 0) delta = 0.05f;  //delta保护（避免小于等于0）

    float abs_e = fabs(e);      //abs_e表示误差的绝对值
    if(abs_e > delta)       //误差大时
    {
        float sgn_e = (e>0) ? 1.0f : (e<0) ? -1.0f : 0.0f;  //符号函数，e>0时为1，e<0时为-1，e=0时为0
        return pow(abs_e,alpha) * sgn_e;    //fal函数为非线性，避免超调
    }
    else        //误差小时
    {
        float denom = pow(delta,1-alpha);
        return e/(denom > 0 ? denom : 1.0f);        //fal函数为线性，保证平滑(分母为0时用1代替，保护作用)
    }
}

//ESO更新
//alpha1给0.5，alpha2给0.25，delta给0.05。可自行修改
void ESO_Update(LADRC *ladrc, float measure, float out,float Ts,float b)
{
    //空指针和非法参数保护
    if (ladrc == NULL || Ts <= 0 || ladrc->w0 <= 0) return;

    //定义变量
    float w0 = ladrc->w0;
    float e1 = ladrc->z1 - measure;     //定义误差

    // //fal函数的值(非线性时使用)
    // float fal1 = fal(e1,0.5f,0.05f);
    // float fal2 = fal(e1,0.25f,0.05f);
    // float fal3 = fal(e1,0.125f,0.05f);

    // //非线性计算下一时刻观测器的状态**（大误差快，小误差稳）
    // float z1_new = ladrc->z1 + Ts * (ladrc->z2 - 3*w0 * fal1);                //更新z1状态，beta01 = 3*w0
    // float z2_new = ladrc->z2 + Ts * (ladrc->z3 - 3*w0*w0 * fal2 + b*out);     //更新z2状态，beta02 = 3*w0*w0
    // float z3_new = ladrc->z3 - Ts * w0*w0*w0 * fal3;                          //更新z3状态，beta03 = w0*w0*w0

    //线性计算下一时刻观测器的状态**(不用微分,直接计算误差和总扰动即可)
    float z1_new = ladrc->z1 + Ts * (ladrc->z3 - 2.0f * w0 * e1 + b * out);           
    // float z2_new = ladrc->z2 + Ts * (ladrc->z3 - 3.0f * w0 * w0 * e1);      
    float z3_new = ladrc->z3 - Ts * (w0 * w0 * e1);

    //更新状态
    ladrc->z1 = z1_new;
    // ladrc->z2 = z2_new;
    ladrc->z3 = z3_new;
}

//LADRC状态更新
float LADRC_Update(LADRC *ladrc, float measure)     //measure表示测量得出的值（实际值）
{
    if (ladrc == NULL) return 0.0f; // 空指针保护

    //定义变量
    float Ts = ladrc->Ts;           //采样周期（步长）
    float wc = ladrc->wc;           //控制器带宽
    float w0 = ladrc->w0;           //观测器带宽
    float b = ladrc->b;             //补偿系数
    float target = ladrc->target;   //目标值
    float limit = ladrc->limit;     //输出限幅

    // //更新跟踪微分器TD,TD平滑目标(TD平滑会使相应变慢，不符合智能车使用场景)
    // TD_Update(ladrc, ladrc->target, ladrc->Ts, ladrc->r);

    //直接让v1=目标值，v2=0（模拟无平滑的TD）
    ladrc->v1 = ladrc->target;
    ladrc->v2 = 0.0f;

    //ESO观测
    ESO_Update(ladrc, measure, ladrc->last_out, ladrc->Ts, ladrc->b);

    //控制律计算
    //1.定义误差
    float e1 = ladrc->v1 - ladrc->z1;   //目标值 - 估计值
    // float e2 = ladrc->v2 - ladrc->z2;

    // //2.非线性状态反馈
    // //定义fal函数（非线性控制率使用）
    // float fal_p = fal(e1,0.5f,0.05f);
    // float fal_d = fal(e2,0.75f,0.05f);
    
    // 给LADRC加极弱积分，加大精度
    float ki = 1.2f;    //极弱积分，避免超调
    ladrc->integral += (ladrc->target - ladrc->z1) * ladrc->Ts;     // 积分计算（改用结构体里的积分项）
    ladrc->integral = fmax(fmin(ladrc->integral, 25.0f), -25.0f);   // 限幅

    // //3.非线性计算核心控制量out0**（LADRC控制律为线性，不用非线性）
    // float out0 = wc * fal_p + 2.0f*wc * fal_d;

    //3.线性控制律**(更加稳)
    // float out0 = wc * wc * e1 + 2.0f * wc * e2;     // 标准线性"PD"控制律
    float out0 = wc * (ladrc->target - ladrc->z1) + ki * ladrc->integral;     // 标准线性"P+扰动补偿"控制律

    //4.扰动补偿
    float out = 0.0f;
    if(b != 0)      //避免除0
    {
        out = (out0 - ladrc->z3) / b;       // u = (u0 - z3) / b
    } 
    else
    {
        out = 0.0f;     //若b为0，则输出为0
    }

    //输出处理
    //1.输出限幅
    if(out > limit)
    {
        out = limit;
    }
    else if(out < -limit)
    {
        out = -limit;
    }
    //2.保存本次输出（输出状态更新）
    ladrc->last_out = out;

    return out;
}

//LADRC电机参数初始化(执行一边即可，需要在读取json数据后使用)
void LADRC_motor_init(void)
{
    LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);  //设置右电机参数
    LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);   //设置左电机参数
}

//LADRC清除误差
void LADRC_Reset(LADRC* ladrc)
{
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->z3 = 0.0f;
    ladrc->integral = 0.0f;
}

//LADRC控制算法控制左电机转速
void LADRC_Lmotor(float speed)
{
    //LADRC的使用
    LADRC_SetTarget(&left_motor,speed);           //将speed的值定为目标值
    L_pwm = LADRC_Update(&left_motor,L_speed);    //将编码器读数作为实际测量值，经过算法变为PWM输出值

    //定义左边电机旋转
    if(L_pwm < 0) motor.set_motor1(1,-L_pwm); //电机反转
    else motor.set_motor1(0,L_pwm);
    
}

//LADRC控制算法控制右电机转速
void LADRC_Rmotor(float speed)
{
    //LADRC的使用
    // LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);  //设置参数
    LADRC_SetTarget(&right_motor,speed);           //将speed的值定为目标值
    R_pwm = LADRC_Update(&right_motor,R_speed);    //将编码器读数作为实际测量值，经过算法变为PWM输出值

    //定义右边电机旋转
    if(R_pwm < 0) motor.set_motor2(1,-R_pwm); //电机反转
    else motor.set_motor2(0,R_pwm);
    
}

//LADRC智能车平滑起步（防起步猛转）
float limit_t = 0.0f;
void LADRC_CarStart(float target, float now_value, int step, LADRC *left_speed, LADRC *right_speed)
{
    //参数保护
    if(step <= 0) return;

    //LADRC的使用
    LADRC_SetTarget(left_speed,target);          //将target的值定为目标值
    L_pwm = LADRC_Update(left_speed,L_speed);    //将编码器读数作为实际测量值，经过算法变为PWM输出值
    LADRC_SetTarget(right_speed,target);         //将speed的值定为目标值
    R_pwm = LADRC_Update(right_speed,R_speed);   //将编码器读数作为实际测量值，经过算法变为PWM输出值

    //通过单边电机判断，先对预设值赋值，使两电机初始限幅为0，以便于平滑启动
    if(left_speed->limit > limit_t)
    {
        limit_t = left_speed->limit;
        left_speed->limit = 0;
        right_speed->limit = 0;
    }

    //电机起步
    if(L_pwm >= 0) motor.set_motor1(0,L_pwm);   //左电机
    else if(L_pwm < 0) motor.set_motor1(1,-L_pwm);
    if(R_pwm >= 0) motor.set_motor2(0,R_pwm);   //右电机
    else if(R_pwm < 0) motor.set_motor2(1,-R_pwm);

    //左右电机限幅值根据步长缓慢上升，做到智能车平滑起步
    if(target - now_value > target / step)
    {
        //左右电机限幅值缓慢上升
        left_speed->limit += limit_t/step;
        right_speed->limit += limit_t/step;
        //防止电机实际限幅值超出预设值
        left_speed->limit = left_speed->limit > limit_t ? limit_t : left_speed->limit; 
        right_speed->limit = right_speed->limit > limit_t ? limit_t : right_speed->limit;
    }

}


