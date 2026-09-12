#include "pid.h"

//定义结构体
// PID servo_pid;
PID Lmotor_PID;     //左电机PID
PID Rmotor_PID;     //右电机PID
PID Photo_PID;      //图像环
PID Angle_PID;      //角速度环
PD_FF Angle_PID_F;  //角速度环
PID Temp_PID;       //临时角度环（偏航角）

//定义类
extern Motor motor;


/************************************普通PID************************************/
/**
 * ************************************************************************
 * @brief 增量式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxOutput  输出限幅值
 * 
 * ************************************************************************
 */
//增量式PID参数的初始化
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxOutput = maxOutput;
    pid->minOutput = minOutput;
}

/**
 * ************************************************************************
 * @brief 增量式PID控制器
 *          
 * @param[in] pid  pid指针
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
//增量式PID控制器
void Incremental_PID_Cal(PID *pid, float set_value,float get_value)
{
	pid->error = set_value - get_value;      									        //计算偏差
	pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + \
				pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			//增量式PI控制器
	pid->lastlastError = pid->lastError;    											//保存上上次误差
	pid->lastError = pid->error;	           											//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;                                                   //输出限幅
    else if (pid->output < pid->minOutput)
        pid->output = pid->minOutput;													//输出限幅

}


/**
 * ************************************************************************
 * @brief 位置式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxI  积分限幅值
 * @param[in] maxOutput  输出限幅值
 * 
 * ************************************************************************
 */
//位置式PID参数的初始化
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
}

/**
 * ************************************************************************
 * @brief 位置式PID控制器
 * 
 * @param[in] pid  pid结构体
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
//位置式PID控制器
void Positional_PID_Cal(PID *pid,float set_value, float get_value)
{
	float dout,pout;
    //更新数据
    pid->lastError = pid->error; 							//将旧error存起来
    pid->error = set_value - get_value; 					//计算新error

    //计算微分
    dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;

    //积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout + dout + pid->integral;
    //输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

//清除PID环的任何时刻误差和积分
void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->lastlastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

//PID左电机设置速度
void PID_Lmotor(int target)
{    
    //解算PID获得电机输出
    Positional_PID_Cal(&Lmotor_PID,target,L_speed);

    if(Lmotor_PID.output < 0) motor.set_motor1(1,-Lmotor_PID.output);
    else motor.set_motor1(0,Lmotor_PID.output);
}

//PID右电机设置速度
void PID_Rmotor(int target)
{
    //解算PID获得电机输出
    Positional_PID_Cal(&Rmotor_PID,target,R_speed);

    if(Rmotor_PID.output < 0) motor.set_motor2(1,-Rmotor_PID.output);
    else motor.set_motor2(0,Rmotor_PID.output);
}

//PID智能车平滑起步，防止电机猛转
float limit_p = 0.0f;
void PID_CarStart(float target, float now_value, int step, PID *left_speed, PID *right_speed)
{
    //参数保护
    if(step <= 0) return;

    //通过单边电机判断，先对预设值赋值，使两电机初始限幅为0，以便于平滑启动
    if(left_speed->maxOutput > limit_p)
    {
        limit_p = left_speed->maxOutput;
        left_speed->maxOutput = 0;
        right_speed->maxOutput = 0;
    }

    //PID的使用
    PID_Rmotor(target);
    R_pwm = Rmotor_PID.output;
    PID_Lmotor(target);
    L_pwm = Lmotor_PID.output;

    //左右电机限幅值根据步长缓慢上升，做到智能车平滑起步
    if(target - now_value > target / step)
    {
        //左右电机限幅值缓慢上升
        left_speed->maxOutput += limit_p/step;
        right_speed->maxOutput += limit_p/step;
        //防止电机实际限幅值超出预设值
        left_speed->maxOutput = left_speed->maxOutput > limit_p ? limit_p : left_speed->maxOutput; 
        right_speed->maxOutput = right_speed->maxOutput > limit_p ? limit_p : right_speed->maxOutput;
    }

}


/************************************PD+前馈************************************/
//PD+前馈控制器初始化
void PD_FF_Init(PD_FF* pd, float kp, float kd, float kff, float kff_acc, float max, float ms)
{
    pd->Kp = kp;
    pd->Kd = kd;
    pd->Kff = kff;
    pd->Kff_acc = kff_acc;

    pd->dt = ms/1000;  //输入毫秒

    pd->maxOutput = max;

    PD_FF_Reset(pd);
}

//PD+前馈控制状态清零
void PD_FF_Reset(PD_FF* pd)
{
    pd->error = 0.0f;
    pd->last_error = 0.0f;
    pd->last_target = 0.0f;
    pd->last_derivative = 0.0f;
    pd->last_target_acc = 0.0f;
    pd->output = 0.0f;
}

//PD+前馈 控制计算
void PD_FF_Cal(PD_FF* pd, float target, float actual)
{
    //防止采样周期过小
    if (pd->dt <= 0.0f) pd->dt = 0.001f;

    //误差
    pd->error = target - actual;

    //微分
    float derivative = (pd->error - pd->last_error) / pd->dt;

    //低通滤波
    float alpha = 0.8f;
    derivative = alpha * derivative + (1.0f-alpha) * pd->last_derivative;
    pd->last_derivative = derivative;

    //目标变化率
    float raw_target_acc = (target - pd->last_target) / pd->dt;

    //低通滤波
    float beta = 0.8f; 
    float filtered_target_acc = beta * raw_target_acc + (1.0f - beta) * pd->last_target_acc;
    pd->last_target_acc = filtered_target_acc;

    //前馈+反馈控制
    float feedforward_acc = pd->Kff_acc * filtered_target_acc;
    float feedforward = pd->Kff * target;
    float feedback = pd->Kp * pd->error + pd->Kd * derivative;

    pd->output = feedforward + feedback + feedforward_acc;    //输出

    //输出限幅
    if(pd->output > pd->maxOutput) pd->output = pd->maxOutput;
    else if(pd->output < -pd->maxOutput) pd->output = -pd->maxOutput;

    //更新状态
    pd->last_error = pd->error;
    pd->last_target = target;

}


/************************************高次项PID************************************/
//高次项PID初始化赋值(在普通PID赋值初始化之后使用)
void Higher::Higher_PID_Init(PID* pid)
{
    //位置式PID初始化赋值
    Positional_PID_Init(pid,pid->kp,pid->ki,pid->kd,pid->maxIntegral,pid->minOutput,pid->maxOutput);

    //Kp的高次项赋值
    Higher_Order_PID* higher_order_pid = (Higher_Order_PID*) pid; 

    higher_order_pid->kp_2 = 0.05;
    higher_order_pid->kp_3 = 0.01;
}

//高次项PID计算
void Higher::Higher_PID_Cal(PID* pid,float set_value,float get_value)
{
    //定义比例项、积分项、微分项输出
    float pout,iout,dout;

    //将输入的普通PID参数结构体转化为告次项PID参数结构体
    Higher_Order_PID* higher_order_pid = (Higher_Order_PID*) pid; 
    
    //更新数据
    higher_order_pid->base.lastError = higher_order_pid->base.error; 		//将旧error存起来
    higher_order_pid->base.error  = set_value - get_value; 					//计算新error

    //Kp 项
    if(higher_order_pid->base.kp)
    {
        pout = higher_order_pid->base.kp * higher_order_pid->base.error
            + higher_order_pid->kp_2 * abs(higher_order_pid->base.error) * higher_order_pid->base.error
            + higher_order_pid->kp_3 * higher_order_pid->base.error * higher_order_pid->base.error * higher_order_pid->base.error;

        pout = _scale(pout,-1000,1000);
    }
    else pout = 0;

    //Ki 项(梯形积分)
    if(higher_order_pid->base.ki)
    {
        if(preprocess.inner_count > 0)
        {
            iout += 0.5 * higher_order_pid->base.ki * 
                (higher_order_pid->base.lastError + higher_order_pid->base.error) * preprocess.inner_count;
        }
        else
        {
            iout += 0.5 * higher_order_pid->base.ki * higher_order_pid->base.error;
        }

        iout = _scale(iout,-higher_order_pid->base.maxIntegral,higher_order_pid->base.maxIntegral);
    }
    else iout = 0;

    //Kd 项（基于误差变化率）
    if(higher_order_pid->base.kd)
    {
        float din = 0.0f;   //定义误差变化率
        din = (higher_order_pid->base.error - higher_order_pid->base.lastError) / preprocess.inner_count;

        dout = higher_order_pid->base.kd * din;
        dout = _scale(dout,-2000,2000);
    }
    else dout = 0;

    //总输出
    higher_order_pid->base.output = pout + iout + dout;

    //输出限幅
    higher_order_pid->base.output = _scale(higher_order_pid->base.output,
        higher_order_pid->base.minOutput,higher_order_pid->base.maxOutput);
    
}

Higher higher_pid;


/************************************模糊PID************************************/
//kp规则表
const FuzzySet Kp_rule_list[7][7] = 
{ 
    {PB,PB,PM,PM,PS,ZO,ZO},        
	{PB,PB,PM,PS,PS,ZO,NS},
	{PM,PM,PM,PS,ZO,NS,NS},
	{PM,PM,PS,ZO,NS,NM,NM},
	{PS,PS,ZO,NS,NS,NM,NM},
	{PS,ZO,NS,NM,NM,NM,NB},
	{ZO,ZO,NM,NM,NM,NB,NB} 
};

//Kd规则表
const FuzzySet Kd_rule_list[7][7] = 
{
    {PS,NS,NB,NB,NB,NM,PS},
    {PS,NS,NB,NM,NM,NS,ZO},
    {ZO,NS,NM,NM,NS,NS,ZO},
    {ZO,NS,NS,NS,NS,NS,ZO},
    {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
    {PB,NS,PS,PS,PS,PS,PB},
    {PB,PM,PM,PM,PS,PS,PB}
};

//量化函数(转为模糊论域)
float Fuzzy::quantization(float x, float minimum, float maximum) //x：当前要模糊化的精确输入值（如当前的误差值)
{
	float qvalues= 12.0f *(x-minimum)/(maximum - minimum) - 6;  //映射到[-6, 6]
	return qvalues;
}

//输入 e 与 de/dt 隶属度计算函数
void Fuzzy::get_grad_membership(float error,float error_c)
{
    if (error > e_membership_values[0] && error < e_membership_values[6]) //判断error是否在预设的隶属度值范围之内
    {   
        for (int i = 0; i < num_area - 1; i++)  //遍历所有可能的区间
		{
			if (error >= e_membership_values[i] && error <= e_membership_values[i + 1])  //判断当前error是否落在第i个区间内
			{
                //计算 error 相对于该区间左右两个模糊集合的隶属度(左隶属度 + 右隶属度 = 1)
				e_gradmembership[0] = -(error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_gradmembership[1] = 1 + (error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                //记录这两个模糊集合的索引，并跳出循环
				e_grad_index[0] = i;
				e_grad_index[1] = i + 1;
				break;
			}
		}
    }
    else
    {
        if(error <= e_membership_values[0])
        {
            e_gradmembership[0] = 1;    //对第一个隶属度为1
			e_gradmembership[1] = 0;    //对第二个集合的隶属度为0（实际不存在第二个集合）
			e_grad_index[0] = 0;        //激活的第一个集合索引为0
			e_grad_index[1] = -1;       //第二个索引置为-1，表示无效
        }
        else if(error >= e_membership_values[6])
        {
            e_gradmembership[0] = 1;    //注意e_gradmembership[0]总是存储第一个激活集合的隶属度，此时第一个也是唯一一个就是索引6
			e_gradmembership[1] = 0;    //第二个隶属度为0
			e_grad_index[0] = 6;        //激活集合索引为6
			e_grad_index[1] = -1;       //第二个索引无效
        }
    }

    if (error_c > ec_membership_values[0] && error_c < ec_membership_values[6]) //判断error_c是否在预设的隶属度值范围之内
	{
		for (int i = 0; i < num_area - 1; i++)
		{
			if (error_c >= ec_membership_values[i] && error_c <= ec_membership_values[i + 1])
			{
				ec_gradmembership[0] = -(error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_gradmembership[1] = 1 + (error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_grad_index[0] = i;
				ec_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (error_c <= ec_membership_values[0])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 0;
			ec_grad_index[1] = -1;
		}
		else if (error_c >= ec_membership_values[6])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 6;
			ec_grad_index[1] = -1;
		}
	}

}

//将输入模糊化(计算输出增量的总隶属度)
void Fuzzy::get_sum_grad(void)
{
    //初始化Kp总的隶属度值为0
	for (int i = 0; i < num_area; i++)
	{
		KpgradSums[i] = 0.0f;   //将存放累加强度的数组全部初始化为0
        KdgradSums[i] = 0.0f;
	}

    //遍历误差激活的两个索引
    for(int i = 0; i < 2; i++)
    {
        if (e_grad_index[i] == -1)  //如果索引为-1,表示该侧无激活集合，则跳过
        {
            continue;
        }

        for(int j = 0; j < 2; j++)  //内层遍历误差变化率激活的索引
        {
            if (ec_grad_index[j] != -1)
            {
                //规则强度（乘积）
                float w = e_gradmembership[i] * ec_gradmembership[j];

                //Kp规则
                int kp_center = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]];    //获取规则结论（中心值)
                int kp_index = (kp_center + 6) / 2;   //将中心值映射为索引(0~6)
                KpgradSums[kp_index] += w;  //累加

                //Kd规则
                int kd_center = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]];
                int kd_index = (kd_center + 6) / 2;
                KdgradSums[kd_index] += w;  //累加

            }
            else
            {
                continue;
            }
        }

    }

}

//计算输出增量kp对应论域值,并计算最终模糊值
float Fuzzy::get_kp(void)
{
    numerator_kp = 0.0f;
    denominator_kp = 0.0f;

    for (int i = 0; i < num_area; i++)
	{
        float w_kp = KpgradSums[i];
		numerator_kp   += kp_membership_values[i] * w_kp;
        denominator_kp += w_kp;
	}
    float fuzzy_kp = (denominator_kp == 0) ? 0 : (numerator_kp / denominator_kp);
    return fuzzy_kp;
}

//计算输出增量kd对应论域值,并计算最终模糊值
float Fuzzy::get_kd(void)
{
    numerator_kd = 0.0f;
    denominator_kd = 0.0f;

    for (int i = 0; i < num_area; i++)
    {
        float w_kd = KdgradSums[i];
        numerator_kd   += kd_membership_values[i] * w_kd;   //kd也用同一套隶属度中心值
        denominator_kd += w_kd;
    }
    float fuzzy_kd = (denominator_kd == 0) ? 0 : (numerator_kd / denominator_kd);
    return fuzzy_kd;
}

//反区间映射函数(解模糊化)
float Fuzzy::inverse_quantization(float qvalues, float minimum, float maximum)
{
	float x = (maximum - minimum) *(qvalues + 6.0f) / 12.0f + minimum;
	return x;
}

//获取Kp增量(转向环)
float Fuzzy::PID_deltaKp(PID *pid)
{
    //1.计算当前误差(并更新误差)
    float error = pid->error;
    float error_c = error - lastError_Kp;  //误差变化率
    lastError_Kp = error;   //Kp和Kd各自用自己的上一次误差，以免一个测量的误差比另外一个慢一帧

    //2.量化
    float qe = quantization(error, ERR_MIN, ERR_MAX);
    float qec = quantization(error_c, EC_MIN, EC_MAX);

    //3.模糊推理（计算隶属度、规则累加、解模糊）
    get_grad_membership(qe, qec);
    get_sum_grad();          
    float fuzzy_kp = get_kp();  //返回[-6,6]的值

    //4.反量化得到Kp增量，并更新当前Kp
    float delta_kp = inverse_quantization(fuzzy_kp, KP_DELTA_MIN, KP_DELTA_MAX);
    
    return delta_kp;
}

float Fuzzy::PID_deltaKd(PID *pid)
{
    //1. 计算当前误差和误差变化率
    float error = pid->error;
    float error_c = error - lastError_Kd;   //保证与Kp用不同的lastError
    lastError_Kd = error;

    //2. 量化
    float qe = quantization(error, ERR_MIN, ERR_MAX);
    float qec = quantization(error_c, EC_MIN, EC_MAX);

    //3. 模糊推理（隶属度、规则累加、解模糊）
    get_grad_membership(qe, qec);
    get_sum_grad();             //同时累加Kp和Kd的强度
    float fuzzy_kd = get_kd();  //返回[-6,6]

    //4. 反量化得到Kd增量
    float delta_kd = inverse_quantization(fuzzy_kd, KD_DELTA_MIN, KD_DELTA_MAX);

    return delta_kd;
}

Fuzzy fuzzy;


