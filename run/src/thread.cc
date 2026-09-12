#include "thread.h"

//定义/声明 类
VL53L0X tof;
extern Motor motor;

//定义锁
std::mutex motor_enable_mutex;          //电机使能互斥锁
std::mutex motor_enable_enable_mutex;   //使能电机使能互斥锁
std::mutex yaw_get_mutex;               //偏航角获取互斥锁
std::mutex pitch_get_mutex;             //俯仰角获取互斥锁
std::mutex distance_get_mutex;          //距离获取互斥锁

//声明变量
static bool timer_started = false;      //设置标志位，防止在平滑起步期间反复计时

extern Speed_state speed_state;             //速度状态
extern Pass_state pass_state;               //绕障状态
extern Pass_direction_state pass_direction; //绕障方向
extern Turn_state turn_state;               //转向方向

extern bool align_mid;     //智能车是否对准中线
extern bool pass_ready;    //是否准备绕行
extern bool pass_on;       //是否开启绕行状态

extern float limit_t;      //LADRC平滑起步输出限制变量
extern float limit_p;      //PID平滑起步输出限制变量

extern float incircle_current_yaw;  //用于记录圆环内识别到图片时当前的偏航角

extern bool was_pass_active;             //上一次的绕行状态
extern bool continue_pass;               //是否继续绕行
extern float final_photo_err;            //绕行过渡时最终的图像误差
extern uint32_t pass_model_time;         //绕行过度时间(ms)
extern bool err_smooth;                  //是否启动误差平滑

//tof初始化
void TOF_Init(void)
{
    //初始化
    if (tof.tof_init() < 0) 
    {
        std::cerr << "TOF 初始化失败" << std::endl;
    }
}

//tof获取距离
void tof_distance_get(void)
{
    tof.upData();
    tof_distance = tof.vl53l0x_distance_mm;
}

//获取偏航角速度
void yaw_gyro_get(void)
{
    madg.getIMU_Data();               //获取角速度、角加速度
    icm42688.thread_syn();            //线程同步  
    yaw_gyro = icm42688.syn_gyro_z - gyro_z_offset;   //读取的偏航角速度赋值(弧度转角度,减去零漂)
}

//获取偏航角
void yaw_angle_get(void)
{
    // madg.getIMU_Data();           //获取角速度、角加速度
    // madg.pose_cal();              //角度解算
    yaw_get_mutex.lock();
    yaw_angle = madg.yaw_get();   //获取偏航角
    yaw_get_mutex.unlock();
}

//重置偏航角
void yaw_clear(void)
{
    yaw_get_mutex.lock();
    yaw_angle = 0;
    yaw_get_mutex.unlock();
}

//获取俯仰角
void pitch_angle_get(void)
{
    // madg.getIMU_Data();               //获取角速度、角加速度
    // madg.pose_cal();                  //角度解算
    pitch_get_mutex.lock();
    pitch_angle = madg.pitch_get();   //获取偏航角
    pitch_get_mutex.unlock();
}

//重置俯仰角
void pitch_clear(void)
{
    pitch_get_mutex.lock();
    pitch_angle = 0;
    pitch_get_mutex.unlock();
}

//获取智能车行驶距离
void motor_distance_get(void)
{
    // //读取编码器数据
    // motor.update_encoders();     //跑车时外部刷新，读取一次即可，内部不用刷新

    //取二者电机行驶距离的平均值就是智能车行驶距离
    distance_get_mutex.lock();
    motor_distance += ((-motor.encoder1_counts * 1.0f) + (-motor.encoder2_counts * 1.0f)) / 392.0f;   //单位cm  
    distance_get_mutex.unlock();
}

//未发车时获取行驶距离
void distance_test(void)
{
    //读取编码器数据
    motor.update_encoders();

    //根据实际情况定义电机速度
    float left_speed = -motor.encoder1_counts * 1.0f;
    float right_speed = -motor.encoder2_counts * 1.0f;
    
    //取二者电机行驶距离的平均值就是智能车行驶距离
    distance_get_mutex.lock();
    motor_distance += (left_speed + right_speed) / 392.0f;     
    distance_get_mutex.unlock();
}

//智能车重置行驶距离
void motor_distance_clear(void)
{
    distance_get_mutex.lock();
    motor_distance = 0;         //智能车行驶距离清零
    distance_get_mutex.unlock();
}

//电机使能启动
void motor_enable_on(void)
{
    //判断使能标志位
    if(motor_enable) return;

    //开启电机使能
    motor_enable_mutex.lock();
    motor_enable = true;        //使能开
    motor_enable_mutex.unlock();
}

//电机失能
void motor_enable_off(void)
{
    //判断使能标志位
    if(! motor_enable) return;

    //关闭电机使能
    motor_enable_mutex.lock();
    motor_enable = false;       //使能关
    motor_enable_mutex.unlock();
}

//使能电机使能控制
void motor_enable_enable_control(void)
{
    //如果使能电机使能为false不启用
    if(!motor_enable_enable) return ;

    //如果使能电机使能为true,且电机使能计数器未到0（>0）
    if(motor_enable_enable && motor_enable_time > 0)
    {
        motor_enable_enable_mutex.lock();
        motor_enable_time--;    //计时减
        motor_enable_enable_mutex.unlock();
    }  
    //如果使能电机使能为true,且电机使能计数器到0（<=0）
    else if(motor_enable_enable && motor_enable_time <= 0)
    {
        motor_enable_enable_mutex.lock();
        motor_enable_enable = false;
        motor_enable_enable_mutex.unlock();
        motor_enable_on();  //电机使能
    }

}

//使能电机使能倒计时(ms)
void motor_enable_count(int ms)
{
    motor_enable_enable_mutex.lock();
    motor_enable_time = ms / Ts;  //电机控制线程为Ts ms调用一次(可改)
    motor_enable_enable = true;
    motor_enable_enable_mutex.unlock();
}

//状态复位
void state_reset(void)
{
    // 标注队列存在时，每次新发车恢复完整序列　运行中的出队只消耗内存队列
    if (model_calib.has_saved_queue()) {
        model_calib.load_saved_queue();
    }

    //起步状态复位
    timer_started = false;  //起步计时状态复位
    is_start = 1;           //起步状态置1（为1表示正在起步或起步前，0为起步完成）
    limit_t = 0.0f;         //LADRC平滑起步输出限制变量
    limit_p = 0.0f;         //PID平滑起步输出限制变量

    //电机使能状态复位
    motor_enable_enable_mutex.lock();
    motor_enable_time = 0;          //电机使能时间
    motor_enable_enable = false;    //使能电机使能
    motor_enable_enable_mutex.unlock();
    motor_enable_mutex.lock();
    motor_enable = false;           //电机使能
    motor_enable_mutex.unlock();

    //陀螺仪状态复位
    is_angle = false;                       //陀螺仪零漂状态为false（表示未完成初始化）
    element_ctrl.yaw_gyro_filter = 0.0f;    //角速度上次滤波后的值
    element_ctrl.final_yaw_gyro = 0.0f;     //角速度滤波后的值
    yaw_lowpass.reset(0.0f);                //角速度低通滤波初始化
    yaw_reset = true;                       //偏航角中线复位（对准中线时偏航角复位）

    //转向、绕行状态复位（上半为控制绕行，下半为视觉绕行）
    speed_state = Stop;        //初始化速度状态为停车
    pass_state = none;         //初始化绕障状态为无
    pass_direction = no_pass;  //初始化绕障方向为不绕障
    turn_state = no_turn;      //初始化转向状态为不转向
    align_mid = false;         //智能车是否对准中线
    pass_ready = false;        //是否准备绕行
    pass_on = false;           //是否开启绕行状态
    incircle_current_yaw = 0.0f;   //用于记录圆环内识别到图片时当前的偏航角
    
    photo_pass.reset(0.0f);            //图像误差低通滤波初始化 
    final_photo_err = 0.0f;            //绕行过渡时最终的图像误差
    pass_model_time = 1000;            //绕行过度时间戳(ms)
    was_pass_active = false;           //上一次的绕行状态
    continue_pass = false;             //是否继续绕行
    err_smooth = false;                //是否启动误差平滑（速度决策用）

    //解算周期计时复位
    out_count = 0;         //外环解算时间
    in_count = 0;          //内环解算时间

    //元素识别复位
    element_state.element_init();

    //模型绕行标志位复位
    element_state.model_line_pass_active = false;

    //负压状态复位
    fans_state_reset();
}

// LADRC / PID 算法强行停车，起保护作用
void car_stop(void)
{
    //电机停止
    if(ladrc_on)
    {
        LADRC_Lmotor(0.0f);     //左电机
        LADRC_Rmotor(0.0f);     //右电机
    }
    else
    {
        PID_Lmotor(0.0f);
        PID_Rmotor(0.0f);
    }  
}

//智能车软停止
void car_soft_stop(void)
{
    //电机停止
    motor.set_motor1(0,0);   //左电机
    motor.set_motor2(0,0);   //右电机
}

//超速保护
void overspeed_protect(void)
{
    if((-motor.encoder1_counts * 1.0f) > 800 || (-motor.encoder2_counts * 1.0f) > 800 || 
        (-motor.encoder1_counts * 1.0f) < -800 || (-motor.encoder2_counts * 1.0f) < -800)
    {
        std::cout << "已超速" <<std::endl;
        Gogo = false;
    }
}

//智能车起步控制
void car_start(void)
{
    //定义左右电机平均速度
    float mid_speed = ((-motor.encoder1_counts * 1.0f) + (-motor.encoder2_counts * 1.0f)) / 2;

    //使能电机
    if(!motor_enable_enable && !timer_started)
    {
        motor_enable_count(2000);       //设置电机启动前的2s倒计时
        timer_started = true;
    }
    motor_enable_enable_control();      //使能电机使能并启动使能电机

    //智能车通过设置限幅值平滑起步，防电机起步猛转
    if(is_start && motor_enable)
    {  
        if(ladrc_on) LADRC_CarStart(car_speed,mid_speed,200,&left_motor,&right_motor);
        else PID_CarStart(car_speed,mid_speed,200,&Lmotor_PID,&Rmotor_PID);
        //以目标速度的一半为平滑启动的截止处
        if(mid_speed >= car_speed / 2.0f)
        {
            is_start = 0;           //起步完成，置标志位
            timer_started = false;  //复位开始计时状态
            pid_ErrOut_clear(); //清除PID误差、积分、输出
            ladrc_Err_clear();  //ladrc误差清除
            std::cout << "起步完成" <<std::endl;
        }

        //超速保护
        overspeed_protect();
    }

}

//PID各个环的误差和输出清零
void pid_ErrOut_clear(void)
{
    //清除误差和输出
    PID_Reset(&Lmotor_PID);
    PID_Reset(&Rmotor_PID);
    PID_Reset(&Angle_PID);
    PD_FF_Reset(&Angle_PID_F);
    PID_Reset(&Photo_PID);
}

//LADRC误差清除
void ladrc_Err_clear(void)
{
    LADRC_Reset(&right_motor);
    LADRC_Reset(&left_motor);
}

//负压占空比设置
void fans_duty_set(int duty)
{
    if(duty < 0 || duty > 80) return;
    fans.set_duty(duty);
}

//负压缓启
float fans_step = 0.0f; //负压占空比增量步长
void fans_soft_start(int target, int time)   //设置目标占空比和启动时间
{
    //参数保护
    if(time <= 0 || target < 0 || target > 80) return;

    //负压未启动完成时
    if(!fans_ok)
    {
        if(fans_duty >= target || fans_time >= time * 1000)
        {
            fans_duty = target;
            fans_duty_set(fans_duty);
            fans_ok = true; //负压启动完成
            std::cout << "负压占空比达到目标值" << std::endl;
        }
        else
        {
            fans_step += (float)target / (time * 1000);  //负压占空比每次增加固定步长
            if(fans_step >= 1.0f)
            {
                int inc = (int)fans_step;   //单次增量
                fans_duty += inc;           //累加增量
                fans_step -= inc;           //保留余数(小数部分)

                if(fans_duty >= target) fans_duty = target; //防超调

                fans_duty_set(fans_duty);   //负压软启
            }
            
            fans_time ++;   //负压计时加
        }
    }
    
}

//负压状态复位
void fans_state_reset(void)
{
    fans_ok = false;    //负压启动完成标志置false
    fans_time = 0;      //负压启动计时初始化为0
    fans_ok_time = 0;   //负压启动成功后稳定时间初始化为0
    fans_duty = 0;      //负压占空比设为0
    fans_step = 0.0f;   //负压占空比增量步长初始化为0
    fans_duty_set(fans_duty);   //负压不转
}

//负压占空比保护
void fans_protect(void)
{
    int duty_value = fans.get_duty();
    if(duty_value > 80)
    {
        std::cout << "负压占空比超限" << std::endl;
        fans_state_reset();
        Gogo = false;   //直接停止，不起步
    }
}


// //打滑检测和解决
// void slip_check(void)
// {

// }


