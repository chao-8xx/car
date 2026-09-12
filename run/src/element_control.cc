#include "element_control.h"

/*********声明类*********/
extern Motor motor;

/*********定义锁*********/
std::mutex pass_state_mutex;              //绕行状态互斥锁
std::mutex turn_state_mutex;              //转向状态互斥锁
extern std::mutex photo_err_get_mutex;    //图像误差获取互斥锁
extern std::mutex distance_get_mutex;     //距离获取互斥锁

/*********定义枚举*********/
Pass_state pass_state = none;                   //初始化绕障状态为无
Turn_state turn_state = no_turn;                //初始化转向状态为无
Pass_direction_state pass_direction = no_pass;  //初始化绕障方向为不绕障

/*********定义状态变量*********/
bool align_mid = false;     //智能车是否对准中线
bool pass_ready = false;    //是否准备绕行
bool pass_on = false;       //是否开启绕行状态
bool err_smooth = false;    //是否启动误差平滑

/*********定义变量*********/
extern int temp_out;    //定义临时环外环解算周期
bool was_pass_active = false;        //上一次的绕行状态
bool continue_pass = false;          //是否继续绕行
uint32_t pass_model_time = 1000;     //绕行过度时间(ms)

//element_state.curve_score;    //常规道路弯道强度
//弯道强度元素参数决策
void Element_Ctrl::curve_desion(void)
{
    ////参数整定////
    //定义外环参数（弯道强度变化）//
    float conv_Kp = 0.0f;   //Kp
    float conv_Kd = 0.0f;   //Kd

    //根据弯道强度确定速度//
    int speed_diff = std::abs(preprocess.normal_speed - preprocess.curve_speed);    //定义直弯道速度差
    if(preprocess.normal_speed > 160)
    {
        speed_diff = std::abs(160 - preprocess.curve_speed);    //虚假直道速度，防止直弯差距过大导致目标速度变化大
    }
    int speed_range = speed_diff / 3.0f;   //定义速度段区间
    if(element_state.curve_score > 0.35f && element_state.curve_score <= 1.0f)  //强弯
    {
        //定义速度
        if(preprocess.normal_speed >= preprocess.curve_speed)   //判断直道速度是否大于弯道
        {
            car_speed = preprocess.curve_speed + speed_range * 0.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
        else
        {
            car_speed = preprocess.curve_speed - speed_range * 0.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
    }
    else if(element_state.curve_score > 0.25f && element_state.curve_score <= 0.35f)  //中弯
    {
        //定义速度
        if(preprocess.normal_speed >= preprocess.curve_speed)   //判断直道速度是否大于弯道
        {
            car_speed = preprocess.curve_speed + speed_range * 1.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
        else
        {
            car_speed = preprocess.curve_speed - speed_range * 1.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
    }
    else if(element_state.curve_score > 0.16f && element_state.curve_score <= 0.25f)  //小弯
    {
        //定义速度
        if(preprocess.normal_speed >= preprocess.curve_speed)   //判断直道速度是否大于弯道
        {
            car_speed = preprocess.curve_speed + speed_range * 2.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
        else
        {
            car_speed = preprocess.curve_speed - speed_range * 2.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
    }
    else if(element_state.curve_score > 0.0f && element_state.curve_score <= 0.15f)  //小小弯
    {
        //定义速度
        if(preprocess.normal_speed >= preprocess.curve_speed)   //判断直道速度是否大于弯道
        {
            car_speed = preprocess.curve_speed + speed_range * 2.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
        else
        {
            car_speed = preprocess.curve_speed - speed_range * 2.0f;
            if(car_speed % 10 >= 5) car_speed = car_speed - (car_speed % 10) + 10;  //对个位做四舍五入处理
            else car_speed = car_speed - (car_speed % 10);
        }
    }
    else    //否则默认为弯道速度
    {
        car_speed = preprocess.curve_speed;
    }

    //弯道目标速度限幅
    if(car_speed > 150) car_speed = 150;

    //根据目标速度确定参数//
    if(car_speed > 95 && car_speed < 105)  //目标速度为100
    {
        conv_Kp = preprocess.curve_100_Kp;
        conv_Kd = preprocess.curve_100_Kd;
    }
    else if(car_speed > 105 && car_speed < 115)  //目标速度为110
    {
        conv_Kp = preprocess.curve_110_Kp;
        conv_Kd = preprocess.curve_110_Kd;
    }
    else if(car_speed > 115 && car_speed < 125)  //目标速度为120
    {
        conv_Kp = preprocess.curve_120_Kp;
        conv_Kd = preprocess.curve_120_Kd;
    }
    else if(car_speed > 125 && car_speed < 135)  //目标速度为130
    {
        conv_Kp = preprocess.curve_130_Kp;
        conv_Kd = preprocess.curve_130_Kd;
    }
    else if(car_speed > 135 && car_speed < 145)  //目标速度为140
    {
        conv_Kp = preprocess.curve_140_Kp;
        conv_Kd = preprocess.curve_140_Kd;
    }
    else if(car_speed > 145 && car_speed < 155)  //目标速度为150
    {
        conv_Kp = preprocess.curve_150_Kp;
        conv_Kd = preprocess.curve_150_Kd;
    }
    else if(car_speed > 155 && car_speed < 165)  //目标速度为160
    {
        conv_Kp = preprocess.curve_160_Kp;
        conv_Kd = preprocess.curve_160_Kd;
    }
    else    //默认弯道参数
    {
        conv_Kp = preprocess.curve_Ph_Kp;
        conv_Kd = preprocess.curve_Ph_Kd;
    }

    Positional_PID_Init(&Photo_PID,conv_Kp,preprocess.curve_Ph_Ki/10,conv_Kd,500,-1000,1000);  //刷新图像环参数
}

//直道控制
void Element_Ctrl::straight_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    }    
    //PID闭环控制
    Positional_PID_Init(&Angle_PID,preprocess.straight_Ag_Kp,preprocess.straight_Ag_Ki/10,preprocess.straight_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,straight_new_kp,preprocess.straight_Ph_Ki/10,straight_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;

        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(err_smooth)  //是否平滑误差
        {
            final_photo_err = photo_pass.update(current_photo_err);
            Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        }
        else
        {
            Positional_PID_Cal(&Photo_PID,0,current_photo_err); //外环图像环解算
        }
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        straight_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.straight_Ph_Kp;    //用模糊PID更新图像环Kp
        straight_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.straight_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }

}

//安全过渡状态控制(直弯道过渡)
void Element_Ctrl::safe_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    } 
    //PID中外环参数
    Positional_PID_Init(&Angle_PID,preprocess.safe_Ag_Kp,preprocess.safe_Ag_Ki/10,preprocess.safe_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,safe_new_kp,preprocess.safe_Ph_Ki/10,safe_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;
        
        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(err_smooth)  //是否平滑误差
        {
            final_photo_err = photo_pass.update(current_photo_err);
            Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        }
        else
        {
            Positional_PID_Cal(&Photo_PID,0,current_photo_err); //外环图像环解算
        }
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        safe_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.safe_Ph_Kp;    //用模糊PID更新图像环Kp
        safe_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.safe_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }
}

//弯道控制
void Element_Ctrl::curve_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    } 
    //PID中外环参数
    Positional_PID_Init(&Angle_PID,preprocess.curve_Ag_Kp,preprocess.curve_Ag_Ki/10,preprocess.curve_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,curve_new_kp,preprocess.curve_Ph_Ki/10,curve_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;

        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(err_smooth)  //是否平滑误差
        {
            final_photo_err = photo_pass.update(current_photo_err);
            Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        }
        else
        {
            Positional_PID_Cal(&Photo_PID,0,current_photo_err); //外环图像环解算
        }
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        curve_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.curve_Ph_Kp;    //用模糊PID更新图像环Kp
        curve_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.curve_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }
}

//十字控制
void Element_Ctrl::crossing_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    } 
    //PID闭环控制
    Positional_PID_Init(&Angle_PID,preprocess.crossing_Ag_Kp,preprocess.crossing_Ag_Ki/10,preprocess.crossing_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,crossing_new_kp,preprocess.crossing_Ph_Ki/10,crossing_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //LADRC电机启动，跟随控制
    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;

        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(err_smooth)  //是否平滑误差
        {
            final_photo_err = photo_pass.update(current_photo_err);
            Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        }
        else
        {
            Positional_PID_Cal(&Photo_PID,0,current_photo_err); //外环图像环解算
        }
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        crossing_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.crossing_Ph_Kp;    //用模糊PID更新图像环Kp
        crossing_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.crossing_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }
}

//圆环控制
void Element_Ctrl::ring_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    } 
    //PID闭环控制
    Positional_PID_Init(&Angle_PID,preprocess.ring_Ag_Kp,preprocess.ring_Ag_Ki/10,preprocess.ring_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,ring_new_kp,preprocess.ring_Ph_Ki/10,ring_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //LADRC电机启动，跟随控制
    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;
    
        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(err_smooth)  //是否平滑误差
        {
            final_photo_err = photo_pass.update(current_photo_err);
            Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        }
        else
        {
            Positional_PID_Cal(&Photo_PID,0,current_photo_err); //外环图像环解算
        }
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        ring_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.ring_Ph_Kp;    //用模糊PID更新图像环Kp
        ring_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.ring_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }
}

//模型控制(识别图片绕行) model_detector.target_state
void Element_Ctrl::model_control(void)
{
    ////参数整定////
    if(ladrc_on)
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    }

    //模型子状态参数整定
    switch(model_detector.target_state)
    {
        case ModelTargetState::warning:     //检测到宽松红框，预备减速
        {
            car_speed = preprocess.model_warning_speed;
            break;
        }
        case ModelTargetState::recognizing: //识别状态，需减速
        {
            car_speed = preprocess.model_recognizing_speed;
            break;
        }
        case ModelTargetState::passing:     //绕行状态，正在绕行
        {
            car_speed = preprocess.model_pass_speed;
            break;
        }
        case ModelTargetState::returning:   //回正状态，可适当提速
        {
            car_speed = preprocess.model_return_speed;
            break;
        }
        default:
        {
            car_speed = preprocess.model_speed;
            break;
        }
    }
    model_state_kp = preprocess.model_Ph_Kp - ((preprocess.model_speed - car_speed) / 10) * preprocess.model_kp_step;

    //PID闭环控制
    Positional_PID_Init(&Angle_PID,preprocess.model_Ag_Kp,preprocess.model_Ag_Ki/10,preprocess.model_Ag_Kd,500,-1000,1000);
    Positional_PID_Init(&Photo_PID,model_new_kp,preprocess.model_Ph_Ki/10,model_new_kd,500,-1000,1000);  //图像环

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //LADRC电机启动，跟随控制
    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //内环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;

        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        final_photo_err = photo_pass.update(current_photo_err); //图像误差平滑滤波

        //缓慢帧跳过
        frame_pass_decision();

        Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        model_new_kp = fuzzy.PID_deltaKp(&Photo_PID) + model_state_kp;            //用模糊PID更新图像环Kp
        model_new_kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.model_Ph_Kd;    //用模糊PID更新图像环Kd
        
    }
}

//常规控制
void Element_Ctrl::normal_control(void)
{
    ////参数整定////
    //电机参数刷新
    if(ladrc_on)    
    {
        //LADRC电机控制
        LADRC_SetParams(&right_motor,0,preprocess.R_wc,preprocess.R_w0,preprocess.R_b,max_pwm);
        LADRC_SetParams(&left_motor,0,preprocess.L_wc,preprocess.L_w0,preprocess.L_b,max_pwm);
    }
    else
    {
        //PID电机控制
        Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-9000,9000);
        Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-9000,9000);
    } 

    //PID参数刷新
    if(element_state.current_track_type == TrackType::model)    //绕行状态时用模型参数
    {
        //模型子状态参数整定
        switch(model_detector.target_state)
        {
            case ModelTargetState::warning:     //检测到宽松红框，预备减速
            {
                car_speed = preprocess.model_warning_speed;
                break;
            }
            case ModelTargetState::recognizing: //识别状态，需减速
            {
                car_speed = preprocess.model_recognizing_speed;
                break;
            }
            case ModelTargetState::passing:     //绕行状态，正在绕行
            {
                car_speed = preprocess.model_pass_speed;
                break;
            }
            case ModelTargetState::returning:   //回正状态，可适当提速
            {
                car_speed = preprocess.model_return_speed;
                break;
            }
            default:
            {
                car_speed = preprocess.model_speed;
                break;
            }
        }
        model_state_kp = preprocess.model_Ph_Kp - ((preprocess.model_speed - car_speed) / 10) * preprocess.model_kp_step;

        Positional_PID_Init(&Angle_PID,preprocess.model_Ag_Kp,preprocess.model_Ag_Ki/10,preprocess.model_Ag_Kd,500,-1000,1000);  //角速度环
        Positional_PID_Init(&Photo_PID,model_state_kp,preprocess.model_Ph_Ki/10,preprocess.model_Ph_Kd,500,-1000,1000);  //图像环
    }
    else if(element_state.current_track_type == TrackType::circle)
    {
        Positional_PID_Init(&Angle_PID,preprocess.ring_Ag_Kp,preprocess.ring_Ag_Ki/10,preprocess.ring_Ag_Kd,500,-1000,1000);  //角速度环
        Positional_PID_Init(&Photo_PID,preprocess.ring_Ph_Kp,preprocess.ring_Ph_Ki/10,preprocess.ring_Ph_Kd,500,-1000,1000);  //图像环
        preprocess.preview = preprocess.ring_preview / 100.0f;
        car_speed = preprocess.ring_speed;
    }
    else
    {
        Positional_PID_Init(&Angle_PID,preprocess.Ag_Kp,preprocess.Ag_Ki/10,preprocess.Ag_Kd,500,-1000,1000);  //角速度环
        Positional_PID_Init(&Photo_PID,preprocess.Ph_Kp,preprocess.Ph_Ki/10,preprocess.Ph_Kd,500,-1000,1000);  //图像环
        preprocess.preview = preprocess.straight_preview / 100.0f;
        car_speed = preprocess.normal_speed;
    }
    
    // ////速度决策////
    // speed_decision();

    ////蜂鸣器控制决策////
    // buzzer_decision();

    //一加一减
    L_target = car_speed - angle_out;
    R_target = car_speed + angle_out;

    //电机启动
    if(ladrc_on)
    {
        //LADRC电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            LADRC_Lmotor(L_target);
            LADRC_Rmotor(R_target);            
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }
    else
    {
        //PID电机启动，跟随控制
        if(motor_enable)    //判断电机是否被使能
        {
            PID_Lmotor(L_target);
            PID_Rmotor(R_target);
        }
        else    //未使能则保持停止
        {
            car_soft_stop();
        }
    }

    ////----三串PID解算----////
    //内环PID
    in_count ++;
    if(in_count >= preprocess.inner_count)
    {
        in_count = 0;

        float current_yaw_gyro = yaw_gyro;  //目前读出的偏航角速度

        //一阶低通滤波(偏航角速度)
        final_yaw_gyro = yaw_lowpass.update(current_yaw_gyro);

        Positional_PID_Cal(&Angle_PID,photo_out,final_yaw_gyro);  //中环角速度环解算
        angle_out = Angle_PID.output; //定义内环角速度环输出
    }

    //外环PID
    out_count ++;
    if(out_count >= preprocess.outer_count)
    {
        out_count = 0;

        ////获取图像误差////
        //图像误差赋值
        photo_err_get_mutex.lock();
        float current_photo_err = photo_err;    //加锁读取图像误差，保护
        photo_err_get_mutex.unlock();

        if(element_state.model_line_pass_active || continue_pass)
        {
            final_photo_err = photo_pass.update(current_photo_err);
        }
        else
        {
            final_photo_err = current_photo_err;
        }

        //缓慢帧跳过
        frame_pass_decision();

        Positional_PID_Cal(&Photo_PID,0,final_photo_err); //外环图像环解算
        photo_out = Photo_PID.output;   //定义外环图像环输出值

        // new_Kp = fuzzy.PID_deltaKp(&Photo_PID) + preprocess.Ph_Kp;    //用模糊PID更新图像环Kp
        // new_Kd = fuzzy.PID_deltaKd(&Photo_PID) + preprocess.Ph_Kd;    //用模糊PID更新图像环Kd
    }
}


//左绕控制(车头往左偏，偏航角增加)
void Element_Ctrl::L_pass_control(float angle, float distance)     //设置绕障碍的转角和距离
{
    //参数保护
    if(distance < 0.0f || angle > 80.0f || angle < 0.0f) return;

    ////状态复位，准备绕行////
    if(align_mid && !yaw_reset)  //判断是否对准中线(对准中线时偏航角复位为0)
    {
        ////初始化状态////
        pass_state_mutex.lock();    //上锁
        pass_on = true;             //对准中线后，开启绕行状态
        pass_state = track_out;     //绕行状态设为出赛道
        align_mid = false;          //对准中线状态置false
        pass_ready = false;         //绕行准备状态置false
        pass_state_mutex.unlock();  //解锁

        ////清零积分和误差////
        PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
        if(ladrc_on)
        {
            LADRC_Reset(&left_motor);   //重置电机状态
            LADRC_Reset(&right_motor);
        }
        else
        {
            PID_Reset(&Lmotor_PID);     //重置电机状态
            PID_Reset(&Rmotor_PID);
        }

        ////参数赋值////
        Positional_PID_Init(&Temp_PID,preprocess.Te_Kp,preprocess.Te_Ki,preprocess.Te_Kd,50,-500,500);  //角度环（临时）
        temp_out = 0;   //计时清0

        ////刷新陀螺仪////
        madg.yaw_angle_reset();   //对准中线时给偏航角置0，开始用陀螺仪控制车        
        
        ////刷新编码器////
        motor_distance_clear();     //清零智能车行使距离

        std::cout << "开始左绕障" <<std::endl;
    }

    if(pass_on) //处于绕障状态时
    {
        Pass_state current_state;   //定义智能车目前状态
        pass_state_mutex.lock();
        current_state = pass_state; //状态赋值
        pass_state_mutex.unlock();

        switch (current_state) //判断绕行过程中的状态机
        {
            case track_out: //出赛道状态
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear();     //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = parallel;      //赛道外切换为与赛道平行状态行驶
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    pass_state_mutex.unlock();

                    break;
                }
                
                //双环PID控制
                double_circle_control(angle);
                motor_distance_get();   //开始获取智能车行驶距离
                
                break;
            }
            case parallel:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = track_in;      //赛道外切换为进入赛道状态
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    pass_state_mutex.unlock();

                    break;
                }                

                //双环PID控制
                double_circle_control(0.0f);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case track_in:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance * 0.85f)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = none;          //进入赛道后，绕障子状态变为空闲状态
                    pass_on = false;            //智能车状态改为非绕障状态
                    pass_ready = false;         //准备绕障状态设置为false
                    yaw_reset = true;           //启动偏航角中线复位
                    pass_direction = no_pass;   //绕障方向状态设置为不绕障
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    std::cout<<"左绕行结束"<<std::endl;
                    pass_state_mutex.unlock();

                    break;
                }

                //双环PID控制
                double_circle_control(-angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            default:
            {
                break;
            }  
        }  
    }
}

//右绕控制(车头往右偏，偏航角减少)
void Element_Ctrl::R_pass_control(float angle, float distance)     //设置绕障碍的转角和距离
{
    //参数保护
    if(distance < 0.0f || angle > 85.0f || angle < 0.0f) return;

    ////状态复位，准备绕行////
    if(align_mid && !yaw_reset)   //判断是否对准中线(对准中线时偏航角复位为0)
    {
        ////初始化状态////
        pass_state_mutex.lock();    //上锁
        pass_on = true;             //对准中线后，开启绕行状态
        pass_state = track_out;     //绕行状态设为出赛道
        align_mid = false;          //对准中线状态置false
        pass_state_mutex.unlock();  //解锁
        
        ////清零积分和误差////
        PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
        if(ladrc_on)
        {
            LADRC_Reset(&left_motor);   //重置电机状态
            LADRC_Reset(&right_motor);
        }
        else
        {
            PID_Reset(&Lmotor_PID);     //重置电机状态
            PID_Reset(&Rmotor_PID);
        }

        ////参数赋值////
        Positional_PID_Init(&Temp_PID,preprocess.Te_Kp,preprocess.Te_Ki,preprocess.Te_Kd,50,-500,500);  //角度环（临时）
        temp_out = 0;   //计时清0

        ////刷新陀螺仪////
        madg.yaw_angle_reset();   //对准中线时给偏航角置0，开始用陀螺仪控制车

        ////刷新编码器////
        motor_distance_clear();     //清零智能车行使距离

        std::cout << "开始右绕障" <<std::endl;
    }

    if(pass_on) //处于绕障状态时
    {
        Pass_state current_state;   //定义智能车目前状态
        pass_state_mutex.lock();
        current_state = pass_state; //状态赋值
        pass_state_mutex.unlock();

        switch (current_state) //判断绕行过程中的状态机
        {
            case track_out: //出赛道状态
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear();     //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = parallel;      //赛道外切换为与赛道平行状态行驶
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    pass_state_mutex.unlock();

                    break;
                }

                //双环PID控制
                double_circle_control(-angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case parallel:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = track_in;      //赛道外切换为进入赛道状态
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    pass_state_mutex.unlock();

                    break;
                }

                //双环PID控制
                double_circle_control(0.0f);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case track_in:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(current_distance >= distance * 0.85f)   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = none;          //进入赛道后，绕障子状态变为空闲状态
                    pass_on = false;            //智能车状态改为非绕障状态
                    pass_ready = false;         //准备绕障状态设置为false
                    yaw_reset = true;           //启动偏航角中线复位
                    pass_direction = no_pass;   //绕障方向状态设置为不绕障
                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0
                    std::cout<<"右绕行结束"<<std::endl;
                    pass_state_mutex.unlock();

                    break;
                }

                //双环PID控制
                double_circle_control(angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            default:
            {

                break;
            }
                
        }
        
    }
    
}

//绕障控制
void Element_Ctrl::pass_control(Pass_direction_state *pass_dir,float angle, float distance)
{
    //参数保护
    if(*pass_dir == no_pass || pass_dir == nullptr) return;

    if(*pass_dir == left_pass)          //左绕
    {
        if(pass_state == none)
        {
            align_mid = true;   //中线校准状态为true，开始转向
            yaw_reset = false;  //禁用偏航角中线复位
        }
        L_pass_control(angle,distance);
    }
    else if(*pass_dir == right_pass)    //右绕
    {
        if(pass_state == none)
        {
            align_mid = true;   //中线校准状态为true，开始转向
            yaw_reset = false;  //禁用偏航角中线复位
        }
        R_pass_control(angle,distance);
    }

}

float incircle_current_yaw = 0.0f;  //用于记录圆环内识别到图片时当前的偏航角
//动态左绕控制（根据编码器积分动态改变绕行目标角度）
void Element_Ctrl::dynamic_Lpass_control(float max_angle)
{
    //参数保护
    if(max_angle > 85.0f || max_angle < 0.0f) return;

    ////状态复位，准备绕行////
    if(align_mid && !yaw_reset)   //判断是否对准中线(对准中线时偏航角复位为0)
    {
        ////初始化状态////
        pass_state_mutex.lock();    //上锁
        pass_on = true;             //对准中线后，开启绕行状态
        pass_state = track_out;     //绕行状态设为出赛道
        align_mid = false;          //对准中线状态置false
        pass_state_mutex.unlock();  //解锁
        
        ////清零积分和误差////
        PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
        if(ladrc_on)
        {
            LADRC_Reset(&left_motor);   //重置电机状态
            LADRC_Reset(&right_motor);
        }
        else
        {
            PID_Reset(&Lmotor_PID);     //重置电机状态
            PID_Reset(&Rmotor_PID);
        }
        
        ////参数赋值////
        Positional_PID_Init(&Temp_PID,preprocess.Te_Kp,preprocess.Te_Ki,preprocess.Te_Kd,50,-500,500);  //角度环（临时）
        temp_out = 0;   //计时清0

        ////非圆环内刷新陀螺仪////
        if(element_state.current_track_type != TrackType::circle)   //圆环需要用到编码器，在圆环内不得刷新编码器
        {
            madg.yaw_angle_reset();   //对准中线时给偏航角置0，开始用陀螺仪控制车
            incircle_current_yaw = 0.0f;    //环内偏航角置0
        }
        else
        {
            incircle_current_yaw = yaw_angle;   //读取当前偏航角度
        }
        
        ////刷新编码器////
        motor_distance_clear();     //清零智能车行使距离

        std::cout << "开始左绕障" <<std::endl;
    }

    ////绕行控制////
    if(pass_on) //处于绕障状态时
    {
        Pass_state current_state;   //定义智能车目前状态
        pass_state_mutex.lock();
        current_state = pass_state; //状态赋值
        pass_state_mutex.unlock();

        switch (current_state) //判断绕行过程中的状态机
        {
            case track_out: //出赛道状态
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断实际角度是否达到目标角度
                if(yaw_angle >= (max_angle + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear();     //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = parallel;      //赛道外切换为与赛道平行状态行驶
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0

                    break;
                }

                //动态系数变化
                float dyn = std::abs((max_angle + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case parallel:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断实际角度是否达到目标角度
                if(yaw_angle <= (0.0f + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = track_in;      //赛道外切换为进入赛道状态
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0

                    break;
                }

                //动态系数变化
                float dyn = std::abs((0.0f + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = max_angle - current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case track_in:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(yaw_angle <= (-max_angle * 0.8f + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = none;          //进入赛道后，绕障子状态变为空闲状态
                    pass_direction = no_pass;   //绕障方向状态设置为不绕障
                    pass_on = false;            //智能车状态改为非绕障状态
                    pass_ready = false;         //准备绕障状态设置为false
                    yaw_reset = true;           //启动偏航角中线复位
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);           //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;                   //解算计时清0
                    incircle_current_yaw = 0.0f;    //环内偏航角清0
                    std::cout<<"左绕行结束"<<std::endl;

                    break;
                }

                //动态系数变化
                float dyn = std::abs((-max_angle * 0.8f + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = - current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            default:
            {

                break;
            }
                
        }
        
    }

}

//动态左绕控制（根据编码器积分动态改变绕行目标角度）
void Element_Ctrl::dynamic_Rpass_control(float max_angle)
{
    //参数保护
    if(max_angle > 85.0f || max_angle < 0.0f) return;

    ////状态复位，准备绕行////
    if(align_mid && !yaw_reset)   //判断是否对准中线(对准中线时偏航角复位为0)
    {
        ////初始化状态////
        pass_state_mutex.lock();    //上锁
        pass_on = true;             //对准中线后，开启绕行状态
        pass_state = track_out;     //绕行状态设为出赛道
        align_mid = false;          //对准中线状态置false
        pass_state_mutex.unlock();  //解锁
        
        ////清零积分和误差////
        PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
        if(ladrc_on)
        {
            LADRC_Reset(&left_motor);   //重置电机状态
            LADRC_Reset(&right_motor);
        }
        else
        {
            PID_Reset(&Lmotor_PID);     //重置电机状态
            PID_Reset(&Rmotor_PID);
        }
        
        ////参数赋值////
        Positional_PID_Init(&Temp_PID,preprocess.Te_Kp,preprocess.Te_Ki,preprocess.Te_Kd,50,-500,500);  //角度环（临时）
        temp_out = 0;   //计时清0

        ////非圆环内刷新陀螺仪////
        if(element_state.current_track_type != TrackType::circle)   //圆环需要用到编码器，在圆环内不得刷新编码器
        {
            madg.yaw_angle_reset();   //对准中线时给偏航角置0，开始用陀螺仪控制车
            incircle_current_yaw = 0.0f;    //环内偏航角置0
        }
        else
        {
            incircle_current_yaw = yaw_angle;   //读取当前偏航角度
        }

        ////刷新编码器////
        motor_distance_clear();     //清零智能车行使距离

        std::cout << "开始右绕障" <<std::endl;
    }

    ////绕行控制////
    if(pass_on) //处于绕障状态时
    {
        Pass_state current_state;   //定义智能车目前状态
        pass_state_mutex.lock();
        current_state = pass_state; //状态赋值
        pass_state_mutex.unlock();

        switch (current_state) //判断绕行过程中的状态机
        {
            case track_out: //出赛道状态
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断实际角度是否达到目标角度
                if(yaw_angle <= (-max_angle + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear();     //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = parallel;      //赛道外切换为与赛道平行状态行驶
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0

                    break;
                }

                //动态系数变化
                float dyn = std::abs((-max_angle + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = -current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case parallel:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断实际角度是否达到目标角度
                if(yaw_angle >= (0.0f + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = track_in;      //赛道外切换为进入赛道状态
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);       //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;               //解算计时清0

                    break;
                }

                //动态系数变化
                float dyn = std::abs((0.0f + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = -max_angle + current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            case track_in:
            {
                //读取行驶距离
                distance_get_mutex.lock();
                float current_distance = motor_distance;
                distance_get_mutex.unlock();
                //判断是否达到目标距离
                if(yaw_angle >= (max_angle * 0.8f + incircle_current_yaw))   //达到目标行驶 距离时清空距离、修改状态并退出
                {
                    motor_distance_clear(); //重置智能车行使距离

                    pass_state_mutex.lock();
                    pass_state = none;          //进入赛道后，绕障子状态变为空闲状态
                    pass_direction = no_pass;   //绕障方向状态设置为不绕障
                    pass_on = false;            //智能车状态改为非绕障状态
                    pass_ready = false;         //准备绕障状态设置为false
                    yaw_reset = true;           //启动偏航角中线复位
                    pass_state_mutex.unlock();

                    PID_Reset(&Temp_PID);           //清除临时环（陀螺仪）的误差和积分
                    temp_out = 0;                   //解算计时清0
                    incircle_current_yaw = 0.0f;    //环内偏航角清0
                    std::cout<<"右绕行结束"<<std::endl;

                    break;
                }

                //动态系数变化
                float dyn = std::abs((max_angle * 0.8f + incircle_current_yaw) - yaw_angle) / max_angle;   //动态变化因子
                float coe = preprocess.yaw_distance * dyn;   //系数
                if(coe < 0.5f) coe = 0.5f;  //系数限幅

                //双环PID控制
                float target_yaw_angle = current_distance * coe + incircle_current_yaw;
                double_circle_control(target_yaw_angle);
                motor_distance_get();   //开始获取智能车行驶距离

                break;
            }
            default:
            {

                break;
            }
                
        }
        
    }
}

//动态目标角度绕行控制
void Element_Ctrl::dynamic_pass_control(Pass_direction_state *pass_dir,float angle)
{
    //参数保护
    if(*pass_dir == no_pass || pass_dir == nullptr || angle < 0.0f || angle > 85.0f) return;

    if(*pass_dir == left_pass)          //左绕
    {
        if(pass_state == none)
        {
            align_mid = true;   //中线校准状态为true，开始转向
            yaw_reset = false;  //禁用偏航角中线复位
        }
        dynamic_Lpass_control(angle);
    }
    else if(*pass_dir == right_pass)    //右绕
    {
        if(pass_state == none)
        {
            align_mid = true;   //中线校准状态为true，开始转向
            yaw_reset = false;  //禁用偏航角中线复位
        }
        dynamic_Rpass_control(angle);
    }
}

static int ready_turn = 1;
//转向控制
void Element_Ctrl::turn_control(Turn_state *turn, float angle, float distance)
{
    //参数保护
    if(turn == nullptr || *turn == no_turn) return;

    //执行转向控制时，先初始化
    if(ready_turn)
    {
        //积分和误差清零
        motor_distance_clear();
        PID_Reset(&Temp_PID);
        //状态切换
        temp_out = 0;       //计时清0
        ready_turn = 0;     //翻转标志位
        yaw_reset = false;  //禁用偏航角中线复位
        //参数赋值
        Positional_PID_Init(&Temp_PID,preprocess.Te_Kp,preprocess.Te_Ki,preprocess.Te_Kd,100,-500,500);  //角度环（临时）
    }

    //转向状态机
    switch (*turn)
    {
        case turn_left:
        {
            //读取行驶距离
            distance_get_mutex.lock();
            float current_distance = motor_distance;
            distance_get_mutex.unlock();
            if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
            {
                motor_distance_clear();    //重置智能车行使距离

                turn_state_mutex.lock();
                *turn = no_turn;      //赛道外切换为与赛道平行状态行驶
                PID_Reset(&Temp_PID);     //清除临时环（陀螺仪）的误差和积分
                ready_turn = 1;           //状态标志位复位
                yaw_reset = true;         //启动偏航角中线复位
                turn_state_mutex.unlock();

                break;
            }

            //读取偏航角
            float current_yaw = yaw_angle;

            //PID控制
            temp_out ++;
            if(temp_out > 5)
            {
                temp_out = 0;
                Positional_PID_Cal(&Temp_PID,angle,current_yaw);  //暂时去除图像环，用电机和偏航角闭环
            }
            float steer = Temp_PID.output;     //定义PID输出值
            if(ladrc_on)
            {
                LADRC_Lmotor(car_speed - steer);       //左电机控速
                LADRC_Rmotor(car_speed + steer);       //右电机控速
            }
            else
            {
                PID_Lmotor(car_speed - steer);     //左电机控速
                PID_Rmotor(car_speed + steer);     //右电机控速
            }
            // if(steer >= 0)
            // {
            //     LADRC_Lmotor(car_speed - steer);     //左电机控速
            //     LADRC_Rmotor(car_speed);             //右电机控速
            // }
            // else if(steer < 0)
            // {
            //     LADRC_Lmotor(car_speed);             //左电机控速
            //     LADRC_Rmotor(car_speed + steer);     //右电机控速
            // }
            //获取距离
            motor_distance_get();   //开始获取智能车行驶距离

            break;
        }

        case turn_right:
        {
            //读取行驶距离
            distance_get_mutex.lock();
            float current_distance = motor_distance;
            distance_get_mutex.unlock();
            if(current_distance >= distance)   //达到目标行驶 距离时清空距离、修改状态并退出
            {
                motor_distance_clear();     //重置智能车行使距离

                turn_state_mutex.lock();
                *turn = no_turn;      //赛道外切换为与赛道平行状态行驶
                PID_Reset(&Temp_PID);     //清除临时环（陀螺仪）的误差和积分
                ready_turn = 1;           //状态标志位复位
                yaw_reset = true;         //启用偏航角中线复位
                turn_state_mutex.unlock();

                break;
            }

            //读取偏航角
            float current_yaw = yaw_angle;

            //PID控制
            temp_out ++;
            if(temp_out > 5)
            {
                temp_out = 0;
                Positional_PID_Cal(&Temp_PID,-angle,current_yaw);  //暂时去除图像环，用电机和偏航角闭环
            }
            float steer = Temp_PID.output;     //定义PID输出值
            if(ladrc_on)
            {
                LADRC_Lmotor(car_speed - steer);       //左电机控速
                LADRC_Rmotor(car_speed + steer);       //右电机控速
            }
            else
            {
                PID_Lmotor(car_speed - steer);     //左电机控速
                PID_Rmotor(car_speed + steer);     //右电机控速
            }
            // if(steer >= 0)
            // {
            //     LADRC_Lmotor(car_speed - steer);     //左电机控速
            //     LADRC_Rmotor(car_speed);             //右电机控速
            // }
            // else if(steer < 0)
            // {
            //     LADRC_Lmotor(car_speed);             //左电机控速
            //     LADRC_Rmotor(car_speed + steer);     //右电机控速
            // }
            //获取距离
            motor_distance_get();   //开始获取智能车行驶距离

            break;
        }

        default:
            break;
    }
}


// //正常参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::normal_parament_lock(void)
// {
//     if(!Gogo && !speed_deci_on)
//     {
//         if(preprocess.normal_speed > 95 && preprocess.normal_speed < 105)   //100速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 105 && preprocess.normal_speed < 115)   //110速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 115 && preprocess.normal_speed < 125)   //120速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 125 && preprocess.normal_speed < 135)   //130速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 135 && preprocess.normal_speed < 145)   //140速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 145 && preprocess.normal_speed < 155)   //150速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 155 && preprocess.normal_speed < 165)   //160速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 165 && preprocess.normal_speed < 175)   //170速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.Ph_Kp = ;    //外环Kp
//             preprocess.Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //直道参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::straight_parament_lock(void)
// {
//     if(!Gogo && speed_deci_on)
//     {
//         if(preprocess.normal_speed > 95 && preprocess.normal_speed < 105)         //100速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 105 && preprocess.normal_speed < 115)   //110速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 115 && preprocess.normal_speed < 125)   //120速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 125 && preprocess.normal_speed < 135)   //130速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 135 && preprocess.normal_speed < 145)   //140速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 145 && preprocess.normal_speed < 155)   //150速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 155 && preprocess.normal_speed < 165)   //160速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 165 && preprocess.normal_speed < 175)   //170速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 175 && preprocess.normal_speed < 185)   //180速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 185 && preprocess.normal_speed < 195)   //190速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.normal_speed > 195 && preprocess.normal_speed < 205)   //200速度
//         {
//             preprocess.straight_preview = ;  //前瞻
//             preprocess.straight_Ph_Kp = ;    //外环Kp
//             preprocess.straight_Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //弯道参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::curve_parament_lock(void)
// {
//     if(!Gogo && speed_deci_on)
//     {
//         if(preprocess.curve_speed > 95 && preprocess.curve_speed < 105)   //100速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_100_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_100_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 105 && preprocess.curve_speed < 115)   //110速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_110_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_110_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 115 && preprocess.curve_speed < 125)   //120速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_120_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_120_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 125 && preprocess.curve_speed < 135)   //130速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_130_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_130_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 135 && preprocess.curve_speed < 145)   //140速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_140_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_140_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 145 && preprocess.curve_speed < 155)   //150速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_150_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_150_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 155 && preprocess.curve_speed < 165)   //160速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = preprocess.curve_160_Kp;    //外环Kp
//             preprocess.curve_Ph_Kd = preprocess.curve_160_Kd;    //外环Kd
//         }
//         else if(preprocess.curve_speed > 165 && preprocess.curve_speed < 175)   //170速度
//         {
//             preprocess.curve_preview = ;  //前瞻
//             preprocess.curve_Ph_Kp = ;    //外环Kp
//             preprocess.curve_Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //十字参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::cross_parament_lock(void)
// {
//     if(!Gogo && speed_deci_on)
//     {
//         if(preprocess.cross_speed > 95 && preprocess.cross_speed < 105)   //100速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 105 && preprocess.cross_speed < 115)   //110速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 115 && preprocess.cross_speed < 125)   //120速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 125 && preprocess.cross_speed < 135)   //130速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 135 && preprocess.cross_speed < 145)   //140速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 145 && preprocess.cross_speed < 155)   //150速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 155 && preprocess.cross_speed < 165)   //160速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.cross_speed > 165 && preprocess.cross_speed < 175)   //170速度
//         {
//             preprocess.cross_preview = ;  //前瞻
//             preprocess.cross_Ph_Kp = ;    //外环Kp
//             preprocess.cross_Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //圆环参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::ring_parament_lock(void)
// {
//     if(!Gogo && speed_deci_on)
//     {
//         if(preprocess.ring_speed > 95 && preprocess.ring_speed < 105)   //100速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 105 && preprocess.ring_speed < 115)   //110速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 115 && preprocess.ring_speed < 125)   //120速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 125 && preprocess.ring_speed < 135)   //130速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 135 && preprocess.ring_speed < 145)   //140速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 145 && preprocess.ring_speed < 155)   //150速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 155 && preprocess.ring_speed < 165)   //160速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.ring_speed > 165 && preprocess.ring_speed < 175)   //170速度
//         {
//             preprocess.ring_preview = ;  //前瞻
//             preprocess.ring_Ph_Kp = ;    //外环Kp
//             preprocess.ring_Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //模型参数锁定(未发车时，便于快速切换参数)
// void Element_Ctrl::model_parament_lock(void)
// {
//     if(!Gogo && speed_deci_on)
//     {
//         if(preprocess.model_speed > 95 && preprocess.model_speed < 105)   //100速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 105 && preprocess.model_speed < 115)   //110速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 115 && preprocess.model_speed < 125)   //120速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 125 && preprocess.model_speed < 135)   //130速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 135 && preprocess.model_speed < 145)   //140速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 145 && preprocess.model_speed < 155)   //150速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 155 && preprocess.model_speed < 165)   //160速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//         else if(preprocess.model_speed > 165 && preprocess.model_speed < 175)   //170速度
//         {
//             preprocess.model_preview = ;  //前瞻
//             preprocess.model_Ph_Kp = ;    //外环Kp
//             preprocess.model_Ph_Kd = ;    //外环Kd
//         }
//     }
// }

// //参数锁定
// void Element_Ctrl::parament_lock(void)
// {
//     if(speed_deci_on)
//     {
//         straight_parament_lock();
//         curve_parament_lock();
//         cross_parament_lock();
//         ring_parament_lock();
//         model_parament_lock();
//     }
//     else
//     {
//         normal_parament_lock();
//     }
// }


Element_Ctrl element_ctrl;




/************************坟场************************/

//----前瞻决策----//
// prospect_decision(); //获取图像误差

//----弯道强度判断----//
// curve_desion();

//----只减不加----//
// if(angle_out >= 0)
// {
//     target_speed_get.lock();
//     L_target = car_speed - angle_out;
//     R_target = car_speed;
//     target_speed_get.unlock();
// }
// else if(angle_out < 0)
// {
//     target_speed_get.lock();
//     L_target = car_speed;
//     R_target = car_speed + angle_out;
//     target_speed_get.unlock();
// }

//----内环角速度环解算----//
// PD_FF_Cal(&Angle_PID_F,photo_out,final_yaw_gyro);  

//----PD+前馈参数初始化----//
// PD_FF_Init(&Angle_PID_F,preprocess.straight_F_Kp,preprocess.straight_F_Kd/1000,preprocess.straight_F_Kff/1000,preprocess.straight_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.safe_F_Kp,preprocess.safe_F_Kd/1000,preprocess.safe_F_Kff/1000,preprocess.safe_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.curve_F_Kp,preprocess.curve_F_Kd/1000,preprocess.curve_F_Kff/1000,preprocess.curve_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.crossing_F_Kp,preprocess.crossing_F_Kd/1000,preprocess.crossing_F_Kff/1000,preprocess.crossing_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.ring_F_Kp,preprocess.ring_F_Kd/1000,preprocess.ring_F_Kff/1000,preprocess.ring_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.model_F_Kp,preprocess.model_F_Kd/1000,preprocess.model_F_Kff/1000,preprocess.ring_F_Kff_acc/1000,500,Ts);  //角速度环
// PD_FF_Init(&Angle_PID_F,preprocess.Ag_F_Kp,preprocess.Ag_F_Kd/1000,preprocess.Ag_Kff/1000,preprocess.Ag_Kff_acc/1000,500,Ts);  //角速度环

//----一阶低通滤波（角速度）----//
// float gryo_flit = 0.2f;
// final_yaw_gyro = gryo_flit * current_yaw_gyro + (1.0f - gryo_flit) * yaw_gyro_filter;    //一阶低通滤波
// yaw_gyro_filter = final_yaw_gyro;   //更新保存


//----绕行切换误差过渡处理----//
// if(was_pass_active && element_state.model_line_pass_active)    //绕行状态时误差平滑
// {
//     final_photo_err = photo_pass.update(current_photo_err);
// }
// else if(!was_pass_active && element_state.model_line_pass_active)  //刚进入绕行
// {
//     pass_model_time = 120 / 5; //刷新绕行时间
// }
// else if(was_pass_active && !element_state.model_line_pass_active)  //刚退出绕行
// {
//     continue_pass = true;   //刚退出绕行时继续保持平滑误差
//     final_photo_err = photo_pass.update(current_photo_err);
// }
// else if(continue_pass && !element_state.model_line_pass_active)   //退出绕行，消耗绕行时间
// {
//     pass_model_time --; //计时减，规定时间内继续平滑误差
//     final_photo_err = photo_pass.update(current_photo_err); //退出状态时仍给一段时间用于平滑，便于更好地过渡到中线
//     if(pass_model_time <= 0)
//     {
//         continue_pass = false;
//     }
// }
// else if(!continue_pass && !element_state.model_line_pass_active)
// {
//     final_photo_err = current_photo_err;
// }

// was_pass_active = element_state.model_line_pass_active; //更新绕行状态
