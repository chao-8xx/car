#include "control_decision.h"

/*********宏定义*********/
//宏定义速度阈值决定速度状态机
#define HIGH_SPEED       300
#define MID_HIGH_SPEED   260
#define MID_LOW_SPEED    220
#define LOW_SPEED        180

//宏定义转向和绕行的目标角度、距离
#define TURN_TARGET_ANGLE       60      //转向目标角度
#define TURN_TARGET_DISTANCE    30      //转向目标距离
#define PASS_TARGET_ANGLE       40      //绕行目标角度
#define PASS_TARGET_DISTANCE    40      //绕行目标距离

/*********定义类*********/
//控制
extern Motor motor;
//元素
extern CircleState circle_state;        //圆环
extern CrossroadState crossroad_state;  //十字路口
extern BarrierState barrier_state;      //路障
extern RampState ramp_state;            //坡道
extern ZebraState zebra_state;          //斑马线

/*********定义锁*********/
std::mutex speed_state_mutex;           //速度状态互斥锁
std::mutex car_speed_get_mutex;         //默认车速获取互斥锁
std::mutex photo_err_get_mutex;         //图像误差获取互斥锁

/*********定义枚举*********/
Speed_state speed_state = Stop; //初始化速度状态为停车

/*********声明枚举*********/
extern bool err_smooth;    //是否启动误差平滑
extern uint32_t pass_model_time;   //绕行过度时间(ms)

//负压控制决策
void fans_decision(void)
{
    if(!fans_ok)
    {
        fans_soft_start(75,2);  //75%目标占空比，2s达到目标
    }
    else
    {
        fans_ok_time ++;
    }
}

//前瞻决策（根据速度状态机决定前瞻）
void prospect_decision(void)
{
    //速度状态决策
    if(car_speed > 0 && car_speed <= LOW_SPEED)
    {
        speed_state_mutex.lock();
        speed_state = Low;      //低速状态
        speed_state_mutex.unlock();
    }
    else if(car_speed > LOW_SPEED && car_speed <= MID_LOW_SPEED)
    {
        speed_state_mutex.lock();
        speed_state = Mid_Low;  //中低速状态
        speed_state_mutex.unlock();
    }
    else if(car_speed > MID_LOW_SPEED && car_speed <= MID_HIGH_SPEED)
    {
        speed_state_mutex.lock();
        speed_state = Mid;      //中速状态
        speed_state_mutex.unlock();
    }
    else if(car_speed > MID_HIGH_SPEED && car_speed <= HIGH_SPEED)
    {
        speed_state_mutex.lock();
        speed_state = Mid_High; //中高速状态
        speed_state_mutex.unlock();
    }
    else if(car_speed > HIGH_SPEED)
    {
        speed_state_mutex.lock();
        speed_state = High;     //高速状态
        speed_state_mutex.unlock();
    }
    else
    {
        speed_state_mutex.lock();
        speed_state = Stop;     //停止状态
        speed_state_mutex.unlock();
    }

    //前瞻决策
    switch(speed_state)
    {
        case Low:       //低速
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead(LOW_SPEED/2,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
        case Mid_Low:   //中低速度
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead((LOW_SPEED+MID_LOW_SPEED)/2,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
        case Mid:       //中速
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead((MID_LOW_SPEED+MID_HIGH_SPEED)/2,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
        case Mid_High:  //中高速
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead((MID_HIGH_SPEED+HIGH_SPEED)/2,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
        case High:      //高速
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead(HIGH_SPEED * 4/3,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
        default:        //停止
        {
            photo_err_get_mutex.lock();
            photo_err = track_base.get_error_lookahead(LOW_SPEED/2,preprocess.preview / 100,preprocess.window_size);
            photo_err_get_mutex.unlock();
            break;
        }
    }
}

/**
 * @brief 带小数累加的速率限制器
 * @param target       目标速度
 * @param current      当前平滑速度（引用，会被修改）
 * @param max_step     每周期允许的最大变化量（可为小数）
 * @param accumulator  小数累加器（静态或外部传入，保持连续性）
 */
//速率限制器
void rate_limiter(float target, float &current, float max_step, float &accumulator)
{
    float diff = target - current;  //目标速度与当前平滑速度之差
    if(diff == 0.0f)    //速度差为0时，小数累加器为0
    {
        accumulator = 0.0f;
        return ;
    }

    float step = (diff > 0)? max_step : -max_step;  //误差存在时，累加步长
    accumulator += step;

    if(fabs(accumulator) >= 1.0f)
    {
        int delta = (int)accumulator;   //提取整数部分
        current += delta;
        accumulator -= delta;           //保留余数
    }

    //防止过冲
    if((diff > 0 && current > target) || (diff < 0 && current < target))
    {
        current = target;
        accumulator = 0.0f;
    }

}

//简单前瞻决策(纯决策前瞻)
void simple_prospect_decision(void)
{
    switch (element_state.current_track_type)     //道路类型
    {
        case TrackType::normal:        //正常道路
        {   
            switch(element_state.normal_road_state)
            {
                case NormalRoadState::straight_fast:  //长直道高速
                {
                    preprocess.preview = preprocess.straight_preview / 100;
                    break;
                }
                case NormalRoadState::safe:  //安全过渡
                {
                    preprocess.preview = preprocess.safe_preview / 100;
                    break;
                }
                case NormalRoadState::curve:  //弯道
                {
                    preprocess.preview = preprocess.curve_preview / 100;
                    break;
                }
                default:
                {
                    preprocess.preview = preprocess.curve_preview / 100;
                    break;
                }

            }
            break;
        }
        case TrackType::crossroad:     //十字
        {
            preprocess.preview = preprocess.crossing_preview / 100;
            break;
        }
        case TrackType::circle:        //环岛
        {
            preprocess.preview = preprocess.ring_preview / 100;
            break;
        }  
        case TrackType::zebra:        //斑马线
        {
            preprocess.preview = preprocess.straight_preview / 100;
            break;
        } 
        case TrackType::ramp:         //坡道
        {
            preprocess.preview = preprocess.curve_preview / 100;
            break;
        }
        case TrackType::barrier:      //障碍
        {
            preprocess.preview = preprocess.curve_preview / 100;
            break;
        }
        case TrackType::model:        //模型贴边线绕行
        {
            preprocess.preview = preprocess.model_preview / 100;
            break;
        }
        default:
        {
            preprocess.preview = preprocess.curve_preview / 100;
            break;
        }
            
    }
}

//局域静态变量
static float smoothed_speed = 0.0f;     // 平滑后的目标速度
static float speed_accumulator = 0.0f;  // 小数累加器
//速度前瞻决策(根据元素决定默认速度和前瞻)
void speed_prospect_decision(void)
{
    float target_raw = 0.0f;        //当前元素的目标速度
    switch (element_state.current_track_type)     //道路类型
    {
        case TrackType::normal:        //正常道路
        {   
            switch(element_state.normal_road_state)
            {
                case NormalRoadState::straight_fast:  //长直道高速
                {
                    preprocess.preview = preprocess.straight_preview / 100;
                    target_raw = preprocess.normal_speed;
                    break;
                }
                case NormalRoadState::safe:  //安全过渡
                {
                    preprocess.preview = preprocess.safe_preview / 100;
                    target_raw = preprocess.safe_speed;
                    break;
                }
                case NormalRoadState::curve:  //弯道
                {
                    preprocess.preview = preprocess.curve_preview / 100;
                    target_raw = preprocess.curve_speed;
                    break;
                }
                default:
                {
                    preprocess.preview = preprocess.curve_preview / 100;
                    target_raw = preprocess.curve_speed;
                    break;
                }

            }
            
            break;
        }
        case TrackType::crossroad:     //十字
        {
            preprocess.preview = preprocess.crossing_preview / 100;
            target_raw = preprocess.cross_speed;
            break;
        }
        case TrackType::circle:        //环岛
        {
            preprocess.preview = preprocess.ring_preview / 100;
            target_raw = preprocess.ring_speed;
            break;
        }  
        case TrackType::zebra:        //斑马线
        {
            preprocess.preview = preprocess.straight_preview / 100;
            target_raw = preprocess.normal_speed * 0.90f;
            break;
        } 
        case TrackType::ramp:         //坡道
        {
            preprocess.preview = preprocess.curve_preview / 100;
            target_raw = preprocess.ramp_speed;
            break;
        }
        case TrackType::barrier:      //障碍
        {
            preprocess.preview = preprocess.curve_preview / 100;
            target_raw = preprocess.curve_speed;
            break;
        }
        case TrackType::model:        //模型贴边线绕行
        {
            preprocess.preview = preprocess.model_preview / 100;
            target_raw = preprocess.model_speed;
            break;
        }
        default:
        {
            preprocess.preview = preprocess.curve_preview / 100;
            target_raw = preprocess.curve_speed;
            break;
        }
            
    }

    ////速率限制平滑////
    //设置步长
    float max_step = 0.0f;
    float diff = target_raw - smoothed_speed;
    if(diff > 0.0f) //加速状态
    {
        max_step = preprocess.max_accel_per_sec;
    }
    else if(diff < 0.0f)   //减速状态
    {
        max_step = preprocess.max_decel_per_sec;
    }
    else
    {
        speed_accumulator = 0.0f;
    }

    //速度平滑
    if(smoothed_speed == 0.0f)  //首次调用时初始化
    {
        smoothed_speed = target_raw;
        speed_accumulator = 0.0f;
    }
    else
    {
        rate_limiter(target_raw, smoothed_speed, max_step, speed_accumulator);
    }

    //// ========== 3. 赋值给全局 car_speed ========== ////
    car_speed_get_mutex.lock();
    car_speed = smoothed_speed;
    car_speed_get_mutex.unlock();

}

TrackType current_element;  //定义当前元素状态
TrackType last_element;     //定义上一次元素状态
//蜂鸣器控制决策
void buzzer_decision(void)
{
    current_element = element_state.current_track_type;    //目前的元素状态

    //蜂鸣器控制////
    remind.buzzer_work_control(200);    //单次鸣叫200ms
    if(current_element != last_element) 
    {
        if(buzzer_work_time <= 0)   //上次计数完成时，才响
        {
            remind.remind_on(); //当前元素不等于上次元素时
        }
    }
    else    //与上一帧元素相等且蜂鸣器未使能时（等上一次蜂鸣器提示结束）,关闭提醒
    {
        if(! buzzer_enable)
        {
            remind.remind_off();
        }
    }

    last_element = current_element;     //更新上一次的状态
}

//缓慢帧跳过决策（防止因图像刷新慢而导致绕行动作滞后）
void frame_pass_decision(void)
{
    if(slow_frame_pass)
    {
        slow_frame_pass_time --;

        if(track_base.current_mode == TrackMode::left_raw)
        {
            final_photo_err = 0.0f - SLOW_PASS_ERROR;
            if(slow_frame_pass_time >= SLOW_PASS_TIME - 1)
            std::cout << "左打死！" << std::endl;
        }
        else if(track_base.current_mode == TrackMode::right_raw)
        {
            final_photo_err = SLOW_PASS_ERROR;
            if(slow_frame_pass_time >= SLOW_PASS_TIME - 1)
            std::cout << "右打死！" << std::endl;
        }

        if(slow_frame_pass_time <= 0 && slow_frame_pass)
        {
            slow_frame_pass_time = SLOW_PASS_TIME;
            slow_frame_pass = false;
            std::cout << "缓慢帧跳过结束" << std::endl;
        }
    }
}

//陀螺仪检测(发车前采样，观察是否数据乱跳)
void gyroscope_check(void)
{
    float sum_yaw = 0.0f;   //偏航角速度累加值
    float aver_yaw = 0.0f;  //平均角速度
    float yaw_t = 0.0f;     //存放角速度临时变量
    const int counts = 250; //采样次数

    std::cout << "---开始进行陀螺仪检测---" << std::endl;

    for (int i = 0; i < counts; i++)
    {
        yaw_t = yaw_gyro;
        if(yaw_t < 0.0f) yaw_t = 0.0f;
        sum_yaw += yaw_gyro;    //累加
        usleep(100); 
    }

    aver_yaw = sum_yaw / counts;    //算出当前平均角速度（正数）

    //判断是否乱跳
    if(aver_yaw > 0.5f)
    {
        std::cout << "xxx陀螺仪数据跳变,停止发车xxx" << std::endl;

        fans.set_duty(0.0f);    //关负压(保险)
        Gogo = false;   //停车
        gyro_check_ok = false;  //标志位复位
    }
    else
    {
        std::cout << "￥￥￥陀螺仪数据正常,允许发车￥￥￥" << std::endl;
        gyro_check_ok = true;  //标志位置位
    }
}


//速度->角度双环控制
int temp_out = 0;    //定义临时环外环解算周期
void double_circle_control(float tar_angle)  //可设置目标角度
{
    //一加一减
    float steer = Temp_PID.output;     //定义PID输出值
    if(ladrc_on)
    {
        LADRC_Lmotor(car_speed - steer);       //左电机控速
        LADRC_Rmotor(car_speed + steer);       //右电机控速
    }
    else
    {
        PID_Lmotor(car_speed - steer);       //左电机控速
        PID_Rmotor(car_speed + steer);       //右电机控速
    }

    //角度环PID控制
    temp_out ++;
    if(temp_out > 5)
    {
        //获取陀螺仪数据
        float current_yaw = yaw_angle;

        Positional_PID_Cal(&Temp_PID,tar_angle,current_yaw);  //暂时去除图像环，用电机和偏航角闭环
    }
}

//常规完赛控制(无速度决策)
void common_control(void) 
{
    ////控制决策////
    if(element_state.model_line_pass_active)
    {
        element_ctrl.normal_control(); // 模型贴边线绕行
    }
    else if(pass_state == none && pass_direction == no_pass &&
            turn_state == no_turn && !element_state.model_line_pass_active)
    {
        element_ctrl.normal_control();  //常规三串PID控制
    }
    else if(pass_state == none && pass_direction == no_pass && 
            turn_state != no_turn && !element_state.model_line_pass_active)
    {
        element_ctrl.turn_control(&turn_state,TURN_TARGET_ANGLE,TURN_TARGET_DISTANCE);      //转向控制
    }
    // else if(pass_direction != no_pass && turn_state == no_turn)
    // {
    //     // element_ctrl.pass_control(&pass_direction,PASS_TARGET_ANGLE,PASS_TARGET_DISTANCE);  //绕行控制
    //     element_ctrl.dynamic_pass_control(&pass_direction,PASS_TARGET_ANGLE);   //目标角度动态变化绕行控制
    // }
}

//速度决策执行周期
static uint8_t speed_call_div = 0;
//元素类型
TrackType last_type = TrackType::normal;    //记录上次元素类型
TrackType current_type = TrackType::normal; //记录当前元素类型
//控制决策实现(写入循环线程)
void control_decision(void)
{
    ////速度前瞻决策////
    speed_prospect_decision();

    ////蜂鸣器控制决策////
    // buzzer_decision();

    ////控制决策////
    if(pass_state == none && pass_direction == no_pass && turn_state == no_turn)
    {
        switch (current_type)     //道路类型
        {
            case TrackType::normal:
            {
                switch (element_state.normal_road_state)
                {
                    case NormalRoadState::straight_fast:    //直道加速
                    {
                        element_ctrl.straight_control();   //直道控制(弱转向)
                        break;
                    }
                    case NormalRoadState::safe:
                    {
                        element_ctrl.safe_control();       //安全过渡控制(强转向)
                        break;
                    }
                    case NormalRoadState::curve:
                    {
                        element_ctrl.curve_control();      //弯道控制(强转向)
                        break;
                    }
                    default:
                    {
                        element_ctrl.curve_control();      //弯道控制(强转向)
                        break;
                    }    
                }
                break;
            }
            case TrackType::crossroad:
            {
                element_ctrl.crossing_control();    //十字控制(弱转向)
                break;
            }
            case TrackType::circle:
            {
                element_ctrl.ring_control();        //圆环控制(强转向)
                break;
            }
            case TrackType::barrier:
            {
                element_ctrl.curve_control();
                break;
            }
            case TrackType::ramp:
            {
                element_ctrl.straight_control();
                break;
            }
            case TrackType::model:
            {
                element_ctrl.model_control(); // 模型贴边线绕行
                break;
            }
            default :
            {
                element_ctrl.curve_control();       //弯道控制(强转向)
                break;
            }
        }
    }
    else if(pass_state == none && pass_direction == no_pass && turn_state != no_turn)
    {
        element_ctrl.turn_control(&turn_state,TURN_TARGET_ANGLE,TURN_TARGET_DISTANCE);      //转向控制
    }
    else if(pass_direction != no_pass && turn_state == no_turn)
    {
        // element_ctrl.pass_control(&pass_direction,PASS_TARGET_ANGLE,PASS_TARGET_DISTANCE);  //绕行控制
        element_ctrl.dynamic_pass_control(&pass_direction,PASS_TARGET_ANGLE);   //绕行控制
    }
    
}



/**************************坟场**************************/

// //只减不加
// float steer = Temp_PID.output;     //定义PID输出值
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

//----误差前瞻决策----//
// if(++speed_call_div >= 3)   //3ms执行一次速度前瞻决策
// {
//     speed_call_div = 0;
//     speed_prospect_decision();
// }

//----确定元素类型和误差平滑决策(用于退出模型元素那段时间也保持误差平滑)----//
// current_type = element_state.current_track_type;  //记录当前元素类型
// if(current_type == TrackType::model && last_type != TrackType::model)   //刚进入模型类别
// {
//     pass_model_time = 150 / 5; //刷新绕行时间
// }
// else if(current_type != TrackType::model && last_type == TrackType::model)  //刚退出模型类别
// {
//     err_smooth = true;  //仍然启动误差平滑（出了模型类型以外的元素使用）
// }
// if(err_smooth == true && pass_model_time > 0) pass_model_time --;   //误差平滑状态时开始计时
// if(pass_model_time <= 0 && err_smooth == true) err_smooth = false;  //计时结束且处于误差平滑状态，退出

// last_type = element_state.current_track_type; //刷新上一次元素类型
