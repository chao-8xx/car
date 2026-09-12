#include "headfile.h"
#include "pthread_control.h"

/***********定义变量***********/
bool reset_state = false;   //状态复位标志
int stop_time = 0;          //停止时间(防止一上电进入强制停车)
/***********声明变量***********/
extern std::atomic<bool> program_run_flag;           // 程序运行标志

/***********声明类和对象***********/
extern Motor motor;
extern LCD lcd;
extern Key key0;
extern CameraStreamServer camera_server;
/***********声明结构体和枚举***********/
extern Menu_Folder *key;

/***********声明线程***********/
void signal_handler_thread();
void frame_get_thread(Camera &cam);
void frame_handle_thread(CameraStreamServer &server);
/******************初始化启动******************/
//启动多线程
void pthread_start(void)
{
    //创建主控制线程(Ts ms)执行一次 motor_ctrl 内的内容
    TimerThread motorThread(motor_ctrl, NULL, Ts);
    //启动线程
    motorThread.start();

    //创建编码器读取线程(Ts ms)执行一次 motor_ctrl 内的内容
    TimerThread enconderThread(enconder_get, NULL, Ts);
    //启动线程
    enconderThread.start();

    //创建陀螺仪解算线程(2 ms)执行一次 vofa_ctrl 内的内容
    TimerThread gyroThread(gyro_cal, NULL, 2);
    //启动线程
    gyroThread.start();

    // // 创建按键VOFA线程(3 ms)执行一次 vofa_ctrl 内的内容
    // TimerThread vofaThread(vofa_ctrl, NULL, 3);
    // //启动线程
    // vofaThread.start();

    // //tof测距线程
    // std::thread tof_worker_thread(tof_thread, nullptr);

    // //创建按键提醒线程(3 ms)执行一次 remind_ctrl 内的内容
    // TimerThread remindThread(remind_ctrl, NULL, 3);
    // //启动线程
    // remindThread.start();

    //启动信号监控线程
    std::thread quit_thread(signal_handler_thread);
    //按键监听线程
    std::thread key_worker_thread(key_thread, nullptr);
    
    // 启动工作线程
    // 注: std::ref()创建引用包装器 传递引用而非拷贝 提高效率
    std::thread frame_producer(frame_get_thread, std::ref(camera));
    std::thread frame_consumer(frame_handle_thread, std::ref(camera_server));

    std::cout<<"-----------------------\n"<<">_<各个线程已开启>_<\n"
    <<">v<冯小超，准备出发>v<\n"<<"-----------------------\n"<<std::endl;
    
    // 回收线程
    frame_producer.join();
    frame_consumer.join();

    // 发送一个无用信号给自己，确保 sigwait 退出 (可选，通常 Ctrl+C 已经触发了它)
    quit_thread.join();
    key_worker_thread.join(); 
    // tof_worker_thread.join();
}


/***************************多线程控制***************************/
//按键监听线程
void key_thread(void *arg) 
{
    (void)arg;
    // key_listeners() 内部是一次阻塞 read，只处理一次事件就会返回。
    // 这里必须用循环持续监听，否则会出现“只能按一次键，之后菜单不再响应”
    while (program_run_flag) {
        key0.key_listeners();

        // 当修改数字时实时刷新
        extern Menu_Folder *key;
        if (key != nullptr && (key->kind == int_Box || key->kind == float_Box)) {
            show_number(); 
            
            // 实时更新摄像头参数
            camera.set_brightness(preprocess.brightness);
            camera.set_contrast(preprocess.contrast);
            camera.set_sharpness(preprocess.sharpness);
            camera.set_saturation(preprocess.saturation);
            camera.set_gain(preprocess.gain);
            camera.set_exposure_absolute(preprocess.exposure);
            
            // 实时保存参数
            preprocess.Preprocess_save();
        }
    }
}

//主控制线程
void motor_ctrl(void *arg)
{  
    if(Gogo)
    {
        if(!is_angle)
        {
            //清除误差、积分和输出
            pid_ErrOut_clear();
            ladrc_Err_clear();

            //状态复位
            state_reset();
            reset_state = false;    //初始化复位状态
            stop_time = 1000;       //初始化停车时间为1s

            //陀螺仪解算初始化
            madg.angle_reset();         //各角度和相关数据复位
            madg.madgwickCtrl_init();   //陀螺仪初始化
            yaw_angle = 0;
            roll_angle = 0;
            pitch_angle = 0;
            yaw_gyro = 0;

            //初始化目标速度为正常速度
            car_speed = preprocess.normal_speed;
            //初始化前瞻为5.0f
            preprocess.preview = 5.0f / 100.0f;

            //翻转状态
            is_angle = !is_angle;
        }
        else
        {
            //发车前进行陀螺仪检测
            if(!gyro_check_ok)
            {
                gyroscope_check();
            }

            if(is_start == 1 && gyro_check_ok)  //陀螺仪检测完成后起步发车
            {
                if(fans_ok_time < 1500) //负压稳定时间小于1.5s时
                {
                    fans_decision();    //执行负压控制
                }
                else
                {
                    car_start();   //智能车平滑起步
                }
                // car_start();   //智能车平滑起步
            }
            else if(is_start == 0 && gyro_check_ok)
            {
                if(speed_deci_on)   //是否启动速度决策
                {
                    control_decision();      //速度决策
                }
                else
                {
                    common_control();        //常规三串PID控制
                }
            }
        }
    }
    else
    {
        ////强行停止////
        if(stop_time > 0)    //1s 时间强行停止
        {
            // car_stop();
            car_soft_stop();    //智能车软停止（保护电机）
            fans.set_duty(0);
            stop_time --;
        }
        ////清除误差、积分以及状态复位////
        else if(!reset_state && stop_time <= 0)    //执行一次即可
        {
            //清除误差和积分
            pid_ErrOut_clear(); //清除PID误差、积分、输出
            ladrc_Err_clear();

            ////停车及其参数初始化////
            car_soft_stop();    //智能车软停止
            fans.set_duty(0);   //关闭负压(再次关闭)

            //关闭提醒
            is_remind = false;

            ////状态复位////
            state_reset();

            reset_state = true;     //复位状态设置为true

            // //关闭蜂鸣器
            // remind.buzzer_enable_off();
            // remind.buzzer_off();
        }

        ////获取误差////
        // prospect_decision(); //前瞻决策获取图像误差

        // photo_err_get_mutex.lock();
        // photo_err = track_base.get_error_lookahead(100,preprocess.preview,preprocess.window_size);
        // photo_err_get_mutex.unlock();

        ////蜂鸣器////
        // buzzer_decision();  //蜂鸣器控制决策

        ////参数锁定////
        // static uint8_t lock_div = 0;
        // lock_div ++;
        // if(lock_div > 100)    //100ms进行一次参数检测
        // {
        //     lock_div = 0;
        //     if(para_lock)
        //     {
        //         element_ctrl.parament_lock();   //参数锁定（根据目标速度）
        //     }   
        // }
    }
}

//编码器获取线程
void enconder_get(void *arg)
{
    //读取编码器数据
    motor.update_encoders();

    //刷新速度值
    float l_read = -motor.encoder1_counts * 1.0f;   //左编码器原始值
    float r_read = -motor.encoder2_counts * 1.0f;   //右编码器原始值

    //编码器滤波
    L_speed = L_enconder_pass.update(l_read);
    R_speed = R_enconder_pass.update(r_read);

    //获取当前车速速度
    average_speed = (L_speed + R_speed)/2.0f;
}

//陀螺仪解算与判断
void gyro_cal(void *arg)
{
    if(is_angle)    //is_angle(陀螺仪初始化后)
    {
        //计算陀螺仪数据
        madg.getIMU_Data();         //获取角速度、角加速度
        madg.pose_cal();            //角度解算
        icm42688.thread_syn();      //线程同步
        
        //获取陀螺仪数据
        yaw_gyro_get();         //获取偏航角速度
        all_angle_get();        //获取所有角

        //偏航角速度极小区间保护
        if(yaw_gyro > -0.5 && yaw_gyro < 0.5) yaw_gyro = 0; //角速度在极小范围内为0
        else yaw_gyro = yaw_gyro;

        //偏航角中线复位
        if(yaw_reset)   //偏航角中线复位允许的情况下
        {
            if(photo_err > -1.0f && photo_err < 1.0f)   //图像误差小时，复位偏航角
            {
                madg.yaw_angle_reset();    //角度复位
            }
        }
    }
}

// //vofa读取数据
// void vofa_ctrl(void *arg)
// {
//     vofa.vofa_read(L_speed,L_target,R_speed,R_target,0,0);
// }

// // 独立的TOF数据获取线程，类似于单片机里的定时器
// // 避免在主图像处理循环中阻塞，提高程序流畅度
// void tof_thread(void *arg) {
//     (void)arg;
//     while (program_run_flag) {
//         tof_distance_get();
//         // 挂起大约33ms
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

// // 音乐播放线程
// void music_thread(void *arg) {
//     (void)arg;
//     // 使用异步播放器播放《晴天》
//     music_player.play(qing_tian, sizeof(qing_tian) / sizeof(Note));
// }


// TrackType remind_last_type = TrackType::normal;    //记录上次元素类型
// TrackType remind_current_type = TrackType::normal; //记录当前元素类型
// //提醒控制
// void remind_ctrl(void *arg)
// {
//     if(is_remind)
//     {
//         remind_current_type = element_state.current_track_type;  //记录当前元素类型
//         //屏幕
//         static rgb565_color_enum rgb = LCD_YELLOW;
//         if(element_state.model_line_pass_dir == ModelLinePassDir::left) //判断屏幕颜色
//         {
//             rgb = LCD_RED;
//         }
//         else if (element_state.model_line_pass_dir == ModelLinePassDir::right)
//         {
//             rgb = LCD_GREEN;
//         }
//         else
//         {
//             rgb = LCD_BROWN;
//         }

//         if(remind_current_type == TrackType::model && remind_last_type != TrackType::model)    //判断是否进入模型
//         {
//             remind.is_screen_remind = true;
//         }
//         else if(remind_current_type != TrackType::model && remind_last_type == TrackType::model)
//         {
//             remind.is_screen_clear = true;
//         }
//         remind.lcd_remind(rgb);
        

//         //蜂鸣器
//         if(remind_current_type == TrackType::model)
//         {
//             remind.buzzer_on();
//         }
//         else
//         {
//             remind.buzzer_off();
//         }
//         remind_last_type = element_state.current_track_type; //刷新上一次元素类型
//     }
// }

// element_state.model_line_pass_dir
