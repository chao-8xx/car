#include "global.h"

/***********定义类***********/
LCD lcd ;
Key key0;
Motor motor;

/**************************代码封装**************************/
//整体初始化
void all_init(void)
{
    /************各设备和算法初始化************/
    key0.key_init();                //按键
    lcd.lcd_init();                 //屏幕
    menu_init();                    //菜单
    preprocess.Preprocess_init();   //json
    model_calib.load_saved_queue(); //模型菜单动作队列
    track_base.track_init();        //巡线
    element_state.element_init();   //元素识别
    show_menu();                    //菜单显示
    madg.angle_reset();             //各角度和相关数据复位
    madg.madgwickCtrl_init();       //陀螺仪解算
    motor.motor_init();             //电机
    fans.brushless_init();          //负压
    fans.set_duty(0);
    // tof_Init();                     //tof
    // vofa.vofa_init();               //vofa
    // remind.buzzer_init();           //蜂鸣器
    
    
    /************参数初始化************/
    //LADRC参数初始化
    LADRC_Init(&right_motor,Ts);
    LADRC_Init(&left_motor,Ts);
    LADRC_motor_init(); //电机LADRC参数初始化

    //速度环位置式PID初始化
    Positional_PID_Init(&Rmotor_PID,preprocess.R_Kp,preprocess.R_Ki,preprocess.R_Kd,6000,-8000,8000);
    Positional_PID_Init(&Lmotor_PID,preprocess.L_Kp,preprocess.L_Ki,preprocess.L_Kd,6000,-8000,8000);

    //位置式PID初始化
    Positional_PID_Init(&Angle_PID,preprocess.Ag_Kp,preprocess.Ag_Ki/10,preprocess.Ag_Kd,800,-1000,1000);  //角速度环
    Positional_PID_Init(&Photo_PID,preprocess.Ph_Kp,preprocess.Ph_Ki/10,preprocess.Ph_Kd,800,-1000,1000);  //图像环

    //摄像头参数调节
    camera.set_brightness(preprocess.brightness);           // 亮度
    camera.set_contrast(preprocess.contrast);               // 对比度
    camera.set_sharpness(preprocess.sharpness);             // 锐度
    camera.set_saturation(preprocess.saturation);           // 饱和度
    camera.set_gain(preprocess.gain);                       // 增益
    camera.set_auto_exposure(false);                        // 手动控制曝光（先关闭自动曝光）
    camera.set_exposure_absolute(preprocess.exposure);      // 曝光

}

/******************定义变量******************/
//陀螺仪
float yaw_gyro = 0.0f;              //偏航角角速度
float yaw_angle = 0.0f;             //偏航角
float pitch_angle = 0.0f;           //俯仰角
float roll_angle = 0.0f;            //翻滚角

//TOF
float tof_distance = 0.0f;          //tof测距

//蜂鸣器
int buzzer_work_time = 0;           //蜂鸣器工作时间

//智能车状态
float motor_distance = 0.0f;        //电机行驶距离（编码器积分）
int motor_enable_time = 0;          //电机启动倒计时
int is_start = 1;                   //智能车起步状态(1为起步状态，0结束起步状态)
float Ts = 1.0f;                    //进入电机控制线程的周期

//有刷电机
float L_speed = 0.0f;               //左电机速度
float R_speed  = 0.0f;              //右电机速度
float average_speed = 0.0f;         //平均速度
int max_pwm = 7000;                 //最大输出PWM
float R_pwm = 0.0f;                 //右电机输出PWM
float L_pwm = 0.0f;                 //左电机输出PWM
//无刷电机
int fans_duty = 0;                  //负压电机占空比
int fans_time = 0;                  //负压启动计时
int fans_ok_time = 0;               //负压启动成功后稳定时间(启动完后等待一段时间再发车)

//PID变量
int out_count = 0;                  //外环解算时间
int in_count = 0;                   //内环解算时间
float photo_err = 0;                //定义外环图像环误差
float photo_out = 0.0f;             //定义外环图像环输出
float angle_out = 0.0f;             //定义内环角度环输出
float final_photo_err = 0.0f;       //绕行过渡时最终的图像误差

//定义目标值
int car_speed = preprocess.normal_speed;     //智能车目标速度
int L_target = 0;                            //左电机目标速度
int R_target = 0;                            //右电机目标速度

//使能控制
bool fans_enable = false;           //负压使能
bool motor_enable = false;          //电机使能
bool motor_enable_enable = false;   //使能电机使能（主要是为了搭配电机启动倒计时，实现电机的延时启动）
bool buzzer_enable = false;         //蜂鸣器使能
bool buzzer_count_enable = false;   //蜂鸣器工作倒计时使能
bool is_remind = false;             //是否启动提醒
bool Gogo = false;                  //发车总控制

//控制模式
bool ladrc_on = false;              //是否启动ladrc自抗扰
bool speed_deci_on = false;         //是否启动速度决策
bool para_lock = false;             //是否启动参数锁定
bool fans_ok = false;               //负压是否软启ok

//判断
bool is_lose = true;                //判断是否丢线
bool is_angle = false;              //陀螺仪是否初始化完成
bool is_buzzer = false;             //蜂鸣器是否开始提醒
bool yaw_reset = true;              //偏航角复位标志（true的话对准中线时yaw复位为0）
bool gyro_check_ok = false;         //陀螺仪检测是否完成

//错误帧跳过
bool slow_frame_pass = false;                   //是否启动错误帧跳过
int slow_frame_pass_time = SLOW_PASS_TIME;      //缓慢帧跳过时间（ms）

//斑马线延时急停
bool wait_stop = false;            //是否等待急停
int wait_stop_time = 5;            //等待5帧后急停

