#ifndef __GLOBAL_H
#define __GLOBAL_H
#include "headfile.h"

//限幅函数
#ifndef _scale
#define _scale(aim,MIN,MAX)  ((aim) < (MIN) ? (MIN) : ((aim) > (MAX) ? (MAX) : (aim)))
#endif  //_scale

/**************************代码封装**************************/
//全部初始化
void all_init(void);

/**************************定义变量**************************/
//陀螺仪
extern float yaw_gyro;             //偏航角角速度
extern float yaw_angle;            //偏航角
extern float pitch_angle;          //俯仰角
extern float roll_angle;           //翻滚角

//TOF
extern float tof_distance;         //tof测距

//蜂鸣器
extern int buzzer_work_time;       //蜂鸣器工作时间

//智能车状态
extern float motor_distance;       //电机行驶距离（编码器积分）
extern int motor_enable_time;      //电机启动倒计时
extern int is_start;               //智能车起步状态(1为起步状态，0结束起步状态)
extern float Ts;                   //进入电机控制线程的周期

//模型绕行变量
extern bool was_pass_active;       //上一帧是否处于模型绕行
extern bool continue_pass;         //模型绕行结束后的继续平滑过渡
extern uint32_t pass_model_time;   //模型绕行结束后的过渡计时

//电机变量
extern float L_speed;              //左电机速度
extern float R_speed;              //右电机速度
extern float average_speed;        //平均速度
extern int max_pwm;                //最大输出PWM
extern float L_pwm;                //左电机输出PWM
extern float R_pwm;                //右电机输出PWM
extern int fans_duty;              //负压电机占空比
extern int fans_time;              //负压启动计时
extern int fans_ok_time;           //负压启动成功后稳定时间

//PID变量
extern int out_count;              //外环解算周期
extern int in_count;               //内环解算周期
extern float photo_err;            //定义外环图像环误差
extern float photo_out;            //定义外环图像环输出
extern float angle_out;            //定义内环角度环输出
extern float final_photo_err;      //绕行过渡时最终的图像误差

//定义目标值
extern int car_speed;              //智能车目标速度
extern int L_target;               //左电机目标速度
extern int R_target;               //右电机目标速度

//使能控制
extern bool fans_enable;           //负压使能
extern bool motor_enable;          //电机使能
extern bool motor_enable_enable;   //使能电机使能（主要是为了搭配电机启动倒计时，实现电机的延时启动）
extern bool buzzer_enable;         //蜂鸣器使能
extern bool buzzer_count_enable;   //蜂鸣器工作倒计时使能
extern bool ladrc_on;              //是否启动ladrc自抗扰
extern bool speed_deci_on;         //是否启动速度决策
extern bool para_lock;             //是否启动参数锁定
extern bool fans_ok;               //负压是否软启ok
extern bool is_remind;             //是否启动提醒
extern bool Gogo;                  //发车总控制

//判断
extern bool is_lose;               //判断是否丢线
extern bool is_angle;              //陀螺仪是否初始化完成
extern bool is_buzzer;             //蜂鸣器是否开始提醒
extern bool yaw_reset;             //偏航角复位标志（true的话对准中线时yaw复位为0）
extern bool gyro_check_ok;         //陀螺仪检测是否完成

// 定义零漂变量
extern float gyro_x_offset;
extern float gyro_y_offset;
extern float gyro_z_offset;

//错误帧跳过
#define SLOW_PASS_ERROR   30.0f    //缓慢帧跳过所设置的误差
#define SLOW_PASS_TIME    18       //缓慢帧跳过时间宏定义（int）
extern bool slow_frame_pass ;      //是否启动缓慢帧跳过
extern int slow_frame_pass_time;   //缓慢帧跳过时间（ms）

//斑马线延时急停
extern bool wait_stop;             //是否等待急停
extern int wait_stop_time;         //等待50帧后急停(写入视觉代码)

#endif  //global

