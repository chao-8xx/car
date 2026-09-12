#ifndef __PREPROCESS_H_
#define __PREPROCESS_H_

#include "headfile.h"

using namespace cv;

class Preprocess
{
public:
    /* 此处可以存放可调试的变量 */
    //相机参数
    int brightness;                  // 亮度
    int contrast;                    // 对比度
    int sharpness;                   // 锐度
    int saturation;                  // 饱和度            
    int gain;                        // 增益
    int exposure;                    // 曝光度

    //参数系数
    float preview;                   //前瞻因子
    int window_size;                 //可视化窗口大小
    float max_accel_per_sec;         //每秒最大加速度，单位：脉冲/秒²(速度平滑参数)
    float max_decel_per_sec;         //每秒最大减速度，单位：脉冲/秒²(速度平滑参数)
    float fix_speed;                 //修速系数，根据图像误差对目标速度进行动态修改
    float yaw_distance;              //绕行系数（绕行时目标偏航角随编码器积分的变化率）

    //元素前瞻
    float straight_preview;          //直道前瞻
    float safe_preview;              //过渡前瞻
    float curve_preview;             //弯道前瞻
    float crossing_preview;          //十字前瞻
    float ring_preview;              //圆环前瞻
    float model_preview;             //模型前瞻
    float model_circle_outer_preview; //圆环内模型外绕前瞻

    //圆环标注
    int circle_count;                 // 圆环数量
    int ring1_type;                   // 0: small, 1: medium, 2: large, 3: big
    int ring2_type;
    int ring3_type;
    int ring4_type;
    int ring5_type;
    int ring6_type;
    int ring7_type;
    int ring8_type;
    int ring9_type;
    int ring10_type;
    int ring11_type;
    int ring12_type;
    int ring13_type;
    int ring14_type;
    int ring15_type;

    float circle_small_exit_yaw;            // 小圆环出环所需角度值 (正常)
    float circle_small_exit_force_yaw;      // 小圆环强制出环所需角度值
    float circle_small_exit_min_yaw;        // 小圆环出环最小偏航角
    float circle_small_exit_max_yaw;        // 小圆环出环最大偏航角
    float circle_small_exit_k_start;        // 小圆环出环开始斜率
    float circle_small_exit_k_end;          // 小圆环出环最后斜率
    // int circle_small_leave_confirm_frames;  // 小圆环离开确认帧数
    float circle_small_leave_min_distance;  // 小圆环离开最小距离
    // float circle_small_preview_scale;       // 小圆环预览缩放比例

    float circle_medium_exit_yaw;            // 中圆环出环所需角度值 (正常)
    float circle_medium_exit_force_yaw;      // 中圆环强制出环所需角度值
    float circle_medium_exit_min_yaw;        // 中圆环出环最小偏航角
    float circle_medium_exit_max_yaw;        // 中圆环出环最大偏航角
    float circle_medium_exit_k_start;        // 中圆环出环开始斜率
    float circle_medium_exit_k_end;          // 中圆环出环最后斜率
    // int circle_medium_leave_confirm_frames;  // 中圆环离开确认帧数
    float circle_medium_leave_min_distance;  // 中圆环离开最小距离
    // float circle_medium_preview_scale;       // 中圆环预览缩放比例

    float circle_large_exit_yaw;            // 第三小圆环出环所需角度值 (正常)
    float circle_large_exit_force_yaw;      // 第三小圆环强制出环所需角度值
    float circle_large_exit_min_yaw;        // 第三小圆环出环最小偏航角
    float circle_large_exit_max_yaw;        // 第三小圆环出环最大偏航角
    float circle_large_exit_k_start;        // 第三小圆环出环开始斜率
    float circle_large_exit_k_end;          // 第三小圆环出环最后斜率
    int circle_large_leave_confirm_frames;  // 第三小圆环离开确认帧数
    float circle_large_leave_min_distance;  // 第三小圆环离开最小距离
    float circle_large_preview_scale;       // 第三小圆环预览缩放比例

    float circle_big_exit_yaw;               // 大圆环出环所需角度值 (正常)
    float circle_big_exit_force_yaw;         // 大圆环强制出环所需角度值
    float circle_big_exit_min_yaw;           // 大圆环出环最小偏航角
    float circle_big_exit_max_yaw;           // 大圆环出环最大偏航角
    float circle_big_exit_k_start;           // 大圆环出环开始斜率
    float circle_big_exit_k_end;             // 大圆环出环最后斜率
    // int circle_big_leave_confirm_frames;  // 大圆环离开确认帧数
    float circle_big_leave_min_distance;     // 大圆环离开最小距离
    // float circle_big_preview_scale;          // 大圆环预览缩放比例

    //LADRC参数
    float L_wc;                       //左电机控制器
    float L_w0;                       //左电机观测器
    float L_b;                        //左电机补偿系数
    float R_wc;                       //右电机控制器
    float R_w0;                       //右电机观测器
    float R_b;                        //右电机补偿系数

    //PID闭环参数
    float Ag_Kp;                       //角速度环Kp
    float Ag_Ki;                       //角速度环Ki
    float Ag_Kd;                       //角度环Kd
    float Ag_F_Kp;                     //角速度环Kp(PD+前馈)
    float Ag_F_Kd;                     //角速度环Kd(PD+前馈)
    float Ag_Kff;                      //角速度环前馈系数(PD+前馈)
    float Ag_Kff_acc;                  //角速度环前馈变化率系数(PD+前馈)
    float Ph_Kp;                       //图像环Kp
    float Ph_Ki;                       //图像环Ki
    float Ph_Kd;                       //图像环Kd
    float L_Kp;                        //左电机Kp
    float L_Ki;                        //左电机Ki
    float L_Kd;                        //左电机Kd
    float R_Kp;                        //左电机Kp
    float R_Ki;                        //左电机Ki
    float R_Kd;                        //左电机Kd
    float Te_Kp;                       //临时环Kp
    float Te_Ki;                       //临时环Ki
    float Te_Kd;                       //临时环Kd

    //弯道强度参数
    float curve_100_Kp;                //100速度的图像环Kp
    float curve_100_Kd;                //100速度的图像环Kd
    float curve_110_Kp;                //110速度的图像环Kp
    float curve_110_Kd;                //110速度的图像环Kd
    float curve_120_Kp;                //120速度的图像环Kp
    float curve_120_Kd;                //120速度的图像环Kd
    float curve_130_Kp;                //130速度的图像环Kp
    float curve_130_Kd;                //130速度的图像环Kd
    float curve_140_Kp;                //140速度的图像环Kp
    float curve_140_Kd;                //140速度的图像环Kd
    float curve_150_Kp;                //150速度的图像环Kp
    float curve_150_Kd;                //150速度的图像环Kd
    float curve_160_Kp;                //160速度的图像环Kp
    float curve_160_Kd;                //160速度的图像环Kd

    //PID各个元素闭环参数
    float straight_F_Kp;               //直道角速度环Kp
    float straight_F_Kd;               //直道角速度环Kd
    float straight_F_Kff;              //直道角速度环Kff
    float straight_F_Kff_acc;          //直道角速度环Kff_acc
    float straight_Ag_Kp;              //直道角速度环Kp
    float straight_Ag_Ki;              //直道角速度环Ki
    float straight_Ag_Kd;              //直道角速度环Kd
    float straight_Ph_Kp;              //直道图像环Kp
    float straight_Ph_Ki;              //直道图像环Ki
    float straight_Ph_Kd;              //直道图像环Kd

    float safe_F_Kp;                  //安全过渡角速度环Kp
    float safe_F_Kd;                  //安全过渡角速度环Kd
    float safe_F_Kff;                 //安全过渡角速度环Kff
    float safe_F_Kff_acc;             //安全过渡角速度环Kff_acc
    float safe_Ag_Kp;                 //安全过渡角速度环Kp
    float safe_Ag_Ki;                 //安全过渡角速度环Ki
    float safe_Ag_Kd;                 //安全过渡角速度环Kd
    float safe_Ph_Kp;                 //安全过渡图像环Kp
    float safe_Ph_Ki;                 //安全过渡图像环Ki
    float safe_Ph_Kd;                 //安全过渡图像环Kd

    float curve_F_Kp;                  //弯道角速度环Kp
    float curve_F_Kd;                  //弯道角速度环Kd
    float curve_F_Kff;                 //弯道角速度环Kff
    float curve_F_Kff_acc;             //弯道角速度环Kff_acc
    float curve_Ag_Kp;                 //弯道角速度环Kp
    float curve_Ag_Ki;                 //弯道角速度环Ki
    float curve_Ag_Kd;                 //弯道角速度环Kd
    float curve_Ph_Kp;                 //弯道图像环Kp
    float curve_Ph_Ki;                 //弯道图像环Ki
    float curve_Ph_Kd;                 //弯道图像环Kd

    float crossing_F_Kp;               //十字角速度环Kp
    float crossing_F_Kd;               //十字角速度环Kd
    float crossing_F_Kff;              //十字角速度环Kff
    float crossing_F_Kff_acc;          //十字角速度环Kff_acc
    float crossing_Ag_Kp;              //十字角速度环Kp
    float crossing_Ag_Ki;              //十字角速度环Ki
    float crossing_Ag_Kd;              //十字角速度环Kd
    float crossing_Ph_Kp;              //十字图像环Kp
    float crossing_Ph_Ki;              //十字图像环Ki
    float crossing_Ph_Kd;              //十字图像环Kd

    float ring_F_Kp;                   //圆环角速度环Kp
    float ring_F_Kd;                   //圆环角速度环Kd
    float ring_F_Kff;                  //圆环角速度环Kff
    float ring_F_Kff_acc;              //圆环角速度环Kff_acc
    float ring_Ag_Kp;                  //圆环角速度环Kp
    float ring_Ag_Ki;                  //圆环角速度环Ki
    float ring_Ag_Kd;                  //圆环角速度环Kd
    float ring_Ph_Kp;                  //圆环图像环Kp
    float ring_Ph_Ki;                  //圆环图像环Ki
    float ring_Ph_Kd;                  //圆环图像环Kd

    float model_F_Kp;                  //模型角速度环Kp
    float model_F_Kd;                  //模型角速度环Kd
    float model_F_Kff;                 //模型角速度环Kff
    float model_F_Kff_acc;             //模型角速度环Kff_acc
    float model_Ag_Kp;                 //模型角速度环Kp
    float model_Ag_Ki;                 //模型角速度环Ki
    float model_Ag_Kd;                 //模型角速度环Kd
    float model_Ph_Kp;                 //模型图像环Kp
    float model_Ph_Ki;                 //模型图像环Ki
    float model_Ph_Kd;                 //模型图像环Kd

    //PID解算周期
    int inner_count;                   //内环解算周期
    int outer_count;                   //外环解算周期

    //模型各个子状态速度参数
    float model_kp_step;               //模型kp步长
    float model_side_kp_step;          //模型侧视目标kp步长
    int model_warning_speed;           //宽松红框识别到时速度
    int model_recognizing_speed;       //严格红框识别到时速度
    int model_side_speed;              //急弯/圆环侧视目标的识别及绕行速度
    int model_emergency_speed;         //未提前预警时的模型紧急识别速度
    int model_pass_speed;              //模型绕行速度
    int model_return_speed;            //模型回线过渡速度
    float model_circle_return_distance; //圆环内模型绕行的专用回线距离
    int model_fail_safe_speed;         //模型失败保护速度
    int model_marker_method;           //模型红框方法: 0=HSV, 1=YCrCb, 2=YUV
    int model_profile_enable;          //模型/视觉耗时统计开关
    int model_profile_interval;        //耗时统计打印间隔帧数
    
    //速度决策速度参数
    int normal_speed;                  //正常速度
    int safe_speed;                    //安全过渡速度
    int curve_speed;                   //弯道速度
    int cross_speed;                   //过十字路口时的速度
    int ring_speed;                    //圆环内速度
    int model_speed;                   //遇到图像时的速度
    int ramp_speed;                     //过坡道时的速度
    int barrier_speed;                 //遇到障碍时的速度


    Preprocess();
    ~Preprocess();

    int Preprocess_init();
    int Preprocess_save();
    
    // 调试数据打包
    std::string pack_debug_data();
    

private:
};

extern Preprocess preprocess;
extern std::mutex alg_mutex; // 保护算法数据结构的互斥锁，防止多线程与网页图传发生段错误

#endif
