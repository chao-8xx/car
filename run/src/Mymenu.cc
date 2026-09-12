#include "headfile.h"

// 声明类
extern LCD lcd;
extern Buzzer buzzer;
extern AsyncMusicPlayer music_player;
extern TrackBase track_base;
extern BiaoDingAuto biaoding_auto;
extern Key key0;

extern std::atomic<bool> program_run_flag;

// 摄像头参数全局变量
// extern int cam_brightness;
// extern int cam_contrast;
// extern int cam_sharpness;
// extern int cam_saturation;
// extern int cam_gain;
// extern int cam_exposure;

//显示字符串
#define     menu_show_string(_x_, _y_, __string__)                  ( lcd.showString( (_x_), (_y_), (__string__) )  )
//显示字符
#define     menu_show_char(_x_, _y_, _char_)                        ( lcd.showChar( (_x_), (_y_), (_char_) )        )
//显示整数
#define     menu_show_int(_x_, _y_, _int_, _len_)                   ( lcd.showInt( (_x_), (_y_), (_int_), (_len_) ) )
//显示浮点数
#define     menu_show_float(_x_, _y_, _float_, _len_z_, _len_x_)    ( lcd.showDouble( (_x_), (_y_), (_float_), (_len_z_), (_len_x_) ) )
//显示字符串
#define     menu_show_Chinese(_x_, _y_, __Chinese__)                ( lcd.showChineseString( (_x_), (_y_), (__Chinese__) )  )
//颜色翻转
#define     menu_show_key(_x_, _y_, _len_, _wid_)                   ( lcd.colorChange( (_x_), (_y_), (_len_), (_wid_) ) )
//清屏
#define     menu_show_clear()                                       ( lcd.clearScreen()  )
//颜色保持
#define     menu_show_keep(_x_, _y_, _len_, _wid_)                  ( lcd.colorKeep( (_x_), (_y_), (_len_), (_wid_) ) )

//显示缓存
static uint8_t AppBuffer[20][128];

//菜单头节点
Menu_Folder head; 
//菜单按键控制索引指针
Menu_Folder *key;

//菜单级数
int degree = 1;
//单次步长
int Int_step = 10;           //整形
float Float_step = 0.5f;     //浮点形

/***************** 外部声明或定义变量 *****************/
// extern float target_angle;
extern bool circle_label_active;   
extern int circle_label_index; 

/***************************************************/

// 菜单回调需要普通函数指针，这里用包装函数调用对象方法
// 图像显示回调函数
// ==============================================================
void track_display_menu(void) 
{
    lcd.clearScreen(); 

    // 阻塞循环：只负责监听按键，绝对不碰 OpenCV！
    while (key->execute && program_run_flag) 
    {

        // track_base.track_display();

        key0.key_listeners(); 
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }

    // 退出后恢复菜单
    lcd.clearScreen();
    show_menu();
}

// 全自动标注回调函数
void ipm_biaoding_menu(void) 
{
    lcd.clearScreen(); 

    biaoding_auto.start_biaoding();

    // 阻塞循环：只负责监听按键
    while (key->execute && program_run_flag && biaoding_auto.current_state != BiaoDingState::idle) 
    {
        key0.key_listeners(); 
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }
    biaoding_auto.current_state = BiaoDingState::idle;

    // 退出后恢复菜单
    lcd.clearScreen();
    show_menu();
}

// 圆环标注回调函数
void circle_label_menu(void)
{
    circle_label_active = true;
    circle_label_index = 0;
    preprocess.circle_count = 0;
    preprocess.Preprocess_save();

    lcd.clearScreen(); 
    biaoding_auto.show_circle_label_menu();

    while (key->execute && program_run_flag && circle_label_active)
    {
        key0.key_listeners();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    circle_label_active = false;
    lcd.clearScreen();
    show_menu();
}

// 模型队列回调函数
void model_action_queue_menu(void)
{
    lcd.clearScreen();
    model_calib.start_calibration();

    while (key->execute && program_run_flag && model_calib.current_state != CalibState::idle)
    {
        key0.key_listeners();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    model_calib.current_state = CalibState::idle;
    lcd.clearScreen();
    show_menu();
}


//菜单初始化
void menu_init()
{
    //菜单头节点初始化
    head.father = NULL;
    head.first_son = NULL;
    head.last_brother = NULL;
    head.next_brother = NULL;
    head.name = "MENU";
    head.kind = Normal_Folder;
    head.data = NULL;
    head.rank = 0;
    head.sons = 0; 
    head.function = NULL;

    //以下添加所增加的节点和变量
    /********************一级菜单节点********************/
    Menu_Folder* Gogogo = Create_Menu_Folder(&head,"出发咯");
    Menu_Folder* para_adjust = Create_Menu_Folder(&head,"参数调整");
    Menu_Folder* camera_para = Create_Menu_Folder(&head,"相机参数");
    Menu_Folder* manage = Create_Menu_Folder(&head,"管理");
    Menu_Folder* image_display = Create_Menu_Execute_Folder(&head,"图像显示", track_display_menu); //track_display_menu
    Menu_Folder* model_action_queue = Create_Menu_Execute_Folder(&head,"模型队列", model_action_queue_menu);
    Menu_Folder* inverse_perspective = Create_Menu_Execute_Folder(&head,"标注半宽", ipm_biaoding_menu); //ipm_biaoding_menu
    
    /********************二级菜单节点********************/
    //“参数调整”的子节点
    Menu_Folder* step_ctrl = Create_Menu_Folder(para_adjust,"步长");
    Menu_Folder* count_time = Create_Menu_Folder(para_adjust,"解算周期");
    Menu_Folder* LADRC_ctrl = Create_Menu_Folder(para_adjust,"电机");
    Menu_Folder* PID_ctrl = Create_Menu_Folder(para_adjust,"皮埃弟"); //PID
    Menu_Folder* preview_ctrl = Create_Menu_Folder(para_adjust,"元素前瞻");
    Menu_Folder* para_factor = Create_Menu_Folder(para_adjust,"加减速度");
    Menu_Folder* control_mode = Create_Menu_Folder(para_adjust,"控制模式");
    //“管理”的子节点
    Menu_Folder* plan = Create_Menu_Folder(manage,"方案管理");
    Menu_Folder* element = Create_Menu_Folder(manage,"元素管理");
    
    //"出发咯"的子节点
    Menu_Folder* speed_deci = Create_Menu_Folder(Gogogo,"速度决策");

    /********************三级菜单节点********************/
    //“自抗扰”的子节点
    Menu_Folder* L_motor = Create_Menu_Folder(LADRC_ctrl,"左电机");
    Menu_Folder* R_motor = Create_Menu_Folder(LADRC_ctrl,"右电机");
    //“PID”的子节点
    Menu_Folder* straight_pid = Create_Menu_Folder(PID_ctrl,"直道");
    Menu_Folder* safe_pid = Create_Menu_Folder(PID_ctrl,"过渡");
    Menu_Folder* curve_pid = Create_Menu_Folder(PID_ctrl,"弯道");
    Menu_Folder* crossing_pid = Create_Menu_Folder(PID_ctrl,"十字");
    Menu_Folder* ring_pid = Create_Menu_Folder(PID_ctrl,"圆环");
    Menu_Folder* model_pid = Create_Menu_Folder(PID_ctrl,"模型");
    Menu_Folder* normal_pid = Create_Menu_Folder(PID_ctrl,"正常");
    // Menu_Folder* Temp_Circle = Create_Menu_Folder(PID_ctrl,"绕行环");

    //"元素管理"子节点
    Menu_Folder* circle_folder = Create_Menu_Folder(element,"圆环");

    //"速度决策"的子节点
    Menu_Folder* model_state = Create_Menu_Folder(speed_deci,"模型");
    
    /********************四级菜单节点********************/
    //"圆环"子节点
    Create_Menu_Execute_Folder(circle_folder,"圆环标注", circle_label_menu);
    Menu_Folder* circle_folder_one = Create_Menu_Folder(element,"圆环一");
    Menu_Folder* circle_folder_two = Create_Menu_Folder(element,"圆环二");
    Menu_Folder* circle_folder_three = Create_Menu_Folder(element,"圆环三");

    //"PID"子节点
    Menu_Folder* straight_Angle = Create_Menu_Folder(straight_pid,"直角度环");    //直道
    Menu_Folder* straight_Photo = Create_Menu_Folder(straight_pid,"直图像环");
    Menu_Folder* safe_Angle = Create_Menu_Folder(safe_pid,"过角度环");            //过渡
    Menu_Folder* safe_Photo = Create_Menu_Folder(safe_pid,"过图像环");
    Menu_Folder* curve_Angle = Create_Menu_Folder(curve_pid,"弯角度环");          //弯道
    Menu_Folder* curve_Photo = Create_Menu_Folder(curve_pid,"弯图像环");
    Menu_Folder* crossing_Angle = Create_Menu_Folder(crossing_pid,"十角度环");    //十字
    Menu_Folder* crossing_Photo = Create_Menu_Folder(crossing_pid,"十图像环");
    Menu_Folder* ring_Angle = Create_Menu_Folder(ring_pid,"圆角度环");            //圆环
    Menu_Folder* ring_Photo = Create_Menu_Folder(ring_pid,"圆图像环");
    Menu_Folder* model_Angle = Create_Menu_Folder(model_pid,"模角度环");          //模型
    Menu_Folder* model_Photo = Create_Menu_Folder(model_pid,"型图像环");
    Menu_Folder* normal_Angle = Create_Menu_Folder(normal_pid,"角速度环");        //正常
    Menu_Folder* normal_Photo = Create_Menu_Folder(normal_pid,"图像环");

    /********************参数********************/
    //“步长”的参数
    Create_Menu_Number(step_ctrl, "整形", &Int_step, int_Box, 0, 50);
    Create_Menu_Number(step_ctrl, "浮点形", &Float_step, float_Box, 0.0f, 10.0f);

    //"控制模式"参数
    Create_Menu_Number(control_mode, "自抗绕开", &ladrc_on, bool_Box, -0.1f, 1.1f);
    Create_Menu_Number(control_mode, "速决策开", &speed_deci_on, bool_Box, -0.1f, 1.1f);
    Create_Menu_Number(control_mode, "参数锁定", &para_lock, bool_Box, -0.1f, 1.1f);

    //“相机参数”的子节点（参数）
    Create_Menu_Number(camera_para, "亮度", &preprocess.brightness, int_Box, -64, 64);
    Create_Menu_Number(camera_para, "对比度", &preprocess.contrast, int_Box, 0, 100);
    Create_Menu_Number(camera_para, "锐度", &preprocess.sharpness, int_Box, 0, 100);
    Create_Menu_Number(camera_para, "饱和度", &preprocess.saturation, int_Box, 0, 100);
    Create_Menu_Number(camera_para, "增益", &preprocess.gain, int_Box, 0, 100);
    Create_Menu_Number(camera_para, "曝光度", &preprocess.exposure, int_Box, 1, 1250);

    //“解算周期”参数
    Create_Menu_Number(count_time, "内环", &preprocess.inner_count, int_Box, 0, 20);
    Create_Menu_Number(count_time, "外环", &preprocess.outer_count, int_Box, 0, 20);

    //"左电机"参数
    Create_Menu_Number(L_motor, "控制器", &preprocess.L_wc, float_Box, 0.0f, 2000.0f);  //wc
    Create_Menu_Number(L_motor, "观测器", &preprocess.L_w0, float_Box, 0.0f, 2000.0f);  //w0
    Create_Menu_Number(L_motor, "补偿系数", &preprocess.L_b, float_Box, 0.0f, 2000.0f); //b
    Create_Menu_Number(L_motor, "克皮", &preprocess.L_Kp, float_Box, 0.0f, 1000.0f); //Kp
    Create_Menu_Number(L_motor, "克埃", &preprocess.L_Ki, float_Box, 0.0f, 1000.0f); //Ki
    Create_Menu_Number(L_motor, "克弟", &preprocess.L_Kd, float_Box, 0.0f, 1000.0f); //Kd
    //“右电机”参数
    Create_Menu_Number(R_motor, "控制器", &preprocess.R_wc, float_Box, 0.0f, 2000.0f);  //wc
    Create_Menu_Number(R_motor, "观测器", &preprocess.R_w0, float_Box, 0.0f, 2000.0f);  //w0
    Create_Menu_Number(R_motor, "补偿系数", &preprocess.R_b, float_Box, 0.0f, 2000.0f); //b
    Create_Menu_Number(R_motor, "克皮", &preprocess.R_Kp, float_Box, 0.0f, 1000.0f); //Kp
    Create_Menu_Number(R_motor, "克埃", &preprocess.R_Ki, float_Box, 0.0f, 1000.0f); //Ki
    Create_Menu_Number(R_motor, "克弟", &preprocess.R_Kd, float_Box, 0.0f, 1000.0f); //Kd

    //元素PID参数
    //"直道角速度环"参数
    Create_Menu_Number(straight_Angle, "克皮", &preprocess.straight_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(straight_Angle, "克埃", &preprocess.straight_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(straight_Angle, "克弟", &preprocess.straight_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“直道图像环”参数
    Create_Menu_Number(straight_Photo, "克皮", &preprocess.straight_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(straight_Photo, "克埃", &preprocess.straight_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(straight_Photo, "克弟", &preprocess.straight_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //"过渡角速度环"参数
    Create_Menu_Number(safe_Angle, "克皮", &preprocess.safe_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(safe_Angle, "克埃", &preprocess.safe_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(safe_Angle, "克弟", &preprocess.safe_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“过渡图像环”参数
    Create_Menu_Number(safe_Photo, "克皮", &preprocess.safe_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(safe_Photo, "克埃", &preprocess.safe_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(safe_Photo, "克弟", &preprocess.safe_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //"弯道角速度环"参数
    Create_Menu_Number(curve_Angle, "克皮", &preprocess.curve_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(curve_Angle, "克埃", &preprocess.curve_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(curve_Angle, "克弟", &preprocess.curve_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“弯道图像环”参数
    Create_Menu_Number(curve_Photo, "克皮", &preprocess.curve_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(curve_Photo, "克埃", &preprocess.curve_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(curve_Photo, "克弟", &preprocess.curve_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //"十字角速度环"参数
    Create_Menu_Number(crossing_Angle, "克皮", &preprocess.crossing_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(crossing_Angle, "克埃", &preprocess.crossing_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(crossing_Angle, "克弟", &preprocess.crossing_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“十字图像环”参数
    Create_Menu_Number(crossing_Photo, "克皮", &preprocess.crossing_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(crossing_Photo, "克埃", &preprocess.crossing_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(crossing_Photo, "克弟", &preprocess.crossing_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //"圆环角速度环"参数
    Create_Menu_Number(ring_Angle, "克皮", &preprocess.ring_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(ring_Angle, "克埃", &preprocess.ring_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(ring_Angle, "克弟", &preprocess.ring_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“圆环图像环”参数
    Create_Menu_Number(ring_Photo, "克皮", &preprocess.ring_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(ring_Photo, "克埃", &preprocess.ring_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(ring_Photo, "克弟", &preprocess.ring_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //"模型角速度环"参数
    Create_Menu_Number(model_Angle, "克皮", &preprocess.model_Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(model_Angle, "克埃", &preprocess.model_Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(model_Angle, "克弟", &preprocess.model_Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“模型图像环”参数
    Create_Menu_Number(model_Photo, "克皮", &preprocess.model_Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(model_Photo, "克埃", &preprocess.model_Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(model_Photo, "克弟", &preprocess.model_Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //“正常控制角速度”参数
    Create_Menu_Number(normal_Angle, "克皮", &preprocess.Ag_Kp, float_Box, 0.0f, 200.0f);  //Kp
    Create_Menu_Number(normal_Angle, "克埃", &preprocess.Ag_Ki, float_Box, 0.0f, 200.0f);  //Ki
    Create_Menu_Number(normal_Angle, "克弟", &preprocess.Ag_Kd, float_Box, 0.0f, 200.0f);  //Kd
    //“正常控制图像环”参数
    Create_Menu_Number(normal_Photo, "克皮", &preprocess.Ph_Kp, float_Box, 0.0f, 200.0f); //Kp
    Create_Menu_Number(normal_Photo, "克埃", &preprocess.Ph_Ki, float_Box, 0.0f, 200.0f); //Ki
    Create_Menu_Number(normal_Photo, "克弟", &preprocess.Ph_Kd, float_Box, 0.0f, 200.0f); //Kd

    //“元素前瞻”的参数
    Create_Menu_Number(preview_ctrl, "直道前瞻", &preprocess.straight_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "过渡前瞻", &preprocess.safe_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "弯道前瞻", &preprocess.curve_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "十字前瞻", &preprocess.crossing_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "圆环前瞻", &preprocess.ring_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "模型前瞻", &preprocess.model_preview, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(preview_ctrl, "圆环外绕", &preprocess.model_circle_outer_preview, float_Box, 0.0f, 50.0f);
    
    //“参数因子”的参数
    Create_Menu_Number(para_factor, "减速度", &preprocess.max_decel_per_sec, float_Box, 0.0f, 3000.0f);
    Create_Menu_Number(para_factor, "加速度", &preprocess.max_accel_per_sec, float_Box, 0.0f, 3000.0f);
    
    // 圆环顺序参数：Type 0=small, 1=medium, 2=large, 3=big
    Create_Menu_Number(circle_folder, "圆环数量", &preprocess.circle_count, int_Box, 0, 15);
    Create_Menu_Number(circle_folder_one, "环一类型", &preprocess.ring1_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_one, "环二类型", &preprocess.ring2_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_one, "环三类型", &preprocess.ring3_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_one, "环四类型", &preprocess.ring4_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_one, "环五类型", &preprocess.ring5_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_one, "环六类型", &preprocess.ring6_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环七类型", &preprocess.ring7_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环八类型", &preprocess.ring8_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环九类型", &preprocess.ring9_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环十类型", &preprocess.ring10_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环十一类型", &preprocess.ring11_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环十二类型", &preprocess.ring12_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环十三类型", &preprocess.ring13_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_two, "环十四类型", &preprocess.ring14_type, int_Box, 0, 3);
    Create_Menu_Number(circle_folder_three, "环十五类型", &preprocess.ring15_type, int_Box, 0, 3);

    //"出发咯"参数
    Create_Menu_Number(Gogogo, "零漂校准", &is_angle, bool_Box, -0.1f, 1.1f);
    Create_Menu_Number(Gogogo, "出发", &Gogo, bool_Box, -0.1f, 1.1f);
    
    //"速度决策"参数
    Create_Menu_Number(speed_deci, "正常", &preprocess.normal_speed, int_Box, 0, 2000);
    Create_Menu_Number(speed_deci, "过渡", &preprocess.safe_speed, int_Box, 0, 2000);
    Create_Menu_Number(speed_deci, "弯道", &preprocess.curve_speed, int_Box, 0, 2000);
    Create_Menu_Number(speed_deci, "十字", &preprocess.cross_speed, int_Box, 0, 2000);
    Create_Menu_Number(speed_deci, "圆环", &preprocess.ring_speed, int_Box, 0, 2000);
    Create_Menu_Number(speed_deci, "坡道", &preprocess.ramp_speed, int_Box, 0, 2000);
    //"模型"参数
    Create_Menu_Number(model_state, "克皮步长", &preprocess.model_kp_step, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(model_state, "侧视步长", &preprocess.model_side_kp_step, float_Box, 0.0f, 50.0f);
    Create_Menu_Number(model_state, "模型基速", &preprocess.model_speed, int_Box, 0, 2000);
    Create_Menu_Number(model_state, "模型识别", &preprocess.model_recognizing_speed, int_Box, 0, 2000);
    Create_Menu_Number(model_state, "侧视模型", &preprocess.model_side_speed, int_Box, 0, 2000);
    Create_Menu_Number(model_state, "模型绕行", &preprocess.model_pass_speed, int_Box, 0, 2000);
    Create_Menu_Number(model_state, "模型回线", &preprocess.model_return_speed, int_Box, 0, 2000);
    Create_Menu_Number(model_state, "圆环回线", &preprocess.model_circle_return_distance, float_Box, 0.0f, 50.0f);
    // Create_Menu_Number(model_state, "模型预警", &preprocess.model_warning_speed, int_Box, 0, 2000);
    // Create_Menu_Number(model_state, "模型急识", &preprocess.model_emergency_speed, int_Box, 0, 2000);
    // Create_Menu_Number(model_state, "红框方法", &preprocess.model_marker_method, int_Box, 0, 2);
    // Create_Menu_Number(model_state, "计时开关", &preprocess.model_profile_enable, int_Box, 0, 1);
    // Create_Menu_Number(model_state, "计时间隔", &preprocess.model_profile_interval, int_Box, 1, 300);
    // Create_Menu_Number(model_state, "模型保底", &preprocess.model_fail_safe_speed, int_Box, 0, 2000);
    

    //初始化光标位置
    key = head.first_son;

}

//显示菜单表格
void Excel_show(void)
{
    // lcd.lcd_init();
    cv::Mat hline = cv::Mat::zeros(1, 121, CV_8UC1); // 避免传递临时MatExpr
    cv::Mat vline = cv::Mat::zeros(160, 1, CV_8UC1); // 避免传递临时MatExpr
    //横线
    for(int i = 0; i < 20; i ++)
    {
        lcd.showCVImage(2, i*18, hline, 121, 1);
    }
    //竖线
    for(int j = 0; j < 2; j++)
    {
        lcd.showCVImage(j*120 + 2, 18, vline, 1, 160);
    }
    lcd.showCVImage(67 + 2, 18, vline, 1, 160);
    menu_show_Chinese(4,2,"菜单");
    menu_show_int(38,2,degree,2);   //显示当前菜单级数
     
}
//菜单表格内容的x轴起点为4，y轴起点为20，y间隔18。相当规整
//参数的x轴起点为72

//显示光标位置
void show_key(void)
{
   Menu_Folder *h = key->father;
   Menu_Folder *s = h->first_son;

   for(int i = 0; i < h->sons; i ++)    //遍历所有子节点
   {
        if(s == key)
        {
            menu_show_key(4,20 + i*18,64,16);   //文件夹节点处显示光标
            if(key->select)
            {
                menu_show_key(72,20 + i*18,48,16);  //光标移到参数处
                menu_show_key(4,20 + i*18,64,16);   //让参数所指向的文件夹再次颜色反转，防止光标在参数所在的文件夹处残留
              
            }
            else
            {
                menu_show_keep(4,20 + i*18,64,16);  //文件夹处颜色保持原样
                menu_show_keep(72,20 + i*18,48,16); //参数处颜色保持原样
            }
            if(key->execute)
            {
                menu_show_key(4,20 + i*18,64,16);   //文件夹节点处再次颜色反转，消除光标
            }
            else
            {
                menu_show_keep(4,20 + i*18,64,16);   //文件夹处颜色保持原样
            }
        }
        else
        {
            menu_show_keep(4,20 + i*18,64,16);  //文件夹处颜色保持原样
            menu_show_keep(72,20 + i*18,48,16); //参数处颜色保持原样
        }

        s = s->next_brother;    //指针s在所有子节点内移动，直到跟key指向同意节点时显示光标
   }
}

//显示参数
void show_number(void)
{
    Menu_Folder *h = key->father;
    Menu_Folder *s = h->first_son;

    for(int i = 0; i < h->sons; i++)    //遍历所有子节点
    {
        switch(s->kind)
        {
            case int_Box:
                menu_show_int(72,20 + 18*i,*(int*)s->data,4);
                break;
            case float_Box:
                if(*(float*)s->data >= 100.0f && *(float*)s->data < 1000.0f)
                menu_show_float(72,20 + 18*i,*(float*)s->data,3,1);
                else if(*(float*)s->data >= 1000.0f)
                menu_show_float(72,20 + 18*i,*(float*)s->data,4,1);
                else
                menu_show_float(72,20 + 18*i,*(float*)s->data,2,1);
                break;
            case bool_Box:
                if(*(bool*)s->data == true)
                {
                    // if(key->name != "控制切换") menu_show_Chinese(72,20 + 18*i,"是");
                    // else menu_show_Chinese(72,20 + 18*i,"自抗扰");
                    menu_show_Chinese(72,20 + 18*i,"是");
                }
                else
                {
                    // if(key->name != "控制切换") menu_show_Chinese(72,20 + 18*i,"否");
                    // else menu_show_Chinese(72,20 + 18*i,"皮埃弟");
                    menu_show_Chinese(72,20 + 18*i,"否");
                }
                break;
            
            default:
                break;

        }

        s = s->next_brother;    //指针s在所有子节点内移动，遍历所有子节点
    }

}

//显示菜单
void show_menu(void)
{
    Menu_Folder *h = key->father;
    Menu_Folder *s = h->first_son;

    if(key->execute != true)        //可执行函数未执行时，才刷新菜单界面
    {
        for(int i = 0; i < h->sons; i ++)
        {
            menu_show_Chinese(4,20 + 18*i,s->name);     //按顺序依次显示该父节点下的子节点
            s = s->next_brother;
        }

        Excel_show();   //显示表格
        show_key();     //显示光标
        show_number();  //显示参数
    }
    
}

//进入菜单
void key_enter(void)
{
    if (key != NULL && key->execute == true && circle_label_active) {
        biaoding_auto.circle_label_push_type(1);
        return;
    }
    if (key != NULL && key->execute == true &&
        model_calib.current_state != CalibState::idle) {
        model_calib.key_enter_handler();
        return;
    }

    if (key != NULL && key->execute == true){   
        if(biaoding_auto.current_state != BiaoDingState::idle){   // 若可执行函数多 可优化
            biaoding_auto.key_enter_handler();       
        }           // 若 IPMAuto 处于非 idle 状态，为执行 IPMAuto 功能,将 key_enter替换为 IPMAuto 的 key_enter_handler
        return;     // 执行完毕 退出
    }

    switch(key->kind)
    {
        case Normal_Folder:
            if(key->sons > 0)   //父节点有子节点时，才能进入该子节点下
            {
                key = key->first_son;   //光标指向该文件夹下的第一个节点
                degree ++;              //进入菜单，菜单级数+1
                menu_show_clear();      //清屏
                show_menu();            //进入菜单后，显示菜单
                show_key();             //光标显示包含在显示菜单的函数内，再次显示光标，防止光标因为二次显示而丢失
                menu_show_int(38,0,degree,2);   //显示当前菜单级数
            }
            break;

        default :
            key_plus();     //参数加
            break;
    }
    
    
}

//退出菜单
void key_quit(void)
{
    if (key != NULL && key->execute == true && circle_label_active) {
        biaoding_auto.finish_circle_label_menu();
        return;
    }
    if (key != NULL && key->execute == true &&
        model_calib.current_state != CalibState::idle) {
        model_calib.key_quit_handler();
        return;
    }

    if (key != NULL && key->execute == true){   
        if(biaoding_auto.current_state != BiaoDingState::idle){   // 若可执行函数多 可优化
            biaoding_auto.key_quit_handler();       
        }
        else key_exit();        // 防止在执行可执行函数时无法正常退出到菜单界面

        return;     
    }

    if(key->select != true && key->execute != true)     //参数未选中且文件夹未执行时，才能退出
    {
        if(key->father->father != NULL)
        {
            key = key->father;  //光标指向该节点的父节点
            degree --;          //退出菜单，菜单级数-1
            menu_show_clear();  //清屏
            Excel_show();       //显示表格
            menu_show_int(38,0,degree,2);   //显示当前菜单级数
        }

    }
    else
    {
        key_sub();      //参数减
    }
}

// 应用退出接口
// 任何无阻塞可执行程序执行完毕后 调用此函数可把控制权交还给菜单 安全退出
void key_exit(void){
    if (key != NULL && key->execute == true){
        key->execute = false;       // 清除执行标志位
        menu_show_clear();          // 清屏后准备回到菜单
        show_menu();                // 回到菜单界面
        show_key();                 // 显示光标，防止光标丢失
    }
}

//参数选择
void key_select(void)
{   
    if (key != NULL && key->execute == true && circle_label_active) {
        biaoding_auto.finish_circle_label_menu();
        return;
    }
    if (key != NULL && key->execute == true &&
        model_calib.current_state != CalibState::idle) {
        model_calib.key_select_handler();
        return;
    }

    if (key != NULL && key->execute == true)        // 若当前节点已被执行，则不允许再次选择 
        return;
    
    switch(key->kind)       //选中区域为文件夹，则进入该文件夹；若是参数，则选中
    {
        case Normal_Folder:
            if(key->first_son != NULL)      //该节点有子节点，则进入菜单
            {
                key_enter();
            }
		    break; 
        case Execute_Folder:
            if(key->function != nullptr)    //绑定的可执行函数不能为空
            {
                key->execute = !key->execute;   //翻转是否执行的状态
                menu_show_clear();              //清屏后准备执行
                if(key->execute)
                {
                    key->function();        //执行绑定的可执行函数
                }
                else
                {
                    menu_show_clear();  //未选中，直接清屏然后回到菜单
                    show_menu();        
                    show_key();         //未加该函数，测试后光标会丢失，加一个防止光标丢失
                }
            }
        
	    default:
            if(key->kind != Normal_Folder && key->kind != bool_Box && key->kind != Execute_Folder)
            {
                show_key();     //消除文件夹框内的光标
                key->select = !key->select;     //翻转参数的选择状态
            }
            break;

    }

}

//参数加
void key_plus(void)
{        
    show_key();     //按键里含有show_menu,show_menu里有show_key。在这里放show_key防止按键按下时光标消失
    switch(key->kind)       //根据参数类型判断，默认：若是整数，加1；若是浮点数，加0.1
    {
        case int_Box:
            if(key->name != "整形") *(int*)key->data += Int_step;   //选中参数不为“整形步长”时，加步长
            else  *(int*)key->data += 1;
            if (key->min_val != key->max_val && *(int*)key->data > (int)key->max_val) *(int*)key->data = (int)key->max_val;
            break;
        case float_Box:
            if(key->name != "浮点形") *(float*)key->data += Float_step; //选中参数不为“浮点形步长”时，加步长
            else *(float*)key->data += 0.1f;
            if (key->min_val != key->max_val && *(float*)key->data > key->max_val) *(float*)key->data = key->max_val;
            break;
        case bool_Box:
            *(bool*)key->data = ! *(bool*)key->data;
            break;
        default:

            break;
    }
}

//参数减
void key_sub(void)
{
    show_key();     //按键里含有show_menu,show_menu里有show_key。在这里放show_key防止按键按下时光标消失
    switch(key->kind)       //根据参数类型判断，默认：若是整数，减1；若是浮点数，减0.1
    {
        case int_Box:
            if(key->name != "整形") *(int*)key->data -= Int_step;   //选中参数不为“整形步长”时，减步长
            else *(int*)key->data -= 1;
            if (key->min_val != key->max_val && *(int*)key->data < (int)key->min_val) *(int*)key->data = (int)key->min_val;
            break;
        case float_Box:
            if(key->name != "浮点形") *(float*)key->data -= Float_step; //选中参数不为“浮点形步长”时，减步长
            else *(float*)key->data -= 0.1f;
            if (key->min_val != key->max_val && *(float*)key->data < key->min_val) *(float*)key->data = key->min_val;
            break;
        case bool_Box:
            *(bool*)key->data = ! *(bool*)key->data;
            break;
        default:

            break;
    }
}

//按键向上
void key_up(void)
{
    if (key != NULL && key->execute == true && circle_label_active) {
        biaoding_auto.circle_label_push_type(0);
        return;
    }
    if (key != NULL && key->execute == true &&
        model_calib.current_state != CalibState::idle) {
        model_calib.key_up_handler();
        return;
    }

    if(key->select == false && key->execute == false)        //参数未选中且文件夹未执行时候，才能向上选择
    {
        if(key->last_brother != NULL)
        {
            show_key();                 //刷新光标，防止光标残留
            key = key->last_brother;    //光标指向上一个节点
        }
        else
        {
            show_key();                     //刷新光标，防止光标残留
            key = key->father->first_son;   //在第一个子节点时向上，光标保持在第一个子节点
        }
    }
    else
    {
        key_plus();     //选中参数时，该向上功能为参数+
    }
    
    
}

//按键向下
void key_down(void)
{
    if (key != NULL && key->execute == true && circle_label_active) {
        biaoding_auto.circle_label_push_type(2);
        return;
    }
    if (key != NULL && key->execute == true &&
        model_calib.current_state != CalibState::idle) {
        model_calib.key_down_handler();
        return;
    }

    if(key->select == false && key->execute == false)        //参数未选中且文件夹未执行时候，才能向上选择
    {
        if(key->next_brother != NULL)
        {
            show_key();                 //刷新光标，防止光标残留
            key = key->next_brother;    //光标指向下一个节点
        }
        else
        {
            show_key();                     //刷新光标，防止光标残留
            key = key->father->first_son;   //最后一个子节点时向下，光标回到第一个子节点
        }
        
    }
    else
    {
        key_sub();      //选中参数时，该向上功能为参数-
    }
    
    
}

//显示测试
void test_show(void)
{
    menu_show_clear();
    menu_show_Chinese(4,20,"你好世界");
    buzzer.buzzer_init();
    // 使用异步音乐播放器播放《晴天》，避免阻塞/占用额外 TimerThread
    music_player.play(qing_tian, sizeof(qing_tian) / sizeof(Note));
}


/***************************坟场***************************/

//----首页----//
// Menu_Folder* replay = Create_Menu_Folder(&head,"回放");

//----“透视变换”的子节点----//
// Menu_Folder* screen_capture = Create_Menu_Execute_Folder(inverse_perspective,"截图",NULL);
// Menu_Folder* perspective_transformation = Create_Menu_Execute_Folder(inverse_perspective,"透视变换",NULL);
// Menu_Folder* image_stretch = Create_Menu_Execute_Folder(inverse_perspective,"图像拉伸",NULL);

//----“弯道强度”的子节点----//
// Menu_Folder* hundred = Create_Menu_Folder(curve_ctrl,"一百");
// Menu_Folder* hundred_one = Create_Menu_Folder(curve_ctrl,"一百一");
// Menu_Folder* hundred_two = Create_Menu_Folder(curve_ctrl,"一百二");
// Menu_Folder* hundred_three = Create_Menu_Folder(curve_ctrl,"一百三");
// Menu_Folder* hundred_four = Create_Menu_Folder(curve_ctrl,"一百四");
// Menu_Folder* hundred_five = Create_Menu_Folder(curve_ctrl,"一百五");
// Menu_Folder* hundred_six = Create_Menu_Folder(curve_ctrl,"一百六");

//----弯道强度参数----//
//“一百”参数
// Create_Menu_Number(hundred, "克皮", &preprocess.curve_100_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred, "克弟", &preprocess.curve_100_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百一”参数
// Create_Menu_Number(hundred_one, "克皮", &preprocess.curve_110_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_one, "克弟", &preprocess.curve_110_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百二”参数
// Create_Menu_Number(hundred_two, "克皮", &preprocess.curve_120_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_two, "克弟", &preprocess.curve_120_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百三”参数
// Create_Menu_Number(hundred_three, "克皮", &preprocess.curve_130_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_three, "克弟", &preprocess.curve_130_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百四”参数
// Create_Menu_Number(hundred_four, "克皮", &preprocess.curve_140_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_four, "克弟", &preprocess.curve_140_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百五”参数
// Create_Menu_Number(hundred_five, "克皮", &preprocess.curve_150_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_five, "克弟", &preprocess.curve_150_Kd, float_Box, 0.0f, 200.0f); //Kd
// //“一百六”参数
// Create_Menu_Number(hundred_six, "克皮", &preprocess.curve_160_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(hundred_six, "克弟", &preprocess.curve_160_Kd, float_Box, 0.0f, 200.0f); //Kd

//----PD+前馈----//
//直道
// Create_Menu_Number(straight_Angle, "克皮", &preprocess.straight_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(straight_Angle, "克弟", &preprocess.straight_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(straight_Angle, "前馈", &preprocess.straight_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(straight_Angle, "前馈变化", &preprocess.straight_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//过渡
// Create_Menu_Number(safe_Angle, "克皮", &preprocess.safe_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(safe_Angle, "克弟", &preprocess.safe_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(safe_Angle, "前馈", &preprocess.safe_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(safe_Angle, "前馈变化", &preprocess.safe_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//弯道
// Create_Menu_Number(curve_Angle, "克皮", &preprocess.curve_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(curve_Angle, "克弟", &preprocess.curve_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(curve_Angle, "前馈", &preprocess.curve_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(curve_Angle, "前馈变化", &preprocess.curve_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//十字
// Create_Menu_Number(crossing_Angle, "克皮", &preprocess.crossing_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(crossing_Angle, "克弟", &preprocess.crossing_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(crossing_Angle, "前馈", &preprocess.crossing_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(crossing_Angle, "前馈变化", &preprocess.crossing_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//圆环
// Create_Menu_Number(ring_Angle, "克皮", &preprocess.ring_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(ring_Angle, "克弟", &preprocess.ring_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(ring_Angle, "前馈", &preprocess.ring_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(ring_Angle, "前馈变化", &preprocess.ring_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//模型
// Create_Menu_Number(ring_Angle, "克皮", &preprocess.ring_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(ring_Angle, "克弟", &preprocess.ring_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(ring_Angle, "前馈", &preprocess.ring_F_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(ring_Angle, "前馈变化", &preprocess.ring_F_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//正常
// Create_Menu_Number(normal_Angle, "克皮", &preprocess.Ag_F_Kp, float_Box, 0.0f, 200.0f);  //Kp
// Create_Menu_Number(normal_Angle, "克弟", &preprocess.Ag_F_Kd, float_Box, 0.0f, 200.0f);  //Kd
// Create_Menu_Number(normal_Angle, "前馈", &preprocess.Ag_Kff, float_Box, 0.0f, 200.0f); //Kff
// Create_Menu_Number(normal_Angle, "前馈变化", &preprocess.Ag_Kff_acc, float_Box, 0.0f, 200.0f); //Kff_acc

//临时绕行环
// Create_Menu_Number(Temp_Circle, "克皮", &preprocess.Te_Kp, float_Box, 0.0f, 200.0f); //Kp
// Create_Menu_Number(Temp_Circle, "克埃", &preprocess.Te_Ki, float_Box, 0.0f, 200.0f); //Ki
// Create_Menu_Number(Temp_Circle, "克弟", &preprocess.Te_Kd, float_Box, 0.0f, 200.0f); //Kd

//----参数因子----//
//Create_Menu_Number(para_factor, "前瞻因子", &preprocess.preview, float_Box, 0.0f, 50.0f);
//Create_Menu_Number(para_factor, "修速系数", &preprocess.fix_speed, float_Box, 0.0f, 100.0f);
//Create_Menu_Number(para_factor, "角度系数", &preprocess.yaw_distance, float_Box, 0.0f, 100.0f);

//----速度决策----//
// Create_Menu_Number(speed_deci, "障碍", &preprocess.barrier_speed, int_Box, 0, 2000);
// Create_Menu_Number(speed_deci, "目标角速", &target_angle, float_Box, 0.0f, 2000.0f);

//----圆环----//
//“圆环”的子节点 
// Menu_Folder* circle_data = Create_Menu_Folder(circle_folder,"圆环数量");
// Menu_Folder* circle_ring1 = Create_Menu_Folder(circle_folder,"圆环一");
// Menu_Folder* circle_ring2 = Create_Menu_Folder(circle_folder,"圆环二");
// Menu_Folder* circle_ring3 = Create_Menu_Folder(circle_folder,"圆环三");
// Menu_Folder* circle_ring4 = Create_Menu_Folder(circle_folder,"圆环四");
// Menu_Folder* circle_ring5 = Create_Menu_Folder(circle_folder,"圆环五");
