#include "preprocess.h"

extern TrackBase track_base;
extern ModelDetector model_detector;

Preprocess::Preprocess()
    : circle_count(0),
      ring1_type(1),
      ring2_type(1),
      ring3_type(1),
      ring4_type(1),
      ring5_type(1),
      ring6_type(1),
      ring7_type(1),
      ring8_type(1),
      ring9_type(1),
      ring10_type(1),
      ring11_type(1),
      ring12_type(1),
      ring13_type(1),
      ring14_type(1),
      ring15_type(1),

      circle_small_exit_yaw(48.0f),
      circle_small_exit_force_yaw(60.0f),
      circle_small_exit_min_yaw(230.0f),
      circle_small_exit_max_yaw(240.0f),
      circle_small_exit_k_start(1.24f),
      circle_small_exit_k_end(1.32f),
    //   circle_small_leave_confirm_frames(8),
      circle_small_leave_min_distance(7.5f),
    //   circle_small_preview_scale(0.65f),
      
      circle_medium_exit_yaw(35.0f),
      circle_medium_exit_force_yaw(42.0f),
      circle_medium_exit_min_yaw(250.0f),
      circle_medium_exit_max_yaw(265.0f),
      circle_medium_exit_k_start(1.32f),
      circle_medium_exit_k_end(1.25f),
    //   circle_medium_leave_confirm_frames(9),
      circle_medium_leave_min_distance(7.5f),
    //   circle_medium_preview_scale(0.60f),
      
      circle_large_exit_yaw(42.0f),
      circle_large_exit_force_yaw(48.0f),
      circle_large_exit_min_yaw(250.0f),
      circle_large_exit_max_yaw(265.0f),
      circle_large_exit_k_start(1.14f),
      circle_large_exit_k_end(1.08f),
    //   circle_large_leave_confirm_frames(10),
      circle_large_leave_min_distance(7.8f),
    //   circle_large_preview_scale(0.55f),

      circle_big_exit_yaw(42.0f),
      circle_big_exit_force_yaw(48.0f),
      circle_big_exit_min_yaw(265.0f),
      circle_big_exit_max_yaw(275.0f),
      circle_big_exit_k_start(1.14f),
      circle_big_exit_k_end(1.08f),
    //   circle_big_leave_confirm_frames(10),
      circle_big_leave_min_distance(7.8f)
    //   circle_big_preview_scale(0.55f)
{}
Preprocess::~Preprocess(){}

// json 参数预处理初始化
int Preprocess::Preprocess_init(void)
{
    std::ifstream file("./preprocess.json");
    if(!file.is_open()) {
        std::cerr << "Error: Failed to Open json file" << std::endl;
        return -1;
    }

    // 解析 JSON 数据
    Json::Value jsonvalue;
    Json::Reader reader;
    if(!reader.parse(file, jsonvalue)) { /* 解析file文件中的数据 -> Value jsonvalue */
        std::cerr << "Error: Failed to parse json file" << std::endl;
        return -1;
    }
    file.close(); // 关闭输入文件

    // 获取 JSON 数据
    /*  * string   str = jsonvalue["str_name"].asString();      获取名字为str_name的字符串
        * 
        * int      age = jsonvalue["age_name"].asInt();         获取名字为age_name的整型
        * 
        * bool     is  = jsonvalue["is_name"].asBool();         获取名字为is_name的Bool值  
        * 
        * Value    val = jsonvalue["val"];                      直接获取名字val json数据类型的值
        * 
        */

    // 相机参数预处理
    if (jsonvalue.isMember("brightness")) brightness = jsonvalue["brightness"].asInt();
    if (jsonvalue.isMember("contrast")) contrast = jsonvalue["contrast"].asInt();
    if (jsonvalue.isMember("sharpness")) sharpness = jsonvalue["sharpness"].asInt();
    if (jsonvalue.isMember("saturation")) saturation = jsonvalue["saturation"].asInt();
    if (jsonvalue.isMember("gain")) gain = jsonvalue["gain"].asInt();
    if (jsonvalue.isMember("exposure")) exposure = jsonvalue["exposure"].asInt();

    std::cout << "相机参数预处理完成" << std::endl;

    if(jsonvalue.isMember("window_size")) window_size = jsonvalue["window_size"].asInt();
    if(jsonvalue.isMember("max_decel_per_sec")) max_decel_per_sec = jsonvalue["max_decel_per_sec"].asFloat();
    if(jsonvalue.isMember("max_accel_per_sec")) max_accel_per_sec = jsonvalue["max_accel_per_sec"].asFloat();

    std::cout << "参数因子预处理完成" << std::endl;
    
    if(jsonvalue.isMember("circle_count")) circle_count = jsonvalue["circle_count"].asInt();
    if(jsonvalue.isMember("ring1_type")) ring1_type = jsonvalue["ring1_type"].asInt();
    if(jsonvalue.isMember("ring2_type")) ring2_type = jsonvalue["ring2_type"].asInt();
    if(jsonvalue.isMember("ring3_type")) ring3_type = jsonvalue["ring3_type"].asInt();
    if(jsonvalue.isMember("ring4_type")) ring4_type = jsonvalue["ring4_type"].asInt();
    if(jsonvalue.isMember("ring5_type")) ring5_type = jsonvalue["ring5_type"].asInt();
    if(jsonvalue.isMember("ring6_type")) ring6_type = jsonvalue["ring6_type"].asInt();
    if(jsonvalue.isMember("ring7_type")) ring7_type = jsonvalue["ring7_type"].asInt();
    if(jsonvalue.isMember("ring8_type")) ring8_type = jsonvalue["ring8_type"].asInt();
    if(jsonvalue.isMember("ring9_type")) ring9_type = jsonvalue["ring9_type"].asInt();
    if(jsonvalue.isMember("ring10_type")) ring10_type = jsonvalue["ring10_type"].asInt();
    if(jsonvalue.isMember("ring11_type")) ring11_type = jsonvalue["ring11_type"].asInt();
    if(jsonvalue.isMember("ring12_type")) ring12_type = jsonvalue["ring12_type"].asInt();
    if(jsonvalue.isMember("ring13_type")) ring13_type = jsonvalue["ring13_type"].asInt();
    if(jsonvalue.isMember("ring14_type")) ring14_type = jsonvalue["ring14_type"].asInt();
    if(jsonvalue.isMember("ring15_type")) ring15_type = jsonvalue["ring15_type"].asInt();
    if(jsonvalue.isMember("circle_small_exit_yaw")) circle_small_exit_yaw = jsonvalue["circle_small_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_small_exit_force_yaw")) circle_small_exit_force_yaw = jsonvalue["circle_small_exit_force_yaw"].asFloat();
    if(jsonvalue.isMember("circle_small_exit_min_yaw")) circle_small_exit_min_yaw = jsonvalue["circle_small_exit_min_yaw"].asFloat();
    if(jsonvalue.isMember("circle_small_exit_max_yaw")) circle_small_exit_max_yaw = jsonvalue["circle_small_exit_max_yaw"].asFloat();
    else if(jsonvalue.isMember("circle_small_inside_to_exit_yaw")) circle_small_exit_max_yaw = jsonvalue["circle_small_inside_to_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_small_exit_k_start")) circle_small_exit_k_start = jsonvalue["circle_small_exit_k_start"].asFloat();
    if(jsonvalue.isMember("circle_small_exit_k_end")) circle_small_exit_k_end = jsonvalue["circle_small_exit_k_end"].asFloat();
    // if(jsonvalue.isMember("circle_small_leave_confirm_frames")) circle_small_leave_confirm_frames = jsonvalue["circle_small_leave_confirm_frames"].asInt();
    if(jsonvalue.isMember("circle_small_leave_min_distance")) circle_small_leave_min_distance = jsonvalue["circle_small_leave_min_distance"].asFloat();
    // if(jsonvalue.isMember("circle_small_preview_scale")) circle_small_preview_scale = jsonvalue["circle_small_preview_scale"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_yaw")) circle_medium_exit_yaw = jsonvalue["circle_medium_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_force_yaw")) circle_medium_exit_force_yaw = jsonvalue["circle_medium_exit_force_yaw"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_min_yaw")) circle_medium_exit_min_yaw = jsonvalue["circle_medium_exit_min_yaw"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_max_yaw")) circle_medium_exit_max_yaw = jsonvalue["circle_medium_exit_max_yaw"].asFloat();
    else if(jsonvalue.isMember("circle_medium_inside_to_exit_yaw")) circle_medium_exit_max_yaw = jsonvalue["circle_medium_inside_to_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_k_start")) circle_medium_exit_k_start = jsonvalue["circle_medium_exit_k_start"].asFloat();
    if(jsonvalue.isMember("circle_medium_exit_k_end")) circle_medium_exit_k_end = jsonvalue["circle_medium_exit_k_end"].asFloat();
    // if(jsonvalue.isMember("circle_medium_leave_confirm_frames")) circle_medium_leave_confirm_frames = jsonvalue["circle_medium_leave_confirm_frames"].asInt();
    if(jsonvalue.isMember("circle_medium_leave_min_distance")) circle_medium_leave_min_distance = jsonvalue["circle_medium_leave_min_distance"].asFloat();
    // if(jsonvalue.isMember("circle_medium_preview_scale")) circle_medium_preview_scale = jsonvalue["circle_medium_preview_scale"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_yaw")) circle_large_exit_yaw = jsonvalue["circle_large_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_force_yaw")) circle_large_exit_force_yaw = jsonvalue["circle_large_exit_force_yaw"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_min_yaw")) circle_large_exit_min_yaw = jsonvalue["circle_large_exit_min_yaw"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_max_yaw")) circle_large_exit_max_yaw = jsonvalue["circle_large_exit_max_yaw"].asFloat();
    else if(jsonvalue.isMember("circle_large_inside_to_exit_yaw")) circle_large_exit_max_yaw = jsonvalue["circle_large_inside_to_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_k_start")) circle_large_exit_k_start = jsonvalue["circle_large_exit_k_start"].asFloat();
    if(jsonvalue.isMember("circle_large_exit_k_end")) circle_large_exit_k_end = jsonvalue["circle_large_exit_k_end"].asFloat();
    // if(jsonvalue.isMember("circle_large_leave_confirm_frames")) circle_large_leave_confirm_frames = jsonvalue["circle_large_leave_confirm_frames"].asInt();
    if(jsonvalue.isMember("circle_large_leave_min_distance")) circle_large_leave_min_distance = jsonvalue["circle_large_leave_min_distance"].asFloat();
    // if(jsonvalue.isMember("circle_large_preview_scale")) circle_large_preview_scale = jsonvalue["circle_large_preview_scale"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_yaw")) circle_big_exit_yaw = jsonvalue["circle_big_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_force_yaw")) circle_big_exit_force_yaw = jsonvalue["circle_big_exit_force_yaw"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_min_yaw")) circle_big_exit_min_yaw = jsonvalue["circle_big_exit_min_yaw"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_max_yaw")) circle_big_exit_max_yaw = jsonvalue["circle_big_exit_max_yaw"].asFloat();
    else if(jsonvalue.isMember("circle_big_inside_to_exit_yaw")) circle_big_exit_max_yaw = jsonvalue["circle_big_inside_to_exit_yaw"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_k_start")) circle_big_exit_k_start = jsonvalue["circle_big_exit_k_start"].asFloat();
    if(jsonvalue.isMember("circle_big_exit_k_end")) circle_big_exit_k_end = jsonvalue["circle_big_exit_k_end"].asFloat();
    // if(jsonvalue.isMember("circle_big_leave_confirm_frames")) circle_big_leave_confirm_frames = jsonvalue["circle_big_leave_confirm_frames"].asInt();
    if(jsonvalue.isMember("circle_big_leave_min_distance")) circle_big_leave_min_distance = jsonvalue["circle_big_leave_min_distance"].asFloat();
    // if(jsonvalue.isMember("circle_big_preview_scale")) circle_big_preview_scale = jsonvalue["circle_big_preview_scale"].asFloat();

    std::cout << "圆环参数预处理完成" << std::endl;

    if(jsonvalue.isMember("L_wc")) L_wc = jsonvalue["L_wc"].asFloat();
    if(jsonvalue.isMember("L_w0")) L_w0 = jsonvalue["L_w0"].asFloat();
    if(jsonvalue.isMember("L_b")) L_b = jsonvalue["L_b"].asFloat();
    if(jsonvalue.isMember("R_wc")) R_wc = jsonvalue["R_wc"].asFloat();
    if(jsonvalue.isMember("R_w0")) R_w0 = jsonvalue["R_w0"].asFloat();
    if(jsonvalue.isMember("R_b")) R_b = jsonvalue["R_b"].asFloat();

    if(jsonvalue.isMember("L_Kp")) L_Kp = jsonvalue["L_Kp"].asFloat();      //左电机
    if(jsonvalue.isMember("L_Ki")) L_Ki = jsonvalue["L_Ki"].asFloat();
    if(jsonvalue.isMember("L_Kd")) L_Kd = jsonvalue["L_Kd"].asFloat();
    if(jsonvalue.isMember("R_Kp")) R_Kp = jsonvalue["R_Kp"].asFloat();      //右电机
    if(jsonvalue.isMember("R_Ki")) R_Ki = jsonvalue["R_Ki"].asFloat();
    if(jsonvalue.isMember("R_Kd")) R_Kd = jsonvalue["R_Kd"].asFloat();

    std::cout << "电机参数预处理完成" << std::endl;

    if(jsonvalue.isMember("Ag_Kp")) Ag_Kp = jsonvalue["Ag_Kp"].asFloat();   //角速度
    if(jsonvalue.isMember("Ag_Ki")) Ag_Ki = jsonvalue["Ag_Ki"].asFloat();
    if(jsonvalue.isMember("Ag_Kd")) Ag_Kd = jsonvalue["Ag_Kd"].asFloat();
    if(jsonvalue.isMember("Ph_Kp")) Ph_Kp = jsonvalue["Ph_Kp"].asFloat();   //图像
    if(jsonvalue.isMember("Ph_Ki")) Ph_Ki = jsonvalue["Ph_Ki"].asFloat();
    if(jsonvalue.isMember("Ph_Kd")) Ph_Kd = jsonvalue["Ph_Kd"].asFloat();

    if(jsonvalue.isMember("inner_count")) inner_count = jsonvalue["inner_count"].asInt();
    if(jsonvalue.isMember("outer_count")) outer_count = jsonvalue["outer_count"].asInt();

    std::cout << "PID参数预处理完成" << std::endl;

    if(jsonvalue.isMember("speed_deci_on")) speed_deci_on = jsonvalue["speed_deci_on"].asBool();        //是否速度决策

    if(jsonvalue.isMember("straight_Ag_Kp")) straight_Ag_Kp = jsonvalue["straight_Ag_Kp"].asFloat();    //直道角速度
    if(jsonvalue.isMember("straight_Ag_Ki")) straight_Ag_Ki = jsonvalue["straight_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("straight_Ag_Kd")) straight_Ag_Kd = jsonvalue["straight_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("straight_Ph_Kp")) straight_Ph_Kp = jsonvalue["straight_Ph_Kp"].asFloat();    //直道图像 
    if(jsonvalue.isMember("straight_Ph_Ki")) straight_Ph_Ki = jsonvalue["straight_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("straight_Ph_Kd")) straight_Ph_Kd = jsonvalue["straight_Ph_Kd"].asFloat();

    if(jsonvalue.isMember("safe_Ag_Kp")) safe_Ag_Kp = jsonvalue["safe_Ag_Kp"].asFloat();             //安全过渡角速度
    if(jsonvalue.isMember("safe_Ag_Ki")) safe_Ag_Ki = jsonvalue["safe_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("safe_Ag_Kd")) safe_Ag_Kd = jsonvalue["safe_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("safe_Ph_Kp")) safe_Ph_Kp = jsonvalue["safe_Ph_Kp"].asFloat();             //安全过渡图像 
    if(jsonvalue.isMember("safe_Ph_Ki")) safe_Ph_Ki = jsonvalue["safe_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("safe_Ph_Kd")) safe_Ph_Kd = jsonvalue["safe_Ph_Kd"].asFloat();

    if(jsonvalue.isMember("curve_Ag_Kp")) curve_Ag_Kp = jsonvalue["curve_Ag_Kp"].asFloat();             //弯道角速度
    if(jsonvalue.isMember("curve_Ag_Ki")) curve_Ag_Ki = jsonvalue["curve_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("curve_Ag_Kd")) curve_Ag_Kd = jsonvalue["curve_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("curve_Ph_Kp")) curve_Ph_Kp = jsonvalue["curve_Ph_Kp"].asFloat();             //弯道图像 
    if(jsonvalue.isMember("curve_Ph_Ki")) curve_Ph_Ki = jsonvalue["curve_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("curve_Ph_Kd")) curve_Ph_Kd = jsonvalue["curve_Ph_Kd"].asFloat();

    if(jsonvalue.isMember("crossing_Ag_Kp")) crossing_Ag_Kp = jsonvalue["crossing_Ag_Kp"].asFloat();    //十字角速度
    if(jsonvalue.isMember("crossing_Ag_Ki")) crossing_Ag_Ki = jsonvalue["crossing_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("crossing_Ag_Kd")) crossing_Ag_Kd = jsonvalue["crossing_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("crossing_Ph_Kp")) crossing_Ph_Kp = jsonvalue["crossing_Ph_Kp"].asFloat();    //十字图像 
    if(jsonvalue.isMember("crossing_Ph_Ki")) crossing_Ph_Ki = jsonvalue["crossing_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("crossing_Ph_Kd")) crossing_Ph_Kd = jsonvalue["crossing_Ph_Kd"].asFloat();

    if(jsonvalue.isMember("ring_Ag_Kp")) ring_Ag_Kp = jsonvalue["ring_Ag_Kp"].asFloat();                //圆环角速度
    if(jsonvalue.isMember("ring_Ag_Ki")) ring_Ag_Ki = jsonvalue["ring_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("ring_Ag_Kd")) ring_Ag_Kd = jsonvalue["ring_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("ring_Ph_Kp")) ring_Ph_Kp = jsonvalue["ring_Ph_Kp"].asFloat();                //圆环图像 
    if(jsonvalue.isMember("ring_Ph_Ki")) ring_Ph_Ki = jsonvalue["ring_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("ring_Ph_Kd")) ring_Ph_Kd = jsonvalue["ring_Ph_Kd"].asFloat();

    if(jsonvalue.isMember("model_Ag_Kp")) model_Ag_Kp = jsonvalue["model_Ag_Kp"].asFloat();             //模型角速度
    if(jsonvalue.isMember("model_Ag_Ki")) model_Ag_Ki = jsonvalue["model_Ag_Ki"].asFloat();
    if(jsonvalue.isMember("model_Ag_Kd")) model_Ag_Kd = jsonvalue["model_Ag_Kd"].asFloat();
    if(jsonvalue.isMember("model_Ph_Kp")) model_Ph_Kp = jsonvalue["model_Ph_Kp"].asFloat();             //模型图像 
    if(jsonvalue.isMember("model_Ph_Ki")) model_Ph_Ki = jsonvalue["model_Ph_Ki"].asFloat();
    if(jsonvalue.isMember("model_Ph_Kd")) model_Ph_Kd = jsonvalue["model_Ph_Kd"].asFloat();

    std::cout << "元素控制参数预处理完成" << std::endl; 

    if(jsonvalue.isMember("straight_preview")) straight_preview = jsonvalue["straight_preview"].asFloat();  //直道前瞻
    if(jsonvalue.isMember("safe_preview")) safe_preview = jsonvalue["safe_preview"].asFloat();              //安全过渡前瞻
    if(jsonvalue.isMember("curve_preview")) curve_preview = jsonvalue["curve_preview"].asFloat();           //弯道前瞻
    if(jsonvalue.isMember("crossing_preview")) crossing_preview = jsonvalue["crossing_preview"].asFloat();  //十字前瞻
    if(jsonvalue.isMember("ring_preview")) ring_preview = jsonvalue["ring_preview"].asFloat();              //圆环前瞻
    if(jsonvalue.isMember("model_preview")) model_preview = jsonvalue["model_preview"].asFloat();           //模型前瞻

    // 圆环外绕补丁
    if(jsonvalue.isMember("model_circle_outer_preview")) model_circle_outer_preview = jsonvalue["model_circle_outer_preview"].asFloat();
    else model_circle_outer_preview = ring_preview;

    std::cout << "元素前瞻参数预处理完成" << std::endl; 

    if(jsonvalue.isMember("normal_speed")) normal_speed = jsonvalue["normal_speed"].asInt();
    if(jsonvalue.isMember("safe_speed")) safe_speed = jsonvalue["safe_speed"].asInt();
    if(jsonvalue.isMember("curve_speed")) curve_speed = jsonvalue["curve_speed"].asInt();
    if(jsonvalue.isMember("cross_speed")) cross_speed = jsonvalue["cross_speed"].asInt();
    if(jsonvalue.isMember("ring_speed")) ring_speed = jsonvalue["ring_speed"].asInt();
    if(jsonvalue.isMember("model_speed")) model_speed = jsonvalue["model_speed"].asInt();

    if(jsonvalue.isMember("model_kp_step")) model_kp_step = jsonvalue["model_kp_step"].asFloat();
    if(jsonvalue.isMember("model_side_kp_step")) model_side_kp_step = jsonvalue["model_side_kp_step"].asFloat();
    if(jsonvalue.isMember("model_warning_speed")) model_warning_speed = jsonvalue["model_warning_speed"].asInt();
    else model_warning_speed = model_speed;
    if(jsonvalue.isMember("model_recognizing_speed")) model_recognizing_speed = jsonvalue["model_recognizing_speed"].asInt();
    else model_recognizing_speed = model_speed;
    if(jsonvalue.isMember("model_side_speed")) model_side_speed = jsonvalue["model_side_speed"].asInt();
    else model_side_speed = model_recognizing_speed;
    if(jsonvalue.isMember("model_emergency_speed")) model_emergency_speed = jsonvalue["model_emergency_speed"].asInt();
    else model_emergency_speed = model_recognizing_speed;
    if(jsonvalue.isMember("model_pass_speed")) model_pass_speed = jsonvalue["model_pass_speed"].asInt();
    else model_pass_speed = model_speed;
    if(jsonvalue.isMember("model_return_speed")) model_return_speed = jsonvalue["model_return_speed"].asInt();
    else model_return_speed = safe_speed;
    if(jsonvalue.isMember("model_circle_return_distance")) model_circle_return_distance = jsonvalue["model_circle_return_distance"].asFloat();
    else model_circle_return_distance = 8.0f;
    if(jsonvalue.isMember("model_fail_safe_speed")) model_fail_safe_speed = jsonvalue["model_fail_safe_speed"].asInt();
    else model_fail_safe_speed = model_emergency_speed;
    if(jsonvalue.isMember("model_marker_method")) model_marker_method = jsonvalue["model_marker_method"].asInt();
    else model_marker_method = 2;
    if(jsonvalue.isMember("model_profile_enable")) model_profile_enable = jsonvalue["model_profile_enable"].asInt();
    else model_profile_enable = 0;
    if(jsonvalue.isMember("model_profile_interval")) model_profile_interval = jsonvalue["model_profile_interval"].asInt();
    else model_profile_interval = 30;
    if(model_profile_interval <= 0) model_profile_interval = 30;
    if(jsonvalue.isMember("ramp_speed")) ramp_speed = jsonvalue["ramp_speed"].asInt();
    if(jsonvalue.isMember("barrier_speed")) barrier_speed = jsonvalue["barrier_speed"].asInt();

    std::cout << "速度决策参数预处理完成" << std::endl;


    return 0;
}

// 保存参数到 json 文件
int Preprocess::Preprocess_save() {
    // 创建一个新的 Json::Value 对象并填充当前值
    Json::Value jsonvalue;
    
    // 填充所有参数到 jsonvalue
    //相机参数
    jsonvalue["brightness"] = brightness;
    jsonvalue["contrast"] = contrast;
    jsonvalue["sharpness"] = sharpness;
    jsonvalue["saturation"] = saturation;
    jsonvalue["gain"] = gain;
    jsonvalue["exposure"] = exposure;

    //参数因子
    jsonvalue["window_size"] = window_size;
    jsonvalue["max_decel_per_sec"] = max_decel_per_sec;
    jsonvalue["max_accel_per_sec"] = max_accel_per_sec;
    jsonvalue["fix_speed"] = fix_speed;
    jsonvalue["yaw_distance"] = yaw_distance;

    //圆环参数
    jsonvalue["circle_count"] = circle_count;
    jsonvalue["ring1_type"] = ring1_type;
    jsonvalue["ring2_type"] = ring2_type;
    jsonvalue["ring3_type"] = ring3_type;
    jsonvalue["ring4_type"] = ring4_type;
    jsonvalue["ring5_type"] = ring5_type;
    jsonvalue["ring6_type"] = ring6_type;
    jsonvalue["ring7_type"] = ring7_type;
    jsonvalue["ring8_type"] = ring8_type;
    jsonvalue["ring9_type"] = ring9_type;
    jsonvalue["ring10_type"] = ring10_type;
    jsonvalue["ring11_type"] = ring11_type;
    jsonvalue["ring12_type"] = ring12_type;
    jsonvalue["ring13_type"] = ring13_type;
    jsonvalue["ring14_type"] = ring14_type;
    jsonvalue["ring15_type"] = ring15_type;
    jsonvalue["circle_small_exit_yaw"] = circle_small_exit_yaw;
    jsonvalue["circle_small_exit_force_yaw"] = circle_small_exit_force_yaw;
    jsonvalue["circle_small_exit_min_yaw"] = circle_small_exit_min_yaw;
    jsonvalue["circle_small_exit_max_yaw"] = circle_small_exit_max_yaw;
    jsonvalue["circle_small_exit_k_start"] = circle_small_exit_k_start;
    jsonvalue["circle_small_exit_k_end"] = circle_small_exit_k_end;
    // jsonvalue["circle_small_leave_confirm_frames"] = circle_small_leave_confirm_frames;
    jsonvalue["circle_small_leave_min_distance"] = circle_small_leave_min_distance;
    // jsonvalue["circle_small_preview_scale"] = circle_small_preview_scale;
    jsonvalue["circle_medium_exit_yaw"] = circle_medium_exit_yaw;
    jsonvalue["circle_medium_exit_force_yaw"] = circle_medium_exit_force_yaw;
    jsonvalue["circle_medium_exit_min_yaw"] = circle_medium_exit_min_yaw;
    jsonvalue["circle_medium_exit_max_yaw"] = circle_medium_exit_max_yaw;
    jsonvalue["circle_medium_exit_k_start"] = circle_medium_exit_k_start;
    jsonvalue["circle_medium_exit_k_end"] = circle_medium_exit_k_end;
    // jsonvalue["circle_medium_leave_confirm_frames"] = circle_medium_leave_confirm_frames;
    jsonvalue["circle_medium_leave_min_distance"] = circle_medium_leave_min_distance;
    // jsonvalue["circle_medium_preview_scale"] = circle_medium_preview_scale;
    jsonvalue["circle_large_exit_yaw"] = circle_large_exit_yaw;
    jsonvalue["circle_large_exit_force_yaw"] = circle_large_exit_force_yaw;
    jsonvalue["circle_large_exit_min_yaw"] = circle_large_exit_min_yaw;
    jsonvalue["circle_large_exit_max_yaw"] = circle_large_exit_max_yaw;
    jsonvalue["circle_large_exit_k_start"] = circle_large_exit_k_start;
    jsonvalue["circle_large_exit_k_end"] = circle_large_exit_k_end;
    // jsonvalue["circle_large_leave_confirm_frames"] = circle_large_leave_confirm_frames;
    jsonvalue["circle_large_leave_min_distance"] = circle_large_leave_min_distance;
    // jsonvalue["circle_large_preview_scale"] = circle_large_preview_scale;
    jsonvalue["circle_big_exit_yaw"] = circle_big_exit_yaw;
    jsonvalue["circle_big_exit_force_yaw"] = circle_big_exit_force_yaw;
    jsonvalue["circle_big_exit_min_yaw"] = circle_big_exit_min_yaw;
    jsonvalue["circle_big_exit_max_yaw"] = circle_big_exit_max_yaw;
    jsonvalue["circle_big_exit_k_start"] = circle_big_exit_k_start;
    jsonvalue["circle_big_exit_k_end"] = circle_big_exit_k_end;
    // jsonvalue["circle_big_leave_confirm_frames"] = circle_big_leave_confirm_frames;
    jsonvalue["circle_big_leave_min_distance"] = circle_big_leave_min_distance;
    // jsonvalue["circle_big_preview_scale"] = circle_big_preview_scale;

    //电机LADRC参数
    jsonvalue["L_wc"] = L_wc;
    jsonvalue["L_w0"] = L_w0;
    jsonvalue["L_b"] = L_b;
    jsonvalue["R_wc"] = R_wc;
    jsonvalue["R_w0"] = R_w0;
    jsonvalue["R_b"] = R_b;
    //电机PID参数
    jsonvalue["L_Kp"] = L_Kp;
    jsonvalue["L_Ki"] = L_Ki;
    jsonvalue["L_Kd"] = L_Kd;
    jsonvalue["R_Kp"] = R_Kp;
    jsonvalue["R_Ki"] = R_Ki;
    jsonvalue["R_Kd"] = R_Kd;

    //PID内外环参数
    jsonvalue["Ag_Kp"] = Ag_Kp;
    jsonvalue["Ag_Ki"] = Ag_Ki;
    jsonvalue["Ag_Kd"] = Ag_Kd;
    jsonvalue["Ph_Kp"] = Ph_Kp;
    jsonvalue["Ph_Ki"] = Ph_Ki;
    jsonvalue["Ph_Kd"] = Ph_Kd;
    jsonvalue["Te_Kp"] = Te_Kp;
    jsonvalue["Te_Ki"] = Te_Ki;
    jsonvalue["Te_Kd"] = Te_Kd;

    //元素前瞻
    jsonvalue["straight_preview"] = straight_preview;
    jsonvalue["safe_preview"] = safe_preview;
    jsonvalue["curve_preview"] = curve_preview;
    jsonvalue["crossing_preview"] = crossing_preview;
    jsonvalue["ring_preview"] = ring_preview;
    jsonvalue["model_preview"] = model_preview;
    jsonvalue["model_circle_outer_preview"] = model_circle_outer_preview;

    //元素PID参数
    jsonvalue["para_lock"] = para_lock;             //是否参数锁定
    jsonvalue["speed_deci_on"] = speed_deci_on;     //是否速度决策
    jsonvalue["ladrc_on"] = ladrc_on;               //是否自抗扰

    jsonvalue["straight_Ag_Kp"] = straight_Ag_Kp;   //直道
    jsonvalue["straight_Ag_Ki"] = straight_Ag_Ki;
    jsonvalue["straight_Ag_Kd"] = straight_Ag_Kd;
    jsonvalue["straight_Ph_Kp"] = straight_Ph_Kp;
    jsonvalue["straight_Ph_Ki"] = straight_Ph_Ki;
    jsonvalue["straight_Ph_Kd"] = straight_Ph_Kd;

    jsonvalue["safe_Ag_Kp"] = safe_Ag_Kp;           //安全过渡
    jsonvalue["safe_Ag_Ki"] = safe_Ag_Ki;
    jsonvalue["safe_Ag_Kd"] = safe_Ag_Kd;
    jsonvalue["safe_Ph_Kp"] = safe_Ph_Kp;
    jsonvalue["safe_Ph_Ki"] = safe_Ph_Ki;
    jsonvalue["safe_Ph_Kd"] = safe_Ph_Kd;

    jsonvalue["curve_Ag_Kp"] = curve_Ag_Kp;         //弯道
    jsonvalue["curve_Ag_Ki"] = curve_Ag_Ki;
    jsonvalue["curve_Ag_Kd"] = curve_Ag_Kd;
    jsonvalue["curve_Ph_Kp"] = curve_Ph_Kp;
    jsonvalue["curve_Ph_Ki"] = curve_Ph_Ki;
    jsonvalue["curve_Ph_Kd"] = curve_Ph_Kd;

    jsonvalue["crossing_Ag_Kp"] = crossing_Ag_Kp;   //十字
    jsonvalue["crossing_Ag_Ki"] = crossing_Ag_Ki;
    jsonvalue["crossing_Ag_Kd"] = crossing_Ag_Kd;
    jsonvalue["crossing_Ph_Kp"] = crossing_Ph_Kp;
    jsonvalue["crossing_Ph_Ki"] = crossing_Ph_Ki;
    jsonvalue["crossing_Ph_Kd"] = crossing_Ph_Kd;

    jsonvalue["ring_Ag_Kp"] = ring_Ag_Kp;           //圆环
    jsonvalue["ring_Ag_Ki"] = ring_Ag_Ki;
    jsonvalue["ring_Ag_Kd"] = ring_Ag_Kd;
    jsonvalue["ring_Ph_Kp"] = ring_Ph_Kp;
    jsonvalue["ring_Ph_Ki"] = ring_Ph_Ki;
    jsonvalue["ring_Ph_Kd"] = ring_Ph_Kd;

    jsonvalue["model_Ag_Kp"] = model_Ag_Kp;         //模型
    jsonvalue["model_Ag_Ki"] = model_Ag_Ki;
    jsonvalue["model_Ag_Kd"] = model_Ag_Kd;
    jsonvalue["model_Ph_Kp"] = model_Ph_Kp;
    jsonvalue["model_Ph_Ki"] = model_Ph_Ki;
    jsonvalue["model_Ph_Kd"] = model_Ph_Kd;

    //PID解算周期
    jsonvalue["inner_count"] = inner_count;
    jsonvalue["outer_count"] = outer_count;

    //速度决策
    jsonvalue["normal_speed"] = normal_speed;
    jsonvalue["safe_speed"] = safe_speed;
    jsonvalue["curve_speed"] = curve_speed;
    jsonvalue["cross_speed"] = cross_speed;
    jsonvalue["ring_speed"] = ring_speed;
    jsonvalue["model_speed"] = model_speed;

    jsonvalue["model_kp_step"] = model_kp_step;
    jsonvalue["model_side_kp_step"] = model_side_kp_step;
    jsonvalue["model_warning_speed"] = model_warning_speed;
    jsonvalue["model_recognizing_speed"] = model_recognizing_speed;
    jsonvalue["model_side_speed"] = model_side_speed;
    jsonvalue["model_emergency_speed"] = model_emergency_speed;
    jsonvalue["model_pass_speed"] = model_pass_speed;
    jsonvalue["model_return_speed"] = model_return_speed;
    jsonvalue["model_circle_return_distance"] = model_circle_return_distance;
    jsonvalue["model_fail_safe_speed"] = model_fail_safe_speed;
    jsonvalue["model_marker_method"] = model_marker_method;
    jsonvalue["model_profile_enable"] = model_profile_enable;
    jsonvalue["model_profile_interval"] = model_profile_interval;
    jsonvalue["ramp_speed"] = ramp_speed;
    jsonvalue["barrier_speed"] = barrier_speed;

    // 打开文件准备写入
    std::ofstream outfile("./preprocess.json");
    if(!outfile.is_open()) {
        std::cerr << "Error: Failed to open file for writing" << std::endl;
        return -1;
    }
    
    // 使用 Json::StyledWriter 生成格式化的 JSON
    Json::StyledWriter writer;
    outfile << writer.write(jsonvalue);
    outfile.close();
    
    // std::cout << "参数已成功保存" << std::endl;
    return 0;
}

// 打包所有用于上位的调试数据并返回 JSON 字符串
std::string Preprocess::pack_debug_data() {
    std::lock_guard<std::mutex> lock(alg_mutex);
    Json::Value root;
    
    // 图传中已用CV库在视频流把密集的像素点画进去了
    // 所以此处只传输右侧状态机的信息,节省带宽和延迟 (如果使用视频看的话会有一些提前 (现实不知))

    // 其他调试数据
    root["track_error"] = track_base.track_error;
    
    // 中线合成模式推导
    std::string mid_mode;
    if (!left_boundary.is_lost && !right_boundary.is_lost) {
        mid_mode = "BOTH";
    } 
    else if (!left_boundary.is_lost && right_boundary.is_lost) {
        mid_mode = "L_HALF";
    } 
    else if (left_boundary.is_lost && !right_boundary.is_lost) {
        mid_mode = "R_HALF";
    } 
    else {
        mid_mode = "PREDICT";
    }
    root["mid_mode"] = mid_mode;

    // 丢线信息
    root["l_lost"] = left_boundary.is_lost;
    root["l_lost_y"] = left_boundary.lost_y;
    root["r_lost"] = right_boundary.is_lost;
    root["r_lost_y"] = right_boundary.lost_y;

    // 拐点信息 (上下角点)
    root["l_crn_down"] = left_boundary.down_lp_state;
    root["l_crn_down_y"] = left_boundary.down_lp_pt.y;
    root["r_crn_down"] = right_boundary.down_lp_state;
    root["r_crn_down_y"] = right_boundary.down_lp_pt.y;
    
    root["l_crn_up"] = left_boundary.up_lp_state;
    root["l_crn_up_y"] = left_boundary.up_lp_pt.y;
    root["r_crn_up"] = right_boundary.up_lp_state;
    root["r_crn_up_y"] = right_boundary.up_lp_pt.y;

    // 中拐点
    root["l_crn_mid"] = left_boundary.mid_lp_state;
    root["l_crn_mid_y"] = left_boundary.mid_lp_pt.y;
    root["r_crn_mid"] = right_boundary.mid_lp_state;
    root["r_crn_mid_y"] = right_boundary.mid_lp_pt.y;

    // // 边线单调性状态（直线段检测）
    // root["l_is_straight"] = left_boundary.is_straight;
    // root["r_is_straight"] = right_boundary.is_straight;
    
    // 边线单调性与方差
    root["l_mnt_len"] = left_boundary.mnt_len;
    root["r_mnt_len"] = right_boundary.mnt_len;
    root["l_dx_var"] = left_boundary.dx_var;
    root["r_dx_var"] = right_boundary.dx_var;

    // // 出环点
    // root["l_crn_out"] = left_boundary.out_lp_state;
    // root["l_crn_out_y"] = left_boundary.out_lp_pt.y;
    // root["r_crn_out"] = right_boundary.out_lp_state;
    // root["r_crn_out_y"] = right_boundary.out_lp_pt.y;

    // 全局状态机与参数打包
    root["track_type"] = static_cast<int>(element_state.current_track_type);
    root["model_line_pass_active"] = element_state.model_line_pass_active;
    root["model_line_pass_dir"] = static_cast<int>(element_state.model_line_pass_dir);

    // 常规道路类型状态（直道/弯道）
    std::string normal_type_st;
    switch (element_state.normal_type) {
        case NormalType::straight:      normal_type_st = "1-STRAIGHT"; break;
        case NormalType::curve:         normal_type_st = "2-CURVE"; break;
        default: normal_type_st = "0-UNKNOWN"; break;
    }
    root["normal_type"] = normal_type_st;
    root["curve_score"] = element_state.curve_score;

    std::string normal_road_state_st;
    switch (element_state.normal_road_state) {
        case NormalRoadState::curve:         normal_road_state_st = "1-CURVE"; break;
        case NormalRoadState::safe:          normal_road_state_st = "2-SAFE"; break;
        case NormalRoadState::straight_fast: normal_road_state_st = "3-STRAIGHT_FAST"; break;
        default: normal_road_state_st = "0-UNKNOWN"; break;
    }
    root["normal_road_state"] = normal_road_state_st;
    root["straight_confirm_count"] = element_state.straight_confirm_count;
    root["curve_confirm_count"] = element_state.curve_confirm_count;

    // 环岛状态
    std::string circle_st;
    switch (element_state.circle_state) {
        case CircleState::weak_approach: circle_st = "1-WEAK_APP"; break;
        case CircleState::approach: circle_st = "2-APPROACH"; break;
        case CircleState::inside:  circle_st = "3-INSIDE"; break;
        case CircleState::exit:    circle_st = "4-EXIT"; break;
        case CircleState::leave:   circle_st = "5-LEAVE"; break;
        default: circle_st = "0-NONE"; break;
    }
    root["circle_state"] = circle_st;
    root["circle_dir"] = static_cast<int>(element_state.circle_dir);
    root["circle_index"] = element_state.circle_run_index + 1;
    root["circle_count"] = circle_count;
    int debug_circle_type = ring1_type;
    switch (element_state.circle_run_index) {
        case 0: debug_circle_type = ring1_type; break;
        case 1: debug_circle_type = ring2_type; break;
        case 2: debug_circle_type = ring3_type; break;
        case 3: debug_circle_type = ring4_type; break;
        case 4: debug_circle_type = ring5_type; break;
        case 5: debug_circle_type = ring6_type; break;
        case 6: debug_circle_type = ring7_type; break;
        case 7: debug_circle_type = ring8_type; break;
        case 8: debug_circle_type = ring9_type; break;
        case 9: debug_circle_type = ring10_type; break;
        case 10: debug_circle_type = ring11_type; break;
        case 11: debug_circle_type = ring12_type; break;
        case 12: debug_circle_type = ring13_type; break;
        case 13: debug_circle_type = ring14_type; break;
        case 14: debug_circle_type = ring15_type; break;
        default: debug_circle_type = ring15_type; break;
    }
    if (debug_circle_type <= 0) root["circle_type"] = "SMALL";
    else if (debug_circle_type == 1) root["circle_type"] = "MEDIUM";
    else if (debug_circle_type == 2) root["circle_type"] = "LARGE";
    else root["circle_type"] = "BIG";
    
    if (debug_circle_type <= 0) root["circle_leave_min_distance"] = circle_small_leave_min_distance;
    else if (debug_circle_type == 1) root["circle_leave_min_distance"] = circle_medium_leave_min_distance;
    else if (debug_circle_type == 2) root["circle_leave_min_distance"] = circle_large_leave_min_distance;
    else root["circle_leave_min_distance"] = circle_big_leave_min_distance;
    
    // 十字路口状态转化
    std::string cross_st;
    switch (element_state.crossroad_state) {
        case CrossroadState::approach: cross_st = "1-APPROACH"; break;
        case CrossroadState::inside: cross_st = "2-INSIDE"; break;
        // case CrossroadState::pass: cross_st = "3-PASS"; break;
        default: cross_st = "0-NORMAL"; break;
    }
    root["cross_state"] = cross_st;
    
    // 路障状态转化
    std::string barrier_st;
    switch (element_state.barrier_state) {
        case BarrierState::detected: barrier_st = "1-DETECTED"; break;
        case BarrierState::avoiding: barrier_st = "2-AVOIDING"; break;
        default: barrier_st = "0-NONE"; break;
    }
    root["barrier_state"] = barrier_st;

    // 坡道状态转化
    std::string ramp_st;
    switch (element_state.ramp_state) {
        case RampState::detected: ramp_st = "1-DETECTED"; break;
        case RampState::climbing: ramp_st = "2-CLIMBING"; break;
        case RampState::ending:   ramp_st = "3-ENDING"; break;
        default: ramp_st = "0-NONE"; break;
    }
    root["ramp_state"] = ramp_st;
    root["ramp_circle_shield_count"] = element_state.ramp_circle_shield_count;
    root["ramp_up_confirm_count"] = element_state.ramp_up_confirm_count;
    root["ramp_down_confirm_count"] = element_state.ramp_down_confirm_count;
    root["ramp_flat_confirm_count"] = element_state.ramp_flat_confirm_count;
    
    root["zebra_state"] = static_cast<int>(element_state.zebra_state);
    root["target_speed"] = car_speed;
    
    // 误差和前瞻
    root["err_total"] = photo_err;
    root["lk_y"] = track_base.debug_lookahead_y;

    // MODEL_STATE 模型状态
    root["model_detected"] = model_detector.last_result.is_detected;
    root["model_class"] = model_detector.last_result.class_name.empty() ? "" : model_detector.last_result.class_name;
    
    // PTS_STATE 点集状态
    root["raw_l_pts_size"] = static_cast<int>(left_boundary.raw_pts.size());
    root["raw_r_pts_size"] = static_cast<int>(right_boundary.raw_pts.size());
    root["seed_state"] = static_cast<int>(track_base.seed_state);

    // SENSOR_FUSION 传感器数据
    root["tof_dist"] = element_state.current_tof_dist;
    root["motor_distance"] = motor_distance;
    root["gyro_pitch"] = pitch_angle;
    root["gyro_yaw"] = yaw_angle;
    // root["gyro_z_int"] = element_state.gyro_z_integration;  // 已屏蔽

    // 出界状态
    root["is_out"] = track_base.is_out_of_bounds;

    // 如果后续有需要新加变量，直接在这里增加即可
    // root["new_variable"] = ...;
    
    Json::FastWriter writer;
    return writer.write(root);
}

Preprocess preprocess;

/********************坟场********************/

// if(jsonvalue.isMember("preview")) preview = jsonvalue["preview"].asFloat();  //前瞻因子
// if(jsonvalue.isMember("fix_speed")) fix_speed = jsonvalue["fix_speed"].asFloat();
// if(jsonvalue.isMember("yaw_distance")) yaw_distance = jsonvalue["yaw_distance"].asFloat();

// if(jsonvalue.isMember("Te_Kp")) Te_Kp = jsonvalue["Te_Kp"].asFloat();   //临时
// if(jsonvalue.isMember("Te_Ki")) Te_Ki = jsonvalue["Te_Ki"].asFloat();
// if(jsonvalue.isMember("Te_Kd")) Te_Kd = jsonvalue["Te_Kd"].asFloat();

// if(jsonvalue.isMember("Ag_F_Kp")) Ag_F_Kp = jsonvalue["Ag_F_Kp"].asFloat();   //正常角速度
// if(jsonvalue.isMember("Ag_F_Kd")) Ag_F_Kd = jsonvalue["Ag_F_Kd"].asFloat();
// if(jsonvalue.isMember("Ag_Kff")) Ag_Kff = jsonvalue["Ag_Kff"].asFloat();
// if(jsonvalue.isMember("Ag_Kff_acc")) Ag_Kff_acc = jsonvalue["Ag_Kff_acc"].asFloat();

// if(jsonvalue.isMember("straight_F_Kp")) straight_F_Kp = jsonvalue["straight_F_Kp"].asFloat();       //直道角速度
// if(jsonvalue.isMember("straight_F_Kd")) straight_F_Kd = jsonvalue["straight_F_Kd"].asFloat();
// if(jsonvalue.isMember("straight_F_Kff")) straight_F_Kff = jsonvalue["straight_F_Kff"].asFloat();
// if(jsonvalue.isMember("straight_F_Kff_acc")) Ag_Kff_acc = jsonvalue["straight_F_Kff_acc"].asFloat();

// if(jsonvalue.isMember("safe_F_Kp")) safe_F_Kp = jsonvalue["safe_F_Kp"].asFloat();                //安全过渡角速度
// if(jsonvalue.isMember("safe_F_Kd")) safe_F_Kd = jsonvalue["safe_F_Kd"].asFloat();
// if(jsonvalue.isMember("safe_F_Kff")) safe_F_Kff = jsonvalue["safe_F_Kff"].asFloat();
// if(jsonvalue.isMember("safe_F_Kff_acc")) safe_F_Kff_acc = jsonvalue["safe_F_Kff_acc"].asFloat();

// if(jsonvalue.isMember("curve_F_Kp")) curve_F_Kp = jsonvalue["curve_F_Kp"].asFloat();                //弯道角速度
// if(jsonvalue.isMember("curve_F_Kd")) curve_F_Kd = jsonvalue["curve_F_Kd"].asFloat();
// if(jsonvalue.isMember("curve_F_Kff")) curve_F_Kff = jsonvalue["curve_F_Kff"].asFloat();
// if(jsonvalue.isMember("curve_F_Kff_acc")) curve_F_Kff_acc = jsonvalue["curve_F_Kff_acc"].asFloat();

// if(jsonvalue.isMember("crossing_F_Kp")) crossing_F_Kp = jsonvalue["crossing_F_Kp"].asFloat();       //十字角速度
// if(jsonvalue.isMember("crossing_F_Kd")) crossing_F_Kd = jsonvalue["crossing_F_Kd"].asFloat();
// if(jsonvalue.isMember("crossing_F_Kff")) crossing_F_Kff = jsonvalue["crossing_F_Kff"].asFloat();
// if(jsonvalue.isMember("crossing_F_Kff_acc")) crossing_F_Kff_acc = jsonvalue["crossing_F_Kff_acc"].asFloat

// if(jsonvalue.isMember("ring_F_Kp")) ring_F_Kp = jsonvalue["ring_F_Kp"].asFloat();                   //圆环角速度
// if(jsonvalue.isMember("ring_F_Kd")) ring_F_Kd = jsonvalue["ring_F_Kd"].asFloat();
// if(jsonvalue.isMember("ring_F_Kff")) ring_F_Kff = jsonvalue["ring_F_Kff"].asFloat();
// if(jsonvalue.isMember("ring_F_Kff_acc")) ring_F_Kff_acc = jsonvalue["ring_F_Kff_acc"].asFloat();

// if(jsonvalue.isMember("model_F_Kp")) model_F_Kp = jsonvalue["model_F_Kp"].asFloat();                //模型角速度
// if(jsonvalue.isMember("model_F_Kd")) model_F_Kd = jsonvalue["model_F_Kd"].asFloat();
// if(jsonvalue.isMember("model_F_Kff")) model_F_Kff = jsonvalue["model_F_Kff"].asFloat();
// if(jsonvalue.isMember("model_F_Kff_acc")) model_F_Kff_acc = jsonvalue["model_F_Kff_acc"].asFloat();

// if(jsonvalue.isMember("curve_100_Kp")) curve_100_Kp = jsonvalue["curve_100_Kp"].asFloat();  //100速度参数
// if(jsonvalue.isMember("curve_100_Kd")) curve_100_Kd = jsonvalue["curve_100_Kd"].asFloat();
// if(jsonvalue.isMember("curve_110_Kp")) curve_110_Kp = jsonvalue["curve_110_Kp"].asFloat();  //110速度参数
// if(jsonvalue.isMember("curve_110_Kd")) curve_110_Kd = jsonvalue["curve_110_Kd"].asFloat();
// if(jsonvalue.isMember("curve_120_Kp")) curve_120_Kp = jsonvalue["curve_120_Kp"].asFloat();  //120速度参数
// if(jsonvalue.isMember("curve_120_Kd")) curve_120_Kd = jsonvalue["curve_120_Kd"].asFloat();
// if(jsonvalue.isMember("curve_130_Kp")) curve_130_Kp = jsonvalue["curve_130_Kp"].asFloat();  //130速度参数
// if(jsonvalue.isMember("curve_130_Kd")) curve_130_Kd = jsonvalue["curve_130_Kd"].asFloat();
// if(jsonvalue.isMember("curve_140_Kp")) curve_140_Kp = jsonvalue["curve_140_Kp"].asFloat();  //140速度参数
// if(jsonvalue.isMember("curve_140_Kd")) curve_140_Kd = jsonvalue["curve_140_Kd"].asFloat();
// if(jsonvalue.isMember("curve_150_Kp")) curve_150_Kp = jsonvalue["curve_150_Kp"].asFloat();  //150速度参数
// if(jsonvalue.isMember("curve_150_Kd")) curve_150_Kd = jsonvalue["curve_150_Kd"].asFloat();
// if(jsonvalue.isMember("curve_160_Kp")) curve_160_Kp = jsonvalue["curve_160_Kp"].asFloat();  //160速度参数
// if(jsonvalue.isMember("curve_160_Kd")) curve_160_Kd = jsonvalue["curve_160_Kd"].asFloat();



// jsonvalue["preview"] = preview;

// jsonvalue["Ag_F_Kp"] = Ag_F_Kp;                 //正常
// jsonvalue["Ag_F_Kd"] = Ag_F_Kd;
// jsonvalue["Ag_Kff"] = Ag_Kff;
// jsonvalue["Ag_Kff_acc"] = Ag_Kff_acc;

// jsonvalue["straight_F_Kp"] = straight_F_Kp;     //直道
// jsonvalue["straight_F_Kd"] = straight_F_Kd;
// jsonvalue["straight_F_Kff"] = straight_F_Kff;
// jsonvalue["straight_F_Kff_acc"] = straight_F_Kff_acc;

// jsonvalue["safe_F_Kp"] = safe_F_Kp;     //安全过渡
// jsonvalue["safe_F_Kd"] = safe_F_Kd;
// jsonvalue["safe_F_Kff"] = safe_F_Kff;
// jsonvalue["safe_F_Kff_acc"] = safe_F_Kff_acc;

// jsonvalue["curve_F_Kp"] = curve_F_Kp;     //弯道
// jsonvalue["curve_F_Kd"] = curve_F_Kd;
// jsonvalue["curve_F_Kff"] = curve_F_Kff;
// jsonvalue["curve_F_Kff_acc"] = curve_F_Kff_acc;

// jsonvalue["crossing_F_Kp"] = crossing_F_Kp;     //十字
// jsonvalue["crossing_F_Kd"] = crossing_F_Kd;
// jsonvalue["crossing_F_Kff"] = crossing_F_Kff;
// jsonvalue["crossing_F_Kff_acc"] = crossing_F_Kff_acc;

// jsonvalue["ring_F_Kp"] = ring_F_Kp;         //圆环
// jsonvalue["ring_F_Kd"] = ring_F_Kd;
// jsonvalue["ring_F_Kff"] = ring_F_Kff;
// jsonvalue["ring_F_Kff_acc"] = ring_F_Kff_acc;

// jsonvalue["model_F_Kp"] = model_F_Kp;         //模型
// jsonvalue["model_F_Kd"] = model_F_Kd;
// jsonvalue["model_F_Kff"] = model_F_Kff;
// jsonvalue["model_F_Kff_acc"] = model_F_Kff_acc;

//弯道强度参数
// jsonvalue["curve_100_Kp"] = curve_100_Kp;      //100
// jsonvalue["curve_100_Kd"] = curve_100_Kd;
// jsonvalue["curve_110_Kp"] = curve_110_Kp;      //110
// jsonvalue["curve_110_Kd"] = curve_110_Kd;
// jsonvalue["curve_120_Kp"] = curve_120_Kp;      //120
// jsonvalue["curve_120_Kd"] = curve_120_Kd;
// jsonvalue["curve_130_Kp"] = curve_130_Kp;      //130
// jsonvalue["curve_130_Kd"] = curve_130_Kd;
// jsonvalue["curve_140_Kp"] = curve_140_Kp;      //140
// jsonvalue["curve_140_Kd"] = curve_140_Kd;
// jsonvalue["curve_150_Kp"] = curve_150_Kp;      //150
// jsonvalue["curve_150_Kd"] = curve_150_Kd;
// jsonvalue["curve_160_Kp"] = curve_160_Kp;      //160
// jsonvalue["curve_160_Kd"] = curve_160_Kd;
