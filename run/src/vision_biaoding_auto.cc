#include "headfile.h"

extern LCD lcd;
extern Camera cam;
extern TrackBase track_base;
extern PointState point_state;
extern void key_exit(void);        // 退出菜单接口

/////////////////////////////////////////////// 　外部函数接口 ///////////////////////////////////////////
// ==========================================================
// 按键交互
// ==========================================================
// 开始标定
void BiaoDingAuto::start_biaoding(void){
    current_state = BiaoDingState::show;
    lcd.clearScreen();
    std::cout << "[全自动标定] 赛道半宽获取模式启动！" << std::endl;
    std::cout << "请将车头居中放置在一段完美的标准直道上，" << std::endl;
}

// 按键确定回调函数
void BiaoDingAuto::key_enter_handler(void){
    if (current_state == BiaoDingState::show){
        current_state = BiaoDingState::biaoding;
        lcd.clearScreen();
    }
}

// 按键取消回调函数
void BiaoDingAuto::key_quit_handler(void){
    if (current_state == BiaoDingState::biaoding || current_state == BiaoDingState::show){
        current_state = BiaoDingState::idle;
        key_exit();     // 退出到菜单界面
    }
}

// ==========================================================
// BiaoDingAuto (半宽计算) 主循环 状态机 (放在摄像头 while 循环内)
// ==========================================================
void BiaoDingAuto::biaoding_main(const cv::Mat &frame){
    if (current_state == BiaoDingState::idle) return;

    if (current_state == BiaoDingState::show){
        // 实时显示画面，等待按下确定键
        cv::Mat disp_frame;
        if (frame.empty()) return;

        cv::resize(frame, disp_frame, cv::Size(100, 75));
        lcd.showCVImage(0, 0, disp_frame, LCD_FULL_WIDTH, LCD_FULL_HEIGHT);

        // 屏幕显示提示信息
        lcd.showString(5, 120, "step: auto halfwidth");
        lcd.showString(5, 140, "press ENTER to start");

    }
    else if (current_state == BiaoDingState::biaoding){
        lcd.showString(5, 120, "calculating...      ");
        
        cv::Mat work_frame = frame.clone();
        if (work_frame.cols != CAM_WIDTH || work_frame.rows != CAM_HEIGHT) {
            cv::resize(work_frame, work_frame, cv::Size(CAM_WIDTH, CAM_HEIGHT));
        }

        // 调用基础处理提取边线
        track_base.process_frame(work_frame);

        int half_width_arr[CAM_HEIGHT] = {0};
        
        for (int y = 0; y < CAM_HEIGHT; y++) {
            int left_x = left_boundary.line[y];
            int right_x = right_boundary.line[y];

            // 只有左右线都有效时才计算真实半宽
            if (left_x >= 0 && right_x >= 0 && right_x > left_x) {
                half_width_arr[y] = (right_x - left_x) / 2;
            } else {
                half_width_arr[y] = 0; // 标记为无效，稍后处理
            }
        }

        // 数据平滑修补 (向下平推)
        // 视野最上方可能因为透视太远丢线，用最近的有效值填补
        // 从底端向顶端推导，保证近处的半宽能向远方延伸
        int last_valid = std::max(1, CAM_WIDTH / 5);
        for (int y = CAM_HEIGHT - 1; y >= 0; y--) {
            if (half_width_arr[y] != 0 && half_width_arr[y] < CAM_WIDTH) { // 过滤异常大值
                last_valid = half_width_arr[y];
            } else {
                half_width_arr[y] = last_valid; 
            }
        }

        // 打印完美格式的 C++ 数组 (拒绝写入文件，直接复制粘贴！)
        std::cout << "\n==================================================" << std::endl;
        std::cout << "✅ 半宽标定成功！请直接复制以下代码，替换 vision_tracking_base.cc 中的 halfRoad 数组：" << std::endl;
        std::cout << "==================================================\n" << std::endl;

        std::cout << "const uint16_t TrackBase::halfRoad[CAM_HEIGHT] = {" << std::endl;
        for (int y = 0; y < CAM_HEIGHT; y++) {
            if (y % 10 == 0) std::cout << "    "; 
            
            // 自动对齐空格
            if (half_width_arr[y] < 10) std::cout << "  " << half_width_arr[y];
            else if (half_width_arr[y] < 100) std::cout << " " << half_width_arr[y];
            else std::cout << half_width_arr[y];
            
            if (y != CAM_HEIGHT - 1) std::cout << ", ";
            else std::cout << " ";
            
            if ((y + 1) % 10 == 0) std::cout << std::endl;
        }
        std::cout << "};" << std::endl;
        std::cout << "==================================================\n" << std::endl;

        // 标定完成，返回 idle 状态
        current_state = BiaoDingState::idle;
        lcd.clearScreen();
        lcd.showString(5, 60, "Half-width OK!");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        key_exit();     // 退出到菜单界面
    }
}

//　圆环标注菜单
bool circle_label_active = false;   
int circle_label_index = 0; 

// 圆环标注位置设置
void BiaoDingAuto::set_circle_slot_type(int index, int type)
{
    if (type < 0) type = 0;
    if (type > 3) type = 3;

    switch (index) {
        case 0: preprocess.ring1_type = type; break;
        case 1: preprocess.ring2_type = type; break;
        case 2: preprocess.ring3_type = type; break;
        case 3: preprocess.ring4_type = type; break;
        case 4: preprocess.ring5_type = type; break;
        case 5: preprocess.ring6_type = type; break;
        case 6: preprocess.ring7_type = type; break;
        case 7: preprocess.ring8_type = type; break;
        case 8: preprocess.ring9_type = type; break;
        case 9: preprocess.ring10_type = type; break;
        case 10: preprocess.ring11_type = type; break;
        case 11: preprocess.ring12_type = type; break;
        case 12: preprocess.ring13_type = type; break;
        case 13: preprocess.ring14_type = type; break;
        case 14: preprocess.ring15_type = type; break;
        default: break;
    }
}

const char* BiaoDingAuto::circle_type_name(int type)
{
    if (type <= 0) return "SMALL";
    if (type == 1) return "MEDIUM";
    if (type == 2) return "LARGE";
    return "BIG";
}

// 圆环标注界面显示
void BiaoDingAuto::show_circle_label_menu(void)
{
    lcd.clearScreen();
    char buf[32];
    lcd.clearScreen();
    lcd.showString(5, 10, "Circle Label");
    std::snprintf(buf, sizeof(buf), "Slot: Ring %d/15", circle_label_index + 1);
    lcd.showString(5, 30, buf);
    std::snprintf(buf, sizeof(buf), "Count: %d", preprocess.circle_count);
    lcd.showString(5, 50, buf);
    lcd.showString(5, 75, "UP=S ENTER=M");
    lcd.showString(5, 95, "DOWN=L SEL=SAVE");
}

void BiaoDingAuto::finish_circle_label_menu(void)
{
    preprocess.Preprocess_save();
    circle_label_active = false;
    key_exit();
}

//  圆环标注类型选择
void BiaoDingAuto::circle_label_push_type(int type)
{
    if (!circle_label_active) return;

    if (circle_label_index < 0) circle_label_index = 0;
    if (circle_label_index >= 15) {
        finish_circle_label_menu();
        return;
    }

    set_circle_slot_type(circle_label_index, type);
    preprocess.circle_count = circle_label_index + 1;
    preprocess.Preprocess_save();

    char buf[32];
    std::snprintf(buf, sizeof(buf), "Ring%d=%s", circle_label_index + 1, circle_type_name(type));
    lcd.showString(5, 115, "                ");
    lcd.showString(5, 115, buf);

    circle_label_index++;
    if (circle_label_index >= 15) {
        finish_circle_label_menu();
        return;
    }

    show_circle_label_menu();
}

// // 圆环标注回调函数
// void BiaoDingAuto::circle_label_menu(void)
// {
//     circle_label_active = true;
//     circle_label_index = 0;
//     preprocess.circle_count = 0;
//     show_circle_label_menu();

//     while (key->execute && program_run_flag && circle_label_active)
//     {
//         key0.key_listeners();
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }

//     circle_label_active = false;
//     lcd.clearScreen();
//     show_menu();
// }

BiaoDingAuto biaoding_auto;
