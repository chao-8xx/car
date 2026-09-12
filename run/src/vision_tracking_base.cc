#include "headfile.h"

using namespace cv;
using namespace std;


/*      迷宫法前进方向定义:
*             2
*          3     1
*        4         0
*          5     7
*             6
*/

// 迷宫法方向定义（已屏蔽）
// const int TrackBase::dir_front[8][2] = {
//     { 1,  0},            // 0: 右
//     { 1, -1},            // 1: 右上
//     { 0, -1},            // 2: 上
//     {-1, -1},            // 3: 左上
//     {-1,  0},            // 4: 左
//     {-1,  1},            // 5: 左下
//     { 0,  1},            // 6: 下
//     { 1,  1}             // 7: 右下
// };


// // 赛道半宽表 (透视补偿)
// // 对应视频图像行: 0(顶端) -> 119(底端)
// const uint16_t TrackBase::halfRoad[120] = {
//      15,  15,  16,  16,  16,  17,  18,  18,  19,  19,
//      20,  20,  21,  21,  22,  23,  23,  24,  24,  24,
//      25,  25,  27,  27,  28,  28,  29,  29,  30,  31,
//      31,  32,  32,  33,  33,  34,  35,  35,  36,  37,
//      37,  37,  38,  39,  39,  40,  41,  41,  41,  42,
//      42,  43,  44,  45,  45,  45,  46,  46,  47,  47,
//      48,  49,  49,  50,  50,  51,  51,  52,  53,  53,
//      53,  54,  55,  55,  55,  56,  57,  57,  58,  58,
//      59,  59,  60,  60,  61,  61,  62,  63,  63,  64,
//      64,  65,  65,  66,  66,  67,  67,  68,  68,  69,
//      69,  70,  70,  71,  71,  72,  72,  73,  73,  73,
//      74,  74,  74,  75,  75,  75,  75,  75,  76,  76,
// };

// // 赛道半宽表 (实际测量 待更新完美一版)

const uint16_t TrackBase::halfRoad[CAM_HEIGHT] = {
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   5,   6,   6,   7,   8,
      9,   9,  10,  11,  12,  12,  13,  14,  15,  16,
     16,  17,  18,  19,  19,  20,  21,  21,  22,  23,
     23,  24,  25,  25,  26,  26,  27,  28,  28,  29,
     16,  16,  16,  16,  16,  16,  16,  16,  16,  16
};






///////////////////////////////////////////// 全局变量 //////////////////////////////////////////////
   
// 对象声明
extern LCD lcd;
// VideoCapture cap;
// extern Camera cam;
PointState point_state;
extern CameraStreamServer camera_server;
// extern ElementState element_state;

//////////////////////////////////////////////////////　外部调用接口 ///////////////////////////////////////////
// 初始化
void TrackBase::track_init(void ){
    // 为成员变量分配内存
    raw_buf.create(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
    gray_buf.create(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
    bin_buf.create(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
    debug_buf.create(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
}
 
// 视觉预处理
void TrackBase::preprocess_frame(const Mat& src){
    // 边界检测
    if (src.empty())    return;
    
    // 摄像头尺寸缩放
    if (src.channels() == 3) {
        resize(src, raw_buf, Size(CAM_WIDTH, CAM_HEIGHT));
        cvtColor(raw_buf, gray_buf, COLOR_BGR2GRAY);
    }
    else {
        resize(src, gray_buf, Size(CAM_WIDTH, CAM_HEIGHT));
        cvtColor(gray_buf, raw_buf, COLOR_GRAY2BGR);
    }

    // 全图中值滤波处理 降噪（可选）
    bool enable_blur = true; 
    blur_median(gray_buf, 3, enable_blur);

    // // 在灰度图的两边上画黑线 (防止迷宫法扫线时数据溢出  暂时用不到 但先保留)
    // cv::line(gray_buf, cv::Point(0, 0), cv::Point(0, gray_buf.rows-1), cv::Scalar(0), 1);
    // cv::line(gray_buf, cv::Point(1, 0), cv::Point(1, gray_buf.rows-1), cv::Scalar(0), 1);
    // cv::line(gray_buf, cv::Point(gray_buf.cols-1, 0), cv::Point(gray_buf.cols-1, gray_buf.rows-1), cv::Scalar(0), 1);
    // cv::line(gray_buf, cv::Point(gray_buf.cols-2, 0), cv::Point(gray_buf.cols-2, gray_buf.rows-1), cv::Scalar(0), 1);
}


// 滑动窗口　巡边线处理　主函数
void TrackBase::process_frame(const Mat& src){
    // 预处理帧
    preprocess_frame(src);
    
    // 边界点集寻找 处理 
    process_boundary();

    // 角点与丢线分析
    point_state.analyze_boundary(left_boundary, true);
    point_state.analyze_boundary(right_boundary, false);

    // 最长白列法拐点检测（与dx突变法并存）
    // point_state.find_corners(bin_buf, left_boundary, right_boundary);

    // 出界检测  拦截所有处理前最高优先级   在绕行时不执行出界检测
    if(pass_state == none && !element_state.model_line_pass_active)
    {
        if (out_checking(bin_buf))
        {
            // 关闭电机 
            car_soft_stop();
            Gogo = false;
            return; 
        }
    }
    
    
    // 元素状态机
    element_state.elements_process(bin_buf, raw_buf, 0);        // 0 替代 tof_distance

    // 拟合中线
    // TrackMode current_mode = TrackMode::auto_mode;
    process_midline(left_boundary, right_boundary, mid, current_mode);

    // // speed为编码器速度(写进了preprocess.cc的打包函数内)，利用前瞻算出瞬时偏差 now_error
    // float now_error = get_error_lookahead(30, 0.5f,7);

    // // 一阶低通滤波平滑偏差
    // track_error = 0.6f * track_error + 0.4f * now_error;

    // // 显示图像信息 (放到菜单 不放到函数里)
    // track_display();

    //获取误差
    photo_err = get_error_lookahead(100,preprocess.preview,preprocess.window_size);
}


// 边界处理主函数（最长白列扫线 + 八邻域边界提取）
void TrackBase::process_boundary(void){
    // 二值化 + 最长白列
    quick_otsu(gray_buf);
    seek_pts_seed(bin_buf);  // 最长白列法（作为备选数据）

    // 尝试八邻域边界提取
    bool use_8n = point_state.extract_boundary_8neighbor(bin_buf, left_boundary, right_boundary);

    // 逐行扫线（无论8邻域是否成功都执行，用于融合补线）
    int row_left[CAM_HEIGHT];
    int row_right[CAM_HEIGHT];
    for (int y = 0; y < CAM_HEIGHT; y++) {
        row_left[y] = -1;
        row_right[y] = -1;
    }

    int center_x = (white_column.starlineX + white_column.endlineX) / 2;
    if (center_x < VALID_LEFT_COL + 2) center_x = VALID_LEFT_COL + 2;
    if (center_x > VALID_RIGHT_COL - 2) center_x = VALID_RIGHT_COL - 2;

    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        const uchar* row = bin_buf.ptr<uchar>(y);

        // 左扫线：从 center_x 向左找白→黑跳变
        int lx = -1;
        for (int x = center_x; x >= VALID_LEFT_COL + 1; x--) {
            if (row[x] > 127 && row[x - 1] <= 127) {
                lx = x;
                break;
            }
        }
        if (lx != -1 && lx <= VALID_LEFT_COL + 2) lx = -1;
        row_left[y] = lx;

        // 右扫线：从 center_x 向右找白→黑跳变
        int rx = -1;
        for (int x = center_x; x < VALID_RIGHT_COL - 1; x++) {
            if (row[x] > 127 && row[x + 1] <= 127) {
                rx = x;
                break;
            }
        }
        if (rx != -1 && rx >= VALID_RIGHT_COL - 2) rx = -1;
        row_right[y] = rx;
    }

    if (!use_8n) {
        // 八邻域失败：完全采用逐行扫线
        for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
            left_boundary.scan_line[y] = row_left[y];
            left_boundary.line[y] = row_left[y];
            if (row_left[y] != -1) left_boundary.raw_pts.push_back(cv::Point(row_left[y], y));

            right_boundary.scan_line[y] = row_right[y];
            right_boundary.line[y] = row_right[y];
            if (row_right[y] != -1) right_boundary.raw_pts.push_back(cv::Point(row_right[y], y));
        }
    }         
    // 八邻域成功：融合逐行扫线，优先补齐上半区断点，避免入环前上部缺线
    else {

        for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
            int l8 = left_boundary.line[y];
            int ls = row_left[y];
            int l_fused = l8;

            if (l_fused == -1) {
                l_fused = ls;
            } else if (ls != -1) {
                int diff = std::abs(l8 - ls);
                if (diff <= 3) {
                    l_fused = (l8 + ls) / 2;
                } else {
                    int prev = (y < VALID_START_ROW) ? left_boundary.scan_line[y + 1] : -1;
                    if (prev != -1) {
                        l_fused = (std::abs(ls - prev) + 1 < std::abs(l8 - prev)) ? ls : l8;
                    } else if (y <= VALID_START_ROW - ROI_HEIGHT / 3) {
                        l_fused = ls;
                    }
                }
            }

            int r8 = right_boundary.line[y];
            int rs = row_right[y];
            int r_fused = r8;

            if (r_fused == -1) {
                r_fused = rs;
            } else if (rs != -1) {
                int diff = std::abs(r8 - rs);
                if (diff <= 3) {
                    r_fused = (r8 + rs) / 2;
                } else {
                    int prev = (y < VALID_START_ROW) ? right_boundary.scan_line[y + 1] : -1;
                    if (prev != -1) {
                        r_fused = (std::abs(rs - prev) + 1 < std::abs(r8 - prev)) ? rs : r8;
                    } else if (y <= VALID_START_ROW - ROI_HEIGHT / 3) {
                        r_fused = rs;
                    }
                }
            }

            left_boundary.scan_line[y] = l_fused;
            right_boundary.scan_line[y] = r_fused;

            // 保留8邻域的主线定义，仅对缺失行做补齐
            if (left_boundary.line[y] == -1) {
                left_boundary.line[y] = l_fused;
                if (l_fused != -1) left_boundary.raw_pts.push_back(cv::Point(l_fused, y));
            }
            if (right_boundary.line[y] == -1) {
                right_boundary.line[y] = r_fused;
                if (r_fused != -1) right_boundary.raw_pts.push_back(cv::Point(r_fused, y));
            }
        }
    }

    detect_far_upper_edge(bin_buf);
}

// 远上边界检测
void TrackBase::detect_far_upper_edge(const cv::Mat &buf) {
    far_upper_edge.reset();
    if (buf.empty() || buf.rows < CAM_HEIGHT || buf.cols < CAM_WIDTH) return;

    const int far_top = VALID_END_ROW + 5;
    const int far_bottom = far_top + 15;
    const int min_seg_len = 7;
    const int min_valid_rows = 4;

    for (int y = far_top; y <= far_bottom; y++) {
        int best_l = -1;
        int best_r = -1;
        int best_len = 0;
        int x = VALID_LEFT_COL;

        while (x <= VALID_RIGHT_COL) {
            const uchar center = buf.at<uchar>(y, x);
            const uchar above = buf.at<uchar>(y - 1, x);
            const uchar below = buf.at<uchar>(y + 1, x);
            bool is_edge = (center > now_threshold &&
                            below > now_threshold &&
                            above <= now_threshold);
            if (!is_edge) {
                x++;
                continue;
            }

            int l = x;
            while (x <= VALID_RIGHT_COL) {
                const uchar c = buf.at<uchar>(y, x);
                const uchar a = buf.at<uchar>(y - 1, x);
                const uchar b = buf.at<uchar>(y + 1, x);
                if (!(c > now_threshold && b > now_threshold && a <= now_threshold)) break;
                x++;
            }

            int r = x - 1;
            int len = r - l + 1;
            if (len > best_len) {
                best_len = len;
                best_l = l;
                best_r = r;
            }
        }

        if (best_len >= min_seg_len) {
            far_upper_edge.left_x[y] = best_l;
            far_upper_edge.right_x[y] = best_r;
            far_upper_edge.count++;
        }
    }

    far_upper_edge.valid = (far_upper_edge.count >= min_valid_rows);
}

// 图像显示在lcd 屏幕
void TrackBase::track_display(void){
    if (gray_buf.empty() || gray_buf.rows <= 0 || gray_buf.cols <= 0) return; // 保护空图像

    // 整体: 128 * 160 的纯黑背景
    cv::Mat background = cv::Mat::zeros(LCD_FULL_HEIGHT, LCD_FULL_WIDTH, CV_8UC3);

    // 主视觉画面 (100x75)
    cv::Mat main_lcd;
    cv::cvtColor(gray_buf, main_lcd, COLOR_GRAY2BGR);
    
    if (main_lcd.empty()) return; // 防止空输出

    // 在主画面上绘制三线与特征点 (与图传对齐)
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        // 左边线 (蓝色)
        if (left_boundary.line[y] >= 0 && left_boundary.line[y] < CAM_WIDTH) {
            main_lcd.at<cv::Vec3b>(y, left_boundary.line[y]) = cv::Vec3b(255, 0, 0); 
        }
        // 右边线 (红色)
        if (right_boundary.line[y] >= 0 && right_boundary.line[y] < CAM_WIDTH) {
            main_lcd.at<cv::Vec3b>(y, right_boundary.line[y]) = cv::Vec3b(0, 0, 255);
        }
        // 中线  (绿色)
        if (mid.line[y] >= 0 && mid.line[y] < CAM_WIDTH) { 
            main_lcd.at<cv::Vec3b>(y, mid.line[y]) = cv::Vec3b(0, 255, 0); 
        }
    }

    // 下角点 黄色
    if (left_boundary.down_lp_state) cv::circle(main_lcd, left_boundary.down_lp_pt, 4, cv::Scalar(0, 255, 255), 2);
    if (right_boundary.down_lp_state) cv::circle(main_lcd, right_boundary.down_lp_pt, 4, cv::Scalar(0, 255, 255), 2);

    // 上角点 紫色
    if (left_boundary.up_lp_state) {
        cv::circle(main_lcd, left_boundary.up_lp_pt, 4, cv::Scalar(255, 0, 255), 2);  
    }
    if (right_boundary.up_lp_state) {
        cv::circle(main_lcd, right_boundary.up_lp_pt, 4, cv::Scalar(255, 0, 255), 2);
    }

    // 中拐点 橙色
    if (left_boundary.mid_lp_state) {
        cv::circle(main_lcd, left_boundary.mid_lp_pt, 4, cv::Scalar(0, 128, 255), 2);
    }
    if (right_boundary.mid_lp_state) {
        cv::circle(main_lcd, right_boundary.mid_lp_pt, 4, cv::Scalar(0, 128, 255), 2);
    }

    // // 最长白列法检测的拐点 - 用不同颜色显示
    // // 最长白列下拐点 青色
    // if (left_boundary.longest_white_down_state) {
    //     cv::circle(main_lcd, left_boundary.longest_white_down_pt, 3, cv::Scalar(255, 255, 0), 2);
    // }
    // if (right_boundary.longest_white_down_state) {
    //     cv::circle(main_lcd, right_boundary.longest_white_down_pt, 3, cv::Scalar(255, 255, 0), 2);
    // }

    // // 最长白列上拐点 青色
    // if (left_boundary.longest_white_up_state) {
    //     cv::circle(main_lcd, left_boundary.longest_white_up_pt, 3, cv::Scalar(255, 255, 0), 2);
    // }
    // if (right_boundary.longest_white_up_state) {
    //     cv::circle(main_lcd, right_boundary.longest_white_up_pt, 3, cv::Scalar(255, 255, 0), 2);
    // }

    // // 出环点 青色
    // if (left_boundary.out_lp_state) {
    //     cv::circle(main_lcd, left_boundary.out_lp_pt, 4, cv::Scalar(255, 255, 0), 2);
    // }
    // if (right_boundary.out_lp_state) {
    //     cv::circle(main_lcd, right_boundary.out_lp_pt, 4, cv::Scalar(255, 255, 0), 2);
    // }

    // // 绘制突变点 (紫色圆圈)
    // if (left_boundary.break_state) cv::circle(main_lcd, left_boundary.break_pt, 4, cv::Scalar(255, 0, 255), 2);
    // if (right_boundary.break_state) cv::circle(main_lcd, right_boundary.break_pt, 4, cv::Scalar(255, 0, 255), 2);

    // ROI 的上下裁剪物理边界 
    cv::line(main_lcd, cv::Point(0, VALID_END_ROW), cv::Point(CAM_WIDTH - 1, VALID_END_ROW), cv::Scalar(50, 50, 50), 1);
    cv::line(main_lcd, cv::Point(0, VALID_START_ROW), cv::Point(CAM_WIDTH - 1, VALID_START_ROW), cv::Scalar(50, 50, 50), 1);
    
    // ROI 的左右裁剪物理边界
    cv::line(main_lcd, cv::Point(VALID_LEFT_COL, 0), cv::Point(VALID_LEFT_COL, CAM_HEIGHT - 1), cv::Scalar(50, 50, 50), 1);
    cv::line(main_lcd, cv::Point(VALID_RIGHT_COL, 0), cv::Point(VALID_RIGHT_COL, CAM_HEIGHT - 1), cv::Scalar(50, 50, 50), 1);

    // 误差计算范围框显示
    if (debug_error_end > 0 && debug_error_start < CAM_HEIGHT && debug_error_end <= debug_error_start) {
        // 在主画面上绘制上下边界构成的红框
        cv::rectangle(main_lcd, 
                      cv::Point(0, debug_error_end), 
                      cv::Point(CAM_WIDTH - 1, debug_error_start), 
                      cv::Scalar(0, 0, 255), 1);
    }

    // 将 160x120 缩放为 100x75 (保持 4:3 比例) 的主画面
    cv::Mat main_lcd_resized;
    cv::resize(main_lcd, main_lcd_resized, cv::Size(100, 75));
    main_lcd_resized.copyTo(background(Rect(0, 0, 100, 75)));

    // 右侧检测状态机 (x_offset = 102) 
    int offset_x = 104; 
    cv::Scalar color_on = cv::Scalar(0, 255, 0);     // 触发变亮绿
    cv::Scalar color_off = cv::Scalar(64, 64, 64);   // 未触发暗灰

    // 角点检测
    cv::putText(background, "L_D", cv::Point(offset_x, 15), cv::FONT_HERSHEY_SIMPLEX, 0.45, left_boundary.down_lp_state ? color_on : color_off, 1);
    cv::putText(background, "R_D", cv::Point(offset_x, 35), cv::FONT_HERSHEY_SIMPLEX, 0.45, right_boundary.down_lp_state ? color_on : color_off, 1); 
    cv::putText(background, "L_M", cv::Point(offset_x, 95), cv::FONT_HERSHEY_SIMPLEX, 0.45, left_boundary.mid_lp_state ? color_on : color_off, 1);
    cv::putText(background, "R_M", cv::Point(offset_x, 115), cv::FONT_HERSHEY_SIMPLEX, 0.45, right_boundary.mid_lp_state ? color_on : color_off, 1);
    cv::putText(background, "L_U", cv::Point(offset_x, 55), cv::FONT_HERSHEY_SIMPLEX, 0.45, left_boundary.up_lp_state ? color_on : color_off, 1);
    cv::putText(background, "R_U", cv::Point(offset_x, 75), cv::FONT_HERSHEY_SIMPLEX, 0.45, right_boundary.up_lp_state ? color_on : color_off, 1);
    // cv::putText(background, "L_O", cv::Point(offset_x, 135), cv::FONT_HERSHEY_SIMPLEX, 0.45, left_boundary.out_lp_state ? color_on : color_off, 1);
    // cv::putText(background, "R_O", cv::Point(offset_x, 155), cv::FONT_HERSHEY_SIMPLEX, 0.45, right_boundary.out_lp_state ? color_on : color_off, 1);

    // 正下方调试面板
    cv::line(background, cv::Point(0, 76), cv::Point(LCD_FULL_WIDTH, 76), cv::Scalar(80, 80, 80), 1);

    char text_buf[32];
    int panel_y = 95;
    int line_spacing = 20;

    // 误差显示
    sprintf(text_buf, "Err : %.1f", photo_err);
    cv::putText(background, text_buf, cv::Point(5, panel_y), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 165, 255), 1);

    // 中线长度
    int ln_count = 0;
    for(int y = 0; y < CAM_HEIGHT; y++) { if(mid.line[y] >= 0 && mid.line[y] < CAM_WIDTH) ln_count++; }
    sprintf(text_buf, "Ln  : %d px", ln_count);
    cv::putText(background, text_buf, cv::Point(5, panel_y + line_spacing * 1), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1); 

    // 丢线监控 
    bool l_lost = left_boundary.is_lost;
    bool r_lost = right_boundary.is_lost;
    cv::putText(background, l_lost ? "L:LOST" : "L:OK", cv::Point(5, panel_y + line_spacing * 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, l_lost ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 1);
    cv::putText(background, r_lost ? "R:LOST" : "R:OK", cv::Point(60, panel_y + line_spacing * 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, r_lost ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 1);

    // 出界警告
    if (is_out_of_bounds) {
        cv::putText(background, "OUT!", cv::Point(5, 155), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 2);
    }

    // 元素状态显示
#if 1
    
    // 常规道路类型显示（只在非特殊元素时显示）
    if (element_state.current_track_type == TrackType::normal) {
        const char* road_type = "RD:?";
        cv::Scalar road_color = cv::Scalar(200, 200, 200);  // 灰色
        
        if (element_state.normal_type == NormalType::straight) {
            road_type = "RD:Str";
            road_color = cv::Scalar(0, 255, 0);  // 绿色 - 直道
        }
        else if (element_state.normal_type == NormalType::curve) {
            road_type = "RD:Crv";
            road_color = cv::Scalar(255, 128, 0);  // 橙色 - 弯道
        }
        
        cv::putText(background, road_type, cv::Point(5, 125), cv::FONT_HERSHEY_SIMPLEX, 0.4, road_color, 1);
    }
    
    // 十字状态显示
    if (element_state.crossroad_state != CrossroadState::none) {
        const char* cs = "CR:?";
        cv::Scalar cs_color = cv::Scalar(0, 255, 255);  // 青色
        
        if (element_state.crossroad_state == CrossroadState::approach) {
            cs = "CR:Appr";
        }
        else if (element_state.crossroad_state == CrossroadState::inside) {
            cs = "CR:Insd";
            cs_color = cv::Scalar(0, 255, 0);  // 绿色
        }
        
        cv::putText(background, cs, cv::Point(5, 155), cv::FONT_HERSHEY_SIMPLEX, 0.4, cs_color, 1);
        
        // 十字模式显示（编码器/视觉）
        #if CROSSROAD_MODE == CROSSROAD_ENCODER
            cv::putText(background, "ENC", cv::Point(70, 155), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 165, 0), 1);
        #elif CROSSROAD_MODE == CROSSROAD_VISION
            cv::putText(background, "VIS", cv::Point(70, 155), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 0), 1);
        #endif
    }
    
    // 圆环状态显示
    if (element_state.circle_state != CircleState::none) {
        const char* cir = "CIR:?";
        cv::Scalar cir_color = cv::Scalar(255, 128, 0);
        
        if (element_state.circle_state == CircleState::weak_approach) {
            cir = "CIR:WkAp";
        }
        else if (element_state.circle_state == CircleState::approach) {
            cir = "CIR:Appr";
        }
        else if (element_state.circle_state == CircleState::inside) {
            cir = "CIR:Insd";
            cir_color = cv::Scalar(0, 255, 0);
        }
        else if (element_state.circle_state == CircleState::exit) {
            cir = "CIR:Exit";
        }
        else if (element_state.circle_state == CircleState::leave) {
            cir = "CIR:Leav";
        }
        
        cv::putText(background, cir, cv::Point(5, 145), cv::FONT_HERSHEY_SIMPLEX, 0.4, cir_color, 1);
        
        // 圆环方向
        const char* dir = "?";
        if (element_state.circle_dir == CircleDir::left) dir = "L";
        else if (element_state.circle_dir == CircleDir::right) dir = "R";
        cv::putText(background, dir, cv::Point(70, 145), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
    }
    
    // 斑马线状态显示
    if (element_state.zebra_state != ZebraState::none) {
        const char* zebra = "ZEB:?";
        cv::Scalar zebra_color = cv::Scalar(0, 255, 255);  // 青色
        
        if (element_state.zebra_state == ZebraState::detected) {
            zebra = "ZEB:Det";
            zebra_color = cv::Scalar(0, 255, 0);  // 绿色
        }
        else if (element_state.zebra_state == ZebraState::shielded) {
            zebra = "ZEB:Shd";
            zebra_color = cv::Scalar(128, 128, 128);  // 灰色
        }
        
        cv::putText(background, zebra, cv::Point(5, 135), cv::FONT_HERSHEY_SIMPLEX, 0.4, zebra_color, 1);
    }

#endif

    // 显示最终合成图像
    lcd.showCVImage(0, 0, background, LCD_FULL_WIDTH, LCD_FULL_HEIGHT);
}

// ==================================================================================
// 辅助函数：坐标映射　全局坐标 320 * 240 <-> 赛道坐标 80 * 60
// ==================================================================================
namespace {
    // 全局大图坐标 -> 赛道缩放坐标 (80 * 60)
    int map_full_x_to_track(int x, int full_width)
    {
        if (full_width <= 0) return 0;
        return std::max(0, std::min(CAM_WIDTH - 1, x * CAM_WIDTH / full_width));
    }

    int map_full_y_to_track(int y, int full_height)
    {
        if (full_height <= 0) return 0;
        return std::max(0, std::min(CAM_HEIGHT - 1, y * CAM_HEIGHT / full_height));
    }

    // 赛道缩放坐标 (80 * 60) -> 全局大图坐标
    int map_track_x_to_full(int x, int full_width)
    {
        return std::max(0, std::min(full_width - 1,
            static_cast<int>(std::lround((x + 0.5f) * full_width / CAM_WIDTH))));
    }

    int map_track_y_to_full(int y, int full_height)
    {
        return std::max(0, std::min(full_height - 1,
            static_cast<int>(std::lround((y + 0.5f) * full_height / CAM_HEIGHT))));
    }
}

// ==================================================================================
// 图传显示  （全动态坐标自适应映射）
// ==================================================================================
void TrackBase::display_upimage(cv::Mat &frame_buf){
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 3 != 0) return; // 帧率限制
    if (frame_buf.empty()) return;

    // 采用彩色图进行图传
    if (frame_buf.channels() == 1) {
        cv::cvtColor(frame_buf, debug_buf, cv::COLOR_GRAY2BGR);
    } 
    else {
        frame_buf.copyTo(debug_buf);
    }

    // 获取当前图像的实际分辨率，用于动态比例换算
    const int full_width = debug_buf.cols;
    const int full_height = debug_buf.rows;

    // 坐标转换 方便后续对 Point 点集进行映射
    auto map_point_to_full = [&](const cv::Point& pt_track) {
        return cv::Point(map_track_x_to_full(pt_track.x, full_width), 
                         map_track_y_to_full(pt_track.y, full_height));
    };

    // 拍照时注释掉
    #if 1
    // 动态转换并绘制 ROI 物理裁剪边界
    int full_roi_top    = map_track_y_to_full(ROI_TOP, full_height);
    int full_roi_bottom = map_track_y_to_full(ROI_BOTTOM, full_height);
    int full_roi_left   = map_track_x_to_full(ROI_LEFT, full_width);
    int full_roi_right  = map_track_x_to_full(ROI_RIGHT, full_width);

    // 绘制上下有效区域 ROI 边界
    cv::line(debug_buf, cv::Point(0, full_roi_top), cv::Point(full_width, full_roi_top), cv::Scalar(100, 100, 100), 2);
    cv::line(debug_buf, cv::Point(0, full_roi_bottom), cv::Point(full_width, full_roi_bottom), cv::Scalar(100, 100, 100), 2);
    
    // 绘制左右有效区域 ROI 边界
    cv::line(debug_buf, cv::Point(full_roi_left, 0), cv::Point(full_roi_left, full_height), cv::Scalar(100, 100, 100), 2);
    cv::line(debug_buf, cv::Point(full_roi_right, 0), cv::Point(full_roi_right, full_height), cv::Scalar(100, 100, 100), 2);

    // 基于赛道一维数组进行安全循环（使用赛道坐标 y 索引，彻底避免越界崩溃）
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        // 先计算当前行对应的全局图像纵坐标
        int full_y = map_track_y_to_full(y, full_height);

        // 左 scan_line 青色小点
        if (left_boundary.scan_line[y] >= 0 && left_boundary.scan_line[y] < CAM_WIDTH) {
            int full_x = map_track_x_to_full(left_boundary.scan_line[y], full_width);
            cv::circle(debug_buf, cv::Point(full_x, full_y), 2, cv::Scalar(255, 255, 0), -1);
        }
        // 右 scan_line 黄色小点
        if (right_boundary.scan_line[y] >= 0 && right_boundary.scan_line[y] < CAM_WIDTH) {
            int full_x = map_track_x_to_full(right_boundary.scan_line[y], full_width);
            cv::circle(debug_buf, cv::Point(full_x, full_y), 2, cv::Scalar(0, 255, 255), -1);
        }
        // 左边线 蓝色
        if (left_boundary.line[y] >= 0 && left_boundary.line[y] < CAM_WIDTH) {
            int full_x = map_track_x_to_full(left_boundary.line[y], full_width);
            cv::circle(debug_buf, cv::Point(full_x, full_y), 2, cv::Scalar(255, 0, 0), -1);
        }
        // 右边线 红色
        if (right_boundary.line[y] >= 0 && right_boundary.line[y] < CAM_WIDTH) {
            int full_x = map_track_x_to_full(right_boundary.line[y], full_width);
            cv::circle(debug_buf, cv::Point(full_x, full_y), 2, cv::Scalar(0, 0, 255), -1);
        }
        // 中线 绿色
        if (mid.line[y] >= 0 && mid.line[y] < CAM_WIDTH) {
            int full_x = map_track_x_to_full(mid.line[y], full_width);
            cv::circle(debug_buf, cv::Point(full_x, full_y), 2, cv::Scalar(0, 255, 0), -1);
        }
        // 远上边线 有效则绘制 青色 线
        if (far_upper_edge.valid &&
            far_upper_edge.left_x[y] >= 0 &&
            far_upper_edge.right_x[y] >= far_upper_edge.left_x[y]) {
            int full_l = map_track_x_to_full(far_upper_edge.left_x[y], full_width);
            int full_r = map_track_x_to_full(far_upper_edge.right_x[y], full_width);
            cv::line(debug_buf, cv::Point(full_l, full_y), cv::Point(full_r, full_y),
                     cv::Scalar(0, 165, 255), 2);
        }
    }

    // 误差计算范围框动态映射显示
    if (debug_error_end > 0 && debug_error_start < CAM_HEIGHT && debug_error_end <= debug_error_start) {
        int full_error_start = map_track_y_to_full(debug_error_start, full_height);
        int full_error_end   = map_track_y_to_full(debug_error_end, full_height);
        
        // 在图传画面上绘制动态红框 (宽度自适应 full_width - 1)
        cv::rectangle(debug_buf, 
                      cv::Point(0, full_error_end), 
                      cv::Point(full_width - 1, full_error_start), 
                      cv::Scalar(0, 0, 255), 2);
    }

    // 圆环调试：显示当前 yaw角度值
    if (element_state.current_track_type == TrackType::circle &&
        element_state.circle_state != CircleState::none) {
        char yaw_text[40];
        char abs_yaw_text[40];
        float now_yaw = yaw_angle;
        float abs_yaw = std::fabs(now_yaw);

        snprintf(yaw_text, sizeof(yaw_text), "YAW:%+.1f", now_yaw);
        snprintf(abs_yaw_text, sizeof(abs_yaw_text), "ABS:%5.1f", abs_yaw);

        cv::rectangle(debug_buf, cv::Point(2, 2), cv::Point(88, 25), cv::Scalar(0, 0, 0), -1);
        cv::putText(debug_buf, yaw_text, cv::Point(5, 11), cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 255, 255), 1);
        cv::putText(debug_buf, abs_yaw_text, cv::Point(5, 22), cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 255, 255), 1);
    }

    // 下角点 黄色
    if (left_boundary.down_lp_state) {
        cv::circle(debug_buf, map_point_to_full(left_boundary.down_lp_pt), 3, cv::Scalar(0, 255, 255), 3);  
    }
    if (right_boundary.down_lp_state) {
        cv::circle(debug_buf, map_point_to_full(right_boundary.down_lp_pt), 3, cv::Scalar(0, 255, 255), 3);
    }

    // 上角点 紫色
    if (left_boundary.up_lp_state) {
        cv::circle(debug_buf, map_point_to_full(left_boundary.up_lp_pt), 3, cv::Scalar(255, 0, 255), 3);  
    }
    if (right_boundary.up_lp_state) {
        cv::circle(debug_buf, map_point_to_full(right_boundary.up_lp_pt), 3, cv::Scalar(255, 0, 255), 3);
    }
    
    // 中拐点 橙色
    if (left_boundary.mid_lp_state) {
        cv::circle(debug_buf, map_point_to_full(left_boundary.mid_lp_pt), 3, cv::Scalar(0, 128, 255), 3);
    }
    if (right_boundary.mid_lp_state) {
        cv::circle(debug_buf, map_point_to_full(right_boundary.mid_lp_pt), 3, cv::Scalar(0, 128, 255), 3);
    }

    #endif

    camera_server.update_frame_mat(debug_buf);
}

//////////////////////////////////////////////////////　内部函数处理 ///////////////////////////////////////////
// 中线处理 (数组体系)
// 计算赛道中线
//
// 根据左右边界点计算中线位置，在弯道和圆环等特殊元素内动态调整半宽系数使车辆切内弯行驶
// 包含数组滤波消除噪点、继承机制填补短暂丢线、以及最小二乘法预测长距离丢线区域
//
// @param left    左边界数据
// @param right   右边界数据
// @param mid     输出的中线数据
// @param mode    巡线模式（自动/强制单边左/强制单边右）
void TrackBase::process_midline(const PointState::BoundaryData& left, const PointState::BoundaryData& right, PointState::MidlineData& mid, TrackMode mode) 
{
    mid.reset();

    float half_width_rate = 1.0f;       // 半宽系数
    
    // 弯道　圆环内部时　　半宽系数减小　使车子切内弯拐弯
    if (element_state.current_track_type == TrackType::normal &&
        element_state.normal_type == NormalType::curve) {
        half_width_rate = 1.15;        // 待调  差不多是中间位置
    }
    else if (element_state.current_track_type == TrackType::circle &&
             (element_state.circle_state == CircleState::approach)) {
        half_width_rate = 0.90f;        // 待调　可以　很内切
    }
    else if (element_state.current_track_type == TrackType::circle &&
              (element_state.circle_state == CircleState::inside)) {
        half_width_rate = 1.25f;        // 待调　还可以　前面会往外拐一下　后面也比较内切　而且如果内切太狠的话　会影响出环角度
    }
    else if (element_state.current_track_type == TrackType::circle &&
              (element_state.circle_state == CircleState::exit)) {
        half_width_rate = 1.20f;        // 待调　还可以
    }
    else if (element_state.current_track_type == TrackType::crossroad &&
              (element_state.crossroad_state == CrossroadState::inside)) {
        half_width_rate = 1.15f;        // 待调　还可以
    }

    // 有效区域内进行中线补全
    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        if (y < 0 || y >= CAM_HEIGHT) continue;

        int lx = left.line[y];
        int rx = right.line[y];
        int mx = -1;
        float half_width = halfRoad[y] * half_width_rate;

        bool far_edge_enable =
            far_upper_edge.valid &&
            y <= VALID_START_ROW - ROI_HEIGHT / 4 &&
            ((element_state.current_track_type == TrackType::normal &&
              element_state.normal_type == NormalType::curve) ||
             (element_state.current_track_type == TrackType::circle &&
              element_state.circle_state != CircleState::none));

        if (far_edge_enable && mode == TrackMode::auto_mode) {
            int upper_l = far_upper_edge.left_x[y];
            int upper_r = far_upper_edge.right_x[y];
            int min_width = std::max(8, (int)std::lround(halfRoad[y] * 1.2f));
            int max_width = VALID_RIGHT_COL - VALID_LEFT_COL;

            if ((rx == -1 || (right.is_lost && right.lost_y >= y)) &&
                lx != -1 && upper_r != -1) {
                int width = upper_r - lx;
                if (width >= min_width && width <= max_width) {
                    rx = upper_r;
                }
            }

            if ((lx == -1 || (left.is_lost && left.lost_y >= y)) &&
                rx != -1 && upper_l != -1) {
                int width = rx - upper_l;
                if (width >= min_width && width <= max_width) {
                    lx = upper_l;
                }
            }
        }

        // 巡线状态机
        if (mode == TrackMode::left_raw && lx != -1) {
            mx = lx;
        }
        else if (mode == TrackMode::right_raw && rx != -1) {
            mx = rx;
        }
        else if (mode == TrackMode::left_full && lx != -1) {     // 强制单边巡线模式 (特殊元素内)
            mx = lx + half_width;
        } 
        else if (mode == TrackMode::right_full && rx != -1) {
            mx = rx - half_width;
        } 
        else {                                              // 正常自动模式
            if (lx != -1 && rx != -1) {
                mx = (lx + rx) / 2;                // 两边都在 取均值
            } 
            else if (lx != -1) {
                mx = lx + half_width;             // 右侧丢线 该行降级为左边推半宽
            } 
            else if (rx != -1) {
                mx = rx - half_width;             // 左侧丢线 该行降级为右边推半宽
            }
        }
        // 仅记录计算出的有效值，继承与预测在之后的处理中进行
        if (mx != -1) {
            mx = clip(mx, VALID_LEFT_COL, VALID_RIGHT_COL);
            mid.line[y] = mx;
        } 
        else  mid.line[y] = -1;
        
    }

    // 数组滤波
    blur_lines(mid);

    // 继承机制
    int break_y = -1;                               // 记录需要预测处理的位置
    int last_valid_x = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;  // 默认ROI中点
    int blank_count = 0;                            // 连续空洞计数器
    const int MAX_GAP = 3;                          // 继承修补的点数最大值

    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        if (mid.line[y] != -1) {
            last_valid_x = mid.line[y];             
            blank_count = 0;                        
        } 
        else {
            blank_count++;
            if (blank_count <= MAX_GAP) {
                mid.line[y] = last_valid_x;         // 继承处理
            } 
            else {
                break_y = y + MAX_GAP;              
                break;                              // 停止继承，交给之后的最小二乘法去预测
            }
        }
    }

    // 最小二乘法去进行预测
    if (break_y != -1 && break_y >= VALID_END_ROW) {
        int start_y = break_y + 1;                  // 从断处往下方找真实的中点
        int valid_count = 0;
        float sum_y = 0, sum_x = 0, sum_yy = 0, sum_yx = 0;
        
        // 往下抓取最多 10 个真实存在的点进行线性拟合
        for (int y = start_y; y <= VALID_START_ROW; y++) {
            if (mid.line[y] != -1) {
                float fy = (float)y;
                float fx = (float)mid.line[y];
                sum_y += fy; 
                sum_x += fx;
                sum_yy += fy * fy; 
                sum_yx += fy * fx;
                valid_count++;
                if (valid_count >= 10) break; 
            }
        }

        // 拿到 5 个以上的点 计算斜率
        if (valid_count >= 5) {
            float denom = valid_count * sum_yy - sum_y * sum_y;
            if (std::abs(denom) > 1e-6) {
                float a = (valid_count * sum_yx - sum_y * sum_x) / denom; // 斜率
                float b = (sum_x - a * sum_y) / (float)valid_count;       // 截距

                // 顺着拟合出来的斜率，一路往上预测，直到顶部 (ROI_TOP)
                for (int y = break_y; y >= ROI_TOP; y--) {
                    int pred_x = (int)std::lround(a * y + b);
                    mid.line[y] = clip(pred_x, VALID_LEFT_COL, VALID_RIGHT_COL);
                }
            }
        }
    } 
}

//==================================================================================
// 路径搜索 - 上交迷宫巡边线（已屏蔽，改用逐行扫线）
//==================================================================================
#if 0
// 自适应寻线算法，根据种子点位置调用对应的寻线函数 
// 注:减少底层参数调用
void TrackBase::find_line_lefthand_adaptive(std::vector<cv::Point> &pts_out){
    if(seed_left.x <= 0 || seed_left.y <= 0) return;
    pts_out.clear();

    findline_lefthand(this->bin_buf, this->seed_left, &pts_out);
}

void TrackBase::find_line_righthand_adaptive(std::vector<cv::Point> &pts_out){
    if(seed_right.x <= 0 || seed_right.y <= 0) return;
    pts_out.clear();

    findline_righthand(this->bin_buf, this->seed_right, &pts_out);
}

/**
 * 左手迷宫巡线算法 (上交迷宫法)
 * 
 * 从指定起点开始，按照左手定则（优先向左转）在二值化图像中寻找路径，
 * 主要用于路径探索 生成原始点集 raw_pts
 * 
 * @param bin_buf       输入的二值化图像矩阵，白色表示可通行路径(255)，黑色表示障碍物(0)
 * @param start_point   起点坐标 种子点坐标
 * @param out_pts       原始点集 raw_pts，存储路径经过的所有点坐标
 */
void TrackBase::findline_lefthand(cv::Mat &bin_buf, cv::Point start_point, std::vector<cv::Point> *out_pts){
    if (start_point.x < 0 || start_point.y < 0) return;

    if(out_pts != nullptr)
        out_pts->clear();

    int x = start_point.x;
    int y = start_point.y;
    int dir = 0;
    int step = 0;
    int max_steps = ROI_BOTTOM * 3;  // 最大步数，防止死循环

    // 开始搜索
    while (step < max_steps){
        // 旋转搜索方向：左手定则
        int scan_dir = (dir + 2) % 8;  // 从左侧方向开始扫描
        bool moved = false;

        // 探索各个方向
        for (int i = 0; i < 7; i++){
            int check_dir = (scan_dir - i + 8) % 8; // 顺时针方向探索路

            int next_x = x + TrackBase::dir_front[check_dir][0];
            int next_y = y + TrackBase::dir_front[check_dir][1];
            
            // 越界检测
            if(next_x < 2 || next_x >= bin_buf.cols - 2 || next_y < ROI_TOP || next_y >= ROI_BOTTOM){
                continue;
            }
            // 防止向右偏离太远
            if(next_x > (bin_buf.cols / 2 + 5)){
                break;
            }

            // 防止后退 检查之前是否访问过此坐标
            bool visited = false;
            if (out_pts != nullptr && out_pts->size() > 0) {
                for (int k = (int)out_pts->size() - 1; k >= std::max(0, (int)out_pts->size() - 10); k--) {
                    if ((*out_pts)[k].x == next_x && (*out_pts)[k].y == next_y) {
                        visited = true;
                        break;
                    }
                }
            }
            if (visited) continue;

            // 检测新位置是否为路（白)
            if (bin_buf.ptr<uchar>(next_y)[next_x] > now_threshold){
                // 移动到新位置
                x = next_x;
                y = next_y;
                dir = check_dir;  // 更新方向
                moved = true;
                
                if (out_pts != nullptr){
                    out_pts->push_back(cv::Point(x, y)); 
                    
                    // 迷宫法只负责找特征点，不写 line[]
                    // if (y >= 0 && y < MAX_ARRAY_SIZE && left_boundary.line[y] == -1)
                    //     left_boundary.line[y] = x;
                }

                // 到达有效区域边缘
                if (y <= ROI_TOP || x <= 2 || x >= bin_buf.cols - 3)
                    return;

                break;
            }
        }

        // 如果四周都是墙（黑），结束搜线
        if (!moved)         
            break;
        
        step ++;
    }
}

// 右手迷宫巡线 (上交迷宫法)
void TrackBase::findline_righthand(cv::Mat &bin_buf, cv::Point start_point, std::vector<cv::Point> *out_pts){
    if (start_point.x < 0 || start_point.y < 0) return;

    if(out_pts != nullptr)
        out_pts->clear();

    int x = start_point.x;
    int y = start_point.y;
    int dir = 0;
    int step = 0;
    int max_steps = ROI_BOTTOM * 3;  // 最大步数，防止死循环

    // 开始搜索
    while (step < max_steps ){
        // 旋转搜索方向：右手定则
        int scan_dir = (dir - 2 + 8) % 8;  // 从右侧方向开始扫描
        bool moved = false;

        // 探索各个方向
        for (int i = 0; i < 7; i++){
            int check_dir = (scan_dir + i) % 8; // 逆时针方向探索路

            int next_x = x + TrackBase::dir_front[check_dir][0];
            int next_y = y + TrackBase::dir_front[check_dir][1];
            
            // 越界检测限制在 Soft ROI 区域内: 视为黑墙
            if(next_x < 2 || next_x >= bin_buf.cols - 2 || next_y < ROI_TOP || next_y >= ROI_BOTTOM){
                continue;
            }

            // 防止向左偏离太远
            if(next_x < (bin_buf.cols / 2 - 5)){
                break;
            }

            // 防止后退 检查之前是否访问过此坐标
            bool visited = false;
            if (out_pts != nullptr && out_pts->size() > 0) {
                for (int k = (int)out_pts->size() - 1; k >= std::max(0, (int)out_pts->size() - 10); k--) {
                    if ((*out_pts)[k].x == next_x && (*out_pts)[k].y == next_y) {
                        visited = true;
                        break;
                    }
                }
            }
            if (visited) continue;

            // 检测新位置是否为路（白）
            if (bin_buf.ptr<uchar>(next_y)[next_x] > now_threshold){
                // 移动到新位置
                x = next_x;
                y = next_y;
                dir = check_dir;  // 更新方向
                moved = true;
                
                if (out_pts != nullptr){
                    out_pts->push_back(cv::Point(x, y));       
                    
                    // 迷宫法只负责找特征点，不写 line[]
                    // if (y >= 0 && y < MAX_ARRAY_SIZE && right_boundary.line[y] == -1)
                    //     right_boundary.line[y] = x;
                }

                // 到达有效区域边缘
                if (y <= ROI_TOP || x <= 2 || x >= bin_buf.cols - 3)
                    return;

                break;
            }
        }

        // 如果四周都是墙（黑），结束搜线
        if (!moved)         
            break;
        
        step ++;
    }
}

#endif // 迷宫法屏蔽结束


// //==================================================================================
// // 边界线跟踪/平移  (待更新为数组版本)
// //==================================================================================
// /**
//  * @brief 边界线跟踪逻辑：计算偏移点
//  * @param point_now 当前点
//  * @param point_prev 前一个参照点
//  * @param point_next 后一个参照点
//  * @param dist 平移距离
//  * @param is_left_line 是左边线还是右边线 (决定平移方向)
//  * @return cv::Point(now_x, now_y) 平移后的点   
//  */
// cv::Point TrackBase::get_offset_point(const cv::Point &point_now, const cv::Point &point_prev, const cv::Point &point_next, int dist, bool is_left_line) {
//     // 计算切线方向向量 (dx dy)
//     float dx = (float)(point_next.x - point_prev.x); 
//     float dy = (float)(point_next.y - point_prev.y);

//     // 归一化
//     float dn = std::sqrt(dx * dx + dy * dy);  
//     if (dn < 1e-6) return point_now;      
//     dx /= dn;
//     dy /= dn;  

//     // 计算平移后的点坐标 (x, y)
//     // 注:左边线需要往右平移 -> 法向量 (-dy, dx)
//     // 注:右边线需要往左平移 -> 法向量 (dy, -dx)
//     int now_x , now_y;
//     if (is_left_line) {
//         now_x = point_now.x - dy * dist;
//         now_y = point_now.y + dx * dist;
//     }
//     else {
//         now_x = point_now.x + dy * dist;
//         now_y = point_now.y - dx * dist;
//     }

//     return cv::Point(now_x, now_y);
// }

// /**
//  * @brief 跟踪/平移 左边界线 (Left -> Middle)
//  * @param pts_in 输入点集
//  * @param pts_out 输出点集
//  * @param window_size 差分窗口大小 (用于计算切线)
//  * @param dist 平移距离 (通常是半个赛道宽)
//  */
// void TrackBase::track_leftline(const std::vector<cv::Point> &pts_in, std::vector<cv::Point> &pts_out,int window_size, int dist) {
//     // 初始化输出点集，并做边界检查
//     pts_out.clear();
//     if (dist < 0 || pts_in.size() < 2)   return;
//     if (window_size < 1) window_size = 1;
    
//     int len = (int)pts_in.size();
//     pts_out.reserve(len);

//     // [增强] 对平移后的中线点做边界 clip，并限制横向最大跳变，降低噪点导致的横跳
//     cv::Point last_out(-1, -1);
//     const int max_step = 8; // 单位:像素，可按赛道宽/噪声调整

//     for (int i = 0; i < len; i++) {
//     int idx_plus = clip(i + window_size, 0, len - 1);
//     int idx_minus = clip(i - window_size, 0, len - 1);
        
//     cv::Point offsef_pt = get_offset_point(pts_in[i], pts_in[idx_minus], pts_in[idx_plus], dist, true);
//     offsef_pt.x = clip(offsef_pt.x, 0, CAM_WIDTH - 1);
//     offsef_pt.y = clip(offsef_pt.y, 0, CAM_HEIGHT - 1);

//         if (last_out.x >= 0) {
//             int dx = offsef_pt.x - last_out.x; 
//             if (dx > max_step) offsef_pt.x = last_out.x + max_step;
//             else if (dx < -max_step) offsef_pt.x = last_out.x - max_step;
//         }

//         pts_out.push_back(offsef_pt);
//         last_out = offsef_pt;
//     }
// }

// /**
//  * @brief 跟踪/平移 右边界线 (Right -> Middle)
//  * @param pts_in 输入点集
//  * @param pts_out 输出点集
//  * @param window_size 差分窗口大小 (用于计算切线)
//  * @param dist 平移距离 (通常是半个赛道宽)
//  */
// void TrackBase::track_rightline(const std::vector<cv::Point> &pts_in, std::vector<cv::Point> &pts_out,int window_size, int dist) {
//     pts_out.clear();
//     if (dist < 0 || pts_in.size() < 2)   return;
//     if (window_size < 1) window_size = 1;
    
//     int len = (int)pts_in.size();
//     pts_out.reserve(len);

//     cv::Point last_out(-1, -1);
//     const int max_step = 12;

//     for (int i = 0; i < len; i++) {
//     int idx_plus = clip(i + window_size, 0, len - 1);
//     int idx_minus = clip(i - window_size, 0, len - 1);
        
//     cv::Point offsef_pt = get_offset_point(pts_in[i], pts_in[idx_minus], pts_in[idx_plus], dist, false);
//     offsef_pt.x = clip(offsef_pt.x, 0, CAM_WIDTH - 1);
//     offsef_pt.y = clip(offsef_pt.y, 0, CAM_HEIGHT - 1);

//         if (last_out.x >= 0) {
//             int dx = offsef_pt.x - last_out.x;
//             if (dx > max_step) offsef_pt.x = last_out.x + max_step;
//             else if (dx < -max_step) offsef_pt.x = last_out.x - max_step;
//         }

//         pts_out.push_back(offsef_pt);
//         last_out = offsef_pt;
//     }
// }

// /**
//  * @brief 跟踪/平移 单个点 
//  * @param pts_in 输入所在的整条边线 (我们需要整条线来计算切线方向)
//  * @param target_idx 要平移的那个点在输入点集 pts_in 中的索引 ID
//  * @param window_size 窗口大小
//  * @param dist 平移距离
//  * @param is_left 是否是左边线
//  * @return cv::Point 平移后的目标点
//  */
// cv::Point TrackBase::track_point(const std::vector<cv::Point> &pts_in, int target_idx, int window_size, int dist, bool is_left) {
//     if (pts_in.size() < 2)     return cv::Point(0, 0); 
    
//     int len = pts_in.size();
//     target_idx = clip(target_idx, 0, len - 1); 

//     int idx_prev = clip(target_idx - window_size, 0, len - 1);
//     int idx_next = clip(target_idx + window_size, 0, len - 1);

//     return get_offset_point(pts_in[target_idx], pts_in[idx_prev], pts_in[idx_next], dist, is_left);
// }

//==================================================================================
// 滤波处理函数
//==================================================================================
/**
 * @brief 自定义中值滤波
 * @param image 输入的灰度图
 * @param kernel_size 滤波核大小 正奇数 
 * @param enable_blur 开关：true开启滤波，false跳过处理
 * @note  在二值化处理之前应用于全图
 */
void TrackBase::blur_median(cv::Mat& image, int kernel_size, bool enable_blur) {
    if (!enable_blur || image.empty() || image.channels() != 1 || kernel_size < 3 || kernel_size % 2 == 0) 
        return;

    cv::Mat dst = image.clone();
    int r = kernel_size / 2;
    int rows = image.rows;
    int cols = image.cols;

    // 手写 3x3 快速选择中值优化
    if (kernel_size == 3) {
        for (int i = 1; i < rows - 1; i++) {
            const uchar* prev = image.ptr<uchar>(i - 1);
            const uchar* curr = image.ptr<uchar>(i);
            const uchar* next = image.ptr<uchar>(i + 1);
            uchar* out = dst.ptr<uchar>(i);

            for (int j = 1; j < cols - 1; j++) {
                uchar window[9] = {
                    prev[j-1], prev[j], prev[j+1],
                    curr[j-1], curr[j], curr[j+1],
                    next[j-1], next[j], next[j+1]
                };
                
                // 仅为了寻找中位数(第5大)进行部分排序
                std::nth_element(window, window + 4, window + 9);
                out[j] = window[4];
            }
        }
    } else {
        // 更大核的泛用实现
        std::vector<uchar> window(kernel_size * kernel_size);
        for (int i = r; i < rows - r; i++) {
            for (int j = r; j < cols - r; j++) {
                int idx = 0;
                for (int ki = -r; ki <= r; ki++) {
                    const uchar* row_ptr = image.ptr<uchar>(i + ki);
                    for (int kj = -r; kj <= r; kj++) {
                        window[idx++] = row_ptr[j + kj];
                    }
                }
                std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
                dst.at<uchar>(i, j) = window[window.size() / 2];
            }
        }
    }
    image = dst; // 结果写回原图
}

/**
 * @brief 对一维数组进行中值滤波 
 * @param mid 一维数组 mid.line
 */
void TrackBase::blur_lines(PointState::MidlineData& mid) {
    int temp[MAX_ARRAY_SIZE];
    for(int i = 0; i < MAX_ARRAY_SIZE; i++) temp[i] = mid.line[i]; // 备份

    for (int y = VALID_END_ROW + 1; y < VALID_START_ROW; y++) {
        if(temp[y-1] != -1 && temp[y] != -1 && temp[y+1] != -1) {
            // 找出连续三个点中的中位数，消除毛刺尖峰
            int window[3] = {temp[y-1], temp[y], temp[y+1]};
            if (window[0] > window[1]) std::swap(window[0], window[1]);
            if (window[1] > window[2]) std::swap(window[1], window[2]);
            if (window[0] > window[1]) std::swap(window[0], window[1]);
            
            mid.line[y] = window[1]; // 将中位数填回原数组
        }
    }
}

/**
 * @brief 对点集进行模糊滤波 (加权平均滤波)
 * @param now_pts 输入点集   now_pts 
 * @param kernel_size 滤波核大小  必须是正奇数 以保证对称性
 */
void TrackBase::blur_points(std::vector<cv::Point> &now_pts, int kernel_size) {
    // 核大小必须为奇数且小于点集大小
    if (now_pts.size() <= (size_t)kernel_size || kernel_size % 2 == 0) 
        return; 

    int half_kernel = kernel_size / 2;  
    std::vector<cv::Point> smoothed;
    smoothed.reserve(now_pts.size());      // 预分配空间

    // 保留起始段 
    for (size_t i = 0; i < half_kernel; i++) {
        smoothed.push_back(now_pts[i]);    
    }

    // 遍历可处理的中心点 i
    for (size_t i = half_kernel; i < now_pts.size() - half_kernel; i++) {
        int sum_x = 0, sum_y = 0;
        int weight_count = 0;   // 权重计数，此处简化处理为核大小

        // 遍历核内邻域点 k      
        for (int k = -half_kernel; k <= half_kernel; k++) {
            int weight = half_kernel + 1 - std::abs(k);      // 结合权重 计算
            
            sum_x += now_pts[i + k].x * weight;
            sum_y += now_pts[i + k].y * weight;
            weight_count += weight;
        }
        smoothed.push_back(cv::Point(sum_x / weight_count, sum_y / weight_count));
    }
    // 保留终止段
    for (size_t i = now_pts.size() - half_kernel; i < now_pts.size(); i++) {
        smoothed.push_back(now_pts[i]);    
    }
    // 更新点集数据
    now_pts = std::move(smoothed);
}


// ==================================================================================

/**
 * @brief 在两点之间生成直线点集 (直线插值)
 * @param start 直线起点
 * @param end 直线终点
 * @param pts 输出容器 (新生成的点会 append 到这里)
 * @param dist 添加点之间的距离
 */
void TrackBase::add_point_by_line(const cv::Point &start, const cv::Point &end, std::vector<cv::Point> &pts, int dist) {
    if (dist <= 0)      return;
    
    // 计算直线参数     
    float dx = (float)(end.x - start.x); 
    float dy = (float)(end.y - start.y); 
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6)     return;

    // 归一化方向向量  即把向量长度变1
    float unit_x = dx / len;                    
    float unit_y = dy / len;                    

    // 已添加点数，剩余距离
    float now_len = (float)dist;
    while (now_len <= len) {
        // 计算下一个点的x y坐标
        cv::Point new_point;
        new_point.x = cv::saturate_cast<int>(start.x + unit_x * now_len);
        new_point.y = cv::saturate_cast<int>(start.y + unit_y * now_len);     
        
        // 边界检查
        if(new_point.x >= 0 && new_point.x < CAM_WIDTH &&
            new_point.y >= 0 && new_point.y < CAM_HEIGHT){
            pts.push_back(new_point);          // 添加点
        }
        else    break;

        now_len += (float)dist;               // 更新距离
    }
}

/**
 * @brief 根据三点定义的二阶贝塞尔曲线添加点 (贝塞尔曲线插值)
 * @param p0 起点
 * @param p1 控制点 (弯曲方向的吸引点)
 * @param p2 终点
 * @param pts 输出容器 (append模式)
 * @param dist 近似点间距
 */
void TrackBase::add_three_point_bezier(const cv::Point &p0, const cv::Point &p1, const cv::Point &p2,
                                        std::vector<cv::Point> &pts,int dist)
{
    if (dist <= 0) return;
    
    // 估算曲线长度 (两段弦长之和 * 修正系数，不够精确但够用)
    float d1 = cv::norm(p1 - p0);
    float d2 = cv::norm(p2 - p1);
    float curve_len = (d1 + d2) * 0.9f;

    if (curve_len < dist) return;

    // 计算采样步数 dt
    int step = (int)(curve_len / dist);
    if (step < 2) step = 2;     // 最少两步

    float dt = 1.0f / (float)step;

    // 贝塞尔曲线采样
    for(int i = 0; i <= step; i++){
        float t = (float)i * dt;
        if(t > 1.0f)    t = 1.0f;

        float t_inv = 1.0f - t;

        // 二阶贝塞尔公式: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
        float a = t_inv * t_inv;
        float b = 2 * t_inv * t;
        float c = t * t;

        float x = a * p0.x + b * p1.x + c * p2.x;
        float y = a * p0.y + b * p1.y + c * p2.y;

        // 计算贝塞尔曲线上的点坐标
        cv::Point new_point(cv::saturate_cast<int>(x), cv::saturate_cast<int>(y));

        // 边界检查
        if(new_point.x >= 0 && new_point.x < CAM_WIDTH &&
            new_point.y >= 0 && new_point.y < CAM_HEIGHT){
            pts.push_back(new_point);          // 添加点
        }
        else    break;
    }
}

//==================================================================================
// 种子点检测 (最长白列)
//==================================================================================
/**
 * @brief 使用最长白列来获取种子点并更新状态
 * @param bin_buf 输入的二值化图像
 */
void TrackBase::seek_pts_seed(const cv::Mat &bin_buf) {

    // 从画面底部向上扫描二值图，寻找最长白列并定位左右种子点
    white_column.lineNum = 0;
    white_column.maxNum = 0;
    white_column.endlineX = 0;
    white_column.maxy = VALID_START_ROW;

    // 从标定区域底部开始扫描
    int y_start = VALID_START_ROW;
    const uchar *ptr_ystart = bin_buf.ptr<uchar>(y_start);

    // 遍历所有列，寻找最长白列
    for (int col = 0; col < bin_buf.cols; col++) {
        if (ptr_ystart[col] > now_threshold) {
            int now_row = y_start;
            int col_len = 1;

            // 向上统计白点数量 (不要超过 ROI_TOP)
            while (now_row >= ROI_TOP && bin_buf.ptr<uchar>(now_row)[col] > now_threshold) {
                col_len++;
                now_row--;
            }

            // 更新最长白列信息
            if (col_len > white_column.maxNum) {
                white_column.lineNum = 1;
                white_column.maxNum = col_len;
                white_column.maxy = now_row + 1;
                white_column.starlineX = col;
                white_column.endlineX = col;
            }
            else if (col_len == white_column.maxNum) {  // 并列最长，只记录数量
                white_column.lineNum++;
                white_column.endlineX = col;
            }
        }
    }

    // 串道补充逻辑：处理最长白列几乎贯穿全区的情况
    int max_roi_len = ROI_BOTTOM - ROI_TOP;
    if(white_column.maxNum >= max_roi_len - 1 ||
       white_column.maxNum >= max_roi_len - 2) {

        // 白列宽度大于数量，说明是分散的多条白线
        if(white_column.endlineX - white_column.starlineX + 1 > white_column.lineNum) {
            const uchar *ptr_maxy = bin_buf.ptr<uchar>(white_column.maxy);

            // 向右统计左侧连续白列数
            int l_conts = 0, start_x = white_column.starlineX + 1;
            while(start_x < bin_buf.cols - 1 && ptr_maxy[start_x] > now_threshold) {
                int y = white_column.maxy;
                int conts = 1;
                while(y < VALID_START_ROW && bin_buf.ptr<uchar>(y)[start_x] > now_threshold) {
                    y++; conts++;
                }
                if(conts >= white_column.maxNum) l_conts++;
                else        break;
                start_x++;
            }

            // 向左统计右侧连续白列数
            int r_conts = 0, end_x = white_column.endlineX - 1;
            while(end_x > 0 && ptr_maxy[end_x] > now_threshold) {
                int y = white_column.maxy;
                int conts = 1;
                while(y < VALID_START_ROW && bin_buf.ptr<uchar>(y)[end_x] > now_threshold) {
                    y++; conts++;
                }
                if(conts >= white_column.maxNum) r_conts++;
                else        break;
                end_x--;
            }

            // 情景一：左右连续均短，向下偏移修正
            if (l_conts <= 10 && r_conts <= 10) {
                int offset = 5;
                if(white_column.maxy + offset >= ROI_BOTTOM) offset = 0;
                const uchar *ptr_maxy_off = bin_buf.ptr<uchar>(white_column.maxy + offset);

                // 重新统计左侧连续数
                int two_l_conts = 0; start_x = white_column.starlineX + 1;
                while(start_x < bin_buf.cols - 1 && ptr_maxy_off[start_x] > now_threshold) {
                    int y = white_column.maxy + offset;
                    int conts = 1;
                    while(y < VALID_START_ROW && bin_buf.ptr<uchar>(y)[start_x] > now_threshold) {
                        y++; conts++;
                    }
                    if(conts >= white_column.maxNum - offset) two_l_conts++;
                    else break;
                    start_x++;
                }

                // 重新统计右侧连续数
                int two_r_conts = 0; end_x = white_column.endlineX - 1;
                while(end_x > 0 && ptr_maxy_off[end_x] > now_threshold) {
                    int y = white_column.maxy + offset;
                    int conts = 1;
                    while(y < VALID_START_ROW && bin_buf.ptr<uchar>(y)[end_x] > now_threshold) {
                        y++; conts++;
                    }
                    if(conts >= white_column.maxNum - offset) two_r_conts++;
                    else break;
                    end_x--;
                }

                // 选择较长侧更新白列信息
                if(two_l_conts >= two_r_conts) {
                    white_column.endlineX = white_column.starlineX + two_l_conts;
                    white_column.lineNum = two_l_conts;
                    white_column.maxy += offset;
                    white_column.maxNum -= offset;
                }
                else {
                    white_column.starlineX = white_column.endlineX - two_r_conts;
                    white_column.lineNum = two_r_conts;
                    white_column.maxy += offset;
                    white_column.maxNum -= offset;
                }
            }
            // 情景二：左右连续差异大，取较长侧
            else if (l_conts >= r_conts) {
                white_column.endlineX = white_column.starlineX + l_conts;
                white_column.lineNum = l_conts;
            }
            else if (l_conts < r_conts) {
                white_column.starlineX = white_column.endlineX - r_conts;
                white_column.lineNum = r_conts;
            }
        }
    }

    // 初始化种子点状态
    uint8_t result_bits = static_cast<uint8_t>(no_seed);
    seed_left = cv::Point(-1, VALID_START_ROW);
    seed_right = cv::Point(bin_buf.cols, VALID_START_ROW);

    // 以最长白列中心为基准
    int center_x = (white_column.starlineX + white_column.endlineX) / 2;

    // 中心点越界保护
    if (center_x < 5) center_x = 5;
    if (center_x > bin_buf.cols - 5) center_x = bin_buf.cols - 5;

    bool found_left = false;
    bool found_right = false;

    // 从标定区域底部向上探测一直到 ROI_TOP
    int start_y = VALID_START_ROW;

    for (int y = start_y; y >= ROI_TOP; y--) {
        const uchar* ptr_y = bin_buf.ptr<uchar>(y);

        // 寻找左种子点  0001跳变点
        if (!found_left) {
            int x = center_x;
            // 从白列通道往左边摸索，直到碰到黑线 (<= now_threshold)
            while (x > 2 && ptr_y[x] > now_threshold) x--; 
            
            // 如果撞到的不是咱们强行画的物理黑边(x <= 1)，说明摸到了真正的赛道黑边！
            if (x > 2) {
                seed_left = cv::Point(x + 1, y);  // 往右退一格就是白点，也就是边界线的起点
                result_bits |= static_cast<uint8_t>(left_pts);
                found_left = true;
            }
        }

        // --- 寻找右种子点 ---
        if (!found_right) {
            int x = center_x;
            // 从白列通道往右边摸索，直到碰到黑线
            while (x < bin_buf.cols - 3 && ptr_y[x] > now_threshold) x++; 
            
            // 如果撞到的不是物理黑边(x >= cols - 2)，说明摸到了真正的赛道黑边！
            if (x < bin_buf.cols - 3) {
                seed_right = cv::Point(x - 1, y); // 往左退一格是白点
                result_bits |= static_cast<uint8_t>(right_pts);
                found_right = true;
            }
        }

        // 如果左右都已经找到了真正的起爬黑点，完美！直接退出循环
        if (found_left && found_right) {
            break;
        }
    }

    // 种子点状态
    seed_state = static_cast<SeedState>(result_bits);
}



//==================================================================================
// 八邻域边界提取方法
//==================================================================================

// 八邻域静态数据（避免频繁分配）
static uint16_t points_l[USE_num][2] = {{0}};  // 左线轨迹点
static uint16_t points_r[USE_num][2] = {{0}};  // 右线轨迹点
static uint16_t dir_l[USE_num] = {0};          // 左边生长方向
static uint16_t dir_r[USE_num] = {0};          // 右边生长方向

/**
 * @brief 从图像底部中间寻找左右起点
 * @param bin_buf 二值化图像
 * @param start_row 起始行号
 * @param start_point_l 输出左起点 [x, y]
 * @param start_point_r 输出右起点 [x, y]
 * @return true=找到起点, false=未找到
 */
static bool get_start_point_8n(const cv::Mat& bin_buf, uint8_t start_row, 
                                uint8_t start_point_l[2], uint8_t start_point_r[2])
{
    bool l_found = false, r_found = false;
    
    // 清零
    start_point_l[0] = 0; start_point_l[1] = 0;
    start_point_r[0] = 0; start_point_r[1] = 0;
    
    const uchar* row = bin_buf.ptr<uchar>(start_row);
    
    // 从中间往左找白→黑跳变
    for (int i = IMG_W / 2; i > border_min; i--) {
        if (row[i] == 255 && row[i - 1] == 0) {
            start_point_l[0] = i;
            start_point_l[1] = start_row;
            l_found = true;
            break;
        }
    }
    
    // 从中间往右找白→黑跳变
    for (int i = IMG_W / 2; i < border_max; i++) {
        if (row[i] == 255 && row[i + 1] == 0) {
            start_point_r[0] = i;
            start_point_r[1] = start_row;
            r_found = true;
            break;
        }
    }
    
    return (l_found && r_found);
}

/**
 * @brief 八邻域搜索左右边界
 * @param bin_buf 二值化图像
 * @param l_stastic 输出左边找到的点数
 * @param r_stastic 输出右边找到的点数
 * @param l_start_x 左起点x
 * @param l_start_y 左起点y
 * @param r_start_x 右起点x
 * @param r_start_y 右起点y
 * @param hightest 输出最高点y坐标
 */
static void search_l_r_8n(const cv::Mat& bin_buf, uint16_t* l_stastic, uint16_t* r_stastic,
                          uint8_t l_start_x, uint8_t l_start_y, 
                          uint8_t r_start_x, uint8_t r_start_y, uint8_t* hightest)
{
    uint16_t break_flag = USE_num;
    
    // 左边八邻域种子（顺时针）
    static int8_t seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
        {0, -1}, {1, -1}, {1, 0}, {1, 1}
    };
    
    // 右边八邻域种子（逆时针）
    static int8_t seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };
    
    uint8_t search_filds_l[8][2] = {{0}};
    uint8_t search_filds_r[8][2] = {{0}};
    uint8_t center_point_l[2] = {l_start_x, l_start_y};
    uint8_t center_point_r[2] = {r_start_x, r_start_y};
    uint8_t temp_l[8][2] = {{0}};
    uint8_t temp_r[8][2] = {{0}};
    uint8_t index_l = 0, index_r = 0;
    
    uint16_t l_data_statics = *l_stastic;
    uint16_t r_data_statics = *r_stastic;
    
    // 开启邻域循环
    while (break_flag--) {
        // 左边：生成8邻域坐标
        for (int i = 0; i < 8; i++) {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }
        
        // 记录当前中心点
        points_l[l_data_statics][0] = center_point_l[0];
        points_l[l_data_statics][1] = center_point_l[1];
        l_data_statics++;
        
        // 右边：生成8邻域坐标
        for (int i = 0; i < 8; i++) {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }
        
        // 记录当前中心点
        points_r[r_data_statics][0] = center_point_r[0];
        points_r[r_data_statics][1] = center_point_r[1];
        
        // 左边：寻找黑→白跳变点
        index_l = 0;
        for (int i = 0; i < 8; i++) {
            temp_l[i][0] = 0;
            temp_l[i][1] = 0;
        }
        
        for (int i = 0; i < 8; i++) {
            int y1 = search_filds_l[i][1];
            int x1 = search_filds_l[i][0];
            int y2 = search_filds_l[(i + 1) & 7][1];
            int x2 = search_filds_l[(i + 1) & 7][0];
            
            // 边界检查
            if (y1 < 0 || y1 >= IMG_H || x1 < 0 || x1 >= IMG_W) continue;
            if (y2 < 0 || y2 >= IMG_H || x2 < 0 || x2 >= IMG_W) continue;
            
            if (bin_buf.at<uchar>(y1, x1) == 0 && bin_buf.at<uchar>(y2, x2) == 255) {
                temp_l[index_l][0] = search_filds_l[i][0];
                temp_l[index_l][1] = search_filds_l[i][1];
                index_l++;
                dir_l[l_data_statics - 1] = i;
            }
            
            if (index_l) {
                // 更新中心点（选择y最小的点，即最靠近图像顶部的点）
                center_point_l[0] = temp_l[0][0];
                center_point_l[1] = temp_l[0][1];
                for (int j = 0; j < index_l; j++) {
                    if (center_point_l[1] > temp_l[j][1]) {
                        center_point_l[0] = temp_l[j][0];
                        center_point_l[1] = temp_l[j][1];
                    }
                }
            }
        }
        
        // 退出条件1：三次进入同一个点
        if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] && 
             points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
            (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
             points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
             points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
             points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1])) {
            break;
        }
        
        // 退出条件2：左右相遇
        if (std::abs((int)points_r[r_data_statics][0] - (int)points_l[l_data_statics - 1][0]) < 2 &&
            std::abs((int)points_r[r_data_statics][1] - (int)points_l[l_data_statics - 1][1]) < 2) {
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;
            break;
        }
        
        // 左边等待右边
        if (points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]) {
            continue;
        }
        
        // 左边向下生长时等待右边
        if (dir_l[l_data_statics - 1] == 7 && 
            points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]) {
            center_point_l[0] = points_l[l_data_statics - 1][0];
            center_point_l[1] = points_l[l_data_statics - 1][1];
            l_data_statics--;
        }
        
        r_data_statics++;
        
        // 右边：寻找黑→白跳变点
        index_r = 0;
        for (int i = 0; i < 8; i++) {
            temp_r[i][0] = 0;
            temp_r[i][1] = 0;
        }
        
        for (int i = 0; i < 8; i++) {
            int y1 = search_filds_r[i][1];
            int x1 = search_filds_r[i][0];
            int y2 = search_filds_r[(i + 1) & 7][1];
            int x2 = search_filds_r[(i + 1) & 7][0];
            
            // 边界检查
            if (y1 < 0 || y1 >= IMG_H || x1 < 0 || x1 >= IMG_W) continue;
            if (y2 < 0 || y2 >= IMG_H || x2 < 0 || x2 >= IMG_W) continue;
            
            if (bin_buf.at<uchar>(y1, x1) == 0 && bin_buf.at<uchar>(y2, x2) == 255) {
                temp_r[index_r][0] = search_filds_r[i][0];
                temp_r[index_r][1] = search_filds_r[i][1];
                index_r++;
                dir_r[r_data_statics - 1] = i;
            }
            
            if (index_r) {
                // 更新中心点（选择y最小的点）
                center_point_r[0] = temp_r[0][0];
                center_point_r[1] = temp_r[0][1];
                for (int j = 0; j < index_r; j++) {
                    if (center_point_r[1] > temp_r[j][1]) {
                        center_point_r[0] = temp_r[j][0];
                        center_point_r[1] = temp_r[j][1];
                    }
                }
            }
        }
    }
    
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/**
 * @brief 从八邻域轨迹点提取左边线
 * @param total_L 左边轨迹点总数
 * @param left_data 左边界数据引用
 */
static void get_left_8n(uint16_t total_L, PointState::BoundaryData& left_data)
{
    // 初始化
    for (int i = 0; i < IMG_H; i++) {
        left_data.line[i] = -1;
        left_data.scan_line[i] = -1;
    }
    
    int filled_count = 0;
    
    // 直接遍历所有追踪到的点，填充到 line[] 数组
    for (uint16_t j = 0; j < total_L; j++) {
        int x = points_l[j][0];
        int y = points_l[j][1];
        
        // 边界检查
        if (y >= 0 && y < IMG_H && x >= border_min && x <= border_max) {
            // 如果该行还没有填充过，或者新点更靠左（更接近真实边界）
            if (left_data.line[y] == -1) {
                left_data.line[y] = x;
                left_data.scan_line[y] = x;
                left_data.raw_pts.push_back(cv::Point(x, y));
                filled_count++;
            }
        }
    }
}

/**
 * @brief 从八邻域轨迹点提取右边线
 * @param total_R 右边轨迹点总数
 * @param right_data 右边界数据引用
 */
static void get_right_8n(uint16_t total_R, PointState::BoundaryData& right_data)
{
    // 初始化
    for (int i = 0; i < IMG_H; i++) {
        right_data.line[i] = -1;
        right_data.scan_line[i] = -1;
    }
    
    int filled_count = 0;
    
    // 直接遍历所有追踪到的点，填充到 line[] 数组
    for (uint16_t j = 0; j < total_R; j++) {
        int x = points_r[j][0];
        int y = points_r[j][1];
        
        // 边界检查
        if (y >= 0 && y < IMG_H && x >= border_min && x <= border_max) {
            // 如果该行还没有填充过，或者新点更靠右（更接近真实边界）
            if (right_data.line[y] == -1) {
                right_data.line[y] = x;
                right_data.scan_line[y] = x;
                right_data.raw_pts.push_back(cv::Point(x, y));
                filled_count++;
            }
        }
    }
}

/**
 * @brief 八邻域边界提取主函数
 * @param bin_buf 二值化图像
 * @param left_data 左边界数据引用
 * @param right_data 右边界数据引用
 * @return true=成功提取, false=未找到起点
 */
bool PointState::extract_boundary_8neighbor(const cv::Mat& bin_buf, 
                                            BoundaryData& left_data, 
                                            BoundaryData& right_data)
{
    // 清空原有数据
    left_data.raw_pts.clear();
    right_data.raw_pts.clear();
    
    // 寻找起点
    uint8_t start_point_l[2] = {0};
    uint8_t start_point_r[2] = {0};
    bool found_start = false;
    
    // 获取最长白列信息
    extern TrackBase track_base;
    
    // 策略：优先使用最长白列法找到的位置（赛道实际存在的地方）
    if (track_base.white_column.maxNum > 5) {
        uint8_t start_row = VALID_START_ROW;  // 最长白列的底部
        
        // 确保起点在有效范围内
        if (start_row >= VALID_END_ROW && start_row <= VALID_START_ROW) {
            found_start = get_start_point_8n(bin_buf, start_row, start_point_l, start_point_r);
        }
    }
    
    // 备选：如果最长白列法失败，尝试从有效区域底部开始找起点
    if (!found_start) {
        uint8_t start_row = VALID_START_ROW;
        found_start = get_start_point_8n(bin_buf, start_row, start_point_l, start_point_r);
    }
    
    // 如果仍然找不到起点，返回失败
    if (!found_start) {
        return false;
    }
    
    // 左边八邻域方向偏移
    // 方向编号：0=下, 1=左下, 2=左, 3=左上, 4=上, 5=右上, 6=右, 7=右下
    static int8_t seeds_l[8][2] = {
        {0, 1},   // 0: 下
        {-1, 1},  // 1: 左下
        {-1, 0},  // 2: 左
        {-1, -1}, // 3: 左上
        {0, -1},  // 4: 上
        {1, -1},  // 5: 右上
        {1, 0},   // 6: 右
        {1, 1}    // 7: 右下
    };
    
    // 右边八邻域方向偏移
    static int8_t seeds_r[8][2] = {
        {0, 1},   // 0: 下
        {1, 1},   // 1: 右下
        {1, 0},   // 2: 右
        {1, -1},  // 3: 右上
        {0, -1},  // 4: 上
        {-1, -1}, // 5: 左上
        {-1, 0},  // 6: 左
        {-1, 1}   // 7: 左下
    };
    
    // 初始化边界数组
    for (int i = 0; i < IMG_H; i++) {
        left_data.line[i] = -1;
        right_data.line[i] = -1;
        left_data.scan_line[i] = -1;
        right_data.scan_line[i] = -1;
    }
    
    // 清空方向序列
    left_data.dir_count = 0;
    right_data.dir_count = 0;
    left_data.dir_valid = 0;
    right_data.dir_valid = 0;
    
    // ========== 左边界追踪 ==========
    int curr_x = start_point_l[0];
    int curr_y = start_point_l[1];
    int step_count = 0;
    const int MAX_STEPS = 200;
    
    // 记录起点
    if (curr_y >= 0 && curr_y < IMG_H) {
        left_data.line[curr_y] = curr_x;
        left_data.scan_line[curr_y] = curr_x;
        left_data.raw_pts.push_back(cv::Point(curr_x, curr_y));
    }
    
    // 左边界向上追踪
    while (step_count < MAX_STEPS && curr_y > VALID_END_ROW) {
        bool found = false;
        int best_x = -1, best_y = -1;
        int best_dir = -1;
        
        // 优先向上搜索：4(上) > 3(左上) > 5(右上) > 2(左) > 6(右) > 1(左下) > 7(右下) > 0(下)
        int search_order[8] = {4, 3, 5, 2, 6, 1, 7, 0};
        
        for (int idx = 0; idx < 8; idx++) {
            int dir = search_order[idx];
            int next_x = curr_x + seeds_l[dir][0];
            int next_y = curr_y + seeds_l[dir][1];
            
            // 边界检查
            if (next_y < VALID_END_ROW || next_y >= IMG_H) continue;
            if (next_x <= border_min || next_x >= border_max) continue;
            
            // 改进的边界检测：寻找白→黑跳变
            // 方法1：当前点是白色，左侧是黑色
            bool is_boundary = false;
            if (bin_buf.at<uchar>(next_y, next_x) == 255 && 
                bin_buf.at<uchar>(next_y, next_x - 1) == 0) {
                is_boundary = true;
            }
            // 方法2：当前点是黑色，右侧是白色（也是边界）
            else if (bin_buf.at<uchar>(next_y, next_x) == 0 && 
                     next_x < border_max - 1 &&
                     bin_buf.at<uchar>(next_y, next_x + 1) == 255) {
                is_boundary = true;
                next_x++;  // 使用白色侧的点
            }
            
            if (is_boundary) {
                best_x = next_x;
                best_y = next_y;
                best_dir = dir;
                found = true;
                break;  // 找到第一个就停止
            }
        }
        
        if (!found) {
            // 如果8邻域都找不到，尝试放宽条件：只要是白色点就行
            for (int idx = 0; idx < 8; idx++) {
                int dir = search_order[idx];
                int next_x = curr_x + seeds_l[dir][0];
                int next_y = curr_y + seeds_l[dir][1];
                
                if (next_y < VALID_END_ROW || next_y >= IMG_H) continue;
                if (next_x <= border_min || next_x >= border_max) continue;
                
                if (bin_buf.at<uchar>(next_y, next_x) == 255) {
                    best_x = next_x;
                    best_y = next_y;
                    best_dir = dir;
                    found = true;
                    break;
                }
            }
        }
        
        if (!found) break;  // 完全找不到，停止追踪
        
        // 更新当前点
        curr_x = best_x;
        curr_y = best_y;
        
        // 填充边界数组（只填充一次，避免重复）
        if (curr_y >= 0 && curr_y < IMG_H && left_data.line[curr_y] == -1) {
            left_data.line[curr_y] = curr_x;
            left_data.scan_line[curr_y] = curr_x;
            left_data.raw_pts.push_back(cv::Point(curr_x, curr_y));
            
            // 记录方向序列
            if (left_data.dir_count < USE_num) {
                left_data.dir_sequence[left_data.dir_count] = best_dir;
                left_data.trajectory_points[left_data.dir_count][0] = curr_x;
                left_data.trajectory_points[left_data.dir_count][1] = curr_y;
                left_data.dir_count++;
            }
        }
        
        step_count++;
    }
    
    left_data.dir_valid = (left_data.dir_count > 10) ? 1 : 0;
    
    // ========== 右边界追踪 ==========
    curr_x = start_point_r[0];
    curr_y = start_point_r[1];
    step_count = 0;
    
    // 记录起点
    if (curr_y >= 0 && curr_y < IMG_H) {
        right_data.line[curr_y] = curr_x;
        right_data.scan_line[curr_y] = curr_x;
        right_data.raw_pts.push_back(cv::Point(curr_x, curr_y));
    }
    
    // 右边界向上追踪
    while (step_count < MAX_STEPS && curr_y > VALID_END_ROW) {
        bool found = false;
        int best_x = -1, best_y = -1;
        int best_dir = -1;
        
        // 优先向上搜索：4(上) > 3(右上) > 5(左上) > 2(右) > 6(左) > 1(右下) > 7(左下) > 0(下)
        int search_order[8] = {4, 3, 5, 2, 6, 1, 7, 0};
        
        for (int idx = 0; idx < 8; idx++) {
            int dir = search_order[idx];
            int next_x = curr_x + seeds_r[dir][0];
            int next_y = curr_y + seeds_r[dir][1];
            
            // 边界检查
            if (next_y < VALID_END_ROW || next_y >= IMG_H) continue;
            if (next_x <= border_min || next_x >= border_max) continue;
            
            // 改进的边界检测：寻找白→黑跳变
            // 方法1：当前点是白色，右侧是黑色
            bool is_boundary = false;
            if (bin_buf.at<uchar>(next_y, next_x) == 255 && 
                next_x < IMG_W - 1 &&
                bin_buf.at<uchar>(next_y, next_x + 1) == 0) {
                is_boundary = true;
            }
            // 方法2：当前点是黑色，左侧是白色（也是边界）
            else if (bin_buf.at<uchar>(next_y, next_x) == 0 && 
                     next_x > 0 &&
                     bin_buf.at<uchar>(next_y, next_x - 1) == 255) {
                is_boundary = true;
                next_x--;  // 使用白色侧的点
            }
            
            if (is_boundary) {
                best_x = next_x;
                best_y = next_y;
                best_dir = dir;
                found = true;
                break;
            }
        }
        
        if (!found) {
            // 如果8邻域都找不到，尝试放宽条件：只要是白色点就行
            for (int idx = 0; idx < 8; idx++) {
                int dir = search_order[idx];
                int next_x = curr_x + seeds_r[dir][0];
                int next_y = curr_y + seeds_r[dir][1];
                
                if (next_y < VALID_END_ROW || next_y >= IMG_H) continue;
                if (next_x <= border_min || next_x >= border_max) continue;
                
                if (bin_buf.at<uchar>(next_y, next_x) == 255) {
                    best_x = next_x;
                    best_y = next_y;
                    best_dir = dir;
                    found = true;
                    break;
                }
            }
        }
        
        if (!found) break;
        
        // 更新当前点
        curr_x = best_x;
        curr_y = best_y;
        
        // 填充边界数组（只填充一次，避免重复）
        if (curr_y >= 0 && curr_y < IMG_H && right_data.line[curr_y] == -1) {
            right_data.line[curr_y] = curr_x;
            right_data.scan_line[curr_y] = curr_x;
            right_data.raw_pts.push_back(cv::Point(curr_x, curr_y));
            
            // 记录方向序列
            if (right_data.dir_count < USE_num) {
                right_data.dir_sequence[right_data.dir_count] = best_dir;
                right_data.trajectory_points[right_data.dir_count][0] = curr_x;
                right_data.trajectory_points[right_data.dir_count][1] = curr_y;
                right_data.dir_count++;
            }
        }
        
        step_count++;
    }
    
    right_data.dir_valid = (right_data.dir_count > 10) ? 1 : 0;
    
    // 检查是否成功提取到足够的边界点
    if (left_data.raw_pts.size() < 5 || right_data.raw_pts.size() < 5) {
        return false;
    }
    
    return true;
}

//==================================================================================
// 阈值处理与边缘检测
//==================================================================================
// 快速二值化算法
void TrackBase::quick_otsu(cv::Mat &gray_buf) {
    frame_counter ++;

    // 间隔3帧进行一次大津法　且只计算底部1/2的部分
    if (frame_counter % OTSU_jiange == 0){
        cv::Rect roi (0, gray_buf.rows * 1/2, CAM_WIDTH, gray_buf.rows * 1/2);
        cv::Mat roi_img = gray_buf(roi);

        float new_thresh = cv::threshold(roi_img, bin_buf, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // 历史加权　低通滤波
        now_threshold = (int)(0.7 * now_threshold + 0.3 * new_thresh);
    }
    // 将底部的二值化的阈值应用于全局
    cv::threshold(gray_buf, bin_buf, now_threshold, 255, cv::THRESH_BINARY);
}

//==================================================================================
// 误差计算函数
//==================================================================================
// // 误差计算 结合速度动态改变
// 速度结合动态前瞻 (红框子版本)
float TrackBase::get_error_lookahead(int speed, float speed_decision, int window_size){

    // 强制误差为0 标志位
    if (lock_err_zero){
        track_error = 0.0f;
        return 0.0f;
    }

    float error;
    int base_offset;     
    if (element_state.model_line_pass_active){
        base_offset = 3;
    }
    else{
        base_offset = 10;
    }                                                  // 基础误差偏移量
    int lookahead_distance = (int)(speed * speed_decision) + base_offset;       // 前瞻距离

    int center_row = VALID_START_ROW - lookahead_distance;                      // 误差中心所在行
    int start_row = center_row + window_size;                                   // 窗口起始行 (下边界)
    int end_row = center_row - window_size;                                     // 窗口结束行 (上边界)

    if (start_row > VALID_START_ROW)    start_row = VALID_START_ROW;
    if (end_row < VALID_END_ROW)        end_row = VALID_END_ROW;

    debug_error_start = start_row;                                              // 误差显示
    debug_error_end = end_row;

    // 在目标行附近搜索有效的中线点
    long sum_error = 0;
    int valid_count = 0;
    for (int y = end_row; y <= start_row; y++) {
        if (mid.line[y] != -1) {
            sum_error += (mid.line[y] - CAM_WIDTH / 2);
            valid_count++;
        }
    }

    // 最终误差
    if (valid_count > 0) {
        error = (float)sum_error / valid_count;
    } 
    else {
        // 误差继承，但防止 NaN 传播
        if (std::isnan(track_error) || std::isinf(track_error)) {
            return 0.0f;
        }
        return track_error;
    }

    // 特殊处理　　在十字内部，限制误差变化幅度
    if (element_state.current_track_type == TrackType::crossroad &&
        element_state.crossroad_state == CrossroadState::inside &&
        !std::isnan(track_error) && !std::isinf(track_error)) {
        const float max_crossroad_error_step = 12.0f;
        float diff = error - track_error;
        if (diff > max_crossroad_error_step) {
            error = track_error + max_crossroad_error_step;
        } 
        else if (diff < -max_crossroad_error_step) {
            error = track_error - max_crossroad_error_step;
        }
    }

    track_error = error;
    return track_error;
}

// 模型重建中线
void TrackBase::model_nowmidline(void)
{
    process_midline(left_boundary, right_boundary, mid, current_mode);
}

// 有效区域加权误差计算 (前瞻点版本)
// float TrackBase::get_error_lookahead(int speed, float speed_decision){
//     // 速度比例换算 动态前瞻
//     float base_rate = 0.30f;               
//     float final_rate = base_rate + (speed * speed_decision);

//     // 比例限幅
//     if (final_rate > 0.80f) final_rate = 0.80f;
//     if (final_rate < 0.25f) final_rate = 0.25f;

//     // 将比例映射为真实的图像 Y 坐标
//     int roi_bottom = VALID_START_ROW;
//     int prospect_y = VALID_START_ROW - (int)(ROI_HEIGHT * final_rate);   // 误差计算区域的最高行

//     // 二次安全限幅
//     prospect_y = clip(prospect_y, VALID_END_ROW, VALID_START_ROW);

//     // 前瞻点坐标
//     int prospect_x = mid.line[prospect_y];
//     if (prospect_x == -1) prospect_x = CAM_WIDTH / 2;       // 盲区默认中心点
    
//     // 增加前瞻点X轴限幅
//     prospect_x = clip(prospect_x, 20, CAM_WIDTH - 21);

//     // debug_lookahead_y = prospect_y;       // 前瞻点坐标
//     // debug_lookahead_x = prospect_x;
//     // debug_error_start = roi_bottom;       // 误差框
//     // debug_error_end   = prospect_y;

//     // 计算平均误差
//     int mid_y = (roi_bottom + prospect_y) / 2;    // 分割误差计算的上下半区

//     long sum_top = 0, sum_bottom = 0;
//     int count_top = 0, count_bottom = 0;

//     // 统计上半区误差值 (远处，预判，权重 30%)
//     for (int y = prospect_y; y <= mid_y; y++) {
//         if (mid.line[y] != -1) {
//             sum_top += (mid.line[y] - (CAM_WIDTH / 2));
//             count_top++;
//         }
//     }

//     // 统计下半区误差值 (近处，循迹，权重 70%)
//     for (int y = mid_y + 1; y <= roi_bottom; y++) {
//         if (mid.line[y] != -1) {
//             sum_bottom += (mid.line[y] - (CAM_WIDTH / 2));
//             count_bottom++;
//         }
//     }

//     // 计算平均误差
//     float err_top    = (count_top > 0)    ? ((float)sum_top / count_top)       : 0.0f;
//     float err_bottom = (count_bottom > 0) ? ((float)sum_bottom / count_bottom) : 0.0f;

//     if (count_top > 0 && count_bottom > 0) {
//         track_error = (err_top * 0.50) + (err_bottom * 0.50);
//     } 
//     else if (count_bottom > 0) {
//         track_error = err_bottom;
//     } 
//     else if (count_top > 0) {
//         track_error = err_top;
//     }

//     // 更新显示坐标
//     int center_of_mass_y = (mid_y + prospect_y) / 2;  

//     debug_lookahead_y = center_of_mass_y;       // 误差计算的加权中心区域
//     debug_lookahead_x = mid.line[center_of_mass_y] != -1 ? mid.line[center_of_mass_y] : CAM_WIDTH / 2;
//     debug_error_start = roi_bottom;       // 误差框
//     debug_error_end   = prospect_y;

//     return track_error;
// }
//==================================================================================
// 出界检测函数
//==================================================================================
/**
 * @brief 出界检测函数
 * @param img 输入的图像矩阵   bin_buf
 * @return bool 返回是否出界的检测结果，true表示出界，false表示正常
 */
bool TrackBase::out_checking(const cv::Mat& img)
{
    if(img.empty()) return false;
    
    bool out = false;

    // 条件1: 检测图像倒数第3行的白色像素数量
    int16_t row_bottom = VALID_START_ROW - 3;
    int whiteCounts = 0;
    
    const uint8_t* ptr_bottom = img.ptr<uint8_t>(row_bottom);
    for(int i = 0; i < img.cols; i++) {
        if(ptr_bottom[i] > now_threshold) {
            whiteCounts++;
        }
    }
    
    // 白色像素点阈值判断
    if(whiteCounts < 15) {
        out = true;
    }

    // // 条件2：白列辅助判断   最长的白列太短  易误判
    // if (white_column.maxNum < 10 || white_column.lineNum < 5) {
    //     out = true;
    // }

    // // 条件3: 在当前前瞻点附近划定一个局部 ROI
    // // 判断预测的前瞻点是否已经飞出赛道扎进黑区
    // if (debug_lookahead_y > 0 && debug_lookahead_y < img.rows && 
    //     debug_lookahead_x > 0 && debug_lookahead_x < img.cols) {
        
    //     int check_width = 40; 
    //     int white_cnt = 0;
    //     int black_cnt = 0;
    //     int target_y = debug_lookahead_y;
    //     int target_x = debug_lookahead_x;
    //     int window_size = 4; // 跟 get_error 里面的窗口一致
        
    //     for (int y = target_y - window_size; y <= target_y + window_size; y++) {
    //         if (y < 0 || y >= img.rows) continue;
    //         const uint8_t* ptr = img.ptr<uint8_t>(y);
            
    //         int x_start = clip(target_x - check_width / 2, 0, CAM_WIDTH - 1);
    //         int x_end   = clip(target_x + check_width / 2, 0, CAM_WIDTH - 1);
            
    //         for (int x = x_start; x <= x_end; x++) {
    //             if (ptr[x] > now_threshold) white_cnt++; 
    //             else black_cnt++;
    //         }
    //     }
        
    //     // 如果黑区面积大于或等于白色面积即可认定出界
    //     if (black_cnt >= white_cnt && (black_cnt + white_cnt) > 0) { 
    //         out = true;
    //     }
    // }

    is_out_of_bounds = out;
    return out;
}

TrackBase track_base;
//==================================================================================
// 【互斥截断保护】迷宫法专用，已屏蔽
//==================================================================================
#if 0
void TrackBase::mutex_boundary_truncation()
{
    // 从底部往上扫描，如果发现两线相距太近 (或交叉)，就在此处果断截断，丢弃以上的所有脏数据
    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        int lx = left_boundary.line[y];
        int rx = right_boundary.line[y];
        
        if (lx != -1 && rx != -1) {
            // 如果右线比左线大不了多少，甚至跑到了左线左边 (间距小于赛道最小透视宽度 15 像素)
            if (rx - lx < 15) { 
                // 发生串线！直接在此行上方全部截断 (设为-1)
                for (int ty = y; ty >= VALID_END_ROW; ty--) {
                    left_boundary.line[ty] = -1;
                    right_boundary.line[ty] = -1;
                }
                
                // 同时把出问题的原始点集也截断，防止角点状态机被误导
                while (!left_boundary.raw_pts.empty() && left_boundary.raw_pts.back().y <= y) {
                    left_boundary.raw_pts.pop_back();
                }
                while (!right_boundary.raw_pts.empty() && right_boundary.raw_pts.back().y <= y) {
                    right_boundary.raw_pts.pop_back();
                }
                break; // 截断完毕，退出检查
            }
        }
    }
}
#endif
