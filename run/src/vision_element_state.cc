#include "vision_element_state.h"

///////////////////////////////////////////////// 元素主处理 ///////////////////////////////////////////
/**
 * @brief 根据当前元素状态返回拐点约束配置
 * @param is_left 是否为左边线
 * @return Limit 拐点约束配置
 */
ElementState::Limit ElementState::get_corner_limit(bool is_left) {
    Limit limit{};

    // 拐点查找固定区域
    limit.enable_global_search = true;
    limit.search_y_min = VALID_END_ROW;
    limit.search_y_max = VALID_START_ROW;
    limit.down_up_min_gap = 5;

    // limit.lock_down_pt = false;
    // limit.lock_up_pt = false;
    // limit.max_lock_frames = 15;

    limit.enable_spatial_constraint = true;
    limit.need_down_lp = true;
    limit.need_mid_lp  = true;
    limit.need_up_lp   = true;
    limit.dx_threshold_scale = 1.0f;

    const int x_mid = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
    const int y_span = ROI_HEIGHT;

    // 固定Y区域（允许重叠）
    // 下拐点：偏下半区
    limit.down_y_min = VALID_END_ROW + (int)(0.60f * y_span); // 35
    limit.down_y_max = VALID_START_ROW;

    // 中拐点：中间大区（和上下都重叠）
    limit.mid_y_min = VALID_END_ROW + (int)(0.75f * y_span);  // 40
    limit.mid_y_max = VALID_START_ROW;

    // 上拐点：偏上半区
    limit.up_y_min = VALID_END_ROW;
    limit.up_y_max = VALID_END_ROW + (int)(0.40f * y_span);   // 28

    // 固定X区域（按左右边分半幅）
    if (is_left) {
        limit.down_x_min = VALID_LEFT_COL;
        limit.down_x_max = x_mid;

        limit.mid_x_min  = VALID_LEFT_COL;
        limit.mid_x_max  = x_mid;

        limit.up_x_min   = VALID_LEFT_COL;
        limit.up_x_max   = x_mid;
    } 
    else {
        limit.down_x_min = x_mid;
        limit.down_x_max = VALID_RIGHT_COL;

        limit.mid_x_min  = x_mid;
        limit.mid_x_max  = VALID_RIGHT_COL;

        limit.up_x_min   = x_mid;
        limit.up_x_max   = VALID_RIGHT_COL;
    }

    // 安全限幅（防止越界）
    limit.down_x_min = clip(limit.down_x_min, VALID_LEFT_COL, VALID_RIGHT_COL);
    limit.down_x_max = clip(limit.down_x_max, VALID_LEFT_COL, VALID_RIGHT_COL);
    limit.mid_x_min  = clip(limit.mid_x_min,  VALID_LEFT_COL, VALID_RIGHT_COL);
    limit.mid_x_max  = clip(limit.mid_x_max,  VALID_LEFT_COL, VALID_RIGHT_COL);
    limit.up_x_min   = clip(limit.up_x_min,   VALID_LEFT_COL, VALID_RIGHT_COL);
    limit.up_x_max   = clip(limit.up_x_max,   VALID_LEFT_COL, VALID_RIGHT_COL);

    limit.down_y_min = clip(limit.down_y_min, VALID_END_ROW, VALID_START_ROW);
    limit.down_y_max = clip(limit.down_y_max, VALID_END_ROW, VALID_START_ROW);
    limit.mid_y_min  = clip(limit.mid_y_min,  VALID_END_ROW, VALID_START_ROW);
    limit.mid_y_max  = clip(limit.mid_y_max,  VALID_END_ROW, VALID_START_ROW);
    limit.up_y_min   = clip(limit.up_y_min,   VALID_END_ROW, VALID_START_ROW);
    limit.up_y_max   = clip(limit.up_y_max,   VALID_END_ROW, VALID_START_ROW);

    return limit;
}

// 元素识别初始化
void ElementState::element_init(void)
{
    // 初始化各个状态变量
    current_track_type = TrackType::normal;
    track_base.current_mode = TrackMode::auto_mode;

    zebra_state = ZebraState::shielded;
    zebra_confirm_count = 0;
    is_shielded = true; // 起步默认屏蔽斑马线
    motor_distance_clear();

    crossroad_state = CrossroadState::none;
    crossroad_confirm_count = 0;
    crossroad_pass_count = 0;
    crossroad_inside_anchor_locked = false;
    crossroad_left_bottom_anchor = cv::Point(-1, -1);
    crossroad_right_bottom_anchor = cv::Point(-1, -1);
    crossroad_left_quality = 0.0f;
    crossroad_right_quality = 0.0f;
    // crossroad_cooldown_count = 0;
    crossroad_last_error = 0.0f;
    // crossroad_distance = 0;
    
    // locked_Lk = 0;
    // locked_Lb = 0;
    // locked_Rk = 0;
    // locked_Rb = 0;

    // 路障初始化
    barrier_state = BarrierState::none;
    barrier_dir = BarrierDir::none;
    barrier_confirm_count = 0;
    barrier_offset = 0;

    // 模型绕行初始化
    model_line_pass_active = false;
    model_line_pass_dir = ModelLinePassDir::none;
    model_line_pass_distance = 11.2f;
    model_line_pass_snapshot_valid = false;

    // 坡道初始化
    ramp_state = RampState::none;
    ramp_circle_shield_count = 0;
    ramp_up_confirm_count = 0;
    ramp_down_confirm_count = 0;
    ramp_flat_confirm_count = 0;

    // long_straight_count = 0;

    // 圆环初始化
    circle_state = CircleState::none;
    circle_dir = CircleDir::none;
    approach_confirm_cnt = 0;
    circle_confirm_count = 0;
    circle_pass_count = 0;
    circle_seen_bottom_lost = false;
    fixed_C_pt = cv::Point(-1, -1);
    // last_circle_up_pt = cv::Point(-1, -1);
    // circle_up_stable_count = 0;
    circle_exit_start_yaw = 0.0f;
    circle_run_index = 0;
    circle_exit_confirm_count = 0;
    circle_leave_confirm_count = 0;
    circle_inside_bottom_anchor = cv::Point(-1, -1);
    
    // 常规道路类型初始化
    normal_type = NormalType::straight;
    normal_road_state = NormalRoadState::straight_fast;
    curve_score = 0.0f;
    
    // 直弯道防抖计数器初始化
    straight_confirm_count = 0;
    curve_confirm_count = 0;
    
    // 斜率历史初始化
    last_left_k = 0.0f;
    last_right_k = 0.0f;
    k_init = false;
}

/**
 * @brief 元素识别主函数，每帧调用 (元素识别状态机)
 * @param bin_buf 输入的二值化图像  track_base.bin_buf
 * @param rgb_img 彩色图像(用于路障)    track_base.raw_buf
 * @param tof_distance TOF传感器(用于坡道)
 */
void ElementState::elements_process(const cv::Mat &bin_buf, const cv::Mat &rgb_img, uint16_t tof_distance)
{
    current_tof_dist = tof_distance; // 存数据发送上位机

    if (bin_buf.empty())
        return;

    // // 坡道屏蔽圆环检测
    // if (pitch_angle <= -5.0f) {
    //     ramp_circle_shield_count = 25;
    // } 
    // else if (ramp_circle_shield_count > 0) {
    //     ramp_circle_shield_count--;
    // }

    // 模型绕行处理
    if (model_line_pass_active)
    {
        // 模型绕行期间优先级最高
        current_track_type = TrackType::model;
        normal_type = NormalType::curve;
                
        preprocess.preview = preprocess.model_preview / 100.0f;
        track_base.current_mode = (model_line_pass_dir == ModelLinePassDir::left) ?
                                  TrackMode::left_raw : TrackMode::right_raw;
        track_base.model_nowmidline();
        
        motor_distance_get();
        // std::cout << "编码器当前积分：" << motor_distance << std::endl;
        if (motor_distance >= model_line_pass_distance) {
            stop_model_line_pass();
        }
        return;
    }

    // 斑马线屏蔽处理：起步默认屏蔽斑马线　编码器达到阈值后解除屏蔽
    if (is_shielded)
    {
        motor_distance_get();
        zebra_shield(motor_distance, 15.0f); // 解除屏蔽
    }

    // 模型绕行结束后的继续平滑阶段，仍然屏蔽特殊元素检测
    if (continue_pass && current_track_type == TrackType::normal)
    {
        // zebra_confirm_count = 0;
        circle_confirm_count = 0;
        barrier_confirm_count = 0;
        crossroad_confirm_count = 0;
        classify_normal_road();
        return;
    }

    // 元素检测：优先检测特殊元素，只有在非特殊元素时才分类常规道路
    if (current_track_type == TrackType::normal)
    {
        CircleDir c_dir = CircleDir::none;
        BarrierDir b_dir = BarrierDir::none;

        // if (tof_distance > TOF_MIN_DIS && tof_distance < TOF_MAX_DIS)
        // {
        //     if (barrier_detect(rgb_img, tof_distance, b_dir))
        //     {
        //         current_track_type = TrackType::barrier;
        //         barrier_state = BarrierState::detected;
        //         barrier_dir = b_dir;
        //     }
        //     else if (ramp_detect(tof_distance, rgb_img))
        //     {
        //         current_track_type = TrackType::ramp;
        //         ramp_state = RampState::detected;
        //     }
        // }

        // 检测优先级：斑马线 > 十字 > 圆环 > 路障 > 坡道
        CrossroadType cr_type = CrossroadType::none;

        // // 圆环冷却：退出圆环后短暂屏蔽
        // static int circle_cd = 0;
        // if (circle_cd > 0)
        //     circle_cd--;

        // // 十字冷却递减
        // if (crossroad_cooldown_count > 0)
        //     crossroad_cooldown_count--;

        // 斑马线检测
        if (!is_shielded && zebra_detect(bin_buf))
        {
            current_track_type = TrackType::zebra;
            // 注意：千万不要在这里直接把状态赋为 detected，否则防抖期遇到丢线会触发永久屏蔽
            zebra_confirm_count = 1; 
        }
        // // 十字检测 (冷却期间禁止检测)
        // else if (crossroad_cooldown_count == 0 && crossroad_detect(cr_type))

        // 十字检测 (添加防抖机制)
        else if (crossroad_detect(cr_type))
        {
            crossroad_confirm_count++;
            if (crossroad_confirm_count >= 3) {
                current_track_type = TrackType::crossroad;
                crossroad_state = CrossroadState::approach;
                crossroad_type = cr_type;
                crossroad_confirm_count = 0;  
            }
        }
        // 圆环检测（两阶段：弱入环→确认入环）
        else if (circle_entry_allowed() && circle_detect_weak(c_dir))
        {
            circle_confirm_count++;
            if (circle_confirm_count >= 2) {
                // 进入弱入环状态
                current_track_type = TrackType::circle;
                circle_state = CircleState::weak_approach;
                circle_dir = c_dir;
                // 弱入环状态同帧切换到直道侧单边巡线，避免等下一帧才响应
                track_base.current_mode = (c_dir == CircleDir::left) ? TrackMode::right_full : TrackMode::left_full;
                approach_confirm_cnt = 0;
                circle_confirm_count = 0;
                circle_pass_count = 0;
                circle_seen_bottom_lost = false;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_inside_bottom_anchor = cv::Point(-1, -1);
                fixed_C_pt = cv::Point(-1, -1);
                // last_circle_up_pt = cv::Point(-1, -1);
                // circle_up_stable_count = 0;
                circle_exit_start_yaw = 0.0f;
                motor_distance_clear();  // 清零编码器，开始计时
            }
        }

        // // 路障检测
        // else if (barrier_detect(rgb_img, b_dir))
        // {
        //     barrier_confirm_count++;
        //     if (barrier_confirm_count >= 3) {
        //         current_track_type = TrackType::barrier;
        //         barrier_state = BarrierState::avoiding;
        //         barrier_dir = b_dir;
        //         barrier_confirm_count = 0;
        //         barrier_offset = (b_dir == BarrierDir::left) ? MID_OFFSET : -MID_OFFSET;    // 路障在左 → 中线向右偏，反之亦然
        //         motor_distance_clear();  
        //     }
        // }

        // // 坡道检测
        // else if (ramp_detect())
        // {
        //     current_track_type = TrackType::ramp;
        //     ramp_state = RampState::detected;
        // }

        
        // 没检测到就清零防抖
        else
        {
            // 没有检测到任何特殊元素，清零防抖计数器
            zebra_confirm_count = 0;
            circle_confirm_count = 0; 
            barrier_confirm_count = 0;
            crossroad_confirm_count = 0;
            
            // 只有在确认是常规道路时才进行直道/弯道分类
            classify_normal_road();
        }
    }

    // 元素处理

    // 常规元素有专门的处理代码
    // if (current_track_type == TrackType::normal)
    //     track_base.current_mode = TrackMode::auto_mode;

    switch (current_track_type)
    {
    case TrackType::normal:
        break;

    case TrackType::zebra:
        zebra_process(bin_buf);
        break;

    case TrackType::circle:
        circle_process(bin_buf);
        break;

    case TrackType::crossroad:
        crossroad_process(bin_buf);
        break;

    case TrackType::barrier:
        barrier_process(rgb_img);
        break;

    case TrackType::model:
        break;          // 模型绕行处理在控制决策中处理

    case TrackType::ramp:
        ramp_process();
        break;
    }

    //延时急停(冲过斑马线一段距离再急停)
    if(wait_stop)
    {
        wait_stop_time --;
        if(wait_stop_time <= 0)
        {
            wait_stop_time = 0; //计时清零
            wait_stop = false;  //翻转状态
            car_soft_stop();    //关闭电机
            fans_duty_set(0);   //再次关闭负压
            Gogo = false;       //触发急停

            std::cout << "斑马线停车完毕" << std::endl;
        }
    }
}

///////////////////////////////////////////////// 各个元素内部处理 ///////////////////////////////////////////


// ==============
// 圆环 (视觉 + 陀螺仪角度)
// ==============

namespace {
// 圆环参数结构体
struct CircleRunParam {
    float exit_yaw;                 // 出环所需角度值
    float exit_force_yaw;           // 出环强制结束所需角度值
    float exit_min_yaw;             // 出环拉线时的最小角度
    float exit_max_yaw;             // 出环拉线时的最大角度(退出角度)
    float exit_k_start;             // 出环起始拉线斜率
    float exit_k_end;               // 出环结束拉线斜率
    // int leave_confirm_frames;       // 离开确认帧数
    float leave_min_distance;       // 离开时最小距离
    // float preview_scale;            // 预览图像缩放比例
};

// 圆环索引限制与更新
int get_circle_type_by_index(int index)
{
    int count = preprocess.circle_count;
    if (count <= 0) return 1;
    if (count > 5) count = 5;

    if (index < 0) index = 0;
    if (index >= count) index = count - 1;

    int type = 1;
    switch (index) {
        case 0: type = preprocess.ring1_type; break;
        case 1: type = preprocess.ring2_type; break;
        case 2: type = preprocess.ring3_type; break;
        case 3: type = preprocess.ring4_type; break;
        case 4: type = preprocess.ring5_type; break;
        default: type = 1; break;
    }

    if (type < 0) type = 0;
    if (type > 3) type = 3;
    return type;
}

// 圆环索引限制与更新　同上
int advance_circle_index(int index)
{
    int count = preprocess.circle_count;
    if (count <= 0) return 0;
    if (count > 5) count = 5;
    if (index < count - 1) return index + 1;
    return index;
}

const char* circle_type_to_ascii(int type)
{
    if (type == 0) return "SMALL";
    if (type == 1) return "MEDIUM";
    if (type == 2) return "LARGE";
    return "BIG";
}
// 圆环参数生成  类型：0-小圆环，1-中圆环，2-第三小圆环，3-大圆环
CircleRunParam make_circle_param(int circle_type)
{
    if (circle_type == 3) {
        return {
            preprocess.circle_big_exit_yaw,
            preprocess.circle_big_exit_force_yaw,
            preprocess.circle_big_exit_min_yaw,
            preprocess.circle_big_exit_max_yaw,
            preprocess.circle_big_exit_k_start,
            preprocess.circle_big_exit_k_end,
            // std::max(1, preprocess.circle_big_leave_confirm_frames),
            preprocess.circle_big_leave_min_distance,
            // preprocess.circle_big_preview_scale
        };
    }

    if (circle_type == 2) {
        return {
            preprocess.circle_large_exit_yaw,
            preprocess.circle_large_exit_force_yaw,
            preprocess.circle_large_exit_min_yaw,
            preprocess.circle_large_exit_max_yaw,
            preprocess.circle_large_exit_k_start,
            preprocess.circle_large_exit_k_end,
            // std::max(1, preprocess.circle_large_leave_confirm_frames),
            preprocess.circle_large_leave_min_distance,
            // preprocess.circle_large_preview_scale
        };
    }

    if (circle_type == 0) {
        return {
            preprocess.circle_small_exit_yaw,
            preprocess.circle_small_exit_force_yaw,
            preprocess.circle_small_exit_min_yaw,
            preprocess.circle_small_exit_max_yaw,
            preprocess.circle_small_exit_k_start,
            preprocess.circle_small_exit_k_end,
            // std::max(1, preprocess.circle_small_leave_confirm_frames),
            preprocess.circle_small_leave_min_distance,
            // preprocess.circle_small_preview_scale
        };
    }

    return {
        preprocess.circle_medium_exit_yaw,
        preprocess.circle_medium_exit_force_yaw,
        preprocess.circle_medium_exit_min_yaw,
        preprocess.circle_medium_exit_max_yaw,
        preprocess.circle_medium_exit_k_start,
        preprocess.circle_medium_exit_k_end,
        // std::max(1, preprocess.circle_medium_leave_confirm_frames),
        preprocess.circle_medium_leave_min_distance,
        // preprocess.circle_medium_preview_scale
    };
}
}
/**
 * @brief 圆环入口检测
 *
 * 只决定是否允许触发 weak_approach，不影响圆环内部状态机
 */
bool ElementState::circle_entry_allowed()
{
    if (ramp_circle_shield_count > 0) return false;
    if (ramp_state != RampState::none) return false;
    return true;
}

/**
 * @brief 圆环弱入环检测（第一阶段）
 *
 * 条件一：环侧有下拐点（下拐点一侧即为环侧）后续优化中拐点
 * 条件二：一侧直道（丢线行数 <= 5行），另一侧丢线较多（丢线行数 >= 12行）
 *         且丢线起始坐标在底部靠近区域（lost_y >= 75，类似十字检测的限制）
 * 两个条件都满足才判断为弱入环
 *
 * @param dir 输出检测到的圆环方向
 * @return 是否检测到弱入环特征
 */
bool ElementState::circle_detect_weak(CircleDir &dir)
{
    // 方案1扩展：圆环检测前先进行中下拐点融合
    auto fuse_mid_down = [](PointState::BoundaryData& bd) {
        const int merge_dist = 4;
        const int max_y_diff = 5;  // 优先级2：增加Y坐标差限制
        
        // 如果中下拐点距离很近，融合为下拐点
        if (bd.mid_lp_state && bd.down_lp_state) {
            int dx = std::abs(bd.mid_lp_pt.x - bd.down_lp_pt.x);
            int dy = std::abs(bd.mid_lp_pt.y - bd.down_lp_pt.y);
            // 距离近 且 Y坐标差不大（避免跨度太大的点被融合）
            if (dx <= merge_dist && dy <= max_y_diff) {
                // 取平均值作为融合后的下拐点
                bd.down_lp_pt.x = (bd.down_lp_pt.x + bd.mid_lp_pt.x) / 2;
                bd.down_lp_pt.y = (bd.down_lp_pt.y + bd.mid_lp_pt.y) / 2;
                bd.mid_lp_state = false;
                bd.mid_lp_pt = cv::Point(-1, -1);
            }
        }
        // 如果只有mid没有down，且mid在合理位置，提升为down
        else if (bd.mid_lp_state && !bd.down_lp_state && bd.mid_lp_pt.y >= VALID_END_ROW + 10) {
            bd.down_lp_state = true;
            bd.down_lp_pt = bd.mid_lp_pt;
            bd.mid_lp_state = false;
            bd.mid_lp_pt = cv::Point(-1, -1);
        }
    };
    
    fuse_mid_down(left_boundary);
    fuse_mid_down(right_boundary);
    
    // 左环弱入环
    bool left_flag = false;
    
    // 条件1：环侧（左侧）有 lower 角点（down 或 mid）
    bool left_has_lower = left_boundary.down_lp_state || left_boundary.mid_lp_state;
    
    // 条件2：环侧（左侧）有上拐点
    bool left_has_upper = left_boundary.up_lp_state;
    
    // 条件3：对侧（右侧）不丢线
    bool right_not_lost = (!right_boundary.is_lost || (right_boundary.is_lost && right_boundary.lost_y <= 22));
    
    // 条件4：对侧（右侧）是直道（单调性好）
    bool right_is_straight = right_boundary.is_straight;
    
    // 条件5：环侧（左侧）丢线多 + 丢线Y坐标在底部
    bool left_lost_ok = left_boundary.is_lost && (left_boundary.lost_y >= 30);
    
    // 条件6：左右连续线段之差 ≥ 25
    int line_diff = right_boundary.mnt_len - left_boundary.mnt_len;
    bool line_diff_ok = (line_diff >= 12);

    // 圆环入口断线型 lower：普通 down/mid 容易漏检时，用更严格的组合特征补充
    // 仍然要求上拐点、环侧中部丢线、环侧线长很短、对侧直线稳定，避免急弯仅凭 upper 误判
    bool left_break_like_lower = left_has_upper &&
                                 left_boundary.is_lost &&
                                 (left_boundary.lost_y >= 30) &&
                                 (left_boundary.mnt_len <= 6) &&
                                 right_not_lost &&
                                 right_is_straight &&
                                 (right_boundary.mnt_len >= 20);
    bool left_circle_lower = left_has_lower || left_break_like_lower;
    
    if ((left_circle_lower && left_has_upper)&& right_not_lost && right_is_straight && 
        left_lost_ok && line_diff_ok) {
        left_flag = true;
    }
    
    // 右环弱入环
    bool right_flag = false;
    
    // 条件1：环侧（右侧）有 lower 角点（down 或 mid）
    bool right_has_lower = right_boundary.down_lp_state || right_boundary.mid_lp_state;
    
    // 条件2：环侧（右侧）有上拐点
    bool right_has_upper = right_boundary.up_lp_state;
    
    // 条件3：对侧（左侧）不丢线
    bool left_not_lost = (!left_boundary.is_lost || (left_boundary.is_lost && left_boundary.lost_y <= 22));
    
    // 条件4：对侧（左侧）是直道（单调性好）
    bool left_is_straight = left_boundary.is_straight;
    
    // 条件5：环侧（右侧）丢线多 + 丢线Y坐标在底部
    bool right_lost_ok = right_boundary.is_lost && (right_boundary.lost_y >= 30);
    
    // 条件6：左右连续线段之差 ≥ 25
    line_diff = left_boundary.mnt_len - right_boundary.mnt_len;
    line_diff_ok = (line_diff >= 12);

    bool right_break_like_lower = right_has_upper &&
                                  right_boundary.is_lost &&
                                  (right_boundary.lost_y >= 30) &&
                                  (right_boundary.mnt_len <= 6) &&
                                  left_not_lost &&
                                  left_is_straight &&
                                  (left_boundary.mnt_len >= 20);
    bool right_circle_lower = right_has_lower || right_break_like_lower;
    
    if ((right_circle_lower && right_has_upper) && left_not_lost && left_is_straight && 
        right_lost_ok && line_diff_ok) {
        right_flag = true;
    }

    if (left_flag) {
        dir = CircleDir::left;
        return true;
    }
    if (right_flag) {
        dir = CircleDir::right;
        return true;
    }

    return false;
}

/**
 * @brief 圆环处理主函数 (5状态机：视觉+陀螺仪混合方案，去除帧计数器)
 *
 * weak_approach: 弱入环 — 检测到拐点特征，对侧单边巡线
 *                切换条件：环侧丢线（物理状态：车已进入圆环）
 * approach:      入环 — 清零陀螺仪，环侧单边巡线
 *                切换条件：转过90度 + 直道侧不再丢线（物理状态：已转过入口弯）
 * inside:        环内 — 直道侧单边巡线，陀螺仪270度　|| 直道侧丢线切出
 * exit:          出环 — 固定拉线引导出口
 * leave:         出环直行 — 直道侧单边巡线
 */
void ElementState::circle_process(const cv::Mat &bin_buf)
{
    if (bin_buf.empty())
        return;

    // 环侧/对侧引用
    bool ring_is_left = (circle_dir == CircleDir::left);
    auto &ring_bd = ring_is_left ? left_boundary : right_boundary;
    auto &other_bd = ring_is_left ? right_boundary : left_boundary;
    const int x_mid = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;

    // 状态判断使用清线/补线之前的原始特征，避免补线污染
    struct CircleSideSnapshot {
        bool is_lost;
        int lost_y;
        int lost_count;
        int mnt_len;
        bool down_lp_state;
        bool mid_lp_state;
        bool up_lp_state;
        cv::Point down_lp_pt;
        cv::Point mid_lp_pt;
        cv::Point up_lp_pt;
    };

    auto make_snapshot = [](const PointState::BoundaryData& bd) -> CircleSideSnapshot {
        CircleSideSnapshot snap;
        snap.is_lost = bd.is_lost;
        snap.lost_y = bd.lost_y;
        snap.lost_count = bd.lost_count;
        snap.mnt_len = bd.mnt_len;
        snap.down_lp_state = bd.down_lp_state;
        snap.mid_lp_state = bd.mid_lp_state;
        snap.up_lp_state = bd.up_lp_state;
        snap.down_lp_pt = bd.down_lp_pt;
        snap.mid_lp_pt = bd.mid_lp_pt;
        snap.up_lp_pt = bd.up_lp_pt;
        return snap;
    };

    const CircleSideSnapshot ring_raw = make_snapshot(ring_bd);
    const CircleSideSnapshot other_raw = make_snapshot(other_bd);
    const int circle_type = get_circle_type_by_index(circle_run_index);
    const CircleRunParam circle_param = make_circle_param(circle_type);

    // 控制用清线：只影响本帧中线生成，不参与状态判断
    auto clear_line = [&](PointState::BoundaryData& bd) {
        for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
            bd.line[y] = -1;
        }
    };

    // 工具函数：防串边（串到对侧就强制推到边界）
    auto clamp_half = [&](PointState::BoundaryData& bd, bool is_left_side) {
        for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
            if (bd.line[y] == -1) continue;
            if (is_left_side) {
                // 左边线：如果串到右半区（>= x_mid），强制推到最左边界
                if (bd.line[y] >= x_mid) {
                    bd.line[y] = VALID_LEFT_COL;
                } else {
                    bd.line[y] = clip(bd.line[y], VALID_LEFT_COL, VALID_RIGHT_COL);
                }
            } 
            else {
                // 右边线：如果串到左半区（<= x_mid），强制推到最右边界
                if (bd.line[y] <= x_mid) {
                    bd.line[y] = VALID_RIGHT_COL;
                } else {
                    bd.line[y] = clip(bd.line[y], VALID_LEFT_COL, VALID_RIGHT_COL);
                }
            }
        }
    };

    // 圆环状态切换后立即同步巡线模式，避免状态已经变了但中线仍按上一状态生成
    auto sync_circle_track_mode = [&](CircleState next_state) {
        switch (next_state) {
            case CircleState::weak_approach:
                track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
                clear_line(ring_bd);
                clamp_half(other_bd, !ring_is_left);
                break;
            case CircleState::approach:
                track_base.current_mode = ring_is_left ? TrackMode::left_full : TrackMode::right_full;
                clear_line(other_bd);
                clamp_half(ring_bd, ring_is_left);
                break;
            case CircleState::inside:
                track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
                clear_line(ring_bd);
                break;
            case CircleState::exit:
                track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
                clear_line(ring_bd);
                break;
            case CircleState::leave:
                track_base.current_mode = TrackMode::auto_mode;
                clamp_half(other_bd, !ring_is_left);
                break;
            default:
                track_base.current_mode = TrackMode::auto_mode;
                break;
        }
    };

    switch (circle_state)
    {
        case CircleState::weak_approach:
        {
            // circle_pass_count++;  // 开启帧计数
            
            // 强制直道侧单边巡线
            track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
            
            // // 清空环侧拐点（避免误判）
            // ring_bd.down_lp_state = false;
            // ring_bd.mid_lp_state = false;
            // ring_bd.up_lp_state = false;
            
            // // 入环确认用原始控制线判断，必须放在清线前　环侧底部整段看不到线，才认为车真正到入口附近
            // int ring_bottom_lost_cnt = 0;
            // int bottom_lost_start = VALID_BOTTOM_ROW - 10;
            // int bottom_lost_end = VALID_BOTTOM_ROW;
            // for (int y = bottom_lost_start; y <= bottom_lost_end; y++) {
            //     if (ring_bd.line[y] == -1) {
            //         ring_bottom_lost_cnt++;
            //     } else {
            //         break;
            //     }
            // }
            // int bottom_lost_need = (bottom_lost_end - bottom_lost_start) + 1;
            // bool ring_bottom_lost_ok = ring_bottom_lost_cnt >= 8;
   
            // 入环确认必须放在清线前，用原始丢线位置判断。
            // 先看到 lost_y 压到底部，再等 lost_y 离开底部，说明车已经越过下面入口。
            bool ring_lost_at_bottom = ring_raw.is_lost && (ring_raw.lost_y >= VALID_BOTTOM_ROW - 2);
            if (ring_lost_at_bottom) {
                circle_seen_bottom_lost = true;
            }

            bool ring_lost_leave_bottom = circle_seen_bottom_lost &&
                                           ((ring_raw.is_lost && ring_raw.lost_y <= VALID_BOTTOM_ROW - 3) || (!ring_raw.is_lost));

   

            // 环侧清线，仅用于控制中线生成
            clear_line(ring_bd);
            
            // 直道侧是有效巡线侧，强制在对应半区（防串边）
            clamp_half(other_bd, !ring_is_left);
            
            //获取编码器积分
            motor_distance_get();

            //暂时禁止偏航角中线复位
            yaw_reset = false;
            
            // 退出条件检测
            
            // 退出条件1：误判检测（直道侧同时出现中下+上拐点）
            bool other_has_lower = other_raw.down_lp_state || other_raw.mid_lp_state;
            bool other_has_upper = other_raw.up_lp_state;
            
            if (other_has_lower && other_has_upper) {
                // 直道侧出现多个拐点，判定为误判，退回检测
                circle_state = CircleState::none;
                current_track_type = TrackType::normal;
                track_base.current_mode = TrackMode::auto_mode;
                circle_dir = CircleDir::none;
                approach_confirm_cnt = 0;
                circle_confirm_count = 0;
                circle_pass_count = 0;
                circle_seen_bottom_lost = false;
                yaw_reset = true;  //重启偏航角中线复位
                fixed_C_pt = cv::Point(-1, -1);
                // last_circle_up_pt = cv::Point(-1, -1);
                // circle_up_stable_count = 0;
                break;
            }

            // // 新入环判断逻辑：先识别缺口，再等待双边恢复
            // static bool seen_ring_hole = false;      // 增加静态标志位，记住是否见过入口
            
            // // 条件A：环侧丢线（说明已经到入口附近）
            // bool ring_hole = ring_raw.is_lost && (ring_raw.lost_y >= 85);
            // if (ring_hole) {
            //     seen_ring_hole = true;
            // }

            // // // 条件B：底部双边恢复（说明车身已经进入两个路口中间段）
            // // bool bottom_recover = check_circle_bottom_recovered();

            // // 直道侧需保持稳定
            // bool other_nolost_ok = (!other_raw.is_lost || (other_raw.is_lost && other_raw.lost_y >= 55)) && other_raw.mnt_len >= 20;

            // 直道侧需保持稳定，环侧丢线需经历“到底部 -> 离开底部”
            bool other_nolost_ok = (!other_raw.is_lost || (other_raw.is_lost && other_raw.lost_y <= 28)) && other_raw.mnt_len >= 13;
            bool approach_ok = ring_lost_leave_bottom && other_nolost_ok;

            // bool approach_ok = seen_ring_hole && other_nolost_ok;

            if (approach_ok) {
                approach_confirm_cnt++;
            } 
            else {
                approach_confirm_cnt = 0;
            }

            if (approach_ok && approach_confirm_cnt >= 2) {
                // 进入确认入环状态
                circle_state = CircleState::approach;
                sync_circle_track_mode(circle_state);
                circle_pass_count = 0;
                approach_confirm_cnt = 0;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_inside_bottom_anchor = cv::Point(-1, -1);
                circle_seen_bottom_lost = false;
                // seen_ring_hole = false; // 复位标志位
                madg.yaw_angle_reset(); //各角度和相关数据复位
                motor_distance_clear();  //清零编码器，准备下一阶段
                break;
            }

            // // 退出条件3：超时强制进环（兜底保护 - 双保险）
            // // bool frame_timeout = circle_pass_count >= 3;  // 帧数超时
            // bool distance_timeout = motor_distance >= 16.0f;  // 编码器超时（150cm）
            
            // if (distance_timeout) {
            //     // 超时强制进入环内，跳过approach直接进inside
            //     circle_state = CircleState::approach;
            //     circle_pass_count = 0;
            //     approach_confirm_cnt = 0;
            //     circle_seen_bottom_lost = false;
            //     // seen_ring_hole = false;
            //     madg.angle_reset(); //各角度和相关数据复位
            //     yaw_clear();  //清零存放偏航角的变量
            //     motor_distance_clear();  // 清零编码器
            //     fixed_C_pt = cv::Point(-1, -1);
            // }
            break;
        }

        case CircleState::approach:
        {
            circle_pass_count++;
            
            // 入环：强制使用圆环侧单边巡线
            track_base.current_mode = ring_is_left ? TrackMode::left_full : TrackMode::right_full;
            
            // // 清空直道侧拐点（避免误判）
            // other_bd.down_lp_state = false;
            // other_bd.mid_lp_state = false;
            // other_bd.up_lp_state = false;
            
            // 直道侧清线，仅用于控制中线生成
            clear_line(other_bd);
            
            // 环侧是有效巡线侧，强制在对应半区（防串边）
            clamp_half(ring_bd, ring_is_left);

            // if (fixed_C_pt.x >= VALID_LEFT_COL && fixed_C_pt.x <= VALID_RIGHT_COL &&
            //     fixed_C_pt.y >= VALID_END_ROW && fixed_C_pt.y <= VALID_START_ROW) {
            //     cv::Point approach_start(
            //         ring_is_left ? VALID_LEFT_COL : VALID_RIGHT_COL,
            //         VALID_START_ROW
            //     );
            //     connect_points(ring_bd.line, approach_start, fixed_C_pt);
            //     clamp_half(ring_bd, ring_is_left);
            // }
            
            // 强制种子点在环侧（覆盖自动检测结果）
            if (ring_is_left) {
                // 左环：强制种子点在左侧
                track_base.white_column.starlineX = VALID_LEFT_COL + 5;
                track_base.white_column.endlineX = VALID_LEFT_COL + 10;
            } 
            else {
                // 右环：强制种子点在右侧
                track_base.white_column.starlineX = VALID_RIGHT_COL - 5;
                track_base.white_column.endlineX = VALID_RIGHT_COL - 10;
            }

            // 实时获取陀螺仪角度
            // yaw_angle_get();
            float abs_yaw = std::fabs(yaw_angle);

            //暂时禁止偏航角中线复位
            yaw_reset = false;
                        
            // 退出条件：转过90度 + 直道侧不再丢线（物理状态：已转过入口弯）
            bool angle_ok = abs_yaw >= 75.0f;
            bool straight_ok = !other_raw.is_lost || (other_raw.is_lost && other_raw.lost_y <= 35);
            
            // 双重条件：角度 + 视觉
            // if (angle_ok && straight_ok) {
            if (angle_ok) {
                circle_state = CircleState::inside;
                sync_circle_track_mode(circle_state);
                circle_pass_count = 0;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_inside_bottom_anchor = cv::Point(-1, -1);
                fixed_C_pt = cv::Point(-1, -1);
                // last_circle_up_pt = cv::Point(-1, -1);
                // circle_up_stable_count = 0;
            }
            
            // // 保底：防止陀螺仪失效卡死（帧数兜底，阈值放宽）
            // if (circle_pass_count >= 50 && straight_ok) {
            //     circle_state = CircleState::inside;
            //     circle_pass_count = 0;
            //     fixed_C_pt = cv::Point(-1, -1);
            // }
            
            break;
        }

        case CircleState::inside:
        {
            // circle_pass_count++;

            // 环内保持圆环侧单边巡线，避免 auto_mode 在双边/左推/右推之间切换造成中线跳变
            track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
            
            // track_base.current_mode = TrackMode::auto_mode; //进入环内后不强制单边巡线，交给正常的弯道处理去适应复杂情况            
            // // 清空环侧拐点（避免误判）
            // ring_bd.down_lp_state = false;
            // ring_bd.mid_lp_state = false;
            // ring_bd.up_lp_state = false;
            
            // 环侧清线，仅用于控制中线生成
            clear_line(ring_bd);
            
            // 直道侧是有效巡线侧，强制在对应半区（防串边）
            // clamp_half(other_bd, !ring_is_left); // 环内线乱飞？　应该是因为这个
            // fix_circle_inside_line(other_bd, !ring_is_left);

            //暂时禁止偏航角中线复位
            yaw_reset = false;

            // yaw_angle_get();
            float abs_yaw = std::fabs(yaw_angle);

            // 出环条件：转过230度 或 直道侧丢线（物理状态：接近出口）
            bool angle_ok = abs_yaw >= circle_param.exit_max_yaw;
            bool straight_lost_ok = other_raw.is_lost && (other_raw.lost_y >= 25);
            
            if (angle_ok || (abs_yaw >= circle_param.exit_min_yaw && straight_lost_ok)) {
                circle_state = CircleState::exit;
                sync_circle_track_mode(circle_state);
                circle_pass_count = 0;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_exit_start_yaw = abs_yaw;
                circle_inside_bottom_anchor = cv::Point(-1, -1);
                yaw_reset = false;  // 出环渐变拉线仍需要连续 yaw
            }
            
            fixed_C_pt = cv::Point(-1, -1);
            break;
        }

        case CircleState::exit:
        {
            // 出环：绕环侧巡线，直到直道侧恢复。
            // track_base.current_mode = ring_is_left ? TrackMode::left_full : TrackMode::right_full;
            // clear_line(other_bd);
            // clamp_half(ring_bd, ring_is_left);
            
            // // 清空拐点（避免误判）
            // ring_bd.down_lp_state = false;
            // ring_bd.mid_lp_state = false;
            // ring_bd.up_lp_state = false;
            // other_bd.down_lp_state = false;
            // other_bd.mid_lp_state = false;
            // other_bd.up_lp_state = false;

            // circle_pass_count++;  // 仅用于超时保护

            // 出环：使用固定斜率拉线引导车辆驶出
            // track_base.current_mode = TrackMode::auto_mode;
            track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
                        
            // 环侧边线补到边缘
            clear_line(ring_bd);
            
            // 出环时的 yaw 开始累计角度　　用于出环渐变和兜底保护
            float abs_yaw = std::fabs(yaw_angle);
            // float progress = (abs_yaw - circle_exit_start_yaw) / 85.0f;
            float exit_yaw = abs_yaw - circle_exit_start_yaw;

            //　出环拉线
            float progress = exit_yaw / circle_param.exit_yaw;
            if (progress < 0.0f) progress = 0.0f;
            if (progress > 1.0f) progress = 1.0f;

            float k_start = ring_is_left ? circle_param.exit_k_start : -circle_param.exit_k_start;
            float k_end = ring_is_left ? circle_param.exit_k_end : -circle_param.exit_k_end;
            float k = k_start + (k_end - k_start) * progress;
            int base_x = ring_is_left ? VALID_RIGHT_COL : VALID_LEFT_COL;

            for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
                int x = static_cast<int>(base_x + k * (y - VALID_START_ROW));
                other_bd.line[y] = clip(x, VALID_LEFT_COL, VALID_RIGHT_COL);
            }
            
            // 正常离开：达到目标角度且直道侧恢复稳定，连续确认后进入 leave
            // 强制离开：达到独立的 force yaw 参数后直接满足确认门槛，防止卡死
            bool exit_angle_ok = exit_yaw >= circle_param.exit_yaw;
            bool force_leave_by_yaw = exit_yaw >= circle_param.exit_force_yaw;
            bool other_recovered = (!other_raw.is_lost || (other_raw.is_lost && other_raw.lost_y <= 28)) &&
                                   other_raw.mnt_len >= 4;
            bool bottom_recovered = check_circle_bottom_recovered();

            // if (force_leave_by_yaw) {
            //     circle_state = CircleState::leave;
            //     circle_pass_count = 0;
            //     circle_exit_confirm_count = 0;
            //     circle_leave_confirm_count = 0;
            //     circle_seen_bottom_lost = false;
            //     // circle_confirm_count = 0;
            //     fixed_C_pt = cv::Point(-1, -1);
            //     circle_exit_start_yaw = 0.0f;
            //     motor_distance_clear();
            // }

            if ((exit_angle_ok && other_recovered && bottom_recovered) || force_leave_by_yaw) {
                circle_exit_confirm_count = force_leave_by_yaw ? 2 : circle_exit_confirm_count + 1;
            } 
            else {
                circle_exit_confirm_count = 0;
            }

            if (circle_exit_confirm_count >= 2) {
                circle_state = CircleState::leave;
                sync_circle_track_mode(circle_state);
                circle_pass_count = 0;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_seen_bottom_lost = false;
                // circle_confirm_count = 0;
                fixed_C_pt = cv::Point(-1, -1);
                circle_exit_start_yaw = 0.0f;
                motor_distance_clear();
            }
                        
            // // 超时保护：防止卡死在出环状态（兜底）
            // if (circle_pass_count >= 50) {
            //     circle_state = CircleState::leave;
            //     circle_pass_count = 0;
            //     fixed_C_pt = cv::Point(-1, -1);
            // }
            break;
        }

        case CircleState::leave:
        {
            circle_pass_count++;  // 仅用于超时保护

            // 离开：用直道侧斜率补环侧，恢复双边，减小单边推线的抖动
            track_base.current_mode = TrackMode::auto_mode;
            
            // // 清空环侧拐点（避免误判）
            // ring_bd.down_lp_state = false;
            // ring_bd.mid_lp_state = false;
            // ring_bd.up_lp_state = false;
            
            // 直道侧是有效巡线侧，强制在对应半区（防串边）
            clamp_half(other_bd, !ring_is_left);

            // 用直道侧斜率反向补全环侧
            bool other_line_ok = (!other_raw.is_lost || (other_raw.is_lost && other_raw.lost_y <= 28)) || other_raw.mnt_len >= 9;
            if (other_line_ok) {
                float straight_k = point_state.get_line_k(other_bd.line, VALID_END_ROW, VALID_START_ROW);
                if (std::abs(straight_k) > 2.5f) {
                    straight_k = (straight_k > 0.0f) ? 2.5f : -2.5f;
                }
                float ring_k = -straight_k;

                int last_x = -1;
                int last_y = -1;
                for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
                    int x = -1;
                    if (other_bd.line[y] != -1) {
                        x = ring_is_left ?
                            (int)std::round(other_bd.line[y] - TrackBase::halfRoad[y] * 2.0f) :
                            (int)std::round(other_bd.line[y] + TrackBase::halfRoad[y] * 2.0f);
                    } 
                    // 直道侧上方短断时，用直道侧斜率的反向斜率外推环侧虚拟线
                    else if (last_x != -1) {
                       
                        x = (int)std::round(last_x + ring_k * (y - last_y));
                    }

                    if (x != -1) {
                        x = clip(x, VALID_LEFT_COL, VALID_RIGHT_COL);
                        ring_bd.line[y] = x;
                        last_x = x;
                        last_y = y;
                    }
                    else {
                        ring_bd.line[y] = -1;
                    }
                }
                clamp_half(ring_bd, ring_is_left);
            }
            else {
                clear_line(ring_bd);
                track_base.current_mode = ring_is_left ? TrackMode::right_full : TrackMode::left_full;
            }
                        
            // 退出条件：当前只看编码器　　环侧不再丢线（之前）
            motor_distance_get();

            bool leave_distance_ok = motor_distance >= circle_param.leave_min_distance;
            if (leave_distance_ok) {
                circle_leave_confirm_count++;
            }
            else {
                circle_leave_confirm_count = 0;
            }

            if (circle_leave_confirm_count >= 3) {
                circle_state = CircleState::none;
                current_track_type = TrackType::normal;
                track_base.current_mode = TrackMode::auto_mode;
                circle_dir = CircleDir::none;
                approach_confirm_cnt = 0;
                circle_confirm_count = 0;
                circle_pass_count = 0;
                circle_seen_bottom_lost = false;
                circle_exit_confirm_count = 0;
                circle_leave_confirm_count = 0;
                circle_inside_bottom_anchor = cv::Point(-1, -1);
                fixed_C_pt = cv::Point(-1, -1);
                // last_circle_up_pt = cv::Point(-1, -1);
                // circle_up_stable_count = 0;
                circle_exit_start_yaw = 0.0f;
                yaw_reset = true;  //重启偏航角中线复位
                circle_run_index = advance_circle_index(circle_run_index);
            }
            
            // // 超时保护：防止卡死在离开状态
            // if (circle_pass_count > 80) {
            //     circle_state = CircleState::none;
            //     current_track_type = TrackType::normal;
            //     track_base.current_mode = TrackMode::auto_mode;
            //     circle_dir = CircleDir::none;
            //     approach_confirm_cnt = 0;
            //     circle_confirm_count = 0;
            //     circle_pass_count = 0;
            //     circle_seen_bottom_lost = false;
            //     circle_exit_confirm_count = 0;
            //     circle_leave_confirm_count = 0;
            //     circle_inside_bottom_anchor = cv::Point(-1, -1);
            //     fixed_C_pt = cv::Point(-1, -1);
            //     // last_circle_up_pt = cv::Point(-1, -1);
            //     // circle_up_stable_count = 0;
            //     circle_exit_start_yaw = 0.0f;
            //     yaw_reset = true;  //重启偏航角中线复位
            //     circle_run_index = advance_circle_index(circle_run_index);
            // }
            
            break;
        }

        default:
            circle_state = CircleState::none;
            current_track_type = TrackType::normal;
            track_base.current_mode = TrackMode::auto_mode;
            circle_dir = CircleDir::none;
            approach_confirm_cnt = 0;
            circle_confirm_count = 0;
            // circle_pass_count = 0;
            circle_seen_bottom_lost = false;
            fixed_C_pt = cv::Point(-1, -1);
            // last_circle_up_pt = cv::Point(-1, -1);
            // circle_up_stable_count = 0;
            circle_exit_start_yaw = 0.0f;
            madg.yaw_angle_reset(); //各角度和相关数据复位
            yaw_reset = true;  //重启偏航角中线复位
            break;
    }

    // 确保切换后状态持续生效
    if (circle_state != CircleState::leave) {
        sync_circle_track_mode(circle_state);
    }
}

// ==============
// 十字路口
// ==============

/**
 * @brief 十字路口检测 (目前只有正入)
 *
 * 条件：
 *   1.上/下角点同时存在 且行数差 < 15行（正入）或 拐点数 >= 3
 *   2.两边丢线 且丢线行坐标大  y > 70
 *
 * 分类：
 *   正入：双侧下拐点 + y差 < 10
 *   左斜入：只有右侧下拐点
 *   右斜入：只有左侧下拐点
 */
bool ElementState::crossroad_detect(CrossroadType &type)
{
    // 十字检测前先进行中下拐点融合
    auto fuse_mid_down = [](PointState::BoundaryData& bd) {
        const int merge_dist = 4;
        const int max_y_diff = 5;   // 优先级2：增加Y坐标差限制
        
        // 如果中下拐点距离很近，融合为下拐点
        if (bd.mid_lp_state && bd.down_lp_state) {
            int dx = std::abs(bd.mid_lp_pt.x - bd.down_lp_pt.x);
            int dy = std::abs(bd.mid_lp_pt.y - bd.down_lp_pt.y);
            // 距离近 且 Y坐标差不大（避免跨度太大的点被融合）
            if (dx <= merge_dist && dy <= max_y_diff) {
                // 取平均值作为融合后的下拐点
                bd.down_lp_pt.x = (bd.down_lp_pt.x + bd.mid_lp_pt.x) / 2;
                bd.down_lp_pt.y = (bd.down_lp_pt.y + bd.mid_lp_pt.y) / 2;
                bd.mid_lp_state = false;
                bd.mid_lp_pt = cv::Point(-1, -1);
            }
        }
        // 如果只有mid没有down，且mid在合理位置，提升为down
        else if (bd.mid_lp_state && !bd.down_lp_state && bd.mid_lp_pt.y >= VALID_END_ROW + 10) {
            bd.down_lp_state = true;
            bd.down_lp_pt = bd.mid_lp_pt;
            bd.mid_lp_state = false;
            bd.mid_lp_pt = cv::Point(-1, -1);
        }
    };
    
    fuse_mid_down(left_boundary);
    fuse_mid_down(right_boundary);

    // // 统计当前四个角点的数量
    // int corner_count = 0;
    // if (left_boundary.down_lp_state) corner_count++;
    // if (right_boundary.down_lp_state) corner_count++;
    // if (left_boundary.up_lp_state) corner_count++;
    // if (right_boundary.up_lp_state) corner_count++;

    // 特征判断
    // 上/下角点同时存在 且行数差 < 12行（正入）或 拐点数 >= 3
    // bool up_all = left_boundary.longest_white_up_state && right_boundary.longest_white_up_state && 
    //                std::abs(left_boundary.longest_white_up_pt.y - right_boundary.longest_white_up_pt.y) < 12;
    // bool down_all = left_boundary.longest_white_down_state && right_boundary.longest_white_down_state && 
    //                  std::abs(left_boundary.longest_white_down_pt.y - right_boundary.longest_white_down_pt.y) < 12;
                
    // 上/下角点同时存在 且行数差 < 12行（正入）或 拐点数 >= 3
    // bool up_all = left_boundary.up_lp_state && right_boundary.up_lp_state && 
    //                std::abs(left_boundary.up_lp_pt.y - right_boundary.up_lp_pt.y) < 15;
    bool down_all = left_boundary.down_lp_state && right_boundary.down_lp_state && 
                     std::abs(left_boundary.down_lp_pt.y - right_boundary.down_lp_pt.y) <= 8;
  
    // bool cond_1 = up_all || down_all || (corner_count >= 2);    // 条件一
    bool cond_1 =  down_all;    // 条件一

    // 两边丢线 且丢线行坐标在有效底部以下
    bool left_lost_ok_1 = left_boundary.is_lost && (left_boundary.lost_y >= 25);
    bool right_lost_ok_1 = right_boundary.is_lost && (right_boundary.lost_y >= 25);
    bool lost_all = std::abs(left_boundary.lost_y - right_boundary.lost_y) <= 8;
    
    bool cond_2 = left_lost_ok_1 && right_lost_ok_1 && lost_all;    // 条件二

    // int lost_threshold = 8;  // 丢线计数阈值
    // bool left_lost_ok_2  = left_boundary.lost_count >= lost_threshold;
    // bool right_lost_ok_2 = right_boundary.lost_count >= lost_threshold;
        
    const int lost_check_y = 15;        // 只统计45行以下的近车丢线，避免上方断线误触发十字
    const int lost_threshold = 5;       // 丢线计数阈值
    auto count_lower_lost = [](const PointState::BoundaryData& bd, int start_y) {
        int count = 0;
        int y0 = clip(start_y, VALID_END_ROW, VALID_START_ROW);
        for (int y = y0; y <= VALID_START_ROW; y++) {
            if (bd.line[y] == -1) {
                count++;
            }
        }
        return count;
    };

    int left_lower_lost_count = count_lower_lost(left_boundary, lost_check_y);
    int right_lower_lost_count = count_lower_lost(right_boundary, lost_check_y);
    bool left_lost_ok_2  = left_lower_lost_count >= lost_threshold;
    bool right_lost_ok_2 = right_lower_lost_count >= lost_threshold;

    bool double_lost = left_lost_ok_2 && right_lost_ok_2;    // 双丢线

    bool cond_3 = double_lost;    // 条件三


    // // 双下拐点 且间距满足
    // bool cond_double_down = left_boundary.down_lp_state && right_boundary.down_lp_state &&
    //                         (right_boundary.down_lp_pt.x - left_boundary.down_lp_pt.x >= 30);

    // // 角点总数 >= 3 且有丢线
    // bool cond_multi_corners = (corner_count >= 3) && (left_boundary.lost_count > 8 || right_boundary.lost_count > 8);

    // if (!cond_double_down && !cond_multi_corners) {
    //     return false;
    // }

    // // 下拐点绝不能在底部近处盲区（如果有下拐点的话）
    // if (left_boundary.down_lp_state && left_boundary.down_lp_pt.y > VALID_START_ROW - 5) return false;
    // if (right_boundary.down_lp_state && right_boundary.down_lp_pt.y > VALID_START_ROW - 5) return false;

    // // 右下拐点必须在右侧，左下拐点必须在左侧
    // if (left_boundary.down_lp_state && left_boundary.down_lp_pt.x > CAM_WIDTH / 2) return false;  
    // if (right_boundary.down_lp_state && right_boundary.down_lp_pt.x < CAM_WIDTH / 2) return false; 

    // 条件二成立　且条件一或三成立
    // if (!cond_2 || !cond_3) {
    if (!(cond_2 && (cond_1 || cond_3))) {
        return false;
    }

    // if (!cond_2) {
    //     return false;
    // }

    // // 分类
    // type = CrossroadType::straight;
    // if (left_boundary.down_lp_state && right_boundary.down_lp_state) {
    //     int dy = std::abs(left_boundary.down_lp_pt.y - right_boundary.down_lp_pt.y);
    //     if (dy < 10) {
    //         type = CrossroadType::straight;
    //     } 
    //     else if (left_boundary.down_lp_pt.y > right_boundary.down_lp_pt.y) {
    //         type = CrossroadType::right_diagonal;
    //     } 
    //     else {
    //         type = CrossroadType::left_diagonal;
    //     }
    // }

    return true;
}


/**
 * @brief 十字路口处理
 *
 * 状态一 approach：有上下拐点，则连线 若无，则用找到的上或者下角点，实时补线
 * 状态二 inside：上角点向下补线
 * 状态三 pass：退出
 *
 * 修改说明：
 * 1. approach -> inside 时清零编码器
 * 2. inside 状态增加编码器距离兜底退出
 * 3. inside 退出时清零编码器并恢复 auto_mode
 */
void ElementState::crossroad_process(const cv::Mat &bin_buf)
{
    if (bin_buf.empty())
        return;

    // // 误差跳变检测   暂未实现
    // float current_error = track_base.track_error;
    // float error_diff = std::abs(current_error - crossroad_last_error);
    
    // // 误差跳变超过10  即认为发生了跳变　　使用上一帧误差
    // if (error_diff > 10.0f) {
    //     track_base.track_error = crossroad_last_error;
    // }
    
    // // 更新上一帧误差
    // crossroad_last_error = track_base.track_error;

    // 退出十字路口，重置状态数据
    auto finish_crossroad = [&](bool clear_distance) {
        crossroad_state = CrossroadState::none;
        current_track_type = TrackType::normal;
        track_base.current_mode = TrackMode::auto_mode;
        track_base.lock_err_zero = false;
        crossroad_inside_anchor_locked = false;
        crossroad_left_bottom_anchor = cv::Point(-1, -1);
        crossroad_right_bottom_anchor = cv::Point(-1, -1);
        crossroad_left_quality = 0.0f;
        crossroad_right_quality = 0.0f;
        crossroad_pass_count = 0;
        if (clear_distance) {
            motor_distance_clear();
        }
    };

    // 十字状态一处理　延长十字下方两角点
    auto apply_crossroad_approach_process = [&]() {
        extend_line(left_boundary.line, true, true);
        extend_line(right_boundary.line, true, false);
    };

    // 十字状态二处理　固定底部锚点，向上延伸线段
    auto apply_crossroad_inside_process = [&]() {
        track_base.current_mode = TrackMode::auto_mode;

        // 固定底部锚点
        if (!crossroad_inside_anchor_locked) {
            crossroad_inside_anchor_locked = true;
            crossroad_left_bottom_anchor = cv::Point(20, VALID_START_ROW);
            crossroad_right_bottom_anchor = cv::Point(60, VALID_START_ROW);
        }

        bool left_line_ok = fix_crossroad_vertical_line(left_boundary, true);
        bool right_line_ok = fix_crossroad_vertical_line(right_boundary, false);

        // 十字内部不再使用单边推半宽：如果一侧补线失败，就用另一侧补好的斜率镜像出缺失侧
        auto calc_line_k = [](const int* line, float& k_out) -> bool {
            int top_y = -1;
            int bottom_y = -1;

            for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
                if (line[y] != -1) {
                    top_y = y;
                    break;
                }
            }

            for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
                if (line[y] != -1) {
                    bottom_y = y;
                    break;
                }
            }

            if (top_y == -1 || bottom_y == -1 || bottom_y <= top_y) {
                return false;
            }

            k_out = (float)(line[bottom_y] - line[top_y]) / (float)(bottom_y - top_y);
            return true;
        };

        // 如果有一侧补线失败，就用另一侧补好的斜率镜像出缺失侧
        auto mirror_line_from_other_side = [&](PointState::BoundaryData& target,
                                               bool target_is_left,
                                               float source_k) {
            const int x_mid = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
            cv::Point anchor = target_is_left ? crossroad_left_bottom_anchor
                                              : crossroad_right_bottom_anchor;
            float mirror_k = -source_k;

            for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
                int x = (int)std::round(anchor.x + mirror_k * (y - anchor.y));
                if (target_is_left) {
                    x = clip(x, VALID_LEFT_COL, x_mid + 5);
                } else {
                    x = clip(x, x_mid - 5, VALID_RIGHT_COL);
                }
                target.line[y] = x;
            }
        };

        // 如果一侧补线失败，就用另一侧补好的斜率镜像出缺失侧
        if (left_line_ok && !right_line_ok) {
            float left_k = 0.0f;
            if (calc_line_k(left_boundary.line, left_k)) {
                mirror_line_from_other_side(right_boundary, false, left_k);
                right_line_ok = true;
                crossroad_right_quality = crossroad_left_quality;
            }
        }
        else if (!left_line_ok && right_line_ok) {
            float right_k = 0.0f;
            if (calc_line_k(right_boundary.line, right_k)) {
                mirror_line_from_other_side(left_boundary, true, right_k);
                left_line_ok = true;
                crossroad_left_quality = crossroad_right_quality;
            }
        }

        if (!(left_line_ok && right_line_ok)) {
            crossroad_left_quality = 0.0f;
            crossroad_right_quality = 0.0f;
        }
    };

    // 过渡阶段：决定状态并重置状态数据
    switch (crossroad_state) {
        case CrossroadState::approach:
        {
            if (!left_boundary.is_lost && !right_boundary.is_lost) {
                finish_crossroad(false);
                break;
            }

            bool lost_bottom = (left_boundary.is_lost && left_boundary.lost_y >= VALID_BOTTOM_ROW - 2) &&
                               (right_boundary.is_lost && right_boundary.lost_y >= VALID_BOTTOM_ROW - 2);

            if (lost_bottom) {
                crossroad_state = CrossroadState::inside;
                crossroad_inside_anchor_locked = false;
                crossroad_left_bottom_anchor = cv::Point(-1, -1);
                crossroad_right_bottom_anchor = cv::Point(-1, -1);
                crossroad_pass_count = 0;
                motor_distance_clear();  // 清零编码器
            }
            break;
        }

        case CrossroadState::inside:
        {
            bool left_recovered = !left_boundary.is_lost || (left_boundary.is_lost && left_boundary.lost_y <= 30);
            bool right_recovered = !right_boundary.is_lost || (right_boundary.is_lost && right_boundary.lost_y <= 30);

            motor_distance_get();

            bool exit_ok = (left_recovered || right_recovered) || check_bottom_recovered();
            bool distance_exit_ok = motor_distance >= 1.0f;

            if (exit_ok || distance_exit_ok) {
                crossroad_pass_count++;
                if (crossroad_pass_count >= 3) {
                    finish_crossroad(true);
                }
            }
            else {
                crossroad_pass_count = 0;
            }
            break;
        }

        default:
            finish_crossroad(false);
            break;
    }

    // 切换状态机后　处理操作
    switch (crossroad_state) {
        case CrossroadState::approach:
            apply_crossroad_approach_process();
            break;

        case CrossroadState::inside:
            apply_crossroad_inside_process();
            break;

        default:
            break;
    }
}

// ==============
// 路障
// ==============

/**
 * @brief 路障检测 — 在赛道两侧找砖红色色块
 * 
 * 区分路障和目标板前红色矩形：
 *   路障一定在赛道两侧（距中心 >= 10cm），红色矩形一定在赛道正中间
 *   所以只要红色色块中心偏离赛道中线足够远，就认为是路障
 */
bool ElementState::barrier_detect(const cv::Mat &rgb_img, BarrierDir &dir)
{
    if (rgb_img.empty()) return false;

    // 色块检测 (红色)
    cv::Mat hsv, mask, mask1, mask2;
    cv::cvtColor(rgb_img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // 轮廓检测 筛选
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    cv::Rect best_rect;
    bool found = false;

    for (const auto &cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < 1200) continue;           // 过滤噪声

        cv::Rect rect = cv::boundingRect(cnt);
        int cx = rect.x + rect.width / 2;
        int cy = rect.y + rect.height / 2;

        // 必须在画面下半部分
        if (cy < ROI_HEIGHT / 2) continue;

        // 色块中心必须偏离赛道中线 (核心区分条件)
        int offset_from_center = std::abs(cx - CAM_WIDTH / 2);
        if (offset_from_center < 10) continue;

        if (area > max_area) {
            max_area = area;
            best_rect = rect;
            found = true;
        }
    }

    if (!found) return false;

    // 判断路障方向
    int cx = best_rect.x + best_rect.width / 2;
    if (cx < CAM_WIDTH / 2) {
        dir = BarrierDir::left;  
    } 
    else {
        dir = BarrierDir::right; 
    }
    return true;
}

/**
 * @brief 路障处理 — 中线偏移 + 编码器计距退出
 * 
 * avoiding: 每帧给 mid.line[] 加偏移，编码器计距
 */
void ElementState::barrier_process(const cv::Mat &rgb_img)
{
    switch (barrier_state) {

    case BarrierState::avoiding: {

        // 每帧给中线加偏移
        for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
            if (mid.line[y] != -1) {
                mid.line[y] = clip(mid.line[y] + barrier_offset, 0, CAM_WIDTH - 1);
            }
        }

        // 路障长 24cm + 车身，大约需要走 45cm 1500
        if (motor_distance > 1500) {

            // 避障完成
            barrier_state = BarrierState::none;
            barrier_dir = BarrierDir::none;
            barrier_offset = 0;
            current_track_type = TrackType::normal;
        }
        break;
    }

    default:
            barrier_state = BarrierState::none;
            barrier_dir = BarrierDir::none;
            barrier_offset = 0;
            current_track_type = TrackType::normal;
        break;
    }
}

// ==============
// 坡道 (纯IMU pitch角度检测)
// ==============
/**
 * @brief 坡道检测
 * 
 * pitch >= 8° → 上坡
 */
bool ElementState::ramp_detect(void)
{
    // 读取当前 pitch 角度
    float now_pitch = pitch_angle; 

    // pitch 超过阈值  (双边直道　暂不开启)
    // if (now_pitch >= 8.0f &&
    //     !left_boundary.is_lost && !right_boundary.is_lost) {
    //     ramp_up_confirm_count++;
    // }
    if (now_pitch >= 8.0f) {
        ramp_up_confirm_count++;
    }
    else {
        ramp_up_confirm_count = 0;
    }

    return ramp_up_confirm_count >= 3;
}

/**
 * @brief 坡道处理
 * climbing: pitch >= 8°  → 上坡   (减速)
 * ending:   pitch <= -5° → 下坡  恢复速度
 */
void ElementState::ramp_process(void)
{
    float now_pitch = pitch_angle;

    // 坡道状态机
    switch (ramp_state) 
    {
        // 检测到:　初始化一下
        case RampState::detected:
            ramp_up_confirm_count = 0;
            ramp_down_confirm_count = 0;
            ramp_flat_confirm_count = 0;
            motor_distance_clear();
            ramp_state = RampState::climbing;
            break;

        // 上坡中：减速 (待速度决策分配速度)
        case RampState::climbing:

            // // 上坡中：减速
            // car_speed = car_speed / 2; 

            // pitch 变负且编码器距离足够 → 开始下坡　(待后续细分状态机)
            motor_distance_get();
            if (now_pitch <= -5.0f && motor_distance >= 2.5f) {
                ramp_down_confirm_count++;
            }
            else {
                ramp_down_confirm_count = 0;
            }

            if (ramp_down_confirm_count >= 2) {
                ramp_state = RampState::ending;
                ramp_flat_confirm_count = 0;
            }
            break;

        // 坡道结束：恢复速度　屏蔽圆环
        case RampState::ending:

            // pitch 回到接近 0°且编码器距离足够 → 退出坡道
            motor_distance_get();
            if (std::abs(now_pitch) <= 5.0f && motor_distance >= 5.0f) {
                ramp_flat_confirm_count++;
            }
            else {
                ramp_flat_confirm_count = 0;
            }

            if (ramp_flat_confirm_count >= 2) {
                ramp_state = RampState::none;
                ramp_circle_shield_count = 25;
                ramp_up_confirm_count = 0;
                ramp_down_confirm_count = 0;
                ramp_flat_confirm_count = 0;
                current_track_type = TrackType::normal;
            }
            break;

        default:
            ramp_state = RampState::none;
            ramp_up_confirm_count = 0;
            ramp_down_confirm_count = 0;
            ramp_flat_confirm_count = 0;
            current_track_type = TrackType::normal;
            break;
    }
}

// ==============
// 斑马线
// ==============
/**
 * @brief 斑马线处理函数
 * @param bin_buf 输入的二值化图像 track_base.bin_buf
 */
void ElementState::zebra_process(const cv::Mat &bin_buf)
{
    if (bin_buf.empty())
        return;

    // 如果处于屏蔽状态，直接跳回 normal 并退出
    if (is_shielded)
    {
        zebra_state = ZebraState::shielded;
        current_track_type = TrackType::normal;
        return;
    }

    // 斑马线检测逻辑
    bool is_detected = zebra_detect(bin_buf);

    if (is_detected)
    {
        zebra_confirm_count++;

        // 连续3帧检测到 确认是斑马线
        if (zebra_confirm_count >= 2)
        {
            // 只在第一次确认时发停车指令
            if (zebra_state != ZebraState::detected)
            {
                zebra_state = ZebraState::detected;
                car_soft_stop();
                Gogo = false;
            }
        }
    }
    else
    {
        // 再次检测到
        if (zebra_state == ZebraState::detected)
        {
            is_shielded = true;
            zebra_state = ZebraState::shielded;
            motor_distance_clear();
            current_track_type = TrackType::normal;
        }
        else
        {
            current_track_type = TrackType::normal;
        }
        zebra_confirm_count = 0;
    }
}

/**
 * @brief 斑马线检测函数
 * @param bin_buf 输入的二值化图像
 * @return 返回true表示当前帧检测到斑马线特征
 */
bool ElementState::zebra_detect(const cv::Mat &bin_buf)
{
    int target_transitions = 4; // 阈值：单行竖向连续黑块数大于4认为有斑马线特征

    // 收集每行的跳变次数，用于连续性和方差检测
    int trans_arr[CAM_HEIGHT];
    int trans_count = 0;

    // 连续满足条件的行数
    int consecutive = 0;
    int max_consecutive = 0;

    // 方差计算用
    float sum_t = 0;
    float sum_t2 = 0;
    int match_total = 0;

    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        int transitions = count_transitions(bin_buf, y);
        trans_arr[trans_count++] = transitions;

        if (transitions > target_transitions) {
            consecutive++;
            if (consecutive > max_consecutive) {
                max_consecutive = consecutive;
            }
            // 累加用于方差计算
            sum_t += transitions;
            sum_t2 += transitions * transitions;
            match_total++;
        } else {
            consecutive = 0;
        }
    }

    // 条件1：必须有连续 3+ 行满足跳变条件
    if (max_consecutive < 3)
        return false;

    // 条件2：满足条件的行的跳变次数方差要小（斑马线每行跳变数接近）
    if (match_total >= 5) {
        float mean = sum_t / match_total;
        float var = sum_t2 / match_total - mean * mean;
        if (var > 6.0f)  // 方差大于6说明跳变数差异大，不像斑马线
            return false;
    }

    return true;
}


/**
 * @brief 斑马线屏蔽函数
 * @param distance 当前车辆运行距离 (如编码器积分)
 * @param max_distance 解除屏蔽的距离阈值
 * @note  起步时默认屏蔽斑马线，只有当车辆行驶超过 max_distance 后才解除屏蔽，开始检测斑马线
 * @note  暂时未使用 等之后添加使用
 */
void ElementState::zebra_shield(float distance, float max_distance)
{
    // 当处于屏蔽状态，且行驶距离超过阈值时，解除屏蔽
    if (is_shielded && distance > max_distance)
    {
        is_shielded = false;
        zebra_state = ZebraState::none;
        std::cout << "屏蔽斑马线解除" << std::endl;
    }
    else if (is_shielded)
    {
        zebra_state = ZebraState::shielded;
        zebra_confirm_count = 0;
    }
}

// 进入模型前保存外部元素快照
void ElementState::enter_model_context(void)
{
    if (!model_line_pass_snapshot_valid && current_track_type != TrackType::model) {
        model_line_pass_snapshot.current_track_type = current_track_type;
        model_line_pass_snapshot.current_mode = track_base.current_mode;
        model_line_pass_snapshot.normal_type = normal_type;
        model_line_pass_snapshot.normal_road_state = normal_road_state;
        model_line_pass_snapshot.circle_state = circle_state;
        model_line_pass_snapshot.circle_dir = circle_dir;
        model_line_pass_snapshot.approach_confirm_cnt = approach_confirm_cnt;
        model_line_pass_snapshot.circle_confirm_count = circle_confirm_count;
        model_line_pass_snapshot.circle_pass_count = circle_pass_count;
        model_line_pass_snapshot.circle_seen_bottom_lost = circle_seen_bottom_lost;
        model_line_pass_snapshot.fixed_C_pt = fixed_C_pt;
        model_line_pass_snapshot.circle_exit_start_yaw = circle_exit_start_yaw;
        model_line_pass_snapshot.circle_run_index = circle_run_index;
        model_line_pass_snapshot.circle_exit_confirm_count = circle_exit_confirm_count;
        model_line_pass_snapshot.circle_leave_confirm_count = circle_leave_confirm_count;
        model_line_pass_snapshot.circle_inside_bottom_anchor = circle_inside_bottom_anchor;
        model_line_pass_snapshot.crossroad_state = crossroad_state;
        model_line_pass_snapshot.crossroad_type = crossroad_type;
        model_line_pass_snapshot.crossroad_confirm_count = crossroad_confirm_count;
        model_line_pass_snapshot.crossroad_pass_count = crossroad_pass_count;
        model_line_pass_snapshot.crossroad_inside_anchor_locked = crossroad_inside_anchor_locked;
        model_line_pass_snapshot.crossroad_left_bottom_anchor = crossroad_left_bottom_anchor;
        model_line_pass_snapshot.crossroad_right_bottom_anchor = crossroad_right_bottom_anchor;
        model_line_pass_snapshot.crossroad_left_quality = crossroad_left_quality;
        model_line_pass_snapshot.crossroad_right_quality = crossroad_right_quality;
        model_line_pass_snapshot.crossroad_last_error = crossroad_last_error;
        model_line_pass_snapshot.barrier_state = barrier_state;
        model_line_pass_snapshot.barrier_dir = barrier_dir;
        model_line_pass_snapshot.barrier_confirm_count = barrier_confirm_count;
        model_line_pass_snapshot.barrier_offset = barrier_offset;
        model_line_pass_snapshot.ramp_state = ramp_state;
        model_line_pass_snapshot.ramp_circle_shield_count = ramp_circle_shield_count;
        model_line_pass_snapshot.ramp_up_confirm_count = ramp_up_confirm_count;
        model_line_pass_snapshot.ramp_down_confirm_count = ramp_down_confirm_count;
        model_line_pass_snapshot.ramp_flat_confirm_count = ramp_flat_confirm_count;
        model_line_pass_snapshot.zebra_state = zebra_state;
        model_line_pass_snapshot.zebra_confirm_count = zebra_confirm_count;
        model_line_pass_snapshot.is_shielded = is_shielded;
        model_line_pass_snapshot.straight_confirm_count = straight_confirm_count;
        model_line_pass_snapshot.curve_confirm_count = curve_confirm_count;
        model_line_pass_snapshot.curve_score = curve_score;
        model_line_pass_snapshot.last_left_k = last_left_k;
        model_line_pass_snapshot.last_right_k = last_right_k;
        model_line_pass_snapshot.k_init = k_init;
        model_line_pass_snapshot.motor_distance = motor_distance;
        model_line_pass_snapshot_valid = true;
    }

    current_track_type = TrackType::model;
}

// 模型结束后恢复快照
void ElementState::restore_model_context(void)
{
    model_line_pass_active = false;
    model_line_pass_dir = ModelLinePassDir::none;
    continue_pass = false;

    // 还原为绕行前状态
    if (model_line_pass_snapshot_valid) {
        current_track_type = model_line_pass_snapshot.current_track_type;
        track_base.current_mode = model_line_pass_snapshot.current_mode;
        normal_type = model_line_pass_snapshot.normal_type;
        normal_road_state = model_line_pass_snapshot.normal_road_state;
        circle_state = model_line_pass_snapshot.circle_state;
        circle_dir = model_line_pass_snapshot.circle_dir;
        approach_confirm_cnt = model_line_pass_snapshot.approach_confirm_cnt;
        circle_confirm_count = model_line_pass_snapshot.circle_confirm_count;
        circle_pass_count = model_line_pass_snapshot.circle_pass_count;
        circle_seen_bottom_lost = model_line_pass_snapshot.circle_seen_bottom_lost;
        fixed_C_pt = model_line_pass_snapshot.fixed_C_pt;
        circle_exit_start_yaw = model_line_pass_snapshot.circle_exit_start_yaw;
        circle_run_index = model_line_pass_snapshot.circle_run_index;
        circle_exit_confirm_count = model_line_pass_snapshot.circle_exit_confirm_count;
        circle_leave_confirm_count = model_line_pass_snapshot.circle_leave_confirm_count;
        circle_inside_bottom_anchor = model_line_pass_snapshot.circle_inside_bottom_anchor;
        crossroad_state = model_line_pass_snapshot.crossroad_state;
        crossroad_type = model_line_pass_snapshot.crossroad_type;
        crossroad_confirm_count = model_line_pass_snapshot.crossroad_confirm_count;
        crossroad_pass_count = model_line_pass_snapshot.crossroad_pass_count;
        crossroad_inside_anchor_locked = model_line_pass_snapshot.crossroad_inside_anchor_locked;
        crossroad_left_bottom_anchor = model_line_pass_snapshot.crossroad_left_bottom_anchor;
        crossroad_right_bottom_anchor = model_line_pass_snapshot.crossroad_right_bottom_anchor;
        crossroad_left_quality = model_line_pass_snapshot.crossroad_left_quality;
        crossroad_right_quality = model_line_pass_snapshot.crossroad_right_quality;
        crossroad_last_error = model_line_pass_snapshot.crossroad_last_error;
        barrier_state = model_line_pass_snapshot.barrier_state;
        barrier_dir = model_line_pass_snapshot.barrier_dir;
        barrier_confirm_count = model_line_pass_snapshot.barrier_confirm_count;
        barrier_offset = model_line_pass_snapshot.barrier_offset;
        ramp_state = model_line_pass_snapshot.ramp_state;
        ramp_circle_shield_count = model_line_pass_snapshot.ramp_circle_shield_count;
        ramp_up_confirm_count = model_line_pass_snapshot.ramp_up_confirm_count;
        ramp_down_confirm_count = model_line_pass_snapshot.ramp_down_confirm_count;
        ramp_flat_confirm_count = model_line_pass_snapshot.ramp_flat_confirm_count;
        zebra_state = model_line_pass_snapshot.zebra_state;
        zebra_confirm_count = model_line_pass_snapshot.zebra_confirm_count;
        is_shielded = model_line_pass_snapshot.is_shielded;
        straight_confirm_count = model_line_pass_snapshot.straight_confirm_count;
        curve_confirm_count = model_line_pass_snapshot.curve_confirm_count;
        curve_score = model_line_pass_snapshot.curve_score;
        last_left_k = model_line_pass_snapshot.last_left_k;
        last_right_k = model_line_pass_snapshot.last_right_k;
        k_init = model_line_pass_snapshot.k_init;
        motor_distance = model_line_pass_snapshot.motor_distance;
    }
    else if (current_track_type == TrackType::model) {
        current_track_type = TrackType::normal;
        track_base.current_mode = TrackMode::auto_mode;
        normal_road_state = NormalRoadState::safe;
        normal_type = NormalType::curve;
        motor_distance_clear();
    }

    model_line_pass_snapshot_valid = false;
}

// 模型绕行开始
void ElementState::start_model_line_pass(ModelLinePassDir dir, float distance)
{
    if (dir == ModelLinePassDir::none || distance <= 0.0f) return;
    if (model_line_pass_active) return;

    // 快照应该在进入模型大状态前保存　　如果没有提前保存，这里兜底保存一次，但不覆盖已有快照
    if (!model_line_pass_snapshot_valid) {
        enter_model_context();
    }

    // 初始化
    model_line_pass_active = true;
    model_line_pass_dir = dir;
    model_line_pass_distance = distance;
    
    current_track_type = TrackType::model;
    normal_type = NormalType::curve;
    normal_road_state = NormalRoadState::curve;

    track_base.current_mode = (dir == ModelLinePassDir::left) ? TrackMode::left_raw : TrackMode::right_raw;
    motor_distance_clear();

    // 误差过渡机制
    slow_frame_pass = true;  //允许跳过错误帧
    slow_frame_pass_time = SLOW_PASS_TIME;    //刷新错误帧过渡时间
    std::cout << "开启误差过渡" << std::endl;

    // 串口打印
    std::cout << "[模型绕行] 开始 "
              << ((dir == ModelLinePassDir::left) ? "LEFT_RAW" : "RIGHT_RAW")
              << " distance=" << distance << std::endl;
}

// 模型绕行控制结束
void ElementState::stop_model_line_pass(void)
{
    // 第一次调用：passing 编码器到距离，只结束 raw 绕行并进入 returning，不恢复快照
    if (model_line_pass_active) {
        std::cout << "[模型绕行] 结束，进入 returning" << std::endl;
        model_line_pass_active = false;
        model_line_pass_dir = ModelLinePassDir::none;
        current_track_type = TrackType::model;
        track_base.current_mode = TrackMode::auto_mode;
        continue_pass = true;
        return;
    }

    // 第二次调用：returning 编码器到距离，才恢复进入模型前的元素快照
    restore_model_context();
}

///////////////////////////////////////////////// 辅助函数 //////////////////////////////////////////////
/**
 * @brief 计算单行竖向连续黑块数量
 * @param bin_buf 输入的二值化图像
 * @param y 要检测的行号
 * @return int 竖向连续黑块数量
 * @note 检测白→黑跳变，且上下行对应位置也是黑色，确保是竖向条纹而非噪点
 */
int ElementState::count_transitions(const cv::Mat &bin_buf, int y)
{
    if (y <= 0 || y >= bin_buf.rows - 1) return 0;

    const uchar *row_cur  = bin_buf.ptr<uchar>(y);
    const uchar *row_up   = bin_buf.ptr<uchar>(y - 1);
    const uchar *row_down = bin_buf.ptr<uchar>(y + 1);

    int count = 0;

    // 在左右边线范围内扫描
    for (int x = 1; x < bin_buf.cols - 1; x++) {
        // 白→黑跳变，且上下行该位置也是黑色（竖向连续黑块）
        if (row_cur[x] > 127 && row_cur[x + 1] <= 127 &&
            (row_up[x + 1] <= 127 && row_down[x + 1] <= 127)) {
            count++;
            // 跳过当前黑块，避免重复或噪点干扰
            while(x < bin_buf.cols - 1 && row_cur[x + 1] <= 127) {
                x++;
            }
        }
    }
    return count;
}


/**
 * @brief 在一维数组上插值补线 (两点连线)
 * @param line_arr 需要修改的有效边界数组 (left_boundary.line)
 * @param p1 连线起点
 * @param p2 连线终点
 * @note 　上下拐点相连
 */
void ElementState::connect_points(int *line_arr, const cv::Point &p1, const cv::Point &p2)
{
    // 从画面下方往上方连线
    int y_start = std::max(p1.y, p2.y);
    int y_end = std::min(p1.y, p2.y);

    y_start = std::min(y_start, CAM_HEIGHT - 1);
    y_end = std::max(y_end, 0);

    if (y_start < y_end)
        return;         // 防止数据崩溃

    // 处理水平连线/单点特殊情况 (斜率无穷大)
    if (y_start == y_end) {
        int x = std::min(std::max(p1.x, 0), CAM_WIDTH - 1);
        line_arr[y_start] = x;
        return;
    }

    // 计算直线方程: x = ky + b
    float k = (float)(p2.x - p1.x) / (p2.y - p1.y);
    float b = p1.x - k * p1.y;

    // 覆写有效边界数组 line 里的缺口
    for (int y = y_start; y >= y_end; y--)
    {
        int x = (int)std::round(k * y + b); // std::round用以四舍五入
        // 越界保护
        if (x < 0)
            x = 0;
        if (x >= CAM_WIDTH)
            x = CAM_WIDTH - 1;

        line_arr[y] = x;
    }
}


/**
 * @brief 基于下拐点下方的直线趋势，向上方发射射线补线
 * @param line_arr 需要修改的有效边界数组
 * @param corner_pt 找到的下拐点
 * @param is_left 是否为左边线
 */
void ElementState::extend_line_up(int *line_arr, const cv::Point &corner_pt, bool is_left)
{
    if (corner_pt.y < 0 || corner_pt.y >= CAM_HEIGHT) return;

    int y_base1 = corner_pt.y + 2;
    int y_base2 = corner_pt.y + 5;

    int max_valid = std::min((int)VALID_START_ROW, CAM_HEIGHT - 1);
    if (y_base2 >= max_valid)
        y_base2 = max_valid;

    int safe_y_base1 = std::max(0, std::min(y_base1, CAM_HEIGHT - 1));
    int safe_y_base2 = std::max(0, std::min(y_base2, CAM_HEIGHT - 1));

    // 优先：尝试用±3和±7两点拟合
    if (y_base1 < y_base2 && line_arr[safe_y_base1] != -1 && line_arr[safe_y_base2] != -1)
    {
        float k = (float)(line_arr[safe_y_base1] - line_arr[safe_y_base2]) / (safe_y_base1 - safe_y_base2);
        float b = line_arr[safe_y_base1] - k * safe_y_base1;

        // 斜率异常保护
        if (std::abs(k) <= 3.0f)
        {
            // 斜率合理，使用两点拟合结果
            for (int y = std::min(corner_pt.y, CAM_HEIGHT - 1); y > VALID_END_ROW && y >= 0; y--)
            {
                int x = (int)std::round(k * y + b);

                // 左线绝不能过中线右侧，右线绝不能过中线左侧
                if (is_left)
                {
                    if (x > CAM_WIDTH / 2 + 3)
                        x = CAM_WIDTH / 2 + 3;
                }
                else
                {
                    if (x < CAM_WIDTH / 2 - 3)
                        x = CAM_WIDTH / 2 - 3;
                }

                if (x < 0)
                    x = 0;
                if (x >= CAM_WIDTH)
                    x = CAM_WIDTH - 1;
                line_arr[y] = x;
            }
            return;
        }
    }

    // 降级：尝试用拐点和+3点拟合（两点拟合）
    if (line_arr[safe_y_base1] != -1)
    {
        float k = (float)(line_arr[safe_y_base1] - corner_pt.x) / (safe_y_base1 - corner_pt.y);
        
        // 斜率合理性检查
        if (std::abs(k) <= 3.0f)
        {
            float b = corner_pt.x - k * corner_pt.y;
            
            for (int y = std::min(corner_pt.y, CAM_HEIGHT - 1); y > VALID_END_ROW && y >= 0; y--)
            {
                int x = (int)std::round(k * y + b);

                if (is_left)
                {
                    if (x > CAM_WIDTH / 2 + 3)
                        x = CAM_WIDTH / 2 + 3;
                }
                else
                {
                    if (x < CAM_WIDTH / 2 - 3)
                        x = CAM_WIDTH / 2 - 3;
                }

                if (x < 0)
                    x = 0;
                if (x >= CAM_WIDTH)
                    x = CAM_WIDTH - 1;
                line_arr[y] = x;
            }
            return;
        }
    }

    // // 最后：垂直补线（注意：需要两边一起垂直补线才调用）
    // for (int y = std::min(corner_pt.y, CAM_HEIGHT - 1); y > VALID_END_ROW && y >= 0; y--) {
    //     line_arr[y] = corner_pt.x; 
    // }
}

/**
 * @brief 从角点往下方延伸补线 (用角点上方数据拟合斜率)
 * @param line_arr 边界数组
 * @param corner_pt 角点坐标 (上拐点)
 * @param is_left 是否为左边线
 */
void ElementState::extend_line_down(int *line_arr, const cv::Point &corner_pt, bool is_left)
{
    if (corner_pt.y < 0 || corner_pt.y >= CAM_HEIGHT) return;

    // 从角点上方取数据拟合斜率
    int y_base1 = corner_pt.y - 2;
    int y_base2 = corner_pt.y - 5;   

    int min_valid = std::max((int)VALID_END_ROW, 0);
    if (y_base2 < min_valid) y_base2 = min_valid;
    if (y_base1 < min_valid) y_base1 = min_valid;

    int safe_y_base1 = std::max(0, std::min(y_base1, CAM_HEIGHT - 1));
    int safe_y_base2 = std::max(0, std::min(y_base2, CAM_HEIGHT - 1));

    // 优先：尝试用-3和-7两点拟合
    if (y_base1 > y_base2 && line_arr[safe_y_base1] != -1 && line_arr[safe_y_base2] != -1) {
        float k = (float)(line_arr[safe_y_base1] - line_arr[safe_y_base2]) / (safe_y_base1 - safe_y_base2);
        float b = line_arr[safe_y_base1] - k * safe_y_base1;

        // 斜率异常保护
        if (std::abs(k) <= 3.0f) {
            // 斜率合理，使用两点拟合结果
            for (int y = corner_pt.y; y <= VALID_START_ROW; y++) {
                int x = (int)std::round(k * y + b);
                if (is_left) {
                    if (x > CAM_WIDTH / 2 + 5) x = CAM_WIDTH / 2 + 3;
                } else {
                    if (x < CAM_WIDTH / 2 - 5) x = CAM_WIDTH / 2 - 3;
                }
                if (x < 0) x = 0;
                if (x >= CAM_WIDTH) x = CAM_WIDTH - 1;
                line_arr[y] = x;
            }
            return;
        }
    }

    // 降级：尝试用拐点和-3点拟合（两点拟合）
    if (line_arr[safe_y_base1] != -1) {
        float k = (float)(line_arr[safe_y_base1] - corner_pt.x) / (safe_y_base1 - corner_pt.y);
        
        // 斜率合理性检查
        if (std::abs(k) <= 3.0f) {
            float b = corner_pt.x - k * corner_pt.y;
            
            for (int y = corner_pt.y; y <= VALID_START_ROW; y++) {
                int x = (int)std::round(k * y + b);
                if (is_left) {
                    if (x > CAM_WIDTH / 2 + 5) x = CAM_WIDTH / 2 + 3;
                } else {
                    if (x < CAM_WIDTH / 2 - 5) x = CAM_WIDTH / 2 - 3;
                }
                if (x < 0) x = 0;
                if (x >= CAM_WIDTH) x = CAM_WIDTH - 1;
                line_arr[y] = x;
            }
            return;
        }
    }

//     // 最后：垂直补线（注意：需要两边一起垂直补线才调用）
//     for (int y = corner_pt.y; y <= VALID_START_ROW; y++) {
//         line_arr[y] = corner_pt.x;
//     }
}

/**
 * @brief 使用最长连续线段进行最小二乘拟合并延伸补线
 * 
 * 核心改进：
 * 1. 合并上下方向的记忆（左右边界各一组参数，不区分上下）
 * 2. 找不到线时使用历史参数强行补线（惯性保底机制）
 * 3. 十字状态下全段贯通补线，避免状态切换时的断档
 * 
 * @param boundary_line 边界线数组
 * @param is_up true=向上延伸, false=向下延伸
 * @param is_left true=左边界, false=右边界
 * @return 是否成功补线
 */
bool ElementState::extend_line(int* boundary_line, bool is_up, bool is_left)
{
    // 存储上一帧的拟合参数（左右边界分别维护）
    static float last_k_left = 0.0f, last_b_left = 0.0f;
    static float last_k_right = 0.0f, last_b_right = 0.0f;
    static bool first_frame_left = true;
    static bool first_frame_right = true;
    
    const float rate = 0.7f;  // 滤波系数
    
    // 十字状态下的边界点过滤（在寻找最长线段之前）
    if (current_track_type == TrackType::crossroad) {
        // X坐标突变过滤：截断突变点及以上所有点（从下往上扫描，遇到突变就截断到顶）
        const int x_jump_threshold = 15;  // X坐标突变阈值（放宽到30，避免误判正常边界）
        for (int y = VALID_START_ROW - 1; y >= VALID_END_ROW; y--) {
            if (boundary_line[y] != -1 && boundary_line[y+1] != -1) {
                int x_diff = std::abs(boundary_line[y] - boundary_line[y+1]);
                if (x_diff > x_jump_threshold) {
                    // 截断突变点及以上所有点（从突变点到顶部全部设为-1）
                    for (int yy = y; yy >= VALID_END_ROW; yy--) {
                        boundary_line[yy] = -1;
                    }
                    // 因为可能存在多个突变，为了安全只处理第一次遇到的突变（最底下的突变）
                    break;
                }
            }
        }
    }
    
    // 寻找最长的连续线段（Y允许间隔<=3行，X差值<5像素）
    int best_start = -1, best_end = -1, best_len = 0;
    int current_start = -1, current_end = -1, current_len = 0;
    int last_y = -1, last_x = -1;
    
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        if (boundary_line[y] != -1) {
            int curr_x = boundary_line[y];
            
            if (current_start == -1) {
                // 开始新线段
                current_start = y;
                current_end = y;
                current_len = 1;
                last_y = y;
                last_x = curr_x;
            } 
            else {
                int y_gap = y - last_y;
                int x_diff = std::abs(curr_x - last_x);
                
                // 检查是否可以延续当前线段：Y间隔<=3 且 X差值<5
                if (y_gap <= 2 && x_diff < 4) {
                    // 延续当前线段
                    current_end = y;
                    current_len++;
                    last_y = y;
                    last_x = curr_x;
                } 
                else {
                    // 不能延续，检查当前线段是否有效
                    // 十字状态下：过滤横向线段（Y变化必须大于X变化）
                    bool is_valid_segment = true;
                    if (current_track_type == TrackType::crossroad && current_len >= 2) {
                        int y_range = current_end - current_start;
                        int x_range = std::abs(boundary_line[current_end] - boundary_line[current_start]);
                        // 如果是横向线段（Y变化 < X变化 * 0.5），则无效
                        if (y_range < x_range * 0.5) {
                            is_valid_segment = false;
                        }
                    }
                    
                    // 保存有效线段
                    if (is_valid_segment && current_len > best_len) {
                        best_start = current_start;
                        best_end = current_end;
                        best_len = current_len;
                    }
                    
                    // 开始新线段
                    current_start = y;
                    current_end = y;
                    current_len = 1;
                    last_y = y;
                    last_x = curr_x;
                }
            }
        }
    }
    
    // 检查最后一段
    bool is_valid_segment = true;
    if (current_track_type == TrackType::crossroad && current_len >= 2) {
        int y_range = current_end - current_start;
        int x_range = std::abs(boundary_line[current_end] - boundary_line[current_start]);
        if (y_range < x_range * 0.5) {
            is_valid_segment = false;
        }
    }
    if (is_valid_segment && current_len > best_len) {
        best_start = current_start;
        best_end = current_end;
        best_len = current_len;
    }
    
    // 拟合参数
    float k = 0, b = 0;
    bool fit_success = false;
    
    // 如果找到了足够长的线段，就计算新的 k 和 b
    if (best_len >= 3) {
        float sum_y = 0, sum_x = 0, sum_yy = 0, sum_xy = 0;
        int n = 0;
        for (int y = best_start; y <= best_end; y++) {
            if (boundary_line[y] != -1) {
                float fy = (float)y;
                float fx = (float)boundary_line[y];
                sum_y += fy;
                sum_x += fx;
                sum_yy += fy * fy;
                sum_xy += fy * fx;
                n++;
            }
        }
        
        if (n >= 3) {
            float denom = n * sum_yy - sum_y * sum_y;
            if (std::abs(denom) > 1e-4f) {
                k = (n * sum_xy - sum_y * sum_x) / denom;
                b = (sum_x - k * sum_y) / n;
                
                // 十字状态下斜率限幅和符号约束
                if (current_track_type == TrackType::crossroad) {
                    // 剔除异常垂直线：如果斜率绝对值过小(即X随Y变化极小，线趋近于直上直下的虚拟边界)，
                    // 说明可能是假边界，将该边全部置为-1，使其不参与求中线运算，依靠另一边单边补线
                    if (std::abs(k) < 0.3f) {
                        for (int yy = VALID_END_ROW; yy <= VALID_START_ROW; yy++) {
                            boundary_line[yy] = -1;
                        }
                        return false;
                    }

                    const float k_max = 1.4f;
                    const float k_min = -1.4f;
                    
                    // 限制幅度
                    if (k > k_max) k = k_max;
                    if (k < k_min) k = k_min;
                    
                    // 强制符号：左边界必须 <= 0（向左上或垂直），右边界必须 >= 0（向右上或垂直）
                    if (is_left && k > 0) {
                        k = -0.9;  // 左边界不能向右，设为垂直
                    }
                    else if (!is_left && k < 0) {
                        k = 0.9;  // 右边界不能向左，设为垂直
                    }
                }
                
                fit_success = true;
            }
        }
    }
    
    // 参数滤波与惯性保底机制
    float* prev_k_ptr = is_left ? &last_k_left : &last_k_right;
    float* prev_b_ptr = is_left ? &last_b_left : &last_b_right;
    bool* first_frame_ptr = is_left ? &first_frame_left : &first_frame_right;
    
    if (fit_success) {
        if (*first_frame_ptr) {
            // 第一帧，直接使用当前值
            *prev_k_ptr = k;
            *prev_b_ptr = b;
            *first_frame_ptr = false;
        } 
        else {
            // 平滑滤波
            k = rate * k + (1.0f - rate) * (*prev_k_ptr);
            b = rate * b + (1.0f - rate) * (*prev_b_ptr);
            *prev_k_ptr = k;
            *prev_b_ptr = b;
        }
    } 
    else {
        // 若找不到线　用上一帧的历史斜率补线
        if (*first_frame_ptr) {
            return false;  // 如果从来没成功过，那没救，退出
        }
        k = *prev_k_ptr;
        b = *prev_b_ptr;
    }
    
    // 十字状态下只补丢线区域，非十字状态也只补丢线区域
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        if (boundary_line[y] == -1) {
            int x = (int)std::round(k * y + b);
            
            // 边界限制：限制在VALID范围内
            if (is_left) {
                if (x < VALID_LEFT_COL) x = VALID_LEFT_COL;
                if (x > (VALID_LEFT_COL + VALID_RIGHT_COL) / 2 + 10) 
                    x = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2 + 10;
            } 
            else {
                if (x > VALID_RIGHT_COL) x = VALID_RIGHT_COL;
                if (x < (VALID_LEFT_COL + VALID_RIGHT_COL) / 2 - 10) 
                    x = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2 - 10;
            }
            
            boundary_line[y] = x;
        }
    }
    
    return true;
}


// 检测底部双边 scan_line 是否恢复正常
bool ElementState::check_bottom_recovered()
{
    int ok = 0;
    int end_y = VALID_BOTTOM_ROW;

    // 检测底部双边是否丢线　且宽度合理
    for (int y = VALID_START_ROW; y >= end_y; y--)
    {
        int lx = left_boundary.scan_line[y];
        int rx = right_boundary.scan_line[y];
        if (lx != -1 && rx != -1)
        {
            int w = rx - lx;
            if (w > 6)
                ok++;
        }
    }
    return ok >= 4;
}

// 圆环专用：检测底部近场双边是否恢复
// 只要求最底部几行左右 scan_line 不丢线，上方允许丢线
bool ElementState::check_circle_bottom_recovered()
{
    int ok = 0;

    // 只看最底部 5~8 行，别往上看太多
    const int check_rows = 6;

    int end_y = VALID_BOTTOM_ROW - check_rows;
    if (end_y > VALID_BOTTOM_ROW) {
        end_y = VALID_BOTTOM_ROW;
    }

    for (int y = VALID_BOTTOM_ROW; y >= end_y; y--)
    {
        int lx = left_boundary.scan_line[y];
        int rx = right_boundary.scan_line[y];

        // 底部左右边界都存在，就认为这一行恢复
        if (lx != -1 && rx != -1)
        {
            int w = rx - lx;

            // 加一个最简单的宽度保护，防止左右线串到一起
            if (w > 10) {
                ok++;
            }
        }
    }

    // 底部 6 行里有 4 行恢复，就认为底部恢复
    return ok >= 4;
}

/**
 * @brief 十字内部补线：最长可靠竖向线段连接到底部锁定锚点
 *
 * 十字内部白区很大，左右边界容易被横线、噪点、色块边缘误导。
 * 这里基本沿用之前效果较好的最长连续线段方案，但把底部点锁成锚点：
 *   1. 必须在对应半区，避免左边界串到右边、右边界串到左边；
 *   2. 必须连续，行间隔和相邻 x 跳变都不能太大；
 *   3. 只做较宽松的竖向比例过滤，然后优先选择最长线段。
 *
 * 斜率定义为 k = dx / dy，也就是 line[y] = x 中 x 随 y 的变化量。
 * 最后把选出的上方可靠线段连接到进入 inside 后锁住的底部锚点，减少每帧重新找底点造成的中线抖动。
 *
 * @param boundary 边界数据
 * @param is_left 是否为左边界
 */
bool ElementState::fix_crossroad_vertical_line(PointState::BoundaryData& boundary, bool is_left)
{
    const int x_mid = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
    constexpr int max_y_gap = 3;          // 连续线段允许的最大断行
    constexpr int max_x_step = 3;         // 相邻有效点的最大 x 跳变，用来过滤突变假边界
    constexpr int min_segment_len = 3;    // 至少 3 个点才参与评分，避免两点噪声直接成线
    constexpr float min_vertical_ratio = 0.6f; // 旧版宽松竖向比例过滤，避免横向假线段直接胜出
        
    // 半区约束 防止边线串道
    auto on_correct_side = [&](int x) -> bool {
        return is_left ? (x < x_mid + 5) : (x > x_mid - 5);
    };

    // 补线结果限制在对应半区
    auto clamp_side = [&](int x) -> int {
        if (is_left) {
            return clip(x, VALID_LEFT_COL, x_mid + 5);
        }
        return clip(x, x_mid - 5, VALID_RIGHT_COL);
    };

    // 线段评分机制
    auto save_segment = [&](int start, int end, int len,
                            int& best_start, int& best_end, int& best_len,
                            float& best_score) {
        if (len < min_segment_len || start < 0 || end < start) {
            return;
        }

        int y_range = end - start;
        if (y_range <= 0) {
            return;
        }

        int x_range = std::abs(boundary.line[end] - boundary.line[start]);
        bool vertical_enough = y_range >= (int)std::round(x_range * min_vertical_ratio);
        bool side_ok = on_correct_side(boundary.line[start]) && on_correct_side(boundary.line[end]);

        // 旧版效果较好的地方是“连续长度优先”，所以这里只用长度做主评分；
        // y_range 作为很小的加分项，让同样点数时更偏向覆盖范围更长的线段。
        float score = (float)len + (float)y_range * 0.01f;

        if (vertical_enough && side_ok && score > best_score) {
            best_start = start;
            best_end = end;
            best_len = len;
            best_score = score;
        }
    };
        
    // 底部锚点优先用原始 scan_line，再退到处理后的 line，最后用下拐点
    // inside 状态下这个锚点会被锁住，后续帧复用，避免底部交界点每帧跳动
    auto find_bottom_anchor = [&]() -> cv::Point {
        const int bottom_search_top = std::max((int)VALID_END_ROW, (int)VALID_START_ROW - 14);

        for (int y = VALID_START_ROW; y >= bottom_search_top; y--) {
            int x = boundary.scan_line[y];
            if (x != -1 && on_correct_side(x)) {
                return cv::Point(clamp_side(x), VALID_START_ROW);
            }
        }

        for (int y = VALID_START_ROW; y >= bottom_search_top; y--) {
            int x = boundary.line[y];
            if (x != -1 && on_correct_side(x)) {
                return cv::Point(clamp_side(x), VALID_START_ROW);
            }
        }

        if (boundary.down_lp_state && on_correct_side(boundary.down_lp_pt.x)) {
            return cv::Point(clamp_side(boundary.down_lp_pt.x), VALID_START_ROW);
        }

        return cv::Point(-1, -1);
    };

    int best_start = -1, best_end = -1, best_len = 0;
    float best_score = -1e9f;
    int current_start = -1, current_end = -1, current_len = 0;
    int last_y = -1, last_x = -1;

    // 从上到下扫描边界数组，把满足半区和连续性要求的点切成若干段，再交给 save_segment 评分
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        int x = boundary.line[y];
        if (x == -1 || !on_correct_side(x)) {
            save_segment(current_start, current_end, current_len,
                         best_start, best_end, best_len, best_score);
            current_start = current_end = -1;
            current_len = 0;
            last_y = last_x = -1;
            continue;
        }

        if (current_start == -1) {
            current_start = current_end = y;
            current_len = 1;
            last_y = y;
            last_x = x;
            continue;
        }

        int y_gap = y - last_y;
        int x_diff = std::abs(x - last_x);
        if (y_gap <= max_y_gap && x_diff <= max_x_step) {
            current_end = y;
            current_len++;
            last_y = y;
            last_x = x;
        } 
        else {
            save_segment(current_start, current_end, current_len,
                         best_start, best_end, best_len, best_score);
            current_start = current_end = y;
            current_len = 1;
            last_y = y;
            last_x = x;
        }
    }
    save_segment(current_start, current_end, current_len,
                 best_start, best_end, best_len, best_score);

    if (best_len < min_segment_len) {
        if (is_left) {
            crossroad_left_quality = 0.0f;
        } else {
            crossroad_right_quality = 0.0f;
        }
        return false;
    }

    cv::Point top_pt(clamp_side(boundary.line[best_start]), best_start);
    cv::Point& locked_bottom_pt = is_left ? crossroad_left_bottom_anchor : crossroad_right_bottom_anchor;
    if (locked_bottom_pt.x == -1) {
        locked_bottom_pt = find_bottom_anchor();
    }
    cv::Point bottom_pt = locked_bottom_pt;
        
    // 如果没有找到可锁定的底部锚点，就按当前最佳线段的斜率外推到底部作为兜底
    if (bottom_pt.x == -1) {

        int x_bottom = boundary.line[best_end];
        int y_range = best_end - best_start;
        if (y_range > 0) {
            float k = (float)(boundary.line[best_end] - boundary.line[best_start]) / y_range;
            if (k > 0.8f) k = 0.8f;
            if (k < -0.8f) k = -0.8f;
            x_bottom = (int)std::round(boundary.line[best_end] + k * (VALID_START_ROW - best_end));
        }
        bottom_pt = cv::Point(clamp_side(x_bottom), VALID_START_ROW);
        locked_bottom_pt = bottom_pt;
    }

    // 记录该侧补线质量
    // 当前 inside 不再用质量差清掉单侧边界，只把它作为调试量保留，避免再次退回单边推中线
    int y_range = best_end - best_start;
    float k = 0.0f;
    if (y_range > 0) {
        k = (float)(boundary.line[best_end] - boundary.line[best_start]) / y_range;
    }
    float quality = (float)best_len * 1.5f - std::abs(k) * 10.0f;
    if (std::abs(k) > 1.0f) {
        quality -= 8.0f;
    }

    if (is_left) {
        crossroad_left_quality = quality;
    } else {
        crossroad_right_quality = quality;
    }

    if (quality < 1.0f) {
        return false;
    }

    connect_points(boundary.line, top_pt, bottom_pt);
    return true;
}

// 圆环环内内部补线：连接到最底部的圆环锚点
bool ElementState::fix_circle_inside_line(PointState::BoundaryData& boundary, bool is_left)
{
    const int x_mid = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
    constexpr int max_y_gap = 3;
    constexpr int max_x_step = 4;
    constexpr int min_segment_len = 5;

    auto on_correct_side = [&](int x) -> bool {
        return is_left ? (x < x_mid + 6) : (x > x_mid - 6);
    };

    auto clamp_side = [&](int x) -> int {
        return is_left ? clip(x, VALID_LEFT_COL, x_mid + 6)
                       : clip(x, x_mid - 6, VALID_RIGHT_COL);
    };

    int best_start = -1;
    int best_end = -1;
    int best_len = 0;
    int current_start = -1;
    int current_end = -1;
    int current_len = 0;
    int last_y = -1;
    int last_x = -1;

    auto save_segment = [&]() {
        if (current_len > best_len && current_len >= min_segment_len) {
            best_start = current_start;
            best_end = current_end;
            best_len = current_len;
        }
    };

    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        int x = boundary.line[y];
        if (x == -1 || !on_correct_side(x)) {
            save_segment();
            current_start = current_end = -1;
            current_len = 0;
            last_y = last_x = -1;
            continue;
        }

        if (current_start == -1) {
            current_start = current_end = y;
            current_len = 1;
            last_y = y;
            last_x = x;
            continue;
        }

        int y_gap = y - last_y;
        int x_diff = std::abs(x - last_x);
        if (y_gap <= max_y_gap && x_diff <= max_x_step) {
            current_end = y;
            current_len++;
            last_y = y;
            last_x = x;
        }
        else {
            save_segment();
            current_start = current_end = y;
            current_len = 1;
            last_y = y;
            last_x = x;
        }
    }
    save_segment();

    if (best_len < min_segment_len) {
        return false;
    }

    if (circle_inside_bottom_anchor.x == -1) {
        const int bottom_top = std::max((int)VALID_END_ROW, (int)VALID_START_ROW - 14);
        for (int y = VALID_START_ROW; y >= bottom_top; y--) {
            int x = boundary.scan_line[y];
            if (x != -1 && on_correct_side(x)) {
                circle_inside_bottom_anchor = cv::Point(clamp_side(x), VALID_START_ROW);
                break;
            }
        }
        if (circle_inside_bottom_anchor.x == -1) {
            for (int y = VALID_START_ROW; y >= bottom_top; y--) {
                int x = boundary.line[y];
                if (x != -1 && on_correct_side(x)) {
                    circle_inside_bottom_anchor = cv::Point(clamp_side(x), VALID_START_ROW);
                    break;
                }
            }
        }
    }

    if (circle_inside_bottom_anchor.x == -1) {
        return false;
    }

    cv::Point top_pt(clamp_side(boundary.line[best_start]), best_start);
    connect_points(boundary.line, top_pt, circle_inside_bottom_anchor);
    return true;
}

// // 从x列向上找白列顶端y（在 bin_buf 上扫）
// int ElementState::find_white_top_at_x(const cv::Mat &bin_buf, int x)
// {
//     if (x < 2 || x >= CAM_WIDTH - 2)
//         return VALID_START_ROW;
//     for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--)
//     {
//         if (bin_buf.ptr<uchar>(y)[x] <= 127)
//         {
//             return y + 1; // 上一行还是白的
//         }
//     }
//     return VALID_END_ROW; // 整列都是白
// }

///////////////////////////////////////////////// 常规道路分类 ///////////////////////////////////////////

// 弯道限幅
static float element_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 计算中线平均x
static bool get_average_mid_x(int top_y, int bottom_y, float &avg_x)
{
    int sum_x = 0;
    int count = 0;

    //　限幅
    if (top_y < VALID_END_ROW + 8) top_y = VALID_END_ROW + 8;
    if (bottom_y > VALID_START_ROW) bottom_y = VALID_START_ROW;

    // 计算平均x
    for (int y = top_y; y <= bottom_y; y++) {
        if (mid.line[y] != -1) {
            sum_x += mid.line[y];
            count++;
        }
    }

    if (count <= 0) return false;
    avg_x = (float)sum_x / (float)count;
    return true;
}

// 计算中线斜率
static float get_fit_midline_k(int top_y, int bottom_y)
{
    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;
    int count = 0;

    // 限幅
    if (top_y < VALID_END_ROW + 8) top_y = VALID_END_ROW + 8;
    if (bottom_y > VALID_START_ROW) bottom_y = VALID_START_ROW;

    // 计算斜率
    for (int y = top_y; y <= bottom_y; y++) {
        if (mid.line[y] != -1) {
            float fy = (float)y;
            float fx = (float)mid.line[y];
            sum_y += fy;
            sum_x += fx;
            sum_yy += fy * fy;
            sum_yx += fy * fx;
            count++;
        }
    }

    if (count < 4) return 0.0f;

    float denom = count * sum_yy - sum_y * sum_y;
    if (std::abs(denom) < 0.001f) return 0.0f;

    return (count * sum_yx - sum_y * sum_x) / denom;
}

// 拟合中线二次曲线，计算中线曲率
static float get_midline_curvature(int top_y, int bottom_y)
{
    double sum_0 = 0.0;
    double sum_y = 0.0;
    double sum_yy = 0.0;
    double sum_yyy = 0.0;
    double sum_yyyy = 0.0;
    double sum_x = 0.0;
    double sum_yx = 0.0;
    double sum_yyx = 0.0;
    int count = 0;

    // 限幅
    if (top_y < VALID_END_ROW + 8) top_y = VALID_END_ROW + 8;
    if (bottom_y > VALID_START_ROW) bottom_y = VALID_START_ROW;

    // 计算二次曲线系数
    for (int y = top_y; y <= bottom_y; y++) {
        if (mid.line[y] != -1) {
            double fy = (double)y;
            double fx = (double)mid.line[y];
            double fy2 = fy * fy;
            sum_0 += 1.0;
            sum_y += fy;
            sum_yy += fy2;
            sum_yyy += fy2 * fy;
            sum_yyyy += fy2 * fy2;
            sum_x += fx;
            sum_yx += fy * fx;
            sum_yyx += fy2 * fx;
            count++;
        }
    }

    if (count < 6) return 0.0f;

    auto det3 = [](double a11, double a12, double a13,
                   double a21, double a22, double a23,
                   double a31, double a32, double a33) {
        return a11 * (a22 * a33 - a23 * a32)
             - a12 * (a21 * a33 - a23 * a31)
             + a13 * (a21 * a32 - a22 * a31);
    };

    double denom = det3(sum_yyyy, sum_yyy, sum_yy,
                        sum_yyy,  sum_yy,  sum_y,
                        sum_yy,   sum_y,   sum_0);
    if (std::abs(denom) < 0.001) return 0.0f;

    double a = det3(sum_yyx, sum_yyy, sum_yy,
                    sum_yx,  sum_yy,  sum_y,
                    sum_x,   sum_y,   sum_0) / denom;
    double b = det3(sum_yyyy, sum_yyx, sum_yy,
                    sum_yyy,  sum_yx,  sum_y,
                    sum_yy,   sum_x,   sum_0) / denom;

    // 计算最大曲率
    double max_curvature = 0.0;
    for (int y = top_y; y <= bottom_y; y++) {
        if (mid.line[y] != -1) {
            double slope = 2.0 * a * (double)y + b;
            double denom_curv = 1.0 + slope * slope;
            double curvature = std::abs(2.0 * a) / (denom_curv * std::sqrt(denom_curv));
            if (curvature > max_curvature) {
                max_curvature = curvature;
            }
        }
    }

    return (float)max_curvature;
}

// 计算弯道强度(弯道曲率得分)
static float calc_visual_curve_score()
{
    const int far_top = VALID_END_ROW + 8;                  // 21
    const int near_bottom = VALID_START_ROW;                // 51    
    const int roi_len = near_bottom - far_top;              // 30

    const int far_bottom = far_top + roi_len / 3;           // 31
    const int mid_top = far_bottom + 1;                     // 32
    const int mid_bottom = mid_top + roi_len / 3;           // 42
    const int near_top = mid_bottom + 1;                    // 43

    // 将中线划分为三段　并分段计算远段、中段和近段的平均中线x坐标
    // 注：此处为简易的二阶差分　看斜率有没有变化　即看发生的弯折的情况
    float far_x = 0.0f;
    float middle_x = 0.0f;
    float near_x = 0.0f;
    bool has_far = get_average_mid_x(far_top, far_bottom, far_x);
    bool has_mid = get_average_mid_x(mid_top, mid_bottom, middle_x);
    bool has_near = get_average_mid_x(near_top, near_bottom, near_x);

    // 将中线划分为两段 计算远段和近段的斜率
    // 注：此处为一阶差分　看斜率
    float far_k = get_fit_midline_k(far_top, far_bottom);
    float near_k = get_fit_midline_k(near_top, near_bottom);

    // 计算整个中线的曲率
    float all_curvature = get_midline_curvature(far_top, near_bottom);

    // 计算这三项的得分　并累加为最终的弯道强度得分
    float score = 0.0f;
    if (has_far && has_mid && has_near) {
        float bend = std::abs((far_x - middle_x) - (middle_x - near_x));
        score += element_clampf(bend / 18.0f, 0.0f, 1.0f) * 0.50f;      // 最大斜率变化量待测
    }
    score += element_clampf(std::abs(far_k - near_k) / 1.0f, 0.0f, 1.0f) * 0.30f;
    score += element_clampf(all_curvature / 0.40f, 0.0f, 1.0f) * 0.20f; // 最大曲率待测

    return element_clampf(score, 0.0f, 1.0f);
}

/**
 * @brief 直道检测（使用单调性标志）
 * 条件：双边单调性好 + 双边有效点多 + 双边斜率接近
 */
bool ElementState::detect_straight_shape()
{
    // 统计左右边界的有效点数
    int left_valid_count = 0;
    int right_valid_count = 0;
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        if (left_boundary.line[y] != -1) left_valid_count++;
        if (right_boundary.line[y] != -1) right_valid_count++;
    }
    
    // 条件一：单边有效点数都要足够多（>= 80%）
    bool left_good = left_valid_count >= (ROI_HEIGHT * 0.60);
    bool right_good = right_valid_count >= (ROI_HEIGHT * 0.60);
    bool cond_1 = left_good && right_good;
    
    // 条件二：双边单调性好（使用现有的is_straight标志）
    bool cond_2 = left_boundary.is_straight && right_boundary.is_straight;

    // 条件三：双边斜率接近（|k1-k2|<0.3）
    float left_k = point_state.get_line_k(left_boundary.line, VALID_END_ROW, VALID_START_ROW);
    float right_k = point_state.get_line_k(right_boundary.line, VALID_END_ROW, VALID_START_ROW);
    bool k_near = std::abs(std::abs(left_k) - std::abs(right_k)) <= 0.3f;
    bool cond_3 = k_near;
    
    // 直道形状：必须同时满足三个条件
    return cond_1 && cond_2 && cond_3;
}

// 判断是否为直道（带排除条件）
bool ElementState::detect_straight()
{
    return detect_straight_shape() && curve_score < 0.08f;
}

/**
 * @brief 弯道检测（带排除条件）
 * 条件：不是直道 + 不是特殊元素 + (双边斜率差异大 或 单边大丢线 或 单边非单调)
 */
bool ElementState::detect_curve()
{
    // 排除条件一：不能是直道
    if (left_boundary.is_straight || right_boundary.is_straight) {
        return false;
    }
    
    // 排除条件二：不能是特殊元素（环岛、十字等）
    if (circle_state != CircleState::none || 
        crossroad_state != CrossroadState::none ||
        barrier_state != BarrierState::none ||
        ramp_state != RampState::none ||
        zebra_state == ZebraState::detected) {
        return false;
    }
    
    // 统计左右边界的有效点数
    int left_valid_count = 0;
    int right_valid_count = 0;
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        if (left_boundary.line[y] != -1) left_valid_count++;
        if (right_boundary.line[y] != -1) right_valid_count++;
    }
    
    // 条件一：双边斜率差异大（不对称，说明是弯道）
    float left_k = point_state.get_line_k(left_boundary.line, VALID_END_ROW, VALID_START_ROW);
    float right_k = point_state.get_line_k(right_boundary.line, VALID_END_ROW, VALID_START_ROW);
    bool k_diff_large = std::abs(left_k - right_k) >= 0.4f;
    bool cond_1 = k_diff_large;
    
    // 条件二：单边有效点少（弯道导致单边大丢线）
    bool low_valid = (left_valid_count < ROI_HEIGHT * 0.65) || (right_valid_count < ROI_HEIGHT * 0.65);
    bool cond_2 = low_valid;
    
    // 条件三：单边非单调（弯道特征）
    bool non_mnt = !left_boundary.is_straight && !right_boundary.is_straight;
    bool cond_3 = non_mnt;
    
    // 弯道判断：斜率差异大 或 有效点少 或 单边非单调
    return cond_1 || cond_2 || cond_3;
}

/**
 * @brief 常规道路分类主函数（带防抖）
 * 入弯快，出弯慢  不确定区进入安全状态
 */
void ElementState::classify_normal_road()
{
    // 计算当前帧的弯道强度 滤波防突变
    float curve_score_now = calc_visual_curve_score();
    curve_score = curve_score * 0.7f + curve_score_now * 0.3f;

    // 检测当前帧的道路类型
    bool is_straight_shape_now = detect_straight_shape();
    bool is_curve_now = detect_curve();

    // 常规道路状态特征划分及不同帧切换
    switch (normal_road_state)
    {
        case NormalRoadState::curve:
        {
            if (curve_score <= 0.18f && is_straight_shape_now) {
                straight_confirm_count++;
                curve_confirm_count = 0;

                if (straight_confirm_count >= 2) {
                    normal_road_state = NormalRoadState::safe;
                    straight_confirm_count = 0;
                }
            }
            else {
                straight_confirm_count = 0;
                curve_confirm_count = 0;
            }
            break;
        }

        case NormalRoadState::safe:
        {
            if (curve_score >= 0.25f || is_curve_now) {
                curve_confirm_count++;
                straight_confirm_count = 0;

                if (curve_confirm_count >= 2) {
                    normal_road_state = NormalRoadState::curve;
                    curve_confirm_count = 0;
                }
            }
            else if (curve_score <= 0.08f && is_straight_shape_now) {
                straight_confirm_count++;
                curve_confirm_count = 0;

                if (straight_confirm_count >= 5) {
                    normal_road_state = NormalRoadState::straight_fast;
                    straight_confirm_count = 0;
                }
            }
            else {
                straight_confirm_count = 0;
                curve_confirm_count = 0;
            }
            break;
        }

        case NormalRoadState::straight_fast:
        {
            if (curve_score >= 0.15f || is_curve_now) {
                curve_confirm_count++;

                if (curve_confirm_count >= 2) {
                normal_road_state = NormalRoadState::safe;
                // straight_confirm_count = 0;
                curve_confirm_count = 0;
                }
            }
            else{
                straight_confirm_count = 0;
                curve_confirm_count = 0;
            }
            break;
        }

        default:
        {
            normal_road_state = NormalRoadState::safe;
            straight_confirm_count = 0;
            curve_confirm_count = 0;
            break;
        }
    }

    // 待测试成功后删除更换normal_type
    normal_type = (normal_road_state == NormalRoadState::straight_fast) ?
                  NormalType::straight : NormalType::curve;
}

ElementState element_state;
