#include "headfile.h"

// 匿名命名空间
namespace {
    // YUV红框检测参数调节
    const int YUV_RED_Y_MIN = 18;   // 颜色阈值
    const int YUV_RED_Y_MAX = 250;
    const int YUV_RED_V_MIN = 140;
    const int YUV_RED_VU_MIN = 18;
    const int YUV_RED_RG_MIN = 10;
    const int YUV_RED_RB_MIN = 8;

    const double YUV_NOISE_MIN_AREA = 38.0;     // 面积阈值
    const double YUV_FAR_MIN_AREA = 1000.0;     // 预警时面积阈值(已无用)
    const double YUV_MID_MIN_AREA = 45.0;
    const double YUV_LANE_OVERLAP_MIN = 0.75;   // 车道重叠阈值
    // 坐标变换函数
    int map_full_x_to_track(int x, int full_width);
    int map_full_y_to_track(int y, int full_height);
    int map_track_x_to_full(int x, int full_width);
    int map_track_y_to_full(int y, int full_height);

    // 红框等级排序
    int marker_level_rank(ModelMarkerLevel level)
    {
        if (level == ModelMarkerLevel::strict) return 2;
        if (level == ModelMarkerLevel::warning) return 1;
        return 0;
    }

    // 判断是否应该替换当前的红框
    bool should_replace_marker(ModelMarkerLevel next_level,
                            const TargetBoardCandidate& next,
                            ModelMarkerLevel current_level,
                            const TargetBoardCandidate& current,
                            bool prefer_same_level)
    {
        if (next_level == ModelMarkerLevel::none || !next.valid) return false;
        if (current_level == ModelMarkerLevel::none || !current.valid) return true;

        int next_rank = marker_level_rank(next_level);
        int current_rank = marker_level_rank(current_level);

        // strict 永远优先于 warning
        if (next_rank != current_rank) {
            return next_rank > current_rank;
        }

        // 同等级时，如果指定优先，就直接让 next 覆盖 current
        if (prefer_same_level) {
            return true;
        }

        // 否则才比较分数
        return next.score > current.score;
    }

    // 红框来源枚举
    const char* marker_source_to_text(MarkerDetectSource source)
    {
        switch (source) {
            case MarkerDetectSource::hsv: return "HSV";
            case MarkerDetectSource::ycrcb: return "YCC";
            case MarkerDetectSource::yuv: return "YUV";
            default: return "NONE";
        }
    }

    const char* marker_method_to_text(int method)
    {
        switch (method) {
            case 1: return "YCrCb";
            case 2: return "YUV";
            default: return "HSV";
        }
    }

    double elapsed_ms(const std::chrono::steady_clock::time_point& start)
    {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start).count();
    }

    // 红框等级枚举转文本
    const char* marker_level_to_text(ModelMarkerLevel level)
    {
        switch (level) {
            case ModelMarkerLevel::strict:  return "STRICT";
            case ModelMarkerLevel::warning: return "WARN";
            default: return "NONE";
        }
    }
}

// 目标板模型识别模块
// 这个文件里目前有两条路径：
// 1. process_action_queue_trigger() 菜单标注 : 
//    只检测红色目标板底座，确认后从菜单动作队列弹出一个动作并打印
// 2. process_model()/model_main() ncnn模型路径:
//    ncnn 三分类模型路径，检测红框 -> ROI裁剪 -> ncnn推理 -> 动作绕行
    

// 初始化模型 加载模型参数和模型权重 成功返回 true
bool ModelDetector::init(const std::string& param_path, const std::string& bin_path){
    model_ready = false;
    net.clear();

    // // 开启 ncnn 的 fp16 模式，加速推理速度
    // // 实测发现 fp16 较 fp32 慢1帧左右 暂不开启 fp16 模式
    // net.opt.use_fp16_packed = true;
    // net.opt.use_fp16_storage = true;
    // net.opt.use_fp16_arithmetic = true;
    
    // 加载模型参数和模型权重文件
    if (net.load_param(param_path.c_str()) != 0) {
        return false;
    }
    if (net.load_model(bin_path.c_str()) != 0) {
        return false;
    }

    net.opt.num_threads = 1; // 单核
    model_ready = true;
    std::cout << "[Model] ncnn ready, class order: supplies, transport, weapon" << std::endl;
    return true;
}


// 模型识别主函数 可选择ncnn模型触发或菜单标定触发两条路径
void ModelDetector::model_main(cv::Mat& frame)
{
    ModelActionResult status;
    const bool use_queue = model_calib.queue_trigger_enable &&
                           model_calib.queue_size() > 0;

    if (use_queue) {
        status = process_action_queue_trigger(frame);
    }
    else {
        status = process_model_trigger(frame);
    }

    // 标志位拦截状态机
    static bool return_distance_started = false;    // 回线状态回线标志位

    if (element_state.model_line_pass_active) {
        return_distance_started = false;
        element_state.current_track_type = TrackType::model;
        target_state = ModelTargetState::passing;
    }

    else if (continue_pass) {
        element_state.current_track_type = TrackType::model;
        target_state = ModelTargetState::returning;
        if (!return_distance_started) {
            return_distance_started = true;
            motor_distance_clear();
            // if(element_state.current_track_type == TrackType::circle)
            //     preprocess.preview = preprocess.ring_preview / 100.0f;
            // else
                preprocess.preview = preprocess.straight_preview / 100.0f;
        }
        motor_distance_get();

        if (motor_distance >= model_return_distance) {
            // returning 结束：此时才恢复进入模型前的元素快照
            element_state.restore_model_context();
            return_distance_started = false;
            // target_state = ModelTargetState::normal;
            motor_distance_clear();
        }
    }
    else {
        return_distance_started = false;
    }

    // // 更新状态 历史中线和图传
    // update_long_midline(status);
    display_model_upimage(frame, status);
    // draw_marker_debug_text();

    if (!use_queue && preprocess.model_profile_enable) {
        static int model_profile_count = 0;
        int interval = std::max(1, preprocess.model_profile_interval);
        model_profile_count++;
        if (model_profile_count % interval == 0) {
            const char* level_text = "NONE";
            if (status.marker_detected) {
                level_text = (status.marker_visual_type == MarkerVisualType::model_marker) ? "STRICT" : "WARN";
            }
            std::cout << "[ModelProfile] method=" << marker_method_to_text(status.marker_method)
                      << " marker=" << status.marker_ms << "ms"
                      << " infer=" << status.infer_ms << "ms"
                      << " total=" << status.model_total_ms << "ms"
                      << " source=" << marker_source_to_text(status.marker_source)
                      << " level=" << level_text
                      << std::endl;
        }
    }
    
}


///////////////////////////////////////////////// 模型处理主函数　(两路径) //////////////////////////////
ModelTargetState last_target_state = ModelTargetState::normal;      //上一次的模型子状态
// ncnn模型触发路径：
// 远距离宽松锁定，中距离裁剪 ROI 跑模型，近距离进入盲区不再识别
ModelActionResult ModelDetector::process_model_trigger(cv::Mat& frame)
{
    auto model_total_start = std::chrono::steady_clock::now();

    // Init per-frame model status.
    ModelActionResult status;
    status.cooldown_count = target_cooldown_count;
    status.action = last_action;
    status.vote_count = action_vote_count;
    status.marker_method = preprocess.model_marker_method;
    auto finish_status = [&]() -> ModelActionResult {
        status.model_total_ms = elapsed_ms(model_total_start);
        return status;
    };

    if (frame.empty()) {
        return finish_status();
    }

    // 标志位拦截状态机
    if (element_state.model_line_pass_active) {
        target_state = ModelTargetState::passing;
    }
    else if (continue_pass) {
        target_state = ModelTargetState::returning;
    }
    // else if (target_state == ModelTargetState::passing ||
    //          target_state == ModelTargetState::returning) {
    //     target_state = ModelTargetState::normal;
    //     loose_warning_confirmed = false;
    // }

    // 冷却期内不再触发新目标板
    if (target_cooldown_count > 0) {
        target_cooldown_count--;
        status.cooldown_count = target_cooldown_count;
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        target_warning_active = false;
        target_warning_count = 0;
        target_warning_keep_count = 0;
        loose_warning_confirmed = false;
        // if (!element_state.model_line_pass_active && !continue_pass) {
        //     target_state = ModelTargetState::normal;
        // }
        // display_model_upimage(frame, status);
        return finish_status();
    }

    // 模型法红框检测: HSV/YCrCb/YUV 三选一，避免同一帧重复跑多套颜色检测
    TargetBoardCandidate model_marker;
    ModelMarkerLevel marker_level = ModelMarkerLevel::none;
    if (!element_state.model_line_pass_active && !continue_pass) {
        auto marker_start = std::chrono::steady_clock::now();
        switch (preprocess.model_marker_method) {
            case 1:
                marker_level = find_ycrcb_marker(frame, model_marker);
                break;
            case 2:
                marker_level = find_yuv_marker(frame, model_marker);
                break;
            default:
                marker_level = find_red_marker(frame, model_marker);
                break;
        }
        status.marker_ms = elapsed_ms(marker_start);
    }

    // 如果判断为预警状态
    if (marker_level == ModelMarkerLevel::warning && model_marker.valid) {
        status.board_detected = true;
        status.board_box = model_marker.full_rect;
        status.board_score = model_marker.score;
        status.distance = model_marker.distance;
        status.board_count = target_warning_count + 1;
        status.marker_detected = true;
        status.marker_box = model_marker.full_rect;
        status.marker_visual_type = MarkerVisualType::loose_warning;
        status.marker_source = model_marker.source;

        target_visible_count = 0;
        target_warning_count++;
        target_warning_keep_count = WARNING_KEEP_FRAMES;
        if (target_warning_count >= WARNING_CONFIRM_FRAMES && !target_warning_active) {
            target_warning_active = true;
            loose_warning_confirmed = true;
            element_state.enter_model_context();
            target_state = ModelTargetState::warning;
            target_warning_start_distance = motor_distance;
        }
    }
    else if (marker_level == ModelMarkerLevel::none) {
        target_visible_count = 0;
        target_warning_count = 0;
        if (target_warning_keep_count > 0) target_warning_keep_count--;
    }

    // 状态机拦截
    if (element_state.model_line_pass_active || continue_pass) {
        target_warning_active = false;
        target_warning_count = 0;
        target_warning_keep_count = 0;
        loose_warning_confirmed = false;
        if (element_state.model_line_pass_active) {
            target_state = ModelTargetState::passing;
        }
        else {
            target_state = ModelTargetState::returning;
        }
    }

    // 预警状态错误检测时退出
    else if (target_warning_active && marker_level != ModelMarkerLevel::strict) {
        float warning_distance = std::fabs(motor_distance - target_warning_start_distance);
        if (target_warning_keep_count <= 0 || warning_distance >= WARNING_MAX_DISTANCE_CM) {
            target_warning_active = false;
            target_warning_count = 0;
            target_warning_keep_count = 0;
            loose_warning_confirmed = false;
            if (target_state == ModelTargetState::warning ||
                target_state == ModelTargetState::recognizing ||
                target_state == ModelTargetState::fail_safe) {
                target_state = ModelTargetState::normal;
                element_state.restore_model_context();
            }
        }
    }

    cv::Rect marker_rect;
    bool has_marker = (marker_level == ModelMarkerLevel::strict && model_marker.valid);
    if (has_marker) {
        marker_rect = model_marker.full_rect;
        status.marker_detected = true;
        status.marker_box = marker_rect;
        status.marker_visual_type = MarkerVisualType::model_marker;
        status.marker_source = model_marker.source;
        status.board_detected = true;
        status.board_box = marker_rect;
        status.board_score = model_marker.score;
        status.distance = model_marker.distance;
        target_visible_count++;

        target_warning_count = 0;
        loose_warning_confirmed = false;
        if (!target_warning_active) {
            target_warning_start_distance = motor_distance;
        }
        target_warning_active = true;
        target_warning_keep_count = WARNING_KEEP_FRAMES;
        if (target_state != ModelTargetState::passing &&
            target_state != ModelTargetState::returning) {
            element_state.enter_model_context();
            target_state = ModelTargetState::recognizing;
        }
    }
    status.marker_count = target_visible_count;

    // // 急救包检测补丁
    // if (!has_marker) {
    //     cv::Rect supplies_rect;
    //     if (find_large_red_supplies(frame, supplies_rect)) {
    //         status.marker_detected = true;
    //         status.marker_box = supplies_rect;
    //         status.marker_visual_type = MarkerVisualType::large_supplies;
    //         status.model = {"supplies", 1.0f, true, supplies_rect};
    //         status.action = ACTION_RIGHT;

    //         int supplies_bottom = map_full_y_to_track(
    //             supplies_rect.y + supplies_rect.height - 1, frame.rows);
    //         status.distance = calc_board_distance(supplies_bottom);

    //         target_visible_count++;
    //         status.marker_count = target_visible_count;
    //         if (target_visible_count < MARKER_CONFIRM_THRESHOLD) {
    //             return status;
    //         }

    //         status.marker_confirmed = true;
    //         status.action_confirmed = true;
    //         last_action = ACTION_RIGHT;
    //         action_vote_count = ACTION_CONFIRM_THRESHOLD;
    //         status.vote_count = action_vote_count;

    //         if (!last_action_printed) {
    //             std::cout << "[Model] supplies fallback confirmed action="
    //                       << model_calib.action_to_ascii(ACTION_RIGHT)
    //                       << ", rect=(" << supplies_rect.x << "," << supplies_rect.y << ","
    //                       << supplies_rect.width << "," << supplies_rect.height << ")"
    //                       << std::endl;

    //             pass_direction = no_pass;
    //             element_state.start_model_line_pass(ModelLinePassDir::right);
    //             last_action_printed = true;
    //         }

    //         target_cooldown_count = TARGET_COOLDOWN_FRAMES;
    //         status.cooldown_count = target_cooldown_count;
    //         target_visible_count = 0;
    //         board_visible_count = 0;
    //         locked_loose_age = 0;
    //         locked_loose_candidate = TargetBoardCandidate{};
    //         last_result = status.model;
    //         return status;
    //     }

    //     target_visible_count = 0;
    //     status.marker_count = 0;
    // }

    // 目标距离等级计算
    BoardDistanceLevel active_distance = BoardDistanceLevel::none;        
    
    // 先看红框的底部坐标来估算距离
    if (has_marker) {
        int marker_bottom = map_full_y_to_track(marker_rect.y + marker_rect.height - 1, frame.rows);
        active_distance = calc_board_distance(marker_bottom);
        status.distance = active_distance;
    }        
    // // 没红框时　由宽松疑似目标板估计的距离作为替代
    // else if (has_board) {
    //     active_distance = board.distance;
    // }

    // 近距离已来不及绕行，直接清掉本次识别状态，避免误触发
    if (active_distance == BoardDistanceLevel::near) {
        same_class_counter = 0;
        action_vote_count = 0;
        last_action = ACTION_STRAIGHT;
        last_action_printed = false;
        board_visible_count = 0;
        locked_loose_age = 0;
        locked_loose_candidate = TargetBoardCandidate{};
        target_warning_active = false;
        target_warning_count = 0;
        target_warning_keep_count = 0;
        loose_warning_confirmed = false;
        target_state = ModelTargetState::normal;
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return finish_status();
    }

    // 远距离只锁定疑似目标，不跑模型，避免目标太小模型错误
    if (active_distance == BoardDistanceLevel::far) {
        same_class_counter = 0;
        action_vote_count = 0;
        target_visible_count = 0;
        status.marker_count = 0;

        bool far_as_loose = model_marker.valid;

        // 远距离锁定宽松目标板　进入预警状态
        if (far_as_loose) {
            bool had_loose_board = status.board_detected;
            status.marker_detected = true;
            status.marker_box = marker_rect;
            status.marker_visual_type = MarkerVisualType::loose_warning;
            status.board_detected = true;
            status.board_box = marker_rect;
            status.board_score = 0.0f;
            status.distance = BoardDistanceLevel::far;

            if (!had_loose_board) {
                target_warning_count++;
            }
            status.board_count = target_warning_count;
            target_warning_keep_count = WARNING_KEEP_FRAMES;
            if (target_warning_count >= WARNING_CONFIRM_FRAMES && !target_warning_active) {
                target_warning_active = true;
                loose_warning_confirmed = true;
                target_warning_start_distance = motor_distance;
            }
            if (target_state != ModelTargetState::passing &&
                target_state != ModelTargetState::returning) {
                element_state.enter_model_context();
                target_state = ModelTargetState::warning;
            }
        }
        else {
            status.marker_detected = false;
            status.marker_box = cv::Rect(0, 0, 0, 0);
            status.marker_visual_type = MarkerVisualType::none;
        }

        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return finish_status();
    }

    //　构建要去跑模型的ROI　　优先级: 红框 > 距离宽松候选板
    cv::Rect roi_rect;
    // 以红框为底 裁出包含目标物和红框的正方形
    if (has_marker && build_target_roi(marker_rect, frame.size(), roi_rect)) {
        status.roi_box = roi_rect;
        status.roi_source = ModelRoiSource::red_marker;
    }
    // // 中距离候选板强推
    // else if (has_board && board.distance == BoardDistanceLevel::mid &&
    //          build_target_roi_from_candidate(board, frame.size(), roi_rect)) {
    //     status.roi_box = roi_rect;
    //     status.roi_source = ModelRoiSource::loose_marker;
    // }

    // 连续多次检测到红框
    bool visual_confirmed = false;
    // 发现真红框，并连续帧达到阈值
    if (has_marker && target_visible_count >= MARKER_CONFIRM_THRESHOLD) {
        visual_confirmed = true;
    }
    // // 高评分宽松候选板且达到连续帧阈值
    // if (!visual_confirmed && has_board &&
    //     board.distance == BoardDistanceLevel::mid &&
    //     board_visible_count >= BOARD_CONFIRM_THRESHOLD &&
    //     board.score >= 3.0f) {      // 分数待调　降低
    //     visual_confirmed = true;
    // }

    // 确认失败或者构建 ROI 失败　跳过本帧
    if (!visual_confirmed || roi_rect.area() <= 0) {
        // 重置变量
        if (!has_marker) {
            same_class_counter = 0;
            action_vote_count = 0;
            last_action = ACTION_STRAIGHT;
            last_action_printed = false;
        }
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return finish_status();
    }

    status.marker_confirmed = visual_confirmed;

    // 如果红框在底线附近，且未确认动作，则认为识别失败　进入安全模式
    int marker_bottom_full = has_marker ? (marker_rect.y + marker_rect.height - 1) : 0;
    if (has_marker && marker_bottom_full >= MODEL_ACTION_DEADLINE_FULL_Y &&
        !status.action_confirmed && last_result.is_detected == false &&
        target_state == ModelTargetState::recognizing) {
        target_state = ModelTargetState::fail_safe;
    }

    // NCNN 模型推理和多帧动作投票机制
    if (!model_ready) {
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return finish_status();
    }

    // 将 ROI 区域送入 ncnn 跑前向推理
    status.model_ran = true;
    {
        auto infer_start = std::chrono::steady_clock::now();
        status.model = infer_roi(frame, roi_rect);
        status.infer_ms = elapsed_ms(infer_start);
    }
    status.roi_box = roi_rect;
    cache_model_debug_display(status, frame, roi_rect);

    // 分类成功处理
    if (status.model.is_detected) {
        // 解析分类出的动作枚举
        ActionType action = class_to_action(status.model.class_name);
        status.action = action;

        // 对相同的识别结果进行投票，减少结果误判
        if (action == last_action) {
            action_vote_count++;
        }
        else {
            last_action = action;   // 如果动作变了，就投给新动作
            action_vote_count = 1;
            last_action_printed = false;
        }
        status.vote_count = action_vote_count;

        // 如果该动作的投票数满足阈值　则认为识别成功
        if (action_vote_count >= ACTION_CONFIRM_THRESHOLD) {
            status.action_confirmed = true;
            target_state = ModelTargetState::passing;
            
            // 播报一次结果
            std::cout << "[Model] confirmed class=" << status.model.class_name
                      << " prob=" << status.model.probability
                      << " action=" << model_calib.action_to_ascii(action)
                      << " board=(" << status.board_box.x << "," << status.board_box.y
                      << "," << status.board_box.width << "," << status.board_box.height << ")"
                      << " marker=(" << status.marker_box.x << "," << status.marker_box.y
                      << "," << status.marker_box.width << "," << status.marker_box.height << ")"
                      << " roi=(" << roi_rect.x << "," << roi_rect.y
                      << "," << roi_rect.width << "," << roi_rect.height << ")"
                      << " source=" << roi_source_to_text(status.roi_source)
                      << std::endl;

            // 绕行动作执行
            switch (action)
            {
                case ACTION_LEFT:
                    pass_direction = no_pass;
                    element_state.start_model_line_pass(ModelLinePassDir::left);
                    break;

                case ACTION_RIGHT:
                    pass_direction = no_pass;
                    element_state.start_model_line_pass(ModelLinePassDir::right);
                    break;

                case ACTION_STRAIGHT:
                    // element_state.start_model_line_pass(ModelLinePassDir::none);
                    // element_state.enter_model_context();
                    continue_pass = true;
                    target_state = ModelTargetState::returning;
                    break;
                    
                default:
                    pass_direction = no_pass;
                    break;
            }

            // // 切入 passing 的同一帧，拉低前瞻并重建中线
            // if(!speed_deci_on)  //未执行速度决策时
            // {
            //     if (element_state.model_line_pass_active)
            //     {
            //         preprocess.preview = preprocess.model_preview / 100.0f;
            //         track_base.model_nowmidline();
            //         // if(last_target_state != ModelTargetState::passing)   //从识别进入绕行的那一瞬间
            //         // {
            //         //     err_frame_pass = true;  //允许跳过错误帧
            //         //     err_frame_pass_time = ERR_PASS_TIME;    //刷新错误帧过渡时间
            //         //     std::cout << "开启误差过渡" << std::endl;
            //         // }
            //     }
            //     else
            //     {
            //         preprocess.preview = preprocess.straight_preview / 100.0f;
            //     }
            // }
            
            last_action_printed = true;
            

            // 进入模型检测冷却期
            target_cooldown_count = TARGET_COOLDOWN_FRAMES;
            
            // 清理状态
            target_visible_count = 0;
            board_visible_count = 0;
            locked_loose_age = 0;
            locked_loose_candidate = TargetBoardCandidate{};
            target_warning_active = false;
            target_warning_count = 0;
            target_warning_keep_count = 0;
            loose_warning_confirmed = false;
        }
    }
    else {
        action_vote_count = 0;
        last_action_printed = false;
    }

    last_target_state = target_state;      //刷新上一次的模型子状态
    // // 模型图传显示
    // display_model_upimage(frame, status);
    return finish_status();
}

// 菜单动作队列调试路径：检测到红框后从队列弹出一个动作
ModelActionResult ModelDetector::process_action_queue_trigger(cv::Mat& frame)
{
    ModelActionResult status;
    status.queue_mode = true;
    status.cooldown_count = target_cooldown_count;
    status.action = last_action;
    status.vote_count = action_vote_count;

    if (!model_calib.queue_trigger_enable || frame.empty()) return status;

    // 模型绕行或绕行结束过渡阶段，不弹菜单动作队列
    if (element_state.model_line_pass_active || continue_pass) {
        if (element_state.model_line_pass_active) {
            target_state = ModelTargetState::passing;
        }
        else {
            target_state = ModelTargetState::returning;
        }
        return status;
    }

    // 冷却帧：避免同一个红框连续存在几十帧，把整条队列瞬间弹空
    // 当前冷却值是一个保守初值，待后续更新换成编码器距离
    if (target_cooldown_count > 0) {
        target_cooldown_count--;
        status.cooldown_count = target_cooldown_count;
        return status;
    }

    // // 宽松红框预警：菜单法只提前减速和显示紫框，不弹队列、不直接绕行
    // if (!element_state.model_line_pass_active && !continue_pass) {
    //     TargetBoardCandidate warning_board;
    //     bool has_warning = find_loose_marker_candidate(frame, warning_board);
    //     if (has_warning) {
    //         status.board_detected = true;
    //         status.board_box = warning_board.full_rect;
    //         status.board_score = warning_board.score;
    //         status.distance = warning_board.distance;
    //         status.board_count = target_warning_count + 1;
    //         status.marker_visual_type = MarkerVisualType::loose_warning;
    //         status.marker_box = warning_board.full_rect;

    //         // 宽松红框只进入 warning 状态，让控制侧提前降速
    //         target_warning_count++;
    //         target_warning_keep_count = WARNING_KEEP_FRAMES;
    //         if (target_warning_count >= WARNING_CONFIRM_FRAMES && !target_warning_active) {
    //             target_warning_active = true;
    //             loose_warning_confirmed = true;
    //             element_state.enter_model_context();
    //             target_state = ModelTargetState::warning;
    //             target_warning_start_distance = motor_distance;
    //         }
    //     }
    //     else {
    //         target_warning_count = 0;
    //         if (target_warning_keep_count > 0) target_warning_keep_count--;
    //     }

    //     // 宽松红框误检保护：连续丢失或编码器距离超时后退出 warning
    //     if (target_warning_active) {
    //         float warning_distance = std::fabs(motor_distance - target_warning_start_distance);
    //         if (target_warning_keep_count <= 0 || warning_distance >= WARNING_MAX_DISTANCE_CM) {
    //             target_warning_active = false;
    //             target_warning_count = 0;
    //             target_warning_keep_count = 0;
    //             loose_warning_confirmed = false;
    //             // if (target_state == ModelTargetState::warning ||
    //             //     target_state == ModelTargetState::fail_safe) {
    //             //     target_state = ModelTargetState::normal;
    //             // }
    //         }
    //     }
    // }

    // 严格红框检测  严格红框确认后消耗菜单动作队列
    cv::Rect marker_rect;
    // bool has_marker = find_red_marker(frame, marker_rect);

    //　两次红框检测
    TargetBoardCandidate queue_marker;
    TargetBoardCandidate hsv_marker;
    // TargetBoardCandidate ycrcb_marker;
    ModelMarkerLevel hsv_level = find_red_marker(frame, hsv_marker);
    ModelMarkerLevel marker_level = hsv_level;
    queue_marker = hsv_marker;
    // ModelMarkerLevel ycrcb_level = find_ycrcb_marker(frame, ycrcb_marker);

    // ModelMarkerLevel marker_level = ycrcb_level;
    // queue_marker = ycrcb_marker;

    // // HSV 优先：同等级时 HSV 覆盖 YCrCb
    // if (should_replace_marker(hsv_level, hsv_marker, marker_level, queue_marker, true)) {
    //     marker_level = hsv_level;
    //     queue_marker = hsv_marker;
    // }
    
    // 宽松红框预警：只提前减速和显示紫框，不弹队列、不直接绕行
    if (marker_level == ModelMarkerLevel::warning && queue_marker.valid) {
        status.board_detected = true;
        status.board_box = queue_marker.full_rect;
        status.board_score = queue_marker.score;
        status.distance = queue_marker.distance;
        status.board_count = target_warning_count + 1;
        status.marker_detected = true;
        status.marker_box = queue_marker.full_rect;
        status.marker_visual_type = MarkerVisualType::loose_warning;
        status.marker_source = queue_marker.source;

        target_visible_count = 0;
        target_warning_count++;
        target_warning_keep_count = WARNING_KEEP_FRAMES;
        if (target_warning_count >= WARNING_CONFIRM_FRAMES && !target_warning_active) {
            target_warning_active = true;
            loose_warning_confirmed = true;
            element_state.enter_model_context();
            target_state = ModelTargetState::warning;
            target_warning_start_distance = motor_distance;
        }
        if (target_warning_active) {
            float warning_distance = std::fabs(motor_distance - target_warning_start_distance);
            if (warning_distance >= WARNING_MAX_DISTANCE_CM) {
                target_warning_active = false;
                target_warning_count = 0;
                target_warning_keep_count = 0;
                loose_warning_confirmed = false;
                if (target_state == ModelTargetState::warning) {
                    target_state = ModelTargetState::normal;
                    element_state.restore_model_context();
                }
                status.marker_detected = false;
                status.board_detected = false;
                status.marker_box = cv::Rect(0, 0, 0, 0);
                status.board_box = cv::Rect(0, 0, 0, 0);
                status.marker_visual_type = MarkerVisualType::none;
                return status;
            }
        }
        return status;
    }
    
    // 严格红框检测：确认后才消耗菜单动作队列
    bool has_marker = (marker_level == ModelMarkerLevel::strict && queue_marker.valid);
    if (!has_marker) {

        // 严格红框消失后清零连续帧计数，下次重新确认
        target_visible_count = 0;
        last_action_printed = false;

        // warning阶段允许暂时没有严格红框，但不消耗菜单动作队列
        if (target_state == ModelTargetState::warning ||
            target_state == ModelTargetState::fail_safe) {
            if (target_warning_keep_count > 0) {
                target_warning_keep_count--;
                return status;
            }
        }

        target_warning_active = false;
        target_warning_count = 0;
        target_warning_keep_count = 0;
        loose_warning_confirmed = false;
        target_state = ModelTargetState::normal;
        return status;
    }

    // marker_rect = queue_marker.full_rect;

    status.marker_detected = true;
    status.marker_box = marker_rect;
    // status.marker_visual_type = MarkerVisualType::queue_marker;
    // status.marker_source = queue_marker.source;
    target_visible_count ++;

    status.marker_count = target_visible_count;

    // 目标距离等级计算
    BoardDistanceLevel active_distance = BoardDistanceLevel::none;        
    
    // 先看红框的底部坐标来估算距离
    if (has_marker) {
        int marker_bottom = map_full_y_to_track(marker_rect.y + marker_rect.height - 1, frame.rows);
        active_distance = calc_board_distance(marker_bottom);
        status.distance = active_distance;
    }        
    // // 没红框时　由宽松疑似目标板估计的距离作为替代
    // else if (has_board) {
    //     active_distance = board.distance;
    // }

    // 近距离已来不及绕行，直接清掉本次识别状态，避免误触发
    if (active_distance == BoardDistanceLevel::near) {
        same_class_counter = 0;
        action_vote_count = 0;
        last_action = ACTION_STRAIGHT;
        last_action_printed = false;
        board_visible_count = 0;
        locked_loose_age = 0;
        locked_loose_candidate = TargetBoardCandidate{};
        target_warning_active = false;
        target_warning_count = 0;
        target_warning_keep_count = 0;
        loose_warning_confirmed = false;
        target_state = ModelTargetState::normal;
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return status;
    }

    // 远距离只锁定疑似目标，不跑模型，避免目标太小模型错误
    if (active_distance == BoardDistanceLevel::far) {
        same_class_counter = 0;
        action_vote_count = 0;
        target_visible_count = 0;
        status.marker_count = 0;

        int marker_bottom = map_full_y_to_track(marker_rect.y + marker_rect.height - 1, frame.rows);
        int trusted_center = 0;
        bool far_as_loose = is_reliable_loose_row(marker_bottom, &trusted_center);

        // 宽松目标板　如果红框中心在可信范围内，则认为是宽松目标板
        if (far_as_loose) {
            int marker_cx = marker_rect.x + marker_rect.width / 2;
            int trusted_center_full = map_track_x_to_full(trusted_center, frame.cols);
            int max_center_err = std::max(8, frame.cols * 5 / CAM_WIDTH);
            far_as_loose = std::abs(marker_cx - trusted_center_full) <= max_center_err;
        }

        // 远距离锁定宽松目标板　进入预警状态
        if (far_as_loose) {
            bool had_loose_board = status.board_detected;
            status.marker_detected = true;
            status.marker_box = marker_rect;
            status.marker_visual_type = MarkerVisualType::loose_warning;
            status.board_detected = true;
            status.board_box = marker_rect;
            status.board_score = 0.0f;
            status.distance = BoardDistanceLevel::far;

            if (!had_loose_board) {
                target_warning_count++;
            }
            status.board_count = target_warning_count;
            target_warning_keep_count = WARNING_KEEP_FRAMES;
            if (target_warning_count >= WARNING_CONFIRM_FRAMES && !target_warning_active) {
                target_warning_active = true;
                loose_warning_confirmed = true;
                target_warning_start_distance = motor_distance;
            }
            if (target_state != ModelTargetState::passing &&
                target_state != ModelTargetState::returning) {
                element_state.enter_model_context();
                target_state = ModelTargetState::warning;
            }
        }
        else {
            status.marker_detected = false;
            status.marker_box = cv::Rect(0, 0, 0, 0);
            status.marker_visual_type = MarkerVisualType::none;
        }

        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
        // display_model_upimage(frame, status);
        return status;
    }

    // 为了抗噪声，必须连续看到红框才认为“这是一个目标板事件”
    status.marker_count = target_visible_count;

    if (target_visible_count < MARKER_CONFIRM_THRESHOLD) {
        return status;
    }

    status.marker_confirmed = true;
    target_warning_active = false;
    target_warning_count = 0;
    target_warning_keep_count = 0;
    loose_warning_confirmed = false;
    element_state.enter_model_context();
    target_state = ModelTargetState::recognizing;

    // 如果红框在底线附近，且未确认动作，则认为识别失败　进入安全模式
    int marker_bottom_full = has_marker ? (marker_rect.y + marker_rect.height - 1) : 0;
    if (has_marker && marker_bottom_full >= MODEL_ACTION_DEADLINE_FULL_Y &&
        !status.action_confirmed && last_result.is_detected == false &&
        target_state == ModelTargetState::recognizing) {
        target_state = ModelTargetState::fail_safe;
    }

    ActionType action = ACTION_STRAIGHT;
    // 只有实际弹出队列动作才确认本次触发　　空队列不再生成默认直行动作
    const bool from_queue = model_calib.pop_next_action(&action);
    if (!from_queue) {
        target_visible_count = 0;
        last_action_printed = false;
        target_state = ModelTargetState::normal;
        return status;
    }    
    status.action = action;
    status.action_confirmed = true;

    // std::cout << "[目标队列] 红色框检测成功, action="
    //           << model_calib.action_to_ascii(action)
    //           << ", source=QUEUE"
    //           << ", remain=" << model_calib.queue_size()
    //           << ", rect=(" << marker_rect.x << "," << marker_rect.y << ","
    //           << marker_rect.width << "," << marker_rect.height << ")"
    //           << std::endl;

    // 菜单标注法绕行距离，与模型法的距离不同
    const float queue_pass_distance = 12.0f;

    std::cout << "[目标队列] 红色框检测成功, action="
            << model_calib.action_to_ascii(action)
            << ", source=QUEUE"
            << ", remain=" << model_calib.queue_size()
            << ", rect=(" << marker_rect.x << "," << marker_rect.y << ","
            << marker_rect.width << "," << marker_rect.height << ")"
            << std::endl;

    // 队列动作已经弹出，这里必须无条件执行一次，不能再被 last_action_printed 卡住
    switch (action)
    {
        case ACTION_LEFT:
            pass_direction = no_pass;
            element_state.start_model_line_pass(ModelLinePassDir::left, queue_pass_distance);
            target_state = ModelTargetState::passing;
            break;

        case ACTION_RIGHT:
            pass_direction = no_pass;
            element_state.start_model_line_pass(ModelLinePassDir::right, queue_pass_distance);
            target_state = ModelTargetState::passing;
            break;

        case ACTION_STRAIGHT:
            // element_state.start_model_line_pass(ModelLinePassDir::none);
            // element_state.enter_model_context();
            continue_pass = true;
            target_state = ModelTargetState::returning;
            break;

        default:
            pass_direction = no_pass;
            target_state = ModelTargetState::normal;
            break;
    }

    // // 菜单法弹出 LEFT/RIGHT 并切入 passing 的同一帧，拉低前瞻并重建中线
    // if(!speed_deci_on)  //未执行速度决策时
    // {
    //     if (target_state == ModelTargetState::passing)
    //     {
    //         preprocess.preview = preprocess.model_preview / 100.0f;
    //         track_base.model_nowmidline();
    //         // if(last_target_state != ModelTargetState::passing)   //从识别进入绕行的那一瞬间
    //         // {
    //         //     err_frame_pass = true;  //允许跳过错误帧
    //         //     err_frame_pass_time = ERR_PASS_TIME;    //刷新错误帧过渡时间
    //         //     std::cout << "开启误差过渡" << std::endl;
    //         // }
    //     }
    //     else
    //     {
    //         preprocess.preview = preprocess.straight_preview / 100.0f;
    //     }
    // }
    // 触发成功后，重置计数器并进入冷却期
    target_visible_count = 0;
    target_cooldown_count = 45;
    status.cooldown_count = target_cooldown_count;

    // 刷新模型子状态
    last_target_state = target_state;      //刷新上一次的模型子状态

    return status;
}


// 测试模型单帧处理流程　实时显示识别到的内容以及来源
ModelActionResult ModelDetector::process_model_debug_frame(cv::Mat& frame)
{
    ModelActionResult status;
    if (frame.empty()) return status;

    TargetBoardCandidate board;
    bool has_board = find_loose_marker_candidate(frame, board);
    if (has_board) {
        status.board_detected = true;
        status.board_box = board.full_rect;
        status.distance = board.distance;
        status.board_score = board.score;
        status.board_count = 1;
        status.marker_visual_type = MarkerVisualType::loose_warning;
    }

    // 红框检测：YUV
    TargetBoardCandidate selected_marker;

    status.marker_method = 2;
    auto marker_start = std::chrono::steady_clock::now();
    ModelMarkerLevel marker_level = find_yuv_marker(frame, selected_marker);
    status.marker_ms = elapsed_ms(marker_start);

    bool has_marker = (marker_level != ModelMarkerLevel::none && selected_marker.valid);
    cv::Rect marker_rect;

    if (has_marker) {
        marker_rect = selected_marker.full_rect;

        status.marker_detected = true;
        status.marker_box = marker_rect;
        status.marker_source = selected_marker.source;
        status.marker_count = 1;

        status.distance = selected_marker.distance;
        status.board_score = selected_marker.score;

    // 调试显示：中距离 strict 显示 MODEL，远距离 warning 显示 WARN
        if (marker_level == ModelMarkerLevel::strict) {
            status.marker_confirmed = true;
            status.marker_visual_type = MarkerVisualType::model_marker;
        }
        else {
            status.marker_confirmed = false;
            status.marker_visual_type = MarkerVisualType::loose_warning;
        }
    }

// 画出调试文字
    auto draw_marker_debug_text = [&]() {
        const char* selected_source = has_marker
            ? marker_source_to_text(selected_marker.source)
            : "NONE";

        char line1[160];
        std::snprintf(line1, sizeof(line1),
                      "METHOD:%s  LEVEL:%s",
                      marker_method_to_text(status.marker_method),
                      marker_level_to_text(marker_level));

        char line2[160];
        std::snprintf(line2, sizeof(line2),
                      "SEL:%s %s %.2f",
                      selected_source,
                      marker_level_to_text(marker_level),
                      has_marker ? selected_marker.score : 0.0f);

        cv::putText(frame, line1,
                    cv::Point(5, 45),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.38,
                    cv::Scalar(0, 255, 255),
                    1);

        cv::putText(frame, line2,
                    cv::Point(5, 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.38,
                    cv::Scalar(0, 255, 255),
                    1);
    };

    // // 急救包补丁检测
    // if (!has_marker) {
    //     cv::Rect supplies_rect;
    //     if (find_large_red_supplies(frame, supplies_rect)) {
    //         status.marker_detected = true;
    //         status.marker_confirmed = true;
    //         status.marker_box = supplies_rect;
    //         status.marker_visual_type = MarkerVisualType::large_supplies;
    //         status.model = {"supplies", 1.0f, true, supplies_rect};
    //         status.action = ACTION_RIGHT;
    //         status.action_confirmed = true;
    //         status.marker_count = 1;
    //         status.vote_count = 1;
    //         int supplies_bottom = map_full_y_to_track(
    //             supplies_rect.y + supplies_rect.height - 1, frame.rows);
    //         status.distance = calc_board_distance(supplies_bottom);
    //         display_model_upimage(frame, status);
    //         return status;
    //     }
    // }

    //　生成roi区域　　优先级:红框 > 候选板（中距离）
    cv::Rect roi_rect;
    //  红框生成roi
    if (has_marker && status.distance == BoardDistanceLevel::far) {
        status.marker_confirmed = false;
        status.marker_visual_type = MarkerVisualType::loose_warning;
        status.board_detected = true;
        status.board_box = marker_rect;
        status.board_score = selected_marker.score;
        status.board_count = 1;

        // // 更新状态 历史中线和图传
        // update_long_midline(status);
        display_model_upimage(frame, status);
        draw_marker_debug_text();
        return status;
    }

    if (has_marker && build_target_roi(marker_rect, frame.size(), roi_rect)) {
        status.roi_box = roi_rect;
        status.roi_source = ModelRoiSource::red_marker; //　记录roi是红框生成的
    }
    // // 如果没有红框，但在中距离检测到了候选板
    // else if (has_board && board.distance == BoardDistanceLevel::mid &&
    //          build_target_roi_from_candidate(board, frame.size(), roi_rect)) {
    //     status.roi_box = roi_rect;
    //     status.roi_source = ModelRoiSource::loose_marker; // 记录roi是由候选板生成的
    // }

    // NCNN 神经网络前向推理
    if (model_ready && roi_rect.area() > 0) {
        status.model_ran = true;                   // 记录本帧执行了模型推理
        auto infer_start = std::chrono::steady_clock::now();
        status.model = infer_roi(frame, roi_rect); // 调用推理函数获取结果
        status.infer_ms = elapsed_ms(infer_start);
        cache_model_debug_display(status, frame, roi_rect);
        
        // 如果推理函数连续确认为同一类别
        if (status.model.is_detected) {
            // 绕行动作枚举输出
            status.action = class_to_action(status.model.class_name);
            status.action_confirmed = true;  // 调试模式直接视为最终确认动作
            status.vote_count = 1;           // 虚拟一票到底
        }
    }
    else {
        // 如果模型未加载或没有ROI，清除上一次残留的结果并归零状态
        last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
    }

    // // 更新状态 历史中线和图传
    // update_long_midline(status);
    // display_model_upimage(frame, status);
    draw_marker_debug_text();
    return status;
}


// 对单帧图像进行模型推理  ncnn 三分类模型路径：检测到红框后，对 ROI 区域做 ncnn 三分类推理
// 初版模型代码　现已不用
ModelResult ModelDetector::process_model(cv::Mat& frame)
{
    // 初始化
    ModelResult result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
    if (!model_ready || frame.empty()) {
        last_result = result;
        return result;
    }

    // 红框检测
    cv::Rect marker_rect;
    if (!find_red_marker(frame, marker_rect)) {
        same_class_counter = 0;
        last_result = result;
        return result;
    }

    // 构建　ROI
    cv::Rect roi_rect;
    if (!build_target_roi(marker_rect, frame.size(), roi_rect)) {
        last_result = result;
        return result;
    }
    return infer_roi(frame, roi_rect);
}

//////////////////////////////////////////// 模型具体功能函数实现 ////////////////////////////////////////////
// 目标板距离等级划分　　　根据目标板底座下沿的 Y 轴坐标
BoardDistanceLevel ModelDetector::calc_board_distance(int bottom_y) const
{
    // 划分距离等级
    int roi_span = MODEL_HEIGHT;
    int far_y = roi_span * 0.453f + 1;   // 中远距离分界   20 待调　　80行    55cm
    int mid_y = roi_span * 0.95f + 1;    // 中近距离分界   41 待调　　160行   8cm

    // 距离限幅
    if (bottom_y < MODEL_TOP || bottom_y > MODEL_BOTTOM) return BoardDistanceLevel::none;
    if (bottom_y < far_y) return BoardDistanceLevel::far;
    if (bottom_y < mid_y) return BoardDistanceLevel::mid;
    return BoardDistanceLevel::near;
}

// 调试显示　将距离等级映射转为文本
const char* ModelDetector::board_distance_to_text(BoardDistanceLevel level) const
{
    switch (level) {
        case BoardDistanceLevel::far: return "FAR";
        case BoardDistanceLevel::mid: return "MID";
        case BoardDistanceLevel::near: return "NEAR";
        default: return "NONE";
    }
}

// 调试显示  将ROI来源种类转为文本
const char* ModelDetector::roi_source_to_text(ModelRoiSource source) const
{
    switch (source) {
        case ModelRoiSource::red_marker: return "RED";
        case ModelRoiSource::loose_marker: return "LOOSE";
        default: return "NONE";
    }
}

// 更新历史中线
void ModelDetector::update_long_midline(const ModelActionResult& status)
{
    // 冻结中线标志位
    bool freeze_memory = status.board_detected || status.marker_detected ||
                         target_warning_active ||
                         element_state.model_line_pass_active || continue_pass;
    if (freeze_memory) return;

    const float alpha = 0.80f;  // 历史中线平滑系数
    for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
        if (y < 0 || y >= CAM_HEIGHT) continue;
        if (left_boundary.line[y] == -1 || right_boundary.line[y] == -1) continue;

        // 获取边线
        int left_x = left_boundary.line[y];
        int right_x = right_boundary.line[y];
        if (left_x > right_x) std::swap(left_x, right_x);

        // 赛道宽度检查
        int lane_width = right_x - left_x;
        int expected_width = TrackBase::halfRoad[y] * 2;
        if (expected_width <= 0) expected_width = VALID_RIGHT_COL - VALID_LEFT_COL;
        int min_width = std::max(8, (int)std::round(expected_width * 0.75f));
        int max_width = std::min(CAM_WIDTH - 1, (int)std::round(expected_width * 1.25f));
        if (lane_width < min_width || lane_width > max_width) continue;

        // 现存中线获取
        int boundary_center = (left_x + right_x) / 2;
        int live_center = boundary_center;
        if (mid.line[y] != -1 && std::abs(mid.line[y] - boundary_center) <= 8) {
            live_center = mid.line[y];
        }

        // 历史中线更新 首次记录，之后平滑处理
        live_center = std::max(0, std::min(CAM_WIDTH - 1, live_center));
        if (!memory_mid_valid[y]) {
            memory_mid_line[y] = live_center;
            memory_mid_valid[y] = true;
        }
        else {
            float updated = alpha * live_center + (1.0f - alpha) * memory_mid_line[y];
            memory_mid_line[y] = std::max(0, std::min(CAM_WIDTH - 1,
                                      static_cast<int>(std::round(updated))));
        }
    }
}

// 历史中线位置判断  没有历史值时退回当前帧传入的中线
int ModelDetector::long_center_at(int track_y, int fallback_center) const
{
    if (track_y >= 0 && track_y < CAM_HEIGHT && memory_mid_valid[track_y]) {
        return memory_mid_line[track_y];
    }
    return fallback_center;
}

// 远距离识别与宽松红框融合
bool ModelDetector::is_reliable_loose_row(int track_y, int* trusted_center) const
{
    if (track_y < VALID_END_ROW || track_y > VALID_START_ROW) return false;
    if (left_boundary.line[track_y] == -1 || right_boundary.line[track_y] == -1) return false;

    // 筛选条件1: 赛道边界合理性检查
    int left_x = left_boundary.line[track_y];
    int right_x = right_boundary.line[track_y];
    if (left_x > right_x) std::swap(left_x, right_x);

    // 筛选条件2: 赛道宽度检查　
    int lane_width = right_x - left_x;
    int expected_width = TrackBase::halfRoad[track_y] * 2;
    if (expected_width <= 0) expected_width = VALID_RIGHT_COL - VALID_LEFT_COL;

    int min_width = std::max(8, (int)std::round(expected_width * 0.75f));
    int max_width = std::min(CAM_WIDTH - 1, (int)std::round(expected_width * 1.25f));
    if (lane_width < min_width || lane_width > max_width) return false;

    // 筛选条件3: 中线可信度检查
    int center = (left_x + right_x) / 2;
    if (mid.line[track_y] != -1 && std::abs(mid.line[track_y] - center) <= 5) {
        center = mid.line[track_y];
    }

    if (trusted_center != nullptr) {
        *trusted_center = center;
    }
    return true;
}

// YcRcb 红框识别
ModelMarkerLevel ModelDetector::find_ycrcb_marker(cv::Mat& frame, TargetBoardCandidate& candidate)
{
    candidate = TargetBoardCandidate{};
    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) return ModelMarkerLevel::none;

    // // 红框识别阈值　宽松红框与严格红框分数
    // const float warn_score_th = 0.40f;
    // const float strict_score_th = 0.60f;

    cv::Mat ycrcb;
    cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> yc;
    cv::split(ycrcb, yc);

    // 红色区域检测
    cv::Mat y_mask, cr_mask, diff, diff_mask, red_mask;
    cv::inRange(yc[0], cv::Scalar(45), cv::Scalar(245), y_mask);
    cv::inRange(yc[1], cv::Scalar(135), cv::Scalar(255), cr_mask);
    cv::subtract(yc[1], yc[2], diff, cv::noArray(), CV_16S);
    cv::Mat diff_ref(diff.size(), diff.type(), cv::Scalar(20));
    cv::compare(diff, diff_ref, diff_mask, cv::CMP_GT);
    red_mask = y_mask & cr_mask & diff_mask;

    cv::Mat hsv, white_mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 80), cv::Scalar(180, 95, 255), white_mask);

    // 限制红框检测在赛道内
    cv::Mat lane_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    for (int track_y = MODEL_TOP; track_y <= MODEL_BOTTOM; track_y++) {
        if (track_y < 0 || track_y >= CAM_HEIGHT) continue;

        int half_road = TrackBase::halfRoad[track_y];
        if (half_road <= 0) half_road = (VALID_RIGHT_COL - VALID_LEFT_COL) / 2;
        int left_x = -1;
        int right_x = -1;

        // 赛道边界
        if (left_boundary.line[track_y] != -1 && right_boundary.line[track_y] != -1) {
            left_x = left_boundary.line[track_y];
            right_x = right_boundary.line[track_y];
        }
        else if (left_boundary.line[track_y] != -1) {
            left_x = left_boundary.line[track_y];
            right_x = left_x + half_road * 2;
        }
        else if (right_boundary.line[track_y] != -1) {
            right_x = right_boundary.line[track_y];
            left_x = right_x - half_road * 2;
        }
        else {
            continue;
        }

        if (left_x > right_x) std::swap(left_x, right_x);
        left_x = std::max(VALID_LEFT_COL, left_x);
        right_x = std::min(VALID_RIGHT_COL, right_x);

        int lane_width = right_x - left_x;
        int expected_width = std::max(4, half_road * 2);
        if (lane_width < expected_width * 0.75f || lane_width > expected_width * 1.35f) continue;

        int margin = std::max(1, half_road / 8);    //　边界距　
        left_x += margin;
        right_x -= margin;
        if (left_x >= right_x) continue;

        // 赛道边界转换
        int full_y0 = map_track_y_to_full(track_y, frame.rows);
        int full_y1 = map_track_y_to_full(std::min(track_y + 1, CAM_HEIGHT - 1), frame.rows);
        int full_x0 = map_track_x_to_full(left_x, frame.cols);
        int full_x1 = map_track_x_to_full(right_x, frame.cols);
        if (full_x0 > full_x1) std::swap(full_x0, full_x1);
        if (full_y0 > full_y1) std::swap(full_y0, full_y1);
        cv::rectangle(lane_mask,
                      cv::Point(full_x0, full_y0),
                      cv::Point(full_x1, full_y1),
                      cv::Scalar(255),
                      cv::FILLED);
    }

    // 开闭运算
    cv::bitwise_and(red_mask, lane_mask, red_mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = static_cast<double>(frame.cols * frame.rows);
    const double min_area = std::max(80.0, frame_area * 0.0013f);  // 100
    const double max_area = frame_area * 0.0163f;                  // 1250  扩大　为急救包考虑

    float best_score = -1.0f;
    TargetBoardCandidate best;
    ModelMarkerLevel best_level = ModelMarkerLevel::none;

    // 截取的矩形形状
    auto clipped_rect = [&](int x, int y, int w, int h) -> cv::Rect {
        int x0 = std::max(0, x);
        int y0 = std::max(0, y);
        int x1 = std::min(frame.cols, x + w);
        int y1 = std::min(frame.rows, y + h);
        if (x1 <= x0 || y1 <= y0) return cv::Rect(0, 0, 0, 0);
        return cv::Rect(x0, y0, x1 - x0, y1 - y0);
    };

    // 白色比例计算
    auto white_ratio = [&](const cv::Rect& r) -> float {
        if (r.area() <= 0) return 0.0f;
        return static_cast<float>(cv::countNonZero(white_mask(r))) /
               static_cast<float>(std::max(1, r.area()));
    };

    // 红框评分：筛选后选择综合分数最高的候选，等级只由距离决定
    for (const auto& cnt : contours) {
        // 筛选条件1 色块面积阈值
        double area = cv::contourArea(cnt);
        if (area < min_area || area > max_area) continue;

        // 筛选条件2 红色矩形宽高阈值
        cv::Rect rect = cv::boundingRect(cnt);
        if (rect.width < 7 || rect.height < 2) continue;

        // 红色框是横边更长的矩形　宽高比阈值　待调
        float rate = static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));
        if (rate < 0.70f || rate > 6.5f) continue;      // 直道差不多是3~4

        int cx = rect.x + rect.width / 2;           // 红框中心x坐标
        int cy = rect.y + rect.height / 2;          // 红框中心y坐标
        
        // 筛选条件3　红框底部在赛道内 且 距离适中
        int bottom_y = map_full_y_to_track(rect.y + rect.height - 1, frame.rows);
        BoardDistanceLevel dist = calc_board_distance(bottom_y);
        if (dist == BoardDistanceLevel::none || dist == BoardDistanceLevel::near) continue;
        if (bottom_y < MODEL_TOP || bottom_y > MODEL_BOTTOM) continue;

        // 筛选条件4　红色像素占比阈值
        cv::Mat red_roi = red_mask(rect);
        int red_pixels = cv::countNonZero(red_roi);
        float red_ratio = static_cast<float>(red_pixels) /
                          static_cast<float>(std::max(1, rect.area()));
        if (red_pixels < 3 || red_ratio < 0.05f) continue;

        //  筛选条件5　白色边框占比阈值
        int band = std::max(2, std::min(8, std::max(rect.height, rect.width / 4)));
        cv::Rect top_r = clipped_rect(rect.x - band, rect.y - band, rect.width + band * 2, band);
        cv::Rect bottom_r = clipped_rect(rect.x - band, rect.y + rect.height, rect.width + band * 2, band);
        cv::Rect left_r = clipped_rect(rect.x - band, rect.y, band, rect.height);
        cv::Rect right_r = clipped_rect(rect.x + rect.width, rect.y, band, rect.height);

        float top_white = white_ratio(top_r);
        float bottom_white = white_ratio(bottom_r);
        float left_white = white_ratio(left_r);
        float right_white = white_ratio(right_r);
        float side_white = std::max(left_white, right_white);
        float white_score = std::min(1.0f,
                                     bottom_white * 0.40f +
                                     side_white * 0.30f +
                                     top_white * 0.20f +
                                     (left_white + right_white) * 0.05f);
        if (bottom_white < 0.08f || (side_white < 0.06f && top_white < 0.06f)) continue;

        // 各项得分
        float red_score = std::min(red_ratio / 0.35f, 1.0f);
        float aspect_score = 1.0f - std::min(std::fabs(rate - 3.0f) / 3.0f, 1.0f);
        float size_score = static_cast<float>(std::min(area / (frame_area * 0.006), 1.0));
        float shape_score = std::min(1.0f, aspect_score * 0.65f + size_score * 0.35f);
        float lane_score = (cv::countNonZero(lane_mask(rect)) > rect.area() * 0.30f) ? 1.0f : 0.0f;
        if (lane_score <= 0.0f) continue;

        ModelMarkerLevel level = (dist == BoardDistanceLevel::mid)
                                     ? ModelMarkerLevel::strict
                                     : ModelMarkerLevel::warning;

        float score = red_score * 0.35f + shape_score * 0.25f + lane_score * 0.20f + white_score * 0.20f;
        if (score < 0.35f) continue;    // 最低分限制　待调

        if (score > best_score) {
            best_score = score;
            best_level = level;
            best.valid = true;
            best.full_rect = rect;
            best.track_rect = cv::Rect(map_full_x_to_track(rect.x, frame.cols),
                                       map_full_y_to_track(rect.y, frame.rows),
                                       std::max(1, rect.width * CAM_WIDTH / frame.cols),
                                       std::max(1, rect.height * CAM_HEIGHT / frame.rows));
            best.center_x = map_full_x_to_track(cx, frame.cols);
            best.center_y = map_full_y_to_track(cy, frame.rows);
            best.bottom_y = bottom_y;
            best.stable_rows = rect.height;
            best.score = score;
            best.distance = dist;
            best.source = MarkerDetectSource::ycrcb;
        }
    }

    if (!best.valid) return ModelMarkerLevel::none;
    candidate = best;
    return best_level;
}

// YUV 红框识别
ModelMarkerLevel ModelDetector::find_yuv_marker(cv::Mat& frame, TargetBoardCandidate& candidate)
{
    candidate = TargetBoardCandidate{};
    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) return ModelMarkerLevel::none;

    //　各个掩码创建
    cv::Mat red_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat white_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat lane_mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    cv::Rect scan_bounds;
    bool has_scan_bounds = false;
    bool curve_context = (element_state.normal_type == NormalType::curve && element_state.curve_score >= 0.15f);
    bool big_curve_context = (element_state.normal_type == NormalType::curve && element_state.curve_score >= 0.33f);
    bool circle_inside = (element_state.current_track_type == TrackType::circle &&
                          element_state.circle_state == CircleState::inside);
    bool left_circle = (element_state.circle_dir == CircleDir::left);
    bool right_circle = (element_state.circle_dir == CircleDir::right);
    int far_y = MODEL_HEIGHT * 0.453f + 1;      // 中远边界线

    // 限制YUV扫描区域　
    for (int track_y = MODEL_TOP; track_y <= MODEL_BOTTOM; track_y++) {
        int half_road = TrackBase::halfRoad[track_y];
        if (half_road <= 0) half_road = (VALID_RIGHT_COL - VALID_LEFT_COL) / 2;

        int left_x = -1;
        int right_x = -1;
        bool has_left = left_boundary.line[track_y] != -1;
        bool has_right = right_boundary.line[track_y] != -1;

        // 正常情况下单边界互推
        if (has_left) left_x = left_boundary.line[track_y];
        if (has_right) right_x = right_boundary.line[track_y];
        if (has_left && !has_right) right_x = left_x + (int)std::round(half_road * 2.05f);  // 适当扩大些
        if (!has_left && has_right) left_x = right_x - (int)std::round(half_road * 2.05f);
        if (!has_left && !has_right) continue;

        // 在圆环环内状态的推边界　左圆环左半区
        if (circle_inside && (has_left != has_right) && (left_circle || right_circle)) {
            int image_center = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
            int center_overlap = std::max(4, half_road / 2);

            if (left_circle) {
                left_x = VALID_LEFT_COL;
                right_x = std::min(VALID_RIGHT_COL, image_center + center_overlap);
            }
            else {
                left_x = std::max(VALID_LEFT_COL, image_center - center_overlap);
                right_x = VALID_RIGHT_COL;
            }
        }

        if (left_x > right_x) std::swap(left_x, right_x);
        left_x = std::max(VALID_LEFT_COL, left_x);
        right_x = std::min(VALID_RIGHT_COL, right_x);

        int lane_width = right_x - left_x;
        if (lane_width < 8) continue;

        bool far_row = track_y < far_y;
        int shrink = (far_row && !curve_context)
                         ? std::max(2, lane_width / 8)      // 边缘距
                         : std::max(1, lane_width / 16);
        left_x += shrink;
        right_x -= shrink;
        if (left_x >= right_x) continue;

        int full_y0 = map_track_y_to_full(track_y, frame.rows);
        int full_y1 = map_track_y_to_full(std::min(track_y + 1, CAM_HEIGHT - 1), frame.rows);
        int full_x0 = map_track_x_to_full(left_x, frame.cols);
        int full_x1 = map_track_x_to_full(right_x, frame.cols);
        if (full_x0 > full_x1) std::swap(full_x0, full_x1);
        if (full_y0 > full_y1) std::swap(full_y0, full_y1);
        full_x0 = std::max(0, std::min(full_x0, frame.cols - 1));
        full_x1 = std::max(0, std::min(full_x1, frame.cols - 1));
        full_y0 = std::max(0, std::min(full_y0, frame.rows - 1));
        full_y1 = std::max(0, std::min(full_y1, frame.rows - 1));
        if (full_x0 >= full_x1 || full_y0 > full_y1) continue;

        cv::rectangle(lane_mask,
                      cv::Point(full_x0, full_y0),
                      cv::Point(full_x1, full_y1),
                      cv::Scalar(255),
                      cv::FILLED);

        cv::Rect row_scan(full_x0, full_y0, full_x1 - full_x0 + 1, full_y1 - full_y0 + 1);
        scan_bounds = has_scan_bounds ? (scan_bounds | row_scan) : row_scan;
        has_scan_bounds = true;

        // 手写 YUV 阈值 少用CV库
        for (int y = full_y0; y <= full_y1; y++) {
            const cv::Vec3b* row = frame.ptr<cv::Vec3b>(y);
            uchar* red_row = red_mask.ptr<uchar>(y);
            uchar* white_row = white_mask.ptr<uchar>(y);
            for (int x = full_x0; x <= full_x1; x++) {
                int b = row[x][0];
                int g = row[x][1];
                int r = row[x][2];

                int yy = (77 * r + 150 * g + 29 * b) >> 8;
                int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8);
                int v = 128 + ((128 * r - 107 * g - 21 * b) >> 8);

                if (yy >= YUV_RED_Y_MIN &&
                    yy <= YUV_RED_Y_MAX &&
                    v >= YUV_RED_V_MIN &&
                    (v - u) >= YUV_RED_VU_MIN &&
                    r >= g + YUV_RED_RG_MIN &&
                    r >= b + YUV_RED_RB_MIN) {
                    red_row[x] = 255;
                }
                if (yy >= 80 && std::abs(u - 128) <= 30 && std::abs(v - 128) <= 38) {
                    white_row[x] = 255;
                }
            }
        }
    }

    if (!has_scan_bounds || scan_bounds.area() <= 0) return ModelMarkerLevel::none;

    cv::Mat red_scan = red_mask(scan_bounds);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(red_scan, red_scan, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_scan, red_scan, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_scan, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = static_cast<double>(frame.cols * frame.rows);
    // const double min_area = std::max(80.0, frame_area * 0.0013f);   // 100
    const double max_area = frame_area * 0.00781f;                  // 600  扩大　为急救包考虑

    float best_score = -1.0f;
    TargetBoardCandidate best;
    ModelMarkerLevel best_level = ModelMarkerLevel::none;

    //　裁剪矩形
    auto clipped_rect = [&](int x, int y, int w, int h) -> cv::Rect {
        int x0 = std::max(0, x);
        int y0 = std::max(0, y);
        int x1 = std::min(frame.cols, x + w);
        int y1 = std::min(frame.rows, y + h);
        if (x1 <= x0 || y1 <= y0) return cv::Rect(0, 0, 0, 0);
        return cv::Rect(x0, y0, x1 - x0, y1 - y0);
    };

    // 计算白色区域的像素比例
    auto white_ratio = [&](const cv::Rect& r) -> float {
        if (r.area() <= 0) return 0.0f;
        return static_cast<float>(cv::countNonZero(white_mask(r))) /
               static_cast<float>(std::max(1, r.area()));
    };

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < YUV_NOISE_MIN_AREA || area > max_area) continue;

        cv::Rect rect = cv::boundingRect(cnt);
        rect.x += scan_bounds.x;
        rect.y += scan_bounds.y;
        if (rect.width <= 4 || rect.height <= 2) continue;

        float rate = static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));
        if (rate <= 1.2f || rate >= 6.8f) continue;

        int cx = rect.x + rect.width / 2;
        int cy = rect.y + rect.height / 2;
        int bottom_y = map_full_y_to_track(rect.y + rect.height - 1, frame.rows);
        BoardDistanceLevel dist = calc_board_distance(bottom_y);
        if (dist != BoardDistanceLevel::mid) continue;
        if (bottom_y < MODEL_TOP || bottom_y > MODEL_BOTTOM) continue;

        if (area < YUV_MID_MIN_AREA) continue;

        int red_pixels = cv::countNonZero(red_mask(rect));
        float red_ratio = static_cast<float>(red_pixels) /
                          static_cast<float>(std::max(1, rect.area()));
        if (red_pixels <= 6 || red_ratio <= 0.35f) continue;      // 红色像素点和红色比例　待调

        int band = std::max(2, std::min(8, std::max(rect.height, rect.width / 4)));     // 白色检测区域的厚度

        // 检测白色区域的像素
        cv::Rect bottom_r = clipped_rect(rect.x - band, rect.y + rect.height, rect.width + band * 2, band);
        cv::Rect left_r = clipped_rect(rect.x - band, rect.y, band, rect.height);
        cv::Rect right_r = clipped_rect(rect.x + rect.width, rect.y, band, rect.height);

        float bottom_white = white_ratio(bottom_r);
        float left_white = white_ratio(left_r);
        float right_white = white_ratio(right_r);
        float side_white = std::max(left_white, right_white);
        float side_mean = (left_white + right_white) * 0.5f;
        float white_score = std::min(1.0f,
                                     side_white * 0.55f +
                                     side_mean * 0.25f +
                                     bottom_white * 0.20f);
        if (side_white < 0.32f || bottom_white < 0.38) continue;

        float red_score = std::min(red_ratio / 0.35f, 1.0f);
        float aspect_score = 1.0f - std::min(std::fabs(rate - 5.5f) / 3.0f, 1.0f);
        float size_score = static_cast<float>(std::min(area / (frame_area * 0.006), 1.0));
        float shape_score = std::min(1.0f, aspect_score * 0.65f + size_score * 0.35f);
        float lane_score = (cv::countNonZero(lane_mask(rect)) >= rect.area() * YUV_LANE_OVERLAP_MIN) ? 1.0f : 0.0f;
        if (lane_score <= 0.0f) continue;

        ModelMarkerLevel level = ModelMarkerLevel::strict;

        float score = red_score * 0.35f + shape_score * 0.25f + lane_score * 0.20f + white_score * 0.20f;
        // if (score < 0.35f) continue;

        if (score > best_score) {
            best_score = score;
            best_level = level;
            best.valid = true;
            best.full_rect = rect;
            best.track_rect = cv::Rect(map_full_x_to_track(rect.x, frame.cols),
                                       map_full_y_to_track(rect.y, frame.rows),
                                       std::max(1, rect.width * CAM_WIDTH / frame.cols),
                                       std::max(1, rect.height * CAM_HEIGHT / frame.rows));
            best.center_x = map_full_x_to_track(cx, frame.cols);
            best.center_y = map_full_y_to_track(cy, frame.rows);
            best.bottom_y = bottom_y;
            best.stable_rows = rect.height;
            best.score = score;
            best.distance = dist;
            best.source = MarkerDetectSource::yuv;
        }
    }

    if (!best.valid) return ModelMarkerLevel::none;
    candidate = best;
    return best_level;
}

// 寻找宽松红框候选
bool ModelDetector::find_loose_marker_candidate(cv::Mat& frame, TargetBoardCandidate& candidate)
{
    candidate = TargetBoardCandidate{};
    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) return false;

    cv::Mat hsv, mask_red1, mask_red2, mask_dark, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 宽松范围只给远距离锁定用　　包含暗红、棕红和低亮度深色目标
    cv::inRange(hsv, cv::Scalar(0, 35, 35), cv::Scalar(30, 255, 230), mask_red1);
    cv::inRange(hsv, cv::Scalar(145, 35, 35), cv::Scalar(180, 255, 230), mask_red2);
    cv::inRange(hsv, cv::Scalar(0, 0, 25), cv::Scalar(180, 255, 135), mask_dark);
    mask = mask_red1 | mask_red2 | mask_dark;

    cv::Mat lane_mask = cv::Mat::zeros(frame.size(), CV_8UC1);      // 掩码
    int last_trusted_mid = -1;          // 最近一次可信的赛道中间位置

    for (int track_y = VALID_END_ROW; track_y <= VALID_START_ROW; track_y++) {
        int track_mid = CAM_WIDTH / 2;
        int left_x = VALID_LEFT_COL;
        int right_x = VALID_RIGHT_COL;

        // 筛选条件１: 只使用左右边界都可信的行
        if (left_boundary.line[track_y] == -1 || right_boundary.line[track_y] == -1) {
            continue;
        }
        
        //　计算赛道中间位置　　　优先级：寻迹mid线　-> 左右边界线平均值 -> 图像中心
        if (mid.line[track_y] != -1) {
            track_mid = mid.line[track_y];
        }
        else if (left_boundary.line[track_y] != -1 && right_boundary.line[track_y] != -1) {
            track_mid = (left_boundary.line[track_y] + right_boundary.line[track_y]) / 2;
        }
        else if (left_boundary.line[track_y] != -1) {
            track_mid = left_boundary.line[track_y] + TrackBase::halfRoad[track_y];
        }
        else if (right_boundary.line[track_y] != -1) {
            track_mid = right_boundary.line[track_y] - TrackBase::halfRoad[track_y];
        }

        if (left_boundary.line[track_y] != -1) left_x = left_boundary.line[track_y];
        if (right_boundary.line[track_y] != -1) right_x = right_boundary.line[track_y];
        if (left_x > right_x) std::swap(left_x, right_x);

        int lane_width = right_x - left_x;

        // 筛选条件2: 赛道宽度合理
        int expected_width = TrackBase::halfRoad[track_y] * 2;      // 赛道宽度预期值
        if (expected_width <= 0) expected_width = VALID_RIGHT_COL - VALID_LEFT_COL;
        int min_width = std::max(8, (int)std::round(expected_width * 0.75f));
        int max_width = std::min(CAM_WIDTH - 1, (int)std::round(expected_width * 1.25f));
        if (lane_width < min_width || lane_width > max_width) {
            continue;
        }
        // // 使用历史中线
        // track_mid = long_center_at(track_y, track_mid);
        // track_mid = std::max(left_x, std::min(right_x, track_mid));

        // // 筛选条件3: 赛道中间位置不能离边界太远，且不能跳变过大
        // int boundary_mid = (left_x + right_x) / 2;
        // if (std::abs(track_mid - boundary_mid) > 4) {
        //     track_mid = boundary_mid;
        // }
        // if (last_trusted_mid >= 0 && std::abs(track_mid - last_trusted_mid) > 8) {
        //     continue;
        // }
        // last_trusted_mid = track_mid;

        int half_window = (int)std::round(lane_width * 0.15f);  // 自适应扫描窗半宽 已调 0.11f差不多是最下面　　也就是最大时
        
        // 扫描窗X轴左右边界
        int x0 = track_mid - half_window;
        int x1 = track_mid + half_window;

        if (x0 > x1) std::swap(x0, x1);

        // 完整分辨率扫描窗
        int full_y0 = map_track_y_to_full(track_y, frame.rows);
        int full_y1 = map_track_y_to_full(std::min(track_y + 1, CAM_HEIGHT - 1), frame.rows);
        int full_x0 = map_track_x_to_full(x0, frame.cols);
        int full_x1 = map_track_x_to_full(x1, frame.cols);
        if (full_x0 > full_x1) std::swap(full_x0, full_x1);
        if (full_y0 > full_y1) std::swap(full_y0, full_y1);

        cv::rectangle(lane_mask,
                      cv::Point(full_x0, full_y0),
                      cv::Point(full_x1, full_y1),
                      cv::Scalar(255),
                      cv::FILLED);
    }

    // 与赛道中间线和边界线的掩码相与
    cv::bitwise_and(mask, lane_mask, mask);

    // 开闭运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = static_cast<double>(frame.cols * frame.rows);
    const double min_area = frame_area * 0.00013;     // 10　待调　变大些
    const double max_area = frame_area * 0.0065;      // 500 待调　变小些
    float best_score = -1.0f;
    TargetBoardCandidate best;

    // 遍历轮廓 寻找最佳
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < min_area || area > max_area) continue;   // 面积限制 待调

        cv::Rect rect = cv::boundingRect(cnt);
        if (rect.width < 3 || rect.height < 2) continue;    // 宽高限制 待调

        // 掩码区域的红像素和暗像素比例判断 待调
        cv::Mat red_roi = mask(rect);
        cv::Mat dark_roi = mask_dark(rect);
        int red_pixels = cv::countNonZero(red_roi);
        int dark_pixels = cv::countNonZero(dark_roi);
        float red_ratio = static_cast<float>(red_pixels) /
                          static_cast<float>(std::max(1, rect.area()));
        float dark_ratio = static_cast<float>(dark_pixels) /
                           static_cast<float>(std::max(1, rect.area()));
        if (red_pixels < 3 || red_ratio < 0.05f) continue;

        float rate = static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));
        if (rate < 0.8f || rate > 6.0f) continue;           // 宽高比　待调

        int cx = rect.x + rect.width / 2;
        int cy = rect.y + rect.height / 2;
        if (!is_within_lane(cx, cy, frame.size())) continue; // 中线限制第一层

        int under_y = map_full_y_to_track(rect.y + rect.height - 1, frame.rows);
        BoardDistanceLevel dist = calc_board_distance(under_y);     // 距离判断 近距离不要
        if (dist == BoardDistanceLevel::none || dist == BoardDistanceLevel::near) continue;
        if (under_y < ROI_TOP || under_y >= ROI_BOTTOM ||
            left_boundary.line[under_y] == -1 || right_boundary.line[under_y] == -1) {
            continue;
        }

        // 中线限制第二层　赛道中间位置不能离边界太远，且不能跳变过大
        int track_mid = frame.cols / 2;
        if (under_y >= ROI_TOP && under_y < ROI_BOTTOM && mid.line[under_y] != -1) {
            track_mid = map_track_x_to_full(mid.line[under_y], frame.cols);
        }
        else if (under_y >= ROI_TOP && under_y < ROI_BOTTOM &&
                 left_boundary.line[under_y] != -1 && right_boundary.line[under_y] != -1) {
            int track_center = (left_boundary.line[under_y] + right_boundary.line[under_y]) / 2;
            track_mid = map_track_x_to_full(track_center, frame.cols);
        }
        else if (under_y >= ROI_TOP && under_y < ROI_BOTTOM && left_boundary.line[under_y] != -1) {
            int track_center = left_boundary.line[under_y] + TrackBase::halfRoad[under_y];
            track_mid = map_track_x_to_full(track_center, frame.cols);
        }
        else if (under_y >= ROI_TOP && under_y < ROI_BOTTOM && right_boundary.line[under_y] != -1) {
            int track_center = right_boundary.line[under_y] - TrackBase::halfRoad[under_y];
            track_mid = map_track_x_to_full(track_center, frame.cols);
        }

        // 可信中心线　赛道中间位置不能离边界太远，且不能跳变过大
        int trusted_center = (left_boundary.line[under_y] + right_boundary.line[under_y]) / 2;
        if (mid.line[under_y] != -1 && std::abs(mid.line[under_y] - trusted_center) <= 5) {
            trusted_center = mid.line[under_y];
        }
        track_mid = map_track_x_to_full(trusted_center, frame.cols);

        int center_err = std::abs(cx - track_mid);    // 中心误差
        int max_center_err = 8;                       // 最大偏离阈值　待调    1到2的偏差
        if (center_err > max_center_err) continue;

        // 屏蔽斑马线　用宽度和宽高比限制 待调
        if (rect.width > frame.cols * 0.22f && rate > 4.0f) continue;

        // 计算得分
        float center_score = 1.0f - std::min(static_cast<float>(center_err) / max_center_err, 1.0f);
        float area_score = static_cast<float>(std::min(area / (frame_area * 0.01), 1.0));   // 768　待调　小
        float dark_score = std::min(dark_ratio, 1.0f) * 0.5f;
        float distance_score = (dist == BoardDistanceLevel::mid) ? 1.0f : 0.5f;
        float score = center_score * 3.0f + area_score + dark_score + distance_score;

        // 更新最佳候选
        if (score > best_score) {
            best_score = score;
            best.valid = true;
            best.full_rect = rect;
            best.track_rect = cv::Rect(map_full_x_to_track(rect.x, frame.cols),
                                       map_full_y_to_track(rect.y, frame.rows),
                                       std::max(1, rect.width * CAM_WIDTH / frame.cols),
                                       std::max(1, rect.height * CAM_HEIGHT / frame.rows));
            best.center_x = map_full_x_to_track(cx, frame.cols);
            best.center_y = map_full_y_to_track(cy, frame.rows);
            best.bottom_y = under_y;
            best.stable_rows = rect.height;
            best.score = score;
            best.distance = dist;
        }
    }

    if (!best.valid) return false;
    candidate = best;
    return true;
}

// 没看到严格红框时，用远距离锁定的宽松候选往上推固定 ROI
bool ModelDetector::build_target_roi_from_candidate(const TargetBoardCandidate& candidate,
                                                    const cv::Size& frame_size,
                                                    cv::Rect& roi_rect) const
{
    if (!candidate.valid || frame_size.width <= 0 || frame_size.height <= 0) return false;

    // 初始化
    int side = (int)std::round(candidate.full_rect.width * 0.95f); // 中远处目标板roi边长 待调 估计要缩小 17
    side = std::max(side, 12); // 限制最小边长为 12 像素，防止 ROI 过小导致模型报错

    int center_x = candidate.full_rect.x + (candidate.full_rect.width / 2); // 目标板中心
    int track_y = candidate.bottom_y;                                       // 目标板底部y坐标

    // 黑色目标板X轴中心坐标变化　　
    if (mid.line[track_y] != -1) {
        int mid_full = map_track_x_to_full(mid.line[track_y], frame_size.width);
        center_x = (center_x + mid_full) / 2;               // 让黑框中心尽量靠一下中线　微调了一下
    }

    // 黑色目标板底部坐标变化
    int bottom_full = map_track_y_to_full(candidate.bottom_y, frame_size.height);
    
    // 向上强推的像素点
    int fixed_y = std::max(4, (int)(side * 0.35f));     // 目标板往上固定推的距离　待调  6
    int y = bottom_full - fixed_y;                      // 要去模型的目标板roi的起算高度y坐标
    int x = center_x - side / 2;                        // 目标板左边坐标

    // int slide = std::max(6, side / 5);              // 往上推固定的额外的像素高度
    // y -= slide / 2;

    // 限幅
    x = std::max(0, std::min(x, frame_size.width - 1));
    y = std::max(0, std::min(y, frame_size.height - 1));

    // 裁剪roi目标板区域的宽和高
    int w = std::min(side, frame_size.width - x);
    int h = std::min(side, frame_size.height - y);
    
    // 如果受限截取到的 ROI 过小，则放弃
    if (w < 10 || h < 10) return false;

    roi_rect = cv::Rect(x, y, w, h);
    return true;
}


// 辅助函数：坐标映射　全局坐标 320 * 240 <-> 赛道坐标 80 * 60
namespace {
        
    // 全局坐标 320 * 240 <-> 赛道坐标 80 * 60  
    int map_full_x_to_track(int x, int full_width)
    {
        if (full_width <= 0) return 0;
        return std::max(0, std::min(CAM_WIDTH - 1, x * CAM_WIDTH / full_width));
    }

    // 全局坐标 320 * 240 <-> 赛道坐标 80 * 60  
    int map_full_y_to_track(int y, int full_height)
    {
        if (full_height <= 0) return 0;
        return std::max(0, std::min(CAM_HEIGHT - 1, y * CAM_HEIGHT / full_height));
    }

    // 赛道坐标 80 * 60 <-> 全局坐标 320 * 240
    int map_track_x_to_full(int x, int full_width)
    {
        return std::max(0, std::min(full_width - 1,
            static_cast<int>(std::lround((x + 0.5f) * full_width / CAM_WIDTH))));
    }

    // 赛道坐标 80 * 60 <-> 全局坐标 320 * 240
    int map_track_y_to_full(int y, int full_height)
    {
        return std::max(0, std::min(full_height - 1,
            static_cast<int>(std::lround((y + 0.5f) * full_height / CAM_HEIGHT))));
    }
}
    
// 对 ROI 做 ncnn 三分类
ModelResult ModelDetector::infer_roi(cv::Mat& frame, const cv::Rect& roi_rect)
{
    // 初始化
    ModelResult result = {"", 0.0f, false, roi_rect};
    if (!model_ready || frame.empty() || roi_rect.area() <= 0) {
        last_result = result;
        return result;
    }

    // cv::Mat roi_img = frame(roi_rect);
    cv::Mat roi_img = frame(roi_rect).clone(); // 克隆 ROI 图像
    
    // =========================================================
    // NCNN 推理部分
    // =========================================================
    // 输入图像转换和归一化
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(
        roi_img.data,
        ncnn::Mat::PIXEL_BGR2RGB,       // 这里必须是 BGR 转 RGB
        roi_img.cols, roi_img.rows,
        input_size, input_size
    );

    // 归一化参数 对齐 PyTorch 的 transforms.Normalize
    // OpenCV 像素是 0~255 减去 127.5 再乘以 1/127.5，范围就完美变成了 -1.0 ~ 1.0
    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {1/127.5f, 1/127.5f, 1/127.5f};
    in.substract_mean_normalize(mean_vals, norm_vals);      // 标准化

    // 创建提取器 进行推理
    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in);             // 输入层
    ncnn::Mat out;
    ex.extract("out0", out);         // 输出层
    
    if (out.w <= 0) {
        last_result = result;
        return result;
    }
    if (out.w != 3) {
        std::cerr << "[Model] invalid output size: " << out.w << std::endl;
        last_result = result;
        return result;
    }

    // 计算概率分布　　先减最大 logit 再 softmax，避免 exp 数值过大
    // Softmax函数 计算概率分布
    float max_logit = out[0];
    for (int j = 1; j < out.w; j++) {
        if (out[j] > max_logit) max_logit = out[j];
    }

    std::vector<float> scores(out.w);
    float sum = 0.0f;
    for (int j = 0; j < out.w; j++) {
        scores[j] = std::exp(out[j] - max_logit);
        sum += scores[j];
    }
    if (sum <= 0.0f) {
        last_result = result;
        return result;
    }

    int max_idx = 0;
    float max_prob = 0.0f;
    for (int j = 0; j < out.w; j++) {
        scores[j] /= sum;
        if (j < 3) {
            result.class_scores[j] = scores[j];
        }
        if (scores[j] > max_prob) {
            max_prob = scores[j];
            max_idx = j;
        }
    }
    result.class_count = std::min(out.w, 3);

    if (max_idx < 0 || max_idx >= static_cast<int>(class_names.size())) {
        last_result = result;
        return result;
    }

    // 结果处理  识别到的类别及置信度赋值给 result 结构体
    std::string current_class = class_names[max_idx];
    result.class_name = current_class;
    result.probability = max_prob;

    // 单次置信度超过阈值后，再要求连续同类，得到 result.is_detected
    if (max_prob > 0.5f) {
        if (current_class == last_class_name) {
            same_class_counter++;
        } 
        else {
            same_class_counter = 1;
            last_class_name = current_class;
        }
    } 
    else {
        same_class_counter = 0;
    }

    // 连续识别为同一物体的计数器达到阈值，认为目标已经被识别
    if (same_class_counter >= CONFIRM_THRESHOLD) {
        result.is_detected = true;
        if(same_class_counter > 20) same_class_counter = 20; 
    }

    last_result = result;
    return result;
}

// 缓存模型调试显示信息
void ModelDetector::cache_model_debug_display(const ModelActionResult& status,
                                              const cv::Mat& frame,
                                              const cv::Rect& roi_rect)
{
    if (!status.model_ran || status.model.class_count != 3) return;

    for (int i = 0; i < 3; ++i) {
        display_model_scores[i] = status.model.class_scores[i];
    }

    int top1_idx = 0;
    int top2_idx = 1;
    if (display_model_scores[top2_idx] > display_model_scores[top1_idx]) {
        std::swap(top1_idx, top2_idx);
    }
    for (int i = 2; i < 3; ++i) {
        if (display_model_scores[i] > display_model_scores[top1_idx]) {
            top2_idx = top1_idx;
            top1_idx = i;
        }
        else if (display_model_scores[i] > display_model_scores[top2_idx]) {
            top2_idx = i;
        }
    }

    const char* short_names[3] = {"S", "T", "W"};
    display_model_top = short_names[top1_idx];
    display_model_margin = display_model_scores[top1_idx] - display_model_scores[top2_idx];
    display_model_debug_frames = MARKER_DISPLAY_FRAMES;
    display_model_debug_ran = true;
    display_model_infer_ms = status.infer_ms;
    display_model_confirm_count = same_class_counter;
    display_model_vote_count = action_vote_count;

    cv::Rect safe_roi = roi_rect & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_roi.area() > 0) {
        cv::Mat raw_roi = frame(safe_roi).clone();
        cv::resize(raw_roi, display_roi_preview, cv::Size(input_size, input_size));
    }
}

// 红色框矩形检测  HSV 色块检测 　　采用综合选择：面积、宽高比、矩形度、是否在赛道内、是否靠近中线、位置是否合理
ModelMarkerLevel ModelDetector::find_red_marker(cv::Mat& frame, TargetBoardCandidate& candidate)
{
    candidate = TargetBoardCandidate{};
    if (frame.empty()) return ModelMarkerLevel::none;

    // 在急弯/圆环场景放宽远处红框门槛，直道保持原来筛选
    bool curve_context = (element_state.current_track_type == TrackType::circle) ||
                         (element_state.normal_type == NormalType::curve && element_state.curve_score >= 0.15f);

    // HSV 色块检测 (红色)
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 35, 55), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 45, 35), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // 开闭运算去掉红色噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 轮廓检测 筛选
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 面积阈值随分辨率自适应　待调
    const double frame_area = static_cast<double>(frame.cols * frame.rows);
    const double min_area = curve_context ?
                            std::max(40.0, frame_area * 0.00065f) :// 50
                            std::max(80.0, frame_area * 0.0013f);  // 100
    const double max_area = frame_area * 0.0163f;                  // 1250  扩大　为急救包考虑
    const float aspect_target = 3.6f;                             // 宽高比目标               阈值 待调

    bool found_target = false;
    double best_score = -1.0;
    TargetBoardCandidate best_candidate;
    ModelMarkerLevel best_level = ModelMarkerLevel::none;

    for (const auto& cnt : contours) {
        // 筛选条件1 色块面积阈值
        double area = cv::contourArea(cnt);
        if (area < min_area || area > max_area) continue;

        // 筛选条件2 红色矩形宽高阈值
        cv::Rect rect = cv::boundingRect(cnt);
        if (rect.width < 7 || rect.height < 2) continue;   //　差不多是15~17

        // 红色框是横边更长的矩形　宽高比阈值　待调
        float rate = static_cast<float>(rect.width) / static_cast<float>(rect.height);
        float min_rate = curve_context ? 0.8f : 1.8f;
        float max_rate = curve_context ? 6.0f : 4.5f;
        if (rate < min_rate || rate > max_rate) continue;   // 直道差不多是3~4

        // // 矩形度越高，越像打印出来的红色框
        // float rectangularity = static_cast<float>(area) / static_cast<float>( rect.width * rect.height);
        // if (rectangularity < 0.45f) continue; 

        int cx = rect.x + rect.width / 2;           // 红框中心x坐标
        int cy = rect.y + rect.height / 2;          // 红框中心y坐标
        
        // 红色框在目标板下方，出现在画面最上方不进行识别框选　同时屏蔽头顶的横幅等　待调
        if (cy <= frame.rows * 0.3f) continue;         // 72
        if (!is_within_lane(cx, cy, frame.size())) continue;

        // 出现在画面最下方不进行识别框选　待调
        if (cy >= frame.rows * 0.784f) continue;        // 188  VALID_START_ROW
        if (!is_within_lane(cx, cy, frame.size())) continue;

        // 筛选条件3 红框必须在赛道中线附近  优先级：寻迹mid线　-> 左右边界线平均值 -> 图像中心
        int track_mid = frame.cols / 2;                          // 中线坐标
        int track_y = map_full_y_to_track(cy, frame.rows);
        if (track_y >= ROI_TOP && track_y < ROI_BOTTOM && mid.line[track_y] != -1) {
            track_mid = map_track_x_to_full(mid.line[track_y], frame.cols);
        } 
        else if (track_y >= ROI_TOP && track_y < ROI_BOTTOM &&
                   left_boundary.line[track_y] != -1 && right_boundary.line[track_y] != -1) {
            track_mid = (left_boundary.line[track_y] + right_boundary.line[track_y]) / 2;
            track_mid = map_track_x_to_full(track_mid, frame.cols);
        }

        float center_err = std::abs(cx - track_mid);
        float max_center_err = curve_context ? 12.0f : 8.0f;   // 待调
        if (center_err > max_center_err) continue;

        int bottom_y = map_full_y_to_track(rect.y + rect.height - 1, frame.rows);
        BoardDistanceLevel distance = calc_board_distance(bottom_y);
        if (distance == BoardDistanceLevel::none || distance == BoardDistanceLevel::near) continue;

        // 红框分数计算　
        float center_score = 1.0f - std::min(center_err / max_center_err, 1.0f);
        float aspect_score = 1.0f - std::min(std::abs(rate - aspect_target) / aspect_target, 1.0f);
        // float rect_score = std::min(rectangularity, 1.0f);
        float lower_score = static_cast<float>(cy) / static_cast<float>(std::max(1, frame.rows));
        float area_score = static_cast<float>(std::min(area / (frame_area * 0.03), 1.0));

        // 综合评分
        double score = center_score * 3.0 + aspect_score * 2.0 + lower_score + area_score;
        // if (score < 2.45f) continue;        // 最低分数限制　0.35

        // 等级只由距离决定
        ModelMarkerLevel level = (distance == BoardDistanceLevel::mid)
                                     ? ModelMarkerLevel::strict
                                     : ModelMarkerLevel::warning;

        // 筛选后选择综合分数最高的候选
        if (score > best_score) {
            best_score = score;
            best_level = level;
            best_candidate.valid = true;
            best_candidate.full_rect = rect;
            best_candidate.track_rect = cv::Rect(map_full_x_to_track(rect.x, frame.cols),
                                                 map_full_y_to_track(rect.y, frame.rows),
                                                 std::max(1, rect.width * CAM_WIDTH / frame.cols),
                                                 std::max(1, rect.height * CAM_HEIGHT / frame.rows));
            best_candidate.center_x = map_full_x_to_track(cx, frame.cols);
            best_candidate.center_y = map_full_y_to_track(cy, frame.rows);
            best_candidate.bottom_y = bottom_y;
            best_candidate.stable_rows = rect.height;
            best_candidate.score = static_cast<float>(score);
            best_candidate.distance = distance;
            best_candidate.source = MarkerDetectSource::hsv;
            found_target = true;
        }
    }

    if (!found_target) return ModelMarkerLevel::none;
    candidate = best_candidate;
    return best_level;
}

// 兼容旧接口：只返回是否检测到红框和红框位置
bool ModelDetector::find_red_marker(cv::Mat& frame, cv::Rect& target_rect)
{
    TargetBoardCandidate candidate;
    ModelMarkerLevel level = find_red_marker(frame, candidate);
    if (level == ModelMarkerLevel::none || !candidate.valid) return false;
    target_rect = candidate.full_rect;
    return true;
}


// 补丁  急救包补救检测: 红框结合急救包面积大于限制时，若其他条件均满足　直接按物资右绕处理
bool ModelDetector::find_large_red_supplies(cv::Mat& frame, cv::Rect& target_rect)
{
    if (frame.empty()) return false;

    // 在急弯/圆环场景放宽远处红框门槛，直道保持原来筛选
    bool curve_context = (element_state.current_track_type == TrackType::circle) ||
                         (element_state.normal_type == NormalType::curve && element_state.curve_score >= 0.15f);

    // HSV 色块检测 (红色)
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 35, 55), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 45, 35), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // 开闭运算去掉红色噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 轮廓检测 筛选
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 面积阈值随分辨率自适应　待调
    const double frame_area = static_cast<double>(frame.cols * frame.rows);
    const double marker_max_area = frame_area * 0.011;      // 真红框最大面积 待调 844
    const double supplies_max_area = frame_area * 0.080;    // 急救包最大面积 待调 

    bool found_target = false;
    double best_score = -1.0;

    cv::Rect best_rect;
    for (const auto& cnt : contours) {
        // 筛选条件1 色块面积阈值
        double area = cv::contourArea(cnt);
        if (area <= marker_max_area || area > supplies_max_area) continue;

        // 筛选条件2 红色矩形宽高阈值
        cv::Rect rect = cv::boundingRect(cnt);
        if (rect.width < 7 || rect.height < 7) continue;   //　待调　跟红框相比应该只有高增加了

        int cx = rect.x + rect.width / 2;
        int cy = rect.y + rect.height / 2;
        
        // 红色框在目标板下方，出现在画面最上方不进行识别框选　同时屏蔽头顶的横幅等　待调
        if (cy <= frame.rows * 0.3f) continue;         // 72
        if (!is_within_lane(cx, cy, frame.size())) continue;

        // 出现在画面最下方不进行识别框选　待调
        if (cy >= frame.rows * 0.784f) continue;        // 188  VALID_START_ROW
        if (!is_within_lane(cx, cy, frame.size())) continue;

        // 筛选条件3 红框必须在赛道中线附近  优先级：寻迹mid线　-> 左右边界线平均值 -> 图像中心
        int track_mid = frame.cols / 2;                          // 中线坐标
        int track_y = map_full_y_to_track(cy, frame.rows);
        if (track_y >= ROI_TOP && track_y < ROI_BOTTOM && mid.line[track_y] != -1) {
            track_mid = map_track_x_to_full(mid.line[track_y], frame.cols);
        } 
        else if (track_y >= ROI_TOP && track_y < ROI_BOTTOM &&
                   left_boundary.line[track_y] != -1 && right_boundary.line[track_y] != -1) {
            track_mid = (left_boundary.line[track_y] + right_boundary.line[track_y]) / 2;
            track_mid = map_track_x_to_full(track_mid, frame.cols);
        }

        float center_err = std::abs(cx - track_mid);
        float max_center_err = curve_context ? 12.0f : 8.0f;   // 待调
        if (center_err > max_center_err) continue;

        // 红框分数计算　
        float center_score = 1.0f - std::min(center_err / max_center_err, 1.0f);
        // float aspect_score = 1.0f - std::min(std::abs(rate - aspect_target) / aspect_target, 1.0f);
        // float rect_score = std::min(rectangularity, 1.0f);
        float lower_score = static_cast<float>(cy) / static_cast<float>(std::max(1, frame.rows));
        float area_score = static_cast<float>(std::min(area /(marker_max_area + supplies_max_area), 1.0));
       
        double score = center_score * 3.0 + lower_score + area_score;

        if (score > best_score) {
            best_score = score;
            best_rect = rect;
            found_target = true;
        }
    }

    if (!found_target) return false;
    target_rect = best_rect;
    return true;
}
    
// 根据红色提示矩形计算目标板 ROI
bool ModelDetector::build_target_roi(const cv::Rect& marker_rect, const cv::Size& frame_size, cv::Rect& roi_rect) const
{
    if (marker_rect.width <= 0 || marker_rect.height <= 0 ||
        frame_size.width <= 0 || frame_size.height <= 0) {
        return false;
    }

    // ROI大小和位置计算初始化
    const float roi_scale = 1.05f;           // ROI大小相对于红框的缩放比例　待调　减小
    const float bottom_margin_scale = 0.03f; // 红框下方保留一点余量，避免贴边裁掉红框
    int side = static_cast<int>(std::round(marker_rect.width * roi_scale));
    side = std::max(side, 12);

    int center_x = marker_rect.x + marker_rect.width / 2;
    int marker_bottom = marker_rect.y + marker_rect.height;
    int bottom_margin = static_cast<int>(std::round(side * bottom_margin_scale));

    int x = center_x - side / 2;
    int y = marker_bottom + bottom_margin - side;

    int unclamped_y = y;
    int unclamped_bottom = y + side;
    x = std::max(0, std::min(x, frame_size.width - 1));
    y = std::max(0, std::min(y, frame_size.height - 1));

    int w = std::min(side, frame_size.width - x);
    int h = std::min(side, frame_size.height - y);

    // 如果目标板 ROI 大部分已经超出画面，就跳过这一帧
    if (unclamped_y < 0 && h < side * 0.70f) return false;
    if (unclamped_bottom > frame_size.height && h < side * 0.70f) return false;
    if (w < 8 || h < 8) return false;

    roi_rect = cv::Rect(x, y, w, h);
    return true;
}

// 类别名称对应动作类型
ActionType ModelDetector::class_to_action(const std::string& class_name) const
{
    // 武器 weapon -> 左绕   物资 supplies -> 右绕   载具 transport -> 直行
    if (class_name == "weapon")   return ACTION_LEFT;
    if (class_name == "supplies") return ACTION_RIGHT;
    return ACTION_STRAIGHT;
}

// 粗略判断红框中心是否在赛道左右边界之间
// 这只是第一层约束，find_red_marker() 里还会再做中线附近约束
bool ModelDetector::is_within_lane(int cx, int cy, const cv::Size& frame_size)
{
    if (frame_size.width <= 0 || frame_size.height <= 0) return false;

    const int track_y = map_full_y_to_track(cy, frame_size.height);
    int left_x = 0;
    int right_x = frame_size.width - 1;

    if (track_y >= 0 && track_y < CAM_HEIGHT) {
        if (left_boundary.line[track_y] != -1) {
            left_x = map_track_x_to_full(left_boundary.line[track_y], frame_size.width);
        }
        if (right_boundary.line[track_y] != -1) {
            right_x = map_track_x_to_full(right_boundary.line[track_y], frame_size.width);
        }
    }

    if (left_x > right_x) std::swap(left_x, right_x);

    const int lane_margin = std::max(20, frame_size.width / 16);
    return (cx >= left_x - lane_margin && cx <= right_x + lane_margin);
}

cv::Rect ModelDetector::get_safe_roi(const cv::Rect& box, int padding, int w, int h) {
    int x = std::max(0, box.x - padding);
    int y = std::max(0, box.y - padding);
    int roi_w = std::min(w - x, box.width + 2 * padding);
    int roi_h = std::min(h - y, box.height + 2 * padding);
    return cv::Rect(x, y, roi_w, roi_h);
}

// 模型图传显示
void ModelDetector::display_model_upimage(cv::Mat& frame, const ModelActionResult& status)
{
    if (frame.empty()) return;

    // 重置显示状态
    if (display_frame_size != frame.size()) {
        display_frame_size = frame.size();
        display_marker_box = cv::Rect(0, 0, 0, 0);
        display_marker_type = MarkerVisualType::none;
        display_marker_label.clear();
        display_marker_frames = 0;
        display_roi_box = cv::Rect(0, 0, 0, 0);
        display_roi_frames = 0;
        display_roi_preview.release();
        display_model_debug_frames = 0;
    }

    // // 画中线
    // for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
    //     if (y < 0 || y >= CAM_HEIGHT || mid.line[y] < 0 || mid.line[y] >= CAM_WIDTH) continue;
    //     int full_x = map_track_x_to_full(mid.line[y], frame.cols);
    //     int full_y = map_track_y_to_full(y, frame.rows);
    //     cv::circle(frame, cv::Point(full_x, full_y), 2, cv::Scalar(0, 255, 255), -1);
    // }

    // // 画历史中线
    // for (int y = VALID_END_ROW; y <= VALID_START_ROW; y++) {
    //     if (y < 0 || y >= CAM_HEIGHT || !memory_mid_valid[y]) continue;
    //     int full_x = map_track_x_to_full(memory_mid_line[y], frame.cols);
    //     int full_y = map_track_y_to_full(y, frame.rows);
    //     cv::circle(frame, cv::Point(full_x, full_y), 1, cv::Scalar(255, 255, 0), -1);
    // }

    // 菜单红框黄、模型红框蓝 ROI 绿、紫框表示宽松疑似目标     急救包(已注释)
    if (status.board_detected && status.board_box.area() > 0) {
        cv::rectangle(frame, status.board_box, cv::Scalar(255, 0, 255), 2);
    }

    bool marker_updated = false;
    if (status.marker_detected && status.marker_box.area() > 0) {

        // 更新状态
        display_marker_box = status.marker_box;
        display_marker_type = status.marker_visual_type;
        display_marker_frames = MARKER_DISPLAY_FRAMES;
        marker_updated = true;

        // 只有模型红框才显示 ROI，其他类型红框不显示 ROI
        if (status.marker_visual_type != MarkerVisualType::model_marker) {
            display_roi_box = cv::Rect(0, 0, 0, 0);
            display_roi_frames = 0;
        }

        // 标签文本 分类不同类型的识别类型
        char marker_label[64] = {0};
        const char* source_label = marker_source_to_text(status.marker_source);
        if (status.marker_visual_type == MarkerVisualType::large_supplies) {
            std::snprintf(marker_label, sizeof(marker_label), "SUPPLY-R");
        }
        else if (status.marker_visual_type == MarkerVisualType::queue_marker) {
            if (status.action_confirmed) {
                std::snprintf(marker_label, sizeof(marker_label), "Q:%s:%s",
                              source_label, model_calib.action_to_ascii(status.action));
            }
            else if (status.distance == BoardDistanceLevel::far) {
                std::snprintf(marker_label, sizeof(marker_label), "Q:%s:FAR", source_label);
            }
            else {
                std::snprintf(marker_label, sizeof(marker_label), "QUEUE:%s", source_label);
            }
        }
        else if (status.marker_visual_type == MarkerVisualType::model_marker) {
            std::snprintf(marker_label, sizeof(marker_label), "MODEL:%s", source_label);
        }
        else if (status.marker_visual_type == MarkerVisualType::loose_warning) {
            std::snprintf(marker_label, sizeof(marker_label), "WARN:%s", source_label);
        }
        display_marker_label = marker_label;
    }

    // 画不同颜色的框子
    if (display_marker_frames > 0 && display_marker_box.area() > 0) {
        cv::Scalar marker_color(255, 0, 0);
        if (display_marker_type == MarkerVisualType::queue_marker) {
            marker_color = cv::Scalar(0, 255, 255);
        }
        else if (display_marker_type == MarkerVisualType::loose_warning) {
            marker_color = cv::Scalar(255, 0, 255);
        }
        // else if (display_marker_type == MarkerVisualType::large_supplies) {
        //     marker_color = cv::Scalar(255, 0, 255);
        // }

        // 显示标签文本
        cv::rectangle(frame, display_marker_box, marker_color, 4);
        if (!display_marker_label.empty()) {
            cv::putText(frame, display_marker_label,
                        cv::Point(display_marker_box.x,
                                  std::max(10, display_marker_box.y - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, marker_color, 1);
        }
        if (!marker_updated) display_marker_frames--;
    }

    // 画 ROI 框子，只在模型红框时显示
    bool roi_updated = false;
    if (status.roi_box.area() > 0) {
        display_roi_box = status.roi_box;
        display_roi_frames = MARKER_DISPLAY_FRAMES;
        roi_updated = true;
    }
    if (display_roi_frames > 0 && display_roi_box.area() > 0) {
        cv::rectangle(frame, display_roi_box, cv::Scalar(0, 255, 0), 4);
        cv::putText(frame, "ROI",
                    cv::Point(display_roi_box.x,
                              std::min(frame.rows - 4,
                                       display_roi_box.y + display_roi_box.height + 12)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
        if (!roi_updated) display_roi_frames--;
    }

    // // 文字显示红框稳定计数、动作投票计数、冷却计数
    // char line1[96];
    // std::snprintf(line1, sizeof(line1), "M:%d/%d V:%d/%d CD:%d",
    //               status.marker_count, MARKER_CONFIRM_THRESHOLD,
    //               status.vote_count, ACTION_CONFIRM_THRESHOLD,
    //               status.cooldown_count);
    // cv::putText(frame, line1, cv::Point(3, 12),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

    // 文字显示类别和动作
    // char line1[128];
    // std::snprintf(line1, sizeof(line1), "M:%d/%d V:%d/%d CD:%d ROI:%dx%d",
    //               status.marker_count, MARKER_CONFIRM_THRESHOLD,
    //               status.vote_count, ACTION_CONFIRM_THRESHOLD,
    //               status.cooldown_count,
    //               status.roi_box.width, status.roi_box.height);
    // cv::putText(frame, line1, cv::Point(3, 12),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

    // // 左上角文字显示目标板距离、目标板来源、目标板置信度
    // char line1[128];
    // std::snprintf(line1, sizeof(line1), "L:%d %s %.1f R:%s",
    //               status.board_count,
    //               board_distance_to_text(status.distance),
    //               status.board_score,
    //               roi_source_to_text(status.roi_source));
    // cv::putText(frame, line1, cv::Point(3, 12),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

    // char line2[128];
    // if (status.model_ran && !status.model.class_name.empty()) {
    //     std::snprintf(line2, sizeof(line2), "%s %.2f -> %s%s",
    //                   status.model.class_name.c_str(), status.model.probability,
    //                   model_calib.action_to_ascii(status.action),
    //                   status.action_confirmed ? " OK" : "");
    // } 
    // else if (status.marker_detected) {
    //     std::snprintf(line2, sizeof(line2), "marker%s",
    //                   status.marker_confirmed ? " confirmed" : "");
    // } 
    // // 宽松识别已不再使用
    // // else if (status.board_detected) {
    // //     std::snprintf(line2, sizeof(line2), "loose %s",
    // //                   board_distance_to_text(status.distance));
    // // }
    // else {
    //     std::snprintf(line2, sizeof(line2), "no marker");
    // }
    // cv::putText(frame, line2, cv::Point(3, 26),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

    // 左上角显示误差
    char err_line[64];
    std::snprintf(err_line, sizeof(err_line), "ERR %.2f", final_photo_err);

    cv::putText(frame, err_line,
                cv::Point(5, 15),
                cv::FONT_HERSHEY_SIMPLEX,
                0.40,
                cv::Scalar(0, 255, 255),
                1);

    // 左上角显示模型状态机
    const char* state_text = "NORMAL";
    switch (target_state)
    {
        case ModelTargetState::warning: state_text = "WARNING"; break;
        case ModelTargetState::recognizing: state_text = "RECOG"; break;
        case ModelTargetState::passing: state_text = "PASS"; break;
        case ModelTargetState::returning: state_text = "RETURN"; break;
        case ModelTargetState::fail_safe: state_text = "FAIL"; break;
        default: break;
    }
    char state_line[64];
    std::snprintf(state_line, sizeof(state_line), "MSTATE %s", state_text);
    cv::putText(frame, state_line,
                cv::Point(5, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.40,
                cv::Scalar(0, 255, 255),
                1);


    // 右上角显示绕行编码器积分，结束后的误差平滑阶段显示 PASS-END
    if (display_model_debug_frames > 0) {
        bool model_debug_updated = status.model_ran && status.model.class_count == 3;
        if (model_debug_updated) {
            display_model_confirm_count = same_class_counter;
            display_model_vote_count = action_vote_count;
            display_model_infer_ms = status.infer_ms;
        }
        const char* run_text = model_debug_updated ? "RUN" : "HOLD";

        char score_line[128];
        std::snprintf(score_line, sizeof(score_line),
                      "S:%.2f T:%.2f W:%.2f",
                      display_model_scores[0],
                      display_model_scores[1],
                      display_model_scores[2]);

        char top_line[128];
        std::snprintf(top_line, sizeof(top_line),
                      "TOP:%s M:%.2f",
                      display_model_top.empty() ? "-" : display_model_top.c_str(),
                      display_model_margin);

        char run_line[128];
        std::snprintf(run_line, sizeof(run_line),
                      "%s C:%d/%d V:%d/%d I:%.1fms",
                      run_text,
                      display_model_confirm_count,
                      CONFIRM_THRESHOLD,
                      display_model_vote_count,
                      ACTION_CONFIRM_THRESHOLD,
                      display_model_infer_ms);

        cv::putText(frame, score_line, cv::Point(5, 46),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(0, 255, 255), 1);
        cv::putText(frame, top_line, cv::Point(5, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(0, 255, 255), 1);
        cv::putText(frame, run_line, cv::Point(5, 74),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(0, 255, 255), 1);

        if (!display_roi_preview.empty()) {
            cv::Mat preview;
            cv::resize(display_roi_preview, preview, cv::Size(96, 96), 0, 0, cv::INTER_NEAREST);
            int px = std::max(0, frame.cols - preview.cols - 6);
            int py = std::max(0, frame.rows - preview.rows - 6);
            cv::Rect dst_rect(px, py, preview.cols, preview.rows);
            preview.copyTo(frame(dst_rect));
            cv::rectangle(frame, dst_rect, cv::Scalar(0, 255, 0), 1);
            cv::putText(frame, "MODEL INPUT",
                        cv::Point(px, std::max(10, py - 4)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.32,
                        cv::Scalar(0, 255, 0),
                        1);
        }

        if (!model_debug_updated) display_model_debug_frames--;
        display_model_debug_ran = model_debug_updated;
    }

    char enc_line[64] = {0};
    if (element_state.model_line_pass_active) {
        std::snprintf(enc_line, sizeof(enc_line), "ENC %.1f/%.1f",
                      motor_distance, element_state.model_line_pass_distance);
    }
    else if (continue_pass) {
        std::snprintf(enc_line, sizeof(enc_line), "PASS-END %.1f/%.1f", 
                     motor_distance, model_return_distance);
        // std::snprintf(enc_line, sizeof(enc_line), "PASS-END");

    }

    if (enc_line[0] != '\0') {
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(enc_line, cv::FONT_HERSHEY_SIMPLEX,
                                             0.40, 1, &baseline);
        int x = std::max(3, frame.cols - text_size.width - 30);
        int y = 14;
        cv::rectangle(frame,
                      cv::Rect(x - 3, y - text_size.height - 3,
                               text_size.width + 6, text_size.height + baseline + 6),
                      cv::Scalar(0, 0, 0),
                      cv::FILLED);
        cv::putText(frame, enc_line, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(0, 255, 255), 1);
    }
}

ModelDetector model_detector;
