#include "headfile.h"


///////////////////////////////////////////////// 　结构体定义及赋值 ///////////////////////////////////////////

// 左边界数据结构体，存储左边界的原始点、处理点等信息
// 注: 该结构体包含 std::vector 等成员，不能用 {0} 初始化；改为默认构造(最小改动)
PointState::BoundaryData left_boundary;

// 右边界数据结构体，存储右边界的原始点、处理点等信息
PointState::BoundaryData right_boundary;

// 中线数据结构体，存储中线的原始点、处理点、距离数组等信息
PointState::MidlineData mid;



///////////////////////////////////////////////// 　外部函数接口 ///////////////////////////////////////////

static void DP_Update_Left_Lost(PointState::BoundaryData& data);
static void DP_Update_Right_Lost(PointState::BoundaryData& data);
static uint8_t DP_Update_Left_Mnt(PointState::BoundaryData& data);
static uint8_t DP_Update_Right_Mnt(PointState::BoundaryData& data);
static cv::Point Get_Left_UTL_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_Left_RTU_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_Left_UTR_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_Right_UTR_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_Right_LTU_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_Right_UTL_Point(PointState::BoundaryData& data, uint8_t mode);
static cv::Point Get_L_Arc_Turn_Point(PointState::BoundaryData& data);
static cv::Point Get_R_Arc_Turn_Point(PointState::BoundaryData& data);

// 辅助函数：范围检查
static bool in_range(int v, int lo, int hi) {
    return v >= lo && v <= hi;
}

// 辅助函数：检查点是否在下拐点ROI内
static bool in_down_roi(const cv::Point &p, const ElementState::Limit &limit) {
    if (p.x < 0 || p.y < 0) return false;
    if (!limit.enable_spatial_constraint) return true;
    return in_range(p.x, limit.down_x_min, limit.down_x_max) &&
           in_range(p.y, limit.down_y_min, limit.down_y_max);
}

// 辅助函数：检查点是否在中拐点ROI内
static bool in_mid_roi(const cv::Point &p, const ElementState::Limit &limit) {
    if (p.x < 0 || p.y < 0) return false;
    if (!limit.enable_spatial_constraint) return true;
    return in_range(p.x, limit.mid_x_min, limit.mid_x_max) &&
           in_range(p.y, limit.mid_y_min, limit.mid_y_max);
}

// 辅助函数：检查点是否在上拐点ROI内
static bool in_up_roi(const cv::Point &p, const ElementState::Limit &limit) {
    if (p.x < 0 || p.y < 0) return false;
    if (!limit.enable_spatial_constraint) return true;
    return in_range(p.x, limit.up_x_min, limit.up_x_max) &&
           in_range(p.y, limit.up_y_min, limit.up_y_max);
}


/**
 * @brief 综合调用各处理函数，判断左右边界的拐点、直线段和丢线状态
 * @note  调用 filter_point 和 count_angle 处理边界数据  补充更新 is_straight 和 is_lost 状态
 * @param left  左边界数据结构体指针
 * @param right 右边界数据结构体指针
 */
void PointState::analyze_boundary(BoundaryData &data, bool is_left) {
    // 获取当前元素状态的拐点约束
    ElementState::Limit limit = element_state.get_corner_limit(is_left);
    
    // 初始化
    data.is_lost = false;
    data.lost_y = -1;
    data.lost_count = 0;

    // data.Lp_id = -1;

    data.down_lp_state = false;
    data.down_lp_pt = cv::Point(-1, -1);
    data.up_lp_state = false;
    data.up_lp_pt = cv::Point(-1, -1);
    data.mid_lp_state = false;
    data.mid_lp_pt = cv::Point(-1, -1);
    data.out_lp_state = false;
    data.out_lp_pt = cv::Point(-1, -1);

    // data.break_state = false;
    // data.break_pt = cv::Point(-1, -1);
    // data.is_weak_alone = false;
    data.is_straight = false;
    data.mnt_len = 0;
    data.dx_var = -1.0f;

    

    // 方向序列特征检测（八邻域，优先使用）
    if (data.dir_valid && data.dir_count >= 5) {
        // // 1. 丢线检测
        // if (is_left) {
        //     DP_Update_Left_Lost(data);
        // } 
        // else {
        //     DP_Update_Right_Lost(data);
        // }
        
        // 2. 单调性检测
        uint8_t mnt_len = 0;
        if (is_left) {
            mnt_len = DP_Update_Left_Mnt(data);
        } else {
            mnt_len = DP_Update_Right_Mnt(data);
        }
        data.mnt_len = mnt_len;
        // // 如果单调段长度超过阈值，认为是直线段
        // if (mnt_len >= ROI_HEIGHT * 0.75) { 
        //     data.is_straight = true;
        // }
        
        // 3. 拐点检测（使用宽松模式2，专门针对圆环）
        if (is_left) {
            // 左边界拐点检测
            cv::Point utl = Get_Left_UTL_Point(data, 2);  // 上转左 → 下拐点（使用mode=2）
            cv::Point rtu = Get_Left_RTU_Point(data, 2);  // 右转上 → 上拐点（使用mode=2）
            cv::Point utr = Get_Left_UTR_Point(data, 2);  // 上转右 → 中拐点（圆环）（使用mode=2）
            cv::Point arc = Get_L_Arc_Turn_Point(data);   // 圆弧拐点
            
            // 映射到 BoundaryData 字段
            if (in_down_roi(utl, limit)) {
                data.down_lp_state = true;
                data.down_lp_pt = utl;
            }
            if (in_up_roi(rtu, limit)) {
                data.up_lp_state = true;
                data.up_lp_pt = rtu;
            }
            // 中拐点先不直接生效：交由后续严格mid流程二次验证
            (void)utr;
            (void)arc;
            
            // 边线跳变检测作为补充验证
            // 如果方向序列检测没有找到拐点，尝试使用跳变检测
            if (!data.down_lp_state) {
                cv::Point jump_down = detect_left_down_jump(data, 3);
                if (in_down_roi(jump_down, limit)) {
                    data.down_lp_state = true;
                    data.down_lp_pt = jump_down;
                }
            }
            
            if (!data.up_lp_state) {
                cv::Point jump_up = detect_left_up_jump(data, 3);
                if (in_up_roi(jump_up, limit)) {
                    data.up_lp_state = true;
                    data.up_lp_pt = jump_up;
                }
            }
            
            // 单调性检测作为弧形拐点的补充
            if (!data.mid_lp_state) {
                cv::Point mono_arc = detect_left_arc_monotonicity(data, 3);
                if (in_mid_roi(mono_arc, limit)) {
                    data.mid_lp_state = true;
                    data.mid_lp_pt = mono_arc;
                }
            }
        } 
        else {
            // 右边界拐点检测
            cv::Point utr = Get_Right_UTR_Point(data, 1);  // 上转右 → 下拐点（使用mode=2）
            cv::Point ltu = Get_Right_LTU_Point(data, 1);  // 左转上 → 上拐点（使用mode=2）
            cv::Point utl = Get_Right_UTL_Point(data, 1);  // 上转左 → 中拐点（圆环）（使用mode=2）
            cv::Point arc = Get_R_Arc_Turn_Point(data);    // 圆弧拐点
            
            // 映射到 BoundaryData 字段
            if (in_down_roi(utr, limit)) {
                data.down_lp_state = true;
                data.down_lp_pt = utr;
            }
            if (in_up_roi(ltu, limit)) {
                data.up_lp_state = true;
                data.up_lp_pt = ltu;
            }
            // 中拐点先不直接生效：交由后续严格mid流程二次验证
            (void)utl;
            (void)arc;
            
            // 边线跳变检测作为补充验证
            // 如果方向序列检测没有找到拐点，尝试使用跳变检测
            if (!data.down_lp_state) {
                cv::Point jump_down = detect_right_down_jump(data, 3);
                if (in_down_roi(jump_down, limit)) {
                    data.down_lp_state = true;
                    data.down_lp_pt = jump_down;
                }
            }
            
            if (!data.up_lp_state) {
                cv::Point jump_up = detect_right_up_jump(data, 3);
                if (in_up_roi(jump_up, limit)) {
                    data.up_lp_state = true;
                    data.up_lp_pt = jump_up;
                }
            }
            
            // 单调性检测作为弧形拐点的补充
            if (!data.mid_lp_state) {
                cv::Point mono_arc = detect_right_arc_monotonicity(data, 3);
                if (in_mid_roi(mono_arc, limit)) {
                    data.mid_lp_state = true;
                    data.mid_lp_pt = mono_arc;
                }
            }
        }
    }
    // 统一主流程：dx 检测每帧执行（不再仅在方向序列无效时作为 fallback）
    {
    // dx 突变法主流程
    // 丢线检测（只统计有效底部区域的丢线）
    int continuous_lost = 0;        // 连续丢线行数
    int max_continuous_lost = 0;    // 最大连续丢线行数
    int first_lost_y = -1;          // 连续丢线的起始y (最靠近底部的那行)
    int temp_lost_start = -1;       // 当前连续丢线段的起始y

    for (int y = VALID_BOTTOM_ROW; y >= VALID_END_ROW; y--) {
        if (data.line[y] == -1) {
            data.lost_count++;
            if (continuous_lost == 0) {
                temp_lost_start = y;    // 记录这段丢线的起始行
            }
            continuous_lost++;
            if (continuous_lost > max_continuous_lost) {
                max_continuous_lost = continuous_lost;
                first_lost_y = temp_lost_start;
            }
        } 
        else {
            continuous_lost = 0;
        }
    }

    // 连续丢线超过5行，标记为丢线状态
    if (max_continuous_lost >= 5) {
        data.is_lost = true;
        data.lost_y = first_lost_y;
    }


    // 收集有效行数据
    struct row_data {int y; int x;};    // 有效行的 (y, x) 对，从底部往上排列
    row_data rows[CAM_HEIGHT];
    int row_count = 0;                  // 有效行数

    // 应用搜索范围约束 limit类
    // int search_start = limit.enable_global_search ? limit.search_y_min : VALID_END_ROW;
    // int search_end = limit.search_y_max;

    // for (int y = search_end; y >= search_start; y--) {
    for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
        if (data.line[y] != -1) {
            rows[row_count++] = {y, data.line[y]};
        }
    }

    // 最小有效行数不足，直接返回
    if (row_count <= 6) return;

    // 拐点检测参数（简化版本）
    bool straight_hint = data.is_straight && (data.lost_count <= 4);    // 直线提示：方向序列判断为直线且丢线不严重（最多4行）
    int mid_need_rows = 6;              // 中拐点候选点上下各需要至少11行有效数据（共23行窗口）才能判断（包含候选点所在行）
    int mid_win = 2;                     // 中拐点候选点两侧各需2行有效数据（共5行窗口）
    int mid_tol = 0;                     // 中拐点候选点允许误差（0.5%）
    int mid_prominence = 2;              // 中拐点候选点需要比两侧点至少高出2行（0.5%）才能判断为拐点
    int near_down_ban = 2;               // 下拐点候选点与底部丢线起始行之间至少需要6行有效数据才能判断为下拐点（防止丢线干扰）
    int down_lost_need = 2;              // 下拐点候选点与丢线起始行之间至少需要2行有效数据才能判断为下拐点
    int up_lost_need = 2;                // 上拐点候选点与丢线起始行之间至少需要2行有效数据才能判断为上拐点
    const int hard_missing_rows = 6;     // 强判据窗口行数
    const int hard_noise_allow = 3;      // 强判据允许噪点数
    const int mid_window_up = 5;         // 中拐点窗口：上方5行
    const int mid_window_down = 6;       // 中拐点窗口：下方6行，共12行
    const int mid_window_miss_max = 4;   // 12行窗口内最多允许缺失2行
    const int seg_y_gap_max = 2;         // 最长线段：Y间隔阈值
    const int seg_x_diff_max = 3;        // 最长线段：X差阈值
    const int seg_min_len = 5;           // 最长线段最小长度阈值

    // 连续丢线强判据（容噪）：候选点上方/下方N行内，非丢线点数不能超过noise_allow
    auto has_continuous_missing_above = [&](int center_y, int need_rows, int noise_allow) {
        if (center_y - need_rows < VALID_END_ROW) return false;
        int noise = 0;
        for (int dy = 1; dy <= need_rows; dy++) {
            int y = center_y - dy;
            if (y < VALID_END_ROW || y < 0) return false;
            if (data.scan_line[y] != -1) {
                noise++;
                if (noise > noise_allow) return false;
            }
        }
        return true;
    };
    auto has_continuous_missing_below = [&](int center_y, int need_rows, int noise_allow) {
        if (center_y + need_rows > VALID_START_ROW) return false;
        int noise = 0;
        for (int dy = 1; dy <= need_rows; dy++) {
            int y = center_y + dy;
            if (y > VALID_START_ROW || y >= CAM_HEIGHT) return false;
            if (data.scan_line[y] != -1) {
                noise++;
                if (noise > noise_allow) return false;
            }
        }
        return true;
    };

    auto down_candidate_ok = [&](const cv::Point &p) {
        if (p.x < 0 || p.y < 0) return false;
        if (limit.enable_spatial_constraint) return in_down_roi(p, limit);

        if (p.y < CAM_HEIGHT / 2) return false;
        int mid_x = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
        if (is_left && p.x >= mid_x) return false;
        if (!is_left && p.x <= mid_x) return false;
        return true;
    };

    auto up_candidate_ok = [&](const cv::Point &p) {
        if (p.x < 0 || p.y < 0) return false;
        if (limit.enable_spatial_constraint) return in_up_roi(p, limit);

        int mid_x = (VALID_LEFT_COL + VALID_RIGHT_COL) / 2;
        if (is_left && p.x >= mid_x) return false;
        if (!is_left && p.x <= mid_x) return false;
        return true;
    };

    // 从丢线点向下搜索连续线段，返回第一个连续段的起点（最靠近丢线点）
    auto find_continuous_segment_from_lost_down = [&](int lost_y, cv::Point &result_pt) {
        result_pt = cv::Point(-1, -1);
        if (!data.is_lost || lost_y < 0) return false;
        
        const int min_continuous = 3;  // 最少连续5行
        int continuous_count = 0;
        int segment_start_y = -1;
        
        // 从丢线点向下扫描
        for (int y = lost_y; y <= VALID_START_ROW; y++) {
            if (data.line[y] != -1) {
                if (continuous_count == 0) {
                    segment_start_y = y;  // 记录连续段起点
                }
                continuous_count++;
                
                // 找到足够长的连续段，立即返回起点
                if (continuous_count >= min_continuous) {
                    result_pt = cv::Point(data.line[segment_start_y], segment_start_y);
                    return true;
                }
            } else {
                // 遇到丢线，重置计数
                continuous_count = 0;
                segment_start_y = -1;
            }
        }
        
        return false;
    };
    
    // 从丢线点向上搜索连续线段，返回第一个连续段的起点（最靠近丢线点）
    auto find_continuous_segment_from_lost_up = [&](int lost_y, cv::Point &result_pt) {
        result_pt = cv::Point(-1, -1);
        if (!data.is_lost || lost_y < 0) return false;
        
        const int min_continuous = 3;  // 最少连续5行
        int continuous_count = 0;
        int segment_start_y = -1;
        
        // 从丢线点向上扫描
        for (int y = lost_y; y >= VALID_END_ROW; y--) {
            if (data.line[y] != -1) {
                if (continuous_count == 0) {
                    segment_start_y = y;  // 记录连续段起点
                }
                continuous_count++;
                
                // 找到足够长的连续段，立即返回起点
                if (continuous_count >= min_continuous) {
                    result_pt = cv::Point(data.line[segment_start_y], segment_start_y);
                    return true;
                }
            } else {
                // 遇到丢线，重置计数
                continuous_count = 0;
                segment_start_y = -1;
            }
        }
        
        return false;
    };

    // 中拐点窗口稳定性：候选点所在12行窗口内缺失不超过阈值
    auto mid_window_stable = [&](const int *src_line, int center_y) {
        int y0 = center_y - mid_window_up;
        int y1 = center_y + mid_window_down;
        if (y0 < VALID_END_ROW || y1 > VALID_START_ROW) return false;

        int miss = 0;
        for (int y = y0; y <= y1; y++) {
            if (y < 0 || y >= CAM_HEIGHT) {
                miss++;
                continue;
            }
            if (src_line[y] == -1) miss++;
        }
        return miss <= mid_window_miss_max;
    };

    // 坐标突变　　　计算 dx 即相邻两行的x变化量
    int dx[CAM_HEIGHT];
    for (int i = 0; i < row_count - 1; i++) {
        dx[i] = rows[i + 1].x - rows[i].x;
    }
    int dx_count = row_count - 1;

    // 候选点上方外扩连续性验证：抑制单点抖动触发
    auto outward_follow_ok = [&](int idx) {
        int look = 3;
        int need = 2;
        int ok = 0;
        int samples = 0;
        for (int k = idx + 1; k <= idx + look && k < dx_count; k++) {
            samples++;
            if (is_left) {
                if (dx[k] <= 0) ok++;
            } 
            else {
                if (dx[k] >= 0) ok++;
            }
        }
        if (samples == 0) return true;
        return ok >= need;
    };

    // 下拐点检测（三层判据：硬判据 -> 丢线点连续段搜索 -> dx兜底）
    
    // 应用空间约束：检查是否需要检测下拐点
    if (limit.enable_spatial_constraint && !limit.need_down_lp) {
        // 空间约束启用且不需要下拐点，跳过检测
        data.down_lp_state = false;
        data.down_lp_pt = cv::Point(-1, -1);
    } 
    else {
        // 第一层：硬判据（容噪丢线检测）
        if (!data.down_lp_state) {
            for (int i = 0; i < dx_count; i++) {
                cv::Point cand(rows[i].x, rows[i].y);
                bool hard_down_hit = has_continuous_missing_above(cand.y, hard_missing_rows, hard_noise_allow);
                if (hard_down_hit && down_candidate_ok(cand)) {
                    data.down_lp_state = true;
                    data.down_lp_pt = cand;
                    break;
                }
            }
        }

        // 第二层：从丢线点向下搜索连续线段（依赖 is_lost 状态）
        if (!data.down_lp_state && data.is_lost) {
            cv::Point seg_pt;
            if (find_continuous_segment_from_lost_down(data.lost_y, seg_pt) && down_candidate_ok(seg_pt)) {
                data.down_lp_state = true;
                data.down_lp_pt = seg_pt;
            }
        }

        // 第三层：dx 突变兜底（最后保底机制）
        if (!data.down_lp_state) {
            float average_dx = 0.0f;
            int average_count = 0;
            float dx_thresh = DX_THRESH * limit.dx_threshold_scale;

            for (int i = 0; i < dx_count; i++) {

                // 应用空间约束：Y坐标范围限制
                if (limit.enable_spatial_constraint) {
                    if (rows[i].y < limit.down_y_min || rows[i].y > limit.down_y_max) {
                        // 滑动窗口仍需更新
                        average_dx += (float)dx[i];
                        average_count++;
                        if (average_count > SMOOTH_WIN * 2) {
                            average_dx *= 0.5f;
                            average_count /= 2;
                        }
                        continue;
                    }
                } 
                else {
                    // 未启用空间约束时，保持原有逻辑：下拐点从底部找到图像中间即可
                    if (rows[i].y < CAM_HEIGHT / 2) {
                        break;
                    }
                }

                // 累积平均dx
                if (average_count >= SMOOTH_WIN) {
                    float avg = average_dx / average_count;
                    float diff = (float)dx[i] - avg;

                    // 左线：突然向左外扩 即 dx大幅变负 　 右线：突然向右外扩 即 dx大幅变正 
                    bool dx_trigger = is_left ? (diff < -dx_thresh) : (diff > dx_thresh);

                    // 空间约束模式：放宽条件为"或"关系
                    if (limit.enable_spatial_constraint) {
                    // 下拐点丢线验证（上方丢线）
                    int lost_above = 0;
                    for (int dy = 1; dy <= 5; dy++) {
                        int check_y = rows[i].y - dy;
                        if (check_y < VALID_END_ROW || check_y < 0) break;
                        if (data.scan_line[check_y] == -1) lost_above++;
                    }
                        bool lost_above_ok = (lost_above >= down_lost_need);
                        bool follow_ok = outward_follow_ok(i);
                    
                        // 下拐点软条件（常规判据）
                        bool down_cond = dx_trigger && lost_above_ok && follow_ok;
                        if (down_cond) {
                            int pt_x = rows[i].x;        
                            int pt_y = rows[i].y;
                        
                            // 应用空间约束：X坐标范围限制
                            if (pt_x >= limit.down_x_min && pt_x <= limit.down_x_max) {
                                // 找到有效下拐点
                                data.down_lp_state = true;
                                data.down_lp_pt = cv::Point(pt_x, pt_y);
                                break;
                            }
                        }
                    } 
                    else {
                        // 未启用空间约束：保持原有软条件逻辑
                        if (dx_trigger) {
                            bool follow_ok = outward_follow_ok(i);
                            if (!follow_ok) continue;

                            // 曲率验证　　　比较当前dx和前面2行的dx，变化量要够大
                            bool curv_ok = false;
                            if (i >= 2) {
                                int ddx = std::abs(dx[i] - dx[i - 2]);
                                curv_ok = (ddx >= 2);
                            } 
                            else {
                                curv_ok = true; // 数据不够时信任dx突变
                            }

                            // 下拐点丢线验证
                            if (curv_ok) {
                            // 注: 拐点上方必须有 scan_line 丢线，否则只是弯道大曲率，不是十字拐点
                            //     从候选点往上看 10 行，至少 3 行 scan_line == -1
                                int lost_above = 0;
                                for (int dy = 1; dy <= 5; dy++) {
                                    int check_y = rows[i].y - dy;
                                    if (check_y < VALID_END_ROW || check_y < 0) break;
                                    if (data.scan_line[check_y] == -1) lost_above++;
                                }
                                if (lost_above < down_lost_need) continue; // 上方没丢线，跳过这个候选点

                                // 下拐点横坐标限制　左拐点在左　右在右
                                int pt_x = rows[i].x;        
                                int pt_y = rows[i].y;
                            
                                if (is_left && pt_x >= (VALID_LEFT_COL + VALID_RIGHT_COL) / 2) continue;
                                if (!is_left && pt_x <= (VALID_LEFT_COL + VALID_RIGHT_COL) / 2) continue;

                                // 找到有效下拐点
                                data.down_lp_state = true;
                                data.down_lp_pt = cv::Point(pt_x, pt_y);
                                break;
                            }
                        }
                    }
                }

                // 滑动窗口更新　 (仿队列)
                average_dx += (float)dx[i];
                average_count++;
                if (average_count > SMOOTH_WIN * 2) {
                    average_dx *= 0.5f;
                    average_count /= 2;
                }
            }
        }
    }

    // 中拐点检测 (圆环边局部极值点)
    // 先在 line[] 上找，找不到再在 scan_line[] 上找
    
    // 应用空间约束：检查是否需要检测中拐点
    if (limit.enable_spatial_constraint && !limit.need_mid_lp) {
        // 空间约束启用且不需要中拐点，跳过检测
        data.mid_lp_state = false;
        data.mid_lp_pt = cv::Point(-1, -1);
    } 
    else if (!straight_hint && row_count >= mid_need_rows) {
        int best_mid_idx = -1;
        int pole_val = is_left ? -1 : CAM_WIDTH;
        
        // 避开底部和顶部，在中间找凸起
        for (int i = mid_win; i < row_count - mid_win; i++) {
            bool is_pole = true;
            int px = rows[i].x;
            int py = rows[i].y;
            
            // 应用空间约束：Y坐标范围限制
            if (limit.enable_spatial_constraint) {
                if (py < limit.mid_y_min || py > limit.mid_y_max) {
                    continue;
                }
            }

            // 如果下拐点已找到，中拐点搜索时排除其邻域
            if (data.down_lp_state && std::abs(py - data.down_lp_pt.y) <= near_down_ban) {
                continue;
            }

            // 中拐点12行窗口稳定性约束（line[] 主检测）
            if (!mid_window_stable(data.line, py)) {
                continue;
            }

            // 左线找局部极大值 (相邻两个点，放宽条件)
            if (is_left) {
                for (int j = 1; j <= mid_win; j++) {
                    if (rows[i-j].x > px + mid_tol || rows[i+j].x > px + mid_tol) {
                        is_pole = false;
                        break;
                    }
                }
                int neigh_max = std::max(rows[i-1].x, rows[i+1].x);
                bool prominence_ok = (px - neigh_max) >= mid_prominence;
                if (is_pole && prominence_ok && px > pole_val) {
                    pole_val = px;
                    best_mid_idx = i;
                }
            } 

            // 右线找局部极小值 (相邻两个点，放宽条件)
            else {
                for (int j = 1; j <= mid_win; j++) {
                    if (rows[i-j].x < px - mid_tol || rows[i+j].x < px - mid_tol) {
                        is_pole = false;
                        break;
                    }
                }
                int neigh_min = std::min(rows[i-1].x, rows[i+1].x);
                bool prominence_ok = (neigh_min - px) >= mid_prominence;
                if (is_pole && prominence_ok && px < pole_val) {
                    pole_val = px;
                    best_mid_idx = i;
                }
            }
        }
        
        // 连续性验证已注释：只用局部极值判断就够了
        if (best_mid_idx != -1) {
            int mid_y = rows[best_mid_idx].y;
            int mid_x = rows[best_mid_idx].x;
            // bool continuous = true;
            // for (int dy = -1; dy <= 1; dy++) {
            //     int check_y = mid_y + dy;
            //     if (check_y < VALID_END_ROW || check_y > VALID_START_ROW) {
            //         continuous = false; break;
            //     }
            //     if (data.line[check_y] == -1) {
            //         continuous = false;
            //         break;
            //     }
            // }
            
            // 应用空间约束：X坐标范围限制
            // if (continuous) {
                if (limit.enable_spatial_constraint) {
                    if (mid_x >= limit.mid_x_min && mid_x <= limit.mid_x_max) {
                        data.mid_lp_state = true;
                        data.mid_lp_pt = cv::Point(mid_x, mid_y);
                    }
                } else {
                    data.mid_lp_state = true;
                    data.mid_lp_pt = cv::Point(mid_x, mid_y);
                }
            // }
        }
    }

    // 中拐点备选检测：如果 line[] 上没找到，用 scan_line[] 再找一次
    if (!data.mid_lp_state && !straight_hint) {
        // 收集 scan_line 有效行
        struct scan_row_data { int y; int x; };
        scan_row_data scan_rows[CAM_HEIGHT];
        int scan_count = 0;
        for (int y = VALID_START_ROW; y >= VALID_END_ROW; y--) {
            if (data.scan_line[y] != -1) {
                scan_rows[scan_count++] = {y, data.scan_line[y]};
            }
        }

    if (scan_count >= 10) {
            int best_scan_idx = -1;
            int scan_pole = is_left ? -1 : CAM_WIDTH;
        int sw = 2;

            for (int i = sw; i < scan_count - sw; i++) {
                bool is_pole = true;
                int px = scan_rows[i].x;

                if (is_left) {
                    for (int j = 1; j <= sw; j++) {
                        if (scan_rows[i-j].x > px ||
                            scan_rows[i+j].x > px) {
                            is_pole = false; break;
                        }
                    }
                    int neigh_max = std::max(scan_rows[i-1].x, scan_rows[i+1].x);
                    bool prominence_ok = (px - neigh_max) >= mid_prominence;
                    if (is_pole && prominence_ok && px > scan_pole) {
                        scan_pole = px;
                        best_scan_idx = i;
                    }
                } 
                else {
                    for (int j = 1; j <= sw; j++) {
                        if (scan_rows[i-j].x < px ||
                            scan_rows[i+j].x < px) {
                            is_pole = false; break;
                        }
                    }
                    int neigh_min = std::min(scan_rows[i-1].x, scan_rows[i+1].x);
                    bool prominence_ok = (neigh_min - px) >= mid_prominence;
                    if (is_pole && prominence_ok && px < scan_pole) {
                        scan_pole = px;
                        best_scan_idx = i;
                    }
                }
            }

            // 12行窗口稳定性验证（scan_line 备选检测）
            if (best_scan_idx != -1) {
                int mid_y = scan_rows[best_scan_idx].y;
                if (mid_window_stable(data.scan_line, mid_y)) {
                    data.mid_lp_state = true;
                    data.mid_lp_pt = cv::Point(scan_rows[best_scan_idx].x, scan_rows[best_scan_idx].y);
                }
            }
        }
    }

    // // 十字类上拐点检测 (自上而下双向寻线)
    // {
    //     // 寻找顶部连续线段
    //     struct top_row_data {int y; int x;};
    //     top_row_data top_rows[CAM_HEIGHT];
    //     int top_row_count = 0;
        
    //     int current_run = 0;
    //     int best_start_y = -1;
    //     int best_end_y = -1;
    //     int temp_start_y = -1;

    //     // 限制　(上拐点从上往下找，最多找到图像中间)  十字内部的上拐点会跑到下面　故不再限制
    //     int up_end_y = std::min((int)VALID_START_ROW, CAM_HEIGHT);
    //     for (int y = VALID_END_ROW; y <= up_end_y; y++) {
    //         if (data.scan_line[y] != -1) {
    //             if (current_run == 0) temp_start_y = y;
    //             current_run++;
    //         }

    //         else {                                   
    //             if (current_run >= 3) {     // 找到了足够长的线段(>=5行)
    //                 best_start_y = temp_start_y;
    //                 best_end_y = y - 1;
    //                 break;
    //             }
    //             current_run = 0; // 不够长 清空往下重新找
    //         }
    //     }

    //     // 如果到底部都没断开，也记录下来
    //     if (best_start_y == -1 && current_run >= 3) {
    //         best_start_y = temp_start_y;
    //         best_end_y = VALID_START_ROW;
    //     }

    //     // 分析找到的线段特征
    //     if (best_start_y != -1) {
    //         for (int y = best_start_y; y <= best_end_y; y++) {
    //             top_rows[top_row_count++] = {y, data.scan_line[y]};
    //         }

    //         // 检测dx突变　(与下拐点逻辑类似)
    //         int top_dx[CAM_HEIGHT];
    //         for (int i = 0; i < top_row_count - 1; i++) {
    //             top_dx[i] = top_rows[i + 1].x - top_rows[i].x;
    //         }
    //         int top_dx_count = top_row_count - 1;

    //         float average_dx_top = 0.0f;
    //         int average_count_top = 0;

    //         for (int i = 0; i < top_dx_count; i++) {
    //             if (average_count_top >= SMOOTH_WIN) {
    //                 float avg_dx = average_dx_top / average_count_top;
    //                 float diff = (float)top_dx[i] - avg_dx;

    //                 // 上角点 (此时的视角为从上到下)
    //                 bool dx_trigger = is_left ? (diff < -DX_THRESH) : (diff > DX_THRESH);

    //                 // 丢线验证：突变点下方必须有扫线大缺口 (十字 此处保留　　十字上拐点很重要　故检测条件较为严格)
    //                 if (dx_trigger) {
    //                     int pt_y = top_rows[i].y;
                        
    //                     // 如果中拐点已找到，上拐点搜索时排除中拐点±8行区域
    //                     if (data.mid_lp_state && std::abs(pt_y - data.mid_lp_pt.y) <= 5) {
    //                         continue;
    //                     }
                        
    //                     // 如果下拐点已找到，上拐点搜索时排除下拐点±12行区域
    //                     if (data.down_lp_state && std::abs(pt_y - data.down_lp_pt.y) <= 8) {
    //                         continue;
    //                     }
                        
    //                     int lost_below = 0;
    //                     for (int dy = 1; dy <= 10; dy++) {
    //                         int check_y = top_rows[i].y + dy;
    //                         if (check_y > VALID_START_ROW)  break;
    //                         if (data.scan_line[check_y] == -1) lost_below++;
    //                     }

    //                     // 防出界误判 (左右)
    //                     if (lost_below >= 2) {
    //                         int pt_x = top_rows[i].x;
    //                         if (pt_x >= 5 && pt_x <= CAM_WIDTH - 6) {
    //                             data.up_lp_state = true;
    //                             data.up_lp_pt = cv::Point(pt_x, top_rows[i].y);
    //                             break;
    //                         }
    //                     }
    //                 }
    //             }
                
    //             // 滑动窗口更新　 (仿队列)
    //             average_dx_top += (float)top_dx[i];
    //             average_count_top++;
    //             if (average_count_top > SMOOTH_WIN * 2) {
    //                 average_dx_top *= 0.5f;
    //                 average_count_top /= 2;
    //             }
    //         }

    //         // 补充　当无突变时，自然断点为上拐点　　(连续线段的最下端断点)
    //         if (!data.up_lp_state && best_end_y <= VALID_START_ROW - 8) {
    //             // 如果中拐点已找到，排除中拐点±15行区域
    //             bool too_close_to_mid = data.mid_lp_state && std::abs(best_end_y - data.mid_lp_pt.y) <= 5;
    //             // 如果下拐点已找到，排除下拐点±15行区域
    //             bool too_close_to_down = data.down_lp_state && std::abs(best_end_y - data.down_lp_pt.y) <= 8;
                
    //             if (!too_close_to_mid && !too_close_to_down) {
    //                 int lost_below = 0;
    //                 for (int dy = 1; dy <= 8; dy++) {
    //                     int check_y = best_end_y + dy;
    //                     if (check_y <= VALID_START_ROW && data.scan_line[check_y] == -1) {
    //                         lost_below++;
    //                     }
    //                 }

    //                 // 检查断点下方丢线情况
    //                 if (lost_below >= 2) {
    //                     int pt_x = data.scan_line[best_end_y];

    //                     // 防出界误判　(左右)
    //                     if (pt_x >= 5 && pt_x <= CAM_WIDTH - 6) {
    //                         data.up_lp_state = true;
    //                         data.up_lp_pt = cv::Point(pt_x, best_end_y);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // 上拐点检测（三层判据：硬判据 -> 丢线点连续段搜索 -> dx兜底）
    // 应用空间约束：检查是否需要检测上拐点
    if (limit.enable_spatial_constraint && !limit.need_up_lp) {
        // 空间约束启用且不需要上拐点，跳过检测
        data.up_lp_state = false;
        data.up_lp_pt = cv::Point(-1, -1);
    } else if (limit.enable_spatial_constraint) {
        // 启用空间约束：放宽检测条件
    int search_limit_idx = 0; // 默认最多扫到底部

        // 如果有中拐点，则只扫到中拐点上方；如果没有，就可以往下多扫一点
        if (data.mid_lp_state) {
            for (int i = 0; i < row_count; i++) {
                if (rows[i].y == data.mid_lp_pt.y) { 
                    // 允许继续向下搜索若干行，避免mid误检拖死up
                    search_limit_idx = std::max(0, i - 2); 
                    break; 
                }
            }
        }
        
        // 第一层：硬判据（容噪丢线检测）
        if (!data.up_lp_state) {
            for (int i = row_count - 1; i > search_limit_idx; i--) {
                int pt_y = rows[i].y;
                int pt_x = rows[i].x;
                bool hard_up_hit = has_continuous_missing_below(pt_y, hard_missing_rows, hard_noise_allow);
                if (!hard_up_hit) continue;

                if (data.mid_lp_state && std::abs(pt_y - data.mid_lp_pt.y) <= 2) continue;
                if (data.down_lp_state && std::abs(pt_y - data.down_lp_pt.y) <= 5) continue;

                cv::Point cand(pt_x, pt_y);
                if (up_candidate_ok(cand)) {
                    data.up_lp_state = true;
                    data.up_lp_pt = cand;
                    break;
                }
            }
        }

        // 第二层：从丢线点向上搜索连续线段（依赖 is_lost 状态）
        if (!data.up_lp_state && data.is_lost) {
            cv::Point seg_pt;
            if (find_continuous_segment_from_lost_up(data.lost_y, seg_pt)) {
                if (!(data.mid_lp_state && std::abs(seg_pt.y - data.mid_lp_pt.y) <= 2) &&
                    !(data.down_lp_state && std::abs(seg_pt.y - data.down_lp_pt.y) <= 5) &&
                    up_candidate_ok(seg_pt)) {
                    data.up_lp_state = true;
                    data.up_lp_pt = seg_pt;
                }
            }
        }

        // 第三层：从最上方有效点往下扫，寻找向外的突变点（dx兜底）
        if (!data.up_lp_state && row_count - search_limit_idx > 5) {
            float avg_dx_down = 0.0f;
            int count_down = 0;
            
            // 应用dx阈值缩放
            float dx_thresh = DX_THRESH * limit.dx_threshold_scale;

            int up_y_max_relaxed = std::min(VALID_START_ROW, limit.up_y_max + 8);
            for (int i = row_count - 1; i > search_limit_idx; i--) {
                int pt_y = rows[i].y;
                int pt_x = rows[i].x;
                
                // 应用空间约束：Y坐标范围限制
                if (pt_y < limit.up_y_min || pt_y > up_y_max_relaxed) {
                    // 滑动窗口仍需更新
                    int dx_down = rows[i - 1].x - rows[i].x;
                    avg_dx_down += dx_down;
                    count_down++;
                    if (count_down > SMOOTH_WIN * 2) {
                        avg_dx_down *= 0.5f;
                        count_down /= 2;
                    }
                    continue;
                }
                
                int dx_down = rows[i - 1].x - rows[i].x;

                if (count_down >= SMOOTH_WIN) {
                    float avg = avg_dx_down / count_down;
                    float diff = (float)dx_down - avg;
                    
                    // 检测向赛道外突变
                    bool dx_trigger = is_left ? (diff < -dx_thresh) : (diff > dx_thresh);
                    
                    // 上拐点软判据：下方 hard_missing_rows 行允许少量噪点
                    int lost_below = 0;
                    int check_range = hard_missing_rows;
                    int actual_range = 0;
                    for (int dy = 1; dy <= check_range; dy++) {
                        int check_y = pt_y + dy;
                        if (check_y > VALID_START_ROW) break;
                        actual_range++;
                        if (data.scan_line[check_y] == -1) lost_below++;
                    }
                    int noise_below = actual_range - lost_below;
                    bool lost_below_ok = (actual_range >= hard_missing_rows) && (noise_below <= hard_noise_allow);

                    int valid_above = 0;
                    for (int dy = 1; dy <= 4; dy++) {
                        int check_y = pt_y - dy;
                        if (check_y < VALID_END_ROW || check_y < 0) break;
                        if (data.scan_line[check_y] != -1) valid_above++;
                    }
                    bool head_break = (valid_above >= 2) && (lost_below >= up_lost_need);
                    bool up_cond = dx_trigger && (lost_below_ok || head_break);  // 软条件（常规判据）
                    
                    if (up_cond) {
                        // 排除中拐点±5行区域 (如果存在)
                        if (data.mid_lp_state && std::abs(pt_y - data.mid_lp_pt.y) <= 2) {
                            continue;
                        }
                        
                        // 排除下拐点±8行区域 (如果存在)
                        if (data.down_lp_state && std::abs(pt_y - data.down_lp_pt.y) <= 5) {
                            continue;
                        }
                        
                        // 应用空间约束：X坐标范围限制
                        if (pt_x >= limit.up_x_min && pt_x <= limit.up_x_max) {
                            data.up_lp_state = true;
                            data.up_lp_pt = cv::Point(pt_x, pt_y);
                            break;
                        }
                    }
                }

                // 更新滑动窗口
                avg_dx_down += dx_down;
                count_down++;
                if (count_down > SMOOTH_WIN * 2) {
                    avg_dx_down *= 0.5f;
                    count_down /= 2;
                }
            }
        }

        // 上拐点兜底：scan_line 顶部向下找首个短缺口断点（放宽）
        if (!data.up_lp_state) {
            int run = 0;
            int start_y = -1;
            int end_y = -1;
            int miss = 0;
            for (int y = limit.up_y_min; y <= std::min(VALID_START_ROW, limit.up_y_max + 6); y++) {
                if (data.scan_line[y] != -1) {
                    if (run == 0) start_y = y;
                    run++;
                    if (run >= 4) end_y = y;
                } else {
                    miss++;
                    if (miss > 1 && run >= 4) break;
                    if (miss > 1) { run = 0; start_y = -1; }
                }
            }
            if (start_y != -1 && end_y != -1) {
                int cand_y = end_y;
                if (data.mid_lp_state && cand_y >= data.mid_lp_pt.y - 2) {
                    cand_y = -1;
                }
                if (cand_y == -1) {
                    // do nothing
                } else {
                int cand_x = data.scan_line[cand_y];
                cv::Point cand(cand_x, cand_y);
                int lost_below = 0;
                for (int dy = 1; dy <= 6; dy++) {
                    int check_y = cand_y + dy;
                    if (check_y > VALID_START_ROW) break;
                    if (data.scan_line[check_y] == -1) lost_below++;
                }
                if (cand_x != -1 && in_up_roi(cand, limit) && lost_below >= up_lost_need) {
                    data.up_lp_state = true;
                    data.up_lp_pt = cand;
                }
                }
            }
        }
    } else if (element_state.circle_state == CircleState::approach && !data.up_lp_state) {
        // 未启用空间约束：仅在圆环 approach 状态时检测（保持原有逻辑）
        int search_limit_idx = 0; // 默认最多扫到底部

        // 如果有中拐点，则只扫到中拐点上方；如果没有，就可以往下多扫一点
        if (data.mid_lp_state) {
            for (int i = 0; i < row_count; i++) {
                if (rows[i].y == data.mid_lp_pt.y) { 
                    search_limit_idx = i; 
                    break; 
                }
            }
        }
        
        // 第一层：硬条件
        for (int i = row_count - 1; i > search_limit_idx && !data.up_lp_state; i--) {
            int pt_y = rows[i].y;
            int pt_x = rows[i].x;
            bool hard_up_hit = has_continuous_missing_below(pt_y, hard_missing_rows, hard_noise_allow);
            if (!hard_up_hit) continue;
            if (data.mid_lp_state && std::abs(pt_y - data.mid_lp_pt.y) <= 5) continue;
            if (data.down_lp_state && std::abs(pt_y - data.down_lp_pt.y) <= 8) continue;
            cv::Point cand(pt_x, pt_y);
            if (up_candidate_ok(cand)) {
                data.up_lp_state = true;
                data.up_lp_pt = cand;
            }
        }

        // 第二层：从丢线点向上搜索连续线段（依赖 is_lost 状态）
        if (!data.up_lp_state && data.is_lost) {
            cv::Point seg_pt;
            if (find_continuous_segment_from_lost_up(data.lost_y, seg_pt)) {
                if (!(data.mid_lp_state && std::abs(seg_pt.y - data.mid_lp_pt.y) <= 5) &&
                    !(data.down_lp_state && std::abs(seg_pt.y - data.down_lp_pt.y) <= 8) &&
                    up_candidate_ok(seg_pt)) {
                    data.up_lp_state = true;
                    data.up_lp_pt = seg_pt;
                }
            }
        }

        // 第三层：dx兜底
        if (!data.up_lp_state && row_count - search_limit_idx > 5) {
            float avg_dx_down = 0.0f;
            int count_down = 0;
            
            // 应用dx阈值缩放
            float dx_thresh = DX_THRESH * limit.dx_threshold_scale;

            for (int i = row_count - 1; i > search_limit_idx; i--) {
                int pt_y = rows[i].y;
                int pt_x = rows[i].x;
                
                int dx_down = rows[i - 1].x - rows[i].x;

                if (count_down >= SMOOTH_WIN) {
                    float avg = avg_dx_down / count_down;
                    float diff = (float)dx_down - avg;
                    
                    // 检测向赛道外突变
                    bool dx_trigger = is_left ? (diff < -dx_thresh) : (diff > dx_thresh);
                    if (dx_trigger) {
                        // 排除中拐点±5行区域 (如果存在)
                        if (data.mid_lp_state && std::abs(pt_y - data.mid_lp_pt.y) <= 5) {
                            continue;
                        }
                        
                        // 排除下拐点±8行区域 (如果存在)
                        if (data.down_lp_state && std::abs(pt_y - data.down_lp_pt.y) <= 8) {
                            continue;
                        }
                        
                        data.up_lp_state = true;
                        data.up_lp_pt = cv::Point(pt_x, pt_y); // 标记突变前一个点为上拐点
                        break;
                    }
                }

                // 更新滑动窗口
                avg_dx_down += dx_down;
                count_down++;
                if (count_down > SMOOTH_WIN * 2) {
                    avg_dx_down *= 0.5f;
                    count_down /= 2;
                }
            }
        }
    } 
    else if (!limit.enable_spatial_constraint) {
        // 未启用空间约束时，保持原有逻辑：非圆环 approach 状态，强制清零上拐点
        data.up_lp_state = false;
        data.up_lp_pt = cv::Point(-1, -1);
    }
        
    // 角点冲突处理：近距离合并 + 优先级（up > down > mid）
    auto clear_down = [&]() {
        data.down_lp_state = false;
        data.down_lp_pt = cv::Point(-1, -1);
    };
    auto clear_mid = [&]() {
        data.mid_lp_state = false;
        data.mid_lp_pt = cv::Point(-1, -1);
    };
    auto clear_up = [&]() {
        data.up_lp_state = false;
        data.up_lp_pt = cv::Point(-1, -1);
    };
    auto near_pt = [](const cv::Point &a, const cv::Point &b, int d) {
        return std::abs(a.x - b.x) <= d && std::abs(a.y - b.y) <= d;
    };

    // ROI外点清理，避免漂浮角点残留
    if (data.down_lp_state && !in_down_roi(data.down_lp_pt, limit)) clear_down();
    if (data.mid_lp_state && !in_mid_roi(data.mid_lp_pt, limit)) clear_mid();
    if (data.up_lp_state && !in_up_roi(data.up_lp_pt, limit)) clear_up();

    const int merge_dist = 5;  // 增大融合距离阈值，从6→8像素
    const int up_lower_min_gap = std::max(8, limit.down_up_min_gap);  // 增大上下角点最小间距，从3→15像素

    // lower融合：mid/down近点合一，统一投影到down，避免中下争点
    // 策略：当mid和down距离小于merge_dist时，取两者平均值作为down，清除mid
    if (data.mid_lp_state && data.down_lp_state && near_pt(data.mid_lp_pt, data.down_lp_pt, merge_dist)) {
        data.down_lp_pt.x = (data.down_lp_pt.x + data.mid_lp_pt.x) / 2;
        data.down_lp_pt.y = (data.down_lp_pt.y + data.mid_lp_pt.y) / 2;
        clear_mid();
    }

    // 若仅有mid且落在down ROI，提升为down，作为统一lower角点
    if (!data.down_lp_state && data.mid_lp_state && in_down_roi(data.mid_lp_pt, limit)) {
        data.down_lp_state = true;
        data.down_lp_pt = data.mid_lp_pt;
        clear_mid();
    }
    
    // 若仅有down且落在mid ROI（且不在down ROI），降级为mid
    if (data.down_lp_state && !data.mid_lp_state && !in_down_roi(data.down_lp_pt, limit) && in_mid_roi(data.down_lp_pt, limit)) {
        data.mid_lp_state = true;
        data.mid_lp_pt = data.down_lp_pt;
        clear_down();
    }

    if (data.up_lp_state && data.mid_lp_state && near_pt(data.up_lp_pt, data.mid_lp_pt, merge_dist)) {
        clear_mid();   // 重点修复：上拐点不再被中拐点占位
    }
    if (data.up_lp_state && data.down_lp_state && near_pt(data.up_lp_pt, data.down_lp_pt, merge_dist)) {
        clear_up();
    }
    if (data.mid_lp_state && data.down_lp_state && near_pt(data.mid_lp_pt, data.down_lp_pt, merge_dist)) {
        clear_mid();
    }

    // 纵向关系：down.y > mid.y > up.y，不满足时优先保留 up/down，压制 mid
    if (data.down_lp_state && data.mid_lp_state && data.down_lp_pt.y <= data.mid_lp_pt.y) {
        clear_mid();
    }
    if (data.mid_lp_state && data.up_lp_state && data.mid_lp_pt.y <= data.up_lp_pt.y) {
        clear_mid();
    }
    // 上拐点几何约束：up必须明显在lower（down或mid）上方
    // 优先检查down，其次检查mid
    cv::Point lower_pt = data.down_lp_state ? data.down_lp_pt : 
                         (data.mid_lp_state ? data.mid_lp_pt : cv::Point(-1, -1));
    
    if (data.up_lp_state && lower_pt.y >= 0) {
        // up.y必须 < lower.y - gap，否则清除up
        if (data.up_lp_pt.y >= lower_pt.y) {
            clear_up();
        } else if (data.up_lp_pt.y >= lower_pt.y - up_lower_min_gap) {
            clear_up();
        }
    }

    // 左右X方向几何限制（放宽且不连坐删除up）
    if (is_left) {
        if (data.down_lp_state && data.up_lp_state && data.up_lp_pt.x <= data.down_lp_pt.x) {
            clear_up();
        }
        if (data.mid_lp_state && data.down_lp_state && data.mid_lp_pt.x <= data.down_lp_pt.x) {
            clear_mid();
        }
        if (data.mid_lp_state && data.up_lp_state && data.mid_lp_pt.x >= data.up_lp_pt.x) {
            clear_mid();
        }
    } else {
        if (data.down_lp_state && data.up_lp_state && data.up_lp_pt.x >= data.down_lp_pt.x) {
            clear_up();
        }
        if (data.mid_lp_state && data.down_lp_state && data.mid_lp_pt.x >= data.down_lp_pt.x) {
            clear_mid();
        }
        if (data.mid_lp_state && data.up_lp_state && data.mid_lp_pt.x <= data.up_lp_pt.x) {
            clear_mid();
        }
    }

    // 中拐点边界保护
    if (data.mid_lp_state && (data.mid_lp_pt.x < VALID_LEFT_COL + 4 || data.mid_lp_pt.x > VALID_RIGHT_COL - 4)) {
        clear_mid();
    }

    // 对侧抑制逻辑已移除

    // 重叠点合并：mid/down近距离时保留down
    if (data.mid_lp_state && data.down_lp_state && near_pt(data.mid_lp_pt, data.down_lp_pt, merge_dist)) {
        clear_mid();
    }

    // 时序防抖：抑制拐点闪烁和大跳变（down/up/mid）
    auto temporal_stabilize = [&](bool &state,
                                  cv::Point &pt,
                                  cv::Point &prev_pt,
                                  int &stable_count,
                                  int confirm_need,
                                  int jump_x,
                                  int jump_y) {
        auto valid_pt = [](const cv::Point &p) { return p.x >= 0 && p.y >= 0; };

        if (state && valid_pt(pt)) {
            if (valid_pt(prev_pt)) {
                int dx_jump = std::abs(pt.x - prev_pt.x);
                int dy_jump = std::abs(pt.y - prev_pt.y);
                bool jump = (dx_jump > jump_x) || (dy_jump > jump_y);

                if (jump && stable_count >= confirm_need) {
                    // 已稳定后出现大跳，优先信任历史点
                    pt = prev_pt;
                } else if (jump) {
                    // 未稳定时跳变，用历史点做平滑过渡
                    pt.x = (prev_pt.x * 3 + pt.x) / 4;
                    pt.y = (prev_pt.y * 3 + pt.y) / 4;
                    stable_count = std::max(1, stable_count);
                } else {
                    stable_count = std::min(stable_count + 1, 10);
                }
            } else {
                stable_count = 1;
            }

            prev_pt = pt;
            state = true;
            return;
        }

        // 当前帧未检出，短时保持上一帧稳定点，减少闪烁
        if (valid_pt(prev_pt) && stable_count >= confirm_need) {
            state = true;
            pt = prev_pt;
            stable_count = std::max(confirm_need - 1, stable_count - 1);
        } else {
            state = false;
            pt = cv::Point(-1, -1);
            stable_count = 0;
        }
    };

    // 优先级1：上拐点时序保护改为3帧
    // 优先级2：下拐点和中拐点保持2帧保护
    temporal_stabilize(data.down_lp_state, data.down_lp_pt, data.prev_down_lp_pt,
                      data.down_stable_count, 2, 3, 3);
    temporal_stabilize(data.up_lp_state, data.up_lp_pt, data.prev_up_lp_pt,
                      data.up_stable_count, 3, 3, 4);
    temporal_stabilize(data.mid_lp_state, data.mid_lp_pt, data.prev_mid_lp_pt,
                      data.mid_stable_count, 2, 3, 4);

    // // 应用拐点约束配置
    // // 强制下拐点在上拐点下方
    // if (limit.force_down_below_up && data.down_lp_state && data.up_lp_state) {
    //     if (data.down_lp_pt.y < data.up_lp_pt.y) {
    //         // 位置反转，根据稳定计数决定保留哪个
    //         if (data.down_stable_count > data.up_stable_count) {
    //             data.up_lp_state = false;  // 丢弃上拐点
    //             data.up_lp_pt = cv::Point(-1, -1);
    //         } else {
    //             data.down_lp_state = false;  // 丢弃下拐点
    //             data.down_lp_pt = cv::Point(-1, -1);
    //         }
    //     }
    // }

    // 禁止旧状态残留；状态为 true 但坐标无效时也强制清掉，避免 YES(-1) 误导调试和元素判断
    if (data.down_lp_state && (data.down_lp_pt.x < 0 || data.down_lp_pt.y < 0)) {
        data.down_lp_state = false;
    }
    if (data.mid_lp_state && (data.mid_lp_pt.x < 0 || data.mid_lp_pt.y < 0)) {
        data.mid_lp_state = false;
    }
    if (data.up_lp_state && (data.up_lp_pt.x < 0 || data.up_lp_pt.y < 0)) {
        data.up_lp_state = false;
    }
    if (!data.down_lp_state) data.down_lp_pt = cv::Point(-1, -1);
    if (!data.mid_lp_state) data.mid_lp_pt = cv::Point(-1, -1);
    if (!data.up_lp_state) data.up_lp_pt = cv::Point(-1, -1);
    
    // // 优先级3：直道场景下的拐点抑制（直道不应该有mid/up）
    // // 判断是否为直道：单调段长度足够 且 方差小 且 丢线少
    // bool is_straight_road = data.is_straight;
    
    // if (is_straight_road) {
    //     // 直道场景：清除中拐点和上拐点
    //     data.mid_lp_state = false;
    //     data.mid_lp_pt = cv::Point(-1, -1);
    //     data.up_lp_state = false;
    //     data.up_lp_pt = cv::Point(-1, -1);
    // }
    
    // // 中拐点必须在下拐点和上拐点之间 (down.y > mid.y > up.y)
    // if (data.mid_lp_state) {
    //     bool mid_ok = false;
        
    //     // 中拐点不能在下拐点下方
    //     if (data.down_lp_state && data.mid_lp_pt.y >= data.down_lp_pt.y) {
    //         mid_ok = true;
    //     }
        
    //     // 中拐点不能在上拐点上方
    //     if (data.up_lp_state && data.mid_lp_pt.y <= data.up_lp_pt.y) {
    //         mid_ok = true;
    //     }
        
    //     if (mid_ok) {
    //         data.mid_lp_state = false;
    //         data.mid_lp_pt = cv::Point(-1, -1);
    //     }
    // }
    
    // // 上下拐点最小间距约束
    // if (data.down_lp_state && data.up_lp_state) {
    //     int gap = std::abs(data.down_lp_pt.y - data.up_lp_pt.y);
    //     if (gap < limit.down_up_min_gap) {
    //         // 间距过小，保留稳定计数高的
    //         if (data.down_stable_count > data.up_stable_count) {
    //             data.up_lp_state = false;
    //             data.up_lp_pt = cv::Point(-1, -1);
    //         } else {
    //             data.down_lp_state = false;
    //             data.down_lp_pt = cv::Point(-1, -1);
    //         }
    //     }
    // }
    
    // // 应用锁定约束（带超时保护）
    // if (limit.lock_down_pt && data.prev_down_lp_pt.x > 0) {
    //     data.down_lock_frames++;
        
    //     // 超时保护：锁定超过最大帧数，强制解除
    //     if (data.down_lock_frames <= limit.max_lock_frames) {
    //         data.down_lp_pt = data.prev_down_lp_pt;
    //         data.down_lp_state = true;
    //     }
    //     else {
    //         // 超时，解除锁定，重新检测
    //         data.down_lock_frames = 0;
    //     }
    // }
    // else {
    //     data.down_lock_frames = 0;  // 不锁定时重置计数
    // }
    
    // if (limit.lock_up_pt && data.prev_up_lp_pt.x > 0) {
    //     data.up_lock_frames++;
        
    //     // 超时保护：锁定超过最大帧数，强制解除
    //     if (data.up_lock_frames <= limit.max_lock_frames) {
    //         data.up_lp_pt = data.prev_up_lp_pt;
    //         data.up_lp_state = true;
    //     }
    //     else {
    //         // 超时，解除锁定，重新检测
    //         data.up_lock_frames = 0;
    //     }
    // }
    // else {
    //     data.up_lock_frames = 0;  // 不锁定时重置计数
    // }


    // // 出环点检测 (圆环)
    // float out_avg_dx = 0.0f;
    // int out_avg_cnt = 0;

    // // 遍历所有有效行的dx，寻找dx突变点
    // for (int i = 0; i < dx_count; i++) {
    //     if (out_avg_cnt >= SMOOTH_WIN) {
    //         float avg = out_avg_dx / out_avg_cnt;
    //         float diff = (float)dx[i] - avg;
            
    //         // 出环时外扩剧烈，用大一些的阈值检测
    //         bool exit_trigger = is_left ? (diff < -DX_THRESH * 1.5f) : (diff > DX_THRESH * 1.5f);
                            
    //         // 丢线检测
    //         if (exit_trigger) {
    //             int lost_above = 0;
    //             for (int dy = 1; dy <= 8; dy++) {
    //                 int check_y = rows[i].y - dy;
    //                 if (check_y < VALID_END_ROW || check_y < 0) break;
    //                 if (data.scan_line[check_y] == -1) lost_above++;
    //             }

    //             // 伴随丢线则确认为出环点
    //             if (lost_above >= 3) {
    //                 int out_x = rows[i].x;
    //                 // 出环点不能太靠边界
    //                 if (out_x >= 5 && out_x <= CAM_WIDTH - 6) {
    //                     data.out_lp_state = true;
    //                     data.out_lp_pt = cv::Point(out_x, rows[i].y);
    //                     break;
    //                 }
    //             }
    //         }
    //     }
        
    //     // 更新滑动窗口
    //     out_avg_dx += dx[i];
    //     out_avg_cnt++;
    //     if (out_avg_cnt > SMOOTH_WIN * 2) {
    //         out_avg_dx *= 0.5f;
    //         out_avg_cnt /= 2;
    //     }
    // }

    // 统一直道判定逻辑 
    bool straight_by_mnt = false;   // 单调性判定
    bool straight_by_var = false;   // 方差判定
    
    // 1. 八邻域单调性检测（优先级高）
    if (data.mnt_len >= ROI_HEIGHT * 0.60) {
        straight_by_mnt = true;
    }
    
    // 2. dx方差检测
    if (row_count >= 10) {
        // 取底部15行有效数据的dx方差
        int check_n = std::min(15, dx_count);
        float sum = 0, sum_sq = 0;

        // 计算方差
        for (int i = 0; i < check_n; i++) {
            sum += dx[i];
            sum_sq += dx[i] * dx[i];
        }
        float mean = sum / check_n;
        float var = sum_sq / check_n - mean * mean;
        data.dx_var = var;

        // 方差小于0.4认为是直道（收紧阈值）
        if (var <= 0.30f) {
            straight_by_var = true;
        }
    }
    
    // 3. 融合判定（两者都认为是直道才确认）
    data.is_straight = straight_by_mnt && straight_by_var;
    
    // 4. 特殊情况：如果有明显双拐点，强制非直道
    if (data.down_lp_state && data.up_lp_state) {
        data.is_straight = false;
    }
    if (data.mid_lp_state && data.up_lp_state) {
        data.is_straight = false;
    }
    if (data.down_lp_state && data.mid_lp_state) {
        data.is_straight = false;
    }
    }  // dx 突变法主流程结束


    // // 时序平滑处理（防止拐点跳变）
    // if (data.down_lp_state) stabilize_corner(data, 0, is_left);
    // if (data.up_lp_state) stabilize_corner(data, 1, is_left);
    // if (data.mid_lp_state) stabilize_corner(data, 2, is_left);
}


// /**
//  * @brief 使用最长白列法检测十字路口拐点（复用 seek_pts_seed 的结果）
//  * @details 直接使用 track_base.white_column 的数据，不重复扫描
//  * @param bin_buf 二值化图像
//  * @param left_data 左边界数据引用
//  * @param right_data 右边界数据引用
//  * @return 是否成功检测到有效的十字路口拐点
//  */
// bool PointState::find_corners(const cv::Mat& bin_buf, BoundaryData& left_data, BoundaryData& right_data) {
//     if (bin_buf.empty()) {
//         return false;
//     }

//     // 直接使用 track_base.white_column 的结果
//     extern TrackBase track_base;
    
//     int max_length = track_base.white_column.maxNum;
//     int center_x = (track_base.white_column.starlineX + track_base.white_column.endlineX) / 2;
    
//     // 最小白列长度阈值
//     const int MIN_WHITE_COLUMN_LENGTH = 10;
//     if (max_length < MIN_WHITE_COLUMN_LENGTH) {
//         return false;
//     }
    
//     // 高度突变阈值（白列高度减少超过这个值认为是突变）- 进一步降低
//     const int HEIGHT_DROP_THRESH = 5;  // 从10降到5，更敏感
    
//     // 辅助函数：统计某一列从指定行向上的白色连续长度
//     auto calc_white_column_height = [&](int col, int start_y) -> int {
//         int height = 0;
//         for (int y = start_y; y >= ROI_TOP; y--) {
//             if (bin_buf.ptr<uchar>(y)[col] > 127) {
//                 height++;
//             } else {
//                 break;
//             }
//         }
//         return height;
//     };
    
//     // === 向左扫描，寻找左侧拐点 ===
//     int left_up_x = -1, left_up_y = -1;
//     int left_down_x = -1, left_down_y = -1;
//     int prev_height = max_length;
//     int corner_count = 0;
    
//     for (int x = center_x; x >= 5; x--) {
//         int curr_height = calc_white_column_height(x, VALID_START_ROW);
        
//         // 检测高度突变（高度突然减小）
//         if (prev_height - curr_height > HEIGHT_DROP_THRESH) {
//             corner_count++;
            
//             // 第一个突变点 = 上拐点
//             if (corner_count == 1) {
//                 left_up_x = x + 1;  // 突变前一列
//                 // Y坐标：在该列找白色区域的顶端
//                 for (int y = ROI_TOP; y <= VALID_START_ROW; y++) {
//                     if (bin_buf.ptr<uchar>(y)[left_up_x] > 127) {
//                         left_up_y = y;
//                         break;
//                     }
//                 }
//             }
//             // 第二个突变点 = 下拐点
//             else if (corner_count == 2) {
//                 left_down_x = x + 1;
//                 // Y坐标：在该列找白色区域的底端
//                 for (int y = VALID_START_ROW; y >= ROI_TOP; y--) {
//                     if (bin_buf.ptr<uchar>(y)[left_down_x] > 127) {
//                         left_down_y = y;
//                         break;
//                     }
//                 }
//                 break;  // 找到两个拐点，停止向左扫描
//             }
//         }
        
//         prev_height = curr_height;
//     }
    
//     // === 向右扫描，寻找右侧拐点 ===
//     int right_up_x = -1, right_up_y = -1;
//     int right_down_x = -1, right_down_y = -1;
//     prev_height = max_length;
//     corner_count = 0;
    
//     for (int x = center_x; x < CAM_WIDTH - 5; x++) {
//         int curr_height = calc_white_column_height(x, VALID_START_ROW);
        
//         // 检测高度突变
//         if (prev_height - curr_height > HEIGHT_DROP_THRESH) {
//             corner_count++;
            
//             // 第一个突变点 = 上拐点
//             if (corner_count == 1) {
//                 right_up_x = x - 1;  // 突变前一列
//                 // Y坐标：在该列找白色区域的顶端
//                 for (int y = ROI_TOP; y <= VALID_START_ROW; y++) {
//                     if (bin_buf.ptr<uchar>(y)[right_up_x] > 127) {
//                         right_up_y = y;
//                         break;
//                     }
//                 }
//             }
//             // 第二个突变点 = 下拐点
//             else if (corner_count == 2) {
//                 right_down_x = x - 1;
//                 // Y坐标：在该列找白色区域的底端
//                 for (int y = VALID_START_ROW; y >= ROI_TOP; y--) {
//                     if (bin_buf.ptr<uchar>(y)[right_down_x] > 127) {
//                         right_down_y = y;
//                         break;
//                     }
//                 }
//                 break;  // 找到两个拐点，停止向右扫描
//             }
//         }
        
//         prev_height = curr_height;
//     }
    
//     // 验证是否找到了有效的拐点（至少找到一个拐点就算有效）
//     bool left_has_corner = (left_up_x > 0 && left_up_y > 0) || (left_down_x > 0 && left_down_y > 0);
//     bool right_has_corner = (right_up_x > 0 && right_up_y > 0) || (right_down_x > 0 && right_down_y > 0);
    
//     if (!left_has_corner && !right_has_corner) {
//         return false;  // 左右都没找到任何拐点，检测失败
//     }
    
//     // 更新左边界数据（只要找到就更新）
//     if (left_up_x > 0 && left_up_y > 0) {
//         left_data.longest_white_up_state = true;
//         left_data.longest_white_up_pt = cv::Point(left_up_x, left_up_y);
//     }
//     if (left_down_x > 0 && left_down_y > 0) {
//         left_data.longest_white_down_state = true;
//         left_data.longest_white_down_pt = cv::Point(left_down_x, left_down_y);
//     }
    
//     // 更新右边界数据（只要找到就更新）
//     if (right_up_x > 0 && right_up_y > 0) {
//         right_data.longest_white_up_state = true;
//         right_data.longest_white_up_pt = cv::Point(right_up_x, right_up_y);
//     }
//     if (right_down_x > 0 && right_down_y > 0) {
//         right_data.longest_white_down_state = true;
//         right_data.longest_white_down_pt = cv::Point(right_down_x, right_down_y);
//     }
    
//     return true;
// }

// 斜率计算与突变检测

/**
 * @brief 计算边线斜率（最小二乘法）
 * @param line 边线数组
 * @param start_y 起始行
 * @param end_y 结束行
 * @return 斜率 k
 */
float PointState::get_line_k(int* line, int start_y, int end_y)
{
    float sum_y = 0, sum_x = 0, sum_yy = 0, sum_xy = 0;
    int n = 0;
    
    for (int y = start_y; y <= end_y; y++) {
        if (line[y] != -1) {
            sum_y += y;
            sum_x += line[y];
            sum_yy += y * y;
            sum_xy += y * line[y];
            n++;
        }
    }
    
    if (n < 3) return 0.0f;  // 点太少，无法计算
    
    float denom = n * sum_yy - sum_y * sum_y;
    if (std::abs(denom) < 1e-4f) return 0.0f;
    
    float k = (n * sum_xy - sum_y * sum_x) / denom;
    return k;
}

/**
 * @brief 检测斜率突变
 * @param now_k 当前斜率
 * @param last_k 上一帧斜率
 * @param threshold 突变阈值
 * @return true=发生突变, false=未突变
 */
bool PointState::check_line_k_change(float now_k, float last_k, float threshold)
{
    return std::abs(now_k - last_k) > threshold;
}




// 方向序列特征点检测函数　八邻域
/**
 * @brief 左边界丢线检测（基于方向序列）
 * @param data 边界数据
 */
static void DP_Update_Left_Lost(PointState::BoundaryData& data) {
    // 安全检查
    if (!data.dir_valid || data.dir_count < 10) {
        data.is_lost = false;
        return;
    }
    
    // 计算最高点（从轨迹点中找）
    uint8_t hightest = ROI_BOTTOM; 
    for (uint16_t i = 0; i < data.dir_count; i++) {
        if (data.trajectory_points[i][1] < hightest) {
            hightest = data.trajectory_points[i][1];
        }
    }
    
    if (hightest + 5 >= VALID_BOTTOM_ROW) {
        data.is_lost = false;
        return;
    }
    
    // 统计丢线行数
    uint8_t continuous_cnt = 0;
    for (uint8_t y = VALID_BOTTOM_ROW; y > hightest + 5; y--) {
        // 在轨迹点中查找该行的X坐标
        int current_x = -1;
        for (uint16_t i = 0; i < data.dir_count; i++) {
            if (data.trajectory_points[i][1] == y) {
                current_x = data.trajectory_points[i][0];
                break;
            }
        }
        
        // 如果该行没有轨迹点，或X坐标靠近左边界，认为丢线
        if (current_x == -1 || current_x <= VALID_LEFT_COL + 2) {  // 放宽边界判定
            continuous_cnt++;
        }
    }
    
    // 标志位更新 - 修复：降低阈值以适配ROI_HEIGHT=64
    data.is_lost = (continuous_cnt >= 8);  // 从12降低到8
    if (data.is_lost) {
        data.lost_count = continuous_cnt;
        data.lost_y = hightest + 5;  // 记录丢线位置
    }
}

/**
 * @brief 右边界丢线检测（基于方向序列）
 * @param data 边界数据
 */
static void DP_Update_Right_Lost(PointState::BoundaryData& data) {
    // 安全检查
    if (!data.dir_valid || data.dir_count < 10) {
        data.is_lost = false;
        return;
    }
    
    // 计算最高点 
    uint8_t hightest = ROI_BOTTOM; 
    for (uint16_t i = 0; i < data.dir_count; i++) {
        if (data.trajectory_points[i][1] < hightest) {
            hightest = data.trajectory_points[i][1];
        }
    }
    
    if (hightest + 5 >= VALID_BOTTOM_ROW) {
        data.is_lost = false;
        return;
    }
    
    // 统计丢线行数
    uint8_t continuous_cnt = 0;
    for (uint8_t y = VALID_BOTTOM_ROW; y > hightest + 5; y--) {
        // 在轨迹点中查找该行的X坐标
        int current_x = -1;
        for (uint16_t i = 0; i < data.dir_count; i++) {
            if (data.trajectory_points[i][1] == y) {
                current_x = data.trajectory_points[i][0];
                break;
            }
        }
        
        // 如果该行没有轨迹点，或X坐标靠近右边界，认为丢线
        if (current_x == -1 || current_x >= VALID_RIGHT_COL - 2) {  // 放宽边界判定
            continuous_cnt++;
        }
    }
    
    // 标志位更新 - 修复：降低阈值以适配ROI_HEIGHT=64
    data.is_lost = (continuous_cnt >= 8);  // 从12降低到8
    if (data.is_lost) {
        data.lost_count = continuous_cnt;
        data.lost_y = hightest + 5;  // 记录丢线位置
    }
}


/**
 * @brief 左边界单调性检测（基于方向序列）
 * @param data 边界数据
 * @return 单调段长度
 */
static uint8_t DP_Update_Left_Mnt(PointState::BoundaryData& data) {
    // 安全检查
    if (!data.dir_valid || data.dir_count < 10) {
        return 0;
    }
    
    // 计算最高点 
    uint8_t hightest = ROI_BOTTOM;  
    for (uint16_t i = 0; i < data.dir_count; i++) {
        if (data.trajectory_points[i][1] < hightest) {
            hightest = data.trajectory_points[i][1];
        }
    }
    
    if (VALID_BOTTOM_ROW <= hightest) {
        return 0;
    }
    
    // 主功能循环：检测单调性
    uint8_t valid_cnt = 0;
    for (uint16_t i = 0; i < data.dir_count - 1; i++) {
        uint8_t current_y = data.trajectory_points[i][1];
        uint8_t next_y = data.trajectory_points[i+1][1];
        
        // 跳过不在扫描范围内的点
        if (current_y > VALID_BOTTOM_ROW || current_y <= hightest) {
            continue;
        }
        
        // 检查X坐标变化（左线单调：X递减或保持，即向左或向上）
        int16_t current_x = data.trajectory_points[i][0];
        int16_t next_x = data.trajectory_points[i+1][0];
        int16_t diff = current_x - next_x;
        
        // 单调性判断：X变化小于阈值且不向右
        if (std::abs(diff) <= 2 && diff <= 0) {  // mnt_threshold = 2
            valid_cnt++;
        } 
        else {
            break;  // 发现非单调点立即退出
        }
    }
    
    // 返回单调段长度
    return valid_cnt;
}

/**
 * @brief 右边界单调性检测（基于方向序列）
 * @param data 边界数据
 * @return 单调段长度
 */
static uint8_t DP_Update_Right_Mnt(PointState::BoundaryData& data) {
    // 安全检查
    if (!data.dir_valid || data.dir_count < 10) {
        return 0;
    }
    
    // 计算最高点
    uint8_t hightest = ROI_BOTTOM;
    for (uint16_t i = 0; i < data.dir_count; i++) {
        if (data.trajectory_points[i][1] < hightest) {
            hightest = data.trajectory_points[i][1];
        }
    }
    
    if (VALID_BOTTOM_ROW <= hightest) {
        return 0;
    }
    
    // 主功能循环：检测单调性
    uint8_t valid_cnt = 0;
    for (uint16_t i = 0; i < data.dir_count - 1; i++) {
        uint8_t current_y = data.trajectory_points[i][1];
        uint8_t next_y = data.trajectory_points[i+1][1];
        
        // 跳过不在扫描范围内的点
        if (current_y > VALID_BOTTOM_ROW || current_y <= hightest) {
            continue;
        }
        
        // 检查X坐标变化（右线单调：X递增或保持，即向右或向上）
        int16_t current_x = data.trajectory_points[i][0];
        int16_t next_x = data.trajectory_points[i+1][0];
        int16_t diff = current_x - next_x;
        
        // 单调性判断：X变化小于阈值且不向左
        if (std::abs(diff) <= 2 && diff >= 0) {  // mnt_threshold = 2
            valid_cnt++;
        } 
        else {
            break;  // 发现非单调点立即退出
        }
    }
    
    // 返回单调段长度
    return valid_cnt;
}

/**
 * @brief 左边界 UTL 拐点检测（上转左）
 * @param data 边界数据
 * @param mode 检测模式 (0=严格, 1=宽松)
 * @return 拐点坐标，未检测到返回 (-1, -1)
 */
static cv::Point Get_Left_UTL_Point(PointState::BoundaryData& data, uint8_t mode) {
    // 安全检测
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    // 主检测逻辑
    for (uint16_t i = scan_end; i > 3; i--) {
        bool prev_cond = false;
        bool next_cond = false;
        
        switch(mode) {
            case 0: {
                // 严格模式：前序方向序列检查（检测向上趋势）
                prev_cond =
                    (data.dir_sequence[i-4] == L_VU || data.dir_sequence[i-4] == L_RU) &&
                    (data.dir_sequence[i-3] == L_VU || data.dir_sequence[i-3] == L_RU) &&
                    (data.dir_sequence[i-2] == L_VU || data.dir_sequence[i-2] == L_RU);
                
                // 后续方向序列检查（检测左转趋势）
                next_cond =
                    (data.dir_sequence[i]   == L_VL || data.dir_sequence[i]   == L_LD || data.dir_sequence[i]   == L_LU) &&
                    (data.dir_sequence[i+1] == L_VL || data.dir_sequence[i+1] == L_LD) &&
                    (data.dir_sequence[i+2] == L_VL || data.dir_sequence[i+2] == L_LD) &&
                    (data.dir_sequence[i+3] == L_VL || data.dir_sequence[i+3] == L_LD);
                break;
            }
            case 1: {
                // 宽松模式1：允许左上方向
                prev_cond =
                    (data.dir_sequence[i-4] == L_VU || data.dir_sequence[i-4] == L_RU) &&
                    (data.dir_sequence[i-3] == L_VU || data.dir_sequence[i-3] == L_RU) &&
                    (data.dir_sequence[i-2] == L_VU || data.dir_sequence[i-2] == L_RU);
                
                next_cond =
                    (data.dir_sequence[i+1] == L_VL || data.dir_sequence[i+1] == L_LD || data.dir_sequence[i+1] == L_LU) &&
                    (data.dir_sequence[i+2] == L_VL || data.dir_sequence[i+2] == L_LD || data.dir_sequence[i+2] == L_LU) &&
                    (data.dir_sequence[i+3] == L_VL || data.dir_sequence[i+3] == L_LD || data.dir_sequence[i+3] == L_LU);
                break;
            }
            case 2: {
                // 宽松模式2（圆环专用）：使用计数法，不要求严格连续
                // 前序：3个点中至少2个是"向上"方向（包括左上、上、右上）
                int up_count = 0;
                if (data.dir_sequence[i-4] == L_VU || data.dir_sequence[i-4] == L_RU || data.dir_sequence[i-4] == L_LU) up_count++;
                if (data.dir_sequence[i-3] == L_VU || data.dir_sequence[i-3] == L_RU || data.dir_sequence[i-3] == L_LU) up_count++;
                if (data.dir_sequence[i-2] == L_VU || data.dir_sequence[i-2] == L_RU || data.dir_sequence[i-2] == L_LU) up_count++;
                prev_cond = (up_count >= 2);
                
                // 后续：3个点中至少2个是"向左"方向（包括左下、左、左上）
                int left_count = 0;
                if (data.dir_sequence[i+1] == L_VL || data.dir_sequence[i+1] == L_LD || data.dir_sequence[i+1] == L_LU) left_count++;
                if (data.dir_sequence[i+2] == L_VL || data.dir_sequence[i+2] == L_LD || data.dir_sequence[i+2] == L_LU) left_count++;
                if (data.dir_sequence[i+3] == L_VL || data.dir_sequence[i+3] == L_LD || data.dir_sequence[i+3] == L_LU) left_count++;
                next_cond = (left_count >= 2);
                break;
            }
            default:
                break;
        }
        
        // Y坐标变化检查
        int check_y = (data.trajectory_points[i+1][1] - data.trajectory_points[i+2][1]) +
                      (data.trajectory_points[i+2][1] - data.trajectory_points[i+3][1]) +
                      (data.trajectory_points[i+3][1] - data.trajectory_points[i+4][1]);
        check_y /= 3;
        
        // 综合条件判断
        if (prev_cond && next_cond && check_y >= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 左边界 RTU 拐点检测（右转上）
 * @param data 边界数据
 * @param mode 检测模式
 * @return 拐点坐标
 */
static cv::Point Get_Left_RTU_Point(PointState::BoundaryData& data, uint8_t mode) {
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    for (uint16_t i = 3; i < scan_end; i++) {
        uint8_t prev_cond = 0;
        uint8_t next_cond = 0;
        
        switch(mode) {
            case 0: {
                prev_cond =
                    (data.dir_sequence[i-4] == L_RD || data.dir_sequence[i-4] == L_VR) &&
                    (data.dir_sequence[i-3] == L_RD || data.dir_sequence[i-3] == L_VR) &&
                    (data.dir_sequence[i-2] == L_RD || data.dir_sequence[i-2] == L_VR) &&
                    (data.dir_sequence[i-1] == L_RD || data.dir_sequence[i-1] == L_VR);
                
                next_cond =
                    (data.dir_sequence[i]   == L_VU || data.dir_sequence[i]   == L_RU) &&
                    (data.dir_sequence[i+1] == L_VU || data.dir_sequence[i+1] == L_RU) &&
                    (data.dir_sequence[i+2] == L_VU || data.dir_sequence[i+2] == L_RU) &&
                    (data.dir_sequence[i+3] == L_VU || data.dir_sequence[i+3] == L_RU);
                break;
            }
            case 1: {
                prev_cond =
                    (data.dir_sequence[i-4] == L_RD || data.dir_sequence[i-4] == L_VR) &&
                    (data.dir_sequence[i-3] == L_RD || data.dir_sequence[i-3] == L_VR) &&
                    (data.dir_sequence[i-2] == L_RD || data.dir_sequence[i-2] == L_VR) &&
                    (data.dir_sequence[i-1] == L_RD || data.dir_sequence[i-1] == L_VR);
                
                next_cond =
                    (data.dir_sequence[i]   == L_VU || data.dir_sequence[i]   == L_RU || data.dir_sequence[i]   == L_VD) &&
                    (data.dir_sequence[i+1] == L_VU || data.dir_sequence[i+1] == L_RU || data.dir_sequence[i+1] == L_VD) &&
                    (data.dir_sequence[i+2] == L_VU || data.dir_sequence[i+2] == L_RU || data.dir_sequence[i+2] == L_VD) &&
                    (data.dir_sequence[i+3] == L_VU || data.dir_sequence[i+3] == L_RU || data.dir_sequence[i+3] == L_VD);
                break;
            }
        }
        
        int check_y = (data.trajectory_points[i-1][1] - data.trajectory_points[i-2][1]) +
                      (data.trajectory_points[i-2][1] - data.trajectory_points[i-3][1]) +
                      (data.trajectory_points[i-3][1] - data.trajectory_points[i-4][1]);
        check_y /= 3;
        
        if (prev_cond && next_cond && check_y <= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 左边界 UTR 拐点检测（上转右）
 * @param data 边界数据
 * @param mode 检测模式
 * @return 拐点坐标
 */
static cv::Point Get_Left_UTR_Point(PointState::BoundaryData& data, uint8_t mode) {
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    for (uint16_t i = scan_end; i > 3; i--) {
        uint8_t prev_cond = 0;
        uint8_t next_cond = 0;
        
        switch(mode) {
            case 0: {
                prev_cond =
                    (data.dir_sequence[i-4] == L_VU || data.dir_sequence[i-4] == L_LU) &&
                    (data.dir_sequence[i-3] == L_VU || data.dir_sequence[i-3] == L_LU) &&
                    (data.dir_sequence[i-2] == L_VU || data.dir_sequence[i-2] == L_LU);
                
                next_cond =
                    (data.dir_sequence[i]   == L_VR || data.dir_sequence[i]   == L_RD || data.dir_sequence[i]   == L_RU) &&
                    (data.dir_sequence[i+1] == L_VR || data.dir_sequence[i+1] == L_RD) &&
                    (data.dir_sequence[i+2] == L_VR || data.dir_sequence[i+2] == L_RD) &&
                    (data.dir_sequence[i+3] == L_VR || data.dir_sequence[i+3] == L_RD);
                break;
            }
            case 1: {
                prev_cond =
                    (data.dir_sequence[i-4] == L_VU || data.dir_sequence[i-4] == L_RU) &&
                    (data.dir_sequence[i-3] == L_VU || data.dir_sequence[i-3] == L_RU) &&
                    (data.dir_sequence[i-2] == L_VU || data.dir_sequence[i-2] == L_RU);
                
                next_cond =
                    (data.dir_sequence[i]   == L_VR || data.dir_sequence[i]   == L_RD || data.dir_sequence[i]   == L_RU) &&
                    (data.dir_sequence[i+1] == L_VR || data.dir_sequence[i+1] == L_RD || data.dir_sequence[i+1] == L_RU) &&
                    (data.dir_sequence[i+2] == L_VR || data.dir_sequence[i+2] == L_RD || data.dir_sequence[i+2] == L_RU) &&
                    (data.dir_sequence[i+3] == L_VR || data.dir_sequence[i+3] == L_RD || data.dir_sequence[i+3] == L_RU);
                break;
            }
        }
        
        int check_y = (data.trajectory_points[i+1][1] - data.trajectory_points[i+2][1]) +
                      (data.trajectory_points[i+2][1] - data.trajectory_points[i+3][1]) +
                      (data.trajectory_points[i+3][1] - data.trajectory_points[i+4][1]);
        check_y /= 3;
        
        if (prev_cond && next_cond && check_y >= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}


/**
 * @brief 右边界 UTR 拐点检测（上转右）
 * @param data 边界数据
 * @param mode 检测模式
 * @return 拐点坐标
 */
static cv::Point Get_Right_UTR_Point(PointState::BoundaryData& data, uint8_t mode) {
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    for (uint16_t i = scan_end; i > 3; i--) {
        bool prev_cond = false;
        bool next_cond = false;
        
        switch(mode) {
            case 0: {
                // 严格模式
                prev_cond =
                    (data.dir_sequence[i-4] == R_VU || data.dir_sequence[i-4] == R_LU) &&
                    (data.dir_sequence[i-3] == R_VU || data.dir_sequence[i-3] == R_LU) &&
                    (data.dir_sequence[i-2] == R_VU || data.dir_sequence[i-2] == R_LU);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VR || data.dir_sequence[i]   == R_RD) &&
                    (data.dir_sequence[i+1] == R_VR || data.dir_sequence[i+1] == R_RD) &&
                    (data.dir_sequence[i+2] == R_VR || data.dir_sequence[i+2] == R_RD) &&
                    (data.dir_sequence[i+3] == R_VR || data.dir_sequence[i+3] == R_RD);
                break;
            }
            case 1: {
                // 宽松模式1
                prev_cond =
                    (data.dir_sequence[i-4] == R_VU || data.dir_sequence[i-4] == R_LU) &&
                    (data.dir_sequence[i-3] == R_VU || data.dir_sequence[i-3] == R_LU) &&
                    (data.dir_sequence[i-2] == R_VU || data.dir_sequence[i-2] == R_LU);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VR || data.dir_sequence[i]   == R_RD || data.dir_sequence[i]   == R_RU) &&
                    (data.dir_sequence[i+1] == R_VR || data.dir_sequence[i+1] == R_RD || data.dir_sequence[i+1] == R_RU) &&
                    (data.dir_sequence[i+2] == R_VR || data.dir_sequence[i+2] == R_RD || data.dir_sequence[i+2] == R_RU) &&
                    (data.dir_sequence[i+3] == R_VR || data.dir_sequence[i+3] == R_RD || data.dir_sequence[i+3] == R_RU);
                break;
            }
            case 2: {
                // 宽松模式2（圆环专用）：使用计数法
                // 前序：3个点中至少2个是"向上"方向（包括左上、上、右上）
                int up_count = 0;
                if (data.dir_sequence[i-4] == R_VU || data.dir_sequence[i-4] == R_LU || data.dir_sequence[i-4] == R_RU) up_count++;
                if (data.dir_sequence[i-3] == R_VU || data.dir_sequence[i-3] == R_LU || data.dir_sequence[i-3] == R_RU) up_count++;
                if (data.dir_sequence[i-2] == R_VU || data.dir_sequence[i-2] == R_LU || data.dir_sequence[i-2] == R_RU) up_count++;
                prev_cond = (up_count >= 2);
                
                // 后续：3个点中至少2个是"向右"方向（包括右下、右、右上）
                int right_count = 0;
                if (data.dir_sequence[i+1] == R_VR || data.dir_sequence[i+1] == R_RD || data.dir_sequence[i+1] == R_RU) right_count++;
                if (data.dir_sequence[i+2] == R_VR || data.dir_sequence[i+2] == R_RD || data.dir_sequence[i+2] == R_RU) right_count++;
                if (data.dir_sequence[i+3] == R_VR || data.dir_sequence[i+3] == R_RD || data.dir_sequence[i+3] == R_RU) right_count++;
                next_cond = (right_count >= 2);
                break;
            }
            default:
                break;
        }
        
        int check_y = (data.trajectory_points[i+1][1] - data.trajectory_points[i+2][1]) +
                      (data.trajectory_points[i+2][1] - data.trajectory_points[i+3][1]) +
                      (data.trajectory_points[i+3][1] - data.trajectory_points[i+4][1]);
        check_y /= 3;
        
        if (prev_cond && next_cond && check_y >= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 右边界 LTU 拐点检测（左转上）
 * @param data 边界数据
 * @param mode 检测模式
 * @return 拐点坐标
 */
static cv::Point Get_Right_LTU_Point(PointState::BoundaryData& data, uint8_t mode) {
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    for (uint16_t i = 3; i < scan_end; i++) {
        uint8_t prev_cond = 0;
        uint8_t next_cond = 0;
        
        switch(mode) {
            case 0: {
                prev_cond =
                    (data.dir_sequence[i-4] == R_LD || data.dir_sequence[i-4] == R_VL) &&
                    (data.dir_sequence[i-3] == R_LD || data.dir_sequence[i-3] == R_VL) &&
                    (data.dir_sequence[i-2] == R_LD || data.dir_sequence[i-2] == R_VL) &&
                    (data.dir_sequence[i-1] == R_LD || data.dir_sequence[i-1] == R_VL);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VU || data.dir_sequence[i]   == R_LU) &&
                    (data.dir_sequence[i+1] == R_VU || data.dir_sequence[i+1] == R_LU) &&
                    (data.dir_sequence[i+2] == R_VU || data.dir_sequence[i+2] == R_LU) &&
                    (data.dir_sequence[i+3] == R_VU || data.dir_sequence[i+3] == R_LU);
                break;
            }
            case 1: {
                prev_cond =
                    (data.dir_sequence[i-4] == R_LD || data.dir_sequence[i-4] == R_VL || data.dir_sequence[i-4] == R_VD) &&
                    (data.dir_sequence[i-3] == R_LD || data.dir_sequence[i-3] == R_VL || data.dir_sequence[i-3] == R_VD) &&
                    (data.dir_sequence[i-2] == R_LD || data.dir_sequence[i-2] == R_VL || data.dir_sequence[i-2] == R_VD) &&
                    (data.dir_sequence[i-1] == R_LD || data.dir_sequence[i-1] == R_VL || data.dir_sequence[i-1] == R_VD);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VU || data.dir_sequence[i]   == R_LU) &&
                    (data.dir_sequence[i+1] == R_VU || data.dir_sequence[i+1] == R_LU) &&
                    (data.dir_sequence[i+2] == R_VU || data.dir_sequence[i+2] == R_LU) &&
                    (data.dir_sequence[i+3] == R_VU || data.dir_sequence[i+3] == R_LU);
                break;
            }
        }
        
        int check_y = (data.trajectory_points[i-1][1] - data.trajectory_points[i-2][1]) +
                      (data.trajectory_points[i-2][1] - data.trajectory_points[i-3][1]) +
                      (data.trajectory_points[i-3][1] - data.trajectory_points[i-4][1]);
        check_y /= 3;
        
        if (prev_cond && next_cond && check_y <= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 右边界 UTL 拐点检测（上转左）
 * @param data 边界数据
 * @param mode 检测模式
 * @return 拐点坐标
 */
static cv::Point Get_Right_UTL_Point(PointState::BoundaryData& data, uint8_t mode) {
    if (!data.dir_valid || data.dir_count < 10) {
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    const uint16_t scan_end = data_len - 3;
    
    for (uint16_t i = scan_end; i > 3; i--) {
        uint8_t prev_cond = 0;
        uint8_t next_cond = 0;
        
        switch(mode) {
            case 0: {
                prev_cond =
                    (data.dir_sequence[i-4] == R_VU || data.dir_sequence[i-4] == R_RU) &&
                    (data.dir_sequence[i-3] == R_VU || data.dir_sequence[i-3] == R_RU) &&
                    (data.dir_sequence[i-2] == R_VU || data.dir_sequence[i-2] == R_RU);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VL || data.dir_sequence[i]   == R_LD || data.dir_sequence[i]   == R_LU) &&
                    (data.dir_sequence[i+1] == R_VL || data.dir_sequence[i+1] == R_LD) &&
                    (data.dir_sequence[i+2] == R_VL || data.dir_sequence[i+2] == R_LD) &&
                    (data.dir_sequence[i+3] == R_VL || data.dir_sequence[i+3] == R_LD);
                break;
            }
            case 1: {
                prev_cond =
                    (data.dir_sequence[i-4] == R_VU || data.dir_sequence[i-4] == R_LU) &&
                    (data.dir_sequence[i-3] == R_VU || data.dir_sequence[i-3] == R_LU) &&
                    (data.dir_sequence[i-2] == R_VU || data.dir_sequence[i-2] == R_LU);
                
                next_cond =
                    (data.dir_sequence[i]   == R_VL || data.dir_sequence[i]   == R_LD || data.dir_sequence[i]   == R_LU) &&
                    (data.dir_sequence[i+1] == R_VL || data.dir_sequence[i+1] == R_LD || data.dir_sequence[i+1] == R_LU) &&
                    (data.dir_sequence[i+2] == R_VL || data.dir_sequence[i+2] == R_LD || data.dir_sequence[i+2] == R_LU) &&
                    (data.dir_sequence[i+3] == R_VL || data.dir_sequence[i+3] == R_LD || data.dir_sequence[i+3] == R_LU);
                break;
            }
        }
        
        int check_y = (data.trajectory_points[i+1][1] - data.trajectory_points[i+2][1]) +
                      (data.trajectory_points[i+2][1] - data.trajectory_points[i+3][1]) +
                      (data.trajectory_points[i+3][1] - data.trajectory_points[i+4][1]);
        check_y /= 3;
        
        if (prev_cond && next_cond && check_y >= 0) {
            return cv::Point(data.trajectory_points[i][0], data.trajectory_points[i][1]);
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 左边界圆弧拐点检测（基于方向序列）
 * @param data 边界数据
 * @return 圆弧拐点坐标
 */
static cv::Point Get_L_Arc_Turn_Point(PointState::BoundaryData& data) {
    // 安全检测
    if (!data.dir_valid || data.dir_count < 13) {  // 需要至少13个点：i-6到i+5
        return cv::Point(-1, -1);
    }
             
    const uint16_t data_len = data.dir_count;
    
    // 主检测逻辑
    for (uint16_t i = 7; i < (data_len - 5); i++) {
        // 坐标有效性检查
        const uint8_t y_pos = data.trajectory_points[i][1];
        if (y_pos < 3 || y_pos > (CAM_HEIGHT - 8)) {  // LINEy_MIN + 2, LINEy_MAX - 7
            continue;
        }
        
        // 几何特征检测（X坐标极值点判断）
        const uint8_t x_pos = data.trajectory_points[i][0];
        if ((x_pos > data.trajectory_points[i - 6][0]) &&    // 前6点
            (x_pos > data.trajectory_points[i - 4][0]) &&     // 前4点
            (x_pos >= data.trajectory_points[i - 1][0]) &&    // 前1点
            (x_pos >= data.trajectory_points[i + 1][0]) &&    // 后1点
            (x_pos > data.trajectory_points[i + 3][0]) &&     // 后3点
            (x_pos > data.trajectory_points[i + 5][0]))       // 后5点
        {
            // 方向特征验证（圆弧拐点模式）
            if ((data.dir_sequence[i + 1] == L_LU || data.dir_sequence[i + 1] == L_VU) &&
                (data.dir_sequence[i + 3] == L_LU || data.dir_sequence[i + 3] == L_VU) &&
                (data.dir_sequence[i - 1] == L_RU || data.dir_sequence[i - 1] == L_VU) &&
                (data.dir_sequence[i - 3] == L_RU || data.dir_sequence[i - 3] == L_VU))
            {
                return cv::Point(x_pos, y_pos);
            }
        }
    }
    
    return cv::Point(-1, -1);
}

/**
 * @brief 右边界圆弧拐点检测（基于方向序列）
 * @param data 边界数据
 * @return 圆弧拐点坐标
 */
static cv::Point Get_R_Arc_Turn_Point(PointState::BoundaryData& data) {
    // 安全检测
    if (!data.dir_valid || data.dir_count < 20) {  // 需要至少20个点
        return cv::Point(-1, -1);
    }
    
    const uint16_t data_len = data.dir_count;
    
    // 主检测逻辑
    for (uint16_t i = 7; i < (data_len - 5); i++) {
        // 坐标有效性检查（Y方向边界保护）
        const uint8_t y_pos = data.trajectory_points[i][1];
        if (y_pos < 3 || y_pos > (CAM_HEIGHT - 3)) {  // LINEy_MIN + 2, LINEy_MAX - 2
            continue;
        }
        
        // 几何特征检测（X坐标极值点判断）
        const uint8_t x_pos = data.trajectory_points[i][0];
        if ((x_pos < data.trajectory_points[i - 6][0]) &&    // 前6点（右侧边界X递减）
            (x_pos < data.trajectory_points[i - 4][0]) &&     // 前4点
            (x_pos <= data.trajectory_points[i - 1][0]) &&    // 前1点
            (x_pos <= data.trajectory_points[i + 1][0]) &&    // 后1点
            (x_pos < data.trajectory_points[i + 3][0]) &&     // 后3点
            (x_pos < data.trajectory_points[i + 5][0]))       // 后5点
        {
            // 方向特征验证（圆弧拐点模式）
            if ((data.dir_sequence[i + 1] == R_VU || data.dir_sequence[i + 1] == R_RU) &&  // 后1点
                (data.dir_sequence[i + 3] == R_VU || data.dir_sequence[i + 3] == R_RU) &&  // 后3点
                (data.dir_sequence[i - 1] == R_VU || data.dir_sequence[i - 1] == R_LU) &&  // 前1点
                (data.dir_sequence[i - 3] == R_VU || data.dir_sequence[i - 3] == R_LU))    // 前3点
            {
                return cv::Point(x_pos, y_pos);
            }
        }
    }
    
    return cv::Point(-1, -1);
}

//==================================================================================
// 新增拐点检测方法 - 边线跳变检测
//==================================================================================

/**
 * @brief 边线跳变检测 - 左边界下拐点（向左跳变）
 * @param data 边界数据
 * @param jump_threshold X坐标跳变阈值（默认5像素）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_left_down_jump(BoundaryData& data, int jump_threshold) {
    // 从下往上扫描，检测X坐标向左突变（line[i-1] - line[i] >= threshold）
    for (int i = VALID_START_ROW; i > VALID_END_ROW + 3; i--) {
        if (data.line[i] == -1 || data.line[i-1] == -1) continue;
        
        int jump = data.line[i-1] - data.line[i];  // 修正：向左跳变
        if (jump >= jump_threshold) {
            // 验证跳变的稳定性（检查前后几行）
            bool stable = true;
            for (int j = 1; j <= 2 && (i-j) > VALID_END_ROW; j++) {
                if (data.line[i-j] == -1) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                return cv::Point(data.line[i], i);
            }
        }
    }
    return cv::Point(-1, -1);
}

/**
 * @brief 边线跳变检测 - 左边界上拐点（向左跳变）
 * @param data 边界数据
 * @param jump_threshold X坐标跳变阈值（默认5像素）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_left_up_jump(BoundaryData& data, int jump_threshold) {
    // 从上往下扫描，检测X坐标向左突变（line[i-1] - line[i] >= threshold）
    for (int i = VALID_END_ROW + 3; i < VALID_START_ROW; i++) {
        if (data.line[i] == -1 || data.line[i-1] == -1) continue;
        
        int jump = data.line[i-1] - data.line[i];
        if (jump >= jump_threshold) {
            // 验证跳变的稳定性
            bool stable = true;
            for (int j = 1; j <= 2 && (i+j) < VALID_START_ROW; j++) {
                if (data.line[i+j] == -1) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                return cv::Point(data.line[i], i);
            }
        }
    }
    return cv::Point(-1, -1);
}

/**
 * @brief 边线跳变检测 - 右边界下拐点（向右跳变）
 * @param data 边界数据
 * @param jump_threshold X坐标跳变阈值（默认5像素）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_right_down_jump(BoundaryData& data, int jump_threshold) {
    // 从下往上扫描，检测X坐标向右突变（line[i] - line[i-1] >= threshold）
    for (int i = VALID_START_ROW; i > VALID_END_ROW + 3; i--) {
        if (data.line[i] == -1 || data.line[i-1] == -1) continue;
        
        int jump = data.line[i] - data.line[i-1];  // 修正：向右跳变
        if (jump >= jump_threshold) {
            // 验证跳变的稳定性
            bool stable = true;
            for (int j = 1; j <= 2 && (i-j) > VALID_END_ROW; j++) {
                if (data.line[i-j] == -1) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                return cv::Point(data.line[i], i);
            }
        }
    }
    return cv::Point(-1, -1);
}

/**
 * @brief 边线跳变检测 - 右边界上拐点（向右跳变）
 * @param data 边界数据
 * @param jump_threshold X坐标跳变阈值（默认5像素）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_right_up_jump(BoundaryData& data, int jump_threshold) {
    // 从上往下扫描，检测X坐标向右突变（line[i] - line[i-1] >= threshold）
    for (int i = VALID_END_ROW + 3; i < VALID_START_ROW; i++) {
        if (data.line[i] == -1 || data.line[i-1] == -1) continue;
        
        int jump = data.line[i] - data.line[i-1];
        if (jump >= jump_threshold) {
            // 验证跳变的稳定性
            bool stable = true;
            for (int j = 1; j <= 2 && (i+j) < VALID_START_ROW; j++) {
                if (data.line[i+j] == -1) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                return cv::Point(data.line[i], i);
            }
        }
    }
    return cv::Point(-1, -1);
}

//==================================================================================
// 新增拐点检测方法 - 单调性检测（弧形拐点）
//==================================================================================

/**
 * @brief 单调性检测 - 左边界弧形拐点（X坐标局部极值）
 * @param data 边界数据
 * @param check_window 检测窗口大小（前后各check_window行）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_left_arc_monotonicity(BoundaryData& data, int check_window) {
    // 左边界弧形拐点：寻找X坐标的局部最大值（最右侧的点）
    for (int i = VALID_END_ROW + check_window; i < VALID_START_ROW - check_window; i++) {
        if (data.line[i] == -1) continue;
        
        bool is_local_max = true;
        int current_x = data.line[i];
        
        // 检查前后窗口内的点
        for (int j = 1; j <= check_window; j++) {
            // 检查前面的点
            if (i - j >= VALID_END_ROW && data.line[i-j] != -1) {
                if (data.line[i-j] >= current_x) {
                    is_local_max = false;
                    break;
                }
            }
            // 检查后面的点
            if (i + j <= VALID_START_ROW && data.line[i+j] != -1) {
                if (data.line[i+j] >= current_x) {
                    is_local_max = false;
                    break;
                }
            }
        }
        
        if (is_local_max) {
            return cv::Point(current_x, i);
        }
    }
    return cv::Point(-1, -1);
}

/**
 * @brief 单调性检测 - 右边界弧形拐点（X坐标局部极值）
 * @param data 边界数据
 * @param check_window 检测窗口大小（前后各check_window行）
 * @return 检测到的拐点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_right_arc_monotonicity(BoundaryData& data, int check_window) {
    // 右边界弧形拐点：寻找X坐标的局部最小值（最左侧的点）
    for (int i = VALID_END_ROW + check_window; i < VALID_START_ROW - check_window; i++) {
        if (data.line[i] == -1) continue;
        
        bool is_local_min = true;
        int current_x = data.line[i];
        
        // 检查前后窗口内的点
        for (int j = 1; j <= check_window; j++) {
            // 检查前面的点
            if (i - j >= VALID_END_ROW && data.line[i-j] != -1) {
                if (data.line[i-j] <= current_x) {
                    is_local_min = false;
                    break;
                }
            }
            // 检查后面的点
            if (i + j <= VALID_START_ROW && data.line[i+j] != -1) {
                if (data.line[i+j] <= current_x) {
                    is_local_min = false;
                    break;
                }
            }
        }
        
        if (is_local_min) {
            return cv::Point(current_x, i);
        }
    }
    return cv::Point(-1, -1);
}

//==================================================================================
// 新增拐点检测方法 - V点和A点检测（环岛特征）
//==================================================================================

/**
 * @brief V点检测 - 轨迹点Y坐标局部最大值（环岛入口特征）
 * @param data 边界数据
 * @param check_window 检测窗口大小
 * @return 检测到的V点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_v_point(BoundaryData& data, int check_window) {
    // 需要有效的方向序列数据
    if (!data.dir_valid || data.dir_count < (check_window * 2 + 1)) {
        return cv::Point(-1, -1);
    }
    
    // 寻找Y坐标的局部最大值（图像坐标系中Y越大越靠下）
    for (uint16_t i = check_window; i < data.dir_count - check_window; i++) {
        uint16_t current_y = data.trajectory_points[i][1];
        bool is_local_max = true;
        
        // 检查前后窗口内的点
        for (int j = 1; j <= check_window; j++) {
            if (data.trajectory_points[i-j][1] >= current_y || 
                data.trajectory_points[i+j][1] >= current_y) {
                is_local_max = false;
                break;
            }
        }
        
        if (is_local_max) {
            return cv::Point(data.trajectory_points[i][0], current_y);
        }
    }
    return cv::Point(-1, -1);
}

/**
 * @brief A点检测 - 轨迹点Y坐标局部最小值（环岛出口特征）
 * @param data 边界数据
 * @param check_window 检测窗口大小
 * @return 检测到的A点坐标，未检测到返回(-1,-1)
 */
cv::Point PointState::detect_a_point(BoundaryData& data, int check_window) {
    // 需要有效的方向序列数据
    if (!data.dir_valid || data.dir_count < (check_window * 2 + 1)) {
        return cv::Point(-1, -1);
    }
    
    // 寻找Y坐标的局部最小值（图像坐标系中Y越小越靠上）
    for (uint16_t i = check_window; i < data.dir_count - check_window; i++) {
        uint16_t current_y = data.trajectory_points[i][1];
        bool is_local_min = true;
        
        // 检查前后窗口内的点
        for (int j = 1; j <= check_window; j++) {
            if (data.trajectory_points[i-j][1] <= current_y || 
                data.trajectory_points[i+j][1] <= current_y) {
                is_local_min = false;
                break;
            }
        }
        
        if (is_local_min) {
            return cv::Point(data.trajectory_points[i][0], current_y);
        }
    }
    return cv::Point(-1, -1);
}

//==================================================================================
// 新增拐点检测方法 - 边线连续性检测
//==================================================================================

/**
 * @brief 边线连续性检测
 * @param data 边界数据
 * @param continuity_threshold 连续性阈值（默认5像素）
 * @return 第一个断点的行号，无断点返回-1
 */
int PointState::detect_discontinuity(BoundaryData& data, int continuity_threshold) {
    // 从下往上扫描，检测边线是否连续
    for (int i = VALID_START_ROW; i > VALID_END_ROW; i--) {
        if (data.line[i] == -1) {
            // 遇到丢线点，返回该行号
            return i;
        }
        
        if (i > VALID_END_ROW && data.line[i-1] != -1) {
            // 检查相邻两行的X坐标差值
            int diff = std::abs(data.line[i] - data.line[i-1]);
            if (diff >= continuity_threshold) {
                // 发现不连续点，返回该行号
                return i;
            }
        }
    }
    
    // 边线连续，无断点
    return -1;
}
