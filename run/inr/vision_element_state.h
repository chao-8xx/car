#ifndef ELEMENT_STATE_H
#define ELEMENT_STATE_H

#include "headfile.h"

// 参数宏定义
#define   TOF_MIN_DIS               10
#define   TOF_MAX_DIS               1000

#define   MID_OFFSET                10


// // 十字路口模式选择
// #define CROSSROAD_VISION        1            // 纯视觉模式
// #define CROSSROAD_ENCODER       2            // 编码器模式
// #define CROSSROAD_MODE          1            // 当前十字内部所使用模式

// 赛道大类元素枚举
enum class TrackType {
    normal,                   // 常规道路 (不再区分直道和弯道)
    crossroad,                // 十字
    circle,                   // 环岛
    zebra,                    // 斑马线
    ramp,                     // 坡道
    barrier,                  // 路障
    model,                    // 模型
    // out_of_bounds             // 出界
};

// 常规类型枚举
enum class NormalType {
    straight,                 // 直道
    curve                     // 弯道
};

// 常规道路类型枚举
enum class NormalRoadState {
    curve,                    // 弯道
    safe,                     // 安全
    straight_fast             // 直道加速
};

// 圆环方向枚举
enum class CircleDir {
    none,
    left,
    right
};

// 圆环状态机枚举 (6状态：弱入环+陀螺仪混合方案)
enum class CircleState {
    none,
    weak_approach,            // 弱入环 — 检测到拐点特征但未确认，准备状态
    approach,                 // 确认入环 — 拐点+边界靠边确认，清零陀螺仪
    inside,                   // 环内 — 固定拉线，陀螺仪判断转过270度
    exit,                     // 出环 — 固定拉线引导出口，陀螺仪判断转过360度
    leave                     // 出环直行 — 单边巡线，里程计判断行驶200cm
};

// 十字路口类型枚举
enum class CrossroadType {
    none,
    straight,                 // 正入十字 
    left_diagonal,            // 左斜入 
    right_diagonal            // 右斜入 
};

enum class CrossroadState {
    none,                     // 非十字
    approach,                 // 准备进入十字 (特征识别)
    inside,                   // 十字前半段 (有上角点)
    // pass                      // 十字后半段 (出十字)
};

// 路障方向枚举
enum class BarrierDir {
    none,
    left,
    right
};

// 路障状态机枚举
enum class BarrierState {
    none,
    detected,                 // 发现路障
    avoiding,                 // 正在避障 (单边巡线)
};

// 贴边线绕行方向枚举
enum class ModelLinePassDir {
    none,
    left,
    right
};

// 斑马线状态机枚举
enum class ZebraState {
    none,                     // 未检测到斑马线
    shielded,                 // 斑马线被屏蔽
    detected                  // 检测到斑马线
};                                

// 坡道状态机枚举
enum class RampState {
    none,
    detected,                 // 检测到坡道 (准备微踩刹车防飞坡)
    climbing,                 // 正在上坡 (开环/屏蔽上视野，防止被天花板干扰)
    ending                    // 坡道结束
};

// 元素状态类
class ElementState {
public:
    ElementState() = default;
    ~ElementState() = default;

    // 拐点约束配置结构体
    struct Limit {
        bool enable_global_search;      // 是否启用全局搜索
        int search_y_min;               // Y轴搜索范围最小值
        int search_y_max;               // Y轴搜索范围最大值
        int down_up_min_gap;            // 下拐点与上拐点最小间距
        bool lock_down_pt;              // 锁定下拐点（不更新）
        bool lock_up_pt;                // 锁定上拐点（不更新）
        int max_lock_frames;            // 最大锁定帧数（超时保护）
        
        // 空间约束字段
        bool enable_spatial_constraint; // 是否启用空间约束（默认false，保持向后兼容）
        
        // 拐点类型需求标志
        bool need_down_lp;              // 是否需要检测下拐点（默认true）
        bool need_mid_lp;               // 是否需要检测中拐点（默认true）
        bool need_up_lp;                // 是否需要检测上拐点（默认true）
        
        // 下拐点矩形搜索区域
        int down_x_min;                 // 下拐点X坐标最小值
        int down_x_max;                 // 下拐点X坐标最大值
        int down_y_min;                 // 下拐点Y坐标最小值
        int down_y_max;                 // 下拐点Y坐标最大值
        
        // 中拐点矩形搜索区域
        int mid_x_min;                  // 中拐点X坐标最小值
        int mid_x_max;                  // 中拐点X坐标最大值
        int mid_y_min;                  // 中拐点Y坐标最小值
        int mid_y_max;                  // 中拐点Y坐标最大值
        
        // 上拐点矩形搜索区域
        int up_x_min;                   // 上拐点X坐标最小值
        int up_x_max;                   // 上拐点X坐标最大值
        int up_y_min;                   // 上拐点Y坐标最小值
        int up_y_max;                   // 上拐点Y坐标最大值
        
        // 阈值调整
        float dx_threshold_scale;       // dx突变阈值缩放因子（默认1.0）
    };

    // 赛道当前主状态
    TrackType current_track_type;
    uint16_t current_tof_dist; // 添加保存一份TOF数据用于调试显示
    TrackMode track_mode;      // 当前巡线模式 (由元素状态机设置，供 process_midline 使用)

    // ==================== 变量 ====================
    // 圆环
    CircleState circle_state;
    CircleDir circle_dir;
    int approach_confirm_cnt;      // 入环确认计数器 (弱入环阶段)
    int circle_confirm_count;          // 检测防抖计数
    int circle_pass_count;             // 状态内帧计数 (超时保护)
    bool circle_seen_bottom_lost;       // 弱入环阶段：是否已经看到环侧丢线压到底部
    // bool circle_approach_seen_other_unstable; // 入环阶段：直道侧是否经历过变差
    cv::Point fixed_C_pt;              // 固定的环侧锚点 (下拐点) (出环点)
    // cv::Point last_circle_up_pt;        // 上拐点历史追踪 (环内稳定性判断)
    // int circle_up_stable_count;      // 上拐点稳定计数 (环内稳定性判断)
    float circle_exit_start_yaw;        // 出环起始偏航角 (陀螺仪方案)

    int circle_run_index;               // 圆环索引
    int circle_exit_confirm_count;      // 出环确认计数器 (陀螺仪方案)
    int circle_leave_confirm_count;      // 出环确认计数器 (陀螺仪方案)
    cv::Point circle_inside_bottom_anchor;  // 环内固定底部锚点 (环内状态下的稳定参考点)

    // 十字路口
    CrossroadState crossroad_state;
    CrossroadType crossroad_type;
    int crossroad_confirm_count;
    int crossroad_pass_count;
    bool crossroad_inside_anchor_locked;        // 十字锚点锁定状态
    cv::Point crossroad_left_bottom_anchor;     // 左下锚点
    cv::Point crossroad_right_bottom_anchor;    // 右下锚点
    float crossroad_left_quality;               // 左下边线质量
    float crossroad_right_quality;              // 右下边线质量
    // int crossroad_cooldown_count;      // 十字冷却计数器
    float crossroad_last_error;       // 十字状态下的上一帧误差
    // int crossroad_distance;           // 十字距离 (像素) 编码器计算
    // float locked_Lk, locked_Lb;    // 缓存的左线趋势
    // float locked_Rk, locked_Rb;    // 缓存的右线趋势
    // bool locked_L_valid;           // 左线缓存是否有效
    // bool locked_R_valid;           // 右线缓存是否有效
    // cv::Point fixed_L_pt;          // 固定的左线锚点 (上拐点或下拐点)
    // cv::Point fixed_R_pt;          // 固定的右线锚点

    // 路障
    BarrierState barrier_state;
    BarrierDir barrier_dir;
    int barrier_confirm_count;
    int barrier_offset;          // 中线偏移量 (像素)

    bool model_line_pass_active = false;              // 模型贴边线绕行触发
    ModelLinePassDir model_line_pass_dir = ModelLinePassDir::none;  // 模型贴边线绕行方向
    float model_line_pass_distance = 8.5f;           // 编码器退出距离

    // 坡道 (IMU版本  待更新tof版本)
    RampState ramp_state;
    int ramp_circle_shield_count;       // 坡道内圆环屏蔽计数器
    int ramp_up_confirm_count;          // 上坡触发确认计数
    int ramp_down_confirm_count;        // 下坡触发确认计数
    int ramp_flat_confirm_count;        // 回平退出确认计数
    
    // 常规道路类型
    NormalType normal_type;
    NormalRoadState normal_road_state; // 常规道路状态划分
    float curve_score;                 // 常规道路弯道强度
    
    // 直弯道切换防抖计数器
    int straight_confirm_count;    // 直道确认计数
    int curve_confirm_count;       // 弯道确认计数
    
    // 斜率历史记录（用于突变检测）
    float last_left_k;
    float last_right_k;
    bool k_init;
    
    // 全局控制建议变量
    // int long_straight_count;       // 用于长直道判定的防抖计数器

    // 斑马线
    ZebraState zebra_state;
    int zebra_confirm_count;       // 连续检测确认计数器 (防抖)
    bool is_shielded;              // 是否处于屏蔽状态 (起步屏蔽)

    // ==================== 核心处理接口 ====================
    // 元素识别初始化函数，在程序启动时调用一次
    void element_init(void);
    // 元素识别主函数，每帧调用。作为完整的状态机整合所有特殊元素、长直道和弯道
    void elements_process(const cv::Mat& bin_buf, const cv::Mat& rgb_img, uint16_t tof_distance);

    // // 当前帧统一特征缓存
    // PointState::FeatureSet left_features;
    // PointState::FeatureSet right_features;

    // 根据当前元素状态返回拐点约束配置
    Limit get_corner_limit(bool is_left);


    // ==================== 具体元素检测函数 ====================
    // 圆环
    void circle_process(const cv::Mat& bin_buf);
    bool circle_entry_allowed();                  // 圆环入口检测门控
    bool circle_detect_weak(CircleDir& dir);      // 弱入环检测（第一阶段）
    bool circle_detect_confirm();                  // 确认入环检测（第二阶段）

    // 十字路口
    void crossroad_process(const cv::Mat& bin_buf);
    bool crossroad_detect(CrossroadType &type);

    // 圆环辅助
    void create_fixed_line(bool is_left, int target_x, int target_y);

    // 路障
    void barrier_process(const cv::Mat& rgb_img);
    bool barrier_detect(const cv::Mat& rgb_img, BarrierDir& dir);

    // 坡道
    void ramp_process(void);
    bool ramp_detect(void);
    // bool ramp_detect(uint16_t tof_distance);

    // 斑马线
    void zebra_process(const cv::Mat& bin_buf);
    bool zebra_detect(const cv::Mat& bin_buf);
    void zebra_shield(float distance, float max_distance);

    // 模型绕行
    void start_model_line_pass(ModelLinePassDir dir, float distance = 11.2f);
    void stop_model_line_pass(void);
    void enter_model_context(void);      // 第一次进入模型大状态前保存外部元素快照
    void restore_model_context(void);    // returning 结束后恢复进入模型前的元素快照

    // 常规道路分类
    void classify_normal_road();
    bool detect_straight_shape();
    bool detect_straight();
    bool detect_curve();


  private:

    // 模型绕行状态前状态
    struct ModelLinePassSnapshot {
        TrackType current_track_type;
        TrackMode current_mode;

        CircleState circle_state;
        CircleDir circle_dir;
        int approach_confirm_cnt;
        int circle_confirm_count;
        int circle_pass_count;
        bool circle_seen_bottom_lost;
        cv::Point fixed_C_pt;
        float circle_exit_start_yaw;
        int circle_run_index;
        int circle_exit_confirm_count;
        int circle_leave_confirm_count;
        cv::Point circle_inside_bottom_anchor;

        CrossroadState crossroad_state;
        CrossroadType crossroad_type;
        int crossroad_confirm_count;
        int crossroad_pass_count;
        bool crossroad_inside_anchor_locked;
        cv::Point crossroad_left_bottom_anchor;
        cv::Point crossroad_right_bottom_anchor;
        float crossroad_left_quality;
        float crossroad_right_quality;
        float crossroad_last_error;

        BarrierState barrier_state;
        BarrierDir barrier_dir;
        int barrier_confirm_count;
        int barrier_offset;

        RampState ramp_state;
        int ramp_circle_shield_count;
        int ramp_up_confirm_count;
        int ramp_down_confirm_count;
        int ramp_flat_confirm_count;

        ZebraState zebra_state;
        int zebra_confirm_count;
        bool is_shielded;
        
        NormalType normal_type;
        NormalRoadState normal_road_state;
        int straight_confirm_count;
        int curve_confirm_count;
        float curve_score;
        float last_left_k;
        float last_right_k;
        bool k_init;
        float motor_distance;
    };

    ModelLinePassSnapshot model_line_pass_snapshot;
    bool model_line_pass_snapshot_valid = false;

    // 辅助函数  
    void connect_points(int *line_arr, const cv::Point &p1, const cv::Point &p2);   // 两点连线
    void extend_line_up(int *line_arr, const cv::Point &corner_pt, bool is_left);   // 从角点往上方延伸补线 
    void extend_line_down(int *line_arr, const cv::Point &corner_pt, bool is_left); // 从角点往下方延伸补线
    bool extend_line(int* boundary_line, bool is_up, bool is_left); // 最长连续线段补线（带视觉惯性）
    void reset_line_extension_memory(); // 重置补线记忆
    bool fix_crossroad_vertical_line(PointState::BoundaryData& boundary, bool is_left); // 修正十字内部垂直线段
    bool fix_circle_inside_line(PointState::BoundaryData& boundary, bool is_left); // 圆环内部最长可靠线段锁锚补线
    
    // int find_white_top_at_x(const cv::Mat &bin_buf, int x);         
    // int find_white_bottom_at_x(const cv::Mat &bin_buf, int x);       
    bool check_bottom_recovered();
    bool check_circle_bottom_recovered();
    int count_transitions(const cv::Mat& bin_buf, int y);

};
    // 值的限幅
    static inline int clip(int val, int min, int max) {
        return (val < min) ? min : ((val > max) ? max : val);
    }

extern ElementState element_state;

#endif
