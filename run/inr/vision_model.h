#ifndef VISION_MODEL_H_
#define VISION_MODEL_H_

#include "vision_model_queue.h"

#define  MODEL_TOP   8
#define  MODEL_BOTTOM   50
#define  MODEL_HEIGHT  (MODEL_BOTTOM - MODEL_TOP)      // 有效区域高度 42

// 模型检测结构体
struct ModelResult {
    std::string class_name;      // 类别名称
    float probability;           // 置信度
    bool is_detected;            // 是否检测到物体
    cv::Rect box;                // 边界框
    // float class_scores[3] = {0.0f, 0.0f, 0.0f}; // softmax概率分布
    // int class_count = 0;         // 类别数量
    float class_scores[3]; // supplies, transport, weapon softmax
    int class_count;

    ModelResult()
        : class_name(""),
          probability(0.0f),
          is_detected(false),
          box(cv::Rect(0, 0, 0, 0)),
          class_scores{0.0f, 0.0f, 0.0f},
          class_count(0) {}

    ModelResult(const std::string& name, float prob, bool detected, const cv::Rect& rect)
        : class_name(name),
          probability(prob),
          is_detected(detected),
          box(rect),
          class_scores{0.0f, 0.0f, 0.0f},
          class_count(0) {}
};

// 模型距离划分
enum class BoardDistanceLevel {
    none,
    far,
    mid,
    near
};

// 模型中距离识别到的ROI来源
enum class ModelRoiSource {
    none,
    red_marker,
    loose_marker
};

// 红框检测等级       warning: 中远距离预警；strict: 中距离严格红框
enum class ModelMarkerLevel {
    none,
    warning,
    strict
};

// 模型图传显示不同框枚举
enum class MarkerVisualType {
    none,
    queue_marker,
    model_marker,
    large_supplies,
    loose_warning
};

// 红框检测来源
enum class MarkerDetectSource {
    none,
    hsv,
    ycrcb,
    yuv
};

// 模型识别状态机
enum class ModelTargetState {
    normal,              // 正常状态，等待检测到目标板 占位符
    warning,             // 警告状态，检测到宽松红框
    recognizing,         // 识别状态，正在通过模型识别目标板
    passing,             // 绕行状态，目标板识别成功 正在绕行
    returning,           // 回线状态，绕行结束　正在回线
    fail_safe            // 安全模式，检测到目标板但未通过识别
};

// 宽松颜色候选结构体
struct TargetBoardCandidate {
    bool valid = false;                             // 是否有效候选
    cv::Rect track_rect = cv::Rect(0, 0, 0, 0);  
    cv::Rect full_rect = cv::Rect(0, 0, 0, 0);     
    int center_x = 0;
    int center_y = 0;
    int bottom_y = 0;
    int stable_rows = 0;                           // 稳定行数
    float score = 0.0f;
    BoardDistanceLevel distance = BoardDistanceLevel::none;
    MarkerDetectSource source = MarkerDetectSource::none;
};

// 模型运行流程状态
struct ModelActionResult {
    bool queue_mode = false;           // 本帧结果是否来自菜单动作队列路径
    MarkerVisualType marker_visual_type = MarkerVisualType::none;
    bool marker_detected = false;      // 本帧是否检测到红色提示矩形
    bool board_detected = false;       // 本帧是否检测到目标板
    bool marker_confirmed = false;     // 红色提示矩形是否已经连续稳定
    bool model_ran = false;            // 本帧是否真正执行了 ncnn 推理
    bool action_confirmed = false;     // 动作是否经过多帧投票确认
    ActionType action = ACTION_STRAIGHT;
    ModelResult model = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
    cv::Rect marker_box;                // 红色提示矩形框位置
    cv::Rect board_box;                 // 目标板边界框位置 
    cv::Rect roi_box;                   // 目标板 ROI 位置
    BoardDistanceLevel distance = BoardDistanceLevel::none;
    ModelRoiSource roi_source = ModelRoiSource::none;
    MarkerDetectSource marker_source = MarkerDetectSource::none;
    float board_score = 0.0f;
    int marker_count = 0;
    int board_count = 0;
    int vote_count = 0;
    int cooldown_count = 0;               // 确认目标板后的冷却帧数
    int marker_method = 0;                // 0=HSV, 1=YCrCb, 2=YUV
    double marker_ms = 0.0;               // red marker detect cost
    double infer_ms = 0.0;                // ncnn inference cost
    double model_total_ms = 0.0;          // model path total cost
};

class ModelDetector {
public:
    ModelDetector() = default;
    ~ModelDetector() = default;

    // 初始化模型 加载模型参数和模型权重 成功返回 true
    bool init(const std::string& param_path, const std::string& bin_path);

    // 检查模型是否准备好
    bool is_ready() const { return model_ready;}

    // 模型推理主函数 用于处理图像帧 并显示检测结果
    void model_main(cv::Mat &frame);

    // 模型处理函数，返回检测结果
    ModelResult process_model(cv::Mat& frame);

    // 菜单动作队列触发路径
    ModelActionResult process_action_queue_trigger(cv::Mat& frame);

    // 正式模型优先路线：
    // 每帧检测红色提示矩形；红框连续稳定后，每帧运行 ncnn，直到动作投票确认
    ModelActionResult process_model_trigger(cv::Mat& frame);

    // 调试测试　　只输出本帧疑似框子/红框/ROI/模型结果，不进入冷却，也不修改控制动作
    ModelActionResult process_model_debug_frame(cv::Mat& frame);

    // 辅助：判断色块是否在赛道边界内
    // left_limit, right_limit 是当前行(y)对应的赛道左右边界x坐标
    bool is_within_lane(int cx, int cy, const cv::Size& frame_size);

    // 辅助：安全扩大 ROI
    cv::Rect get_safe_roi(const cv::Rect& box, int padding, int w, int h);

    // 辅助：寻找红色矩形标志；返回 warning/strict 等级
    ModelMarkerLevel find_red_marker(cv::Mat& frame, TargetBoardCandidate& candidate);
    bool find_red_marker(cv::Mat& frame, cv::Rect& target_rect);
    bool find_large_red_supplies(cv::Mat& frame, cv::Rect& target_rect);
    bool build_target_roi(const cv::Rect& marker_rect, const cv::Size& frame_size, cv::Rect& roi_rect) const; // 根据红框裁剪包含目标物和红框的 ROI
    ActionType class_to_action(const std::string& class_name) const; // 模型类别映射成左绕/右绕/直行

    ModelResult last_result = {"", 0.0f, false, cv::Rect(0, 0, 0, 0)};
    bool is_target_warning_active() const { return target_warning_active; }
    ModelTargetState get_target_state() const { return target_state; }
    bool has_loose_warning_before_strict() const { return loose_warning_confirmed; }

    ModelTargetState target_state = ModelTargetState::normal;   //目标板识别子状态

    float model_return_distance = 3.3f;  //　回线距离

private:
    void update_long_midline(const ModelActionResult& status);
    int long_center_at(int track_y, int fallback_center) const;
    ModelMarkerLevel find_ycrcb_marker(cv::Mat& frame, TargetBoardCandidate& candidate);
    ModelMarkerLevel find_yuv_marker(cv::Mat& frame, TargetBoardCandidate& candidate);
    bool find_loose_marker_candidate(cv::Mat& frame, TargetBoardCandidate& candidate);
    bool is_reliable_loose_row(int track_y, int* trusted_center = nullptr) const;
    bool build_target_roi_from_candidate(const TargetBoardCandidate& candidate, const cv::Size& frame_size, cv::Rect& roi_rect) const;
    BoardDistanceLevel calc_board_distance(int bottom_y) const;
    const char* board_distance_to_text(BoardDistanceLevel level) const;
    const char* roi_source_to_text(ModelRoiSource source) const;
    ModelResult infer_roi(cv::Mat& frame, const cv::Rect& roi_rect);
    void display_model_upimage(cv::Mat& frame, const ModelActionResult& status);
    void cache_model_debug_display(const ModelActionResult& status, const cv::Mat& frame, const cv::Rect& roi_rect);


    // 状态记忆变量
    std::string last_class_name = ""; // 上一帧识别的类别
    int same_class_counter = 0;       // 连续识别为同一物体的计数器
    const int CONFIRM_THRESHOLD = 2;  // 连续识别为同一物体的计数器阈值
    bool model_ready = false;         // 模型是否初始化成功

    // 目标队列变量
    int target_visible_count = 0;     // 目标确认计数器
    int target_cooldown_count = 0;    // 目标冷却计数器（避免同一红框连续触发）
    
    int board_visible_count = 0;      // 目标板确认计数器（连续稳定出现才认为真正看到目标板，避免误触）
    cv::Rect last_board_track_rect = cv::Rect(0, 0, 0, 0);
    TargetBoardCandidate locked_loose_candidate;  // 远距离锁定的宽松候选，给中距离短暂丢框时兜底
    int locked_loose_age = 0;                     // 锁定候选剩余有效帧数
    bool target_warning_active = false;           // 目标板检测成功　宽松/严格红框都算
    int target_warning_count = 0;                 // 宽松红框连续确认帧
    int target_warning_keep_count = 0;            // 宽松红框存在确认帧
    float target_warning_start_distance = 0.0f;   // 宽松红框距离变化阈值
    bool loose_warning_confirmed = false;         // 宽松红框识别成功标志位
    int memory_mid_line[CAM_HEIGHT] = {0};         // 历史中线，只给目标板/红框可信判断使用
    bool memory_mid_valid[CAM_HEIGHT] = {false};   // 历史中线有效标志

    ActionType last_action = ACTION_STRAIGHT;      // 上一次投票动作
    int action_vote_count = 0;                     // 当前动作连续出现次数
    bool last_action_printed = false;              // 防止同一目标板反复刷串口

    static const int MARKER_DISPLAY_FRAMES = 30;    // 模型图传显示框子显示帧数
    cv::Rect display_marker_box = cv::Rect(0, 0, 0, 0);
    MarkerVisualType display_marker_type = MarkerVisualType::none;
    std::string display_marker_label;
    int display_marker_frames = 0;
    cv::Rect display_roi_box = cv::Rect(0, 0, 0, 0);
    int display_roi_frames = 0;
    cv::Size display_frame_size = cv::Size(0, 0);
    float display_model_scores[3] = {0.0f, 0.0f, 0.0f};
    float display_model_margin = 0.0f;
    std::string display_model_top;
    cv::Mat display_roi_preview;
    int display_model_debug_frames = 0;
    bool display_model_debug_ran = false;
    double display_model_infer_ms = 0.0;
    int display_model_confirm_count = 0;
    int display_model_vote_count = 0;

    // 模型参数变量
    const int MARKER_CONFIRM_THRESHOLD = 1;        // 红色提示矩形连续稳定出现才算真正看到 红框识别很死　一帧足够
    const int BOARD_CONFIRM_THRESHOLD = 1;         // 目标板连续稳定出现才算真正看到，避免误触
    const int ACTION_CONFIRM_THRESHOLD = 1;        // 模型动作连续 3 次才确认
    const int TARGET_COOLDOWN_FRAMES = 3;         // 确认后冷却，避免同一目标板重复触发
    const int WARNING_CONFIRM_FRAMES = 2;          // 宽松红框连续确认帧
    const int WARNING_KEEP_FRAMES = 18;            // 宽松红框如果在这个帧数内没有再次看到认为误检　取消减速状态
    const float WARNING_MAX_DISTANCE_CM = 15.0f;   // 宽松红框如果距离变化过大认为是误检　取消减速状态
    const int MODEL_ACTION_DEADLINE_FULL_Y = 158;  // 模型识别终止线 同中近距离分界线　8cm　　待调　估计变大
     
    // NCNN 网络模型变量
    ncnn::Net net;
    const int input_size = 64;        // 模型输入尺寸 
    std::vector<const char*> class_names = {"supplies", "transport", "weapon"}; // 类别名称列表 (与pytorch 模型中的保持一致)
};

extern ModelDetector model_detector;

#endif /* VISION_MODEL_H_ */
