#ifndef __MODEL_CALIBRATION_H__
#define __MODEL_CALIBRATION_H__

#include "vision_model_queue.h"

// 标定状态枚举
enum class CalibState {
    idle = 0,           // 空闲模式
    editing = 1,        // 编辑模式（添加/删除动作）
    saving = 2          // 保存模式
};

class ModelCalibration {
public:
    ModelCalibration();
    ~ModelCalibration() = default;

    CalibState current_state = CalibState::idle;
    Queue action_queue;  // 动作队列
    
    // 动作触发标志（供控制模块读取）
    bool action_triggered = false;                  // 是否触发了新动作
    ActionType current_action = ACTION_STRAIGHT;    // 当前要执行的动作
    bool queue_trigger_enable = false;              // 队列非空时启用菜单队列触发

    // 菜单接口
    void start_calibration(void);       // 开始标定
    void add_action(ActionType action); // 添加动作到队列
    void delete_last_action(void);      // 删除队列最后一个元素
    void save_and_exit(void);           // 保存队列并退出
    
    // 主循环（放在菜单回调中）
    void calibration_main(void);

    // 按键处理
    void key_enter_handler(void);
    void key_quit_handler(void);
    void key_up_handler(void);
    void key_down_handler(void);
    void key_select_handler(void);

    // 文件操作
    bool save_to_file(const std::string& filename);
    bool load_from_file(const std::string& filename);
    bool save_saved_queue(void);
    bool load_saved_queue(void);
    bool has_saved_queue(void) const { return saved_queue_available; }

    // 队列操作
    bool pop_next_action(ActionType* action);
    int queue_size(void) const;
    void refresh_queue_trigger_enable(void);

    // 辅助函数
    void display_queue(void);                               // 显示当前队列
    std::string action_to_string(ActionType action);        // 动作类型转字符串
    const char* action_to_ascii(ActionType action) const;   // 动作类型转ASCII字符

private:
    bool saved_queue_available = false;
};

extern ModelCalibration model_calib;

#endif // __MODEL_CALIBRATION_H__
