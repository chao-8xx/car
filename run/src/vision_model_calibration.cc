#include "headfile.h"

// 模型/目标板动作标注模块
// 这个文件目前只负责“菜单里录入动作队列”和“被红框触发时弹出动作”

extern LCD lcd;
extern void key_exit(void);

// 模型菜单标注存放文件位置
namespace {
const char* kModelActionQueueFile = "model_action_queue.json";
}


// 固定数组队列没有构造函数，所以全局对象创建时手动清空一次
ModelCalibration::ModelCalibration()
{
    queue_manager.queue_init(&action_queue);
    refresh_queue_trigger_enable();
}

// 菜单标注主循环 未正常使用　　在菜单里的　model_action_queue_menu　此函数才是菜单标注主循环
void ModelCalibration::calibration_main(void)
{
    if (current_state != CalibState::editing) return;
    display_queue();
}

//////////////////////////////////////// 菜单接口 ////////////////////////////////

// 开始标定模式，清空队列并进入编辑状态  (可以只进入不标定　自动使用模型法)
void ModelCalibration::start_calibration(void)
{
    // 初始化队列，清空之前的动作
    current_state = CalibState::editing;
    action_triggered = false;
    current_action = ACTION_STRAIGHT;
    queue_manager.queue_init(&action_queue);
    refresh_queue_trigger_enable();
    save_saved_queue();

    std::cout << "[模型标定] 动作序列标定模式启动！" << std::endl;
    std::cout << "[ModelCalib] UP=LEFT, ENTER=STRAIGHT, DOWN=RIGHT, SELECT=DELETE, QUIT=SAVE" << std::endl;
    display_queue();
}

// 添加动作到队列
void ModelCalibration::add_action(ActionType action)
{
    // 只有在编辑模式下才允许入队，避免普通菜单误触
    if (current_state != CalibState::editing) return;

    // 队列存的是“动作”，不是目标类别
    // 这样控制侧只需要关心左/直/右，不需要再关心武器/物资/交通工具。
    if (queue_manager.queue_in(&action_queue, action)) {
        std::cout << "[模型标定] add " << action_to_ascii(action)
                  << ", size=" << queue_size() << std::endl;
    } 
    else {
        std::cout << "[模型标定] 队列已满，无法添加！" << std::endl;
    }
    refresh_queue_trigger_enable();
    save_saved_queue();
    display_queue();
}

// 删除队列最后一个元素
void ModelCalibration::delete_last_action(void)
{
    // 菜单录入时按错了，可以删除最后一个动作。
    // 这里删的是队尾，不影响已经确认的前面动作顺序。
    if (current_state != CalibState::editing) return;

    if (action_queue.rear > action_queue.front) {
        action_queue.rear--;
        std::cout << "[模型标定] 删除最后一个动作, size=" << queue_size() << std::endl;
    } 
    else {
        std::cout << "[模型标定] 队列为空，无法删除！" << std::endl;
    }
    refresh_queue_trigger_enable();
    save_saved_queue();
    display_queue();
}

// 保存队列并退出
void ModelCalibration::save_and_exit(void)
{
    // 退出标注模式时保存一份 JSON，方便之后复盘/加载
    // 注意：保存后内存里的队列不会被清空，马上跑车仍然能用 适合比赛时不更换目标板的情况
    if (current_state != CalibState::editing) return;

    current_state = CalibState::saving;
    
    // 保存到文件
    refresh_queue_trigger_enable();
    if (save_saved_queue()) {
        std::cout << "[模型标定] saved model_action_queue.json" << std::endl;
        lcd.clearScreen();
        lcd.showString(5, 60, "Save Success!");
    } 
    else {
        std::cout << "[模型标定] 保存失败" << std::endl;
        lcd.clearScreen();
        lcd.showString(5, 60, "Save Failed");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    current_state = CalibState::idle;
    key_exit();  // 退出到菜单界面
}


//////////////////////////////////////////////// 按键处理 ////////////////////////////////
void ModelCalibration::key_enter_handler(void)
{
    // 在“模型队列”执行界面中，确认键录入“直行”
    add_action(ACTION_STRAIGHT);
}

void ModelCalibration::key_quit_handler(void)
{
    // 退出键：保存队列并回到普通菜单
    save_and_exit();
}

void ModelCalibration::key_up_handler(void)
{
    // 上键：录入“左绕”
    add_action(ACTION_LEFT);
}

void ModelCalibration::key_down_handler(void)
{
    // 下键：录入“右绕”
    add_action(ACTION_RIGHT);
}

void ModelCalibration::key_select_handler(void)
{
    // 选择键：删除最后一个录入动作
    delete_last_action();
}


//////////////////////////////////////////////// 文件操作 ////////////////////////////////
// 保存队列到文件
bool ModelCalibration::save_to_file(const std::string& filename)
{
    // 保存格式：
    // {
    //   "version": "1.0",
    //   "queue_size": 3,
    //   "action_queue": [{"index":0,"action":"left"}, ...]
    // }
    // 用字符串保存而不是数字，是为了现场查看文件时更直观
    Json::Value root;
    root["version"] = "1.0";
    root["queue_size"] = queue_size();

    Json::Value actions(Json::arrayValue);
    for (int i = action_queue.front; i < action_queue.rear; i++) {
        Json::Value item;
        item["index"] = i - action_queue.front;

        ActionType action = static_cast<ActionType>(action_queue.data[i]);
        switch (action) {
            case ACTION_LEFT: item["action"] = "left"; break;
            case ACTION_RIGHT: item["action"] = "right"; break;
            case ACTION_STRAIGHT:
            default: item["action"] = "straight"; break;
        }
        actions.append(item);
    }
    root["action_queue"] = actions;

    const std::string temp_filename = filename + ".tmp";
    std::ofstream file(temp_filename);
    if (!file.is_open()) {
        std::cerr << "[ModelCalib] cannot open " << temp_filename << std::endl;
        return false;
    }

    // 使用 Json::StyledWriter 生成格式化 JSON 文本
    Json::StyledWriter writer;
    file << writer.write(root);
    file.flush();
    if (!file.good()) {
        file.close();
        std::remove(temp_filename.c_str());
        return false;
    }
    file.close();

    // 保存成功后，删除临时文件，替换原文件
    if (std::rename(temp_filename.c_str(), filename.c_str()) != 0) {
        std::remove(filename.c_str());
        if (std::rename(temp_filename.c_str(), filename.c_str()) != 0) {
            std::remove(temp_filename.c_str());
            std::cerr << "[ModelCalib] cannot replace " << filename << std::endl;
            return false;
        }
    }
    return true;
}

// 从文件加载队列
bool ModelCalibration::load_from_file(const std::string& filename)
{
    const auto reset_runtime_queue = [this]() {
        queue_manager.queue_init(&action_queue);
        current_action = ACTION_STRAIGHT;
        action_triggered = false;
        refresh_queue_trigger_enable();
    };

    // 从 JSON 恢复动作队列。当前菜单第一版主要用“现场录入”，
    // 但这个接口保留着，后面可以加“加载队列”菜单项。
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ModelCalib] cannot open " << filename << std::endl;
        reset_runtime_queue();
        return false;
    }

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(file, root)) {
        std::cerr << "[ModelCalib] json parse failed: " << filename << std::endl;
        reset_runtime_queue();
        return false;
    }

    // 检查 JSON 格式是否正确
    const Json::Value actions = root["action_queue"];
    if (!actions.isArray()) {
        std::cerr << "[ModelCalib] invalid action_queue: " << filename << std::endl;
        reset_runtime_queue();
        return false;
    }
    if (actions.size() > MAX_SIZE) {
        std::cerr << "[ModelCalib] action queue too large: " << actions.size() << std::endl;
        reset_runtime_queue();
        return false;
    }

    // 检查队列大小是否超过最大限制
    Queue loaded_queue;
    queue_manager.queue_init(&loaded_queue);
    for (unsigned int i = 0; i < actions.size(); i++) {
        const std::string action_str = actions[i]["action"].asString();
        ActionType loaded_action = ACTION_STRAIGHT;
        if (action_str == "left") {
            loaded_action = ACTION_LEFT;
        } 
        else if (action_str == "right") {
            loaded_action = ACTION_RIGHT;
        } 
        else if (action_str == "straight") {
            loaded_action = ACTION_STRAIGHT;
        } 
        else {
            std::cerr << "[ModelCalib] invalid action at index " << i << std::endl;
            reset_runtime_queue();
            return false;
        }
        if (!queue_manager.queue_in(&loaded_queue, loaded_action)) {
            reset_runtime_queue();
            return false;
        }
    }

    action_queue = loaded_queue;
    current_action = ACTION_STRAIGHT;
    action_triggered = false;
    refresh_queue_trigger_enable();

    std::cout << "[ModelCalib] loaded " << queue_size()
              << " actions from " << filename
              << ", queue_trigger_enable=" << queue_trigger_enable
              << std::endl;
    return true;
}

// 保存当前队列到文件
bool ModelCalibration::save_saved_queue(void)
{
    const bool saved = save_to_file(kModelActionQueueFile);
    if (saved) saved_queue_available = queue_size() > 0;
    return saved;
}

// 从文件加载队列
bool ModelCalibration::load_saved_queue(void)
{
    const bool loaded = load_from_file(kModelActionQueueFile);
    saved_queue_available = loaded && queue_size() > 0;
    return loaded;
}


////////////////////////////////////////////////// 队列操作 ////////////////////////////////

// 弹出队头动作，返回是否成功
bool ModelCalibration::pop_next_action(ActionType* action)
{
    // 红框确认触发后调用这个函数
    // 成功：从队头弹出一个动作，并返回 true
    // 失败：队列为空，只返回 false，不生成默认动作
    if (action == nullptr) return false;

    if (!queue_manager.queue_out(&action_queue, action)) {
        action_triggered = false;
        refresh_queue_trigger_enable();
        return false;
    }

    current_action = *action;
    action_triggered = true;
    refresh_queue_trigger_enable();
    return true;
}

// 获取队列大小
int ModelCalibration::queue_size(void) const
{
    // 队列是简单顺序队列，rear - front 就是当前剩余动作数
    return action_queue.rear - action_queue.front;
}

// 刷新队列触发状态   当队列不为空时，红框确认触发后会弹出动作
void ModelCalibration::refresh_queue_trigger_enable(void)
{
    queue_trigger_enable = queue_size() > 0;
}


////////////////////////////////////////// 辅助函数 ////////////////////////////////////
// 显示当前队列
void ModelCalibration::display_queue(void)
{
    // LCD 只显示队列前几个动作，避免屏幕放不下
    // 串口会打印完整的入队/出队信息，调试时主要看串口
    lcd.clearScreen();
    lcd.showString(5, 5, "Action Queue");

    char buf[64];
    std::snprintf(buf, sizeof(buf), "Size:%d/%d", queue_size(), MAX_SIZE);
    lcd.showString(5, 25, buf);

    // 显示队列内容（最多显示6个）
    int display_count = std::min(6, queue_size());
    for (int i = 0; i < display_count; i++) {
        int idx = action_queue.front + i;
        ActionType action = static_cast<ActionType>(action_queue.data[idx]);

        std::snprintf(buf, sizeof(buf), "%d:%s", i + 1, action_to_ascii(action));

        lcd.showString(5, 45 + i * 15, buf);
    }

    
    lcd.showString(5, 140, "U:L E:S D:R");
    lcd.showString(5, 155, "Sel:Del Q:Save");
}

// 动作类型转字符串
std::string ModelCalibration::action_to_string(ActionType action)
{
    // 保留 std::string 版本，兼容旧代码里可能直接拿字符串打印的地方。
    return action_to_ascii(action);
}

// 动作类型转 ASCII 字符串
const char* ModelCalibration::action_to_ascii(ActionType action) const
{
    // LCD/串口都先用 ASCII，避免某些字体或编码环境下中文显示乱码。
    switch (action) {
        case ACTION_LEFT: return "LEFT";
        case ACTION_RIGHT: return "RIGHT";
        case ACTION_STRAIGHT:
        default: return "STRAIGHT";
    }
}

ModelCalibration model_calib;
