#ifndef __VISION_BIAODING_AUTO_H__
#define __VISION_BIAODING_AUTO_H__

#include "headfile.h"

enum class BiaoDingState{
    idle = 0,           // 空闲模式
    show = 1,           // 显示模式 实时显示画面等待按键
    biaoding = 2        // 标定模式 计算半宽并直接输出
};


class BiaoDingAuto
{
public:
    BiaoDingAuto() = default;
    ~BiaoDingAuto() = default;        
        
    BiaoDingState current_state = BiaoDingState::idle; // 当前状态

    void start_biaoding(void);      // 开始标定
    void key_enter_handler(void);   // 按键确定回调函数
    void key_quit_handler(void);    // 按键取消回调函数
    void biaoding_main(const cv::Mat &frame);           // 主循环 状态机 (放在摄像头 while 循环内) 


    void set_circle_slot_type(int index, int type);
    const char* circle_type_name(int type);
    void show_circle_label_menu(void);    
    void finish_circle_label_menu(void);
    void circle_label_push_type(int type);
    // void circle_label_menu(void);

private:

};

extern BiaoDingAuto biaoding_auto;
extern bool circle_label_active;
extern int circle_label_index;

#endif // __VISION_BIAODING_AUTO_H__


