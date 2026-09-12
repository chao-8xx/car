#ifndef __BUZZER_REMIND_H
#define __BUZZER_REMIND_H
#include "headfile.h"

class Remind{
public:

/***********************************************蜂鸣器提醒***********************************************/

void buzzer_init(void);

void buzzer_enable_on(void);
void buzzer_enable_off(void);

void buzzer_count_on(void);
void buzzer_count_off(void);

void buzzer_on(void);
void buzzer_off(void);

void buzzer_sound_time(int ms);
void buzzer_single_remind(void);
void buzzer_work_control(int t);

void remind_on(void);
void remind_off(void);

void buzzer_flash(int period);   //蜂鸣器连续提醒


/***********************************************屏幕提醒***********************************************/

void lcd_remind(rgb565_color_enum rgb_color);   //屏幕提醒

bool is_screen_remind = false;   //是否启动屏幕提醒
bool is_screen_clear = false;    //是否启动清屏

private:

};

extern Remind remind;

#endif



