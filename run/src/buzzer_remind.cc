#include "buzzer_remind.h"

//定义锁
std::mutex buzzer_work_mutex;        //蜂鸣器工作锁
std::mutex buzzer_start_mutex;       //蜂鸣器启停锁
std::mutex buzzer_enable_mutex;      //蜂鸣器使能锁
std::mutex buzzer_count_mutex;       //蜂鸣器工作计时锁

Buzzer buzzer0;
extern LCD lcd;

/***********************************************蜂鸣器提醒***********************************************/
//蜂鸣器初始化
void Remind::buzzer_init(void)
{
    buzzer0.buzzer_init();   //蜂鸣器初始化
    buzzer_enable_off();
    buzzer_off();
}

//蜂鸣器使能
void Remind::buzzer_enable_on(void)
{
    buzzer_enable_mutex.lock();
    buzzer_enable = true;
    buzzer_enable_mutex.unlock();
}

//蜂鸣器失能
void Remind::buzzer_enable_off(void)
{
    buzzer_enable_mutex.lock();
    buzzer_enable = false;
    buzzer_enable_mutex.unlock();    
}

//蜂鸣器工作计时开
void Remind::buzzer_count_on(void)
{
    buzzer_count_mutex.lock();
    buzzer_count_enable = true;
    buzzer_count_mutex.unlock();
}

//蜂鸣器工作计时关
void Remind::buzzer_count_off(void)
{
    buzzer_count_mutex.lock();
    buzzer_count_enable = false;
    buzzer_count_mutex.unlock();
}

//蜂鸣器开（给PWM）
void Remind::buzzer_on(void)
{
    buzzer_start_mutex.lock();
    buzzer0.set_duty_freq(90,820);
    buzzer_start_mutex.unlock();
}

//蜂鸣器关(PWM给0)
void Remind::buzzer_off(void)
{
    buzzer_start_mutex.lock();
    buzzer0.set_duty_freq(0, 0);
    buzzer_start_mutex.unlock();
}

//蜂鸣器鸣叫时间
void Remind::buzzer_sound_time(int ms)
{
    buzzer_work_mutex.lock();
    buzzer_work_time = ms/Ts;
    buzzer_count_on();
    buzzer_enable_on();
    buzzer_work_mutex.unlock();
}

//蜂鸣器单次鸣叫提醒
void Remind::buzzer_single_remind(void)
{
    //蜂鸣器使能，则启动
    if(! buzzer_enable) return;

    if(buzzer_enable && buzzer_count_enable && buzzer_work_time > 0)   //蜂鸣器单次工作倒计时
    {
        buzzer_work_mutex.lock();
        buzzer_work_time --;    //计时减
        buzzer_work_mutex.unlock();
    }
    else if(buzzer_enable && buzzer_work_time <= 0) //蜂鸣器结束单次工作倒计时
    {
        buzzer_work_mutex.lock();
        buzzer_enable_off();
        buzzer_work_mutex.unlock();
    }
    
}

//蜂鸣器工作控制
void Remind::buzzer_work_control(int t)    //单次鸣叫 x ms 
{
    //根据使能状态决定蜂鸣器是否鸣叫
    if(buzzer_enable) buzzer_on();
    else buzzer_off();
    
    if(! buzzer_count_enable)
    {
        buzzer_sound_time(t);
    }
    buzzer_single_remind();
}

//蜂鸣器启动鸣叫
void Remind::remind_on(void)
{
    buzzer_count_off(); //蜂鸣器工作计时关，便于重新使能启动
}

void Remind::remind_off(void)
{
    buzzer_count_on();  //蜂鸣器工作计时开，计数失能时会开启蜂鸣器工作和计时
}

/***********************************************屏幕提醒***********************************************/

//蜂鸣器连续提醒(响period ms和间隔period ms)
void Remind::buzzer_flash(int period)
{
    static bool is_run = false;
    static int cnt = 0;
    static int saved_period = 0;

    //周期变更时重置状态
    if (saved_period != period)
    {
        saved_period = period;
        cnt = period;
        is_run = false;
        buzzer_off();
        return;
    }
    
    cnt--;
    if (cnt <= 0)
    {
        //翻转状态
        is_run = !is_run;
        is_run ? buzzer_on() : buzzer_off();
        
        //重置计数器
        cnt = saved_period;
    }
}

//屏幕提醒
void Remind::lcd_remind(rgb565_color_enum rgb_color)
{
    if(is_screen_clear && !is_screen_remind)
    {
        lcd.clearScreen();
        is_screen_clear = false;
    }
    else if(is_screen_remind && !is_screen_clear)
    {
        lcd.showScreen(rgb_color);
        is_screen_remind = false;
    }
}


Remind remind;


