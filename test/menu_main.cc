//菜单测试代码
#include "headfile.h"

Key key0;
AsyncMusicPlayer music_player;
extern LCD tft180;

void key_thread(void *arg);
void music_thread(void *arg);

int main(void)
{
    key0.key_init();

    tft180.lcd_init();
    menu_init();

    TimerThread thread_key(key_thread,NULL,0);
    thread_key.start();
    
    show_menu();

    while(1)
    {
        
    }

    return 0;
}

void key_thread(void *arg)
{
    key0.key_listeners();   //监听按键状态
    show_number();          //按键触发时，参数要实时显示，便于观察
}

void music_thread(void *arg)
{
    (void)arg;
    music_player.play(qing_tian, sizeof(qing_tian) / sizeof(Note));
}