#ifndef __MENU_TASK_H_
#define __MENU_TASK_H_

#include "headfile.h"

void menu_init(void);
void menu_show_All(void);

extern bool check_red_enable;

//菜单控制部分
void Menu_upFuntion(void);          //给按键1
void Menu_downFuntion(void);        //给按键2
void Menu_enterFuntion(void);       //给按键3
void Menu_quitFuntion(void);        //给按键4
void Menu_Setup_Index(void);        //给调整步进用按键
void go_iCar(void);                 //发车键
void fan_Enable(void);              //开负压按键

void all_key_Ctl(void);


#endif