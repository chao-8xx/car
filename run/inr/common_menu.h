#ifndef __MY_MENU_H_
#define __MY_MENU_H_

#include "headfile.h"

//文件类型
typedef enum {
    Normal_Folder,                          //文件夹
    bool_Box,                               //布尔型
    int_Box,                                //常数型
    float_Box                               //浮点数型
}Folder_Class;

typedef struct Folder_Menu {
    const char *name;                       //文件名称
    uint8_t No;                             //当前文件夹下的位次
    Folder_Class kind;                      //文件种类

    uint8_t sons_Count;                     //子文件个数

    struct Folder_Menu *father;             //父文件节点
    struct Folder_Menu *son_first;          //第一个子文件
    struct Folder_Menu *next_brother;       //下一个兄弟文件
    struct Folder_Menu *last_brother;       //上一个兄弟文件
    
    void*   private_data;                   //私有数据
    uint8_t number_box_select;              //数值项是否被选中

} Folder_Menu;

extern Folder_Menu myMenu;
extern Folder_Menu *key_menu_p;

void create_Menu_Folder(Folder_Menu *father, Folder_Menu *me, const char *name);
void create_Menu_CheckBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *check);
void create_Menu_NumberBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *number);
void create_Menu_FloatBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *float_number);
void All_Folder_Menu_Init(Folder_Menu *Menu);


#endif
