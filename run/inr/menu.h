#ifndef __MENU_H_
#define __MENU_H_

#include "headfile.h"

//文件类型（枚举)
typedef enum{
    Normal_Folder,
    Execute_Folder,
    bool_Box,
    int_Box,
    float_Box
} Folder_Kind;

//设置单个文件夹（节点）的结构和性质
typedef struct Menu_Folder{
    const char *name;       //文件名称
    uint8_t rank;           //当前节点所处的位次
    Folder_Kind kind;       //当前文件的类型
    void *data;             //指向所存放的变量
    void(*function)(void);  //指向可执行函数的指针

    uint8_t sons;           //子文件的个数

    struct Menu_Folder *father;         //父文件节点
    struct Menu_Folder *first_son;      //第一个子节点
    struct Menu_Folder *last_brother;   //上一个兄弟节点
    struct Menu_Folder *next_brother;   //下一个兄弟节点

    bool select;            //该节点是否被选中
    bool execute;           //该节点是否被执行

    float min_val;          //参数最小值
    float max_val;          //参数最大值

} Menu_Folder;

//函数声明
void Create_Menu_Item(Menu_Folder *father, Menu_Folder *me, const char name[], void *data, Folder_Kind kind, void(*function)(void));
Menu_Folder* Create_Menu_Folder(Menu_Folder *father, const char name[]);    //创建文件夹
void Create_Menu_Number(Menu_Folder *father, const char name[], void *data, Folder_Kind kind, float min_val = 0, float max_val = 0);  //创建参数
Menu_Folder* Create_Menu_Execute_Folder(Menu_Folder *father,const char name[], void(*function)(void));    //创建可执行文件


#endif
