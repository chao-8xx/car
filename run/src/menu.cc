#include "menu.h"

//给菜单创建一个内存池（静态数组）
#define MENU_SIZE   256                          //内存池大小,若节点数量超过该大小，则需增大内存
static  Menu_Folder menu_memory[MENU_SIZE];     //内存池（静态数组）
static  uint8_t menu_memory_index = 0;          //内存池索引

//初始化节点成员
void Create_Menu_Item(Menu_Folder *father, Menu_Folder *me, const char name[], void *data, Folder_Kind kind, void(*function)(void))
{
    //判断父节点是否为文件夹，若不是文件夹，则创建失败（返回）
    if(father->kind != Normal_Folder)
    {
        return ;
    }

    //节点类型初始化
    me->name = name;
    me->kind = kind;
    me->data = data;
    me->sons = 0;
    me->select = false;
    me->execute = false;
    //节点链表初始化
    me->father = father;
    me->first_son = NULL;
    me->last_brother = NULL;
    me->next_brother = NULL;
    me->function = function;

    //把“我”作为最后一个子节点进行控制
    if(father->sons == 0)
    {
        father->first_son = me;
    }
    else
    {
        Menu_Folder *p = father->first_son;
        while(p->next_brother != NULL)
        {
            p = p->next_brother;
        }
        //双向添加
        p->next_brother = me;
        me->last_brother = p;
    }

    //确定“我”在父节点所在的位次
    father->sons ++;
    me->rank = father->sons;
}

//动态创建文件夹
Menu_Folder* Create_Menu_Folder(Menu_Folder *father, const char name[])
{
    Menu_Folder *me = &menu_memory[menu_memory_index ++];

    Create_Menu_Item(father,me,name,NULL,Normal_Folder, NULL);

    return me;
}

//动态创建文件参数
void Create_Menu_Number(Menu_Folder *father, const char name[], void *data, Folder_Kind kind, float min_val, float max_val)
{
    Menu_Folder *me = &menu_memory[menu_memory_index ++];

    Create_Menu_Item(father, me, name, data, kind, NULL);
    me->min_val = min_val;
    me->max_val = max_val;
}

//动态创建可执行文件夹
Menu_Folder* Create_Menu_Execute_Folder(Menu_Folder *father,const char name[], void(*function)(void))
{
    Menu_Folder *me = &menu_memory[menu_memory_index ++];

    Create_Menu_Item(father,me,name,NULL,Execute_Folder,function);

    return me;
}

