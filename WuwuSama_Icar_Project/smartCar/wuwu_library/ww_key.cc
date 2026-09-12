#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 按键模块
 * 版权所有 (c) 2025 wuwu
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 文件名称：ww_key.cc
 * 所属模块：wuwu_library
 * 功能描述：按键扫描与事件处理封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_key.h"
#include "headfile.h"

#define KEY_NUM         (5)

//菜单头节点
extern Menu_Folder head;
//菜单按键控制索引指针
extern Menu_Folder *key;
extern LCD tft180;

static void KEY1_CallBack(void)
{
    // 检查是否处于模型标定模式
    extern ModelCalibration model_calib;
    if (model_calib.current_state == CalibState::editing) {
        model_calib.add_action(ACTION_LEFT);  // 添加"左绕"
        return;
    }
    
    key_up();       //按键1，按键向上或参数加
    // show_key();     //光标实时显示
    show_menu();    //菜单元素实时显示
}

static void KEY2_CallBack(void)
{
    // 检查是否处于模型标定模式
    extern ModelCalibration model_calib;
    if (model_calib.current_state == CalibState::editing) {
        model_calib.add_action(ACTION_STRAIGHT);  // 添加"直走"
        return;
    }
    
    key_down();    //按键2，按键向下或参数减 
    // show_key();    //光标实时显示
    show_menu();   //菜单元素实时显示
}

static void KEY3_CallBack(void)
{
    // 检查是否处于模型标定模式
    extern ModelCalibration model_calib;
    if (model_calib.current_state == CalibState::editing) {
        model_calib.add_action(ACTION_RIGHT);  // 添加"右绕"
        return;
    }
    
    key_quit();     //按键3，退出菜单
    show_menu();    //刷新屏幕，显示上一级菜单内容
}

static void KEY4_CallBack(void)
{
    // 检查是否处于模型标定模式
    extern ModelCalibration model_calib;
    if (model_calib.current_state == CalibState::editing) {
        model_calib.delete_last_action();  // 删除最后一个
        return;
    }
    
    key_enter();    //按键4，进入菜单
    show_menu();    //刷新屏幕，显示下一级菜单内容
}

static void KEY5_CallBack(void)
{
    // 检查是否处于模型标定模式
    extern ModelCalibration model_calib;
    if (model_calib.current_state == CalibState::editing) {
        model_calib.save_and_exit();  // 保存并退出
        return;
    }
    
    key_select();   //按键5，选择参数
    show_key();    //在参数处显示光标
    // show_menu();     //菜单元素实时显示 
}

Key::Key(void) {}

Key::~Key(void)
{
    if(fd < 0) return;
    close(fd);
}

/*******************************************************************
 * @brief       按键初始化
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @example     //按键初始化
 *              if(key.key_init() < 0) {
 *                  return -1;             
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
int Key::key_init(void)
{
    fd = open(KEY_FILE_ADD, O_RDONLY);
    if(fd < 0) {
        perror("key init error!\n");
        return -1;
    } 
    std::cout << "按键初始化成功" << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       案件监听
 * 
 * @example     //按键阻塞监听
 *              key.key_listeners();
 ******************************************************************/
void Key::key_listeners(void)
{
    struct input_event event;
    if(read(fd, &event, sizeof(event)) != sizeof(event)) 
        return;

    if(event.type == EV_KEY && (event.value == 1 || event.value == 2)) {
        int keycode = event.code;

        switch (keycode)
        {
        case 2: KEY1_CallBack(); break;
        case 3: KEY2_CallBack(); break;
        case 4: KEY3_CallBack(); break;
        case 5: KEY4_CallBack(); break;
        case 6: KEY5_CallBack(); break;
        //...(可以根据设备树继续添加)
        default: break;
        }
    }
}
