#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 无刷电机（Brushless）模块
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
 * 文件名称：ww_brushless.cc
 * 所属模块：wuwu_library
 * 功能描述：无刷电机控制封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-9  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_brushless.h"

Brushless fans;

Brushless::Brushless(void)
    : fd(-1)
{

}

Brushless::~Brushless(void)
{
    if(fd < 0) return;
    close(fd);
}

/*******************************************************************
 * @brief       无刷电机初始化
 * 
 * @return      返回初始化状态
 * @retval      0           初始化成功
 * @retval      -1          初始化失败
 * 
 * @example     //初始化电调 初始化失败退出程序
 *              if(brushless.brushless_init() < 0) {
 *                  return -1;
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
int Brushless::brushless_init(void)
{
    fd = open(FANS_DIR, O_WRONLY);
    if(fd < 0) {
        perror("Error open brushless\r\n");
        return -1;
    } else {
        std::cout << "无刷电机初始化成功" << std::endl;
    }
    return 0;
}

/*******************************************************************
 * @brief       ZF-STC电调百分比控制
 * 
 * @param       输入范围(0～100) 内核态已对输入做保护
 * 
 * @example     //设置电调占空比50%
 *              brushless.set_duty(50);
 * 
 * @note        无须考虑PWM，内核态已做处理，0%～100%对应PWM高电平1ms ～ 2ms
 *              用户直接调用即可
 ******************************************************************/
void Brushless::set_duty(int value)
{
    if(fd < 0) {
        std::cout <<"设备未初始化!!!" << std::endl;
        return;
    }

    struct Brushless_duty brushless_duty;
    value = (value < 0) ? 0 : (value > 100) ? 100 : value;
    brushless_duty.duty = value;
    int err = ioctl(fd, BRUSHLESS_SET_DUTY, &brushless_duty);
    if(err < 0) {
        perror("Failed to set brushless duty\r\n");
        return;
    }
    duty = value;
}

/*******************************************************************
 * @brief       获取当前占空比
 * 
 * @return      返回私有数据duty(该数据在set_duty成功是更新)
 * 
 * @example     //打印占空比
 *              std::cout << brushless.get_duty() << std::endl;
 ******************************************************************/
int Brushless::get_duty(void)
{
    return duty;
}
