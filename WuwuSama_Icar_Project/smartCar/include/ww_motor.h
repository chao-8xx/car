#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 电机驱动头文件
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
 * 文件名称：ww_motor.h
 * 所属模块：wuwu_library
 * 功能描述：电机（编码器）控制接口
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WUWU_MOTOR_H
#define __WUWU_MOTOR_H

#include "headfile.h"

#define DEVICE_MOTOR_ADDR               "/dev/wuwu_motor"

class Motor
{
public:
/**
 * @brief       未进行线程同步的数据
 * 
 * @note        如果读取数据和使用数据在同一线程,直接使用下面的即可
 */
    int encoder1_counts;                //编码器1
    int encoder2_counts;                //编码器2

/**
 * @brief       线程同步后的数据
 * 
 * @note        如果读取不在此线程, 则先需要调用函数 thread_syn();
 *              再使用以下数据, 否则数据为空
 */
    int syn_encoder1_counts;            //编码器1
    int syn_encoder2_counts;            //编码器2

    Motor(void);
    ~Motor(void);
/*******************************************************************
 * @brief   电机初始化(调用内核open函数启动pwm信号)
 *          程序调用内核open初始化pwm信号, 程序崩溃自动调用close关闭电机
 * 
 * @return      返回初始化状态
 * @retval      0           初始化成功
 * @retval      -1          初始化失败
 * 
 * @example     //初始化电机 初始化失败退出程序
 *              if(brushless.brushless_init() < 0) {
 *                  return -1;
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
    int motor_init(void);
    
/*******************************************************************
 * @brief   设置电机GPIO电平
 * 
 * @param   gpio_num    电机GPIO仅有2个,设置在设备树(非1及2)
 * @param   value       设置GPIO电平(非1及0)
 * 
 * @example motor.set_gpio_level(1, 0);
 ******************************************************************/
    void set_gpio_level(int gpio_num, int value);

/*******************************************************************
 * @brief   设置电机PWM占空比
 * 
 * @param   pwm_num     电机PWM仅有2个,设置在设备树(非1及2)
 * @param   duty        设置电机PWM控制信号占空比(0~10000)
 * 
 * @example motor.set_pwm_duty(1, 5000);
 ******************************************************************/
    void set_pwm_duty(int pwm_num, int duty);

/*******************************************************************
 * @brief   设置电机1 GPIO信号以及占空比
 *          (直接调用该函数效率高于 set_gpio_level + set_pwm_duty 组合)
 * 
 * @param   level       设置GPIO电平(非1及0)
 * @param   duty        设置电机PWM控制信号占空比(0~10000)
 * 
 * @example motor.set_motor1(0, 5000);
 ******************************************************************/
    void set_motor1(int level, int duty);

/*******************************************************************
 * @brief   设置电机2 GPIO信号以及占空比
 *          (直接调用该函数效率高于 set_gpio_level + set_pwm_duty 组合)
 * 
 * @param   level       设置GPIO电平(非1及0)
 * @param   duty        设置电机PWM控制信号占空比(0~10000)
 * 
 * @example motor.set_motor2(0, 5000);
 ******************************************************************/
    void set_motor2(int level, int duty);

/*******************************************************************
 * @brief   从驱动中读取编码器当前值
 * 
 * @example motor.update_encoders();
 ******************************************************************/
    void update_encoders(void);

/*******************************************************************
 * @brief       线程同步操作
 * 
 * @example     motor.thread_syn();
 * 
 * @note        在使用syn_xxx数据前, 调用此函数, 用于线程同步, 保证数据安全
 ******************************************************************/
    void thread_syn(void);

private:
    // 设备文件描述符
    int fd;
    // 用于线程同步—互斥锁
    pthread_mutex_t data_mutex;

};

#endif
