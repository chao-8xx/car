#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 内核 IOCTL 定义头
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
 * 文件名称：ww_ioctl.h
 * 所属模块：wuwu_library
 * 功能描述：设备/驱动 IOCTL 常量与结构体定义
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WUWU_IOCRL_H_
#define __WUWU_IOCRL_H_

#include "headfile.h"

/* 本文件均为内部调用，用于用户态和内核态通信，修改将导致无法与内核态通信导致程序异常(请勿修改！) */
/* 本文件均为内部调用，用于用户态和内核态通信，修改将导致无法与内核态通信导致程序异常(请勿修改！) */
/* 本文件均为内部调用，用于用户态和内核态通信，修改将导致无法与内核态通信导致程序异常(请勿修改！) */

/***********************************************************************************
 * @brief           电机IOCTL
 * 
 * @param           MOTOR_IOCTL_MAGIC       IOCTL魔数(不可重复)
 * @param           MOTOR_SET_DIR           电机dir方向修改CMD
 * @param           MOTOR_SET_PWM           电机pwm占空比修改CMD
 * @param           MOTOR_SET_1DIRPWM       电机1 dir-pwm 修改CMD
 * @param           MOTOR_SET_2DIRPWM       电机2 dir-pwm 修改CMD
 * @param           ENCODER_GET_COUNT       编码器获取CMD
 * 
 * @param           dir_struct              控制电机dir信号的传递结构体
 * @param           pwm_struct              控制电机pwm信号的传递结构体
 * @param           GpioPwm_struct          控制电机dir和pwm信号的传递结构体
 * @param           encoder_counts          获取编码器的传递结构体
 * 
 * @note            以下内容均为内部调用，无须修改，修改以下任何参数，修改将导致电机无法正常使用
 *                           
 */
#define MOTOR_IOCTL_MAGIC               'O'
#define MOTOR_SET_DIR                   _IOW(MOTOR_IOCTL_MAGIC, 1, struct dir_struct)
#define MOTOR_SET_PWM                   _IOW(MOTOR_IOCTL_MAGIC, 2, struct pwm_struct)
#define MOTOR_SET_1DIRPWM               _IOW(MOTOR_IOCTL_MAGIC, 3, struct GpioPwm_struct)
#define MOTOR_SET_2DIRPWM               _IOW(MOTOR_IOCTL_MAGIC, 4, struct GpioPwm_struct)
#define ENCODER_GET_COUNT               _IOR(MOTOR_IOCTL_MAGIC, 5, struct encoder_counts)

struct dir_struct {
    int which;
    int dir;
};
struct pwm_struct {
    int which;
    int duty;
};
struct GpioPwm_struct {
    int dir;
    int duty;
};
struct encoder_counts {
    int encoder1_counts;
    int encoder2_counts;
};

/***********************************************************************************
 * @brief           ICM42688IOCTL
 * 
 * @param           ICM42688_IOCTL_MAGIC        IOCTL魔数(不可重复)
 * @param           ICM42688_GET_ACCEL          ICM42688获取加速度计数据CMD
 * @param           ICM42688_GET_GYRO           ICM42688获取陀螺仪数据CMD
 * 
 * @param           icm42688_accel_data         获取ICM42688加速度计数据
 * @param           icm42688_gyro_data          获取ICM42688陀螺仪数据
 * 
 * @note            以下内容均为内部调用，无须修改，修改以下任何参数，修改将导致ICM42688无法正常使用
 */
#define ICM42688_IOCTL_MAGIC            'I'
#define ICM42688_GET_ACCEL              _IOR(ICM42688_IOCTL_MAGIC, 1, struct icm42688_accel_data)
#define ICM42688_GET_GYRO               _IOR(ICM42688_IOCTL_MAGIC, 2, struct icm42688_gyro_data)

struct icm42688_accel_data {
    short x, y, z;
};
struct icm42688_gyro_data {
    short x, y, z;
};

/***********************************************************************************
 * @brief           无缘蜂鸣器IOCTL
 * 
 * @param           BUZEER_IOCTL_MAGIC          IOCTL魔数(不可重复)
 * @param           BUZEER_SET_DUTYHZ           蜂鸣器占空比-频率设置
 * 
 * @param           Buzzer_pwm                  无缘蜂鸣器频率-占空比传递结构体
 * 
 * @note            以下内容均为内部调用，无须修改，修改以下任何参数，修改将导致蜂鸣器无法正常使用
 */
#define BUZEER_IOCTL_MAGIC              'b'
#define BUZEER_SET_DUTYHZ               _IOW(BUZEER_IOCTL_MAGIC, 1, struct Buzzer_pwm)

struct Buzzer_pwm {
    int hz;
    int duty;
};

/***********************************************************************************
 * @brief           ZF无刷电调PWM控制IOCTL
 * 
 * @param           BRUSHLESS_IOCTL_MAGIC       IOCTL魔数(不可重复)
 * @param           BRUSHLESS_SET_DUTY          无刷电调控制
 * 
 * @param           Brushless_duty              无刷电调百分比控制
 * 
 * @note            以下内容均为内部调用，无须修改，修改以下任何参数，修改将导致电调无法正常使用
 */
#define BRUSHLESS_IOCTL_MAGIC           'B'
#define BRUSHLESS_SET_DUTY              _IOW(BRUSHLESS_IOCTL_MAGIC, 1, struct Brushless_duty)

struct Brushless_duty {
    int duty;
};

/***********************************************************************************
 * @brief           TOF读取IOCTL
 * 
 * @param           VL53L0X_IOCTL_MAGIC         IOCTL魔数(不可重复)
 * @param           VL53L0X_GET_DATA            TOF数据获取
 * 
 * @note            以下内容均为内部调用，无须修改，修改以下任何参数，修改将导致TOF无法正常使用
 */
#define VL53L0X_IOCTL_MAGIC             't'
#define VL53L0X_GET_DATA                _IOR(VL53L0X_IOCTL_MAGIC, 1, uint16_t)


#endif
