#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — ICM42688 传感器接口头
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
 * 文件名称：ww_icm42688.h
 * 所属模块：wuwu_library
 * 功能描述：ICM42688 加速度计/陀螺仪驱动头文件
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WUWU_ICM42688_H
#define __WUWU_ICM42688_H

#include "headfile.h"

#define ICM42688_DEVICE                 "/dev/wuwu_icm42688"

// #define I2C_DEV_PATH "/dev/i2c-0" // 直接操作总线
// #define ICM42688_ADDR 0x68        // 你的设备地址

#define DEG_TO_RAD(x) ((x) * 0.01745329252f) // 度转弧度
#define RAD_TO_DEG(x) ((x) * 57.2957795131f) // 弧度转度


class ICM42688 {
public:
/**
 * @brief       未进行线程同步的数据
 * 
 * @note        如果读取数据和使用数据在同一线程,直接使用下面的即可
 */
    float gyro_x;           // 陀螺仪 x         (单位为 °/s)
    float gyro_y;           // 陀螺仪 y         (单位为 °/s)
    float gyro_z;           // 陀螺仪 z         (单位为 °/s)

    float accel_x;          // 加速度计 x       (单位为 g(m/s^2))
    float accel_y;          // 加速度计 y       (单位为 g(m/s^2))
    float accel_z;          // 加速度计 z       (单位为 g(m/s^2))

/**
 * @brief       线程同步后的数据
 * 
 * @note        如果读取不在此线程, 则先需要调用函数 thread_syn();
 *              再使用以下数据, 否则数据为空
 */
    float syn_gyro_x;       // 陀螺仪 x         (单位为 °/s)
    float syn_gyro_y;       // 陀螺仪 y         (单位为 °/s)
    float syn_gyro_z;       // 陀螺仪 z         (单位为 °/s)
    float syn_accel_x;      // 加速度计 x       (单位为 g(m/s^2))
    float syn_accel_y;      // 加速度计 y       (单位为 g(m/s^2))
    float syn_accel_z;      // 加速度计 z       (单位为 g(m/s^2))

    ICM42688(void);
    ~ICM42688(void);
/*******************************************************************
 * @brief       初始化陀螺仪
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @example     //初始化ICM42688
 *              if(icm42688.icm42688_init() < 0) {
 *                  return -1;
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
    int icm42688_init(void);

/*******************************************************************
 * @brief       更新加速度计数据
 * 
 * @example     icm42688.upDataAcc();
 * 
 * @note        更新内部加速度计数据并转换为实际数据(单位为 g(m/s^2))
 ******************************************************************/
    void upDataAcc(void);

/*******************************************************************
 * @brief       更新陀螺仪数据
 * 
 * @example     icm42688.upDataGyro();
 * 
 * @note        更新内部陀螺仪数据并转换为实际数据(单位为 °/s)
 ******************************************************************/
    void upDataGyro(void);

/*******************************************************************
 * @brief       线程同步操作
 * 
 * @example     icm42688.thread_syn();
 * 
 * @note        在使用syn_xxx数据前, 调用此函数, 用于线程同步, 保证数据安全
 ******************************************************************/
    void thread_syn(void);

private:
    // 设备文件描述符
    int fd;
    // 用于线程同步—互斥锁
    pthread_mutex_t data_mutex;

    // 数据转换为实际物理数据的转换系数
    float icm42688_acc_inv = 1;
    float icm42688_gyro_inv = 1;
    
};

// ---------------------------------------------------------
//  新增：Madgwick 算法类定义
//        Madgwick 梯度下降算法类，姿态解算，结合ICM42688使用
// ---------------------------------------------------------
class Madgwick{
public:
    float q0, q1, q2, q3;   // 四元数 表示当前姿态
    float beta;             // 算法增益 决定了对加速度计的信任度(高速运动状态下建议调小)

    Madgwick() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(0.1f) {}

/** 
 * @brief 更新姿态解算
 * 
 * @param gx, gy, gz : 陀螺仪数据 (单位：弧度/秒 rad/s) !!!注意不是度!!!
 * @param ax, ay, az : 加速度计数据 (单位：任意，归一化即可)
 * @param dt         : 两次调用之间的时间间隔 (单位：秒)
 * 
 * @note  使用 Madgwick 梯度下降算法更新四元数表示的姿态
 */
    void update(float gx, float gy, float gz, float ax, float ay, float az, float deltat);


};

extern ICM42688 icm42688;

#endif
