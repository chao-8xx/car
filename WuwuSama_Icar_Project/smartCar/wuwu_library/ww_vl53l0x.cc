/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — VL53L0X 距离传感模块
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
 * 文件名称：ww_vl53l0x.cc
 * 所属模块：wuwu_library
 * 功能描述：VL53L0X 距离传感器驱动封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_vl53l0x.h"

VL53L0X::VL53L0X(void)
    : vl53l0x_distance_mm(8192)
    , syn_vl53l0x_distance_mm(8192)
    , fd(-1)
{
    pthread_mutex_init(&data_mutex, NULL);
}

VL53L0X::~VL53L0X(void)
{
    pthread_mutex_destroy(&data_mutex);
    if(fd < 0) return;
    close(fd);
}

/*******************************************************************
 * @brief       TOF初始化
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @example     //TOF初始化
 *              if(tof.tof_init() < 0) {
 *                  return -1;             
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/ 
int VL53L0X::tof_init(void)
{
    fd = open(DEVICE_TOF_ADDR, O_RDWR);
    if(fd < 0) {
        perror("Error open tof\r\n");
        return -1;
    } 
        
    std::cout << "TOF初始化成功" << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       更新TOF数据
 * 
 * @example     tof.upData();
 ******************************************************************/
void VL53L0X::upData(void)
{
    if(fd < 0) {
        std::cout <<"TOF设备未初始化!!!" << std::endl;
        return;
    }

    uint16_t data = 8192;
    if(ioctl(fd, VL53L0X_GET_DATA, &data) < 0) {
        perror("Failed to get tof data\r\n");
        return;
    }

    // if(std::abs(tof_distance - data) > 500.0f) data = tof_distance;
    // if(data > 2000.0f) data = 2000.0f;

    // TOF数据更新　加锁
    pthread_mutex_lock(&data_mutex);
    vl53l0x_distance_mm = data;
    pthread_mutex_unlock(&data_mutex);
}

/*******************************************************************
 * @brief       线程同步操作
 * 
 * @example     tof.thread_syn();
 * 
 * @note        在使用syn_xxx数据前, 调用此函数, 用于线程同步, 保证数据安全
 ******************************************************************/
void VL53L0X::thread_syn(void)
{
    pthread_mutex_lock(&data_mutex);
    syn_vl53l0x_distance_mm = vl53l0x_distance_mm;
    pthread_mutex_unlock(&data_mutex);
}
