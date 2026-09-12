/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — ICM42688 IMU 模块
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
 * 文件名称：ww_icm42688.cc
 * 所属模块：wuwu_library
 * 功能描述：ICM42688 IMU 传感器驱动与读取封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_icm42688.h"

enum icm42688_afs
{
    ICM42688_AFS_16G,// default
    ICM42688_AFS_8G,
    ICM42688_AFS_4G,
    ICM42688_AFS_2G,
    NUM_ICM42688__AFS
};
enum icm42688_aodr
{
    ICM42688_AODR_32000HZ,// default
    ICM42688_AODR_16000HZ,
    ICM42688_AODR_8000HZ,
    ICM42688_AODR_4000HZ,
    ICM42688_AODR_2000HZ,
    ICM42688_AODR_1000HZ,
    ICM42688_AODR_200HZ,
    ICM42688_AODR_100HZ,
    ICM42688_AODR_50HZ,
    ICM42688_AODR_25HZ,
    ICM42688_AODR_12_5HZ,
    ICM42688_AODR_6_25HZ,
    ICM42688_AODR_3_125HZ,
    ICM42688_AODR_1_5625HZ,
    ICM42688_AODR_500HZ,
    NUM_ICM42688_AODR
};

enum icm42688_gfs
{
    ICM42688_GFS_2000DPS,// default
    ICM42688_GFS_1000DPS,
    ICM42688_GFS_500DPS,
    ICM42688_GFS_250DPS,
    ICM42688_GFS_125DPS,
    ICM42688_GFS_62_5DPS,
    ICM42688_GFS_31_25DPS,
    ICM42688_GFS_15_625DPS,
    NUM_ICM42688_GFS
};
enum icm42688_godr
{
    ICM42688_GODR_32000HZ,// default
    ICM42688_GODR_16000HZ,
    ICM42688_GODR_8000HZ,
    ICM42688_GODR_4000HZ,
    ICM42688_GODR_2000HZ,
    ICM42688_GODR_1000HZ,
    ICM42688_GODR_200HZ,
    ICM42688_GODR_100HZ,
    ICM42688_GODR_50HZ,
    ICM42688_GODR_25HZ,
    ICM42688_GODR_12_5HZ,
    ICM42688_GODR_X0HZ,
    ICM42688_GODR_X1HZ,
    ICM42688_GODR_X2HZ,
    ICM42688_GODR_500HZ,
    NUM_ICM42688_GODR
};

ICM42688::ICM42688(void)
    : fd(-1)
    , gyro_x(0), gyro_z(0), gyro_y(0)
    , accel_x(0), accel_y(0), accel_z(0)
    , syn_gyro_x(0), syn_gyro_y(0), syn_gyro_z(0)
    , syn_accel_x(0), syn_accel_y(0), syn_accel_z(0)
    
{
    //初始化互斥锁
    pthread_mutex_init(&data_mutex, NULL);

    icm42688_afs afs = ICM42688_AFS_16G;                //内核驱动已经按最高性能设置请勿更改
    icm42688_gfs gfs = ICM42688_GFS_2000DPS;            //内核驱动已经按最高性能设置请勿更改
    switch(afs)
    {
    case ICM42688_AFS_2G:
        icm42688_acc_inv = 2 / 32768.0f;                // 加速度计量程为:±2g
        break;
    case ICM42688_AFS_4G:
        icm42688_acc_inv = 4 / 32768.0f;                // 加速度计量程为:±4g
        break;
    case ICM42688_AFS_8G:
        icm42688_acc_inv = 8 / 32768.0f;                // 加速度计量程为:±8g
        break;
    case ICM42688_AFS_16G:
        icm42688_acc_inv = 16 / 32768.0f;               // 加速度计量程为:±16g
        break;
    default:
        icm42688_acc_inv = 1;                           // 不转化为实际数据
        break;
    }
    switch(gfs)
    {
    case ICM42688_GFS_15_625DPS:
        icm42688_gyro_inv = 15.625f / 32768.0f;         // 陀螺仪量程为:±15.625dps
        break;
    case ICM42688_GFS_31_25DPS:
        icm42688_gyro_inv = 31.25f / 32768.0f;          // 陀螺仪量程为:±31.25dps
        break;
    case ICM42688_GFS_62_5DPS:
        icm42688_gyro_inv = 62.5f / 32768.0f;           // 陀螺仪量程为:±62.5dps
        break;
    case ICM42688_GFS_125DPS:
        icm42688_gyro_inv = 125.0f / 32768.0f;          // 陀螺仪量程为:±125dps
        break;
    case ICM42688_GFS_250DPS:
        icm42688_gyro_inv = 250.0f / 32768.0f;          // 陀螺仪量程为:±250dps
        break;
    case ICM42688_GFS_500DPS:
        icm42688_gyro_inv = 500.0f / 32768.0f;          // 陀螺仪量程为:±500dps
        break;
    case ICM42688_GFS_1000DPS:
        icm42688_gyro_inv = 1000.0f / 32768.0f;         // 陀螺仪量程为:±1000dps
        break;
    case ICM42688_GFS_2000DPS:
        icm42688_gyro_inv = 2000.0f / 32768.0f;         // 陀螺仪量程为:±2000dps
        break;
    default:
        icm42688_gyro_inv = 1;                          // 不转化为实际数据
        break;
    }
}

ICM42688::~ICM42688(void)
{
    pthread_mutex_destroy(&data_mutex);
    if(fd < 0) return;
    close(fd);
}

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
int ICM42688::icm42688_init(void)
{
    fd = open(ICM42688_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "打开icm42688失败!!!" << std::endl;
        return -1;
    } else {
        std::cout << "icm42688初始化完毕" << std::endl;
    }
    return 0;
}

/*******************************************************************
 * @brief       更新加速度计数据
 * 
 * @example     icm42688.upDataAcc();
 * 
 * @note        更新内部加速度计数据并转换为实际数据(单位为 g(m/s^2))
 ******************************************************************/
void ICM42688::upDataAcc(void)
{
    if(fd < 0) {
        std::cout <<"陀螺仪设备未初始化!!!" << std::endl;
        return;
    }

    struct icm42688_accel_data data;
    ioctl(fd, ICM42688_GET_ACCEL, &data);
    float accel_x_temp = data.x * icm42688_acc_inv;
    float accel_y_temp = data.y * icm42688_acc_inv;
    float accel_z_temp = data.z * icm42688_acc_inv;

    // if(accel_x_temp >= -0.1 && accel_x_temp <= 0.1) accel_x_temp = 0.0001f;
    // if(accel_y_temp >= -0.1 && accel_y_temp <= 0.1) accel_y_temp = 0.0001f;
    // if(accel_z_temp >= -0.1 && accel_z_temp <= 0.1) accel_z_temp = 0.0001f;

    //用于线程同步
    pthread_mutex_lock(&data_mutex);
    accel_x = accel_x_temp;
    accel_y = accel_y_temp;
    accel_z = accel_z_temp;
    pthread_mutex_unlock(&data_mutex);
}

/*******************************************************************
 * @brief       更新陀螺仪数据
 * 
 * @example     icm42688.upDataGyro();
 * 
 * @note        更新内部陀螺仪数据并转换为实际数据(单位为 °/s)
 ******************************************************************/
void ICM42688::upDataGyro(void)
{
    if(fd < 0) {
        std::cout <<"陀螺仪设备未初始化!!!" << std::endl;
        return;
    }

    struct icm42688_gyro_data data;
    ioctl(fd, ICM42688_GET_GYRO, &data);
    float gyro_x_temp = data.x * icm42688_gyro_inv;
    float gyro_y_temp = data.y * icm42688_gyro_inv;
    float gyro_z_temp = data.z * icm42688_gyro_inv;

    // if(gyro_x_temp >= -0.3 && gyro_x_temp <= 0.3) gyro_x_temp = 0.0001f;
    // if(gyro_y_temp >= -0.3 && gyro_y_temp <= 0.3) gyro_y_temp = 0.0001f;
    // if(gyro_z_temp >= -0.3 && gyro_z_temp <= 0.3) gyro_z_temp = 0.0001f;

    //用于线程同步
    pthread_mutex_lock(&data_mutex);
    gyro_x = gyro_x_temp;
    gyro_y = gyro_y_temp;
    gyro_z = gyro_z_temp;
    pthread_mutex_unlock(&data_mutex);
}

/*******************************************************************
 * @brief       线程同步操作
 * 
 * @example     icm42688.thread_syn();
 * 
 * @note        在使用syn_xxx数据前, 调用此函数, 用于线程同步, 保证数据安全
 ******************************************************************/
void ICM42688::thread_syn(void)
{
    //用于线程同步
    pthread_mutex_lock(&data_mutex);
    syn_gyro_x = gyro_x;
    syn_gyro_y = gyro_y;
    syn_gyro_z = gyro_z;

    syn_accel_x = accel_x;
    syn_accel_y = accel_y;
    syn_accel_z = accel_z;
    pthread_mutex_unlock(&data_mutex);
}



// ---------------------------------------------------------
//  新增：Madgwick 算法类定义
//        Madgwick 梯度下降算法类，姿态解算，结合ICM42688使用
// ---------------------------------------------------------
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 1. 如果加速度计数据无效(全0)，则仅使用陀螺仪积分
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 2. 归一化加速度计数据
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 3. 辅助变量，减少重复计算
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        // 4. 梯度下降算法修正步骤 (利用加速度计修正陀螺仪漂移),修正误差、优化姿态
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // 5. 应用反馈增益 Beta
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;
    } else {
        // 只有陀螺仪积分
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    }

    // 6. 四元数积分 (更新姿态)
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 7. 归一化四元数 (保持单位长度)
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}


ICM42688 icm42688;


/*  四元数算法解释

1.四元数的定义
    q = q0 + q1·i + q2·j + q3·k     (q0实部，q1-3虚部,分别对应xyz)

2.四元数乘法
    由汉密尔顿公式 i² = j² = k² = i·j·k = -1 得
    总结乘法表（行×列）：
    ×	i	j	k
    i	-1	k	-j
    j	-k	-1	i
    k	j	-i	-1

    进一步推导得：
    p⊗q = [ p0·q0 - p1·q1 - p2·q2 - p3·q3,          // 实部
            p0·q1 + p1·q0 + p2·q3 - p3·q2,          // i
            p0·q2 - p1·q3 + p2·q0 + p3·q1,          // j
            p0·q3 + p1·q2 - p2·q1 + p3·q0 ]         // k

3.四元数旋转向量
    设世界坐标系下的一个向量 v_world，写成
        V_w = [0, vx, vy, vz]
    当前姿态对应的旋转四元数为 q（从世界系到机体坐标系），那么该向量在机体坐标系下的表示为：
        V_b = q* ⊗ V_w ⊗ q  (其中 q* 是 q 的共轭: q* = [q0, -q1, -q2, -q3])
    重力向量 在世界系中通常指向地心，我们取其为
        g_world = [0, 0, 0, 1]
    将重力旋转到机体坐标系
        g_body = q* ⊗ [0,0,0,1] ⊗ q
    展开乘法（用上面的乘法公式）得到 g_body 的各分量:
        g_body_real = 0  （纯四元数相乘结果实部为零，符合向量）
        g_body_x = 2·(q1·q3 - q0·q2)
        g_body_y = 2·(q2·q3 + q0·q1)
        g_body_z = q0^2 - q1^2 - q2^2 + q3^2

*/
