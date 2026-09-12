
#include "headfile.h"

/**************************************************************************
 * @brief       定时线程对姿态传感器读取数据 主线程显示 例程
 * 
 * @note        Creation Time :         2025/12/9
 * @note        Author :                WuwuSama
 * @note        E-mail :                1635202242@qq.com
 * 
 * @note        Attention :             这个例程中创建了一个10ms的定时线程, 在创建的线程中
 *                                      更新icm42688对象中的陀螺仪和加速度计数据, 在主线程
 *                                      调用LCD显示六轴数据, 此做法存在数据竞争, 需要使用
 *                                      线程同步机制保证数据安全, 库中已经封装好同步操作, 
 *                                      参考本例程的使用方法。
 * 
 ***************************************************************************/

 // 配置电脑的 IP 和 Port (根据用户个人电脑设置修改)
#define SERVER_IP       "192.168.2.10"
#define SERVER_PORT     2233

//创建全局变量
ICM42688 icm42688;
Madgwick filter;
VofaClient client;

// 定义零漂变量
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

// yaw 连续化(unwrap)变量
static bool yaw_inited = false;
static float yaw_last_deg = 0.0f;
static float yaw_unwrapped_deg = 0.0f;

// 姿态“安装零点”(参考姿态)——用于把静态倾角减掉
static bool att_ref_inited = false;
static float ref_q0 = 1.0f, ref_q1 = 0.0f, ref_q2 = 0.0f, ref_q3 = 0.0f;
static int att_ref_settle_cnt = 0;
static const int att_ref_settle_target = 200; 

//声明函数
void getIMU_Data(void *arg);

int main() {  

    std::cout << ">>> 服务器 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> 服务器 Port: " << SERVER_PORT << std::endl;

    //初始化
    icm42688.icm42688_init();
    filter.beta = 0.2f;             // Madgwick算法增益调整，beta为可调参数，高速状态下建议调小

    // 连接 TCP 服务器
    while (client.connect_server(SERVER_IP, SERVER_PORT) < 0)
    {
        std::cout << "连接失败, 1秒后重试..." << std::endl;
        usleep(1000000);
    }

    // 陀螺仪零漂校准
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int counts = 500;

    std::cout << "陀螺仪零漂校准中, 请保持静止..." << std::endl;

    for (int i = 0; i < counts; i++) {
        icm42688.upDataGyro();

        sum_gx += icm42688.gyro_x;
        sum_gy += icm42688.gyro_y;
        sum_gz += icm42688.gyro_z;
        usleep(2000); 
    }

    std::cout << "陀螺仪零漂校准完成!" << std::endl;

    gyro_x_offset = sum_gx / counts;
    gyro_y_offset = sum_gy / counts;
    gyro_z_offset = sum_gz / counts;

    //创建陀螺仪读取线程(10ms)执行一次 getIMU_Data 内的内容
    TimerThread getIMU_thread(getIMU_Data, NULL, 10);
    //启动线程
    getIMU_thread.start();

    while(1) 
    {
        //线程同步
        icm42688.thread_syn();

        //姿态解算  10ms
        filter.update(
            DEG_TO_RAD(icm42688.syn_gyro_x - gyro_x_offset),
            DEG_TO_RAD(icm42688.syn_gyro_y - gyro_y_offset),
            DEG_TO_RAD(icm42688.syn_gyro_z - gyro_z_offset),
            icm42688.syn_accel_x,
            icm42688.syn_accel_y,
            icm42688.syn_accel_z,
            0.01f); 

        // 设定“安装零点”：把当前姿态作为 0 度参考（自动扣掉静态倾角）
        if (!att_ref_inited) {
            att_ref_settle_cnt++;
            if (att_ref_settle_cnt >= att_ref_settle_target) {
                // 采集够稳定帧数，设定参考姿态
                ref_q0 = filter.q0;
                ref_q1 = filter.q1;
                ref_q2 = filter.q2;
                ref_q3 = filter.q3;
                att_ref_inited = true;

                // 设完零点后，重置 yaw unwrap 基准
                yaw_inited = false;
            }
        }

        // 计算相对姿态：q_rel = conj(q_ref) ⊗ q_cur
        float q0 = filter.q0;
        float q1 = filter.q1;
        float q2 = filter.q2;
        float q3 = filter.q3;
        if (att_ref_inited) {
            const float r0 = ref_q0;
            const float r1 = -ref_q1;
            const float r2 = -ref_q2;
            const float r3 = -ref_q3;
            const float nq0 = r0*q0 - r1*q1 - r2*q2 - r3*q3;
            const float nq1 = r0*q1 + r1*q0 + r2*q3 - r3*q2;
            const float nq2 = r0*q2 - r1*q3 + r2*q0 + r3*q1;
            const float nq3 = r0*q3 + r1*q2 - r2*q1 + r3*q0;
            q0 = nq0; q1 = nq1; q2 = nq2; q3 = nq3;
        }

        // 转换为欧拉角 发送到电脑
        float roll  = RAD_TO_DEG(atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)));
        float pitch_sin = 2.0f * (q0 * q2 - q3 * q1);
        // 防止asin输入值越界
        if (pitch_sin > 1.0f) pitch_sin = 1.0f;
        if (pitch_sin < -1.0f) pitch_sin = -1.0f;
        float pitch = RAD_TO_DEG(asinf(pitch_sin));
        //  yaw 跳变处理
        float yaw_raw = RAD_TO_DEG(atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)));

        // atan2 输出天生在 [-180, 180]，这里做连续化，避免 180 ↔ -180 的显示跳变
        float yaw = yaw_raw;
        if (!yaw_inited) {
            yaw_inited = true;
            yaw_last_deg = yaw_raw;
            yaw_unwrapped_deg = yaw_raw;
        } else {
            float dy = yaw_raw - yaw_last_deg;
            if (dy > 180.0f) dy -= 360.0f;
            if (dy < -180.0f) dy += 360.0f;
            yaw_unwrapped_deg += dy;
            yaw_last_deg = yaw_raw;
            yaw = yaw_unwrapped_deg;
        }

        if(client.is_connected()){
            // 发送欧拉角到VOFA+服务器
            client.send_firewater("anger:%.2f,%.2f,%.2f\n", -roll, pitch, yaw);     
        }

        else{
            std::cout << "连接断开, 正在重连..." << std::endl;
            usleep(1000000);
            client.connect_server(SERVER_IP, SERVER_PORT);
        }

        // 让主循环节拍与 dt=0.01f 匹配，避免用同一帧数据反复 update
        usleep(10000);
    }
      
    return 0;
}

//姿态传感器数据读取线程函数
void getIMU_Data(void *arg)
{
    icm42688.upDataAcc();
    icm42688.upDataGyro();
}