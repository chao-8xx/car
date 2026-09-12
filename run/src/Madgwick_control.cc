#include "Madgwick_control.h"

//创建全局变量
Madgwick filter;

// 定义零漂变量
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

//yaw 连续化(unwrap)变量
static bool yaw_inited = false;
static float yaw_last_deg = 0.0f;
static float yaw_unwrapped_deg = 0.0f;

// 姿态“安装零点”(参考姿态)——用于把静态倾角减掉
static bool att_ref_inited = false;
static float ref_q0 = 1.0f, ref_q1 = 0.0f, ref_q2 = 0.0f, ref_q3 = 0.0f;
static int att_ref_settle_cnt = 0;
static const int att_ref_settle_target = 200; 

//定义三个欧拉角为全局变量，方便分别调用
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

//进入控制线程的周期
extern float Ts;

//陀螺仪控制初始化，主要是零漂校准
void Madgwick_Ctrl::madgwickCtrl_init(void)
{
    //初始化
    icm42688.icm42688_init();
    filter.beta = 0.2f;             // Madgwick算法增益调整，beta为可调参数，高速状态下建议调小
   
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
}

//陀螺仪姿态解算，建议丢进线程或主循环，要求实时解算当前陀螺仪读取的数值
void Madgwick_Ctrl::pose_cal(void)
{
    //线程同步
    icm42688.thread_syn();
    //姿态解算  2ms（需要根据进入该线程的时间间隔进行调整）
    filter.update(
        DEG_TO_RAD((icm42688.syn_gyro_x - gyro_x_offset) * 2.4f),   // 2.4f
        DEG_TO_RAD((icm42688.syn_gyro_y - gyro_y_offset) * 2.4f),
        DEG_TO_RAD((icm42688.syn_gyro_z - gyro_z_offset) * 2.4f),
        icm42688.syn_accel_x,
        icm42688.syn_accel_y,
        icm42688.syn_accel_z,
        0.002f); 
    // 设定“安装零点”：把当前姿态作为 0 度参考（自动扣掉静态倾角）
    if (!att_ref_inited) 
    {
        att_ref_settle_cnt++;
        if (att_ref_settle_cnt >= att_ref_settle_target)
        {
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
    if (att_ref_inited)
    {
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
    roll  = RAD_TO_DEG(atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)));
    float pitch_sin = 2.0f * (q0 * q2 - q3 * q1);
    // 防止asin输入值越界
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    pitch = RAD_TO_DEG(asinf(pitch_sin));
    //  yaw 跳变处理
    float yaw_raw = RAD_TO_DEG(atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)));
    // atan2 输出天生在 [-180, 180]，这里做连续化，避免 180 ↔ -180 的显示跳变
    yaw = yaw_raw;
    if (!yaw_inited) 
    {
        yaw_inited = true;
        yaw_last_deg = yaw_raw;
        yaw_unwrapped_deg = yaw_raw;
    }
    else
    {
        float dy = yaw_raw - yaw_last_deg;
        if (dy > 180.0f) dy -= 360.0f;
        if (dy < -180.0f) dy += 360.0f;
        yaw_unwrapped_deg += dy;
        yaw_last_deg = yaw_raw;
        yaw = yaw_unwrapped_deg;
    }
    // 让主循环节拍与 dt=0.002f 匹配，避免用同一帧数据反复 update
    // usleep(2000);
}

//角度复位
void Madgwick_Ctrl::angle_reset(void)
{
    //将当前姿态设为新的零点
    ref_q0 = filter.q0;
    ref_q1 = filter.q1;
    ref_q2 = filter.q2;
    ref_q3 = filter.q3;
    att_ref_inited = true;

    //重置连续累积状态
    yaw = 0.0f;
    roll = 0.0f;
    pitch = 0.0f;
    yaw_inited = false;           //下一帧会重新初始化累加起点
    yaw_unwrapped_deg = 0.0f;     //只是保险，实际 yaw_inited=false 会重新设置
}

//偏航角度复位
void Madgwick_Ctrl::yaw_angle_reset(void)
{
    angle_reset();
    // //必须在安装零点已设定后才能调用
    // if (!att_ref_inited) return;

    // //获取当前的相对姿态欧拉角
    // float cur_roll  = roll;
    // float cur_pitch = pitch;

    // //构造一个新的相对姿态四元数
    // float half_roll  = DEG_TO_RAD(cur_roll)  * 0.5f;
    // float half_pitch = DEG_TO_RAD(cur_pitch) * 0.5f;

    // // 各轴旋转四元数
    // float qx0 = cosf(half_roll),  qx1 = sinf(half_roll),  qx2 = 0, qx3 = 0;   //绕X
    // float qy0 = cosf(half_pitch), qy1 = 0, qy2 = sinf(half_pitch), qy3 = 0;   //绕Y
    // float qz0 = 1, qz1 = 0, qz2 = 0, qz3 = 0;   //绕Z

    // //计算 q_rel_new = qz * qy * qx
    // float tmp0 = qy0 * qx0 - qy1 * qx1 - qy2 * qx2 - qy3 * qx3;
    // float tmp1 = qy0 * qx1 + qy1 * qx0 + qy2 * qx3 - qy3 * qx2;
    // float tmp2 = qy0 * qx2 - qy1 * qx3 + qy2 * qx0 + qy3 * qx1;
    // float tmp3 = qy0 * qx3 + qy1 * qx2 - qy2 * qx1 + qy3 * qx0;

    // //归一化
    // float norm = 1.0f / sqrtf(tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3);
    // float nq0 = tmp0 * norm;
    // float nq1 = tmp1 * norm;
    // float nq2 = tmp2 * norm;
    // float nq3 = tmp3 * norm;

    // //获取当前绝对姿态四元数 q_cur
    // float cq0 = filter.q0, cq1 = filter.q1, cq2 = filter.q2, cq3 = filter.q3;

    // float cr0 = nq0, cr1 = -nq1, cr2 = -nq2, cr3 = -nq3;
    // float nr0 = cq0*cr0 - cq1*cr1 - cq2*cr2 - cq3*cr3;
    // float nr1 = cq0*cr1 + cq1*cr0 + cq2*cr3 - cq3*cr2;
    // float nr2 = cq0*cr2 - cq1*cr3 + cq2*cr0 + cq3*cr1;
    // float nr3 = cq0*cr3 + cq1*cr2 - cq2*cr1 + cq3*cr0;

    // //更新参考零点
    // ref_q0 = nr0;
    // ref_q1 = nr1;
    // ref_q2 = nr2;
    // ref_q3 = nr3;
    // att_ref_inited = true;

    // //重置偏航连续化状态，下一帧 yaw 会从 0 附近开始
    // yaw_inited = false;
    // yaw_unwrapped_deg = 0.0f;
    // yaw = 0.0f;
}

//根据我们自己智能车陀螺仪的位置，改变定义的欧拉角
//读取俯仰角（y轴向前）
float Madgwick_Ctrl::pitch_get(void)
{
    return roll;
}

//读取翻滚角
float Madgwick_Ctrl::roll_get(void)
{
    return pitch;
}

//读取偏航角
float Madgwick_Ctrl::yaw_get(void)
{
    return yaw ;
}

//姿态传感器数据读取，需要放进线程里不断读取陀螺仪数据
void Madgwick_Ctrl::getIMU_Data(void)
{
    icm42688.upDataAcc();
    icm42688.upDataGyro();
    // pose_cal();                  //角度解算
}

Madgwick_Ctrl madg;

