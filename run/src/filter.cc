#include "filter.h"

/************************************************一阶低通滤波************************************************/
//构造函数：把传入的 alpha 赋给成员变量，并标记未初始化
LowPassFilter::LowPassFilter(float alpha)
    : alpha_(alpha), initialized_(false), output_(0.0f) {}

// 重置：直接设置输出为指定值，并标记已初始化
void LowPassFilter::reset(float initial_value)
{
    output_ = initial_value;
    initialized_ = true;
}

//更新公式
float LowPassFilter::update(float input)
{
    if (!initialized_)
    {
        output_ = input;        //第一次输入，直接作为输出
        initialized_ = true;
    } 
    else
    {
        output_ = alpha_ * input + (1.0f - alpha_) * output_;   // 一阶低通公式
    }
    return output_;
}

//获取当前值（只读）
float LowPassFilter::getValue() const
{
    return output_;
}

//修改平滑系数
void LowPassFilter::setAlpha(float alpha)
{
    alpha_ = alpha;
}


/************************************************滑动均值滤波************************************************/
//构造函数：初始化成员变量，并动态分配缓冲区内存
AverageFilter::AverageFilter(uint8_t buffer_size)
    : buffer_size_(buffer_size), index_(0), count_(0), sum_(0.0f)
{
    buffer_ = new float[buffer_size];   //申请一块能放 buffer_size 个 float 的内存
    memset(buffer_, 0, buffer_size * sizeof(float));    //把整块内存初始化为 0
}

//析构函数：释放之前申请的动态内存，避免内存泄漏
AverageFilter::~AverageFilter()
{
    delete[] buffer_;
}

//重置：把内存窗口内所有值都填成 initial_value
void AverageFilter::reset(float initial_value)
{
    for (uint8_t i = 0; i < buffer_size_; i++)
    {
        buffer_[i] = initial_value;
    }
    sum_ = initial_value * buffer_size_;  //总和 = 值 × 窗口大小
    count_ = buffer_size_;                //认为窗口已经填满
    index_ = 0;                           //下次从第 0 位开始覆盖
}

//输入新值并返回当前平均值
float AverageFilter::update(float input)
{
    if (count_ < buffer_size_)
    {
        buffer_[count_] = input;    //内存窗口还没填满，直接追加到 buffer 末尾
        sum_ += input;
        count_++;
        return sum_ / count_;       //求平均值
    }
    else    //窗口已满，开始滑动，去掉最旧的值，加入新值
    {
        sum_ -= buffer_[index_];               //减去即将被覆盖的旧值
        buffer_[index_] = input;               //存入新值
        sum_ += input;                         //加上新值
        index_ = (index_ + 1) % buffer_size_;  //下标循环后移（环形缓冲区）
        return sum_ / buffer_size_;            //平均值
    }
    
}

//获取当前平均值（只读）
float AverageFilter::getValue() const
{
    if (count_ == 0) return 0.0f;   //窗口完全为空时返回 0
    return sum_ / count_;
}


/************************************************误差过渡器(绕行专用)************************************************/
//构造函数：指定过渡时长，初始状态为非活跃
ErrorFilter::ErrorFilter(uint32_t duration_ms)
    : duration_ms_(duration_ms), active_(false), start_time_(0), old_error_(0.0f), new_error_(0.0f) {}

//重置状态
void ErrorFilter::reset()
{
    active_ = false;
}

//启动过渡：记录旧误差、目标误差和起始时间，并激活过渡状态
void ErrorFilter::startDuration(float old_error, float new_error, uint32_t current_time_ms) 
{
    old_error_ = old_error;
    new_error_ = new_error;
    start_time_ = current_time_ms;
    active_ = true;
}

//获取当前平滑后的误差值（核心过渡逻辑）
float ErrorFilter::getSmoothedError(uint32_t current_time_ms)
{
    //如果不在过渡状态，直接返回目标误差
    if (!active_)
    {
        return new_error_;
    }

    //计算从开始过渡到现在过去了多少毫秒
    uint32_t elapsed = current_time_ms - start_time_;

    //如果已经超过过渡总时长，结束过渡，返回目标误差
    if (elapsed >= duration_ms_)
    {
        active_ = false;
        return new_error_;
    }

    float t = (float)elapsed / duration_ms_;    //线性插值：t 是从 0 到 1 的进度
    
    return old_error_ + (new_error_ - old_error_) * t;  //当前误差 = 旧误差 + (新误差 - 旧误差) * 进度
}

//查询是否还在过渡中
bool ErrorFilter::isActive() const
{
    return active_;
}

//强制结束过渡
void ErrorFilter::end()
{
    active_ = false;
}


/*类的定义*/
LowPassFilter L_enconder_pass(0.9f);     //左编码器，一阶低通滤波
LowPassFilter R_enconder_pass(0.9f);     //右编码器，一阶低通滤波
LowPassFilter yaw_lowpass(0.2f);         //陀螺仪偏航角速度，一阶低通滤波
LowPassFilter photo_pass(0.6f);          //过渡图像误差，一阶低通滤波
LowPassFilter tof_pass(0.6f);            //tof误差，一阶低通滤波
