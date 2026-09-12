#ifndef __FILTER_H__
#define __FILTER_H__
#include "headfile.h"

/***********一阶低通滤波***********/
class LowPassFilter{
public:
    explicit LowPassFilter(float alpha = 0.2f);  //构造函数
    void reset(float initial_value = 0.0f);      //重置滤波器，可指定初始输出值
    float update(float input);                   //输入新的原始值，返回滤波后的值
    float getValue() const;                      //获取当前值（只读）
    void setAlpha(float alpha);                  //修改平滑系数

private:
    float alpha_;       //平滑系数
    bool initialized_;  //是否初始化
    float output_;      //当前滤波输出值

};


/***********滑动均值滤波***********/
class AverageFilter{
public:
    explicit AverageFilter(uint8_t buffer_size = 10);   //构造函数
    ~AverageFilter();                                   //析构函数
    void reset(float initial_value);                    //重置内存窗口均为为某个值
    float update(float input);                          //输入并更新
    float getValue() const;                             //获取当前平均值（只读）

private:
    uint8_t buffer_size_;  //内存大小(个数)
    uint8_t index_;        //当前要写入的环形缓冲区下标
    uint8_t count_;        //已经存入的数据个数（未满窗口时用）
    float sum_;            //当前窗口内所有值的总和
    float *buffer_;        //动态分配的环形缓冲区指针
};


/***********误差过渡器(绕行专用)***********/
class ErrorFilter{
public:
    explicit ErrorFilter(uint32_t duration_ms = 200);   //构造函数
    void reset();                                       //重置状态
    void startDuration(float old_error, float new_error, uint32_t current_time_ms); //启动过渡
    float getSmoothedError(uint32_t current_time_ms);   //获取当前平滑后的误差值（核心过渡逻辑）
    bool isActive() const;                      //查询是否还在过渡中
    void end();                                 //强制结束过渡

private:
    uint32_t duration_ms_;       //过渡总时长
    uint32_t start_time_;        //过渡开始时间
    bool active_;                //是否正处于过渡过程中
    float old_error_;            //过渡起始误差（旧误差）
    float new_error_;            //过渡目标误差（新误差）
};


/*声明类*/
extern LowPassFilter L_enconder_pass;     //左编码器
extern LowPassFilter R_enconder_pass;     //右编码器
extern LowPassFilter yaw_lowpass;         //偏航角速度
extern LowPassFilter photo_pass;          //图像误差滤波
extern LowPassFilter tof_pass;            //tof测距

#endif
