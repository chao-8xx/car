// wuwu_timerthread.h
/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 定时线程头
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
 * 文件名称：ww_timerThread.h
 * 所属模块：wuwu_library
 * 功能描述：定时器线程封装（周期回调）
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-9  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WUWU_TIMERTHREAD_H
#define __WUWU_TIMERTHREAD_H

#include "headfile.h"

class TimerThread {
public:
/******************************************************************
 * @brief   线程回调函数原型
 ******************************************************************/
typedef void (*CallbackFunc)(void*);
    
/******************************************************************
 * @brief       构造函数
 * 
 * @param       func                    传入周期控制函数
 * @param       arg                     传入回调函数的参数, 没有设置为NULL
 * @param       interval_ms             执行间隔（毫秒），默认10ms
 * 
 * @example     TimerThread thread1(xxxxx, NULL, 10);
 ******************************************************************/
    TimerThread(CallbackFunc func, void* arg = NULL, unsigned int interval_ms = 10);

    ~TimerThread(void);
    
/******************************************************************
 * @brief       启动定时线程
 * 
 * @return      成功返回true，失败返回false
 * 
 * @example     thread1.start();
 * 
 * @note        线程启动后自动与主线程分离
 ******************************************************************/
    bool start();

    // [新增] 停止线程并等待退出（线程未启动时安全空操作）
    void stop();
    
private:
    static void* thread_entry(void* param);
    pthread_t thread_id;

    CallbackFunc func;
    void* arg;
    unsigned int interval_ms;

    std::atomic<bool> running;
    std::atomic<bool> started;
    
};

#endif // __WUWU_TIMERTHREAD_H
