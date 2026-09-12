#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — TCP 客户端模块
 * 版权所有 (c) 2025 chao-8xx
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
 * 文件名称：ww_tcp_client.h
 * 所属模块：wuwu_library
 * 功能描述：TCP 客户端接口与 VOFA 协议封装头文件
 *
 * 修改记录：
 * 日期         作者         说明
 * 2025-12-24  chao-8xx    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WW_TCP_CLIENT_H__
#define __WW_TCP_CLIENT_H__

#include "headfile.h"

/*******************************************************************
 * [父类] Tcp Client: 底层通信驱动
 * 职责：只负责 TCP 连接、断开、原始字节的发送和检查连接状态
 * 特点：可用于连接任何 TCP 服务器
 ******************************************************************/
class TcpClient
{
public:
    // 默认连接的服务器 IP 和端口 Port (根据用户个人电脑设置修改)
    static constexpr const char*    DEFAULT_SERVER_IP = "192.168.2.10";
    static constexpr int            DEFAULT_SERVER_PORT = 2233;


    TcpClient(void);
    ~TcpClient(void);
/*******************************************************************
 * @brief       连接TCP服务器 
 * 
 * @param       ip              服务器IP地址
 * @param       port            服务器端口号
 * 
 * @return      返回连接状态
 * @retval      0               连接成功
 * @retval      -1              连接失败
 * 
 * @example     //连接到主机的VOFA+服务器
 *              if(tcp_client.connect_server("192.168.1.101", 2233) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        建立与TCP服务器的连接，供后续数据传输使用
 ******************************************************************/
    int connect_server(const char* ip = DEFAULT_SERVER_IP, int port = DEFAULT_SERVER_PORT);

/*******************************************************************
 * @brief       发送原始字节数据
 * 
 * @param       data    待发送数据指针
 * @param       len     数据长度
 * 
 * @return      true:发送成功, false:发送失败
 * 
 * @note        将上层 TCP 设备协议打包来通信
 ******************************************************************/
    bool send_bytes(const void* data, size_t len);

/*******************************************************************
 * @brief       发送字符串
 * 
 * @param       s       待发送字符串
 * 
 * @return      true:发送成功, false:发送失败
 * 
 * @example     tcp_client.send_string("Hello, TCP Server!");
 * 
 * @note        将上层 TCP 设备协议打包来通信
 ******************************************************************/
    bool send_string(const std::string& s);

/*******************************************************************
 * @brief       断开与TCP服务器的连接
 * 
 * @example     tcp_client.disconnect_server();
 * 
 * @note        断开连接并释放Socket资源
 ******************************************************************/
    void disconnect_server(void);

/*******************************************************************
 * @brief       检查服务器是否已连接
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器已连接
 * @retval      false           服务器未连接
 * 
 * @example     if(tcp_client.is_connected()) {
 *                  //服务器已连接
 *              }
 ******************************************************************/
    bool is_connected(void);

protected:
    // Socket文件描述符
    int sock_fd;

    // 连接状态
    bool connected;

    // Socket互斥锁
    pthread_mutex_t sock_mutex;

};

/*******************************************************************
 * [子类] VofaClient: VOFA+ 协议层封装
 * 负责：将发送数据封装为 VOFA+ 的 FireWater 协议格式帧
 ******************************************************************/
class VofaClient : public TcpClient
{
public:
    VofaClient(void);
    ~VofaClient(void);

/*******************************************************************
 * @brief        发送格式化数据到VOFA+服务器 (FireWater协议)
 * 
 * @param        format  格式化字符串 (如 "data0: %f, data1: %f\n")
 * @param        ...     可变参数列表
 * 
 * @example      // 发送浮点数
 * client.send_firewater("sin: %.2f, %.2f\n", sin_val, cos_val);
 * 
 * @example      // 发送混合数据
 * client.send_firewater("temp: %.1f, %d\n", 25.5, 1);
 * 
 * @note         底层使用 vsnprintf 格式化，完全解耦，不限制数据类型
 *               发送数据格式说明: FireWater 协议格式:"name:csv_numbers\n"
 *******************************************************************/
    void send_firewater(const char* format, ...);
 
private:
    // FireWater 格式化缓冲区
    char firewater_buffer[1024];

};



#endif
