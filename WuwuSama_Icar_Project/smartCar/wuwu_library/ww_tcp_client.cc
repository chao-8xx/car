#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — TCP 客户端模块
 * 版权所有 (c) 2025 chao_8xx
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
 * 文件名称：ww_tcp_client.cc
 * 所属模块：wuwu_library
 * 功能描述：TCP 客户端封装及 VOFA 协议适配
 *
 * 修改记录：
 * 日期        作者          说明
 * 2025-12-24  chao_8xx    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "headfile.h"

/*******************************************************************
 * [父类] Tcp Client: 底层通信驱动
 * 职责：只负责 TCP 连接、断开、原始字节的发送和检查连接状态
 * 特点：可用于连接任何 TCP 服务器
 ******************************************************************/

//我的电脑IP和端口

TcpClient::TcpClient(void)
    : sock_fd(-1)
    , connected(false)
{
    pthread_mutex_init(&sock_mutex, NULL);
}

TcpClient::~TcpClient(void)
{
    disconnect_server();
    pthread_mutex_destroy(&sock_mutex);
}

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
int TcpClient::connect_server(const char* ip, int port)
{
    if (connected) {
        disconnect_server();
    }

    pthread_mutex_lock(&sock_mutex);

    // 创建 Socket
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror("Socket 创建失败");
        pthread_mutex_unlock(&sock_mutex);
        return -1;  
    }

    // 设置服务器地址结构体
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;                   // IPv4(地址族协议)
    server_addr.sin_port = htons(port);                 // 端口号
    server_addr.sin_addr.s_addr = inet_addr(ip);        // IP地址

    // 连接到服务器
    if (connect(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("[TCP] Connect failed 连接失败");
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        pthread_mutex_unlock(&sock_mutex);
        return -1;
    }

    connected = true;
    printf("[TCP] Successfully connected to server %s:%d 连接成功\n", ip, port);

    pthread_mutex_unlock(&sock_mutex);
    return 0;

}

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
bool TcpClient::send_bytes(const void* data, size_t len)
{
    pthread_mutex_lock(&sock_mutex);

    if (!connected || sock_fd < 0 || data == nullptr || len == 0) {
        pthread_mutex_unlock(&sock_mutex);
        return false;
    }

    const char* p = static_cast<const char*>(data);
    size_t offset = 0;

    // 循环发送直到所有数据发送完毕
    while (offset < len) {
        ssize_t sent = send(sock_fd, p + offset, len - offset, MSG_NOSIGNAL);
        if (sent > 0) {
            offset += (size_t)sent;
            continue;
        }
        // 发送失败时的处理
        perror("[TCP] Send failed 发送失败");
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        pthread_mutex_unlock(&sock_mutex);
        return false;
    }

    pthread_mutex_unlock(&sock_mutex);
    return true;
}

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
bool TcpClient::send_string(const std::string& s)
{
    return send_bytes(s.data(), s.size());
}

/*******************************************************************
 * @brief       断开与TCP服务器的连接
 * 
 * @example     tcp_client.disconnect_server();
 * 
 * @note        断开连接并释放Socket资源
 ******************************************************************/
void TcpClient::disconnect_server(void)
{
    pthread_mutex_lock(&sock_mutex);

    if (connected && sock_fd >= 0) {
        close(sock_fd);
        sock_fd = -1;
        connected = false;
        printf("Disconnect 断开连接\n");
    }

    pthread_mutex_unlock(&sock_mutex);
}

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
bool TcpClient::is_connected(void)
{
    pthread_mutex_lock(&sock_mutex);
    bool status = connected;

    pthread_mutex_unlock(&sock_mutex);
    return status;
}


/*******************************************************************
 * [子类] VofaClient: VOFA+ 协议层封装
 * 负责：将发送数据封装为 VOFA+ 的 FireWater 协议格式帧
 ******************************************************************/

VofaClient::VofaClient(void)
    : TcpClient()
{
    // 初始化缓冲区
    memset(firewater_buffer, 0, sizeof(firewater_buffer));
}

VofaClient::~VofaClient(void)
{
}

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
void VofaClient::send_firewater(const char* format, ...)
{
    if (!this->is_connected()) 
        return;

    va_list args;

    // 格式化字符串 (类似 printf)
    va_start(args, format);
    vsnprintf(firewater_buffer, sizeof(firewater_buffer), format, args);
    va_end(args);

    // 发送字符串
    this->send_string(std::string(firewater_buffer));
}


