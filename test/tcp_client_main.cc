#include "headfile.h"

/**************************************************************************
 * @brief       TCP 数据发送例程
 * 
 * @note        Creation Time :         2025/12/24
 * @note        Author :                chao_8xx
 * @note        E-mail :                484544537@qq.com
 * 
 * @note        Attention :             本例程展示了如何将数据通过TCP协议发送到PC端的VOFA+服务器:
 *                                      
 *                                      1. 电脑端　VOFA+ 设置（TCP服务器）
 *                                         -　协议: FireWater
 *                                         -　接口: TCP服务端
 *                                         -  端口:设置为2233 (与代码中端口对应)
 *                                      
 *                                      2. 久久派端连接电脑即可 (TCP客户端)
 *                                      
 *                                      3. 发送一路模拟正弦波形数据及字符串
 *                                      
 *                                      4. 所有通道同一时刻的数据，必须在同一次函数发送中一起发送
 *                                         
 ***************************************************************************/

// 配置电脑的 IP 和 Port (根据用户个人电脑设置修改)
#define SERVER_IP       "192.168.1.107"
#define SERVER_PORT     2233

int main(void)
{
    std::cout << ">>> 服务器 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> 服务器 Port: " << SERVER_PORT << std::endl;

    VofaClient client;

    // 连接 TCP 服务器
    while (client.connect_server(SERVER_IP, SERVER_PORT) < 0)
    {
        std::cout << "连接失败, 1秒后重试..." << std::endl;
        usleep(1000000);
    }
    
    float t = 0.0f;
    while (1) 
    {
        // 实际应用中可替换为传感器 / PID / 控制量等数据

        if(client.is_connected()){
            float sin_val = std::sin(t) * 50;  

            // 发送一路波形数据与字符串到VOFA+服务器
            client.send_firewater("ware:%.3f\n", sin_val);     
            client.send_string("Hello, TCP Server!");
        }

        else{
            std::cout << "连接断开, 正在重连..." << std::endl;
            usleep(1000000);
            client.connect_server(SERVER_IP, SERVER_PORT);
        }

        t += 0.5f;
        usleep(50000); 
    }

    return 0;
}


