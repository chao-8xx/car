#include "vofa.h"

#define SERVER_IP       "10.168.1.181"      //定义ip
// #define SERVER_IP       "192.168.0.100"      //定义ip
#define SERVER_PORT     2233                //定义端口
VofaClient client;

int Vofa::vofa_init(void)
{
    std::cout << ">>> 服务器 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> 服务器 Port: " << SERVER_PORT << std::endl;

    while (client.connect_server(SERVER_IP, SERVER_PORT) < 0)
    {
        std::cout << "连接失败, 1秒后重试..." << std::endl;
        usleep(1000000);
    }

    std::cout << "vofa初始化成功" << std::endl;

    return 0;
}

void Vofa::vofa_read(float parameter1, float parameter2, float parameter3, float parameter4, float parameter5, float parameter6)
{
    // 实际应用中可替换为传感器 / PID / 控制量等数据

        if(client.is_connected()){  

            // 发送一组波形数据与字符串到VOFA+服务器
            client.send_firewater("ware:%.02f,%.02f,%.02f,%.02f,%.02f\n", parameter1, parameter2, parameter3, parameter4, parameter5, parameter6);   
            // client.send_string("Hello, TCP Server!");
        }

        else{
            std::cout << "连接断开, 正在重连..." << std::endl;
            // usleep(1000000);
            client.connect_server(SERVER_IP, SERVER_PORT);
        }

        // usleep(50000); 
}

Vofa vofa;


