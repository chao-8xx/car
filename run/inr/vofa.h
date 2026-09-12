#ifndef __VOFA_H
#define __VOFA_H
#include "headfile.h"

//关于使用vofa相关的类
class Vofa{
public:
    int vofa_init(void);                //向vofa发送数据的初始化
    void vofa_read(float parameter1, float parameter2, float parameter3, float parameter4, float parameter5, float parameter6);     //vofa读取数据

private:
    

};

extern Vofa vofa;
extern VofaClient client;

#endif

