// 使用JsonCpp库进行JSON读写历程

#include "headfile.h"

void save_json(void);

void get_json(void);

int value;

int main() 
{
    value = 100;
    // save_json();     //保存
    get_json();         //读取
    std::cout << value << std::endl;
      
    return 0;
}

void get_json(void)
{
    //创建json数据
    Json::Value json;
    //创建Json风格的读取类
    Json::Reader read;
    //打开文件“data.json”
    std::ifstream file("./data.json");
    //以json风格读出文件到json变量
    read.parse(file, json);
    //关闭文件
    file.close();

    value = json["text"].asInt();
}

void save_json(void)
{
    //创建json数据
    Json::Value json;

    json["text"] = value;

    //打开文件“data.json” 没有会创建
    std::ofstream out("./data.json");
    //创建Json风格的写入类
    Json::StyledWriter write;
    //以json风格写入文件
    out << write.write(json);
    //关闭文件
    out.close();
}