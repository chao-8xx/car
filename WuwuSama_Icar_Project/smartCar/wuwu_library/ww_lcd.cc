/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — LCD 显示模块
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
 * 文件名称：ww_lcd.cc
 * 所属模块：wuwu_library
 * 功能描述：LCD 显示控制封装
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-6-2  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_lcd.h"

LCD::LCD(void){}

LCD::~LCD()
{ 
    if(fd < 0) return;
    // 解除内存映射
    munmap(fbp, screensize);
    // 关闭设备
    close(fd);
}

/*******************************************************************
 * @brief       显示屏初始化
 * 
 * @return      返回初始化状态
 * @retval      0               初始化成功
 * @retval      -1              初始化失败
 * 
 * @example     //显示屏初始化
 *              if(lcd.lcd_init() < 0) {
 *                  return -1;             
 *              }
 * 
 * @note        不使用此函数直接使用下面函数会报错
 ******************************************************************/
int LCD::lcd_init(void)
{
    fd = open(FB_LCD_DEVICE, O_RDWR);
    if(fd < 0) {
        std::cerr << "Error: cannot open framebuffer device" << std::endl;
        return -1;
    } else {
        if(ioctl(fd, FBIOGET_FSCREENINFO, &fix_screeninfo)) {
            std::cerr << "Error reading fixed information" << std::endl;
            return -1;
        }
        if(ioctl(fd, FBIOGET_VSCREENINFO, &var_screeninfo)) {
            std::cerr << "Error reading variable information" << std::endl;
            return -1;
        }
    }

    /*  获取缓冲帧大小  */
    screensize = fix_screeninfo.smem_len;
    std::cout << "初始化屏幕完成\t" << "屏幕缓冲帧大小:" << screensize << std::endl;

    //映射帧缓冲内存
    fbp = (char *)mmap(nullptr, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(fbp == MAP_FAILED) {
        std::cerr << "Error: failed to map framebuffer memory" << std::endl;
        return -1;
    }

    clearScreen();

    return 0;
}

/*******************************************************************
 * @brief   画点
 * 
 * @param   x           横坐标
 * @param   y           纵坐标
 * @param   color       画点颜色
 * 
 * @example lcd.drawPoint(0, 0, 0x0000);
 ******************************************************************/
void LCD::drawPoint(int x, int y, unsigned short color) 
{
    drawPixel(x, y, color);
}

/*******************************************************************
 * @brief   清屏
 * 
 * @example lcd.clearScreen();
 ******************************************************************/
void LCD::clearScreen(void) 
{
    unsigned short *buffer = (unsigned short*)fbp;
    for(int i = 0; i < screensize / 2; i++) {
        buffer[i] = lcd_bgcolor;
    }
}

/*******************************************************************
 * @brief   显示字符
 * 
 * @param   x           字符左上角横坐标
 * @param   y           字符左上角纵坐标
 * @param   dat         字符
 * 
 * @example lcd.showChar(0, 0, '6');
 ******************************************************************/
void LCD::showChar(int x, int y, const char dat) 
{
    switch(lcd_display_font)
    {
        case LCD_6X8_FONT:
        {
            const unsigned char *font = ascii_font_6x8[dat - 32];
            for(int col = 0; col < 6; col++) {
                unsigned char line = font[col];
                for(int row = 0; row < 8; row++) {
                    bool pixel = line & (0x80 >> row);
                    drawPixel(x + col, y + 7 - row, pixel ? lcd_pencolo : lcd_bgcolor);
                }

            }
        }
        break;
        case LCD_8X16_FONT:
        {
            const unsigned char *font = ascii_font_8x16[dat - 32];
            for(int col = 0; col < 8; col++) {
                unsigned char upper = font[col];        // 上半部分（行0-7）
                unsigned char lower = font[col + 8];    // 下半部分（行8-15）
                for(int row = 0; row < 8; row++) {
                    bool pixel = upper & (0x80 >> row);
                    drawPixel(x + col, y + 7 - row, pixel ? lcd_pencolo : lcd_bgcolor);
                }
                for(int row = 0; row < 8; row++) {
                    bool pixel = lower & (0x80 >> row);
                    drawPixel(x + col, y + 15 - row, pixel ? lcd_pencolo : lcd_bgcolor);
                }
            }
        }
        break;
    }
}

void LCD::showChinese(int x, int y, const char *chinese)
{
    //定义数字索引，默认值设为未找到时的索引（这里用 0，对应“，”）
    int index = 0;
    
    //函数内部自动完成 汉字字符->数字索引 的转换
    if (chinese != NULL) { // 先判断传入的汉字字符串不为空，避免崩溃
        if (std::strcmp(chinese, "，") == 0) {
            index = 1; // 自动把 "，" 映射成数字 1
        } else if (std::strcmp(chinese, "。") == 0) {
            index = 2; // 自动把 "。" 映射成数字 2
        } else if (std::strcmp(chinese, "大") == 0) {
            index = 3; // 自动把 "大" 映射成数字 3
        } else if (std::strcmp(chinese, "小") == 0) {
            index = 4; // 自动把 "小" 映射成数字 4
        } else if (std::strcmp(chinese, "世") == 0) {
            index = 5; // 自动把 "世" 映射成数字 5
        } else if (std::strcmp(chinese, "界") == 0) {
            index = 6; // 自动把 "界" 映射成数字 6
        } else if (std::strcmp(chinese, "冯") == 0) {
            index = 7; // 自动把 "冯" 映射成数字 7
        } else if (std::strcmp(chinese, "亚") == 0) {
            index = 8; // 自动把 "亚" 映射成数字 8
        } else if (std::strcmp(chinese, "超") == 0) {
            index = 9; // 自动把 "超" 映射成数字 9
        } else if (std::strcmp(chinese, "控") == 0) {
            index = 10; // 自动把 "控" 映射成数字 10
        } else if (std::strcmp(chinese, "制") == 0) {
            index = 11; // 自动把 "制" 映射成数字 11
        } else if (std::strcmp(chinese, "图") == 0) {
            index = 12; // 自动把 "图" 映射成数字 12
        } else if (std::strcmp(chinese, "像") == 0) {
            index = 13; // 自动把 "像" 映射成数字 13
        } else if (std::strcmp(chinese, "识") == 0) {
            index = 14; // 自动把 "识" 映射成数字 14
        } else if (std::strcmp(chinese, "别") == 0) {
            index = 15; // 自动把 "别" 映射成数字 15
        } else if (std::strcmp(chinese, "左") == 0) {
            index = 16; // 自动把 "左" 映射成数字 16
        } else if (std::strcmp(chinese, "右") == 0) {
            index = 17; // 自动把 "右" 映射成数字 17
        } else if (std::strcmp(chinese, "电") == 0) {
            index = 18; // 自动把 "电" 映射成数字 18
        } else if (std::strcmp(chinese, "机") == 0) {
            index = 19; // 自动把 "机" 映射成数字 19
        } else if (std::strcmp(chinese, "菜") == 0) {
            index = 20; // 自动把 "菜" 映射成数字 20
        } else if (std::strcmp(chinese, "单") == 0) {
            index = 21; // 自动把 "单" 映射成数字 21
        } else if (std::strcmp(chinese, "参") == 0) {
            index = 22; // 自动把 "参" 映射成数字 22
        } else if (std::strcmp(chinese, "数") == 0) {
            index = 23; // 自动把 "数" 映射成数字 23
        } else if (std::strcmp(chinese, "步") == 0) {
            index = 24; // 自动把 "步" 映射成数字 24
        } else if (std::strcmp(chinese, "长") == 0) {
            index = 25; // 自动把 "长" 映射成数字 25
        } else if (std::strcmp(chinese, "因") == 0) {
            index = 26; // 自动把 "因" 映射成数字 26
        } else if (std::strcmp(chinese, "子") == 0) {
            index = 27; // 自动把 "子" 映射成数字 27
        } else if (std::strcmp(chinese, "的") == 0) {
            index = 28; // 自动把 "的" 映射成数字 28
        } else if (std::strcmp(chinese, "是") == 0) {
            index = 29; // 自动把 "是" 映射成数字 29
        } else if (std::strcmp(chinese, "否") == 0) {
            index = 30; // 自动把 "否" 映射成数字 30
        } else if (std::strcmp(chinese, "方") == 0) {
            index = 31; // 自动把 "方" 映射成数字 31
        } else if (std::strcmp(chinese, "案") == 0) {
            index = 32; // 自动把 "案" 映射成数字 32
        } else if (std::strcmp(chinese, "元") == 0) {
            index = 33; // 自动把 "元" 映射成数字 33
        } else if (std::strcmp(chinese, "素") == 0) {
            index = 34; // 自动把 "素" 映射成数字 34
        } else if (std::strcmp(chinese, "管") == 0) {
            index = 35; // 自动把 "管" 映射成数字 35
        } else if (std::strcmp(chinese, "理") == 0) {
            index = 36; // 自动把 "理" 映射成数字 36
        } else if (std::strcmp(chinese, "相") == 0) {
            index = 37; // 自动把 "相" 映射成数字 37
        } else if (std::strcmp(chinese, "翻") == 0) {
            index = 38; // 自动把 "翻" 映射成数字 38
        } else if (std::strcmp(chinese, "调") == 0) {
            index = 39; // 自动把 "调" 映射成数字 39
        } else if (std::strcmp(chinese, "整") == 0) {
            index = 40; // 自动把 "整" 映射成数字 40
        } else if (std::strcmp(chinese, "视") == 0) {
            index = 41; // 自动把 "视" 映射成数字 41
        } else if (std::strcmp(chinese, "频") == 0) {
            index = 42; // 自动把 "频" 映射成数字 42
        } else if (std::strcmp(chinese, "数") == 0) {
            index = 43; // 自动把 "数" 映射成数字 43
        } else if (std::strcmp(chinese, "据") == 0) {
            index = 44; // 自动把 "据" 映射成数字 44
        } else if (std::strcmp(chinese, "回") == 0) {
            index = 45; // 自动把 "回" 映射成数字 45
        } else if (std::strcmp(chinese, "放") == 0) {
            index = 46; // 自动把 "放" 映射成数字 46
        } else if (std::strcmp(chinese, "退") == 0) {
            index = 47; // 自动把 "退" 映射成数字 47
        } else if (std::strcmp(chinese, "出") == 0) {
            index = 48; // 自动把 "出" 映射成数字 48
        } else if (std::strcmp(chinese, "发") == 0) {
            index = 49; // 自动把 "发" 映射成数字 49
        } else if (std::strcmp(chinese, "咯") == 0) {
            index = 50; // 自动把 "咯" 映射成数字 50
        } else if (std::strcmp(chinese, "逆") == 0) {
            index = 51; // 自动把 "逆" 映射成数字 51
        } else if (std::strcmp(chinese, "透") == 0) {
            index = 52; // 自动把 "透" 映射成数字 52
        } else if (std::strcmp(chinese, "显") == 0) {
            index = 53; // 自动把 "显" 映射成数字 53
        } else if (std::strcmp(chinese, "示") == 0) {
            index = 54; // 自动把 "示" 映射成数字 54
        } else if (std::strcmp(chinese, "皮") == 0) {
            index = 55; // 自动把 "P" 映射成数字 55
        } else if (std::strcmp(chinese, "埃") == 0) {
            index = 56; // 自动把 "I" 映射成数字 56
        } else if (std::strcmp(chinese, "弟") == 0) {
            index = 57; // 自动把 "D" 映射成数字 57
        } else if (std::strcmp(chinese, "克") == 0) {
            index = 58; // 自动把 "K" 映射成数字 58
        } else if (std::strcmp(chinese, "差") == 0) {
            index = 59; // 自动把 "差" 映射成数字 59
        } else if (std::strcmp(chinese, "速") == 0) {
            index = 60; // 自动把 "速" 映射成数字 60
        } else if (std::strcmp(chinese, "环") == 0) {
            index = 61; // 自动把 "环" 映射成数字 61
        } else if (std::strcmp(chinese, "变") == 0) {
            index = 62; // 自动把 "变" 映射成数字 62
        } else if (std::strcmp(chinese, "换") == 0) {
            index = 63; // 自动把 "换" 映射成数字 63
        } else if (std::strcmp(chinese, "截") == 0) {
            index = 64; // 自动把 "截" 映射成数字 64
        } else if (std::strcmp(chinese, "拉") == 0) {
            index = 65; // 自动把 "拉" 映射成数字 65
        } else if (std::strcmp(chinese, "伸") == 0) {
            index = 66; // 自动把 "伸" 映射成数字 66
        } else if (std::strcmp(chinese, "器") == 0) {
            index = 67; // 自动把 "器" 映射成数字 67
        } else if (std::strcmp(chinese, "系") == 0) {
            index = 68; // 自动把 "系" 映射成数字 68
        } else if (std::strcmp(chinese, "观") == 0) {
            index = 69; // 自动把 "观" 映射成数字 69
        } else if (std::strcmp(chinese, "测") == 0) {
            index = 70; // 自动把 "测" 映射成数字 70
        } else if (std::strcmp(chinese, "跟") == 0) {
            index = 71; // 自动把 "跟" 映射成数字 71
        } else if (std::strcmp(chinese, "踪") == 0) {
            index = 72; // 自动把 "踪" 映射成数字 72
        } else if (std::strcmp(chinese, "补") == 0) {
            index = 73; // 自动把 "补" 映射成数字 73
        } else if (std::strcmp(chinese, "偿") == 0) {
            index = 74; // 自动把 "偿" 映射成数字 74
        } else if (std::strcmp(chinese, "自") == 0) {
            index = 75; // 自动把 "自" 映射成数字 75
        } else if (std::strcmp(chinese, "抗") == 0) {
            index = 76; // 自动把 "抗" 映射成数字 76
        } else if (std::strcmp(chinese, "扰") == 0) {
            index = 77; // 自动把 "扰" 映射成数字 77
        } else if (std::strcmp(chinese, "前") == 0) {
            index = 78; // 自动把 "前" 映射成数字 78
        } else if (std::strcmp(chinese, "瞻") == 0) {
            index = 79; // 自动把 "瞻" 映射成数字 79
        } else if (std::strcmp(chinese, "考") == 0) {
            index = 80; // 自动把 "考" 映射成数字 80
        } else if (std::strcmp(chinese, "范") == 0) {
            index = 81; // 自动把 "范" 映射成数字 81
        } else if (std::strcmp(chinese, "围") == 0) {
            index = 82; // 自动把 "围" 映射成数字 82
        } else if (std::strcmp(chinese, "亮") == 0) {
            index = 83; // 自动把 "亮" 映射成数字 83
        } else if (std::strcmp(chinese, "度") == 0) {
            index = 84; // 自动把 "度" 映射成数字 84
        } else if (std::strcmp(chinese, "对") == 0) {
            index = 85; // 自动把 "对" 映射成数字 85
        } else if (std::strcmp(chinese, "比") == 0) {
            index = 86; // 自动把 "比" 映射成数字 86
        } else if (std::strcmp(chinese, "锐") == 0) {
            index = 87; // 自动把 "锐" 映射成数字 87
        } else if (std::strcmp(chinese, "饱") == 0) {
            index = 88; // 自动把 "饱" 映射成数字 88
        } else if (std::strcmp(chinese, "和") == 0) {
            index = 89; // 自动把 "和" 映射成数字 89
        } else if (std::strcmp(chinese, "增") == 0) {
            index = 90; // 自动把 "增" 映射成数字 90
        } else if (std::strcmp(chinese, "益") == 0) {
            index = 91; // 自动把 "益" 映射成数字 91
        } else if (std::strcmp(chinese, "曝") == 0) {
            index = 92; // 自动把 "曝" 映射成数字 92
        } else if (std::strcmp(chinese, "光") == 0) {
            index = 93; // 自动把 "光" 映射成数字 93
        } else if (std::strcmp(chinese, "整") == 0) {
            index = 94; // 自动把 "整" 映射成数字 94
        } else if (std::strcmp(chinese, "浮") == 0) {
            index = 95; // 自动把 "浮" 映射成数字 95
        } else if (std::strcmp(chinese, "点") == 0) {
            index = 96; // 自动把 "点" 映射成数字 96
        } else if (std::strcmp(chinese, "形") == 0) {
            index = 97; // 自动把 "形" 映射成数字 97
        } else if (std::strcmp(chinese, "角") == 0) {
            index = 98; // 自动把 "角" 映射成数字 98
        } else if (std::strcmp(chinese, "目") == 0) {
            index = 99; // 自动把 "目" 映射成数字 99
        } else if (std::strcmp(chinese, "标") == 0) {
            index = 100; // 自动把 "标" 映射成数字 100
        } else if (std::strcmp(chinese, "输") == 0) {
            index = 101; // 自动把 "输" 映射成数字 101
        } else if (std::strcmp(chinese, "出") == 0) {
            index = 102; // 自动把 "出" 映射成数字 102
        } else if (std::strcmp(chinese, "行") == 0) {
            index = 103; // 自动把 "行" 映射成数字 103
        } else if (std::strcmp(chinese, "驶") == 0) {
            index = 104; // 自动把 "驶" 映射成数字 104
        } else if (std::strcmp(chinese, "距") == 0) {
            index = 105; // 自动把 "距" 映射成数字 105
        } else if (std::strcmp(chinese, "离") == 0) {
            index = 106; // 自动把 "离" 映射成数字 106
        } else if (std::strcmp(chinese, "陀") == 0) {
            index = 107; // 自动把 "陀" 映射成数字 107
        } else if (std::strcmp(chinese, "螺") == 0) {
            index = 108; // 自动把 "螺" 映射成数字 108
        } else if (std::strcmp(chinese, "仪") == 0) {
            index = 109; // 自动把 "仪" 映射成数字 109
        } else if (std::strcmp(chinese, "零") == 0) {
            index = 110; // 自动把 "零" 映射成数字 110
        } else if (std::strcmp(chinese, "漂") == 0) {
            index = 111; // 自动把 "漂" 映射成数字 111
        } else if (std::strcmp(chinese, "校") == 0) {
            index = 112; // 自动把 "校" 映射成数字 112
        } else if (std::strcmp(chinese, "准") == 0) {
            index = 113; // 自动把 "准" 映射成数字 113
        } else if (std::strcmp(chinese, "解") == 0) {
            index = 114; // 自动把 "解" 映射成数字 114
        } else if (std::strcmp(chinese, "算") == 0) {
            index = 115; // 自动把 "算" 映射成数字 115
        } else if (std::strcmp(chinese, "周") == 0) {
            index = 116; // 自动把 "周" 映射成数字 116
        } else if (std::strcmp(chinese, "期") == 0) {
            index = 117; // 自动把 "期" 映射成数字 117
        } else if (std::strcmp(chinese, "内") == 0) {
            index = 118; // 自动把 "内" 映射成数字 118
        } else if (std::strcmp(chinese, "外") == 0) {
            index = 119; // 自动把 "外" 映射成数字 119
        } else if (std::strcmp(chinese, "半") == 0) {
            index = 120; // 自动把 "半" 映射成数字 120
        } else if (std::strcmp(chinese, "宽") == 0) {
            index = 121; // 自动把 "宽" 映射成数字 121
        } else if (std::strcmp(chinese, "标") == 0) {
            index = 122; // 自动把 "标" 映射成数字 122
        } else if (std::strcmp(chinese, "注") == 0) {
            index = 123; // 自动把 "注" 映射成数字 123
        } else if (std::strcmp(chinese, "临") == 0) {
            index = 124; // 自动把 "临" 映射成数字 124
        } else if (std::strcmp(chinese, "时") == 0) {
            index = 125; // 自动把 "时" 映射成数字 125
        } else if (std::strcmp(chinese, "馈") == 0) {
            index = 126; // 自动把 "馈" 映射成数字 126
        } else if (std::strcmp(chinese, "决") == 0) {
            index = 127; // 自动把 "决" 映射成数字 127
        } else if (std::strcmp(chinese, "策") == 0) {
            index = 128; // 自动把 "策" 映射成数字 128
        } else if (std::strcmp(chinese, "正") == 0) {
            index = 129; // 自动把 "正" 映射成数字 129
        } else if (std::strcmp(chinese, "常") == 0) {
            index = 130; // 自动把 "常" 映射成数字 130
        } else if (std::strcmp(chinese, "十") == 0) {
            index = 131; // 自动把 "十" 映射成数字 131
        } else if (std::strcmp(chinese, "字") == 0) {
            index = 132; // 自动把 "字" 映射成数字 132
        } else if (std::strcmp(chinese, "圆") == 0) {
            index = 133; // 自动把 "圆" 映射成数字 133
        } else if (std::strcmp(chinese, "坡") == 0) {
            index = 134; // 自动把 "坡" 映射成数字 134
        } else if (std::strcmp(chinese, "道") == 0) {
            index = 135; // 自动把 "道" 映射成数字 135
        } else if (std::strcmp(chinese, "障") == 0) {
            index = 136; // 自动把 "障" 映射成数字 136
        } else if (std::strcmp(chinese, "碍") == 0) {
            index = 137; // 自动把 "碍" 映射成数字 137
        } else if (std::strcmp(chinese, "照") == 0) {
            index = 138; // 自动把 "照" 映射成数字 138
        } else if (std::strcmp(chinese, "片") == 0) {
            index = 139; // 自动把 "片" 映射成数字 139
        } else if (std::strcmp(chinese, "化") == 0) {
            index = 140; // 自动把 "化" 映射成数字 140
        } else if (std::strcmp(chinese, "直") == 0) {
            index = 141; // 自动把 "直" 映射成数字 141
        } else if (std::strcmp(chinese, "弯") == 0) {
            index = 142; // 自动把 "弯" 映射成数字 142
        } else if (std::strcmp(chinese, "弱") == 0) {
            index = 143; // 自动把 "弱" 映射成数字 143
        } else if (std::strcmp(chinese, "平") == 0) {
            index = 144; // 自动把 "平" 映射成数字 144
        } else if (std::strcmp(chinese, "滑") == 0) {
            index = 145; // 自动把 "滑" 映射成数字 145
        } else if (std::strcmp(chinese, "修") == 0) {
            index = 146; // 自动把 "修" 映射成数字 146
        } else if (std::strcmp(chinese, "切") == 0) {
            index = 147; // 自动把 "切" 映射成数字 147
        } else if (std::strcmp(chinese, "模") == 0) {
            index = 148; // 自动把 "模" 映射成数字 148
        } else if (std::strcmp(chinese, "式") == 0) {
            index = 149; // 自动把 "式" 映射成数字 149
        } else if (std::strcmp(chinese, "型") == 0) {
            index = 150; // 自动把 "型" 映射成数字 150
        } else if (std::strcmp(chinese, "队") == 0) {
            index = 151; // 自动把 "队" 映射成数字 151
        } else if (std::strcmp(chinese, "列") == 0) {
            index = 152; // 自动把 "列" 映射成数字 152
        } else if (std::strcmp(chinese, "绕") == 0) {
            index = 153; // 自动把 "绕" 映射成数字 153
        } else if (std::strcmp(chinese, "开") == 0) {
            index = 154; // 自动把 "开" 映射成数字 154
        } else if (std::strcmp(chinese, "一") == 0) {
            index = 155; // 自动把 "一" 映射成数字 155
        } else if (std::strcmp(chinese, "二") == 0) {
            index = 156; // 自动把 "二" 映射成数字 156
        } else if (std::strcmp(chinese, "三") == 0) {
            index = 157; // 自动把 "三" 映射成数字 157
        } else if (std::strcmp(chinese, "四") == 0) {
            index = 158; // 自动把 "四" 映射成数字 158
        } else if (std::strcmp(chinese, "五") == 0) {
            index = 159; // 自动把 "五" 映射成数字 159
        } else if (std::strcmp(chinese, "六") == 0) {
            index = 160; // 自动把 "六" 映射成数字 160
        } else if (std::strcmp(chinese, "七") == 0) {
            index = 161; // 自动把 "七" 映射成数字 161
        } else if (std::strcmp(chinese, "八") == 0) {
            index = 162; // 自动把 "八" 映射成数字 162
        } else if (std::strcmp(chinese, "九") == 0) {
            index = 163; // 自动把 "九" 映射成数字 163
        } else if (std::strcmp(chinese, "百") == 0) {
            index = 164; // 自动把 "百" 映射成数字 164
        } else if (std::strcmp(chinese, "强") == 0) {
            index = 165; // 自动把 "强" 映射成数字 165
        } else if (std::strcmp(chinese, "锁") == 0) {
            index = 166; // 自动把 "锁" 映射成数字 166
        } else if (std::strcmp(chinese, "定") == 0) {
            index = 167; // 自动把 "定" 映射成数字 167
        } else if (std::strcmp(chinese, "安") == 0) {
            index = 168; // 自动把 "安" 映射成数字 168
        } else if (std::strcmp(chinese, "全") == 0) {
            index = 169; // 自动把 "全" 映射成数字 169
        } else if (std::strcmp(chinese, "过") == 0) {
            index = 170; // 自动把 "过" 映射成数字 170
        } else if (std::strcmp(chinese, "渡") == 0) {
            index = 171; // 自动把 "渡" 映射成数字 171
        } else if (std::strcmp(chinese, "加") == 0) {
            index = 172; // 自动把 "加" 映射成数字 172
        } else if (std::strcmp(chinese, "减") == 0) {
            index = 173; // 自动把 "减" 映射成数字 173
        } else if (std::strcmp(chinese, "量") == 0) {
            index = 174; // 自动把 "加" 映射成数字 174
        } else if (std::strcmp(chinese, "类") == 0) {
            index = 175; // 自动把 "减" 映射成数字 175
        }


    }
    else{index = 0;} //若指针为空，则输出，默认符号

    const unsigned char *font = Chinese_font_16x16[index];
            for(int col = 0; col < 16; col++) 
            {
                unsigned char upper = font[col];        // 上半部分（行0-15）
                unsigned char lower = font[col + 16];    // 下半部分（行16-32）
                for(int row = 0; row < 8; row++) {
                    bool pixel = upper & (0x80 >> row);
                    drawPixel(x + col, y + 7 - row, pixel ? lcd_pencolo : lcd_bgcolor);
                }
                for(int row = 0; row < 8; row++) {
                    bool pixel = lower & (0x80 >> row);
                    drawPixel(x + col, y + 15 - row, pixel ? lcd_pencolo : lcd_bgcolor);
                }
            }
            
}

/*******************************************************************
 * @brief   显示字符串
 * 
 * @param   x           字符串左上角横坐标
 * @param   y           字符串左上角纵坐标
 * @param   dat         字符串
 * 
 * @example lcd.showString(0, 0, "hello world");
 ******************************************************************/
void LCD::showString(int x, int y, const char dat[])
{
    unsigned short j = 0;
    while('\0' != dat[j])
    {
        switch(lcd_display_font)
        {
            case LCD_6X8_FONT:   showChar(x + 6 * j, y, dat[j]); break;
            case LCD_8X16_FONT:  showChar(x + 8 * j, y, dat[j]); break;
            default: break;
        }
        j++;
    }
}

//实现连续显示中文字符串的函数
void LCD::showChineseString(int x, int y, const char *ChineseSentence)
{
    //防止空指针报错
    if (ChineseSentence == nullptr || *ChineseSentence == '\0')
    {
        return;
    }

    //读取输入的中文字符串的长度，并处理
    int str_len = strlen(ChineseSentence);
    if(str_len % 3 == 2)
    {
        str_len -= 2;   //剔除最后2个无效字节，避免越界
    }
    else if(str_len % 3 == 1)
    {
        str_len -= 1;   //剔除最后1个无效字节，避免越界
    }
    int Chinese_count = str_len / 3;    //中文字符总个数
    if (Chinese_count == 0)     //中文字符串长度为0时，不执行
    {
        return;
    }

    //创建数组
    char single_word[4] = {0};      //存放单个汉字字节（包含终止符号）的小数组
    char total_arr[Chinese_count * 4] = {0};        //存放字符串内所有汉字字节（包含终止符号）的大树组
    const char *Chinese_pointers[Chinese_count];        //用于识别汉字的指针数组，存放汉字指针，用于显示汉字
    int currentX = x;       //记录当前显示的X坐标，实现横向排列
    const char *p = ChineseSentence;        //将输入的汉字字符串指针赋值给指针p

    //将每个汉字字符包含终止符存入用于存放单字字节的小数组，再分别传入所有汉字字节包含终符的大数组
    for (int i = 0; i < Chinese_count; i++)
    {
        single_word[0] = *p++;
        single_word[1] = *p++;
        single_word[2] = *p++;
        single_word[3] = '\0';      //每存入一个汉字字节，在尾部加入'0\',用于识别单个汉字

        int buf_offset = i * 4;
        for (int j = 0; j < 4; j++)     //将包含终止符的每个汉字字节依次传入存放所有汉字字节的大数组
        {
            total_arr[buf_offset + j] = single_word[j];
        }

    }

    //将大数组（包含终止符号）每一个汉字首字节的指针传递给指针数组，用于分别识别汉字
    int dex = 0;
    for(int i = 0; i < Chinese_count; i++)
    {
        Chinese_pointers[i] = &total_arr[dex];
        dex += 4;
    }

    //显示中文字符串
    for (int i = 0; i < Chinese_count; i++)
    {
        showChinese(currentX, y, Chinese_pointers[i]);      // 直接传递指针数组中的地址
        
        currentX += 16;     //每个字占16像素，右移16像素，避免重叠
    }
    
}

/*******************************************************************
 * @brief   显示有符号整形
 * 
 * @param   x           整形左上角横坐标
 * @param   y           整形左上角纵坐标
 * @param   dat         整形
 * @param   num         需要显示的位数
 * 
 * @example lcd.showInt(0, 0, dat, 5);
 ******************************************************************/
void LCD::showInt(int x, int y, int dat, int num)
{
    int dat_temp = dat;
    int offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num + 1);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    showString(x, y, (const char *)&data_buffer);
}

/*******************************************************************
 * @brief   显示无符号整形
 * 
 * @param   x           无符号整形左上角横坐标
 * @param   y           无符号整形左上角纵坐标
 * @param   dat         无符号整形
 * @param   num         需要显示的位数
 * 
 * @example lcd.showUInt(0, 0, dat, 5);
 ******************************************************************/
void LCD::showUInt(int x, int y, int dat, int num)
{
    unsigned int dat_temp = dat;
    int offset = 1;
    char data_buffer[12];
    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    showString(x, y, (const char *)&data_buffer);
}

/*******************************************************************
 * @brief   显示浮点数
 * 
 * @param   x           浮点数左上角横坐标
 * @param   y           浮点数左上角纵坐标
 * @param   dat         浮点数
 * @param   num         需要显示的位数
 * 
 * @example lcd.showDouble(0, 0, dat, 3, 3);
 ******************************************************************/ 
void LCD::showDouble (int x, int y, const double dat, unsigned char num, unsigned char pointnum)
{
    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];
    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num + pointnum + 2);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    showString(x, y, (const char *)&data_buffer);
}

/*******************************************************************
 * @brief   显示OpenCV Mat图像
 * 
 * @param   x                   图像左上角横坐标
 * @param   y                   图像左上角纵坐标
 * @param   image               需要显示的图像
 * @param   target_width        显示宽度
 * @param   target_height       显示高度
 * 
 * @example lcd.showCVImage(0, 0, img, 128, 80);
 ******************************************************************/
void LCD::showCVImage(int x, int y, const cv::Mat& image, int target_width, int target_height)
{
    if(image.empty() || target_width <= 0 || target_height <= 0) return;

    // 非等比缩放处理
    cv::Mat resized_img;
    cv::resize(image, resized_img, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);

    // 颜色空间转换 (BGR -> RGB565)
    cv::Mat rgb565;
    if(resized_img.channels() == 3) {
        cv::Mat rgb_img;
        cv::cvtColor(resized_img, rgb_img, cv::COLOR_BGR2RGB);
        rgb565.create(rgb_img.size(), CV_16UC1);

        for(int i = 0; i < rgb_img.rows; ++i) {
            const cv::Vec3b* ptr = rgb_img.ptr<cv::Vec3b>(i);
            unsigned short* dst = rgb565.ptr<unsigned short>(i);
            
            for(int j = 0; j < rgb_img.cols; ++j) {
                // RGB888转RGB565
                unsigned char r = ptr[j][0] >> 3;   // 5-bit
                unsigned char g = ptr[j][1] >> 2;   // 6-bit 
                unsigned char b = ptr[j][2] >> 3;   // 5-bit
                dst[j] = (r << 11) | (g << 5) | b;
            }
        }
    } else if(resized_img.channels() == 1) {
        // 灰度图直接转RGB565（伪彩色）
        rgb565.create(resized_img.size(), CV_16UC1);
        
        for(int i = 0; i < resized_img.rows; ++i) {
            const uchar* src = resized_img.ptr<uchar>(i);
            unsigned short* dst = rgb565.ptr<unsigned short>(i);
            
            for(int j = 0; j < resized_img.cols; ++j) {
                uchar gray = src[j];
                // 将灰度值均匀分配到RGB通道（模拟灰度显示）
                unsigned short r = (gray >> 3) & 0x1F;  // 5-bit红
                unsigned short g = (gray >> 2) & 0x3F;  // 6-bit绿
                unsigned short b = (gray >> 3) & 0x1F;  // 5-bit蓝
                dst[j] = (r << 11) | (g << 5) | b;
            }
        }
    } else {
        std::cerr << "Unsupported image format" << std::endl;
        return;
    }

    // 边界裁剪处理
    const int screen_width = var_screeninfo.xres;
    const int screen_height = var_screeninfo.yres;
    
    const int draw_width = std::min(target_width, screen_width - x);
    const int draw_height = std::min(target_height, screen_height - y);
    
    if(draw_width <= 0 || draw_height <= 0) return;

    // 直接操作帧缓冲内存
    for(int iy = 0; iy < draw_height; ++iy) {
        const unsigned short* src = rgb565.ptr<unsigned short>(iy);
        unsigned short* dst = reinterpret_cast<unsigned short*>(
            fbp + (y + iy + var_screeninfo.yoffset) * fix_screeninfo.line_length
        ) + x + var_screeninfo.xoffset;
        
        std::memcpy(dst, src, draw_width * sizeof(unsigned short));
    }
}

void LCD::drawPixel(int x, int y, unsigned short color) 
{
    if(x >= 0 && x < var_screeninfo.xres && y >= 0 && y < var_screeninfo.yres) {
        long offset = (x + var_screeninfo.xoffset) * (var_screeninfo.bits_per_pixel / 8) + 
                    (y + var_screeninfo.yoffset) * fix_screeninfo.line_length;
        *((unsigned short*)(fbp + offset)) = color;
    }
}

//颜色反转
void LCD::colorChange(int x, int y, int length, int width)
{
    //确保整个区域都在屏幕有效范围内
    if (x < 0 || y < 0 || 
        x + length > var_screeninfo.xres || 
        y + width > var_screeninfo.yres)
    {
        return;     //区域超出屏幕边界，直接返回避免越界
    }

    //遍历区域内的每一个像素
    for (int dy = 0; dy < width; dy++)
    {
        for (int dx = 0; dx < length; dx++)
        {
            // 计算当前像素的坐标
            int current_x = x + dx;
            int current_y = y + dy;

            // 计算该像素在显存中的字节偏移量
            long offset = (current_x + var_screeninfo.xoffset) * (var_screeninfo.bits_per_pixel / 8) + 
                    (current_y + var_screeninfo.yoffset) * fix_screeninfo.line_length;

            // 读取原始颜色值
            unsigned short original_color = *((unsigned short*)(fbp + offset));

            //交换背景色与画笔色
            if (original_color == lcd_bgcolor)
            {
                *((unsigned short*)(fbp + offset)) = lcd_pencolo;
            }
            else if(original_color == lcd_pencolo)
            {
                *((unsigned short*)(fbp + offset)) = lcd_bgcolor;
            }
        }

    }

}

//颜色保持
void LCD::colorKeep(int x, int y, int length, int width)
{
    //确保整个区域都在屏幕有效范围内
    if (x < 0 || y < 0 || 
        x + length > var_screeninfo.xres || 
        y + width > var_screeninfo.yres)
    {
        return;     //区域超出屏幕边界，直接返回避免越界
    }

    //遍历区域内的每一个像素
    for (int dy = 0; dy < width; dy++)
    {
        for (int dx = 0; dx < length; dx++)
        {
            // 计算当前像素的坐标
            int current_x = x + dx;
            int current_y = y + dy;

            // 计算该像素在显存中的字节偏移量
            long offset = (current_x + var_screeninfo.xoffset) * (var_screeninfo.bits_per_pixel / 8) + 
                    (current_y + var_screeninfo.yoffset) * fix_screeninfo.line_length;

            // 读取原始颜色值
            unsigned short original_color = *((unsigned short*)(fbp + offset));

            //保持背景色与画笔色
            if (original_color == lcd_bgcolor)
            {
                *((unsigned short*)(fbp + offset)) = lcd_bgcolor;
            }
            else if(original_color == lcd_pencolo)
            {
                *((unsigned short*)(fbp + offset)) = lcd_pencolo;
            }
        }

    }

}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
void LCD::func_int_to_str (char *str, int number)
{
    unsigned char data_temp[16];                                                        // 缓冲区
    unsigned char bit = 0;                                                              // 数字位数
    int number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // 负数
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // 或者这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            number_temp = number % 10;
            data_temp[bit ++] = func_abs(number_temp);                          // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

void LCD::func_uint_to_str (char *str, unsigned int number)
{
    char data_temp[16];                                                         // 缓冲区
    unsigned char bit = 0;                                                      // 数字位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[bit ++] = (number % 10);                                  // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

void LCD::func_double_to_str (char *str, double number, unsigned char point_bit)
{
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[12];                                                          // 整数字符缓冲
    int data_temp_point[9];                                                     // 小数字符缓冲
    unsigned char bit = point_bit;                                              // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
        if(point_bit != 0)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
                *str = '0';
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
    }while(0);
}
#pragma GCC diagnostic pop

