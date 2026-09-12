#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — LCD 控制头文件
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
 * 文件名称：ww_lcd.h
 * 所属模块：wuwu_library
 * 功能描述：LCD 绘制与字体控制头文件
 *
 * 修改记录：
 * 日期        作者    说明
 * 2025-12-6  wuwu    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __WUWU_LCD_H
#define __WUWU_LCD_H

#include "headfile.h"
#include "font.h"

#define     FB_LCD_DEVICE                   "/dev/fb0"
#define     LCD_DEFAULT_BGCOLOR             (RGB565_WHITE)                              // 默认的背景颜色
#define     LCD_DEFAULT_PENCOLOR            (RGB565_BLACK)                              // 默认的画笔颜色
#define     LCD_DEFAULT_DISPLAY_FONT        (LCD_8X16_FONT)                              // 默认的字体模式
#define     func_abs(x)                     ((x) >= 0 ? (x): -(x))

class LCD
{
public:

    LCD(void);
    ~LCD();
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
    int lcd_init(void);

/*******************************************************************
 * @brief   画点
 * 
 * @param   x           横坐标
 * @param   y           纵坐标
 * @param   color       画点颜色
 * 
 * @example lcd.drawPoint(0, 0, 0x0000);
 ******************************************************************/
    void drawPoint(int x, int y, unsigned short color);

/*******************************************************************
 * @brief   清屏
 * 
 * @example lcd.clearScreen();
 ******************************************************************/
    void clearScreen(void);

/*******************************************************************
 * @brief   显示字符
 * 
 * @param   x           字符左上角横坐标
 * @param   y           字符左上角纵坐标
 * @param   dat         字符
 * 
 * @example lcd.showChar(0, 0, '6');
 ******************************************************************/
    void showChar(int x, int y, const char dat);
    void showChinese(int x, int y, const char *chinese);

/*******************************************************************
 * @brief   显示字符串
 * 
 * @param   x           字符串左上角横坐标
 * @param   y           字符串左上角纵坐标
 * @param   dat         字符串
 * 
 * @example lcd.showString(0, 0, "hello world");
 ******************************************************************/
    void showString(int x, int y, const char dat[]);
    void showChineseString(int x, int y, const char *ChineseSentence);

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
    void showInt(int x, int y, int dat, int num);

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
    void showUInt(int x, int y, int dat, int num);

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
    void showDouble (int x, int y, const double dat, unsigned char num, unsigned char pointnum);

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
    void showCVImage(int x, int y, const cv::Mat& image, int target_width, int target_height);


/*******************************************************************
 * @brief   颜色反转
 * 
 * @param   x                   起始横坐标
 * @param   y                   起始纵坐标
 * @param   length              颜色反转区域的长度
 * @param   width               颜色反转区域的宽度
 * 
 * @example lcd.colorChange(0, 0, 40, 20);
 ******************************************************************/
    void colorChange(int x, int y, int length, int width);

    void colorKeep(int x, int y, int length, int width);


private:
    int fd;                                     //设备是否存在
    struct fb_fix_screeninfo fix_screeninfo;    //屏幕固定信息
    struct fb_var_screeninfo var_screeninfo;    //屏幕可变信息
    long int screensize;                        //缓冲帧大小
    char *fbp = nullptr;                        //帧缓冲内存指针

    unsigned short      lcd_bgcolor         =   LCD_DEFAULT_BGCOLOR;            // 背景颜色
    unsigned short      lcd_pencolo         =   LCD_DEFAULT_PENCOLOR;           // 画笔颜色(字体色)
    lcd_font_size_enum  lcd_display_font    =   LCD_DEFAULT_DISPLAY_FONT;       // 显示字体类型

    void drawPixel(int x, int y, unsigned short color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [-32768,32767]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_int_to_str(data_buffer, -300);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
    void func_int_to_str (char *str, int number);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [0,65535]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_uint_to_str(data_buffer, 300);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
    void func_uint_to_str (char *str, unsigned int number);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_double_to_str(data_buffer, 3.1415, 2);                     // 结果输出 data_buffer = "3.14"
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
    void func_double_to_str (char *str, double number, unsigned char point_bit);
};

#endif
