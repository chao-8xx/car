#ifndef __IPM_AUTO_BIAODING_H__
#define __IPM_AUTO_BIAODING_H__

#include "headfile.h"

#define SEED_HEIGHT         80

class IPMAuto
{
public:
    IPMAuto() = default;
    ~IPMAuto() = default;

    /**
     * @brief 获取透视变换所需的四个关键点
     * 主要流程：拷贝图像、寻找种子点、处理左右边界点、寻找角点等
     */
    void Get_Four_Points(void);
    
    void save_ipm_param(const std::string& filename);                   // 保存ipm参数到json文件
    bool read_ipm_param(const std::string& filename);                   // 从json文件加载ipm参数
    void update_ipm_matrix(const std::vector<cv::Point2f>& src_pts);    // 通过四个角点计算更新透视矩阵
    void display_ipm_image(const Mat& src, Mat& dst);                   // 显示透视变换后的图像


private:
    // 截取标注图片
    void get_image(void);





};

#endif // __IPM_AUTO_BIAODING_H__



