#ifndef TRACKING_BASE_H
#define TRACKING_BASE_H

#include <opencv2/opencv.hpp>
#include <vector>

#define LCD_FULL_WIDTH  128
#define LCD_FULL_HEIGHT  160
#define LCD_WIDTH   100
#define LCD_HEIGHT   75

#define CAMERA_WIDTH   320
#define CAMERA_HEIGHT  240

#define TRACK_WIDTH     80
#define TRACK_HEIGHT    60

#define CAM_WIDTH   TRACK_WIDTH
#define CAM_HEIGHT  TRACK_HEIGHT
#define CAM_FPS     120

#define OTSU_jiange 3                            // 二值化几帧一次处理

// 图像有效区域
// #define ROI_TOP     2                           // 顶部有效区域行数索引
// #define ROI_BOTTOM  70                           // 底部有效区域行数索引
#define ROI_TOP     10                          // 顶部有效区域行数索引
#define ROI_BOTTOM  50                          // 底部有效区域行数索引
#define ROI_HEIGHT  (ROI_BOTTOM - ROI_TOP)      // 有效区域高度

#define ROI_LEFT    3                           // 左侧有效区域列数索引
#define ROI_RIGHT   77                          // 右侧有效区域列数索引
#define ROI_WIDTH   (ROI_RIGHT - ROI_LEFT)      // 有效区域宽度

#define ROI_AREA    (ROI_WIDTH * ROI_HEIGHT)    // 有效区域面积

// #define ROI_TOP     2                           // 顶部有效区域行数索引
// #define ROI_BOTTOM  70                           // 底部有效区域行数索引

// 八邻域边界提取相关宏定义
#define IMG_H               ROI_BOTTOM         // 图像高度
#define IMG_W               ROI_WIDTH          // 图像宽度
#define USE_num             (IMG_H * 3 / 2)    // 八邻域最大搜索点数
#define border_min          VALID_LEFT_COL     // 左边界最小值
#define border_max          VALID_RIGHT_COL    // 右边界最大值

// 有效起始的y坐标
#define VALID_START_ROW    (ROI_BOTTOM - 1)      // ROI最底端
#define VALID_END_ROW      (ROI_TOP    + 1)      // ROI最顶端

// 有效起始的x坐标
#define VALID_LEFT_COL     (ROI_LEFT   + 1)      // ROI最左端
#define VALID_RIGHT_COL    (ROI_RIGHT  - 1)      // ROI最右端

// 丢线判断的有效底部
#define VALID_BOTTOM_ROW    (VALID_START_ROW - 5)  // 丢线判断的有效底部起始

#include "vision_point_state.h"

// 巡线调度模式枚举
enum class TrackMode {
    auto_mode,                                   // 有双用双，有单补单（适合常规道路）
    // both,                                        // 合成双线，丢线继承上一行（适合长直道、十字前）
    left_full,                                   // 左线 + 半宽（适合右环岛、右路障）
    right_full,                                  // 右线 + 半宽（适合左环岛、左路障）
    left_raw,                                    // 左边线 模型绕行
    right_raw,                                   // 右边线 模型绕行
}; 

// 巡线模式选择判断
enum TrackSide{
    SIDE_LEFT = 0,     
    SIDE_RIGHT = 1,    
};

// 最长白列信息
struct WhiteColumn{
    int lineNum;                //最长白列个数
    int maxNum;                 //最长白列长度（像素点数）
    int maxy;                   //最高点y值
    int starlineX;              //最长白列最左x值   （从左往右遍历）
    int endlineX;               //最长白列最右x值   （从左往右遍历）
};              

// 种子点状态
enum SeedState{
    no_seed,                    // 无效点状态
    left_pts,                   // 左边界点状态
    right_pts,                  // 右边界点状态
    two_pts,                    // 双边界点状态
};

class TrackBase
{
public:
    //　巡线数据结构
    struct FarUpperEdge{
        int left_x[CAM_HEIGHT];
        int right_x[CAM_HEIGHT];
        int count;
        bool valid;

        FarUpperEdge() { reset(); }

        void reset() {
            count = 0;
            valid = false;
            for (int i = 0; i < CAM_HEIGHT; i++) {
                left_x[i] = -1;
                right_x[i] = -1;
            }
        }
    };

    TrackBase() = default;
    ~TrackBase() = default;

    void track_init();                                                              // 巡线初始化
    void preprocess_frame(const cv::Mat& src);                                      // 图像预处理
    void process_frame(const cv::Mat& src);                                         // 处理一帧图像 巡线主函数实现
    bool out_checking(const cv::Mat& img);                                          // 出界保护
    float get_error_lookahead(int speed, float speed_decision, int window_size = 5);    // 误差计算 结合速度动态改变

    void track_display(void);   
    void display_upimage(cv::Mat &frame_buf);                           // 图传显示

 
    cv::Mat ipm_matrix;
    cv::Mat inverse_matrix;
    bool ipm_flag = false;
    // 图像缓存
    cv::Mat raw_buf;
    cv::Mat gray_buf;
    cv::Mat bin_buf;
    cv::Mat debug_buf;

    // 误差调试显示
    int debug_error_start = 0;
    int debug_error_end = 0;
    int debug_lookahead_x = 0;
    int debug_lookahead_y = 0;

    // 出界保护标志
    bool is_out_of_bounds = false;
    FarUpperEdge far_upper_edge;        // 急弯远处横向上沿

    // 最长白列信息
    WhiteColumn white_column;

    // 种子点坐标
    SeedState seed_state = no_seed;
    cv::Point seed_left;                    // 左种子点坐标
    cv::Point seed_right;                   // 右种子点坐标

    // 巡线控制变量
    TrackSide track_side = SIDE_LEFT;
    TrackMode current_mode = TrackMode::auto_mode;

    // 偏差值  负数表示偏左，正数表示偏右
    float track_error = 0.0f;                  
    bool lock_err_zero = false;      // 锁定误差为零            

    //==================================================================================
    // 图像处理相关函数
    //==================================================================================
    // 巡线 + 边界处理
    void process_boundary(void);

    // // 中线模式选择（本版本由边界评分决策）
    // int get_mid_line_mode(void);

    // 原图中线补全调度
    void process_midline(const PointState::BoundaryData& left, const PointState::BoundaryData& right, PointState::MidlineData& mid, TrackMode mode);
    void model_nowmidline(void);

    //==================================================================================
    // 路径搜索 点集生成
    //==================================================================================
    void find_line_lefthand_adaptive(std::vector<cv::Point> &pts_out);
    void find_line_righthand_adaptive(std::vector<cv::Point> &pts_out);
    void findline_lefthand(cv::Mat &bin_buf, cv::Point start_point, std::vector<cv::Point> *out_pts);  
    void findline_righthand(cv::Mat &bin_buf, cv::Point start_point, std::vector<cv::Point> *out_pts);  
   
    //==================================================================================
    // 点集处理和滤波
    //==================================================================================
    void blur_median(cv::Mat& image, int kernel_size, bool enable_blur);
    void blur_lines(PointState::MidlineData& mid);
    void blur_points(std::vector<cv::Point> &now_pts, int kernel_size);
    void add_point_by_line(const cv::Point &start, const cv::Point &end, std::vector<cv::Point> &pts, int dist);
    void add_three_point_bezier(const cv::Point &p0, const cv::Point &p1, const cv::Point &p2,
                                        std::vector<cv::Point> &pts,int dist);

    //==================================================================================
    // 边界线跟踪/平移
    //==================================================================================
    cv::Point get_offset_point(const cv::Point &point_now, const cv::Point &point_prev, const cv::Point &point_next, int dist, bool is_left_line);
    void track_leftline(const std::vector<cv::Point> &pts_in, std::vector<cv::Point> &pts_out,int window_size, int dist);
    void track_rightline(const std::vector<cv::Point> &pts_in, std::vector<cv::Point> &pts_out,int window_size, int dist);
    cv::Point track_point(const std::vector<cv::Point> &pts_in, int target_idx, int window_size, int dist, bool is_left);

    //==================================================================================
    // 种子点检测与状态更新
    //==================================================================================
    void seek_pts_seed(const cv::Mat &buf);
    void detect_far_upper_edge(const cv::Mat &buf);
    bool extract_boundary_8neighbor(const cv::Mat& bin_buf, PointState::BoundaryData& left_data, PointState::BoundaryData& right_data);


    //==================================================================================
    // 阈值处理与边缘检测
    //==================================================================================
    // void draw_black_border(cv::Mat& bin);
    void quick_otsu(cv::Mat &gray_buf);

    // // 互斥截断保护：防止左右线串线重合
    // void mutex_boundary_truncation();


private:

    /**
     * 获取指定像素点的像素值
     * 
     * @param img 输入图像矩阵
     * @param x 像素点的x坐标
     * @param y 像素点的y坐标
     * @return 指定位置像素值
     */
     inline int get_pixel(cv::Mat &img, int x, int y){
       return (img).data[(y) * (img).step + (x)];
    } 

    // 绘制点集
    static inline void draw_points(cv::Mat &img, const std::vector<cv::Point> &pts, cv::Vec3b color,
                               int width, int height)
    {
        if (img.empty()) return;
        for (const auto &p : pts) {
            if (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height)
                img.at<cv::Vec3b>(p.y, p.x) = color;
        }
    }

    // 值的限幅
    static inline int clip(int val, int min, int max) {
        return (val < min) ? min : ((val > max) ? max : val);
    }
    

    // 算法参数
    int frame_counter = 0;
    int now_threshold = 127;

    // 静态变量表
    static const int dir_front[8][2];
public:
    static const uint16_t halfRoad[CAM_HEIGHT];

};

extern TrackBase track_base;
#endif // TRACKING_BASE_H
