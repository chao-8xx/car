#ifndef __POINT_STATE_H__
#define __POINT_STATE_H__

#include "headfile.h"

// 增加调试宏和常量
#define CURVATURE_THRESHOLD 2.0f              // 曲率阈值
#define ANGLE_THRESHOLD 30.0f                 // 直道角度阈值
#define ANGLE_LOW 70.0f                       // 角度阈值下限
#define ANGLE_HIGH 140.0f                     // 角度阈值上限
#define WINDOW_SIZE 5                         // 滑动窗口大小

#define POINT_MAX_WIDTH (CAM_WIDTH / 2)       // 处理点集最大宽度
#define POINT_Step_Max (CAM_HEIGHT / 6)       // 点集最大步长
#define POINT_Start_high VALID_START_ROW      // 起始高度
#define POINT_Mini_high (CAM_HEIGHT * 3 / 4)  // 最小高度


// 点集状态判断类
class PointState{
public:
    PointState() = default;
    ~PointState() = default;


//////////////////////////////////////////// 结构体定义 ////////////////////////////////////////////
    
    /**
     * @brief 拐点坐标映射结构体
     * @details 用于存储特征点在原始点集、变换后点集以及当前点集中的索引信息
     */
    struct CornerMapping{
        cv::Point raw;                          // 原始坐标(x, y)
        cv::Point now;                          // 变换后坐标
        int raw_index;                          // 在raw数组中的索引
        int now_index;                          // 在Now数组中的索引
    };

    /**
     * @brief 边界数据结构体
     * @details 存储边界的原始点、处理点、特征点映射及相关状态信息
     */
    struct BoundaryData{

        std::vector<cv::Point> raw_pts;       // 原始点集 (每行为一个点的x, y)
        std::vector<cv::Point> now_pts;       // 当前点集 (重采样处理后)
        
        CornerMapping map[3];                 // 特征点映射数组
        
        int Lp_id;                            // 角点ID   (在vector中的索引)
        float angle;                          // 当前边界角度 (度)
        bool is_straight;                     // 是否为直线段
        bool Lp_state;                        // 角点状态
        bool is_lost;                         // 丢线标志 

        BoundaryData(){                       // 构造函数 初始化默认值
            all_reset();
        }

        void all_reset(){                      // 重置数据 / 初始化
            raw_pts.clear();
            reset();
        }

        void reset(){                         // 重置计算结果
            now_pts.clear();
            Lp_id = -1;
            angle = 0.0f;
            is_straight = false;
            Lp_state = false;
            is_lost = true;
        }
    };

    /**
     * @brief 前视边界数据结构体
     * @details 存储前视边界的原始点和处理点
     */
    struct FBoundaryData{

        std::vector<cv::Point> raw_pts;       // 原始点集 (每行为一个点的x, y)
        std::vector<cv::Point> now_pts;       // 当前点集 (重采样处理后)

        void reset(){                         // 重置数据
            raw_pts.clear();
            now_pts.clear();
        }
    };
    
    /**
     * @brief   中线数据结构体
     * @details 存储中线的原始点、处理点、距离数组、中线距离
     * @note    dist 与 pts 一一对应
     */   
    struct MidlineData{

    std::vector<cv::Point> pts;
    std::vector<float> dist;

    void reset(){                         // 重置数据
        pts.clear();
        dist.clear();
    }

    };

    // 共有参数
    float perspective_Mat[3][3];        // 透视变换矩阵


///////////////////////////////////////////////// 外部调用函数接口 ////////////////////////////////////////////

    /**
     * @brief 综合调用各处理函数，判断左右边界的拐点、直线段和丢线状态。
     * @note  调用 filter_point 和 count_angle 处理边界数据  补充更新 is_straight 和 is_lost 状态
     * @param left  左边界数据结构体指针
     * @param right 右边界数据结构体指针
     */
    void find_corner(BoundaryData &left, BoundaryData &right);

    /**
     * @brief 在点集内根据不同策略寻找角点（特征点）索引。
     * @param pts_in        输入点集，每行为一个点的(x, y)  由底部向顶部排列
     * @param width         点集长度  用于反向计算距离 默认参数为摄像头宽度 CAM_WIDTH
     * @param reverse_order 策略选择（0~5）
     * @return              选中的角点索引
     */
    int seek_corner(const std::vector<cv::Point> &pts_in, int width = CAM_WIDTH, int reverse_order);
    


///////////////////////////////////////////////// 内部调用函数处理 ////////////////////////////////////////////
private:

    /**
     * @brief 更新角点映射关系，将原始点集中的三个特征点映射到当前点集，并找到其在当前点集中的最近点索引。
     * @note  也就是更新补充 map[3] 这个数组的信息 
     * @param data 边界数据结构体指针
     * @param ID   原始点集中特征点的索引数组，长度为3
     */
    void update_corner_mapping(BoundaryData &data, const std::vector<int> &ID);

    /**
     * @brief 从原始点集中筛选出三个特征点（如角点），并更新映射关系。
     * @param data 边界数据结构体指针
     * @param side 侧别标志，1为左侧，0为右侧
     */
    void filter_point(BoundaryData &data, bool side);

    /**
     * @brief 计算特征点的角度变化，并判断是否为拐点。
     * @note  调用 filter_point 得到的三个特征点 计算局部角度变化  补充更新 Lp_state angle 和 Lp_id 状态
     * @param data   边界数据结构体指针
     * @param image  输入图像（灰度图）
     * @param window 局部窗口大小 (需要小于 3*3 也就是小于9)
     */
    void count_angle(BoundaryData &data, const cv::Mat &image, int window);

/////////////////////////////////////////////// 内部函数辅助函数 ///////////////////////////////////////////////////

    // 计算点集某点的局部夹角 计算索引 idx 处的局部角度 (单位：弧度)
    float local_angle_points(std::vector<cv::Point> &pts_in, int idx, int dist);

    // 局部自适应阈值计算 利用cv::mean计算ROI区域的平均灰度值
    int adaptiveThreshold(const cv::Mat &img, cv::Point pt, int size);

    // 计算三点构成的外接圆曲率  门格尔曲率
    float compute_curvature(const cv::Point &P0, const cv::Point &P1, const cv::Point &P2);

    // 计算两点之间的曼哈顿距离
    inline int manhattan_dist(const cv::Point &p1, const cv::Point &p2){
        return std::abs(p1.x - (int)p2.x) + std::abs(p1.y - (int)p2.y);
    }

    // std::clamp 平替版
    static inline int clamp_int(int val, int min, int max){
    return (val < min) ? min : ((val > max) ? max : val);
    }
};


// 更新全局变量声明
extern PointState::BoundaryData left_boundary;
extern PointState::BoundaryData right_boundary;
extern PointState::FBoundaryData Fleft_boundary;
extern PointState::FBoundaryData Fright_boundary;
extern PointState::MidlineData mid_line;


#endif // __POINT_STATE_H__


