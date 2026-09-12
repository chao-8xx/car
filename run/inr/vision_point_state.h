#ifndef __POINT_STATE_H__
#define __POINT_STATE_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include "vision_tracking_base.h"

#define MAX_ARRAY_SIZE      CAM_HEIGHT      // 数组最大长度
// #define MIN_VALID_ROWS      8               // 最小有效行数

#define DX_THRESH           3               // dx突变阈值
#define SMOOTH_WIN          3               // 平均dx的滑动窗口
#define CURV_WIN            2               // 曲率验证窗口（前后各2行）
// #define CURV_THRESH         3               // 曲率验证阈值（dx变化量）

#define STABLE_THRESHOLD    3               // 需要连续3帧稳定才完全切换 (拐点防跳变)


// 八邻域方向枚举定义
// 左线八邻域方向
// L_LU L_VU L_RU
// L_VL 中心  L_VR
// L_LD L_VD L_RD

enum Direction8_L {
    L_VD = 0,        // 下 (0, -1)
    L_LD = 1,        // 左下 (-1, -1)
    L_VL = 2,        // 左 (-1, 0)
    L_LU = 3,        // 左上 (-1, 1)
    L_VU = 4,        // 上 (0, 1)
    L_RU = 5,        // 右上 (1, 1)
    L_VR = 6,        // 右 (1, 0)
    L_RD = 7         // 右下 (1, -1)
};

// 右线八邻域方向
// R_LU R_VU R_RU
// R_VL 中心  R_VR
// R_LD R_VD R_RD

enum Direction8_R {
    R_VD = 0,        // 下 (0, -1)
    R_RD = 1,        // 右下 (1, -1)
    R_VR = 2,        // 右 (1, 0)
    R_RU = 3,        // 右上 (1, 1)
    R_VU = 4,        // 上 (0, 1)
    R_LU = 5,        // 左上 (-1, 1)
    R_VL = 6,        // 左 (-1, 0)
    R_LD = 7         // 左下 (-1, -1)
};

// #define MIN_GAP             3               // 下拐点上方至少跳过3行再找
// #define MIN_LOST            3               // 至少连续3行丢线才算缺口

// 点集状态判断类
class PointState{
public:
    PointState() = default;
    ~PointState() = default;


//////////////////////////////////////////// 结构体定义 ////////////////////////////////////////////
    /**
     * @brief 边界数据结构体
     * @details 存储边界的原始点、处理点、特征点映射及相关状态信息
     */
    struct BoundaryData{
        // 核心属性
        std::vector<cv::Point> raw_pts;       // 二维点集 (迷宫法 暂时屏蔽)
        int line[MAX_ARRAY_SIZE];             // 一维数组 (最长白列法)
        int scan_line[MAX_ARRAY_SIZE];        // 逐行扫线数组 (从种子点向外扫白→黑跳变)
        
        // 八邻域方向序列数据（用于特征点检测）
        uint16_t dir_sequence[USE_num];       // 方向序列数组
        uint16_t trajectory_points[USE_num][2]; // 轨迹点坐标数组 [x, y]
        uint16_t dir_count;                   // 有效方向序列长度
        bool dir_valid;                       // 方向序列有效标志
        
        // 状态与拐点
        // int Lp_id;                            // 角点ID (在 raw_pts 中的索引)
        // float angle;                          // 当前边界角度 (度)
        bool is_straight;                     // 是否为直线段   (直道 (圆环))
        bool down_lp_state;                   // 下拐点标志位
        cv::Point down_lp_pt;                 // 下拐点坐标
        bool up_lp_state;                     // 上拐点标志位
        cv::Point up_lp_pt;                   // 上拐点坐标
        bool mid_lp_state;                    // 中拐点标志位   (圆环)
        cv::Point mid_lp_pt;                  // 中拐点坐标
        bool out_lp_state;                    // 出环缺口标志位 (圆环)
        cv::Point out_lp_pt;                  // 出环缺口坐标
        
        // 最长白列法检测的拐点 (与dx突变法并存)
        bool longest_white_down_state;        // 下拐点标志位
        cv::Point longest_white_down_pt;      // 下拐点坐标
        bool longest_white_up_state;          // 上拐点标志位
        cv::Point longest_white_up_pt;        // 上拐点坐标
        
        // 拐点历史追踪 (时序平滑)
        cv::Point prev_down_lp_pt;            // 上一帧下拐点
        cv::Point prev_up_lp_pt;              // 上一帧上拐点
        cv::Point prev_mid_lp_pt;             // 上一帧中拐点
        int down_stable_count;                // 下拐点稳定帧计数
        int up_stable_count;                  // 上拐点稳定帧计数
        int mid_stable_count;                 // 中拐点稳定帧计数
        
        // 锁定超时保护
        int down_lock_frames;                 // 下拐点锁定帧数
        int up_lock_frames;                   // 上拐点锁定帧数
        
        // 丢线检测
        bool is_lost;                         // 丢线标志 
        int lost_y;                           // 丢线处 Y 坐标 
        int lost_count;                       // ROI内丢线总行数
        uint8_t mnt_len;                      // 单调段长度
        float dx_var;                         // dx方差
        // bool is_weak_alone;                   // 弱连通标志 

        // // 扩展点
        // bool break_state;                     // 突变点标志位 
        // cv::Point break_pt;                   // 突变点坐标

        BoundaryData(){                       // 构造函数 初始化默认值
            all_reset();
            // 初始化历史追踪变量
            prev_down_lp_pt = cv::Point(-1, -1);
            prev_up_lp_pt = cv::Point(-1, -1);
            prev_mid_lp_pt = cv::Point(-1, -1);
            down_stable_count = 0;
            up_stable_count = 0;
            mid_stable_count = 0;
            down_lock_frames = 0;
            up_lock_frames = 0;
            // 初始化方向序列字段
            for(int i = 0; i < USE_num; i++) {
                dir_sequence[i] = 0;
                trajectory_points[i][0] = 0;
                trajectory_points[i][1] = 0;
            }
            dir_count = 0;
            dir_valid = false;
        }

        void all_reset(){                      // 重置数据 / 初始化
            raw_pts.clear();
            reset();
        }

        void reset(){                         // 重置数据 / 初始化
            for(int i = 0; i < MAX_ARRAY_SIZE; i ++) { line[i] = -1; scan_line[i] = -1; }
            
            // Lp_id = -1;
            // angle = 0.0f;
            is_straight = false;
            down_lp_state = false;
            down_lp_pt = cv::Point(-1, -1);
            up_lp_state = false;
            up_lp_pt = cv::Point(-1, -1);
            mid_lp_state = false;
            mid_lp_pt = cv::Point(-1, -1);
            out_lp_state = false;
            out_lp_pt = cv::Point(-1, -1);
            longest_white_down_state = false;
            longest_white_down_pt = cv::Point(-1, -1);
            longest_white_up_state = false;
            longest_white_up_pt = cv::Point(-1, -1);

            // 历史追踪不重置，保留上一帧信息
            // prev_down_lp_pt, prev_up_lp_pt, prev_mid_lp_pt 保持不变
            // 稳定计数器也不重置

            is_lost = false;
            lost_y = -1;
            lost_count = 0;
            mnt_len = 0;
            dx_var = -1.0f;
            // is_weak_alone = false;

            // break_state = false;
            // break_pt = cv::Point(-1, -1);
        }
    };

    /**
     * @brief   中线数据结构体
     * @details 存储中线的一维数组点
     */
    struct MidlineData{
        int line[MAX_ARRAY_SIZE];               // 一维数组            

        void reset(){                         // 重置数据
            for(int i = 0; i < MAX_ARRAY_SIZE; i ++) line[i] = -1;
        }

    };

    /**
     * @brief 统一特征缓存结构（供元素状态机引用）
     * @details 当前版本先保留最小定义，避免跨模块编译依赖中断；后续可按需扩展字段。
     */
    struct FeatureSet {
        bool valid = false;
    };

    // // 共有参数
    // float perspective_Mat[3][3];        // 透视变换矩阵


///////////////////////////////////////////////// 外部调用函数接口 ////////////////////////////////////////////

    /**
     * @brief 丢线检测与向量拐点提取
     * @param data 边界线数据    left_boundary  right_boundary
     * @param is_left 标志位     传入 true 代表这是左边线，false 代表右边线
     */
    void analyze_boundary(BoundaryData &data, bool is_left);

    // void DP_Update_Left_Lost(void);
    // void DP_Update_Right_Lost(void);
    // void DP_Update_Left_Mnt(void);
    // void DP_Update_Right_Mnt(void);
    // void Get_Left_UTL_Point(uint8_t mode);
    // void Get_Left_RTU_Point(uint8_t mode);
    // void Get_Right_UTR_Point(uint8_t mode);
    // void Get_Right_LTU_Point(uint8_t mode);

    /**
     * @brief 使用最长白列法检测十字路口拐点
     * @param bin_buf 二值化图像
     * @param left_data 左边界数据引用，用于存储检测到的左侧拐点
     * @param right_data 右边界数据引用，用于存储检测到的右侧拐点
     * @return 是否成功检测到有效的十字路口拐点
     */
    bool find_corners(const cv::Mat& bin_buf, BoundaryData& left_data, BoundaryData& right_data);

    /**
     * @brief 八邻域边界提取主函数
     */
    bool extract_boundary_8neighbor(const cv::Mat& bin_buf, BoundaryData& left_data, BoundaryData& right_data);

    /**
     * @brief 计算边线斜率（最小二乘法）
     * @param line 边线数组
     * @param start_y 起始行
     * @param end_y 结束行
     * @return 斜率 k
     */
    float get_line_k(int* line, int start_y, int end_y);

    /**
     * @brief 检测斜率突变
     * @param now_k 当前斜率
     * @param last_k 上一帧斜率
     * @param threshold 突变阈值
     * @return true=发生突变, false=未突变
     */
    bool check_line_k_change(float now_k, float last_k, float threshold);

    /**
     * @brief 边线跳变检测 - 左边界下拐点（向左跳变）
     * @param data 边界数据
     * @param jump_threshold X坐标跳变阈值（默认5像素）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_left_down_jump(BoundaryData& data, int jump_threshold = 3);

    /**
     * @brief 边线跳变检测 - 左边界上拐点（向右跳变）
     * @param data 边界数据
     * @param jump_threshold X坐标跳变阈值（默认5像素）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_left_up_jump(BoundaryData& data, int jump_threshold = 3);

    /**
     * @brief 边线跳变检测 - 右边界下拐点（向右跳变）
     * @param data 边界数据
     * @param jump_threshold X坐标跳变阈值（默认5像素）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_right_down_jump(BoundaryData& data, int jump_threshold = 3);

    /**
     * @brief 边线跳变检测 - 右边界上拐点（向左跳变）
     * @param data 边界数据
     * @param jump_threshold X坐标跳变阈值（默认5像素）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_right_up_jump(BoundaryData& data, int jump_threshold = 3);

    /**
     * @brief 单调性检测 - 左边界弧形拐点（X坐标局部极值）
     * @param data 边界数据
     * @param check_window 检测窗口大小（前后各check_window行）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_left_arc_monotonicity(BoundaryData& data, int check_window = 3);

    /**
     * @brief 单调性检测 - 右边界弧形拐点（X坐标局部极值）
     * @param data 边界数据
     * @param check_window 检测窗口大小（前后各check_window行）
     * @return 检测到的拐点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_right_arc_monotonicity(BoundaryData& data, int check_window = 3);

    /**
     * @brief V点检测 - 轨迹点Y坐标局部最大值（环岛入口特征）
     * @param data 边界数据
     * @param check_window 检测窗口大小
     * @return 检测到的V点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_v_point(BoundaryData& data, int check_window = 3);

    /**
     * @brief A点检测 - 轨迹点Y坐标局部最小值（环岛出口特征）
     * @param data 边界数据
     * @param check_window 检测窗口大小
     * @return 检测到的A点坐标，未检测到返回(-1,-1)
     */
    cv::Point detect_a_point(BoundaryData& data, int check_window = 3);

    /**
     * @brief 边线连续性检测
     * @param data 边界数据
     * @param continuity_threshold 连续性阈值（默认5像素）
     * @return 第一个断点的行号，无断点返回-1
     */
    int detect_discontinuity(BoundaryData& data, int continuity_threshold = 3);

private:
    /**
     * @brief 拐点时序平滑处理，防止帧间跳变
     * @param data 边界数据
     * @param corner_type 拐点类型 (0=下拐点, 1=上拐点, 2=中拐点)
     * @param is_left 是否为左边线
     */
    void stabilize_corner(BoundaryData &data, int corner_type, bool is_left);

    /**
     * @brief 计算两点之间的曼哈顿距离
     */
    inline int manhattan_dist(const cv::Point &p1, const cv::Point &p2){
        return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
    }
};


// 更新全局变量声明
extern PointState::BoundaryData left_boundary;
extern PointState::BoundaryData right_boundary;
extern PointState::MidlineData mid;

extern PointState point_state;


#endif // __POINT_STATE_H__



