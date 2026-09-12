#/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头服务器模块
 * 版权所有 (c) 2025 Blockingsys
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
 * 文件名称：ww_camera_server.h
 * 所属模块：wuwu_library
 * 功能描述：摄像头服务器（HTTP/MJPEG）头文件
 *
 * 修改记录：
 * 日期         作者            说明
 * 2025-12-16  Blockingsys    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#ifndef __CAMERA_SERVER_H__
#define __CAMERA_SERVER_H__

#include "headfile.h"

// 默认端口号
#define CAMERA_STREAM_DEFAULT_PORT 8080

class AsyncMusicPlayer;

class CameraStreamServer
{
public:
    CameraStreamServer(void);

    using ControlCallback = std::function<void(const std::string&)>;
    // 仅保留音乐控制相关的构造（可传 nullptr 关闭音乐控制）
    explicit CameraStreamServer(AsyncMusicPlayer* music);
    ~CameraStreamServer(void);


/*******************************************************************
 * @brief       启动摄像头图传服务器
 * 
 * @param       port            服务器监听端口(默认8080)
 * 
 * @return      返回启动状态
 * @retval      0               启动成功
 * @retval      -1              启动失败
 * 
 * @example     //启动摄像头图传服务器
 *              if(camera_server.start_server(8080) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        在后台线程中启动HTTP服务器，支持浏览器访问
 *              访问 http://<开发板IP>:<port> 即可查看实时画面
 ******************************************************************/
    int start_server(int port = CAMERA_STREAM_DEFAULT_PORT);

/*******************************************************************
 * @brief       更新摄像头帧数据
 * 
 * @param       frame           OpenCV Mat格式的图像帧
 * 
 * @example     camera_server.update_frame_mat(frame);
 * 
 * @note        将最新的摄像头帧推送到服务器，供客户端获取
 *              自动编码为JPEG格式并计算帧率
 ******************************************************************/
    void update_frame_mat(const cv::Mat& frame);

/*******************************************************************
 * @brief       更新摄像头帧数据（零拷贝JPEG）
 * 
 * @param       jpeg_data       JPEG原始数据
 * 
 * @example     camera.capture_frame(frame, false);  // 不解码
 *              camera_server.update_frame_jpeg(camera.jpeg_nowdata);
 * 
 * @note        直接使用JPEG原始数据，跳过编解码步骤
 *              适用于纯图传场景，性能提升50-100倍
 ******************************************************************/
    void update_frame_jpeg(const std::vector<uchar>& jpeg_data);

/*******************************************************************
 * @brief       停止摄像头图传服务器
 * 
 * @example     camera_server.stop_server();
 * 
 * @note        停止服务器并释放所有资源
 ******************************************************************/
    void stop_server(void);

/*******************************************************************
 * @brief       检查服务器是否正在运行
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器正在运行
 * @retval      false           服务器已停止
 * 
 * @example     if(camera_server.is_running()) {
 *                  //服务器正在运行
 *              }
 ******************************************************************/
    bool is_running(void);

private:
    // 服务器socket文件描述符
    int server_sock_fd;
    // 服务器端口
    int server_port;
    // 服务器运行状态
    bool running;
    // 服务器线程ID
    pthread_t server_thread_id;
    
    // 帧数据互斥锁
    pthread_mutex_t frame_mutex;
    // 帧数据条件变量
    pthread_cond_t frame_cond;
    // socket互斥锁
    pthread_mutex_t sock_mutex;
    
    // 当前JPEG数据
    std::vector<unsigned char> current_jpeg;
    // 当前原始帧数据
    cv::Mat current_frame;
    // 最新帧ID
    uint64_t latest_frame_id;
    // 最新捕获时间戳(毫秒)
    uint64_t latest_capture_ts_ms;
    // EMA帧率
    double ema_fps;

    // // 图像翻转标志位
    // bool flip_enable = false;
    // // 图像翻转参数
    // int flip_code = -1;
    // // 缓存编码器以减少重复创建的开销
    // std::vector<int> jpeg_compression_params;
    
    // // 原始帧数据（用于保存高质量图片）
    // cv::Mat original_frame;
    // // 原始帧互斥锁
    // pthread_mutex_t original_frame_mutex;
    
    // 内部方法
    std::string get_local_ip(void);
    void close_server_socket(void);
    uint64_t now_ms(void);
    std::string format_timestamp(uint64_t ts_ms);
    void send_response(int sock, const char* content_type, const char* body, size_t body_len);
    void send_stats_response(int sock);
    void send_mjpeg_stream(int sock);
    void handle_client_request(int sock);
    void handle_snapshot_request(int sock, const std::string& prefix);

    // 网页音乐控制
    void handle_music_request(int sock, const std::string& path);
    void send_music_stats(int sock);

    AsyncMusicPlayer* music_player;
    ControlCallback control_callback;
    pthread_mutex_t control_callback_mutex;
    pthread_mutex_t control_exec_mutex;
    std::string custom_viewer_html;
    
    // 静态线程函数
    static void* server_thread_func(void* arg);
    static void* client_thread_func(void* arg);
    static void signal_handler(int sig);
    
    // 全局实例指针(用于信号处理)
    static CameraStreamServer* instance;
};

#endif
