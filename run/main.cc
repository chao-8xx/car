// 实机操作   线程版本
#if 1

#include "headfile.h"

// 全局共享资源
std::mutex mtx;                                     // 互斥锁
std::condition_variable cv_frame;                   // 条件变量
std::vector<uchar> global_jpeg_buf;                 // 共享的JPEG图像缓冲区
std::atomic<bool> new_frame_flag(false);            // 新图像标志
std::atomic<bool> program_run_flag(true);           // 程序运行标志

CameraStreamServer camera_server(&music_player);
std::mutex alg_mutex; // 互斥锁
double global_capture_ms = 0.0;

extern Key key0;
extern Menu_Folder *key;    //声明光标

// 信号处理 监控线程 (用于打破死锁)
void signal_handler_thread() 
{
    // 准备只接收 SIGINT(Ctrl+C) 和 SIGTERM(kill)
    sigset_t set;                                    // 定义信号集
    sigemptyset(&set);                               // 清空信号集
    sigaddset(&set, SIGINT);                         // 添加 SIGINT 到信号集 (Ctrl+C)
    sigaddset(&set, SIGTERM);                        // 添加 SIGTERM 到信号集 (kill)
    
    int sig;
    // 同步信号处理 这里会挂起等待，直到按下 Ctrl+C。不占用 CPU
    sigwait(&set, &sig);
    
    std::cout << "系统退出" << std::endl;
    
    // 修改程序运行标志，让采集线程和消费线程退出循环
    program_run_flag = false;
    
    // 强制关闭摄像头流
    // 这会让采集线程里的 DQBUF 立即返回，从而跳出循环
    camera.stop(); 
    camera_server.stop_server();
    
    // 通知所有等待的线程 广播
    cv_frame.notify_all();
}

// 生产者线程  图像采集
void frame_get_thread(Camera &cam)
{
    cv::Mat temp_frame;
    global_jpeg_buf.resize(1024 * 300); // 预留 300KB，避免运行时扩容
    
    while(program_run_flag){
        // 如果 capture_frame 返回 false，通常是因为信号处理线程被中断，直接 break
        auto capture_start = std::chrono::steady_clock::now();
        if(!cam.capture_frame(temp_frame, false)){
            
            // 如果标志位已变，说明是正常退出，直接 break
            if (!program_run_flag)      break;
            std::cerr << "采集丢帧" << std::endl;
            car_soft_stop();
            fans.set_duty(0);
            Gogo = false;
            continue;
        }
        double capture_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - capture_start).count();

        // 临界区 加锁 搬运数据
        // 注: lock_guard 自动管理锁的生命周期 解构时自动解锁，不用手动 unlock
        {
            std::lock_guard<std::mutex> lock(mtx);

            // 零拷贝交换指针，避免不必要的内存复制
            // global_jpeg_buf = cam.jpeg_nowdata;
            std::swap(global_jpeg_buf, cam.jpeg_nowdata); 
            global_capture_ms = capture_ms;
            new_frame_flag = true;
        }
        // 通知消费者线程 唤醒
        cv_frame.notify_one();
    }
}


// 消费者线程  图像处理
void frame_handle_thread(CameraStreamServer &server)
{
    // 私有缓存
    cv::Mat raw_frame, flip_frame;
    std::vector<uchar> local_jpeg;
    double local_capture_ms = 0.0;

    while(program_run_flag && server.is_running()){

        // 临界区 加锁 等待新图像或退出信号
        // 注: unique_lock 支持条件变量的同步操作，超时后返回 false，不用手动判断
        //      即通过 std::condition_variable 的 wait() 方法来释放锁并等待条件变量满足，然后重新获取锁并继续执行
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame.wait_for(lock, std::chrono::milliseconds(100), []{
                return new_frame_flag.load() || !program_run_flag.load(); // 访问全局原子变量
            });
            
            // 如果标志位已变，说明是正常退出，直接 break 
            // 如果 cv_frame.wait_for 超时，说明是因为没有新图像，继续等待下一次循环
            if (!program_run_flag) break;
            if (!new_frame_flag)  continue;

            // local_jpeg = global_jpeg_buf;
            std::swap(local_jpeg, global_jpeg_buf);
            local_capture_ms = global_capture_ms;
            new_frame_flag = false; 
        }
        
        // 解码 进行图像处理
        auto frame_start = std::chrono::steady_clock::now();
        auto decode_start = std::chrono::steady_clock::now();
        raw_frame = cv::imdecode(local_jpeg, cv::IMREAD_COLOR);
        double decode_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - decode_start).count();
        if (raw_frame.empty() || raw_frame.cols != CAMERA_WIDTH || raw_frame.rows != CAMERA_HEIGHT) {
            continue;
        }

        // 图像翻转
        auto flip_start = std::chrono::steady_clock::now();
        cv::flip(raw_frame, flip_frame, -1);
        double flip_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - flip_start).count();

        double track_ms = 0.0;
        double model_ms = 0.0;
        double display_ms = 0.0;

        {
            std::lock_guard<std::mutex> alg_lock(alg_mutex);

            // 标定模式
            if (key != nullptr && key->execute == true &&
                biaoding_auto.current_state != BiaoDingState::idle) 
            {
                biaoding_auto.biaoding_main(flip_frame);
            }
            else
            {
                // 图像处理 + 图传
                auto track_start = std::chrono::steady_clock::now();
                track_base.process_frame(flip_frame);
                track_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - track_start).count();
                auto model_start = std::chrono::steady_clock::now();
                model_detector.model_main(flip_frame);
                model_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - model_start).count();
                auto display_start = std::chrono::steady_clock::now();
                track_base.display_upimage(flip_frame);
                // display_ms = std::chrono::duration<double, std::milli>(
                //                  std::chrono::steady_clock::now() - display_start).count();

                // 屏幕显示
                if (key != nullptr && key->execute == true &&
                    strcmp(key->name, "图像显示") == 0)
                {
                    track_base.track_display();
                }
            }
        }

        if (preprocess.model_profile_enable) {
            static int frame_profile_count = 0;
            int interval = std::max(1, preprocess.model_profile_interval);
            frame_profile_count++;
            if (frame_profile_count % interval == 0) {
                double total_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - frame_start).count();
                double fps = (total_ms > 0.001) ? (1000.0 / total_ms) : 0.0;
                std::cout << "[FrameProfile] cap=" << local_capture_ms << "ms"
                          << " decode=" << decode_ms << "ms"
                          << " flip=" << flip_ms << "ms"
                          << " track=" << track_ms << "ms"
                          << " model=" << model_ms << "ms"
                          << " display=" << display_ms << "ms"
                          << " total=" << total_ms << "ms"
                          << " fps=" << fps
                          << std::endl;
            }
        }
    }
}

#include "pthread_control.h"    //多线程控制(声明)

int main()
{
    // 在主线程屏蔽信号
    // 注: 信号处理线程会在主线程阻塞，直到收到 Ctrl+C 信号，然后修改程序运行标志，让采集线程和消费线程退出循环
    //      这样 Ctrl+C 就不会打断主线程，而是被后面的 signal_handler_thread 捕获
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    
    if(camera.init(CAMERA_WIDTH, CAMERA_HEIGHT, CAM_FPS) < 0) {
        std::cerr << "摄像头初始化失败" << std::endl;
        return -1;
    }
    if(camera_server.start_server(8080) < 0)
    {
        std::cerr << "图传服务器启动失败" << std::endl;
        return -1;
    }
    if (!model_detector.init("model_packed.ncnn.param", "model_packed.ncnn.bin")) 
    {
        std::cerr << "模型推理加载失败" << std::endl;
        return -1;
    }

    //等待初始化
    std::cout<<"-----------------------\n"<<std::endl;
    for(int i = 0; i <= 100; i ++)
    {
        std::cout<<"\r正在启动初始化.........%"<< i <<std::flush;
        usleep(6000);
    }
    std::cout<<std::endl;

    std::cout<<"-----------------------\n"<<"o_o开始初始化o_o\n"<<"-----------------------\n"<<std::endl;

    all_init();     //全部初始化
    reset_state = false;    //初始化复位状态

    std::cout<<"-----------------------\n"<<"ovo初始化完成ovo\n"<<"-----------------------\n"<<std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    program_run_flag = true;        // 启动线程前标记运行状态

    pthread_start();    //启动各个线程

    return 0;
}



/* 程序退出流程详解:
 *
 *  用户按下Ctrl+C
 *      ↓
 *  操作系统发送SIGINT信号
 *      ↓
 *  信号监控线程的sigwait()返回
 *      ↓
 *  信号监控线程执行：
 *  1. program_run_flag = false     (原子操作)
 *  2. camera.stop()                (停止硬件)
 *  3. camera_server.stop_server()  (停止网络)
 *  4. cv_frame.notify_all()        (唤醒等待线程)
 *      ↓
 *  生产者线程：
 *  while(program_run_flag) → false → 退出循环
 *      ↓
 *  消费者线程：
 *  wait_for()被唤醒 → 检查program_run_flag → false → 退出循环
 *      ↓
 *  主线程：
 *  join()所有线程 → 程序安全退出
 * 
 */



// /******************************************************陀螺仪测试代码******************************************************/
/*
 * imu_test.cc - ICM42688 陀螺仪/加速度计测试程序（自包含，不依赖 wuwu_library）
 *
 * 功能：持续读取陀螺仪和加速度计数据，方便确认方向和数值
 *   - 直接 ioctl 读取 /dev/wuwu_icm42688
 *   - 量程: 陀螺仪 ±2000°/s, 加速度计 ±16g
 *   - 每10ms采样，打印500次（共5秒）
 *
 * 编译: cmake target imu_test
 * 用法: ./imu_test
 *       手动旋转/倾斜车体，观察各轴读数变化
 */


// #include "headfile.h"

// using namespace std;  // 简化 cout 写法

// /* 量程转换系数 */
// static const float GYRO_INV  = 2000.0f / 32768.0f;   // ±2000°/s
// static const float ACCEL_INV = 16.0f / 32768.0f;      // ±16g


// // 全局共享资源
// std::mutex mtx;                                     // 互斥锁
// std::condition_variable cv_frame;                   // 条件变量
// std::vector<uchar> global_jpeg_buf;                 // 共享的JPEG图像缓冲区
// std::atomic<bool> new_frame_flag(false);            // 新图像标志
// std::atomic<bool> program_run_flag(true);           // 程序运行标志

// // //全局对象 
// // extern Key key0;
// // extern LCD lcd;
// // extern Motor motor;
// CameraStreamServer camera_server(&music_player);
// std::mutex alg_mutex; // 互斥锁

// int main()
// {
//     int fd = open(ICM42688_DEVICE, O_RDWR);
//     if (fd < 0) {
//         perror("打开 ICM42688 失败");
//         return -1;
//     }

//     // ===================== C++ 格式化输出 =====================
//     cout << "ICM42688 初始化成功，开始读取数据（5秒）...\n\n";

//     // 表头左对齐，宽度 10
//     cout << left
//          << setw(10) << "gyro_x"
//          << setw(10) << "gyro_y"
//          << setw(10) << "gyro_z"
//          << setw(10) << "accel_x"
//          << setw(10) << "accel_y"
//          << setw(10) << "accel_z"
//          << endl;

//     cout << left
//          << setw(10) << "(°/s)"
//          << setw(10) << "(°/s)"
//          << setw(10) << "(°/s)"
//          << setw(10) << "(g)"
//          << setw(10) << "(g)"
//          << setw(10) << "(g)"
//          << endl;

//     cout << "-----------------------------------------------------------\n";

//     // 循环读取 500 次
//     for (int i = 0; i < 500; i++) {
//         icm42688_gyro_data  gd = {};
//         icm42688_accel_data ad = {};

//         ioctl(fd, ICM42688_GET_GYRO,  &gd);
//         ioctl(fd, ICM42688_GET_ACCEL, &ad);

//         float gx = gd.x * GYRO_INV;
//         float gy = gd.y * GYRO_INV;
//         float gz = gd.z * GYRO_INV;
//         float ax = ad.x * ACCEL_INV;
//         float ay = ad.y * ACCEL_INV;
//         float az = ad.z * ACCEL_INV;

//         // 对应 printf("%+8.2f  %+8.2f  %+8.2f  %+8.4f  %+8.4f  %+8.4f\n");
//         cout << showpos       // 显示正负号 +
//              << fixed         // 固定小数格式
//              << setprecision(2)  // 陀螺仪 2 位小数
//              << setw(10) << gx
//              << setw(10) << gy
//              << setw(10) << gz
//              << setprecision(4)  // 加速度 4 位小数
//              << setw(10) << ax
//              << setw(10) << ay
//              << setw(10) << az
//              << endl;

//         usleep(10000); // 10ms
//     }

//     close(fd);

//     cout << "\n测试完成，共500次采样（5秒）\n";
//     cout << "\n提示: 手动向左转车体时 gyro_z 为正值 → 中环方向正确\n";
//     cout << "      手动向左转车体时 gyro_z 为负值 → 需要在代码里取反\n";

//     return 0;
// }

#endif

#if 0
/********************************************寒假前的石山代码********************************************/
#include "headfile.h"

using namespace cv;
using namespace std;

// ---------------- 信号处理 (千万不要删) ----------------
static bool running = true;
static void sigint_handler(int sign) { running = false; }
static void system_init(void) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
}
// -----------------------------------------------------

// 实例化异步音乐播放器
AsyncMusicPlayer music_player;

// 传入 music_player 指针，使 Web 端能控制音乐
CameraStreamServer camera_server(&music_player);

// 摄像头对象在 tracking_base_edge.cc 里定义（避免重复打开设备）
extern VideoCapture cap;
extern Camera cam;

/*********定义类*********/

/*********声明类*********/
extern Motor motor;
extern Key key0;

extern int temporary_speed;
bool zero;

// 全局共享资源
std::mutex mtx;                                     // 互斥锁
std::condition_variable cv_frame;                   // 条件变量
std::vector<uchar> global_jpeg_buf;                 // 共享的JPEG图像缓冲区
std::atomic<bool> new_frame_flag(false);            // 新图像标志
std::atomic<bool> program_run_flag(true);           // 程序运行标志
std::mutex alg_mutex;

/*声明线程*/
void motor_ctrl(void *arg);
void key_ctrl(void *arg);
// void vofa_ctrl(void *arg);

int main(int argc, char** argv)
{
    system_init();

    if (camera_server.start_server(8080) < 0){
        std::cerr << "Camera server start failed" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    bool opened_mode = false;
    
    // 先打开视频源：支持摄像头 或 视频文件
    // 用法：
    //   ./main               -> 摄像头0
    //   ./main /root/111.avi -> 播放视频文件
    if (argc >= 2) {
        opened_mode = true;
        if (!cap.open(argv[1])) {
            std::cerr << "Video cannot be opened: " << argv[1] << std::endl;
            return -1;
        }
        std::cout << "当前模式: 视频文件 " << argv[1] << std::endl;
    }
    else {
        opened_mode = false;
        
        if (cam.init(CAM_WIDTH, CAM_HEIGHT, CAM_FPS) < 0) {
            std::cerr << "Camera cannot be opened" << std::endl;
            return -1;
        }
        std::cout << "当前模式: 高效摄像头 " << std::endl;
    }

//     // 设置摄像头参数
//     cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));  
//     cap.set(CAP_PROP_FRAME_WIDTH, 160);     
//     cap.set(CAP_PROP_FRAME_HEIGHT, 120);
//     cap.set(CAP_PROP_FPS, 60);              
   
//     cap.set(CAP_PROP_BRIGHTNESS, 100);      // 亮度 
//     cap.set(CAP_PROP_CONTRAST, 32);         // 对比度 
//     cap.set(CAP_PROP_SHARPNESS, 70);        // 清晰度
//     cap.set(CAP_PROP_SATURATION, 64);       // 饱和度 
//     cap.set(CAP_PROP_GAIN, 74);             // 增益 
//     cap.set(CAP_PROP_EXPOSURE, 3);          // 自动曝光

    all_init();     //整体初始化
    

    cv::Mat frame;
    cv::Mat bin_dummy;

    //创建电机控制线程(Ts ms)执行一次 motor_ctrl 内的内容
    TimerThread motorThread(motor_ctrl, NULL, Ts);
    //启动线程
    motorThread.start();

    //创建按键控制线程(0 ms)执行一次 key_ctrl 内的内容
    TimerThread keyThread(key_ctrl, NULL, 0);
    //启动线程
    keyThread.start();

    // //创建按键VOFA线程(2 ms)执行一次 vofa_ctrl 内的内容
    // TimerThread vofaThread(vofa_ctrl, NULL, 2);
    // //启动线程
    // vofaThread.start();

    
    std::cout << "冯小超!准备出发!" <<std::endl;
    

    while(running && camera_server.is_running())
    {
        bool ret = false;

        // 视频文件模式处理
        if (opened_mode) {
            cap >> frame;
            ret = !frame.empty();
        }

        // 高效摄像头模式处理
        else 
            ret = cam.capture_frame(frame);
        
        if (!ret) {
            std::cerr << "获取图像失败" << std::endl;;
            break;
        }

        // 摄像头反装，故在高效摄像头模式时翻转图像
        if (!opened_mode)
        flip(frame, frame, -1); //翻转图像

        track_base0.process_frame0(frame);
        track_base0.display_upimage0(1);    // 图传显示 0=原图, 1=灰度, 2=二值图

        // motor.thread_syn();
        // motor.syn_encoder1_counts;
        // motor.syn_encoder2_counts;

    }

    // cap.release();
    std::cout << "Program exited" << std::endl;

    return 0;
}

//按键监听线程
void key_ctrl(void *arg)
{
    key0.key_listeners();   //实时监听按键
    show_number();          //按键触发时，参数要实时显示，便于观察
}

//电机控制线程
void motor_ctrl(void *arg)
{
    if(Gogo)
    {      
        if(!is_angle)
        {
            madg.madgwickCtrl_init();       //陀螺仪解算初始化
            yaw_angle = 0;
            is_angle = !is_angle;
        }
        else 
        {
            if(is_start)  //起步完成时，开始控制决策
            {
                car_start();
                photo_err = 0;
            }
            else if(! is_start)
            {
                temporary_control();
                if(L_speed > 2000 || R_speed > 2000) Gogo = ! Gogo;
            }

        }

    }
    else 
    {
        ////获取速度////
        //读取编码器数据
        motor.update_encoders(); 
        motor.encoder2_counts;  
        motor.encoder1_counts;
        //刷新速度值
        L_speed = -motor.encoder1_counts * 10;
        R_speed = -motor.encoder2_counts * 10;
        int mid_speed = (L_speed + R_speed)/2;

        ////获取图像误差////
        photo_err = track_base0.get_error_lookahead0(200,preprocess.preview / 10,preprocess.window_size);

        ////停车及其参数初始化////
        car_soft_stop();    //智能车软停止

        ////清除误差和输出////
        Lmotor_PID.error = 0;
        Lmotor_PID.output = 0;
        Rmotor_PID.error = 0;
        Rmotor_PID.output = 0;
        Angle_PID.error = 0;
        Angle_PID.output = 0;
        Photo_PID.error = 0;
        Photo_PID.output = 0;

    }
    
}

void vofa_ctrl(void *arg)
{
    vofa.vofa_read(0, Angle_PID.error,photo_err);
}

/************************************************************************************************************/




#endif



#if 0

// ==========================================
// 视频验证
// ==========================================

#include "headfile.h"

// 全局共享资源
std::mutex mtx;                                     // 互斥锁
std::condition_variable cv_frame;                   // 条件变量
std::vector<uchar> global_jpeg_buf;                 // 共享的JPEG图像缓冲区
std::atomic<bool> new_frame_flag(false);            // 新图像标志
std::atomic<bool> program_run_flag(true);           // 程序运行标志

//全局对象 
extern Key key0;
extern LCD lcd;
extern Motor motor;
VideoCapture cap;
CameraStreamServer camera_server(&music_player);
std::mutex alg_mutex; // 互斥锁

// 信号处理线程：捕捉 Ctrl+C 
void signal_handler_thread() {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    
    int sig;
    sigwait(&set, &sig);
    
    std::cout << "\n[系统] 收到退出信号，正在安全关闭视频测试端..." << std::endl;
    program_run_flag = false;
    camera_server.stop_server();
}

int main(int argc, char **argv)
{
    // 屏蔽主线程信号，交给 signal_handler_thread 统一捕获
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    // 参数检查：运行时需传入你的 avi 录像或视频路径
    if (argc < 2) {
        std::cerr << "========================================================\n"
                  << "用法错误! 在视频测试模式下，请传入测试视频文件路径!\n"
                  << "示例: ./main /root/test.avi\n"
                  << "========================================================" << std::endl;
        return -1;
    }

    // 启动图传服务器
    if(camera_server.start_server(8080) < 0) { 
        std::cerr << "图传服务器启动失败" << std::endl;
        return -1; 
    }

    // 初始化算法相关的基础组件
    track_base.track_init();
    preprocess.Preprocess_init();
    element_state.element_init();

    // 打开所选视频文件
    if (!cap.open(argv[1])) {
        std::cerr << "视频文件无法打开: " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "成功打开且开始播放视频文件: " << argv[1] << std::endl;

    std::thread quit_thread(signal_handler_thread);

    cv::Mat frame;
    uint16_t tof_dist = 0; // 离线视频测试暂无真实TOF数据，这里输入模拟的0即可

    std::cout << "[系统] 进入纯离线视频播放与算法仿真流水线..." << std::endl;

    while (program_run_flag) {
        cap >> frame;
        
        if (frame.empty()) {
            std::cout << "视频播放到底了，已自动归零循环重播..." << std::endl;
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 循环播放设计
            continue;
        }

        // 把任意分辨率的视频压缩成当前巡线算法工作尺寸 (CAM_WIDTH x CAM_HEIGHT)
        if (frame.cols != CAM_WIDTH || frame.rows != CAM_HEIGHT) {
            cv::resize(frame, frame, cv::Size(CAM_WIDTH, CAM_HEIGHT));
        }

        {
            std::lock_guard<std::mutex> alg_lock(alg_mutex);

            // 基础图像处理
            track_base.process_frame(frame);

            // 出界保护
            if (track_base.out_checking(track_base.bin_buf)) {
                // motor_enable_off();
                // 电机关闭
                
                // element_state.target_speed = 0.0f; 
            }

            // // 元素状态机处理
            // element_state.elements_process(track_base.bin_buf, track_base.raw_buf, tof_dist);

            // 图传
            track_base.display_upimage(frame);
        }

        // 控制离线播放跑帧速度，大约33毫秒给算法喘息和发往图传的时间
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    quit_thread.join();
    std::cout << "测试程序退出完成。" << std::endl;
    return 0;
}

#endif



// 模型测试代码
#if 0

#include "headfile.h"

using namespace cv;

std::mutex mtx;
std::condition_variable cv_frame;
std::vector<uchar> global_jpeg_buf;
std::atomic<bool> new_frame_flag(false);
std::atomic<bool> program_run_flag(true);

CameraStreamServer camera_server(&music_player);
std::mutex alg_mutex;

static const char* yes_no(bool value)
{
    return value ? "Y" : "N";
}

static const char* distance_text(BoardDistanceLevel level)
{
    switch (level) {
        case BoardDistanceLevel::far: return "FAR";
        case BoardDistanceLevel::mid: return "MID";
        case BoardDistanceLevel::near: return "NEAR";
        default: return "NONE";
    }
}

static const char* roi_source_text(ModelRoiSource source)
{
    switch (source) {
        case ModelRoiSource::red_marker: return "RED";
        case ModelRoiSource::loose_marker: return "LOOSE";
        default: return "NONE";
    }
}

static const char* marker_visual_text(MarkerVisualType type)
{
    switch (type) {
        case MarkerVisualType::queue_marker: return "QUEUE";
        case MarkerVisualType::model_marker: return "MODEL";
        case MarkerVisualType::large_supplies: return "SUPPLY";
        case MarkerVisualType::loose_warning: return "WARN";
        default: return "NONE";
    }
}

static void print_rect(const char* name, const cv::Rect& rect)
{
    std::cout << " " << name << "=";
    if (rect.area() <= 0) {
        std::cout << "none";
    }
    else {
        std::cout << "(" << rect.x << "," << rect.y
                  << "," << rect.width << "," << rect.height << ")";
    }
}

static void print_model_status(int frame_id, const ModelActionResult& status)
{
    std::cout << "[model_test]"
              << " frame=" << frame_id
              << " loose=" << yes_no(status.board_detected)
              << " dist=" << distance_text(status.distance)
              << " loose_score=" << std::fixed << std::setprecision(2) << status.board_score
              << " strict_red=" << yes_no(status.marker_detected)
              << " marker_visual=" << marker_visual_text(status.marker_visual_type)
              << " roi_source=" << roi_source_text(status.roi_source)
              << " model=" << yes_no(status.model_ran)
              << " class=" << (status.model.class_name.empty() ? "none" : status.model.class_name)
              << " prob=" << std::fixed << std::setprecision(3) << status.model.probability
              << " scores=S/T/W="
              << std::fixed << std::setprecision(3)
              << status.model.class_scores[0] << "/"
              << status.model.class_scores[1] << "/"
              << status.model.class_scores[2]
              << " detected=" << yes_no(status.model.is_detected)
              << " action=" << model_calib.action_to_ascii(status.action);

    print_rect("loose_rect", status.board_box);
    print_rect("strict_red_rect", status.marker_box);
    print_rect("roi", status.roi_box);
    std::cout << std::endl;
}

void signal_handler_thread()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);

    int sig;
    sigwait(&set, &sig);

    std::cout << "[model_test] exit signal" << std::endl;
    program_run_flag = false;
    camera_server.stop_server();
    cv_frame.notify_all();
}

void frame_get_thread(Camera& cam)
{
    cv::Mat temp_frame;
    global_jpeg_buf.resize(1024 * 300);

    while (program_run_flag) {
        if (!cam.capture_frame(temp_frame, false)) {
            if (!program_run_flag) break;
            std::cerr << "[model_test] capture frame failed" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            std::swap(global_jpeg_buf, cam.jpeg_nowdata);
            new_frame_flag = true;
        }
        cv_frame.notify_one();
    }
}

void frame_handle_thread(CameraStreamServer& server)
{
    cv::Mat raw_frame;
    cv::Mat flip_frame;
    std::vector<uchar> local_jpeg;
    int frame_id = 0;

    while (program_run_flag && server.is_running()) {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame.wait_for(lock, std::chrono::milliseconds(100), [] {
                return new_frame_flag.load() || !program_run_flag.load();
            });

            if (!program_run_flag) break;
            if (!new_frame_flag) continue;

            std::swap(local_jpeg, global_jpeg_buf);
            new_frame_flag = false;
        }

        raw_frame = cv::imdecode(local_jpeg, cv::IMREAD_COLOR);
        if (raw_frame.empty()) {
            std::cerr << "[model_test] jpeg decode failed" << std::endl;
            continue;
        }
        if (raw_frame.cols != CAMERA_WIDTH || raw_frame.rows != CAMERA_HEIGHT) {
            std::cerr << "[model_test] bad frame size "
                      << raw_frame.cols << "x" << raw_frame.rows << std::endl;
            continue;
        }

        cv::flip(raw_frame, flip_frame, -1);

        ModelActionResult status;
        {
            std::lock_guard<std::mutex> alg_lock(alg_mutex);
            track_base.process_frame(flip_frame);
            status = model_detector.process_model_debug_frame(flip_frame);
        }

        print_model_status(frame_id, status);
        server.update_frame_mat(flip_frame);
        frame_id++;
    }
}

int main()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    if (camera.init(CAMERA_WIDTH, CAMERA_HEIGHT, CAM_FPS) < 0) {
        std::cerr << "[model_test] camera init failed" << std::endl;
        return -1;
    }

    if (camera_server.start_server(8080) < 0) {
        std::cerr << "[model_test] camera server start failed" << std::endl;
        return -1;
    }

    preprocess.Preprocess_init();
    track_base.track_init();
    element_state.element_init();

    if (!model_detector.init("model_packed.ncnn.param", "model_packed.ncnn.bin")) {
        std::cerr << "[model_test] model load failed" << std::endl;
        return -1;
    }

    camera.set_brightness(preprocess.brightness);
    camera.set_contrast(preprocess.contrast);
    camera.set_sharpness(preprocess.sharpness);
    camera.set_saturation(preprocess.saturation);
    camera.set_gain(preprocess.gain);
    camera.set_auto_exposure(false);
    camera.set_exposure_absolute(preprocess.exposure);

    std::cout << "[model_test] ready, camera class + vision/model only" << std::endl;
    std::cout << "[model_test] output: loose/strict_red/roi/class/prob/action" << std::endl;

    program_run_flag = true;
    std::thread quit_thread(signal_handler_thread);
    std::thread frame_producer(frame_get_thread, std::ref(camera));
    std::thread frame_consumer(frame_handle_thread, std::ref(camera_server));

    frame_producer.join();
    frame_consumer.join();
    quit_thread.join();

    std::cout << "[model_test] Program exited" << std::endl;
    return 0;
}


#endif


// TOF测试代码
# if 0

#include "headfile.h"

std::mutex mtx;
std::condition_variable cv_frame;
std::vector<uchar> global_jpeg_buf;
std::atomic<bool> new_frame_flag(false);
std::atomic<bool> program_run_flag(true);
std::mutex alg_mutex;
CameraStreamServer camera_server(&music_player);

void frame_get_thread(Camera &cam)
{
    (void)cam;
}

void frame_handle_thread(CameraStreamServer &server)
{
    (void)server;
}

static bool tof_raw_valid(uint16_t raw)
{
    // 8190/8191/8192 这类都视为无效/超量程
    return raw != 0 && raw < 8000 && raw <= 2000;
}

void signal_handler_thread()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);

    int sig;
    sigwait(&set, &sig);

    program_run_flag = false;
}

int main()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    std::thread quit_thread(signal_handler_thread);

    // 使用和主程序同一个 VL53L0X 封装：tof_init -> upData -> thread_syn -> syn_vl53l0x_distance_mm
    VL53L0X tof_test;
    if (tof_test.tof_init() < 0) {
        program_run_flag = false;
        pthread_kill(quit_thread.native_handle(), SIGTERM);
        quit_thread.join();
        return -1;
    }

    if (vofa.vofa_init() < 0) {
        program_run_flag = false;
        pthread_kill(quit_thread.native_handle(), SIGTERM);
        quit_thread.join();
        return -2;
    }

    while (program_run_flag) {

        tof_test.upData();
        tof_test.thread_syn();

        uint16_t raw = tof_test.syn_vl53l0x_distance_mm;

        // 输出 raw
        vofa.vofa_read((float)raw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        std::this_thread::sleep_for(std::chrono::milliseconds(35));
    }

    program_run_flag = false;
    pthread_kill(quit_thread.native_handle(), SIGTERM);
    quit_thread.join();

    return 0;
}

#endif

// 硬件盲盒任务
// PWM信号
# if 0

#include "headfile.h"

using namespace cv;

std::mutex mtx;
std::condition_variable cv_frame;
std::vector<uchar> global_jpeg_buf;
std::atomic<bool> new_frame_flag(false);
std::atomic<bool> program_run_flag(true);

CameraStreamServer camera_server(&music_player);
std::mutex alg_mutex;

static const char* yes_no(bool value)
{
    return value ? "Y" : "N";
}

static const char* distance_text(BoardDistanceLevel level)
{
    switch (level) {
        case BoardDistanceLevel::far: return "FAR";
        case BoardDistanceLevel::mid: return "MID";
        case BoardDistanceLevel::near: return "NEAR";
        default: return "NONE";
    }
}

static const char* roi_source_text(ModelRoiSource source)
{
    switch (source) {
        case ModelRoiSource::red_marker: return "RED";
        case ModelRoiSource::loose_marker: return "LOOSE";
        default: return "NONE";
    }
}

// 调试显示　将红框视觉类型转为文本描述
static const char* marker_visual_text(MarkerVisualType type)
{
    switch (type) {
        case MarkerVisualType::queue_marker: return "QUEUE";
        case MarkerVisualType::model_marker: return "MODEL";
        case MarkerVisualType::large_supplies: return "SUPPLY";
        case MarkerVisualType::loose_warning: return "WARN";
        default: return "NONE";
    }
}

static void print_rect(const char* name, const cv::Rect& rect)
{
    std::cout << " " << name << "=";
    if (rect.area() <= 0) {
        std::cout << "none";
    }
    else {
        std::cout << "(" << rect.x << "," << rect.y
                  << "," << rect.width << "," << rect.height << ")";
    }
}

static void print_model_status(int frame_id, const ModelActionResult& status)
{
    std::cout << "[model_test]"
              << " frame=" << frame_id
              << " loose=" << yes_no(status.board_detected)
              << " dist=" << distance_text(status.distance)
              << " loose_score=" << std::fixed << std::setprecision(2) << status.board_score
              << " strict_red=" << yes_no(status.marker_detected)
              << " marker_visual=" << marker_visual_text(status.marker_visual_type)
              << " roi_source=" << roi_source_text(status.roi_source)
              << " model=" << yes_no(status.model_ran)
              << " class=" << (status.model.class_name.empty() ? "none" : status.model.class_name)
              << " prob=" << std::fixed << std::setprecision(3) << status.model.probability
              << " scores=S/T/W="
              << std::fixed << std::setprecision(3)
              << status.model.class_scores[0] << "/"
              << status.model.class_scores[1] << "/"
              << status.model.class_scores[2]
              << " detected=" << yes_no(status.model.is_detected)
              << " action=" << model_calib.action_to_ascii(status.action);

    print_rect("loose_rect", status.board_box);
    print_rect("strict_red_rect", status.marker_box);
    print_rect("roi", status.roi_box);
    std::cout << std::endl;
}

void signal_handler_thread()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);

    int sig;
    sigwait(&set, &sig);

    std::cout << "[model_test] exit signal" << std::endl;
    program_run_flag = false;
    camera_server.stop_server();
    cv_frame.notify_all();
}

void frame_get_thread(Camera& cam)
{
    cv::Mat temp_frame;
    global_jpeg_buf.resize(1024 * 300);

    while (program_run_flag) {
        if (!cam.capture_frame(temp_frame, false)) {
            if (!program_run_flag) break;
            std::cerr << "[model_test] capture frame failed" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            std::swap(global_jpeg_buf, cam.jpeg_nowdata);
            new_frame_flag = true;
        }
        cv_frame.notify_one();
    }
}

void frame_handle_thread(CameraStreamServer& server)
{
    cv::Mat raw_frame;
    cv::Mat flip_frame;
    std::vector<uchar> local_jpeg;
    int frame_id = 0;

    while (program_run_flag && server.is_running()) {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame.wait_for(lock, std::chrono::milliseconds(100), [] {
                return new_frame_flag.load() || !program_run_flag.load();
            });

            if (!program_run_flag) break;
            if (!new_frame_flag) continue;

            std::swap(local_jpeg, global_jpeg_buf);
            new_frame_flag = false;
        }

        raw_frame = cv::imdecode(local_jpeg, cv::IMREAD_COLOR);
        if (raw_frame.empty()) {
            std::cerr << "[model_test] jpeg decode failed" << std::endl;
            continue;
        }
        if (raw_frame.cols != CAMERA_WIDTH || raw_frame.rows != CAMERA_HEIGHT) {
            std::cerr << "[model_test] bad frame size "
                      << raw_frame.cols << "x" << raw_frame.rows << std::endl;
            continue;
        }

        cv::flip(raw_frame, flip_frame, -1);

        ModelActionResult status;
        {
            std::lock_guard<std::mutex> alg_lock(alg_mutex);
            track_base.process_frame(flip_frame);
            status = model_detector.process_model_debug_frame(flip_frame);
        }

        print_model_status(frame_id, status);
        server.update_frame_mat(flip_frame);
        frame_id++;
    }
}

extern Motor motor;

int main()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    motor.motor_init(); // 电机初始化

    motor.set_motor1(0,5500);   // 55% 占空比

    while(1)
    {
        sleep(6);
    }
    

    return 0;
}


#endif 
