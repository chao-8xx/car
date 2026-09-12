
#include "headfile.h"

// TOF raw test：只依赖 久久派 + TOF 模块
// 目的：绕开摄像头、电机、视觉、元素状态机，只验证 /dev/wuwu_tof 原始 ioctl 数据是否正常。
// 用法：把本文件放到 test/tof_main.cc，按你们 model_main.cc 的方式添加一个测试 target。

// =====================
// 为了让 main 目标链接通过的兼容占位
// 因为 CMake 仍然把 src/*.cc 链接进 main，
// 这些文件会引用原主程序里的全局变量/函数。
// TOF 单独测试时不真正使用它们，只提供符号避免链接失败。
// =====================
std::mutex mtx;
std::condition_variable cv_frame;
std::vector<uchar> global_jpeg_buf;
std::atomic<bool> new_frame_flag(false);

// 注意：Mymenu.cc / pthread_control.cc 里找的是 program_run_flag，
// 不能只写 tof_test_run_flag。
std::atomic<bool> program_run_flag(true);

// preprocess.cc 里会用 alg_mutex。
std::mutex alg_mutex;

// vision_tracking_base.cc / pthread_control.cc 里会找 camera_server。
CameraStreamServer camera_server(&music_player);

// pthread_control.cc 里会找这两个函数。
// TOF 测试不需要摄像头，所以给空实现。
void frame_get_thread(Camera &cam)
{
    (void)cam;
}

void frame_handle_thread(CameraStreamServer &server)
{
    (void)server;
}

static std::atomic<bool> tof_test_run_flag(true);

// 测试参数：不使用 argc/argv，避免你们当前工程 main 签名和 CMake 不一致的问题
static const int TEST_DURATION_SEC = 30;    // 测试总时长，单位秒
static const int READ_INTERVAL_MS  = 35;    // VL53L0X 默认测距预算约33ms，这里用35ms更稳
static const int PRINT_EVERY_N     = 1;     // 每几次打印一次；调试原始跳变建议为1

static const char* yes_no(bool v)
{
    return v ? "Y" : "N";
}

static bool tof_raw_valid(uint16_t raw)
{
    // 当前库里 8192 很常见：可视为初始化值/无效值/超量程异常值
    // 0 和 >2000 先按无效处理。VL53L0X 标称最大约2m。
    return raw != 0 && raw != 8192 && raw <= 2000;
}

void signal_handler_thread()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);

    int sig;
    sigwait(&set, &sig);

    std::cout << "[tof_test] exit signal" << std::endl;
    tof_test_run_flag = false;
}

int main()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    std::thread quit_thread(signal_handler_thread);

    int fd = open(DEVICE_TOF_ADDR, O_RDWR);
    if (fd < 0) {
        perror("[tof_test] open tof failed");
        std::cerr << "[tof_test] 请检查设备节点: ls -l " << DEVICE_TOF_ADDR << std::endl;
        tof_test_run_flag = false;
        pthread_kill(quit_thread.native_handle(), SIGTERM);
        quit_thread.join();
        return -1;
    }

    std::cout << "[tof_test] ready, raw ioctl only" << std::endl;
    std::cout << "[tof_test] device=" << DEVICE_TOF_ADDR
              << " duration=" << TEST_DURATION_SEC << "s"
              << " interval=" << READ_INTERVAL_MS << "ms" << std::endl;
    std::cout << "[tof_test] output: t_ms raw valid filtered invalid_count jump" << std::endl;
    std::cout << "[tof_test] Ctrl+C 可提前退出" << std::endl;

    const auto start_time = std::chrono::steady_clock::now();

    int total_count = 0;
    int valid_count = 0;
    int invalid_count = 0;
    int invalid_zero_count = 0;
    int invalid_8192_count = 0;
    int invalid_over_count = 0;
    int jump_count = 0;

    bool has_last_raw = false;
    uint16_t last_raw = 0;

    bool has_valid = false;
    float filtered = 1000.0f;

    uint16_t min_valid = 65535;
    uint16_t max_valid = 0;
    double sum_valid = 0.0;

    while (tof_test_run_flag) {
        const auto now = std::chrono::steady_clock::now();
        int t_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (t_ms >= TEST_DURATION_SEC * 1000) {
            break;
        }

        uint16_t raw = 8192;
        if (ioctl(fd, VL53L0X_GET_DATA, &raw) < 0) {
            perror("[tof_test] ioctl VL53L0X_GET_DATA failed");
            close(fd);
            tof_test_run_flag = false;
            pthread_kill(quit_thread.native_handle(), SIGTERM);
            quit_thread.join();
            return -2;
        }

        total_count++;

        bool valid = tof_raw_valid(raw);
        bool jump = false;

        if (has_last_raw) {
            int diff = (int)raw - (int)last_raw;
            if (diff < 0) diff = -diff;
            if (diff > 300) {
                jump = true;
                jump_count++;
            }
        }
        last_raw = raw;
        has_last_raw = true;

        if (valid) {
            valid_count++;
            invalid_count = 0;

            if (raw < min_valid) min_valid = raw;
            if (raw > max_valid) max_valid = raw;
            sum_valid += raw;

            if (!has_valid) {
                filtered = (float)raw;
                has_valid = true;
            }
            else {
                filtered = 0.7f * filtered + 0.3f * (float)raw;
            }
        }
        else {
            invalid_count++;
            if (raw == 0) invalid_zero_count++;
            else if (raw == 8192) invalid_8192_count++;
            else if (raw > 2000) invalid_over_count++;

            // 连续无效太多，滤波值回到远距离，方便观察
            if (invalid_count > 5) {
                filtered = 1000.0f;
                has_valid = false;
            }
        }

        if ((total_count % PRINT_EVERY_N) == 0) {
            std::cout << "[tof_test]"
                      << " t_ms=" << t_ms
                      << " raw=" << raw
                      << " valid=" << yes_no(valid)
                      << " filtered=" << std::fixed << std::setprecision(1) << filtered
                      << " invalid_count=" << invalid_count
                      << " jump=" << yes_no(jump)
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(READ_INTERVAL_MS));
    }

    close(fd);
    tof_test_run_flag = false;

    // 测试时间自然结束时，主动唤醒 sigwait 线程，避免 join 卡死
    pthread_kill(quit_thread.native_handle(), SIGTERM);
    quit_thread.join();

    std::cout << "\n========== [tof_test] summary ==========" << std::endl;
    std::cout << "total_count        = " << total_count << std::endl;
    std::cout << "valid_count        = " << valid_count << std::endl;
    std::cout << "invalid_zero_count = " << invalid_zero_count << std::endl;
    std::cout << "invalid_8192_count = " << invalid_8192_count << std::endl;
    std::cout << "invalid_over_count = " << invalid_over_count << std::endl;
    std::cout << "jump_count         = " << jump_count << std::endl;

    if (valid_count > 0) {
        std::cout << "min_valid_mm       = " << min_valid << std::endl;
        std::cout << "max_valid_mm       = " << max_valid << std::endl;
        std::cout << "avg_valid_mm       = " << std::fixed << std::setprecision(1)
                  << (sum_valid / valid_count) << std::endl;
    }
    else {
        std::cout << "[tof_test] 没有读到有效距离。优先检查 TOF 供电、I2C、XSHUT、设备树/驱动、设备节点。" << std::endl;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "[tof_test] Program exited" << std::endl;

    return 0;
}