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
