#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <dirent.h>
#include <sys/stat.h>
#include <cstring>
#include <opencv2/opencv.hpp>

// 按数字自然排序比较函数
struct NaturalCompare {
    bool operator()(const std::string& a, const std::string& b) const {
        // 获取文件名（不带路径）
        std::string a_filename = a.substr(a.find_last_of("/\\") + 1);
        std::string b_filename = b.substr(b.find_last_of("/\\") + 1);
        
        // 移除扩展名
        size_t a_dot = a_filename.find_last_of('.');
        size_t b_dot = b_filename.find_last_of('.');
        if (a_dot != std::string::npos) a_filename = a_filename.substr(0, a_dot);
        if (b_dot != std::string::npos) b_filename = b_filename.substr(0, b_dot);
        
        // 移除 "fps" 前缀
        if (a_filename.compare(0, 3, "fps") == 0) {
            a_filename = a_filename.substr(3);
        }
        if (b_filename.compare(0, 3, "fps") == 0) {
            b_filename = b_filename.substr(3);
        }
        
        // 提取数字
        long a_num = 0, b_num = 0;
        try {
            a_num = std::stol(a_filename);
            b_num = std::stol(b_filename);
        } catch (...) {
            // 如果解析失败，使用字符串排序
            return a_filename < b_filename;
        }
        
        return a_num < b_num;
    }
};

// 检查文件是否存在
bool file_exists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

// 检查是否为目录
bool is_directory(const std::string& path) {
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) != 0) return false;
    return S_ISDIR(statbuf.st_mode);
}

// 收集指定目录下的所有 jpg 文件
std::vector<std::string> collect_jpg_files(const std::string& dir_path) {
    std::vector<std::string> jpg_files;
    
    if (!is_directory(dir_path)) {
        std::cerr << "目录不存在或不是目录: " << dir_path << std::endl;
        return jpg_files;
    }
    
    DIR* dir = opendir(dir_path.c_str());
    if (!dir) {
        std::cerr << "无法打开目录: " << dir_path << std::endl;
        return jpg_files;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        
        // 跳过 . 和 ..
        if (filename == "." || filename == "..") continue;
        
        // 构建完整路径
        std::string full_path = dir_path;
        if (full_path.back() != '/') full_path += "/";
        full_path += filename;
        
        // 检查是否是普通文件
        struct stat statbuf;
        if (stat(full_path.c_str(), &statbuf) != 0) continue;
        if (!S_ISREG(statbuf.st_mode)) continue;
        
        // 检查是否是 jpg/jpeg 文件
        std::string ext = filename;
        size_t dot_pos = ext.find_last_of('.');
        if (dot_pos == std::string::npos) continue;
        
        ext = ext.substr(dot_pos);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        
        if (ext == ".jpg" || ext == ".jpeg") {
            jpg_files.push_back(full_path);
        }
    }
    
    closedir(dir);
    return jpg_files;
}

// 获取有效的视频编码器
int get_valid_fourcc(const std::string& codec_str, const cv::Size& frame_size) {
    std::vector<std::pair<std::string, int> > codec_list;
    codec_list.push_back(std::make_pair("mp4v", cv::VideoWriter::fourcc('M', 'P', '4', 'V')));
    codec_list.push_back(std::make_pair("mp4v", cv::VideoWriter::fourcc('m', 'p', '4', 'v')));
    codec_list.push_back(std::make_pair("xvid", cv::VideoWriter::fourcc('X', 'V', 'I', 'D')));
    codec_list.push_back(std::make_pair("xvid", cv::VideoWriter::fourcc('x', 'v', 'i', 'd')));
    codec_list.push_back(std::make_pair("mjpg", cv::VideoWriter::fourcc('M', 'J', 'P', 'G')));
    codec_list.push_back(std::make_pair("mjpg", cv::VideoWriter::fourcc('m', 'j', 'p', 'g')));
    codec_list.push_back(std::make_pair("x264", cv::VideoWriter::fourcc('X', '2', '6', '4')));
    codec_list.push_back(std::make_pair("h264", cv::VideoWriter::fourcc('H', '2', '6', '4')));
    codec_list.push_back(std::make_pair("avc1", cv::VideoWriter::fourcc('a', 'v', 'c', '1')));
    codec_list.push_back(std::make_pair("i420", cv::VideoWriter::fourcc('I', '4', '2', '0')));
    codec_list.push_back(std::make_pair("i420", cv::VideoWriter::fourcc('i', '4', '2', '0')));
    codec_list.push_back(std::make_pair("iyuv", cv::VideoWriter::fourcc('I', 'Y', 'U', 'V')));
    codec_list.push_back(std::make_pair("uyvy", cv::VideoWriter::fourcc('U', 'Y', 'V', 'Y')));
    codec_list.push_back(std::make_pair("yuy2", cv::VideoWriter::fourcc('Y', 'U', 'Y', '2')));
    
    std::string codec_lower = codec_str;
    std::transform(codec_lower.begin(), codec_lower.end(), codec_lower.begin(), ::tolower);
    
    for (size_t i = 0; i < codec_list.size(); i++) {
        if (codec_list[i].first == codec_lower) {
            // 测试编码器是否可用
            cv::VideoWriter test_writer;
            std::string test_file = "test_encode.mp4";
            if (test_writer.open(test_file, codec_list[i].second, 30, frame_size, true)) {
                test_writer.release();
                remove(test_file.c_str());
                return codec_list[i].second;
            }
        }
    }
    
    // 默认使用 mp4v
    return cv::VideoWriter::fourcc('M', 'P', '4', 'V');
}

// 主编码函数
bool encode_images_to_video(const std::string& input_dir, 
                           const std::string& output_file, 
                           double fps = 30.0,
                           const std::string& codec = "mp4v",
                           int start_frame = 1,
                           int end_frame = 0) {
    
    std::cout << "开始从目录 '" << input_dir << "' 编码视频..." << std::endl;
    
    // 1. 收集所有 jpg 文件
    std::vector<std::string> jpg_files = collect_jpg_files(input_dir);
    if (jpg_files.empty()) {
        std::cerr << "错误: 在目录中未找到 jpg 文件" << std::endl;
        return false;
    }
    
    std::cout << "找到 " << jpg_files.size() << " 个 jpg 文件" << std::endl;
    
    // 2. 按数字顺序排序
    std::sort(jpg_files.begin(), jpg_files.end(), NaturalCompare());
    
    // 3. 应用帧范围
    if (end_frame > 0 && end_frame < (int)jpg_files.size()) {
        jpg_files.resize(end_frame);
    }
    if (start_frame > 1) {
        if (start_frame - 1 < (int)jpg_files.size()) {
            jpg_files.erase(jpg_files.begin(), jpg_files.begin() + (start_frame - 1));
        } else {
            std::cerr << "警告: 起始帧超出范围" << std::endl;
            return false;
        }
    }
    
    if (jpg_files.empty()) {
        std::cerr << "错误: 应用帧范围后没有文件" << std::endl;
        return false;
    }
    
    std::cout << "将处理 " << jpg_files.size() << " 帧" << std::endl;
    
    // 4. 读取第一帧获取尺寸
    cv::Mat first_frame = cv::imread(jpg_files[0], cv::IMREAD_COLOR);
    if (first_frame.empty()) {
        std::cerr << "错误: 无法读取第一帧图片: " << jpg_files[0] << std::endl;
        return false;
    }
    
    cv::Size frame_size(first_frame.cols, first_frame.rows);
    std::cout << "图片尺寸: " << frame_size.width << "x" << frame_size.height << std::endl;
    
    // 5. 创建视频写入器
    int fourcc = get_valid_fourcc(codec, frame_size);
    std::cout << "使用编码器: ";
    std::cout << char(fourcc & 0xFF) 
              << char((fourcc >> 8) & 0xFF) 
              << char((fourcc >> 16) & 0xFF) 
              << char((fourcc >> 24) & 0xFF) << std::endl;
    
    cv::VideoWriter writer;
    bool is_opened = writer.open(output_file, fourcc, fps, frame_size, true);
    
    if (!is_opened) {
        std::cerr << "错误: 无法创建视频文件: " << output_file << std::endl;
        std::cerr << "请尝试: " << std::endl;
        std::cerr << "1. 使用不同的编码器 (--codec mjpg)" << std::endl;
        std::cerr << "2. 检查输出路径是否可写" << std::endl;
        std::cerr << "3. 确保 OpenCV 支持该编码器" << std::endl;
        return false;
    }
    
    std::cout << "开始编码视频: " << output_file << std::endl;
    std::cout << "帧率: " << fps << " fps" << std::endl;
    
    // 6. 逐帧编码
    int frame_count = 0;
    int success_count = 0;
    int failed_count = 0;
    
    for (size_t i = 0; i < jpg_files.size(); i++) {
        frame_count++;
        
        cv::Mat frame = cv::imread(jpg_files[i], cv::IMREAD_COLOR);
        if (frame.empty()) {
            std::cerr << "警告: 无法读取图片: " << jpg_files[i] << std::endl;
            failed_count++;
            continue;
        }
        
        // 确保尺寸匹配
        if (frame.size() != frame_size) {
            cv::resize(frame, frame, frame_size, 0, 0, cv::INTER_LINEAR);
        }
        
        writer << frame;
        success_count++;
        
        // 显示进度
        if (frame_count % 100 == 0) {
            float progress = (float)frame_count / jpg_files.size() * 100.0f;
            std::cout << "\r进度: " << frame_count << "/" << jpg_files.size() 
                     << " (" << std::fixed << std::setprecision(1) << progress << "%)" 
                     << std::flush;
        }
    }
    
    writer.release();
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "视频编码完成!" << std::endl;
    std::cout << "输出文件: " << output_file << std::endl;
    std::cout << "尺寸: " << frame_size.width << "x" << frame_size.height << std::endl;
    std::cout << "帧率: " << fps << " fps" << std::endl;
    std::cout << "总帧数: " << frame_count << std::endl;
    std::cout << "成功: " << success_count << " 帧" << std::endl;
    if (failed_count > 0) {
        std::cout << "失败: " << failed_count << " 帧" << std::endl;
    }
    std::cout << "========================================" << std::endl;
    
    return success_count > 0;
}

// 显示使用帮助
void show_help(const char* prog_name) {
    std::cout << "图片序列转视频编码器" << std::endl;
    std::cout << "用法: " << prog_name << " [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -i, --input DIR     输入目录 (默认: playback)" << std::endl;
    std::cout << "  -o, --output FILE   输出视频文件 (默认: output.mp4)" << std::endl;
    std::cout << "  -f, --fps FPS       视频帧率 (默认: 60.0)" << std::endl;
    std::cout << "  -c, --codec CODEC   视频编码器 (默认: mp4v)" << std::endl;
    std::cout << "  -s, --start N       起始帧号 (从1开始, 默认: 1)" << std::endl;
    std::cout << "  -e, --end N         结束帧号 (包含, 默认: 全部)" << std::endl;
    std::cout << "  -h, --help          显示此帮助信息" << std::endl;
    std::cout << std::endl;
    std::cout << "支持的编码器:" << std::endl;
    std::cout << "  mp4v, xvid, mjpg, x264, h264, avc1" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << prog_name << " -i playback -o output.mp4 -f 25" << std::endl;
    std::cout << "  " << prog_name << " -i frames -o video.avi -c mjpg -f 30" << std::endl;
    std::cout << "  " << prog_name << " -i playback -o clip.mp4 -s 100 -e 500" << std::endl;
}

int main(int argc, char* argv[]) {
    // 默认参数
    std::string input_dir = "playback";
    std::string output_file = "output.mp4";
    double fps = 60.0;
    std::string codec = "mp4v";
    int start_frame = 1;
    int end_frame = 0;
    
    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            show_help(argv[0]);
            return 0;
        }
        else if (arg == "-i" || arg == "--input") {
            if (i + 1 < argc) {
                input_dir = argv[++i];
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) {
                output_file = argv[++i];
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else if (arg == "-f" || arg == "--fps") {
            if (i + 1 < argc) {
                try {
                    fps = std::stod(argv[++i]);
                } catch (...) {
                    std::cerr << "错误: 无效的帧率值" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else if (arg == "-c" || arg == "--codec") {
            if (i + 1 < argc) {
                codec = argv[++i];
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else if (arg == "-s" || arg == "--start") {
            if (i + 1 < argc) {
                try {
                    start_frame = std::stoi(argv[++i]);
                } catch (...) {
                    std::cerr << "错误: 无效的起始帧号" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else if (arg == "-e" || arg == "--end") {
            if (i + 1 < argc) {
                try {
                    end_frame = std::stoi(argv[++i]);
                } catch (...) {
                    std::cerr << "错误: 无效的结束帧号" << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "错误: " << arg << " 选项需要参数" << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "未知选项: " << arg << std::endl;
            show_help(argv[0]);
            return 1;
        }
    }
    
    // 验证参数
    if (fps <= 0) {
        std::cerr << "错误: 帧率必须大于0" << std::endl;
        return 1;
    }
    
    if (start_frame < 1) {
        std::cerr << "错误: 起始帧号必须大于等于1" << std::endl;
        return 1;
    }
    
    if (end_frame > 0 && end_frame < start_frame) {
        std::cerr << "错误: 结束帧号必须大于起始帧号" << std::endl;
        return 1;
    }
    
    // 运行编码
    bool success = encode_images_to_video(input_dir, output_file, fps, codec, start_frame, end_frame);
    
    return success ? 0 : 1;
}