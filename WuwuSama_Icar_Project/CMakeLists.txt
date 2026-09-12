cmake_minimum_required(VERSION 3.10)

project(SMARTCAR)

# 设置项目根目录为project文件夹
set(PROJECT_ROOT_DIR /home/wuwu/project)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#此处可修改交叉编译工具路径
set(CROSSTOOL_DIR "/opt/loongson-gnu-toolchain-13.2")
set(CMAKE_C_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-g++)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/code)
include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/include)
include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/wuwu_library)

#编译的库已经放在工程里啦
include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/opencv/include/opencv4)
include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/json/include)
include_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/ncnn/include/ncnn)

file(GLOB_RECURSE MYSRC
    ${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/*.cc
    ${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/*.cpp  
    ${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/smartCar/*.c
)

link_directories(${CROSSTOOL_DIR}/loongarch64-unknown-linux-gnu/lib)
link_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/opencv/lib)
link_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/json/lib)
link_directories(${PROJECT_ROOT_DIR}/SmartCar_99Pai_OpenSource/WuwuSama_Icar_Project/cross_lib/ncnn/lib)

add_executable(main ${MYSRC})
target_link_libraries(main PRIVATE 
                        opencv_core 
                        opencv_imgcodecs
                        opencv_imgproc
                        opencv_videoio
                        jsoncpp
                        ncnn
                        gomp)