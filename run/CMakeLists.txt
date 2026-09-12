cmake_minimum_required(VERSION 3.10)

# 以本 CMakeLists.txt 所在位置推导仓库根目录，避免依赖固定的 /home/wuwu/car 路径
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(ICAR_PROJECT_DIR ${PROJECT_ROOT_DIR}/WuwuSama_Icar_Project)
set(RUN_DIR ${PROJECT_ROOT_DIR}/run)

# 可通过 -DCROSSTOOL_DIR=/path/to/toolchain 覆盖
set(CROSSTOOL_DIR "/opt/loongson-gnu-toolchain-13.2" CACHE PATH "LoongArch cross toolchain root")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)
set(CMAKE_C_COMPILER "${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-g++")

project(SMARTCAR LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 添加当前run目录的头文件路径
include_directories(${RUN_DIR}/inr)
include_directories(${RUN_DIR}/src)

# 添加WuwuSama_Icar_Project项目的头文件路径
include_directories(${ICAR_PROJECT_DIR}/smartCar/code)
include_directories(${ICAR_PROJECT_DIR}/smartCar/include)
include_directories(${ICAR_PROJECT_DIR}/smartCar/wuwu_library)

#编译的库已经放在工程里啦
include_directories(${ICAR_PROJECT_DIR}/cross_lib/opencv/include/opencv4)
include_directories(${ICAR_PROJECT_DIR}/cross_lib/json/include)
include_directories(${ICAR_PROJECT_DIR}/cross_lib/ncnn/include/ncnn)

# 收集源文件 - 当前run目录（可根据需要启用WuwuSama_Icar_Project代码）
file(GLOB RUN_SRC CONFIGURE_DEPENDS
    ${RUN_DIR}/*.cc
    ${RUN_DIR}/*.cpp  
    ${RUN_DIR}/*.c
)

# 添加src目录下的源文件（如果存在）
file(GLOB SRC_FILES CONFIGURE_DEPENDS
    ${RUN_DIR}/src/*.cc
    ${RUN_DIR}/src/*.cpp  
    ${RUN_DIR}/src/*.c
)

list(APPEND RUN_SRC ${SRC_FILES})

# 如需使用WuwuSama_Icar_Project中的代码，请取消以下注释
file(GLOB_RECURSE ICAR_SRC CONFIGURE_DEPENDS
    ${ICAR_PROJECT_DIR}/smartCar/*.cc
    ${ICAR_PROJECT_DIR}/smartCar/*.cpp  
    ${ICAR_PROJECT_DIR}/smartCar/*.c
)

# 当前只使用run目录下的源文件
set(MYSRC ${RUN_SRC})
# 如需包含WuwuSama_Icar_Project代码，请使用：
set(MYSRC ${RUN_SRC} ${ICAR_SRC})

link_directories(${CROSSTOOL_DIR}/loongarch64-unknown-linux-gnu/lib)
link_directories(${ICAR_PROJECT_DIR}/cross_lib/opencv/lib)
link_directories(${ICAR_PROJECT_DIR}/cross_lib/json/lib)
link_directories(${ICAR_PROJECT_DIR}/cross_lib/ncnn/lib)

add_executable(main ${MYSRC})
target_link_libraries(main PRIVATE 
                        opencv_core 
                        opencv_imgcodecs
                        opencv_imgproc
                        opencv_videoio
                        jsoncpp
                        ncnn
                        gomp)
