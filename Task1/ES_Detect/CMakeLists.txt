# CMake 最低版本号要求

cmake_minimum_required (VERSION 2.8)
#SET(cmake_build_type "DEBUG")

# 项目信息

project (ES_Detect)

find_package(OpenCV  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# 查找当前目录下的所有源文件

# 并将名称保存到 DIR_SRCS 变量

aux_source_directory(. DIR_SRCS)

# include_directories(../Class/include)
# include_directories(/usr/local/include/opencv4/opencv2)

# 添加 math 子目录

add_subdirectory(Class/Energy_Switch)

# 指定生成目标

add_executable(Detect main.cpp)

# 添加链接库

target_link_libraries(Detect ${OpenCV_LIBS})
target_link_libraries(Detect Energy_Switch)
