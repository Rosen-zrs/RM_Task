# CMake 最低版本号要求

cmake_minimum_required (VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 项目信息

project (Armor_Detect)

include(CTest)
enable_testing()

find_package(OpenCV  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# 查找当前目录下的所有源文件

# 并将名称保存到 DIR_SRCS 变量

aux_source_directory(. DIR_SRCS)

include_directories(../Class/include)
include_directories(/usr/local/include/opencv4/opencv2)

# 添加 math 子目录

add_subdirectory(Class/Armor)
add_subdirectory(Class/Light)
add_subdirectory(Class/SerialPort)
add_subdirectory(Class/RMVideoCapture)
add_subdirectory(Class/Armor_Link)


# 指定生成目标

add_executable(Detect main.cpp)

# 添加链接库

target_link_libraries(Detect ${OpenCV_LIBS})
target_link_libraries(Detect Armor)
target_link_libraries(Detect Light)
target_link_libraries(Detect SerialPort)
target_link_libraries(Detect RMVideoCapture)
target_link_libraries(Detect Armor_Link)
