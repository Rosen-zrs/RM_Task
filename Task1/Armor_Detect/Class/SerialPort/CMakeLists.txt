cmake_minimum_required(VERSION 3.0.0)
project(SerialPort VERSION 0.1.0)


add_library(SerialPort STATIC SerialPort.cpp)

target_include_directories(SerialPort PUBLIC include)
