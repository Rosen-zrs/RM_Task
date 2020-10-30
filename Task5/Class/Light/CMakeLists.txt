# 查找当前目录下的所有源文件

# 并将名称保存到 DIR_LIB_SRCS 变量

aux_source_directory(. DIR_LIGHT)

include_directories(../include)
# include_directories(/usr/local/include/opencv4/opencv2)

# 生成链接库

add_library (Light ${DIR_LIGHT})
