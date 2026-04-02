##################################
# 配置 ARM 交叉编译
##################################

# 使用方法：同arm版本cmake共同使用。
# 首先，将TOOLCHAIN_DIR设置为本机上的交叉编译器路径。
# 然后，将文件内容复制到arm版本cmake中，或者使用命令：cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm_linux_setup.cmake ..

message(STATUS "*****************************************************")
message(STATUS "** build on arm platform ")

# 配置 ARM 交叉编译
set(CMAKE_SYSTEM_NAME Linux) #设置目标系统名字
set(CMAKE_SYSTEM_PROCESSOR arm) #设置目标处理器架构

# 指定编译器的 sysroot 路径
set(TOOLCHAIN_ROOT ${CMAKE_SOURCE_DIR}/cmake)
if(PLATFORM STREQUAL "j3-arm")
    set(TOOLCHAIN_DIR ${TOOLCHAIN_ROOT}/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu)         
# elseif(PLATFORM STREQUAL "tda4-arm")
#     set(TOOLCHAIN_DIR ${TOOLCHAIN_ROOT}/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu) 
endif()
message(STATUS "** PLATFORM      : ${PLATFORM}")
message(STATUS "** TOOLCHAIN_ROOT: ${TOOLCHAIN_ROOT}")
message(STATUS "** TOOLCHAIN_DIR : ${TOOLCHAIN_DIR}")

# 指定交叉编译器 arm-gcc 和 arm-g++
set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-none-linux-gnu-g++)

# 为编译器添加编译选项
# set(CMAKE_C_FLAGS "-march=armv7ve -mfpu=neon -mfloat-abi=hard -mcpu=a72")
# set(CMAKE_CXX_FLAGS "-march=armv7ve -mfpu=neon -mfloat-abi=hard -mcpu=a72")

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR}/bin)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

message(STATUS "*****************************************************")

##################################
# end
##################################