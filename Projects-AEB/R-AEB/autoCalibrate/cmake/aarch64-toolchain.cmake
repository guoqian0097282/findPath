set(ARM_GCC_ROOT
    "/home/gq/guoqian/AFA_Algorithm/findPath/cmake/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu"
    CACHE PATH "ARM GNU toolchain installation")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER
    "${ARM_GCC_ROOT}/bin/aarch64-none-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER
    "${ARM_GCC_ROOT}/bin/aarch64-none-linux-gnu-g++")
