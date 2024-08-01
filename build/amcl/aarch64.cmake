# toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 设置工具链路径
set(TOOLCHAIN_PATH /opt/aarch64-linux-musl-cross)

# 指定编译器
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-g++)

# 其他工具
set(CMAKE_AR ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-ar)
set(CMAKE_AS ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-as)
set(CMAKE_NM ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-nm)
set(CMAKE_RANLIB ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-ranlib)
set(CMAKE_STRIP ${TOOLCHAIN_PATH}/bin/aarch64-linux-musl-strip)

# 设置 sysroot，如果需要的话
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH}/aarch64-linux-musl)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# 交叉编译时的搜索路径
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

