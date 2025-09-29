# 统一配置文件 - 管理所有C++项目的库路径和包含目录
# 此文件被所有CMakeLists.txt包含以确保一致的配置

# 项目根目录 - 可以修改此路径来适配不同的系统
set(PROJECT_ROOT "/home/sunny/dora_nav")

# Third Party库路径配置 - 手动编译的依赖库
set(THIRD_PARTY_ROOT "${PROJECT_ROOT}/third_party")
set(THIRD_PARTY_INCLUDE_PATH "${THIRD_PARTY_ROOT}")

# nlohmann/json库配置
set(NLOHMANN_JSON_INCLUDE_DIRS "${THIRD_PARTY_ROOT}/nlohmann_json/include")

# Dora框架路径配置
set(DORA_ROOT "$ENV{HOME}/dora")
set(DORA_INCLUDE_PATH "${DORA_ROOT}/apis/c")
set(DORA_LIB_PATH "${DORA_ROOT}/target/release")

# 系统库路径配置
# 可以根据不同系统或版本进行修改
set(SYSTEM_INCLUDE_BASE "/usr/include")
set(SYSTEM_LIB_BASE "/usr/lib/x86_64-linux-gnu")
set(LOCAL_INCLUDE_BASE "/usr/local/include")
set(LOCAL_LIB_BASE "/usr/local/lib")

# OpenCV配置 - 支持系统安装和manual编译版本
set(OpenCV_SYSTEM_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}/opencv4")
set(OpenCV_SYSTEM_LIBS 
    "${SYSTEM_LIB_BASE}/libopencv_core.so"
    "${SYSTEM_LIB_BASE}/libopencv_imgproc.so"
    "${SYSTEM_LIB_BASE}/libopencv_highgui.so"
    "${SYSTEM_LIB_BASE}/libopencv_imgcodecs.so"
)

# Third Party OpenCV配置（如果要使用手编译版本）
set(OpenCV_THIRD_PARTY_INCLUDE_DIRS "${THIRD_PARTY_ROOT}/opencv/include")
set(OpenCV_THIRD_PARTY_LIBS 
    "${THIRD_PARTY_ROOT}/opencv/lib/libopencv_core.so"
    "${THIRD_PARTY_ROOT}/opencv/lib/libopencv_imgproc.so"
    "${THIRD_PARTY_ROOT}/opencv/lib/libopencv_highgui.so"
    "${THIRD_PARTY_ROOT}/opencv/lib/libopencv_imgcodecs.so"
)

# 默认使用系统OpenCV，如果不存在给出警告信息
if(NOT EXISTS "${OpenCV_SYSTEM_INCLUDE_DIRS}" AND NOT EXISTS "${OpenCV_THIRD_PARTY_INCLUDE_DIRS}")
    message(WARNING "OpenCV库未找到！请安装OpenCV或配置third_party选项")
    message(WARNING "安装方法:")
    message(WARNING "  Ubuntu: sudo apt-get install libopencv-dev")
    message(WARNING "  或下载源码手动编译到 third_party/opencv/ 目录")
endif()

# 智能选择OpenCV路径
if(EXISTS "${OpenCV_SYSTEM_INCLUDE_DIRS}")
    set(OpenCV_INCLUDE_DIRS "${OpenCV_SYSTEM_INCLUDE_DIRS}")
    set(OpenCV_LIBS "${OpenCV_SYSTEM_LIBS}")
    message(STATUS "使用系统OpenCV库: ${OpenCV_INCLUDE_DIRS}")
elseif(EXISTS "${OpenCV_THIRD_PARTY_INCLUDE_DIRS}" AND EXISTS "${OpenCV_THIRD_PARTY_INCLUDE_DIRS}/opencv2")
    set(OpenCV_INCLUDE_DIRS "${OpenCV_THIRD_PARTY_INCLUDE_DIRS}")
    set(OpenCV_LIBS "${OpenCV_THIRD_PARTY_LIBS}")
    message(STATUS "使用Third Party OpenCV库: ${OpenCV_INCLUDE_DIRS}")
else()
    # 设置一个默认路径避免编译错误
    set(OpenCV_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}")
    set(OpenCV_LIBS "")
    message(WARNING "OpenCV库路径未找到，使用默认包含目录")
    message(WARNING "请确保安装了OpenCV或配置了third_party路径")
endif()

# Eigen3配置
set(EIGEN_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}/eigen3")

# SuiteSparse配置
set(SUITESPARSE_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}/suitesparse")
set(SUITESPARSE_LIBRARIES
    "${SYSTEM_LIB_BASE}/libsuitesparseconfig.so"
    "${SYSTEM_LIB_BASE}/libcholmod.so"
    "${SYSTEM_LIB_BASE}/libccolamd.so"
    "${SYSTEM_LIB_BASE}/libcolamd.so"
    "${SYSTEM_LIB_BASE}/libcamd.so"
    "${SYSTEM_LIB_BASE}/libamd.so"
)

# G2O配置 - 支持系统安装和第三方手动编译版本
set(G2O_SYSTEM_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}/g2o")
set(G2O_SYSTEM_LIBRARIES
    "${SYSTEM_LIB_BASE}/libg2o_core.so"
    "${SYSTEM_LIB_BASE}/libg2o_stuff.so"
    "${SYSTEM_LIB_BASE}/libg2o_types_slam2d.so"
    "${SYSTEM_LIB_BASE}/libg2o_types_slam3d.so"
    "${SYSTEM_LIB_BASE}/libg2o_solver_cholmod.so"
    "${SYSTEM_LIB_BASE}/libg2o_solver_pcg.so"
    "${SYSTEM_LIB_BASE}/libg2o_solver_csparse.so"
)

# Third Party G2O配置（TEB模块需要老版本）
set(G2O_THIRD_PARTY_INCLUDE_DIRS "${THIRD_PARTY_ROOT}/g2o/install/include")
set(G2O_THIRD_PARTY_LIBRARIES
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_core.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_stuff.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_types_slam2d.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_types_slam3d.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_solver_cholmod.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_solver_pcg.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_solver_csparse.so"
    "${THIRD_PARTY_ROOT}/g2o/install/lib/libg2o_csparse_extension.so"
)

# 智能选择G2O路径 - TEB模块优先使用老版本
if(G2O_FORCE_THIRD_PARTY OR EXISTS "${G2O_THIRD_PARTY_INCLUDE_DIRS}")
    set(G2O_INCLUDE_DIRS "${G2O_THIRD_PARTY_INCLUDE_DIRS}")
    set(G2O_LIBRARIES "${G2O_THIRD_PARTY_LIBRARIES}")
    message(STATUS "使用Third Party G2O库 (老版本): ${G2O_INCLUDE_DIRS}")
elseif(EXISTS "${G2O_SYSTEM_INCLUDE_DIRS}")
    set(G2O_INCLUDE_DIRS "${G2O_SYSTEM_INCLUDE_DIRS}")
    set(G2O_LIBRARIES "${G2O_SYSTEM_LIBRARIES}")
    message(STATUS "使用系统G2O库: ${G2O_INCLUDE_DIRS}")
else()
    set(G2O_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}")
    set(G2O_LIBRARIES "")
    message(WARNING "G2O库路径未找到，使用默认包含目录")
endif()

# Boost配置
set(BOOST_INCLUDEDIR "${SYSTEM_INCLUDE_BASE}")
set(BOOST_LIBRARYDIR "${SYSTEM_LIB_BASE}")
set(Boost_LIBRARIES
    "${SYSTEM_LIB_BASE}/libboost_system.so"
    "${SYSTEM_LIB_BASE}/libboost_thread.so"
    "${SYSTEM_LIB_BASE}/libboost_graph.so"
)

# FMT格式化库配置
set(FMT_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}")
set(FMT_LIBRARIES "${SYSTEM_LIB_BASE}/libfmt.so")

# LZ4压缩库配置
set(LZ4_INCLUDE_DIR "${SYSTEM_INCLUDE_BASE}")
set(LZ4_LIBRARY "${SYSTEM_LIB_BASE}/liblz4.so")

# Serial通讯库配置
set(SERIAL_INCLUDE_DIRS "${LOCAL_INCLUD_BASE}/serial")
set(SERIAL_LIBRARIES "${LOCAL_LIB_BASE}/libserial.so")

# PCAP网络包捕获库配置
set(PCAP_INCLUDE_DIR "${SYSTEM_INCLUDE_BASE}")
set(PCAP_LIBRARY "-lpcap")

# RL开发库配置 - 假设位于系统目录
set(RT_LIB "-lrt")
set(DL_LIB "-ldl")
set(PTHREAD_LIB "-lpthread")

# YAML配置 - 如果使用yaml-cpp
set(YAMLCPP_INCLUDE_DIRS "${SYSTEM_INCLUDE_BASE}")

# 确保所有路径都存在，如果不存在则使用默认值
if(NOT EXISTS ${DORA_INCLUDE_PATH}/node)
    message(WARNING "Dora头文件路径不存在: ${DORA_INCLUDE_PATH}/node")
endif()

if(NOT EXISTS ${DORA_LIB_PATH}/libdora_node_api_c.a)
    message(WARNING "Dora库文件不存在: ${DORA_LIB_PATH}/libdora_node_api_c.a")
endif()

# 显示配置信息
message(STATUS "配置信息:")
message(STATUS "  项目根目录: ${PROJECT_ROOT}")
message(STATUS "  Dora根目录: ${DORA_ROOT}")
message(STATUS "  Dora头文件: ${DORA_INCLUDE_PATH}/node")
message(STATUS "  Dora库文件: ${DORA_LIB_PATH}/libdora_node_api_c.a")
message(STATUS "  系统库路径: ${SYSTEM_LIB_BASE}")
message(STATUS "  OpenCV库: ${OpenCV_LIBS}")
