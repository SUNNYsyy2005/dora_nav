#!/bin/bash

# DoraNav 项目统一编译脚本
# 使用方法:
#   ./compile.sh                    # 编译所有模块
#   ./compile.sh amcl              # 编译AMCL模块
#   ./compile.sh amcl nav slam    # 编译多个模块
#   ./compile.sh -h               # 显示帮助信息
#   ./compile.sh -l               # 列出所有可用模块
#   ./compile.sh -c               # 清理所有编译文件

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目配置
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
MAX_JOBS=$(nproc)

# 可用模块列表
declare -A MODULES=(
    ["amcl"]="AMCL - 自适应蒙特卡洛定位"
    ["nav"]="NAV - 路径规划"
    ["slam"]="SLAM - 同步定位与建图"
    ["lidar"]="LIDAR - 激光雷达驱动"
    ["imu"]="IMU - 惯性测量单元" 
    ["teb"]="TEB - 时序弹性带局部规划器"
    ["control"]="CONTROL - 运动控制"
)

# 模块构建设置
declare -A BUILD_SETTINGS=(
    ["amcl"]="cd build"
    ["nav"]="cd build" 
    ["slam"]="prebuild_makefile"
    ["lidar"]="cd build"
    ["imu"]="cd build"
    ["teb"]="cd build"
    ["control"]="rust"
)

# 帮助信息
show_help() {
    echo -e "${BLUE}=== DoraNav 编译脚本 ===${NC}"
    echo "用法: ./compile.sh [选项] [模块名...]"
    echo ""
    echo "选项:"
    echo "  -h, --help       显示此帮助信息"
    echo "  -l, --list       列出所有可用模块"
    echo "  -c, --clean      清理所有编译文件"
    echo "  -a, --all        编译所有模块"
    echo "  -j, --jobs N     设置并行编译作业数 (默认: $MAX_JOBS)"
    echo "  -v, --verbose    显示详细编译信息"
    echo "  --test           编译后运行测试"
    echo ""
    echo "示例:"
    echo "  ./compile.sh --all                     # 明确编译所有模块"
    echo "  ./compile.sh                           # 编译所有模块（默认行为）"
    echo "  ./compile.sh amcl nav                   # 编译AMCL和NAV模块"
    echo "  ./compile.sh -j 8 slam                 # 使用8个作业编译SLAM模块"
    echo "  ./compile.sh --clean                   # 清理所有编译文件"
    echo "  ./compile.sh --all --test              # 编译所有模块并运行测试"
    echo ""
    echo "可用模块:"
    for module in "${!MODULES[@]}"; do
        echo "  $module"
    done
}

# 列出所有模块
list_modules() {
    echo -e "${BLUE}=== 可用模块 ===${NC}"
    printf "%-10s %s\n" "模块名" "描述"
    echo "----------------------------------------"
    for module in "${!MODULES[@]}"; do
        printf "%-10s %s\n" "$module" "${MODULES[$module]}"
    done
}

# 检查模块是否存在
validate_module() {
    local module=$1
    if [[ ! ${MODULES[$module]+_} ]]; then
        echo -e "${RED}错误: 未知模块 '$module'${NC}"
        echo "使用 './compile.sh -l' 查看所有可用模块"
        exit 1
    fi
}

# 检查依赖
check_dependencies() {
    echo -e "${BLUE}=== 检查依赖 ===${NC}"
    
    # 检查CMake
    if ! command -v cmake &> /dev/null; then
        echo -e "${RED}错误: 未找到CMake${NC}"
        echo "请安装CMake: sudo apt-get install cmake"
        exit 1
    fi
    
    # 检查Make
    if ! command -v make &> /dev/null; then
        echo -e "${RED}错误: 未找到Make${NC}"
        echo "请安装Make: sudo apt-get install make"
        exit 1
    fi
    
    # 检查项目根目录
    if [[ ! -f "$PROJECT_ROOT/cmake_config.cmake" ]]; then
        echo -e "${RED}错误: 未找到cmake_config.cmake配置文件${NC}"
        echo "请确保在项目根目录运行此脚本"
        exit 1
    fi
    
    # 检查环境变量
    if [[ -z "$DORA_NAV_ROOT" ]]; then
        echo -e "${YELLOW}警告: 未设置DORA_NAV_ROOT环境变量${NC}"
        echo "建议运行: source setup_env.sh"
    else
        echo "✓ DORA_NAV_ROOT: $DORA_NAV_ROOT"
    fi
    
    echo "✓ 基础依赖检查通过"
}

# 构建单个模块
build_module() {
    local module=$1
    local jobs=${2:-$MAX_JOBS}
    
    echo -e "${BLUE}=== 编译模块: $module ===${NC}"
    echo "描述: ${MODULES[$module]}"
    
    # 特殊处理control模块（在项目根目录而不是build目录）
    if [[ "$module" == "control" ]]; then
        local module_dir="$PROJECT_ROOT/$module"
    else
        local module_dir="$BUILD_DIR/$module"
    fi
    
    if [[ ! -d "$module_dir" ]]; then
        echo -e "${RED}错误: 模块目录不存在 - $module_dir${NC}"
        return 1
    fi
    
    cd "$module_dir" || exit 1
    
    # 根据模块类型选择构建方式
    if [[ "${BUILD_SETTINGS[$module]}" == "rust" ]]; then
        # Rust项目编译
        echo "检查Rust环境..."
        if ! command -v cargo &> /dev/null; then
            echo -e "${RED}错误: 未找到cargo编译器${NC}"
            echo "请安装Rust: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
            return 1
        fi
        
        echo "开始Rust编译 (使用 $jobs 个作业)..."
        if [[ "$VERBOSE" == "true" ]]; then
            cargo build --release || return 1
        else
            cargo build --release > /dev/null 2>&1 || return 1
        fi
        
        echo "创建符号链接以兼容数据流配置..."
        mkdir -p ../build/$module/build 2>/dev/null || true
        ln -sf ../../../$module/target/release/control ../build/$module/build/test 2>/dev/null || true
        
    elif [[ "${BUILD_SETTINGS[$module]}" == "prebuild_makefile" ]]; then
        # SLAM模块：先运行原始Makefile，再运行CMake
        echo "步骤1: 编译Slam动态库..."
        mkdir -p slambuild
        if [[ "$VERBOSE" == "true" ]]; then
            make clean && make all || return 1
        else
            make clean > /dev/null 2>&1 && make all > /dev/null 2>&1 || return 1
        fi
        
        echo "步骤2: 配置CMake..."
        mkdir -p build
        cd build || exit 1
        
        if [[ "$VERBOSE" == "true" ]]; then
            cmake .. || return 1
        else
            cmake .. > /dev/null 2>&1 || return 1
        fi
        
        echo "步骤3: 编译SLAM程序..."
        if [[ "$VERBOSE" == "true" ]]; then
            make -j$jobs || return 1
        else
            make -j$jobs || return 1
        fi
        
    elif [[ "${BUILD_SETTINGS[$module]}" == "cd build" ]]; then
        # CMake项目编译
        mkdir -p build
        cd build || exit 1
        
        # 配置CMake
        echo "配置CMake..."
        if [[ "$VERBOSE" == "true" ]]; then
            cmake .. || return 1
        else
            cmake .. > /dev/null 2>&1 || return 1
        fi
        
        echo "开始编译 (使用 $jobs 个作业)..."
        if [[ "$VERBOSE" == "true" ]]; then
            make -j$jobs || return 1
        else
            make -j$jobs || return 1
        fi
    else
        # 直接make编译（旧的构建方式）
        echo "开始编译 (使用 $jobs 个作业)..."
        if [[ "$VERBOSE" == "true" ]]; then
            make -j$jobs || return 1
        else
            make -j$jobs || return 1
        fi
    fi
    
    echo -e "${GREEN}✓ 模块 '$module' 编译成功${NC}"
    
    # 返回项目根目录
    cd "$PROJECT_ROOT" || exit 1
    return 0
}

# 编译所有模块
build_all_modules() {
    echo -e "${BLUE}=== 编译所有模块 ===${NC}"
    local failed_modules=()
    
    for module in "${!MODULES[@]}"; do
        echo ""
        if ! build_module "$module" "$JOBS"; then
            failed_modules+=("$module")
            echo -e "${RED}✗ 模块 '$module' 编译失败${NC}"
        fi
    done
    
    echo ""
    if [[ ${#failed_modules[@]} -eq 0 ]]; then
        echo -e "${GREEN}✓ 所有模块编译成功！${NC}"
    else
        echo -e "${RED}✗ 以下模块编译失败: ${failed_modules[*]}${NC}"
        exit 1
    fi
}

# 清理编译文件
clean_all() {
    echo -e "${BLUE}=== 清理编译文件 ===${NC}"
    
    for module in "${!MODULES[@]}"; do
        local module_dir="$BUILD_DIR/$module"
        if [[ -d "$module_dir" ]]; then
            cd "$module_dir" || continue
            
            # 清理build目录
            if [[ -d "build" ]]; then
                echo "清理 $module/build/"
                rm -rf build/
            fi
            
            # 清理make生成的文件
            if [[ -f "Makefile" ]]; then
                echo "清理 $module/Makefile"
                make clean > /dev/null 2>&1 || true
            fi
            
            # 清理生成的临时文件
            echo "清理 $module 的临时文件"
            find . -name "*.o" -delete 2>/dev/null || true
            
            # 特殊模块清理
            if [[ "$module" == "control" ]]; then
                echo "清理 $module 的Rust编译缓存"
                cd "$module_dir" && cargo clean > /dev/null 2>&1 || true
            elif [[ "$module" == "slam" ]]; then
                echo "清理 $module 的动态库文件"
                rm -rf slambuild/ 2>/dev/null || true
            fi
        fi
    done
    
    echo -e "${GREEN}✓ 编译文件清理完成${NC}"
}

# 运行测试
run_tests() {
    echo -e "${BLUE}=== 运行测试 ===${NC}"
    
    for module in "${!MODULES[@]}"; do
        local module_dir="$BUILD_DIR/$module"
        if [[ -d "$module_dir/build" && -f "$module_dir/build/test" ]]; then
            echo "测试 $module 模块..."
            cd "$module_dir/build" && ./test > /dev/null 2>&1 && echo "✓ $module 测试通过" || echo "✗ $module 测试失败"
        elif [[ -d "$module_dir/build" && -f "$module_dir/build/log2pgm" ]] && [[ "$module" == "slam" ]]; then
            echo "SLAM模块log2pgm程序存在"
        fi
    done
    
    cd "$PROJECT_ROOT"
}

# 主函数
main() {
    # 默认参数
    JOBS=$MAX_JOBS
    VERBOSE="false"
    CLEAN_MODE="false"
    BUILD_ALL_MODES="false"
    TEST_MODE="false"
    MODULES_TO_BUILD=()
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -l|--list)
                list_modules
                exit 0
                ;;
            -c|--clean)
                CLEAN_MODE="true"
                shift
                ;;
            -a|--all)
                BUILD_ALL_MODES="true"
                shift
                ;;
            -j|--jobs)
                JOBS="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE="true"
                shift
                ;;
            --test)
                TEST_MODE="true"
                shift
                ;;
            -*)
                echo -e "${RED}错误: 未知选项 '$1'${NC}"
                show_help
                exit 1
                ;;
            *)
                MODULES_TO_BUILD+=("$1")
                shift
                ;;
        esac
    done
    
    # 清理模式
    if [[ "$CLEAN_MODE" == "true" ]]; then
        clean_all
        exit 0
    fi
    
    # 检查依赖
    check_dependencies
    
    # 确定是否编译所有模块
    if [[ "$BUILD_ALL_MODES" == "true" ]]; then
        echo -e "${BLUE}使用 --all 参数编译所有模块${NC}"
        if [[ "$TEST_MODE" == "true" ]]; then
            build_all_modules && run_tests
        else
            build_all_modules
        fi
    elif [[ ${#MODULES_TO_BUILD[@]} -eq 0 ]]; then
        echo -e "${YELLOW}未指定模块，将编译所有模块${NC}"
        if [[ "$TEST_MODE" == "true" ]]; then
            build_all_modules && run_tests
        else
            build_all_modules
        fi
    else
        # 编译指定模块
        local failed_modules=()
        for module in "${MODULES_TO_BUILD[@]}"; do
            validate_module "$module"
        done
        
        for module in "${MODULES_TO_BUILD[@]}"; do
            echo ""
            if ! build_module "$module" "$JOBS"; then
                failed_modules+=("$module")
                echo -e "${RED}✗ 模块 '$module' 编译失败${NC}"
            fi
        done
        
        if [[ ${#failed_modules[@]} -ne 0 ]]; then
            echo -e "${RED}✗ 以下模块编译失败: ${failed_modules[*]}${NC}"
            exit 1
        fi
        
        if [[ "$TEST_MODE" == "true" ]]; then
            echo ""
            run_tests
        fi
    fi
    
    echo ""
    echo -e "${GREEN}🎉 编译任务完成！${NC}"
}

# 运行主函数
main "$@"