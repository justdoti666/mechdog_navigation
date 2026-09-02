cmake_minimum_required(VERSION 3.16)
project(mechdog_navigation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows (MSVC) 适配: 源码使用指定初始化器(designated initializers, C++20 特性)
# 与 UTF-8 中文注释/输出; gcc/clang 在 C++17 下作为扩展放行, MSVC 需显式开启
if(MSVC)
    add_compile_options(/std:c++20 /utf-8)
endif()
# ALG-9 (v2.2): GCC/Clang 也显式指定 UTF-8 源/执行字符集 (与 MSVC /utf-8 对齐,
# 避免某些 locale 下中文注释/字符串字面量被按 GBK 解码导致乱码或编译警告)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

# 条件编译选项
option(USE_ASTRA_SDK "Enable Astra Pro via Orbbec Astra SDK (真机模式)" OFF)
option(USE_WIRINGPI "Enable HC-SR04 via WiringPi (Raspberry Pi)" OFF)

# Astra SDK 路径 (可通过 cmake 命令行覆盖: -DASTRA_SDK_ROOT=...)
set(ASTRA_SDK_ROOT "" CACHE PATH "Orbbec Astra SDK 安装根目录")

if(USE_ASTRA_SDK)
    add_definitions(-DUSE_ASTRA_SDK)
    if(NOT ASTRA_SDK_ROOT)
        message(FATAL_ERROR "USE_ASTRA_SDK=ON 需要指定 -DASTRA_SDK_ROOT=<sdk根目录> (例如 D:/orbbec ceram/AstraSDK-... 或 /opt/astra)")
    endif()
    if(NOT EXISTS "${ASTRA_SDK_ROOT}/include/astra/astra.hpp")
        message(FATAL_ERROR "未找到 Astra SDK: ${ASTRA_SDK_ROOT} (用 -DASTRA_SDK_ROOT=... 指定)")
    endif()
    message(STATUS "USE_ASTRA_SDK: 运行时请确保 ${ASTRA_SDK_ROOT}/bin 在 PATH")
endif()

if(USE_WIRINGPI)
    add_definitions(-DUSE_WIRINGPI)
    find_library(WIRINGPI_LIB wiringPi REQUIRED)
endif()

# std::thread 依赖线程库（模拟模式同样使用 Astra 采集线程）
find_package(Threads REQUIRED)

# 源文件
set(SOURCES
    main.cpp
    sensor_ultrasonic.cpp
    sensor_astra.cpp
    sensor_ir.cpp
    sensor_fusion.cpp
    path_planner.cpp
    point_cloud.cpp
    ground_segmentation.cpp
)

# 可执行文件 (主程序为 Windows GDI+ 可视化, Linux 下不构建 —
# main.cpp 大段 #ifdef _WIN32, WSL/g++ 编译报 g_tick_hz 未声明, viz 批遗留)
if(WIN32)
    add_executable(${PROJECT_NAME} ${SOURCES} mapping.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    if(USE_WIRINGPI)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${WIRINGPI_LIB})
    endif()

    target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

    if(USE_ASTRA_SDK)
        # 链接三个库: astra_core / astra_core_api / astra (Windows MSVC 实测需要)
        target_include_directories(${PROJECT_NAME} PRIVATE "${ASTRA_SDK_ROOT}/include")
        target_link_directories(${PROJECT_NAME} PRIVATE "${ASTRA_SDK_ROOT}/lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE astra_core astra_core_api astra)
    endif()
endif()

# 模拟模式（Windows/PC开发）
if(NOT USE_WIRINGPI AND NOT USE_ASTRA_SDK)
    message(STATUS "=== 模拟模式: 所有传感器使用模拟数据 ===")
endif()

# 单元测试（F13）
enable_testing()
add_executable(test_fusion tests/test_fusion.cpp
    sensor_ultrasonic.cpp sensor_astra.cpp sensor_ir.cpp sensor_fusion.cpp)
target_include_directories(test_fusion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_fusion PRIVATE Threads::Threads)
if(USE_ASTRA_SDK)
    target_include_directories(test_fusion PRIVATE "${ASTRA_SDK_ROOT}/include")
    target_link_directories(test_fusion PRIVATE "${ASTRA_SDK_ROOT}/lib")
    target_link_libraries(test_fusion PRIVATE astra_core astra_core_api astra)
endif()
add_test(NAME fusion_tests COMMAND test_fusion)

# 点云模块单测 (P0, 设计文档 docs/POINT_CLOUD_DESIGN.md §13)
add_executable(test_point_cloud tests/test_point_cloud.cpp point_cloud.cpp)
target_include_directories(test_point_cloud PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_point_cloud PRIVATE Threads::Threads)
add_test(NAME point_cloud_tests COMMAND test_point_cloud)

# 地面分割与负障碍单测 (P1): 合成场景, 含门口试金石 (T4)
add_executable(test_ground_segmentation tests/test_ground_segmentation.cpp
    ground_segmentation.cpp point_cloud.cpp)
target_include_directories(test_ground_segmentation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_ground_segmentation PRIVATE Threads::Threads)
add_test(NAME ground_segmentation_tests COMMAND test_ground_segmentation)

# 建图模块单测 (P4, 设计文档 docs/POINT_CLOUD_DESIGN.md P4)
# ground_segmentation: FIX_PLAN #1 insert_cloud_filtered 依赖
add_executable(test_mapping tests/test_mapping.cpp
    point_cloud.cpp mapping.cpp ground_segmentation.cpp)
target_include_directories(test_mapping PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_mapping PRIVATE Threads::Threads)
add_test(NAME mapping_tests COMMAND test_mapping)

# 真机建图验证工具 (P4, 仅 USE_ASTRA_SDK 模式编译, Windows 侧运行)
if(USE_ASTRA_SDK)
    add_executable(mapping_real_test tools/mapping_real_test.cpp
        sensor_astra.cpp point_cloud.cpp mapping.cpp)
    target_include_directories(mapping_real_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_definitions(mapping_real_test PRIVATE USE_ASTRA_SDK)
    target_include_directories(mapping_real_test PRIVATE "${ASTRA_SDK_ROOT}/include")
    target_link_directories(mapping_real_test PRIVATE "${ASTRA_SDK_ROOT}/lib")
    target_link_libraries(mapping_real_test astra_core astra_core_api astra)
    target_compile_features(mapping_real_test PRIVATE cxx_std_20)
    install(TARGETS mapping_real_test DESTINATION bin)
endif()

# 安装 (主程序仅 Windows 构建, 见上方 if(WIN32) 守卫)
if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()
install(FILES config.h DESTINATION include/${PROJECT_NAME})
