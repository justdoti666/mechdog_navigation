cmake_minimum_required(VERSION 3.16)
project(mechdog_navigation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows (MSVC) 适配: 源码使用指定初始化器(designated initializers, C++20 特性)
# 与 UTF-8 中文注释/输出; gcc/clang 在 C++17 下作为扩展放行, MSVC 需显式开启
if(MSVC)
    add_compile_options(/std:c++20 /utf-8)
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
)

# 可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

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

# 安装
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config.h DESTINATION include/${PROJECT_NAME})
