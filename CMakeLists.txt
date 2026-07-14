cmake_minimum_required(VERSION 3.16)
project(mechdog_navigation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 条件编译选项
option(USE_OPENNI2  "Enable Astra Pro via OpenNI2" OFF)
option(USE_WIRINGPI "Enable HC-SR04 via WiringPi (Raspberry Pi)" OFF)

if(USE_OPENNI2)
    add_definitions(-DUSE_OPENNI2)
    find_package(OpenNI2 REQUIRED)
endif()

if(USE_WIRINGPI)
    add_definitions(-DUSE_WIRINGPI)
    find_library(WIRINGPI_LIB wiringPi REQUIRED)
endif()

# 源文件
set(SOURCES
    main.cpp
    sensor_ultrasonic.cpp
    sensor_astra.cpp
    sensor_fusion.cpp
    path_planner.cpp
)

# 可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(USE_OPENNI2)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OPENNI2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENNI2_LIBRARIES})
endif()

if(USE_WIRINGPI)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${WIRINGPI_LIB})
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()

# 模拟模式（Windows/PC开发）
if(NOT USE_WIRINGPI AND NOT USE_OPENNI2)
    message(STATUS "=== 模拟模式: 所有传感器使用模拟数据 ===")
endif()

# 安装
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config.h DESTINATION include/${PROJECT_NAME})