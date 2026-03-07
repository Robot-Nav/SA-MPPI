cmake_minimum_required(VERSION 3.8)
project(mppi_laser_example)

# 设置 C++17 标准（必须！）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 添加 /usr/local 到搜索路径
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ========== 查找 xtl 和 xtensor ==========
find_package(xtl REQUIRED)
find_package(xtensor REQUIRED CONFIG)

# 打印找到的信息
message(STATUS "Found xtensor: ${xtensor_DIR}")
message(STATUS "Found xtl: ${xtl_DIR}")
# ========== 查找结束 ==========

# ========== 查找 BLAS/LAPACK ==========
find_library(BLAS_LIBRARY 
    NAMES openblas blas cblas 
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)
find_library(LAPACK_LIBRARY 
    NAMES lapack 
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(NOT BLAS_LIBRARY)
    message(FATAL_ERROR "BLAS library not found. Please install libopenblas-dev")
endif()

if(NOT LAPACK_LIBRARY)
    message(FATAL_ERROR "LAPACK library not found. Please install liblapack-dev")
endif()

find_path(BLAS_INCLUDE_DIR
    NAMES cblas.h openblas_config.h
    PATHS /usr/include /usr/local/include /usr/include/x86_64-linux-gnu
)

message(STATUS "Found BLAS library: ${BLAS_LIBRARY}")
message(STATUS "Found LAPACK library: ${LAPACK_LIBRARY}")
# ========== 查找结束 ==========

# 创建可执行文件
add_executable(mppi_ros2_node src/mppi_ros2_node.cpp)

# ========== 设置包含目录 ==========
target_include_directories(mppi_ros2_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mppi_laser_example
    ${BLAS_INCLUDE_DIR}
)
# ========== 设置结束 ==========

# 查找 pthread
find_package(Threads REQUIRED)

# ========== 链接库 - 关键修改在这里 ==========
# 使用正确的目标名称 'xtensor' 而不是 'xtensor::xtensor'
target_link_libraries(mppi_ros2_node
    Threads::Threads
    xtensor          # 注意：这里是 xtensor，不是 xtensor::xtensor
    ${BLAS_LIBRARY}
    ${LAPACK_LIBRARY}
    m
)

# xtl 会自动通过 xtensor 的依赖链链接进来
# ========== 链接结束 ==========

# 链接 ROS 依赖
ament_target_dependencies(mppi_ros2_node
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
)

# 安装
install(TARGETS mppi_ros2_node
    DESTINATION lib/${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/mppi_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
