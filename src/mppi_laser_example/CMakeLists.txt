cmake_minimum_required(VERSION 3.1)
project(mppi_laser_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    nav_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs sensor_msgs tf
)

# 内嵌 xtensor 第三方库路径
set(XTENSOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/xtensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xsimd/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xtl/include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${XTENSOR_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/mppi_ros1_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

# 安装 Python 脚本为可执行
catkin_install_python(PROGRAMS
    scripts/mppi_path_publisher.py
    scripts/mppi_path_publisher_fz.py
    scripts/JZJ_path.py
    scripts/clean_path.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# 安装 RViz 配置
install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
