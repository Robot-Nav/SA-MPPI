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

# RC-ESDF 库路径
set(RC_ESDF_DIR "$ENV{RC_ESDF_DIR}" CACHE PATH "Path to RC-ESDF library")
if(NOT RC_ESDF_DIR OR NOT EXISTS "${RC_ESDF_DIR}/include/rc_esdf.h")
    set(RC_ESDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../RC-ESDF" CACHE PATH "Path to RC-ESDF library" FORCE)
endif()
message(STATUS "RC-ESDF path: ${RC_ESDF_DIR}")

find_package(Eigen3 REQUIRED)

option(RC_ESDF_VISUALIZE "Enable RC-ESDF visualization (requires OpenCV)" OFF)
if(RC_ESDF_VISUALIZE)
    find_package(OpenCV REQUIRED)
    add_definitions(-DRC_ESDF_VISUALIZE)
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${XTENSOR_INCLUDE_DIRS}
    ${RC_ESDF_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/mppi_ros1_node.cpp
    ${RC_ESDF_DIR}/src/rc_esdf.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
if(RC_ESDF_VISUALIZE)
    target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
endif()

# 安装 Python 脚本为可执行
catkin_install_python(PROGRAMS
    scripts/mppi_path_publisher.py
    scripts/mppi_path_publisher_fz.py
    scripts/JZJ_path.py
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
