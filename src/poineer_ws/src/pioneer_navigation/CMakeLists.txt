cmake_minimum_required(VERSION 3.0.2)
project(pioneer_navigation)

find_package(catkin REQUIRED)

catkin_package(
  # 本包仅提供 launch 和配置文件，无库或节点
)

###########
## 安装 ##
###########

# 安装 launch 文件
install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   PATTERN ".svn" EXCLUDE
)

# 安装 navigation 参数文件
install(DIRECTORY navigation/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/navigation
   PATTERN ".svn" EXCLUDE
)

#############
## 测试 ##
#############
# catkin_add_gtest() ...
