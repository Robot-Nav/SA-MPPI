#!/usr/bin/env bash
set -euo pipefail

# SA-MPPI / MPPI_C++ ROS1 Noetic 环境配置脚本
# 推荐在 Ubuntu 22.04 主机上运行，用 Docker 创建 Ubuntu 20.04 + ROS Noetic 环境。

CONTAINER_NAME="sa_mppi_noetic"
IMAGE_NAME="osrf/ros:noetic-desktop-full"
WORKSPACE="/root/MPPI_ws_ros1sim"
REPO_URL="https://github.com/Robot-Nav/SA-MPPI.git"
REPO_BRANCH="MPPI_C++"

echo "[1/7] 检查 Docker"

if ! command -v docker >/dev/null 2>&1; then
    sudo apt update
    sudo apt install -y docker.io git
    sudo systemctl enable docker
    sudo systemctl start docker
    sudo usermod -aG docker "$USER" || true
    echo "Docker 已安装。首次安装后建议重新登录终端，或者当前命令继续使用 sudo docker。"
fi

DOCKER_CMD="docker"
if ! docker ps >/dev/null 2>&1; then
    DOCKER_CMD="sudo docker"
fi

echo "[2/7] 拉取 ROS Noetic 镜像"
$DOCKER_CMD pull "$IMAGE_NAME"

echo "[3/7] 配置 X11，RViz 需要"
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

echo "[4/7] 创建或启动容器"
if ! $DOCKER_CMD ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    mkdir -p "$HOME/SA_MPPI_data"

    $DOCKER_CMD run -dit \
        --name "$CONTAINER_NAME" \
        --net=host \
        --privileged \
        -e DISPLAY="${DISPLAY:-:0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$HOME/SA_MPPI_data:/root/SA_MPPI_data" \
        "$IMAGE_NAME" \
        bash
else
    $DOCKER_CMD start "$CONTAINER_NAME" >/dev/null
fi

echo "[5/7] 容器内安装系统依赖"
$DOCKER_CMD exec "$CONTAINER_NAME" bash -lc '
set -euo pipefail

apt update

DEBIAN_FRONTEND=noninteractive apt install -y \
  git \
  build-essential \
  cmake \
  python3-pip \
  python3-rosdep \
  python3-catkin-tools \
  ros-noetic-tf \
  ros-noetic-rviz

rosdep init 2>/dev/null || true
rosdep update

grep -qxF "source /opt/ros/noetic/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
'

echo "[6/7] 下载 SA-MPPI MPPI_C++ 分支和第三方头文件库"
$DOCKER_CMD exec "$CONTAINER_NAME" bash -lc "
set -euo pipefail
source /opt/ros/noetic/setup.bash

if [ ! -d '$WORKSPACE/.git' ]; then
    rm -rf '$WORKSPACE'
    git clone -b '$REPO_BRANCH' '$REPO_URL' '$WORKSPACE'
fi

cd '$WORKSPACE'
git fetch origin
git checkout '$REPO_BRANCH'
git pull --ff-only || true

cd '$WORKSPACE/src/mppi_laser_example'

if [ ! -d xtl ]; then
    git clone https://github.com/xtensor-stack/xtl.git
    cd xtl && git checkout 0.7.0 && cd ..
fi

if [ ! -d xsimd ]; then
    git clone https://github.com/xtensor-stack/xsimd.git
    cd xsimd && git checkout 7.4.10 && cd ..
fi

if [ ! -d xtensor ]; then
    git clone https://github.com/xtensor-stack/xtensor.git
    cd xtensor && git checkout 0.21.0 && cd ..
fi
"

echo "[7/7] 安装 ROS 依赖并编译"
$DOCKER_CMD exec "$CONTAINER_NAME" bash -lc "
set -euo pipefail
source /opt/ros/noetic/setup.bash
cd '$WORKSPACE'

rosdep install --from-paths src --ignore-src -r -y || true
catkin_make -DCMAKE_BUILD_TYPE=Release

grep -qxF \"source $WORKSPACE/devel/setup.bash\" ~/.bashrc || \
  echo \"source $WORKSPACE/devel/setup.bash\" >> ~/.bashrc
"

cat <<EOF

配置完成。

进入容器：
$DOCKER_CMD exec -it $CONTAINER_NAME bash

容器内启动 roscore：
source $WORKSPACE/devel/setup.bash
roscore

另开终端进入容器，启动控制器：
source $WORKSPACE/devel/setup.bash
rosparam load \$(rospack find mppi_laser_example)/config/mppi_params.yaml
rosrun mppi_laser_example mppi_ros1_node

如果 mppi_ros1_node 不存在，查看实际可执行文件：
find $WORKSPACE/devel/lib/mppi_laser_example -maxdepth 1 -type f -executable

发布路径：
source $WORKSPACE/devel/setup.bash
rosrun mppi_laser_example mppi_path_publisher.py

启动 RViz：
source $WORKSPACE/devel/setup.bash
rviz -d \$(rospack find mppi_laser_example)/config/mppi_test.rviz

EOF
