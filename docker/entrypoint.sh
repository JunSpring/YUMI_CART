#!/bin/bash
set -e

# ROS 환경 설정
source /opt/ros/noetic/setup.bash

# 워크스페이스 경로
WS_DIR=${CATKIN_WS:-/workspace/YUMI_CART}

mkdir -p /root/.ros/camera_info
ln -sf /yumicart_ws/src/usb_cam/calibration/usb_cam.yaml /root/.ros/camera_info/usb_cam.yaml

# 워크스페이스로 이동
cd "$WS_DIR"

# # 기존 빌드 폴더 삭제
# sudo rm -rf build devel

# catkin_make 실행
catkin_make

# 빌드된 환경 자동 로드
echo "source ${WS_DIR}/devel/setup.bash" >> /root/.bashrc
source "${WS_DIR}/devel/setup.bash"

# bash로 진입
# exec bash

source devel/setup.bash
roslaunch yumicart yumicart.launch
