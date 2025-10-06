#!/bin/bash
set -e

# ROS 환경 설정
source /opt/ros/noetic/setup.bash

# 워크스페이스 경로
WS_DIR=${CATKIN_WS:-/workspace/YUMI_CART}

# 워크스페이스로 이동
cd "$WS_DIR"

# 기존 빌드 폴더 삭제
sudo rm -rf build devel

# catkin_make 실행
catkin_make

# 빌드된 환경 자동 로드
echo "source ${WS_DIR}/devel/setup.bash" >> /root/.bashrc
source "${WS_DIR}/devel/setup.bash"

# bash로 진입
exec bash
