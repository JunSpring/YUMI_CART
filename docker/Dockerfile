FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    vim \
    unzip \
    zip \
    htop \
    net-tools \
    iputils-ping \
    locales \
    sudo \
    bash-completion \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-noetic-ackermann-msgs \
    ros-noetic-vision-msgs \
    ros-noetic-serial \
    ros-noetic-joy \
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENV CATKIN_WS=/workspace/YUMI_CART 
WORKDIR /workspace

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

CMD ["/usr/local/bin/entrypoint.sh"]
