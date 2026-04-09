FROM osrf/ros:humble-desktop-full
LABEL maintainer="Jaehyun Yi <jaehyun.yi@samsung.com>"

# ── Build-time proxy ──────────────────────────────────────────────────────────
ENV http_proxy=http://12.26.204.100:8080
ENV https_proxy=http://12.26.204.100:8080
ENV HTTP_PROXY=http://12.26.204.100:8080
ENV HTTPS_PROXY=http://12.26.204.100:8080

RUN echo 'Acquire::http::Proxy "http://12.26.204.100:8080";' \
      > /etc/apt/apt.conf.d/99proxy

# ── Certificates ─────────────────────────────────────────────────────────────
# McAfee_Certificate.crt must be present in docker/ before building.
# Comment out the two lines below if not in a proxied network.
COPY docker/McAfee_Certificate.crt /usr/share/ca-certificates/McAfee_Certificate.crt
RUN update-ca-certificates

COPY docker/pip.conf /root/.pip/pip.conf

# ── Git SSL / proxy ───────────────────────────────────────────────────────────
RUN git config --global http.sslverify false && \
    git config --global http.proxy  http://12.26.204.100:8080 && \
    git config --global https.proxy http://12.26.204.100:8080

# ── Locale & timezone ─────────────────────────────────────────────────────────
ENV TZ=Asia/Seoul
RUN apt-get update && apt-get install -y --no-install-recommends locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN rm -f /bin/sh && ln -s /bin/bash /bin/sh

# ── System packages ───────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
    curl \
    wget \
    nano \
    vim \
    gdb \
    g++ \
    build-essential \
    software-properties-common \
    x11-xserver-utils \
    libxcb-cursor-dev \
    libncurses-dev \
    xauth \
    htop \
    bash-completion && \
    rm -rf /var/lib/apt/lists/*

# ── ROS2 extras ───────────────────────────────────────────────────────────────
# ros-humble-desktop-full already includes geometry/sensor/std/action/tf2 msgs.
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# ── ManusSDK shared libraries ─────────────────────────────────────────────────
# ManusSDK/ must be present in repo root (tracked via Git LFS — see README).
# Headers are accessed from the workspace volume mount during colcon build.
COPY ManusSDK/lib/libManusSDK.so           /usr/local/lib/
COPY ManusSDK/lib/libManusSDK_Integrated.so /usr/local/lib/
RUN ldconfig

# ── Python packages ───────────────────────────────────────────────────────────
RUN pip install --no-cache-dir \
    empy==3.3.4 \
    pin \
    pink \
    scipy \
    openvr \
    evdev \
    "PySide6==6.7.3" \
    open3d \
    loop-rate-limiters \
    rby1-sdk

# ── Unset proxy (rby1-sdk and robot network need direct connection) ────────────
ENV http_proxy=""
ENV https_proxy=""
ENV HTTP_PROXY=""
ENV HTTPS_PROXY=""

# ── Shell setup ───────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/home/ros2_ws/ManusSDK/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY" >> ~/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
    echo "alias sr='source /home/ros2_ws/install/setup.bash'" >> ~/.bashrc && \
    echo "alias cb='cd /home/ros2_ws && colcon build'" >> ~/.bashrc && \
    echo "alias cbr='cd /home/ros2_ws && colcon build --symlink-install'" >> ~/.bashrc

WORKDIR /home/ros2_ws
