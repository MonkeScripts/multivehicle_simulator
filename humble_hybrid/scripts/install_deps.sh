#!/usr/bin/env bash
set -euo pipefail

# 1. Update and install prerequisites
apt-get update && apt-get upgrade -y
apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release

# 2. Setup Gazebo/OSRF Repository
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Main Installation (Alphabetized & Consolidated)
apt-get update && apt-get install -y --no-install-recommends \
    atop \
    bc \
    build-essential \
    ca-certificates \
    cmake \
    cppcheck \
    dmidecode \
    expect \
    gdb \
    git \
    gnutls-bin \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gz-harmonic \
    iputils-ping \
    libbluetooth-dev \
    libboost-all-dev \
    libccd-dev \
    libcwiid-dev \
    libeigen3-dev \
    libfcl-dev \
    libgflags-dev \
    libgles2-mesa-dev \
    libgoogle-glog-dev \
    libgstreamer-plugins-base1.0-dev \
    libimage-exiftool-perl \
    libspnav-dev \
    libusb-dev \
    libxcb-cursor-dev \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxml2-utils \
    lsb-release \
    net-tools \
    pkg-config \
    protobuf-compiler \
    python3-dbg \
    python3-empy \
    python3-pip \
    python3-rospkg \
    python3-sdformat13 \
    python3-setuptools \
    python3-vcstool \
    python3-venv \
    ros-humble-actuator-msgs \
    ros-humble-ament-cmake-pycodestyle \
    ros-humble-desktop \
    ros-humble-foxglove-bridge \
    ros-humble-gps-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-joy-linux \
    ros-humble-joy-teleop \
    ros-humble-mavros-msgs \
    ros-humble-radar-msgs \
    ros-humble-ros-gzharmonic \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-plot \
    ros-humble-rqt-topic \
    ros-humble-rviz2 \
    ros-humble-vision-msgs \
    ros-humble-xacro \
    ruby \
    software-properties-common \
    sudo \
    vim \
    wget \
    xvfb

# 4. Cleanup
apt-get autoremove -y
apt-get clean
rm -rf /var/lib/apt/lists/*