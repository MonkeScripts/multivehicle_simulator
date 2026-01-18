FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

RUN --mount=type=bind,source=./scripts/install_deps.sh,target=/root/scripts/install_deps.sh \
    /root/scripts/install_deps.sh

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

ARG MICRO_XRCE_DDS_AGENT_VERSION=v2.4.2
ARG PX4_MSGS_VERSION=release/1.16
ARG PX4_ROS2_INTERFACE_LIB_VERSION=release/1.16
ARG PX4_ROS_COM_VERSION=release/1.16
ARG HOME_DIR=/root
ARG PX4_DIR=${HOME_DIR}/px4

RUN --mount=type=bind,source=./scripts/build_ros_deps.sh,target=/root/scripts/build_ros_deps.sh \
    /root/scripts/build_ros_deps.sh \
    ${MICRO_XRCE_DDS_AGENT_VERSION} \
    ${PX4_MSGS_VERSION} \
    ${PX4_ROS2_INTERFACE_LIB_VERSION} \
    ${PX4_ROS_COM_VERSION} \
    ${PX4_DIR}

ARG PX4_VERSION=v1.16.0

RUN --mount=type=bind,source=./scripts/build_px4.sh,target=${PX4_DIR}/scripts/build_px4.sh \
    ${PX4_DIR}/scripts/build_px4.sh \
    ${PX4_VERSION} \
    ${PX4_DIR}

ENV GZ_SIM_RESOURCE_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/models
ENV GZ_SIM_SERVER_CONFIG_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/server.config
