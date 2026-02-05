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

ARG HOME_DIR=/root

ARG PX4_DIR=${HOME_DIR}/px4
ARG PX4_VERSION=v1.16.0
RUN mkdir -p ${PX4_DIR}
RUN --mount=type=bind,source=./scripts/build_px4.sh,target=${PX4_DIR}/scripts/build_px4.sh \
    ${PX4_DIR}/scripts/build_px4.sh \
    ${PX4_VERSION} \
    ${PX4_DIR}

ARG ARDUSUB_DIR=${HOME_DIR}/auv

WORKDIR ${ARDUSUB_DIR}
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

WORKDIR ${ARDUSUB_DIR}/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN sed -i 's/if \[ \$EUID == 0 \]/if [ 1 == 0 ]/g' Tools/environment_install/install-prereqs-ubuntu.sh \
 && sed -i 's/PIP_USER_ARGUMENT="--user"/PIP_USER_ARGUMENT=""/g' Tools/environment_install/install-prereqs-ubuntu.sh \
 && sed -i 's/sudo usermod -a -G dialout $USER/true/g' Tools/environment_install/install-prereqs-ubuntu.sh
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
RUN modules/waf/waf-light configure --board sitl \
 && modules/waf/waf-light build --target bin/ardusub

WORKDIR ${ARDUSUB_DIR}/ardupilot
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

ENV GZ_VERSION=harmonic

# Build ardupilot_gazebo
RUN [ "/bin/bash" , "-c" , " \
  cd ardupilot_gazebo \
  && git checkout 4b30a3d8 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && make -j4" ]

RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && ./install_geographiclib_datasets.sh" ]

WORKDIR ${HOME_DIR}

ENV GZ_SIM_RESOURCE_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/models
ENV GZ_SIM_SERVER_CONFIG_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/server.config
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUSUB_DIR}/ardupilot_gazebo/build
ENV PATH="${ARDUSUB_DIR}/ardupilot/build/sitl/bin:${ARDUSUB_DIR}/ardupilot/Tools/autotest:/root/.local/bin:${PATH}"
