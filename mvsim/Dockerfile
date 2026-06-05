# syntax=docker/dockerfile:1.4
# Self-contained build for the multivehicle (PX4 + ArduSub + USV) simulator,
# plus the bb_robotx_dashboard toolchain. Built via ./build.bash -> mvsim:humble,
# run with rocker via ./run.bash mvsim:humble.
#
# Everything is inlined here (no build-time bind-mounted helper scripts).
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# ---------------------------------------------------------------------------
# 1. System dependencies (formerly scripts/install_deps.sh, now inlined)
# ---------------------------------------------------------------------------
RUN <<'EOF'
set -euo pipefail

# Update and install prerequisites for adding the Gazebo apt repo
apt-get update && apt-get upgrade -y
apt-get install -y --no-install-recommends curl gnupg lsb-release

# Gazebo / OSRF apt repository
curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Main install (alphabetized & consolidated)
apt-get update && apt-get install -y --no-install-recommends \
    atop \
    bash-completion \
    bc \
    build-essential \
    ca-certificates \
    cmake \
    cppcheck \
    dmidecode \
    expect \
    gdb \
    git \
    glmark2 \
    gnutls-bin \
    gstreamer1.0-gl \
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
    libegl1 \
    libeigen3-dev \
    libfcl-dev \
    libgflags-dev \
    libgles2-mesa-dev \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libgoogle-glog-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgz-sim8-dev \
    libimage-exiftool-perl \
    libopencv-dev \
    libspnav-dev \
    libusb-dev \
    libx11-6 \
    libxcb-cursor-dev \
    libxcb-xinerama0 \
    libxext6 \
    libxkbcommon-x11-0 \
    libxml2-utils \
    lsb-release \
    net-tools \
    patch \
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
    python3-wxgtk4.0 \
    rapidjson-dev \
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
    ros-humble-image-common \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-util \
    ros-humble-nav2-common \
    ruby \
    software-properties-common \
    sudo \
    unzip \
    vim \
    wget \
    xterm \
    xvfb

apt-get autoremove -y
apt-get clean
rm -rf /var/lib/apt/lists/*
EOF

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

ARG HOME_DIR=/root

# ---------------------------------------------------------------------------
# 2. PX4 SITL (formerly scripts/build_px4.sh, now inlined as a heredoc;
#    pattern from ros2-docker/dockerfiles/common/Dockerfile.px4_sitl).
#    Runtime is extracted to ${PX4_DIR}/px4_sitl/{bin,romfs} — keep this layout,
#    it is referenced by README.md and the launch flows.
# ---------------------------------------------------------------------------
ARG PX4_DIR=${HOME_DIR}/px4
ARG PX4_VERSION=v1.16.0
RUN <<EOF
set -eo pipefail
git clone --recurse-submodules -b ${PX4_VERSION} \
    https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR}/PX4-Autopilot
cd ${PX4_DIR}/PX4-Autopilot

# Move the PX4_PARAM_* env-var override block to after parameter loading so
# params like PX4_PARAM_UXRCE_DDS_SYNCT survive the autostart defaults.
patch -p1 <<'PATCH'
diff --git a/ROMFS/px4fmu_common/init.d-posix/rcS b/ROMFS/px4fmu_common/init.d-posix/rcS
index fed292b9d1..1d34854077 100644
--- a/ROMFS/px4fmu_common/init.d-posix/rcS
+++ b/ROMFS/px4fmu_common/init.d-posix/rcS
@@ -126,15 +126,6 @@ then
 	set AUTOCNF yes
 fi

-# Allow overriding parameters via env variables: export PX4_PARAM_{name}={value}
-env | while IFS='=' read -r line; do
-  value=${line#*=}
-  name=${line%%=*}
-  case $name in
-    "PX4_PARAM_"*) param set "${name#PX4_PARAM_}" "$value" ;;
-  esac
-done
-
 # multi-instance setup
 # shellcheck disable=SC2154
 param set MAV_SYS_ID $((px4_instance+1))
@@ -239,6 +230,15 @@ then
 	exit 1
 fi

+# Allow overriding parameters via env variables: export PX4_PARAM_{name}={value}
+env | while IFS='=' read -r line; do
+  value=${line#*=}
+  name=${line%%=*}
+  case $name in
+    "PX4_PARAM_"*) param set "${name#PX4_PARAM_}" "$value" ;;
+  esac
+done
+
 dataman start

 # only start the simulator if not in replay mode, as both control the lockstep time
PATCH

./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
make px4_sitl_default
cd ${PX4_DIR}
mkdir -p px4_sitl/bin px4_sitl/romfs/etc
cp -a PX4-Autopilot/build/px4_sitl_default/bin px4_sitl/
tar xf PX4-Autopilot/build/px4_sitl_default/romfs_files.tar -C px4_sitl/romfs/etc
EOF

# ---------------------------------------------------------------------------
# 3. ArduSub (ArduPilot SITL) + ardupilot_gazebo plugin
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# 4. bb_robotx_dashboard toolchain
#    The dashboard source lives in the mounted workspace (not copied in here),
#    so this layer only installs the *tooling*: Node 20 for the Vue frontend,
#    a pinned protoc 33 for the edition-2024 .proto, and the FastAPI backend
#    pip deps. The actual proto-gen / npm build happen at container start via
#    setup_dashboard.sh (see entrypoint below) against the mounted source.
# ---------------------------------------------------------------------------

# Node.js 20 (vite@8 / vue-tsc need Node >= 20; host Node is too old to build
# frontend/dist, so we build it inside the container).
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
 && apt-get install -y nodejs \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Pinned protoc 33 baked offline to /opt/protoc (supports edition = "2024").
# Installed off the system path so it never shadows /usr/bin/protoc;
# setup_dashboard.sh wires it into the dashboard's third_party/protoc/.
ARG PROTOC_VERSION=33.0
RUN <<EOF
set -eo pipefail
arch="$(uname -m)"
case "${arch}" in
  x86_64)        pc_arch="linux-x86_64" ;;
  aarch64|arm64) pc_arch="linux-aarch_64" ;;
  *) echo "unsupported arch: ${arch}" >&2; exit 1 ;;
esac
zip="protoc-${PROTOC_VERSION}-${pc_arch}.zip"
curl -fLo "/tmp/${zip}" \
    "https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOC_VERSION}/${zip}"
mkdir -p /opt/protoc
unzip -q -o "/tmp/${zip}" -d /opt/protoc
chmod +x /opt/protoc/bin/protoc
rm -f "/tmp/${zip}"
/opt/protoc/bin/protoc --version
EOF

# FastAPI backend deps. protobuf must be >= the gencode version baked into
# report_pb2.py: report.proto uses `edition = "2024"`, which only protoc >= 32
# accepts, and that emits gencode 6.33 — so the runtime MUST be 6.x (a 5.x
# runtime raises protobuf VersionError and the dashboard's RoboCommand client
# dies). The isaac README's "<6" rule was to protect opentelemetry-proto, which
# this image doesn't have; verified colcon-core + rclpy + ros2 CLI all work on
# protobuf 7.x here. Still avoid grpcio-tools (it drags setuptools past 80).
RUN pip3 install --no-cache-dir \
        'fastapi' \
        'uvicorn[standard]' \
        'pydantic' \
        'pydantic-settings' \
        'protobuf>=6.33,<8'

# tmux + tmuxp — drive the debug session in
# multivehicle_simulator/tmuxp/mvsim_debug.yaml. Installed late (not in the big
# apt list above) so adding it doesn't bust the PX4/ArduSub build cache.
RUN apt-get update && apt-get install -y --no-install-recommends tmux \
 && apt-get clean && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir tmuxp

# Pin setuptools back down. The PX4/ArduSub prereq scripts above pull in a very
# new setuptools (82.x) into /usr/local, and setuptools >= 70 breaks colcon
# `--symlink-install` of ament_python packages (robotx_gz, bb_robotx_dashboard):
#   error: option --editable not recognized
# Match the isaac_ros_common fix — setuptools 65.7.0 (< 70.0.0).
# See https://github.com/pypa/setuptools/issues/4483. Keep this as the LAST pip
# step so nothing re-upgrades it. PYTHONWARNINGS silences the resulting
# setup.py/easy_install deprecation noise during colcon builds.
RUN pip3 install --no-cache-dir 'setuptools==65.7.0'
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

# Dashboard runtime env vars + one-shot proto bootstrap — baked into ~/.bashrc
# so every interactive shell (and tmux pane) picks them up. We hook via .bashrc
# rather than ENTRYPOINT to stay out of the way of the base ros_entrypoint.sh
# and rocker's own entrypoint wrapping. The bootstrap is idempotent and only
# runs once per shell (DASHBOARD_BOOTSTRAPPED guard); setup_dashboard.sh itself
# no-ops fast when the proto bindings already exist. Rotate
# DASHBOARD_ADMIN_SECRET before exposing the dashboard beyond localhost (needs
# an image rebuild).
COPY scripts/setup_dashboard.sh /usr/local/bin/setup_dashboard.sh
RUN chmod +x /usr/local/bin/setup_dashboard.sh
RUN cat >> /root/.bashrc <<'EOF'

# bb_robotx_dashboard runtime env
export ROBOCOMMAND_HOST=localhost
export ROBOCOMMAND_PORT=9000
export ROBOCOMMAND_TEAM_ID=42
export DASHBOARD_LISTEN_HOST=0.0.0.0
export DASHBOARD_LISTEN_PORT=8080
export DASHBOARD_ADMIN_SECRET=bumblebee

# bb_robotx_dashboard one-shot proto bootstrap (against the mounted workspace)
if [ -z "${DASHBOARD_BOOTSTRAPPED:-}" ]; then
    export DASHBOARD_BOOTSTRAPPED=1
    /usr/local/bin/setup_dashboard.sh || true
fi
EOF

# ---------------------------------------------------------------------------
# 5. Simulation environment exports
# ---------------------------------------------------------------------------
WORKDIR ${HOME_DIR}

ENV GZ_SIM_RESOURCE_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/models
ENV GZ_SIM_SERVER_CONFIG_PATH=${PX4_DIR}/PX4-Autopilot/Tools/simulation/gz/server.config
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUSUB_DIR}/ardupilot_gazebo/build
# NOTE: /opt/protoc/bin is deliberately NOT on PATH — protoc 33 must not shadow
# the system protoc 3.12.4 for general builds (it mismatches libprotobuf and
# would break protoc-using packages). setup_dashboard.sh symlinks /opt/protoc
# into the dashboard's third_party/protoc/ so only compile_protos.sh uses it.
ENV PATH="${ARDUSUB_DIR}/ardupilot/build/sitl/bin:${ARDUSUB_DIR}/ardupilot/Tools/autotest:/root/.local/bin:${PATH}"
