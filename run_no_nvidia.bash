#!/usr/bin/env bash

#
# SPDX-License-Identifier: Apache-2.0
# Adapted from OSRF dockwater (Apache-2.0), Copyright (C) 2018 Open Source
# Robotics Foundation; modified for multivehicle_sim by MonkeScripts.
#

# Runs the mvsim container WITHOUT NVIDIA GPU passthrough, for hosts with an
# Intel/AMD integrated GPU (or no discrete NVIDIA GPU). Rendering falls back to
# Mesa — the host's iGPU via /dev/dri, or llvmpipe software rendering — which is
# slower, so expect a low real-time factor in Gazebo.
#
# IMPORTANT: launch the world with nvidia_offload:=false so gz does not attempt
# the NVIDIA PRIME render-offload, e.g.:
#   ros2 launch multivehicle_sim world.launch.py nvidia_offload:=false
# On a host with no X server, also pass headless:=true.
#
# For NVIDIA hosts use ./run.bash instead.
#
# Requires:
#   docker
#   an X server (for the GUI; omit on a headless host)
#   rocker
# Recommended:
#   A joystick mounted to /dev/input/js0
############################################################
# Help                                                     #
############################################################
Help()
{
   echo "Runs the mvsim container without NVIDIA GPU passthrough (Mesa/iGPU)."
   echo
   echo "Syntax: run_no_nvidia.bash [-h] [IMAGE]"
   echo "  IMAGE   docker image to run (default: mvsim:humble)"
   echo "options:"
   echo "  h     Print this help message and exit."
   echo
   echo "Remember to launch the world with nvidia_offload:=false."
}

while getopts ":h" option; do
  case $option in
    h) Help; exit;;
  esac
done

JOY=/dev/input/js0

# Devices: the DRI render node gives the container the host GPU (iGPU) for Mesa.
# Add the joystick only if it exists, so rocker doesn't error on machines
# without one.
DEVICES="/dev/dri"
if [ -e "$JOY" ]; then
  DEVICES="/dev/dri $JOY"
fi

# Same as run.bash but WITHOUT --nvidia. --x11 forwards the display for the GUI;
# drop it (and pass headless:=true to the world launch) on a headless host.
ROCKER_ARGS="--devices $DEVICES --dev-helpers --x11 --git --volume "$HOME":/root/HOST --network=host"

IMG_NAME=${@:$OPTIND:1}
IMG_NAME=${IMG_NAME:-mvsim:humble}

# Replace `:` with `_` to comply with docker container naming, and append `_runtime`
CONTAINER_NAME="$(tr ':' '_' <<< "$IMG_NAME")_runtime"
ROCKER_ARGS="${ROCKER_ARGS} --name $CONTAINER_NAME"
echo ${ROCKER_ARGS}
echo "Using image <$IMG_NAME> to start container <$CONTAINER_NAME> (no NVIDIA)"
echo "Reminder: launch the world with nvidia_offload:=false"

rocker ${ROCKER_ARGS} $IMG_NAME
