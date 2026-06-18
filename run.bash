#!/usr/bin/env bash

#
# Launch the mvsim container with rocker (NVIDIA GPU + X11 + joystick + host net).
#
# Usage: ./run.bash [-c|-s|-t|-x|-h] <image>
#   <image>   image to run, e.g. mvsim:humble
# On hosts without an NVIDIA GPU, use ./run_no_nvidia.bash instead.
# Requires: docker, an X server, rocker (and the NVIDIA Container Toolkit).
#
# SPDX-License-Identifier: Apache-2.0
# Adapted from OSRF dockwater (Apache-2.0), Copyright (C) 2018 Open Source
# Robotics Foundation; modified for multivehicle_sim by MonkeScripts.
############################################################
# Help                                                     #
############################################################
Help()
{
   echo "Launch the mvsim container with rocker."
   echo
   echo "Syntax: ./run.bash [-c|-s|-t|-x|-h] <image>"
   echo "options:"
   echo "c     Add CUDA support."
   echo "s     Headless cloudsim variant (noVNC/TurboVNC)."
   echo "t     Minimal variant for CI."
   echo "x     Base variant with joystick + user override."
   echo "h     Print this help message and exit."
   echo
}


JOY=/dev/input/js0
CUDA=""
HOST_RDP_PORT=3389
ROCKER_ARGS="--devices /dev/dri $JOY --dev-helpers --nvidia --x11 --git --volume "$HOME":/root/HOST  --network=host"

while getopts ":cstxh" option; do
  case $option in
    c) # enable cuda library support 
      CUDA="--cuda ";;
    s) # Build cloudsim image
      ROCKER_ARGS="--nvidia --novnc --turbovnc --user --user-override-name=developer";;
    t) # Build test image for Continuous Integration 
      echo "Building CI image"
      ROCKER_ARGS="--dev-helpers --nvidia";;
    x) # Build VRX Competition base image
      echo "Building VRX Competition server base image"
      ROCKER_ARGS="--dev-helpers --devices $JOY --nvidia --x11 --user --user-override-name=developer";;
    h) # print this help message and exit
      Help
      exit;; 
  esac
done

IMG_NAME=${@:$OPTIND:1}

# Replace `:` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr ':' '_' <<< "$IMG_NAME")_runtime"
ROCKER_ARGS="${ROCKER_ARGS} --name $CONTAINER_NAME"
echo ${ROCKER_ARGS}
echo "Using image <$IMG_NAME> to start container <$CONTAINER_NAME>"

rocker ${CUDA} ${ROCKER_ARGS} $IMG_NAME 
