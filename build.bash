#!/usr/bin/env bash

#
# Build the mvsim Docker image (Gazebo Harmonic + prebuilt PX4 & ArduSub).
#
# Usage: ./build.bash [dockerfile_dir] [tag]
#   dockerfile_dir  directory containing the Dockerfile (default: mvsim)
#   tag             image tag (default: humble)
# Tags the build as both mvsim:<tag> and mvsim:<timestamp>.
#
# SPDX-License-Identifier: Apache-2.0
# Adapted from OSRF dockwater (Apache-2.0), Copyright (C) 2018 Open Source
# Robotics Foundation; modified for multivehicle_sim by MonkeScripts.
image_name=mvsim
dockerfile_dir=${1:-mvsim}
distro=${2:-humble}

if [ ! -f "${dockerfile_dir}"/Dockerfile ]
then
    echo "Err: Directory '${dockerfile_dir}' does not contain a Dockerfile to build."
    exit 1
fi

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t $image_plus_tag -f "${dockerfile_dir}"/Dockerfile "${dockerfile_dir}" && \
docker tag $image_plus_tag $image_name:$distro && \
echo "Built $image_plus_tag and tagged as $image_name:$distro"
echo "To run:"
echo "./run.bash $image_name:$distro"
