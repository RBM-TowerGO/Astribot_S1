#!/usr/bin/env bash
# Explicit administrator step. Does not start ROS nodes or change driver configuration.
set -euo pipefail
if [ "$(id -u)" != 0 ]; then
  echo 'Run this reviewed script with sudo in an interactive S1 terminal.' >&2
  exit 2
fi
apt-get update
apt-get -o DPkg::Lock::Timeout=120 install -y --no-install-recommends --no-upgrade \
  python3-colcon-common-extensions python3-pytest python3-numpy python3-scipy python3-yaml python3-pil \
  libpcl-dev libeigen3-dev libboost-all-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rosidl-default-generators \
  ros-humble-slam-toolbox ros-humble-nav2-map-server ros-humble-tf2-ros-py \
  ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-py
