#!/usr/bin/env bash
# One-time setup on a Raspberry Pi 5 running Ubuntu 24.04 (64-bit).
# Installs ROS 2 Jazzy, MAVROS and build tools, then builds this workspace.
# Safe to run again after pulling new code.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. /etc/os-release
if [[ "${VERSION_ID}" != "24.04" || "$(uname -m)" != "aarch64" ]]; then
  echo "Expected Ubuntu 24.04 on aarch64, found ${PRETTY_NAME} on $(uname -m)." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${UBUNTU_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-pcl-conversions libpcl-dev \
  ros-jazzy-tf2-ros ros-jazzy-sensor-msgs-py \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  python3-numpy

# MAVROS won't start without the GeographicLib datasets.
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Serial access to the flight controller without sudo (takes effect after re-login).
sudo usermod -aG dialout "$USER"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS"
colcon build --symlink-install

if ! grep -qF "$WS/install/setup.bash" ~/.bashrc; then
  {
    echo 'source /opt/ros/jazzy/setup.bash'
    echo "source $WS/install/setup.bash"
  } >> ~/.bashrc
fi

echo
echo "Build finished. Log out and back in once, then run:"
echo "  ros2 launch uav_lidar_bridge uav.launch.py"
