#!/usr/bin/env bash
# One-time setup on a Raspberry Pi 5 running 64-bit Ubuntu 24.04 or 26.04.
# 24.04 installs ROS 2 Jazzy; 26.04 installs ROS 2 Lyrical. Then MAVROS and
# this workspace are built. Safe to run again after pulling new code.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. /etc/os-release
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Expected aarch64, found $(uname -m)." >&2
  exit 1
fi
case "${VERSION_ID}" in
  24.04) ROS_DISTRO=jazzy ;;
  26.04) ROS_DISTRO=lyrical ;;
  *)
    echo "Expected Ubuntu 24.04 (ROS 2 Jazzy) or 26.04 (ROS 2 Lyrical), found ${PRETTY_NAME}." >&2
    exit 1
    ;;
esac

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
  "ros-${ROS_DISTRO}-ros-base" ros-dev-tools \
  "ros-${ROS_DISTRO}-pcl-conversions" libpcl-dev \
  "ros-${ROS_DISTRO}-tf2-ros" "ros-${ROS_DISTRO}-sensor-msgs-py" \
  "ros-${ROS_DISTRO}-mavros" "ros-${ROS_DISTRO}-mavros-extras" \
  python3-numpy

# MAVROS won't start without the GeographicLib datasets.
sudo "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"

# Serial access to the flight controller without sudo (takes effect after re-login).
sudo usermod -aG dialout "$USER"

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "$WS"
colcon build --symlink-install

if ! grep -qF "$WS/install/setup.bash" ~/.bashrc; then
  {
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "source $WS/install/setup.bash"
  } >> ~/.bashrc
fi

echo
echo "Build finished. Log out and back in once, then run:"
echo "  ros2 launch uav_lidar_bridge uav.launch.py"
