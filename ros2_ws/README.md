# ROS 2 workspace (Raspberry Pi 5)

Runs on the drone's Pi 5 with Ubuntu 24.04 (ROS 2 Jazzy) or Ubuntu 26.04 (ROS 2 Lyrical):

- **Unitree L2 driver** publishes the point cloud (`/unilidar/cloud`) and IMU (`/unilidar/imu`).
- **MAVROS** talks MAVLink to the ArduCopter flight controller and exposes it as ROS 2 topics (`/mavros/state`, `/mavros/local_position/pose`, `/mavros/imu/data`, ...).
- **`obstacle_sectors`** turns the point cloud into 72 distance sectors (5° each) around the aircraft and sends them through MAVROS to ArduPilot as `OBSTACLE_DISTANCE`. ArduPilot can use these for its built-in obstacle avoidance.

Nothing here arms the aircraft, changes flight mode, or sends movement commands.

```
src/
  unilidar_sdk2/      Unitree SDK 2.0.10 + ROS 2 driver, copied unchanged (BSD-3 license)
  uav_lidar_bridge/   our launch files, config, and obstacle sector node
pi/
  install.sh          one-time Pi setup + build
  60-unitree-lidar.yaml   static IP for the LiDAR Ethernet link
  uav-ros.service     optional: start everything at boot
```

## 1. Put Ubuntu on the Pi

These ROS 2 packages exist for Ubuntu, not Raspberry Pi OS. Use Ubuntu Server 24.04 LTS or 26.04 LTS, 64-bit. `pi/install.sh` installs Jazzy on 24.04 and Lyrical on 26.04.

1. Open **Raspberry Pi Imager** → Device: Raspberry Pi 5 → OS: *Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS or 26.04 LTS (64-bit)*.
2. In the settings (gear icon), set the username to `ubuntu`, a password, your Wi-Fi, and enable SSH.
3. Boot the Pi and SSH in: `ssh ubuntu@<pi-ip>`.

Back up anything you need from the current Raspberry Pi OS card first; flashing erases it.

## 2. Install and build

```sh
git clone https://github.com/GmutzaB/3d-mapping-uav.git
cd 3d-mapping-uav/ros2_ws
./pi/install.sh
```

This takes a while the first time. Log out and back in when it finishes. After pulling new code later, `colcon build --symlink-install` in `ros2_ws` is enough.

## 3. Wiring

**LiDAR:** L2 Ethernet → Pi Ethernet port, plus the L2's own power supply. Give the Pi the address the L2 expects:

```sh
sudo cp pi/60-unitree-lidar.yaml /etc/netplan/
sudo chmod 600 /etc/netplan/60-unitree-lidar.yaml
sudo netplan apply
ping 192.168.1.62
```

If your Wi-Fi network also uses `192.168.1.x`, the two will clash. Change the L2's address with Unitree's tool, or use a different Wi-Fi network.

**Flight controller**, either:

- **USB** (easiest for bench testing): H743 USB → Pi USB. Default `fcu_url` is `serial:///dev/ttyACM0:115200`.
- **UART** (for flight): H743 TELEM TX/RX/GND → Pi GPIO 15 (RX) / 14 (TX) / GND. Add `dtparam=uart0=on` to `/boot/firmware/config.txt` and reboot. In ArduPilot set that port's `SERIALn_PROTOCOL = 2` and `SERIALn_BAUD = 921`. Launch with `fcu_url:=serial:///dev/ttyAMA0:921600`.

## 4. Run

```sh
ros2 launch uav_lidar_bridge uav.launch.py
```

Useful options:

```sh
ros2 launch uav_lidar_bridge uav.launch.py fcu_url:=serial:///dev/ttyAMA0:921600
ros2 launch uav_lidar_bridge uav.launch.py mavros:=false      # LiDAR only, no flight controller
ros2 launch uav_lidar_bridge uav.launch.py lidar:=false obstacles:=false   # MAVROS only
ros2 launch uav_lidar_bridge uav.launch.py gcs_url:=udp-b://@  # also relay to Mission Planner on the network
```

Check that data is flowing (in a second SSH session):

```sh
ros2 topic hz /unilidar/cloud
ros2 topic echo /mavros/state --once          # connected: true
ros2 topic echo /mavros/obstacle/send --once  # 72 ranges, NaN = nothing seen
```

To start everything at boot, see the comments at the top of `pi/uav-ros.service`.

## 5. Before trusting obstacle avoidance

1. Measure where the L2 sits on the frame and enter it in `src/uav_lidar_bridge/config/obstacle_sectors.yaml` (`mount_*`). If it's mounted upside down, set `mount_roll_deg: 180`. Edits take effect on the next launch.
2. On the bench, hold a board in front, right, behind, and left of the aircraft. In Mission Planner, open the proximity view (Ctrl-F → Proximity) and confirm each one shows up in the right direction.
3. ArduPilot only uses the data after you set `PRX1_TYPE = 2` (MAVLink) and enable avoidance (`AVOID_ENABLE`, `AVOID_MARGIN`). See [ArduPilot's object avoidance docs](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html). Test in SITL and with props off before flying.

## Developing on a laptop

The sector math has plain unit tests that run without ROS:

```sh
cd ros2_ws/src/uav_lidar_bridge
python -m unittest discover -s test -v
```

To see the point cloud in RViz, use a Linux machine (or VM) with ROS 2 Jazzy on the same network as the Pi, with the same `ROS_DOMAIN_ID`, and run `rviz2`. The Unitree RViz config is in `src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/rviz/view.rviz`.
