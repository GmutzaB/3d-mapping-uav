# System architecture

## Objective and current state

Build a quadcopter that autonomously navigates and produces 3D maps indoors and outdoors.
Whether missions use assigned destinations or self-directed exploration remains undecided.
Seamless indoor/outdoor transitions are a separate requirement to evaluate.

The current implementation includes the Python airflow/thrust demonstration and the offline rigid-body model described in [FLIGHT_MODEL.md](FLIGHT_MODEL.md).
See [CONNECTION_PLAN.md](CONNECTION_PLAN.md) for proposed interfaces and latest hardware uncertainty.
Offline frame conversions, command gating and MAVLink encoding are now implemented
in [src/companion](../src/README.md), with synthetic replay verification.
The live autonomy pipeline below remains proposed and is not flight validated.

## Proposed responsibilities

| Component | Responsibility | Project work |
|---|---|---|
| DAKEFPV H743 running ArduCopter | Stabilization, motor outputs, state estimation and aircraft failsafes | Identify board revision, configure, tune and export parameters |
| Unitree L2 connected to Raspberry Pi 5 | Point cloud and IMU acquisition | Driver setup, timing checks and mounting calibration |
| Mapping package on the Pi | Estimate local motion and accumulate a map | Select compatible package/version and measure drift |
| Obstacle map and planner on the Pi | Represent free/occupied/unknown space and generate trajectories | Integrate inputs, aircraft clearance and trajectory limits |
| Custom mission supervisor on the Pi | Coordinate mission stages and respond to health failures | Implement states, completion criteria and recovery behavior |
| MAVLink integration | Exchange telemetry, external position and navigation commands | Coordinate-frame conversion, timestamps, quality and timeout handling |
| Ground computer | Setup, visualization, recording review and map inspection | QGroundControl and CloudCompare workflow |

Data path: L2 -> driver -> position estimation and mapping -> obstacle map -> planner
-> mission/control integration -> ArduCopter -> ESCs and motors.
Telemetry and health information return to the mission supervisor and ground computer.

## Interfaces to resolve before implementation

- Choose the Pi OS, ROS distribution and mapping implementation together. The vendor's
  Point-LIO example uses ROS 1 Noetic; a ROS 2 driver alone does not make it ROS 2 compatible.
- Define sensor, vehicle, odometry and map coordinate frames. Explicitly handle any
  ROS/ArduPilot frame differences at the interface boundary.
- Measure sensor time offsets, mounting translation/rotation and end-to-end latency.
- Choose one initial command interface and test it in ArduPilot SITL before hardware use.
- Define behavior for stale data, localization loss, planner failure and loss of communication.
- Separate a visually useful map from the occupancy representation used for collision checks.
- Decide whether global map correction/loop closure and GPS georeferencing are required.
- Benchmark the complete pipeline on the Pi; component-level support is not a performance guarantee.

## References

- [ArduPilot external position interface](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
- [ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2.html)
- [DAKEFPV board documentation](https://ardupilot.org/plane/docs/common-dakefpvh743pro.html)
- [Unitree SDK2](https://github.com/unitreerobotics/unilidar_sdk2)
- [Unitree Point-LIO](https://github.com/unitreerobotics/point_lio_unilidar)
