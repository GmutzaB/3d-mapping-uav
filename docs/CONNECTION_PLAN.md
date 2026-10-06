# Connection and communication plan

Planning baseline: September 28, 2026. User's deadline “05/27” is interpreted as
May 2027; exact demonstration day remains unspecified. This is a ground-up build.

## Responsibilities and physical links

| Link | Proposed connection | Information / responsibility |
|---|---|---|
| Battery → distribution → ESC | Power wiring selected from actual component ratings | Motor energy; independent of software |
| Distribution → regulated rails | Separately sized regulators for Pi and selected sensor | Verify voltage, peak current and brownout behavior before wiring |
| H743 → 55A ESC stack → motors | Manufacturer harness and supported motor protocol | H743 owns stabilization and motor commands |
| Candidate L2 → Pi | Ethernet/UDP preferred initial data link | Point clouds and sensor IMU; sensor needs its own verified power connection |
| Pi ↔ H743 | USB serial for initial bench work; secured UART for aircraft if supported | MAVLink 2 telemetry, external odometry, navigation commands |
| RC receiver → H743 | Receiver-specific compatible interface | Pilot control and mode selection independent of Pi |
| Ground computer ↔ Pi / H743 | Bench network / USB first; flight telemetry link unresolved | Logs, setup, mission status and map review |

This is a functional connection plan, not a pinout. “STM H743” identifies the MCU
family, not the board revision or firmware target. Confirm the stack manufacturer,
board photographs, revision and pinout before choosing firmware or UART pads.
UART TX connects to peer RX and vice versa, with a signal ground and verified logic
levels. Do not infer connector pin order or route raw battery voltage to logic rails.
USB power/backfeed behavior also needs the actual board schematic.

The [Unitree SDK](https://github.com/unitreerobotics/unilidar_sdk2) supports Ethernet
and serial; its default Ethernet example configures the host as 192.168.1.2.
Use a dedicated interface/subnet after checking the actual sensor configuration.
L2 remains a candidate; keep acquisition separate from mapping so it can be replaced.

## Software contracts

LiDAR driver → timestamped point clouds + IMU → LiDAR-inertial odometry → local map
and obstacle representation → planner → mission supervisor → MAVLink adapter → H743.
Telemetry flows back to the supervisor. Record raw inputs and processed outputs.
A point-cloud viewer alone is not collision avoidance: the planner needs occupied,
free and unknown space, aircraft clearance, and a braking-distance allowance.

Proposed rates below are project targets, not measured capability:

| Stream | Contract | Initial target |
|---|---|---|
| Acquisition | Sensor timestamp, receive timestamp, frame, sequence, units | Native sensor rates |
| External odometry | Pose, velocity, uncertainty, reset counter, quality | 20–30 Hz if measured pipeline supports it |
| Guided velocity | Three local velocity axes, expiry, mission state | 10 Hz |
| Health | Last valid timestamps, estimator validity, link state, overload | 10 Hz |
| Logging | Config/version metadata and timestamped events | Every state transition; sampled telemetry |

[ArduPilot external navigation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
prefers ODOMETRY and requires at least 4 Hz. Follow its supported LOCAL_FRD/BODY_FRD
frame IDs; do not label a north-aligned vector with an arbitrary local heading.
Verify EKF origin, external-navigation source selection, uncertainty and estimator
reset handling in SITL before applying configuration to a board.

For the first navigation adapter use velocity-only SET_POSITION_TARGET_LOCAL_NED
with all three velocity axes supplied and the correct type mask. Confirm acceptance
and actual motion in SITL. [Guided command documentation](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
describes refreshing velocity commands at least every second and stopping after
three seconds without updates; test behavior against the exact firmware/configuration.
This command timeout does not solve localization loss or guarantee safe landing.

## Coordinate and time rules

The offline dynamics model uses world NED (north, east, down), body FRD (forward,
right, down), SI units and scalar-first quaternions rotating body into world.
A ROS-style ENU world vector maps to NED as [y,x,-z]; body FLU maps to FRD as [x,-y,-z].
Orientation conversion requires BOTH basis changes, not quaternion component swaps:
R_NED_FRD = C_NED_ENU R_ENU_FLU C_FLU_FRD. Covariances must transform too.
For locally initialized SLAM, first establish the heading/origin transform; its axes
are not automatically geographical east and north. Keep continuous odometry separate
from maps that jump during loop closure. Apply measured LiDAR-to-body extrinsics.

Use monotonic receipt time for watchdog age. Preserve sensor capture time separately;
record the conversion between sensor, Pi and FC clocks, plus processing delay.
ArduPilot's ODOMETRY timestamp does not require FC clock synchronization, but pipeline
latency and sensor/IMU synchronization still matter for estimation quality.

## Failure behavior to implement and prove

No arming on startup. Mission commands require explicit operator enable, valid
localization, a healthy FC link and a recent collision-checked trajectory.
Reject nonfinite commands, wrong frames and expired data. If the planner becomes
stale while localization remains valid, request a tested stop/hold transition.
If localization fails, position hold may also fail: use the separately tested FC
fallback and pilot takeover procedure. Do not automatically resume after recovery.
Configure and test FC-side RC, battery and companion-link behavior independently.

## Next integration evidence

1. Identify board/firmware; export parameters; establish read-only telemetry.
2. Run selected sensor on the bench and replay a recorded hand-carried dataset.
3. Select OS, ROS and mapping versions together; pin proven revisions.
4. Exercise frame directions, timestamps and estimator resets in SITL.
5. Add supervisor and adapter, then inject stale data and link loss in SITL.
6. Measure complete pipeline latency and power on the selected Pi before flight.

No physical connection, firmware installation or SITL integration has been performed
by this software change.
