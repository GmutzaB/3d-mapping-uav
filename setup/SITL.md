# Local ArduCopter mission experiment

The mission runner starts its own simulator, takes off to 3 m, holds position,
flies 3 m north and 2 m east, holds again, lands and waits for automatic disarming.
The default quad uses ArduPilot's native GPS/EKF and controller. It is separate
from our uncalibrated Python rigid-body model and is not a model of our hardware.

## Build on this Mac

Xcode Command Line Tools and the project virtual environment are required. From
the repository root (the folder containing `sitl_mission.py`):

```sh
.venv/bin/python -m pip install -r requirements-companion.txt -r setup/requirements-sitl.txt
git clone --depth 1 --branch Copter-4.6.3 https://github.com/ArduPilot/ardupilot.git external/ardupilot
git -C external/ardupilot submodule update --init --depth 1 modules/waf modules/mavlink modules/gtest modules/DroneCAN/DSDL modules/DroneCAN/pydronecan modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard
cd external/ardupilot
../../.venv/bin/python waf configure --board sitl --no-submodule-update
../../.venv/bin/python waf copter -j 4
cd ../..
```

For a OneDrive checkout, compilation can be much slower because it touches many
small files. The same source can be built in a local temporary directory instead.
Copy it with `rsync -a --exclude build --exclude .lock-waf_darwin_build`, configure
and build there using the project's absolute `.venv/bin/python` path, then copy
`build/sitl/bin/arducopter` back to the original checkout at the same relative path.
Keep the source revision and build log with the experiment.

Expected commit: `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`. If the checkout already
exists, do not clone over it; check its revision first. Keep third-party sources
and build products in the ignored `external/` directory. This installs no FC
firmware, ROS, Docker, or GUI ground station.

Official background: [Mac build setup](https://ardupilot.org/dev/docs/building-setup-mac.html),
[ArduPilot build guide](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/BUILD.md),
[Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html).

## Run

In VS Code select **SITL mission — takeoff, hover, waypoint, land** and press F5.
Or run:

```sh
.venv/bin/python sitl_mission.py
```

Use `--north`, `--east`, and `--altitude` to change the bounded mission. Defaults
are 3 m north, 2 m east, 3 m height. `--no-visuals` skips the chart and GIF.
`--output outputs/sitl/my-run` requires a new directory. Default output folders
are uniquely timestamped. Reopen `mission.gif` to replay; it plays once.

The runner binds a new loopback TCP listener and starts the pinned simulator with
its SERIAL0 connected to that listener. It cannot attach to serial hardware or
an arbitrary network vehicle. Socket permission may be needed in a sandbox.
Each run gets fresh simulated parameter storage, a copy of the upstream quad
parameters, and a GCS-link failsafe setting in `defaults.parm`. It terminates
only its own simulator process on exit.

## Completion criteria and evidence

Each TAKEOFF, HOVER and WAYPOINT phase must remain within 0.35 m of its target
and below 0.20 m/s continuously for 3 seconds. Position targets are in LOCAL_NED,
relative to the captured start position; down is positive. The mission sets the
requested horizontal navigation speed to 0.75 m/s. Landing succeeds only after
fresh telemetry reports ON_GROUND and disarmed. No force-arming/disarming is used.

The mission stops sending position targets if telemetry is stale, the FC clock
resets, GUIDED mode is lost, unexpected disarming occurs, or the aircraft exceeds
15 m horizontally or 8 m above the starting point. Each phase has a timeout.
On failure it requests LAND only if still armed in GUIDED with a recent heartbeat;
otherwise it records failure and stops its simulator. No failed flight is marked
as successful merely because a landing command was accepted.

Every run records:

- `summary.json`: pass/fail, settings, targets, settling measurements and error.
- `environment.json`: pinned source revision, binary SHA-256, host and launch command.
- `defaults.parm`, `eeprom.bin`, `logs/`: simulator parameters/state and native flight logs.
- `events.jsonl`: phases, command acknowledgments, status messages and failures.
- `wire.jsonl`: timestamped incoming/outgoing MAVLink bytes.
- `telemetry.csv`: measured position, velocity and attitude with mission phase.
- `sitl.log`: simulator stdout/stderr.
- `mission.png`, `mission.gif`: recorded flight charts and accelerated replay.

Unit tests use a fake transport/vehicle and are labeled as such. Only a completed
run of the actual ArduCopter binary demonstrates SITL integration. This first
mission does not yet validate link-loss failsafes, external odometry, obstacles,
mapping, the Pi, or the H743 aircraft. Those require separate experiments.

Run regressions with `.venv/bin/python -m unittest -v test_mission`, or the full
suite with `.venv/bin/python -m unittest -v` (the UDP test requires local sockets).
