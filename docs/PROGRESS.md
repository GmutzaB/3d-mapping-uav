# Resume checkpoint — September 28, 2026

User is shutting down the computer. Changes are saved locally, not committed.

## Verified

- Offline rigid-body dynamics and original airflow tests: 11 passed.
- Combined suite: 25 tests passed with no skips, including MAVLink 2 encode/decode
  and end-to-end synthetic replay checks. `git diff --check` passed.
- Owned hardware: H743/55A stack and Raspberry Pi 5. Exact FC revision and Pi RAM
  still need confirmation. LiDAR (possibly Unitree L2) and printed frame material
  remain tentative. Deadline interpreted as May 2027.

## New software

- `flight_dynamics.py`: offline 6-DOF model with analytical tests.
- `src/companion/frames.py`: vector, attitude matrix and vector covariance conversion.
- `src/companion/supervisor.py`: command gating, freshness, reset handling, fault latching.
- `src/companion/mavlink.py`: MAVLink 2 odometry and velocity packet encoding.
- `src/companion/replay.py`: synthetic replay verified; outputs saved under outputs/companion_replay/.
- `requirements-companion.txt`: optional pymavlink dependency.

## Next actions

1. Validate the new receive-only telemetry monitor against actual ArduPilot SITL.
   SITL and Docker were not found on PATH. No FC or sensor has been connected.
2. Review supervisor behavior against actual FC failsafes before connecting commands.
   This is an offline command gate, not a deployed flight safety system or collision checker.
3. Continue calibration, mapping/replay, navigation and AI experiments per roadmap.
4. Keep source documentation aligned; src/README.md now describes implementation and limits.

The full goal remains active and incomplete. Packet tests do not prove autopilot
acceptance or aircraft behavior. Do not treat synthetic replay as a SITL flight.

## Telemetry increment

Added explicit-source heartbeat and local-position decoding, corruption rejection,
duplicate detection, boot-counter wrap handling and latched timestamp discontinuity.
Six telemetry unit tests passed. A seventh exercises a real receive-only localhost
UDP socket and raw capture replay; it requires socket access outside the sandbox.
The sandbox denied bind(), so an escalated full-suite verification was requested.
No physical hardware or actual SITL telemetry was used.

## September 30: feedback-control milestone verified

Implemented flight_control.py, control_visuals.py and test_flight_control.py.
The 22-second experiment includes 10-second hover, a 10-degree roll command, return
to level, and a 0.15-second external torque disturbance. Actual metrics and arrays
are saved under outputs/control; comparison.gif contains 111 decoded frames.
Representative rendered frames were inspected for readability and agreement with data.

34 applicable controller, dynamics, airflow and companion regression tests passed.
The unrelated UDP telemetry test was not rerun for this change. diff --check passed.
See FEEDBACK_CONTROL.md for acceptance criteria, measured results, limitations and
the concrete ArduPilot SITL integration plan. This scoped control milestone is done;
the overall aircraft project and actual SITL/hardware validation remain unfinished.
