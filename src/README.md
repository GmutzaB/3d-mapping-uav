# Companion software

The `companion` package provides offline integration components:

- `frames.py`: explicit ENU/FLU to NED/FRD conversions.
- `supervisor.py`: operator enable, freshness checks, bounded velocity commands,
  estimator reset handling and latched hold/fault decisions.
- `mavlink.py`: MAVLink 2 velocity and external-odometry packet encoding.
- `replay.py`: synthetic timeline combining supervisor decisions and packet encoding.

From the repository root:

```sh
.venv/bin/python -m pip install -r requirements-companion.txt
.venv/bin/python -m src.companion.replay
.venv/bin/python -m unittest -v
```

Replay writes `outputs/companion_replay/events.jsonl` and `commands.mavlink`.
It opens no network or serial connection. The test suite decodes the binary packets
and verifies that planner recovery and localization recovery do not automatically
resume motion. The operator enable events in this synthetic scenario are scripted.

`hold_requested` emits a zero-velocity intent while localization is healthy. It is
not confirmation that an aircraft has stopped. `fault_latched` emits no command:
actual fallback behavior requires separately configured and tested flight-controller
failsafes. Disabling command authority also stops emission; it does not disarm or
land the aircraft. These offline components never send arming commands. The separate `mission.py`
runner below sends normal arming commands only to its owned simulator.

The caller must provide trustworthy timestamp/frame/health metadata. In particular,
`collision_checked=True` is an upstream contract, not an implemented obstacle check.
The supervisor alone does not authenticate enable events or verify estimator quality.

MAVLink tests are skipped if pymavlink is absent; install the companion requirements
for complete wire verification. Packet validity is not SITL or aircraft validation.
Still needed: live aircraft/SITL validation, clock alignment, sensor driver, mapping,
collision checking/planning, communication scheduling, FC fallback integration, and
ArduPilot SITL acceptance tests. Record upstream revisions in `setup/dependencies.json`.

## Receive-only telemetry

`telemetry.py` decodes ArduCopter quadrotor heartbeats and LOCAL_POSITION_NED from
explicit system/component IDs. `monitor.py` receives UDP datagrams and records raw
bytes with monotonic arrival times plus periodic decoded snapshots:

```sh
.venv/bin/python -m src.companion.monitor --port 14560 --seconds 30 --output outputs/bench-001
```

Configure an existing SITL/MAVLink router to send to `127.0.0.1:14560` on the same
computer. The monitor does not establish that router, request stream rates, send
heartbeats, change mode, arm, or write parameters. It exits after the specified
interval even if no telemetry arrives. Use a new output directory for each recording.
For a remote router, explicitly choose the local bind address and network setup.

This tool assumes one trusted telemetry source/router. System IDs filter messages;
they do not authenticate a sender. UDP receipt freshness is not sensor capture age.
A backwards FC position timestamp latches a discontinuity (reboot and reordered
packets are not distinguished); investigate and start a new receiver session.
Counter wrap is handled. Duplicate position timestamps do not refresh position age.
A FC local-position packet never automatically establishes external-localization
validity. The supervisor still requires independently validated/time-aligned odometry.

Heartbeat mode interpretation follows [ArduPilot flight modes](https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html).
Payload units, flags and sentinels follow the [MAVLink message definitions](https://mavlink.io/en/messages/common.html).
Only heartbeat and local position are ingested so far; battery, EKF diagnostics,
clock alignment, firmware identity and command acknowledgment handling remain work.

## Owned SITL mission

`mission.py` adds acknowledged commands, heartbeat scheduling, telemetry freshness
checks, bounded position requests, settling criteria and landing confirmation.
Run it through `sitl_mission.py` at the repository root; this launcher owns the
simulator lifecycle and records a fresh experiment. See [SITL setup](../setup/SITL.md).
It uses the simulator's native estimator, not the external-localization supervisor.
