# Simulation feedback-control milestone

Implemented September 30, 2026. This Python controller is educational simulation
code. It is not ArduPilot and must not be copied into aircraft firmware as flight tuning.

## Run and share

Open this repository in VS Code, select **Feedback control — comparison animation**
in Run and Debug, and press F5. Or run:

```sh
.venv/bin/python flight_control.py
.venv/bin/python -m unittest -v test_flight_control test_flight_dynamics test_simulation
```

Outputs in `outputs/control/`:

- `comparison.gif`: side-by-side animated roll views (east/up projection), with angle, motor-thrust and height graphs.
- `comparison.png`: final replay frame; `comparison_charts.png`: static charts.
- `controlled.csv`, `uncontrolled.csv`: full state, requested roll, angles, throttle,
  applied disturbance and saturation flags at each sample.
- Matching NPZ arrays and `metrics.json`: reusable data and measured performance.

`--no-visuals` runs calculations only. `control_visuals.py` regenerates the visual
from existing NPZ files. The 22 s experiment is shown at approximately 2x speed.
Aircraft views are centered, not world-position views; displacement is shown numerically.
Angle and height axes have different scales. Roll traces are unwrapped for continuous display.
Motor colors use internal FR, RR, RL, FL order, not FC output numbering; coincident
traces overlap when motors produce equal thrust.

## Experiment and success criteria

Both cases start in an ideal, level hover with motors already producing equilibrium
thrust. At 10 s, request 10 degrees roll; at 14 s request level. At 17 s, apply
0.12 N m about body roll for 0.15 s; stop at 22 s. Event boundaries are explicitly
included in the integration grid. The fixed-throttle case ignores angle requests;
it receives the identical external torque pulse and has no feedback correction.
It is a baseline, not a second controller attempting the requested maneuver.

Tests require 10 s level hover; commanded-roll and return settling within 2 s;
overshoot below 1 degree; pulse response above 0.5 degree and below 10 degrees;
recovery within 2 s after pulse end; maximum height excursion below 5 cm; final
height error below 5 mm; bounded throttle and no nominal saturation. Tests also
check timestep refinement and show that the uncontrolled baseline fails to recover.

Settling means the first sample after which angle error stays within +/-0.5 degree
through the remainder of that phase, with at least 0.5 s of observation remaining.
Missing settling is JSON null. Roll overshoot is the excess above 10 degrees;
return overshoot is the excursion below zero after the return request. Recovery
settling time is measured from the end of the disturbance, not its onset.

Nominal observed results (dt=0.005 s):

| Measurement | Feedback result |
|---|---:|
| Initial 10 s hover angle/height error | 0 in the ideal equilibrium case |
| 10° command settling | 0.655 s |
| Roll overshoot | 0° |
| Return-to-level settling | 0.655 s |
| Return overshoot | 0° |
| Disturbance peak roll | 3.290° |
| Post-pulse recovery settling | 0.460 s |
| Maximum height excursion | 0.001328 m |
| Final height change | about -0.000000029 m |
| Final horizontal displacement | about 68.0 m |
| Saturated samples | 0 |

The fixed-throttle baseline does not track the roll request and does not recover
from the pulse. Its reported return settling of zero means it was already level
before the disturbance; it is not evidence of command tracking. Without ground
contact it mathematically falls about 82.7 m by the experiment end. This extrapolated
fall is not a prediction of a real aircraft flight or impact.

## Controller and assumptions

Geometric attitude error drives proportional attitude and angular-rate damping.
The controller uses diagonal inertia and a rigid-body cross-coupling compensation
term. Height uses proportional position and velocity feedback with gravity and
thrust-tilt compensation. A four-motor allocation matrix converts collective thrust
and body torques to rotor thrusts, clipped to [0, maximum thrust], then to throttle
using the existing quadratic thrust relation. The original RK4 physics and motor
lag remain active; external body torque is included in every RK4 stage.

All gains and unmeasured aircraft parameters are provisional and saved with results.
This nominal simulation assumes exact state feedback and matched model/controller
parameters. There is no sensor noise, delay, parameter uncertainty, wind, battery
sag, structural flex or ground contact. The tilt compensation denominator is limited
to 0.5; the controller is intended for small attitudes, not inverted-flight recovery.
Rotor clipping is basic, not an optimal/desaturating flight-control mixer.

There is no horizontal position or velocity feedback. Holding 10° tilt produces
horizontal acceleration. Returning level removes the acceleration but does not
brake the accumulated velocity in this zero-drag model, explaining the large final
displacement. Exact hover is a balance check; the pulse experiment checks recovery.

## Concrete next step: ArduPilot SITL

The present milestone is verified independently of SITL. The following is a plan,
not evidence that SITL has been installed or run:

1. Choose a supported development host (Ubuntu Linux/VM is a documented route),
   clone ArduPilot and its submodules, and follow the official build prerequisites.
   Pin the chosen stable release/commit and record the toolchain. Review compatibility
   separately for the actual H743 board revision; an MCU name alone is insufficient.
   [Official SITL setup](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
2. Start a fresh Copter quad simulation using that clone's `Tools/autotest/sim_vehicle.py`
   with `-v ArduCopter -f quad`. Verify launch options with `--help`; keep its parameter
   files in a dedicated experiment directory. Record build revision and parameters.
   [SITL usage](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html).
3. In MAVProxy, route an output using `output add 127.0.0.1:14560`; run our receive-only
   `python -m src.companion.monitor --port 14560 --output outputs/sitl-001` on the same
   host. A VM and Mac have different loopbacks: use the VM host interface or run the
   companion inside the VM. Confirm system/component IDs, heartbeat, mode and local
   position in captured packets. Configure missing telemetry streams deliberately.
4. Build the missing bidirectional session around our packet encoder: heartbeat
   scheduling, target discovery/validation, time handling, mode/arming acknowledgments,
   command expiry and shutdown behavior. Keep this endpoint explicitly tied to SITL.
   The current recorder only receives; the encoder does not send commands by itself.
5. First use SITL's native position estimator. Test bounded Guided velocity requests
   and compare requested versus actual velocity, height and attitude using autopilot
   logs. ArduPilot owns its simulated motor control; this Python controller is not
   inserted into that loop. Test stop, mode change and companion-link loss behavior.
6. Add the external odometry stream with a verified frame/origin transform, measured
   latency, uncertainty, quality and reset counters. Validate EKF acceptance before
   using it for navigation; inject delays, resets and dropouts. Use ArduPilot's
   documented ODOMETRY interface and at least its required 4 Hz update rate.
   [External navigation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html).
7. Archive SITL logs, parameter files and pass/fail criteria before moving to bench
   hardware. A SITL build exercises ArduPilot software but does not validate H743 I/O,
   ESC wiring, sensor mounting, aircraft tuning or the real airframe dynamics.
