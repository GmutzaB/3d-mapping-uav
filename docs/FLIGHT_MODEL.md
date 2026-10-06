# Aircraft dynamics baseline

`flight_dynamics.py` adds an offline six-degree-of-freedom rigid-body model alongside
the existing airflow demonstration. It does not replace ArduPilot or run its controller.

From the repository directory:

```sh
.venv/bin/python flight_dynamics.py --scenario hover
.venv/bin/python flight_dynamics.py --scenario freefall
.venv/bin/python flight_dynamics.py --scenario roll --seconds 0.5
.venv/bin/python -m unittest -v
```

CSV trajectories and JSON metadata go to `outputs/flight/`. Positive world z is down.
Hover starts with motors already at equilibrium. Freefall has no ground collision;
roll is an open-loop differential-thrust experiment, not a controlled maneuver.

State contains position, velocity, body-to-world quaternion, body angular velocity
and four motor thrusts. Forces and moments follow:

- m v_dot = R [0,0,-sum(T)] + m [0,0,g] - drag*v
- I omega_dot = sum(r cross F) + yaw_torque - omega cross (I omega)
- q_dot = 0.5 q multiplied by [0,omega]
- T_dot = (T_max*u² - T)/tau

RK4 integrates all states; attitude is normalized after each step. The CLI limits
step size to half the motor time constant. Faster angular motion may need smaller
steps; this limit alone is not a universal accuracy guarantee.
Internal rotor order is front-right, rear-right, rear-left, front-left. It is NOT an
ArduPilot motor numbering assignment. Yaw signs specify reaction torque on the body.

## Calibration and limitations

Mass is the provisional 2.2 kg target. Arm length, diagonal inertia, thrust ceiling,
yaw torque ratio and motor time constant are placeholders. The 12 N thrust ceiling
is not derived from the existing airflow curve and is not evidence of motor capability.
Use `Parameters(...)` from Python to run sensitivity studies; these are not flight settings.

Measure total assembled mass, center of gravity, motor positions and inertia (CAD
mass distribution followed by an experimental check). Obtain a thrust/current/RPM
curve for the actual motor/propeller/battery combination and motor step response.
The proposed printed polycarbonate/nylon frame also needs structural and vibration
assessment; rigid-body dynamics cannot predict flex, resonance or printed joint failure.

Missing: battery sag and endurance, aerodynamic rotor interactions, ground effect,
wind field, contact, sensor noise/delay, flexible structure, controller and SITL coupling.
Analytical tests cover freefall, equilibrium, motor response, force/moment directions,
and torque-free single-axis rotation. These verify equations and implementation, not
real-aircraft accuracy. Next: calibrate parameters, add logged disturbances/sensors,
and compare trajectories against ArduPilot SITL and then measured data.

## Shareable visual output

Running `flight_dynamics.py` now also creates a PNG chart and animated GIF next to
the CSV/JSON files. Default hover is stationary because thrust balances weight.
For visible motion, choose **Flight model — shareable roll animation** in VS Code's
Run and Debug menu, or run:

```sh
.venv/bin/python flight_dynamics.py --scenario roll --seconds 0.5
```

The GIF displays aircraft pose, trajectory, relative height, roll/pitch and total
thrust versus weight, all from the saved simulation samples. Animation is slowed
for viewing, with actual simulation time printed onscreen. World vertical position
is displayed as height (up positive); CSV data retains down-positive NED coordinates.
The example is an open-loop roll response, not an autonomous or stabilized flight.
Use `--no-visuals` to skip rendering. No GUI window is required: open the output PNG
or GIF and share the file. The orange arm joins rear-right and front-left motors.

A separate [feedback-control experiment](FEEDBACK_CONTROL.md) now exercises this same
model with attitude and height feedback and an external body-torque disturbance.
The original flight_dynamics.py scenarios remain open-loop.

## Launching hover and roll

The Run and Debug menus in CODE SPACE, 3D MAPPING UAV, and the repository folder
all include explicit hover and roll animation entries. Hover runs for 2 seconds;
roll runs for 0.5 seconds. The CLI now uses those same scenario-specific defaults.
An explicit `--seconds` still overrides the duration. Sustained differential thrust
keeps accelerating the roll, so longer open-loop runs can tumble; use
`flight_control.py` for the stabilized 10-degree roll and return demonstration.
A stationary hover animation is expected: the motors start at equilibrium.

GIF exports play once and hold the final frame marked END, avoiding a sudden
reset to the initial level pose. Reopen the GIF to replay it. Each frame includes
the scenario name and playback status.
