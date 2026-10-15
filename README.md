# 3D Mapping UAV

One repository for the aircraft hardware, simulation, supporting software setup, and
future autonomous 3D mapping integration. The goal is indoor and outdoor autonomous
navigation and mapping; the executable code includes the airflow demonstration and an offline rigid-body aircraft model.

## Project guide

- [Connection and communication plan](docs/CONNECTION_PLAN.md)
- [Aircraft flight dynamics model](docs/FLIGHT_MODEL.md)
- [AI feasibility and May delivery plan](docs/AI_AND_DELIVERY_PLAN.md)

- [Hardware and mass budget](HARDWARE.md)
- [Proposed system architecture](docs/ARCHITECTURE.md)
- [Integration roadmap](docs/ROADMAP.md)
- [Setup and dependency management](setup/README.md)
- [Software dependency inventory](setup/dependencies.json) — proposed, not a tested lockfile
- [Configuration](config/README.md)
- [Companion software and offline communication replay](src/README.md)
- [ROS 2 on the Raspberry Pi 5: L2 LiDAR, MAVROS, obstacle sectors](ros2_ws/README.md)
- [Simulation](simulation/README.md)
- [Experiment data](data/README.md)

Existing simulation scripts remain at the root so current commands and debugger settings
continue to work. Supporting robotics packages are not installed by this scaffold.

## Airflow demonstration

Python propeller estimate and 2D incompressible Navier–Stokes airflow demo. Later work can cover aircraft dynamics, sensors, and 3D mapping.

## Setup (VS Code)

Open `3D MAPPING UAV.code-workspace`, or open this folder directly. Install the Python and Python Debugger extensions if you don’t have them. In the terminal:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python simulation.py
python -m unittest -v
```

Select `.venv/bin/python` with **Python: Select Interpreter**. Press F5 to use the launch config. On Windows, activate with `.venv\Scripts\Activate.ps1`.

Outputs are saved in `outputs/`:

- `simulation.png`: airflow and thrust charts; open with macOS Preview.
- `flow.csv`: flow data; open with Numbers or Excel. Each row is one grid point,
  with x/y position in meters, horizontal/vertical velocity and speed in m/s,
  and pressure in pascals.
- `summary.json`: simulation settings and thrust estimates.
- `flow.npz`: NumPy arrays for further Python analysis; use the CSV for spreadsheet viewing.

Example with different throttle:

```sh
python simulation.py --throttle 0.65 --seconds 0.2 --grid 96
```

## Physics

The solver updates horizontal and vertical air velocity:

$$\partial_t\mathbf{u}+(\mathbf{u}\cdot\nabla)\mathbf{u}=-\nabla p/\rho+\nu\nabla^2\mathbf{u}+\mathbf{f},\qquad\nabla\cdot\mathbf{u}=0.$$

It uses Fourier derivatives, a two-thirds spectral filter, pressure projection, and second-order time stepping with adaptive steps. Boundaries are periodic; pressure has zero spatial mean. Background: [NASA Navier–Stokes intro](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/navier-strokes-equation/).

Provisional hardware defaults: 1500 KV, nominal 22.2 V 6S, 4500 mAh, 50C. Capacity and C rating are metadata, not a current or endurance model. Propeller assumptions: 7-inch diameter × 4.5-inch pitch (pitch effects not modeled), 0.80 loaded RPM factor, Ct = 0.11, density 1.225 kg/m³, four motors:

$$RPM=K_V V\eta s,\quad T=C_T\rho(RPM/60)^2D^4,\quad A=\pi D^2/4,\quad\Delta p=T/A,\quad v_i=\sqrt{T/(2\rho A)}.$$

The estimated pressure jump drives a downward Gaussian force near one rotor cross-section. A uniform balancing force cancels the mean so the periodic box doesn’t drift. Disk at y = 0.12 m; downward velocity is negative. Plot pressure is from the flow solve, not the ideal disk estimate.

## Limitations

- 2D slice for learning / early design, not full 3D CFD or mapping.
- Thrust is from the empirical formula; the fluid solver does not invent thrust on its own. Quad totals are 4× one motor, not four rotors in the mesh.
- Periodic boundaries + balancing force → recirculation, not an open wake.
- Default viscosity is 0.001 m²/s (higher than real air) to keep a coarse grid stable. No turbulence model.
- Ct and loaded RPM need measured prop data before you trust numbers for flight. Pitch, torque, battery sag, ground effect, and endurance are not modeled.
- “Supported mass” is static thrust / g, not a payload recommendation.
- Tests check Taylor–Green decay and throttle² scaling. They don’t validate a real propeller.

## Grid comparison

Run `.venv/bin/python compare_grids.py` or choose **Compare airflow grids** in the debugger.
This runs 32×32, 64×64, and 128×128 at the same time, throttle, viscosity, and hardware settings.
Open `outputs/grid_comparison/comparison.png` in Preview and `comparison.csv` in Numbers.
`comparison.txt` explains the numerical differences; each grid folder contains the full outputs.
Colors use shared limits so different runs are directly comparable.

All simulations now use a fixed 0.05 m forcing thickness (previously grid dependent).
This changes the baseline result, but avoids changing the physical forcing width in a grid study.
Override it with `--force-thickness`. The provisional 7×4.5-inch propeller, Ct=0.11, and 0.8 loaded RPM factor
remain unvalidated assumptions, not a hardware compatibility recommendation.

Hardware can change without editing code, for example:

```sh
.venv/bin/python compare_grids.py --motor-kv 1500 --battery-cells 6 --battery-mah 4500 --battery-c 50 --seconds 0.15
```

The same hardware flags work with `simulation.py`. Voltage uses 3.7 V per cell nominal;
battery sag and changing charge state are not simulated. Relative L2 differences measure
whole-field differences at shared grid points; smaller differences with refinement suggest
consistency, but do not establish real-world accuracy. Adaptive time steps also change with
grid size, so this is an initial refinement study, not an isolated spatial convergence proof.

Propeller dimensions are configurable with `--prop-diameter-in 7 --prop-pitch-in 4.5`.
Diameter controls the empirical thrust formula and rotor strip width. Pitch is recorded
and displayed only; Ct and loaded RPM still need data for the chosen motor/prop combination.

## Aircraft mass and hardware details

See [HARDWARE.md](HARDWARE.md) for the supplied motor and two-blade 7×4.5 propeller
specifications and unresolved mounting details. `--mass-kg` defaults to 2.2 kg total
takeoff mass. The thrust chart shows the corresponding hover target; the summary saves
total and per-motor required thrust. This does not validate the estimated thrust curve.

## Share a flight-model visual

In VS Code select **Flight model — shareable roll animation** under Run and Debug,
then press F5. Or run `.venv/bin/python flight_dynamics.py --scenario roll --seconds 0.5`.
Open `outputs/flight/roll.gif` for the animation or `roll.png` for a presentation image.
The animation uses simulated data with provisional parameters; it is not recorded flight.
Running `flight_dynamics.py` without arguments generates the stationary hover example.

## Feedback-control demonstration

Run `flight_control.py` or choose **Feedback control — comparison animation** in VS Code.
See [controller experiment and results](docs/FEEDBACK_CONTROL.md) for the 10-second
hover, 10° roll, return, disturbance recovery, side-by-side animation and SITL next steps.

## ArduCopter SITL mission runner

Run `.venv/bin/python sitl_mission.py` or select **SITL mission — takeoff, hover,
waypoint, land** in Run and Debug. The program launches its own local simulator
and saves telemetry, acknowledgments, completion metrics, a chart and a GIF.
See [build, run and acceptance criteria](setup/SITL.md).
