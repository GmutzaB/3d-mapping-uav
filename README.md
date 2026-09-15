# 3D Mapping UAV

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

Outputs go to `outputs/simulation.png`, `outputs/summary.json`, and `outputs/flow.npz`. Example with different throttle:

```sh
python simulation.py --throttle 0.65 --seconds 0.2 --grid 96
```

## Physics

The solver updates horizontal and vertical air velocity:

$$\partial_t\mathbf{u}+(\mathbf{u}\cdot\nabla)\mathbf{u}=-\nabla p/\rho+\nu\nabla^2\mathbf{u}+\mathbf{f},\qquad\nabla\cdot\mathbf{u}=0.$$

It uses Fourier derivatives, a two-thirds spectral filter, pressure projection, and second-order time stepping with adaptive steps. Boundaries are periodic; pressure has zero spatial mean. Background: [NASA Navier–Stokes intro](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/navier-strokes-equation/).

Propeller numbers (editable in `propeller()`): 2200 KV, 14.8 V 4S, 5-inch diameter, 0.80 loaded RPM factor, Ct = 0.11, density 1.225 kg/m³, four motors:

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
