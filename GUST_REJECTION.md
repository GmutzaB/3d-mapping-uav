# Wind-gust rejection (Dryden turbulence)

`python gust_rejection.py` flies the existing 6-DOF quad through wind and compares two
controllers on the *same* wind sample. Outputs go to `outputs/gust/`:
`gust_rejection.gif` (3D animation), `gust_summary.png` (charts), `metrics.json`, and
`position_hold.npz` / `attitude_only.npz` (raw samples).

Options: `--wind` (mean m/s, default 8), `--gust` (discrete gust peak m/s, default 6),
`--seconds` (default 36), `--seed` (default 7), `--no-visuals`.

## Wind model

- **Mean wind** toward east (from the west), smoothly ramped in over the first 4 s.
- **Dryden continuous turbulence**, MIL-HDBK-1797 / MIL-F-8785C low-altitude form.
  With h in ft and W20 the mean wind at 20 ft:
  σw = 0.1·W20, σu = σv = σw/(0.177 + 0.000823h)^0.4,
  Lw = h, Lu = Lv = h/(0.177 + 0.000823h)^1.2. Altitude is clamped to the model's 10 ft floor.
  Forming filters: Hu = σu√(2Lu/πV)/(1 + (Lu/V)s);
  Hv,w = σ√(L/πV)(1 + √3(L/V)s)/(1 + (L/V)s)².
  They are discretized exactly (Van Loan) and started from their stationary state.
- **Frozen turbulence:** V is the mean wind speed, i.e. the wind carries the gust field
  past a hovering aircraft. The model is not valid for V → 0.
- **Discrete gust:** a 1-cosine crosswind (toward north) at 18 s lasting 3 s.

The wind acts as quadratic drag on the air-relative velocity, with CdA = 0.05 m² sideways
and 0.10 m² vertically. Both are unmeasured estimates.

## Controllers

- **Position hold (new):** cascaded PID. Position error → desired acceleration → tilt
  direction and collective thrust, with a 25° tilt limit and integral anti-windup. It feeds
  the same attitude law as `flight_control.py`. Gains place a triple pole at 1.1 rad/s
  (Kp = 3ω², Kd = 3ω, Ki = ω³), well below the 6 rad/s attitude loop.
- **Attitude only (existing):** `flight_control.Controller`, which holds level and height
  but has no horizontal position feedback. It shows what happens without an outer loop.

## Default results (seed 7)

| | Position hold | Attitude only |
|---|---:|---:|
| RMS horizontal error after wind builds | 6.9 cm | 72 m |
| Max horizontal error | 17 cm (during gust) | 136 m (end) |
| Max altitude error | 5.9 cm | 2.8 cm |
| Max tilt | 9.4° | 0° |
| Motor saturation | none | none |

The integral term removes the steady-wind offset. The drone leans into the wind instead
of being pushed off station.

**Power:** this uses the same hover momentum-theory assumptions as the team's
`Prop_and_Motor_Calcs.xlsx` (FM 0.65, ESC 0.85, 45 W electronics, 10 Ah, 80% usable).
At the model's provisional 2.2 kg, hover is about 331 W (32.2 min). The extra thrust
needed to tilt into the wind adds only about 1% (31.9 min). Momentum theory ignores
translational lift and profile/parasitic power, so treat this as indicative only.

## Limitations

Simulation only. Mass (2.2 kg here vs. 2.42 kg in the spreadsheet's 10 Ah case),
inertia, thrust and drag areas are unmeasured. Perfect state feedback: no GPS/EKF noise,
so real hold error will be larger (GPS hold is typically decimeters to meters). Wind is
uniform across the airframe. ArduPilot's real position controller is not modeled.

## Next

Dryden plus a Navier–Stokes building wake (fly the drone through the wind field around
a structure). This needs inflow/outflow boundaries in the airflow solver. Adding position
sensor noise would also give a more realistic hold-error band.
