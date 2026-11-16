# Flight time, battery, and mapping-survey simulations

Two simulations, added November 16, 2026. Both are simulation only. They use the
selected **6S 9000 mAh 80C LiPo** (cell count assumed 6S, as everywhere else in the
repo) and the same rotor-efficiency assumptions as `gust_rejection.PowerModel`.

| Script | Question it answers | Output |
|---|---|---|
| `battery_endurance.py` | How long can it fly? Which speed flies longest or farthest? How much does weight matter? Is there enough thrust and current margin? | `outputs/endurance/endurance.png`, `summary.json`, `power_curve.csv`, `mass_sweep_hover_min.csv` |
| `survey_flight.py` | Fly a full mapping mission in wind: how long does it take, how much battery, how well does it follow the lines, is the area covered? | `outputs/survey/survey.png`, `summary.json`, one `.npz` per wind case |

```sh
.venv/bin/python battery_endurance.py
.venv/bin/python battery_endurance.py --battery-kg 1.12 --dry-kg 1.20   # once things are weighed
.venv/bin/python survey_flight.py                                       # ~45 s: 3 wind cases
.venv/bin/python survey_flight.py --width 200 --length 150 --speed 7 --winds 0 6
```

Both also appear in VS Code under Run and Debug.

## Models

**Rotor power.** Momentum theory for each rotor with Glauert inflow, so it covers
hover, climb and forward flight. Body drag (0.05 m² drag area, same as the gust study)
tilts the aircraft and increases the thrust it needs. Shaft power is divided by
a figure of merit of 0.65 and a drive (ESC × motor) efficiency of 0.85. A 45 W
avionics load (flight controller, Pi 5, L2, radios) is added. Hover power matches
`gust_rejection.PowerModel` exactly, and a test checks this.

**Battery.** LiPo cell resting voltage vs. state of charge, plus internal resistance
(2.5 mΩ/cell + 4 mΩ wiring, estimates). Voltage therefore sags under load and falls
as the pack empties. A flight ends at a **20 % landing reserve** or **3.5 V/cell under
load**, whichever comes first.

**Survey flight.** It uses the existing 6-DOF quad with wind drag, at the full
takeoff mass. The flight is takeoff → climb → lawnmower lines → return → descend. The
existing cascaded position controller follows a moving reference, with speed and
acceleration feed-forward and trapezoidal speed profiles that stop at each turn. Wind
is steady wind plus Dryden turbulence. Rotor power is computed every step from
the actual rotor thrusts and the air-relative velocity, and the battery is integrated
alongside. Coverage is the fraction of the area within half a LiDAR swath of the
flown lines.

## Results with the current estimates

Takeoff mass 2.20 kg = 1.15 kg battery (estimate) + 1.05 kg everything else (the
old 2.2 kg placeholder minus the battery estimate).

| Quantity | Value |
|---|---|
| Hover | 331 W, 13.3 A, 45 % throttle → **30 min** to the 20 % reserve |
| Longest flight | ~9 m/s cruise, 282 W → **35 min** |
| Farthest flight | ~14 m/s, **~24 km** |
| Weight sensitivity | each +100 g costs roughly 1.5–2 min of hover |
| Thrust / weight | 4.8 : 1 (from the unvalidated static-thrust formula) |
| Full-throttle current | ~155 A, about 21 % of the pack's 720 A (80C) rating |

Survey (120 × 80 m area, 12 m height, 15 m line spacing, 20 m swath, 5 m/s lines):

| Wind | Time | Battery used | Path error on lines (RMS / max) | Max tilt | Coverage |
|---|---|---|---|---|---|
| calm | 4.8 min | 12 % | 0.07 / 0.32 m | 20° | 100 % |
| 5 m/s | 4.8 min | 11 % | 0.07 / 0.31 m | 22° | 100 % |
| 8 m/s | 4.8 min | 11 % | 0.09 / 0.29 m | 23° | 100 % |

At that line spacing, one pack covers roughly 9 ha before the reserve. The speed sweep
uses the least battery per area at about 8–9 m/s.

## How to read this

- **The 80C rating is not the limit.** Even at full throttle the motors draw about a
  fifth of what the pack can supply. Capacity and weight decide flight time, so a
  lighter pack with the same capacity is worth more than a higher C rating.
- **Moderate wind does not cost battery here.** It even saves a little. Hovering in
  wind is forward flight through the air, and a rotor moving through fresh air
  needs less induced power (translational lift). Real aircraft show this too, but
  the model leaves out rotor profile drag and interference between rotors, so the
  real saving is likely smaller. Wind *does* raise peak current, tilt and path error.
- **Path following** is dominated by the turns at the end of each line. The largest
  errors (~0.3 m) happen while decelerating into a corner. ArduPilot's real controller
  and GPS/EKF error are not modeled. GPS alone typically wanders 1–2 m, which would
  be much larger than these numbers.
- **The 20 m swath is a placeholder.** The real useful swath depends on how the L2 is
  mounted (dome up/down/tilted), flight height, surface reflectivity, and the point
  density you need. Set `--swath` and `--spacing` once that is decided.

## Measure these to replace estimates

| Estimate | How to measure |
|---|---|
| Battery mass (1.15 kg), airframe mass (1.05 kg) | Kitchen/postal scale; re-run with `--battery-kg` / `--dry-kg` |
| Cell internal resistance | Most smart chargers report IR per cell |
| Hover power (FM × drive efficiency) | Hover for 60 s, read current/voltage from the FC log (BAT.Curr, BAT.Volt) |
| Avionics power (45 W) | USB/inline power meter on the Pi + L2 on the bench |
| Drag area (0.05 m²) | Fly constant speeds in calm air and compare logged power to the power curve |
| Thrust (Ct = 0.11) | Thrust stand or a scale under one motor + prop |

## Limitations

Steady-flight endurance: no temperature, ageing, or capacity loss at high current.
No rotor profile-drag rise with speed, rotor interference, ground effect, or
vortex-ring state in fast descents. The survey uses perfect position sensing and
uniform wind, and has no ground contact (it ends 0.3 m above the pad).
