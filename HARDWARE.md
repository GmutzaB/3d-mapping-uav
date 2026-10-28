# Quadcopter hardware

Current selection, October 28, 2026: a **9-inch** frame, **900 KV** motors, and either a
**7000 mAh** or a **10000 mAh** battery. Manufacturer, pitch, C rating, and pack mass
were not supplied. The earlier 7×4.5 inch propeller, 1500 KV motor sheet, and 4500 mAh
pack are recorded at the bottom and are no longer the model defaults.

## Mass assumption

Total takeoff mass: **2.2 kg**, including battery and mapping payload. This is still a
planning assumption, not a measured mass. Both battery capacities use this same number
until each pack is weighed. Hover at 2.2 kg requires 21.582 N total, or 5.3955 N per
rotor with equal loading (550 gram-force each).

### Known component mass

| Component | Quantity | Unit mass | Subtotal |
|---|---:|---:|---:|
| Unitree L2 4D LiDAR (candidate) | 1 | 230 g | 230 g |
| 900 KV motors | 4 | not supplied | — |
| 9 inch propellers | 4 | not supplied | — |
| 7000 mAh or 10000 mAh battery | 1 | not supplied | — |

The frame, ESCs, flight controller, Pi 5, wiring, mounts, and landing gear are also
still unweighed. The 2.2 kg figure stays in the models only as that shared placeholder.

## Propeller (each)

- Diameter: **9 inches**. This is the value in the thrust formula and the rotor strip.
- Pitch: not supplied. Pitch is not modeled.
- Blade count, material, mass, hub, and bore: not supplied for this propeller.

## Motor (each)

- KV: **900**
- Count: 4
- Voltage class, mass, dimensions, resistance, and current ratings: not supplied with this change.

The airflow model still uses a nominal **6S / 22.2 V** (3.7 V per cell) because the
voltage was not restated. Say so if these motors are on a different cell count.

## Battery

One aircraft, two candidate packs. `--battery-mah` accepts `7000` or `10000`.
The default run records 7000 mAh and also stores the other pack's energy.

| Pack | Nominal energy at 6S |
|---|---:|
| 7000 mAh | 155.4 Wh |
| 10000 mAh | 222.0 Wh |

Capacity does not change RPM or thrust in this model. C rating and pack mass were not
supplied. A heavier 10000 mAh pack would raise the real hover throttle; the 2.2 kg
placeholder does not show that difference.

## Mapping payload

**Unitree L2 4D LiDAR**, a candidate under consideration (updated September 28, 2026). Mass: **230 g**, supplied by the user.
Power requirements, data interface details, and mounting hardware mass have not yet
been verified for the connection plan.

## What the code uses

`hardware.py` is the shared selection. Motor KV, nominal battery voltage, and propeller
diameter enter the empirical thrust formula in the airflow model. The rigid-body model's
per-motor thrust ceiling is that same formula at full throttle. Pitch is recorded only
when passed on the command line. Ct=0.11 and the 0.8 loaded RPM factor remain unvalidated
assumptions. The hover target is a weight balance, not evidence that this hardware
achieves it. Current draw and endurance are not predicted.

```sh
python simulation.py --battery-mah 7000
python simulation.py --battery-mah 10000
```

Use `--mass-kg` when a pack has been weighed. The arm length in `flight_dynamics.py`
is still the unmeasured 0.20 m. Center-to-center motor spacing at that arm is about
0.28 m, which clears a 9-inch (0.23 m) propeller; it is not a measured frame dimension.

## Flight electronics and onboard computer

- Owned flight controller / ESC: **DAKEFPV H743 / 55A ESC stack**. The user supplied
  a matching listing and reports the board is labeled H743, with no Pro marking.
  Treat it as the standard H743 provisionally; verify board revision and installed
  firmware identification before flashing. Mass, exact ESC variant, voltage rating
  and rating conditions remain unverified.
- Planned flight firmware: **ArduCopter**, target `DAKEFPVH743`. Official support
  exists for this board family; no firmware has been selected or tested on this aircraft.
  [Board documentation](https://ardupilot.org/plane/docs/common-dakefpvh743pro.html)
  and [standard H743 firmware](https://firmware.ardupilot.org/Copter/stable/DAKEFPVH743/).
- Owned onboard computer: **Raspberry Pi 5**, purchase confirmed September 28, 2026. Earlier notes listed 8 GB RAM; capacity needs confirmation.
  Mass, regulator, cooling, storage, and mounting hardware are not yet specified.

These items are part of the 2.2 kg total allowance, not additional mass on top of it.
Their unknown weights have not been added to the known subtotal. Electrical compatibility and onboard computer power consumption have not yet been verified.

## Superseded specification

The previous model used a user-supplied 7×4.5 inch, 2-blade, 6.6 g propeller and a
1500 KV motor sheet (46.7 g with cable, 5–6S, 62 milliohm, 47.1 A peak for 60 seconds,
1089 W peak for 60 seconds), plus a 6S 4500 mAh 50C pack listed at 618 g. Those parts
are not the current selection. Their peak ratings were never hover current, and they
are not applied to the 900 KV motors.

## Earlier clarification — September 28, 2026

The project is being built from the ground up. The owned electronics are an STM H743
flight controller with a 55A ESC stack; the exact manufacturer/revision still needs
confirmation against the earlier DAKEFPV identification. Earlier firmware-target
notes above remain provisional. The onboard computer is an owned Raspberry Pi 5 (purchase confirmed in the follow-up);
RAM capacity remains unconfirmed. Unitree L2 is a candidate, not a final sensor
choice. The frame is tentatively to be printed in a polycarbonate/nylon material;
exact material, geometry, print process, mass and structural properties are unresolved.
The 230 g LiDAR allowance depends on selecting L2.
