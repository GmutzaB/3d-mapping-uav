# Provisional quadcopter hardware

Source: user-supplied propeller specifications and motor specification image.
Exact manufacturer/model and thrust data for this combination are not yet available.

## Mass assumption

Total takeoff mass: **2.2 kg**, including battery and mapping payload. This is a planning
assumption, not a measured mass or confirmed upper bound. Hover requires 21.582 N total,
or 5.3955 N per rotor with equal loading (550 gram-force each).
### Known component mass budget

| Component | Quantity | Unit mass | Subtotal |
|---|---:|---:|---:|
| Motors, including cables | 4 | 46.7 g | 186.8 g |
| Propellers | 4 | 6.6 g | 26.4 g |
| Battery | 1 | 618 g | 618 g |
| Unitree L2 4D LiDAR | 1 | 230 g | 230 g |
| **Known subtotal** | | | **1061.2 g** |
| **Remaining within 2200 g target** | | | **1138.8 g** |

The remaining allowance must cover the frame, ESCs, flight controller, receiver,
other sensors, any companion computer, power distribution/regulators, additional wiring,
mounts, landing gear, and fasteners. Avoid counting the included motor cables twice.
The known components already exceed the earlier hoped-for 1 kg total.
The 2.2 kg value remains a target until the remaining components are weighed.
The user is investigating mounting compatibility and the lack of combination thrust data.

## Propeller (each)

- Diameter: 7 inches; pitch: 4.5 inches
- Blades: 2; material: polycarbonate
- Mass: 6.6 g
- Hub diameter: 13.5 mm; hub thickness: 7.5 mm
- Bore: 5 mm (user specification labeled “shaft”)

## Motor (each; transcribed from image)

- KV: 1500; rated LiPo: 5–6S
- Dimensions: diameter 33.4 × 34.7 mm
- Mass including cable: 46.7 g
- Shaft diameter: 4 mm
- Lead: 20 AWG, 230 mm
- Configuration: 12N14P
- Internal resistance: 62 milliohms
- Idle current at 10 V: 1.2 A
- Peak current (60 seconds): 47.1 A
- Maximum power (60 seconds): 1089 W

These peak ratings are not hover consumption or continuous ratings. The image does not
establish the current or thrust with this specific propeller. Check the actual propeller
mount/adapter dimensions: a listed 4 mm motor shaft and 5 mm propeller bore alone do not
establish fit, since a separate propeller mount may be used.

## Battery

Provisional 6S, 4500 mAh, 50C LiPo; nominal model voltage 22.2 V.
Mass: **618 g**, supplied by the user. Exact model is unknown.

## Mapping payload

**Unitree L2 4D LiDAR**, confirmed by the user. Mass: **230 g**, supplied by the user.
Power requirements, data interface details, and mounting hardware mass have not yet
been verified for the connection plan.

## What the code uses

Motor KV, nominal battery voltage, and propeller diameter enter the empirical thrust
formula. Pitch is recorded only. Blade count, material, motor resistance, and peak ratings
are documented here but are not aerodynamic/electrical model inputs. Ct=0.11 and the
0.8 loaded RPM factor remain unvalidated assumptions. The hover target is a weight balance,
not evidence that this hardware achieves it. Current draw and endurance are not predicted.

Use `--mass-kg 2.2` with either simulation script to change the total mass assumption.

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
- Planned onboard computer: **Raspberry Pi 5 with 8 GB RAM**, confirmed by the user.
  Mass, regulator, cooling, storage, and mounting hardware are not yet specified.

These items are part of the 2.2 kg total allowance, not additional mass on top of it.
Their unknown weights have not been added to the known subtotal. Electrical compatibility and onboard computer power consumption have not yet been verified.
