# Setup and dependency management

## Existing simulation

Run from this repository's root (the checkout containing `HARDWARE.md`):

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest -v
python simulation.py
```

On Windows activate with `.venv\Scripts\Activate.ps1` instead.
The existing requirements file applies only to the airflow simulation.

## Supporting robotics software

`dependencies.json` is an inventory, not an installer or a tested lockfile.
A null version means no version has been selected. No flight firmware, ROS packages,
or third-party source trees are installed by this repository scaffold.

Use these stages independently of selection:

- `unverified`: no project test recorded.
- `bench-tested`: reproducible bench result with evidence.
- `simulation-tested`: integrated simulation result with evidence.
- `hardware-tested`: result recorded on the actual hardware, with test scope stated.

Selection is `planned` or `candidate`; neither means tested.

Before installing the companion stack, select the OS, ROS distribution, driver and mapping
package together. Unitree provides ROS 1 and ROS 2 drivers, but its Point-LIO instructions
currently target ROS 1 Noetic. Do not mix those installation instructions into one environment.

Once a combination is evaluated:

1. Record each source dependency's exact commit in `dependencies.json`, or the exact
   firmware/application release. Add the platform and test evidence.
2. Add platform-specific setup instructions here with explicit versions.
3. For a ROS workspace, add a vcstool `.repos` manifest with full commit hashes; keep
   downloaded sources under ignored `external/`, outside our own `src/` packages.
4. Keep upstream licenses and notices. Use a fork only when changes are needed;
   record both the upstream and fork revisions.
5. Save reviewed hardware-specific configuration under `config/`.

Do not use a floating `main` or `latest` as a reproducibility pin. The application
version ranges in the existing simulation requirements are not an exact environment lock.

## Where software runs

- Development computer: existing Python model; ground tools; future ArduPilot SITL/Gazebo.
- Raspberry Pi 5: proposed sensor acquisition, mapping, planning and mission integration.
- DAKEFPV H743: ArduCopter firmware, using the standard `DAKEFPVH743` target after
  physical identification. The Pro target is distinct.

## Next practical task

Identify the installed Pi operating system and select a compatible mapping environment.
Then benchmark L2 acquisition and mapping before adding autonomous control.
