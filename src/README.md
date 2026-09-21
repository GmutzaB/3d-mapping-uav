# Custom autonomy software

Place our integration code here as real packages are implemented. Planned responsibilities:

- external-position and telemetry integration with ArduCopter;
- mission state coordination;
- planner/trajectory integration;
- data freshness and system-health monitoring;
- map export and experiment metadata where existing tools need adaptation.

No runtime autonomy code exists here yet. Prefer existing drivers, flight control and
mapping algorithms. Keep downloaded third-party sources in ignored `external/` and
record their versions in `../setup/dependencies.json`.
