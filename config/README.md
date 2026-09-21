# Project configuration

This directory will hold reviewed configuration for the actual vehicle. No loadable
flight parameters are supplied yet because board revision, calibration and interfaces
have not been verified on hardware.

Add files by subsystem as they become available:

- `arducopter/`: exported parameters with firmware version, board revision and test notes.
- `lidar/`: driver settings, selected mode and verified network setup.
- `mapping/`: algorithm settings and measured sensor transforms.
- `navigation/`: aircraft clearance, motion limits and mission boundaries.

Commit portable examples; keep device-specific secrets and credentials in ignored
`*.local.*` or `.env` files. Record units, coordinate conventions and provenance for
calibration values. Label examples clearly so they cannot be mistaken for flight-tested settings.
