# Experiment data

Keep large LiDAR recordings, generated maps and flight logs outside normal Git history.
Local datasets can live in ignored `data/raw/` and `data/processed/`; simulation outputs
remain in ignored `outputs/`.

For each experiment, commit a small Markdown record here containing:

- purpose and procedure;
- hardware, firmware and software revisions;
- configuration and calibration reference;
- dataset location and SHA-256 checksum;
- units, coordinate frame and time convention;
- measured result and limitations.

Use a suitable external data store for large artifacts and link it from the experiment
record. Do not commit credentials or recordings containing information you cannot share.
