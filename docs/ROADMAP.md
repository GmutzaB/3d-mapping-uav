# Integration roadmap

Complete each milestone with recorded evidence before marking it working.

- [x] Establish the existing repository as the home for simulation and autonomy work.
- [x] Record the proposed architecture and supporting software inventory.
- [ ] Confirm H743 board revision, installed firmware and sensor identification.
- [ ] Select a compatible Pi OS / ROS / L2 driver / mapping combination.
- [ ] Pin evaluated upstream commits and document reproducible installation commands.
- [ ] Acquire and replay L2 point-cloud and IMU data on the bench.
- [ ] Produce a map from a hand-carried dataset; measure drift and dimensional error.
- [ ] Benchmark CPU, memory, temperature, dropped data and processing latency on the Pi.
- [ ] Connect the companion pipeline to ArduPilot SITL and test coordinate conventions.
- [ ] Demonstrate assigned-destination navigation and obstacle avoidance in simulation.
- [ ] Test localization loss, delayed data, communication loss and planner failure in simulation.
- [ ] Integrate hardware, calibrate, and validate supervised flight in stages.
- [ ] Demonstrate mapping and navigation separately indoors and outdoors.
- [ ] Evaluate self-directed exploration and indoor/outdoor transitions if required.

## Requirements still to choose

Record decisions here or in a linked design note: mission type; map area/volume; required
accuracy; maximum flight speed; live versus postflight visualization; GPS/compass hardware;
flight duration; operational boundaries; and completion/return criteria.

## Evidence for each milestone

Record repository commit, hardware revision, OS/ROS versions, dependency revisions,
configuration used, procedure, expected result, measured result and limitations.
Link large datasets externally using the guidance in `../data/README.md`.

## September 28 software increment

- [x] Add offline rigid-body dynamics and analytical unit tests.
- [x] Draft connection contracts and AI feasibility / May 2027 delivery plan.
- [ ] Confirm exact May deadline and measurable demonstration requirements.

See [delivery plan](AI_AND_DELIVERY_PLAN.md) and [connection plan](CONNECTION_PLAN.md).

- [x] Verify companion frame conversions, latched command gating and MAVLink wire encoding.
- [x] Exercise synthetic planner/localization loss and operator recovery in offline replay.
- [ ] Verify the same scenarios against an actual ArduPilot SITL instance.
