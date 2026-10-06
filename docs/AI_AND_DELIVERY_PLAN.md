# AI feasibility and delivery plan

Deadline interpretation: May 2027; aim for an integrated demonstration by April,
leaving May for final evidence and presentation. Dates are planning targets, not promises.

## Minimum demonstration to propose

A supervised mapping flight in a bounded test area, reproducible map export,
recorded flight/estimator health, and a documented comparison between simulation
and measurements. Prove mapping and basic navigation separately before combining.
Indoor/outdoor transitions and autonomous exploration remain stretch scope.
Confirm required environment, map accuracy, flight duration and mission completion
criteria with the project team before fixing acceptance thresholds.

| Period | Deliverable and exit evidence |
|---|---|
| Sep–Oct 2026 | Confirm hardware and electrical interfaces; baseline dynamics tests; select compatible software stack |
| Nov | Hand-carried sensor recording/replay and map; drift and dimensional error measurements |
| Dec | Telemetry/external-navigation adapter in SITL; frame and timing tests |
| Jan 2027 | Mission supervisor, constrained planning, stale-data and link-loss tests |
| Feb | Pi load/power/thermal benchmarks; integrated bench setup; measured aircraft parameters |
| Mar | Supervised flight validation in stages; compare model against logs |
| Apr | Repeatable mapping demonstration; freeze features and collect report evidence |
| May | Contingency, final report and presentation; exact day to confirm |

Hardware delays should not stop recorded-data, interface or SITL work. Do not make
completion depend on a new AI feature. Reassess scope monthly against measured evidence.

## Which AI is useful?

Mapping and path planning do not inherently require a neural network. The first
candidate is a small perception model that labels objects or regions in the map.
A camera would need to be added for image models; a LiDAR supplies geometry, not RGB
images. LiDAR semantic models require compatible point-cloud data and a separate
benchmark. A second candidate is offline anomaly classification from flight logs,
which can start without changing flight control.

A language model could assist ground-side mission drafting or explain recorded
results. Do not give its free-form output direct motor or flight command authority.
Any future mission proposals must pass the same deterministic bounds and supervisor.

[Official Raspberry Pi AI documentation](https://www.raspberrypi.com/documentation/computers/ai.html)
describes Pi 5 vision inference with AI HAT accelerators, and generative models with
AI HAT+ 2. That makes onboard AI technically possible, but does not establish that
mapping and inference fit this aircraft's power, weight, temperature or latency budget.
Its documented OS/runtime requirements must be reconciled with the mapping stack.
The user confirms that the Raspberry Pi 5 has been purchased; RAM capacity remains
unconfirmed. No additional hardware purchase is recommended yet.

## Experiment before commitment

1. Choose a single task and labeled recorded dataset; specify what counts as useful.
2. Benchmark a small candidate on the actual Pi with mapping running concurrently.
3. Record model/version, input size, median and p95 latency, memory, CPU, temperature,
   power, task accuracy and odometry timing with AI enabled and disabled.
4. Run AI as an optional process with bounded queues and timestamped outputs; discard
   late outputs. Demonstrate that loss/overload of AI cannot stall the navigation loop.
5. Keep AI only if it meets agreed accuracy and resource margins. Otherwise present
   the offline experiment as future work while completing the core drone.

No model has been selected, installed, trained or benchmarked on this aircraft yet.
