# Lab 2 Validation

## Build

`two_drones_pkg` was built successfully with Catkin.

Both executables were generated:

- `frames_publisher_node`
- `plots_publisher_node`

The current RoboStack environment requires C++14 because its newer Boost
version is incompatible with the original C++11 setting. This is an
environment compatibility adjustment and does not change the Lab 2
algorithm.

## Transform runtime test

`frames_publisher_node` successfully published dynamic TF transforms.

Observed properties:

- `world -> av1` changes over time;
- AV1 remains on the unit circle in the world x-y plane;
- AV1 yaw changes consistently with `yaw = t`;
- `world -> av2` changes over time;
- AV2 orientation remains identity;
- AV2 satisfies the expected parabolic relation.

## Deliverable 3 runtime test

The `/visuals` topic ran at approximately 50 Hz.

Observed markers included:

- `Trail av1-world`, frame `world`;
- `Trail av2-world`, frame `world`;
- `Trail av2-av1`, frame `av1`.

The relative AV2-in-AV1 trajectory accumulated successfully and no TF
lookup errors were observed.

Result: PASS.
