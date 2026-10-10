# fr3_tracking

Vision tracking on the [impedance controller](../fr3_mating_controllers/):
`tracking_node` makes the FR3 follow a part seen by the wrist camera by
streaming the controller's equilibrium at 50 Hz. Also here: `state_recorder`,
a 1 kHz robot-state recorder for diagnosis.

How it works, every parameter, the status topic and the log format:
[docs/tracking.md](../docs/tracking.md).

## `tracking_node`

- **Idle until asked.** It comes up idle and publishes nothing until
  `~/start_tracking` (the panel's TRACK button).
- **Camera-centred goal.** It keeps the camera at `tracking_standoff_m` from
  the part (`/object/pose`), at a set in-plane angle.
- **Friction integrator.** A bounded integrator of the measured error pushes
  the equilibrium past the goal until friction's residual closes, never by
  more than 15 N worth.
- **Holds by itself** on stale vision or robot state, without a fresh raw
  detection, at the Z floor, at the workspace box, near a joint limit, or
  past its lead cap (policy `hold`).
- **Stops by itself** and restores the gains on buzz, when the controller
  leaves ACTIVE, past its lead cap under policy `stop`, and on shutdown.
- **Gains.** It keeps the operator's stiffness, caps `zeta` at 0.5 for the
  session and restores everything at STOP.

```bash
ros2 run fr3_tracking tracking_node --ros-args --params-file fr3_cell/config/fr3_params.yaml
ros2 service call /tracking_node/start_tracking std_srvs/srv/Trigger
ros2 service call /tracking_node/stop_tracking std_srvs/srv/Trigger
ros2 topic echo /tracking_node/status
```

The cell launch starts it with `fr3_cell/config/fr3_params.yaml`. Every
`tracking_*` and `track_*` key must be in the parameter file: a missing key
stops the node with its name. START also needs the impedance controller
active, the hand-eye TF and fresh robot state, and the panel writes the
session goal before it; use the panel rather than the service call on a
real arm.

## `state_recorder`

Read-only: subscribes to `/franka_robot_state_broadcaster/robot_state` and
writes one CSV row per message (joint positions, velocities, torques,
external wrench, `O_T_EE`, commanded torque, ...), plus a `.json` with the
inertia and frames that do not change. It publishes and calls nothing.

```bash
ros2 run fr3_tracking state_recorder                                 # until Ctrl-C
ros2 run fr3_tracking state_recorder --ros-args -p duration_s:=60.0
```

Output: `$FR3_LOG_DIR/YYYY-MM-DD/state_<time>.csv` (or `-p out:=<path>`).

## Files

| Path | What |
|---|---|
| `src/tracking_node.cpp` | The node: services, subscriptions, START/STOP, the 50 Hz loop, status and log |
| `include/fr3_tracking/tracking_law.hpp` | The pure tracking law: goal, integrator, glide, verdicts, joint-limit hold, buzz meter, config checks |
| `include/fr3_tracking/mating_geometry.hpp` | Pose math (the node uses `pose_error`) |
| `src/state_recorder.cpp` | The 1 kHz recorder |
| `test/` | gtests for both headers |

## Tests

```bash
colcon test --packages-select fr3_tracking && colcon test-result --verbose
```

`test_tracking_law.cpp` and `test_mating_geometry.cpp` cover the law without
ROS. The node itself is exercised by `fr3_cell/sim/tracking_smoke.py` (run by
`colcon test --packages-select fr3_cell`): the real `tracking_node` in a fake
cell on the isolated DDS domain 87.
