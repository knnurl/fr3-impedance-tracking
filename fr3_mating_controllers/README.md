# fr3_mating_controllers

A `franka_ros2` controller plugin,
`fr3_mating_controllers/CartesianImpedanceStrokeController`: a 1 kHz
Cartesian impedance controller for the Franka FR3.

```
tau = J^T ( K (x_d - x) - D J dq ) + tau_nullspace + coriolis + tau_wall      @ 1 kHz
```

- **Tool-frame stiffness.** `K` is diagonal in the tool axes of the
  equilibrium, so stiffness can differ per tool axis; `D = 2 zeta sqrt(K)`.
- **One input.** Other nodes move the arm by publishing an equilibrium pose
  on `~/equilibrium_pose`. Setpoints are slew-limited, so a jump on the topic
  becomes a bounded ramp.
- **Live tuning.** Gains, damping and slew limits change while it holds the
  arm, all or nothing per parameter set.
- **Float mode.** Coriolis only, so the arm floats under gravity
  compensation; switching it off re-seeds the equilibrium where the arm is.
- **Bounds whatever the gains.** Caps on the commanded wrench (30 N / 10 Nm)
  and per-joint torque, a 1 Nm/ms torque-rate limit, malformed setpoints
  dropped, and a joint-limit wall with a speed brake in every mode.

The control law, every parameter with its range, the tuning procedure and
the gain presets are in [docs/controller.md](../docs/controller.md).

## Files

| Path | What |
|---|---|
| `src/cartesian_impedance_stroke_controller.cpp` | The plugin: interfaces, parameters, the 1 kHz `update()` |
| `include/fr3_mating_controllers/impedance_detail.hpp` | ROS-free safety logic: gain and ceiling limits, the setpoint handoff, the joint wall (header-only, Eigen only) |
| `config/cartesian_impedance_stroke.yaml` | Shipped parameters |
| `fr3_mating_controllers.xml` | Plugin export |
| `test/test_impedance_detail.cpp` | gtests for `impedance_detail.hpp` |

## Interface

| Name | Type | |
|---|---|---|
| `fr3_joint1..7/effort` | command interfaces | Claimed while active; never run alongside `fr3_arm_controller` |
| `~/equilibrium_pose` | `geometry_msgs/PoseStamped` (base frame) | The equilibrium target. Last message wins |
| `~/transition_event` | `lifecycle_msgs/TransitionEvent` | Published by the lifecycle; `tracking_node` stops when the controller leaves ACTIVE |

Main parameters: `k_pos_tool` `[150, 150, 800]` N/m, `k_rot_tool`
`[10, 10, 20]` Nm/rad, `damping_ratio` 1.0, `setpoint_slew_mps` 0.05 m/s,
`float_mode` false (all live); `max_force_n` 30 N, `max_torque_nm` 10 Nm, the
joint wall (configure-time). Full table:
[docs/controller.md#parameters](../docs/controller.md#parameters).

## Usage

The cell launch (`fr3_cell`) spawns it inactive into the driver's
controller manager, and the panel switches it in and out. By hand:

```bash
ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
    --inactive --param-file fr3_mating_controllers/config/cartesian_impedance_stroke.yaml
ros2 param set /cartesian_impedance_stroke_controller k_pos_tool "[300.0, 300.0, 800.0]"
```

The controller manager runs in the driver's terminal, so that terminal must
have this workspace sourced (`fr3_cell/scripts/fr3_env.sh` does it).
Otherwise the spawner fails with "Loader for controller ... not found".

## Build and test

Builds only where `franka_ros2` is available.

```bash
colcon build --packages-select fr3_mating_controllers
colcon test --packages-select fr3_mating_controllers && colcon test-result --verbose
```

The gtests cover the gain, ceiling and slew limits, the setpoint handoff
between the subscriber and the 1 kHz loop, and the joint wall. The control
law itself can only be checked on the real arm: the `franka_ros2` mock
hardware does not integrate torques. Commission it with the ladder in
[docs/controller.md#tuning-procedure](../docs/controller.md#tuning-procedure).
