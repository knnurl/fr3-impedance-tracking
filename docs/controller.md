# The Cartesian impedance controller

`fr3_mating_controllers/CartesianImpedanceStrokeController` is a
`ros2_control` controller plugin for `franka_ros2`. It runs inside the
controller manager of the Franka driver at 1 kHz, commands joint torques on
the seven `fr3_jointN/effort` interfaces, and holds the end effector on a
spring around an *equilibrium pose*. Other nodes move the arm by moving that
equilibrium on `~/equilibrium_pose`. The controller does nothing else: it has
no notion of tasks, targets or vision.

Source: [`cartesian_impedance_stroke_controller.cpp`](../fr3_mating_controllers/src/cartesian_impedance_stroke_controller.cpp),
the safety logic in [`impedance_detail.hpp`](../fr3_mating_controllers/include/fr3_mating_controllers/impedance_detail.hpp)
(header-only, Eigen only, unit-tested without a robot), parameters in
[`cartesian_impedance_stroke.yaml`](../fr3_mating_controllers/config/cartesian_impedance_stroke.yaml).

## Contents

- [The control law](#the-control-law)
- [Stiffness in the tool frame, and the damping](#stiffness-in-the-tool-frame-and-the-damping)
- [The equilibrium and its slew limits](#the-equilibrium-and-its-slew-limits)
- [Caps that hold whatever the gains](#caps-that-hold-whatever-the-gains)
- [Malformed setpoints and bad numbers](#malformed-setpoints-and-bad-numbers)
- [Float mode](#float-mode)
- [The joint-limit wall and speed brake](#the-joint-limit-wall-and-speed-brake)
- [Changing parameters](#changing-parameters)
- [Parameters](#parameters)
- [Tuning procedure](#tuning-procedure)
- [Gain presets](#gain-presets)
- [Tests](#tests)

## The control law

Every 1 ms cycle:

```
F      = K (x_d - x) - D J dq                          wrench at the end effector, base frame
F      = clamp(F)                                      |force| <= max_force_n, |moment| <= max_torque_nm
tau    = J^T F + N (k_ns (q_0 - q) - 2 sqrt(k_ns) dq) + coriolis + tau_wall
tau    = rate_limit(clamp_per_joint(tau))              |tau_i| <= tau_max_nm[i], |dtau_i| <= tau_rate_limit per cycle
```

- `x` is the measured end-effector pose (`franka::Frame::kEndEffector`) and
  `J` the zero Jacobian, both from the Franka robot model. `x_d` is the
  equilibrium.
- The pose error `x_d - x` is the position difference and, for orientation,
  the rotation vector of `q_d * q^-1`, taken the short way round.
- `N = I - J^T (J J^T + 1e-6 I)^-1 J` projects the posture term into the
  nullspace of the task. The posture `q_0` is the joint configuration at
  activation (or at the last re-seed). The damped pseudo-inverse keeps the
  projector bounded near singularities.
- Gravity is not in the law: libfranka adds it.
- `tau_wall` is the [joint-limit wall](#the-joint-limit-wall-and-speed-brake).

## Stiffness in the tool frame, and the damping

`K` is diagonal in the tool axes of the **equilibrium** orientation and is
rotated into the base frame every cycle:

```
K_pos = R_d diag(k_pos_tool) R_d^T        K_rot = R_d diag(k_rot_tool) R_d^T
D_pos = R_d diag(2 zeta sqrt(k_pos_tool)) R_d^T   (the same for rotation)
```

So stiffness can differ per tool axis: the shipped defaults are soft
laterally (X, Y: 150 N/m) and in roll and pitch (10 Nm/rad) so a part can
self-align under contact, and firmer along tool Z (800 N/m) and in yaw
(20 Nm/rad).

`D = 2 zeta sqrt(K)` is critical damping (at `zeta = 1`) for a **1 kg** mass
on each axis. It is used for two reasons:

- **One knob.** Damping follows stiffness, so changing `K` keeps the damping
  ratio roughly where it was. Tuning is one stiffness vector plus `zeta`.
- **It errs on the soft side.** The FR3's apparent Cartesian mass is a few
  kilograms, so the true damping ratio at `zeta = 1` is lower, about 0.5 to
  0.7. Joint friction adds damping on top. In practice a step of the
  equilibrium settles with at most one small overshoot.

Do not raise `zeta` above 1.0. More damping is not more safety here:
rotational damping `2 zeta sqrt(k_rot)` acts through the wrist's small
inertia with the delay of the loop, and at high `k_rot` with `zeta = 1` it
drove a ~40 Hz wrist oscillation once tracking moved the wrist. That is why
the tracking node caps `zeta` at 0.5 while it runs (see
[tracking.md](tracking.md#gains-during-a-session)). The gain presets refuse
`zeta` above 1.0.

## The equilibrium and its slew limits

- **Seeded at the arm.** On activation the equilibrium is set to the
  measured end-effector pose, so the first cycle commands zero spring force.
  The torque rate limiter then ramps in from zero.
- **Setpoints are targets, not jumps.** A message on `~/equilibrium_pose`
  sets a *target*. Each cycle the equilibrium moves toward it by at most
  `setpoint_slew_mps * dt` in a straight line and `setpoint_slew_rps * dt` of
  rotation (slerp). A large step on the topic becomes a bounded ramp.
- **The slew is the speed limit.** In free space the arm keeps up with the
  equilibrium, so `setpoint_slew_mps` sets how fast the arm moves, not the
  stiffness. Raise the slew to move faster.
- **Stopping needs a message.** If publishers simply go silent, the
  equilibrium keeps slewing toward the last target. To hold, publish the
  measured pose once. The tracking node and grip node do this on every hold
  and stop.
- **Last writer wins.** The subscription hands setpoints to the 1 kHz loop
  through `TargetHandoff`: the subscriber thread writes under a mutex, the
  loop only `try_lock`s and never waits. There is no arbitration between
  publishers. Three nodes can publish (the panel's SETPOINT, `tracking_node`
  and `grip_node`); the panel keeps them apart by refusing to start one while
  another runs.

## Caps that hold whatever the gains

| Cap | Default | What it bounds |
|---|---|---|
| `max_force_n` | 30 N | Norm of the commanded force, damping included. With the default gains it engages beyond about 37 mm of tool-Z error (30 N / 800 N/m) or 200 mm laterally. Without it, a blocked arm with a slewing equilibrium winds the spring force up without limit. |
| `max_torque_nm` | 10 Nm | Norm of the commanded moment, damping included. |
| `tau_max_nm` | FR3 limits: 87 Nm (J1-J4), 12 Nm (J5-J7) | Per-joint torque. Configure fails above the FR3 datasheet values. |
| `tau_rate_limit` | 1.0 Nm per 1 ms cycle | Torque rate, the FCI limit on torque discontinuities. Configure fails above 1.0. |

The caps bound how hard the arm can push. They do not make an unstable gain
set stable: `zeta` 0 is an undamped spring and a negative value injects
energy, which is why gains outside their range are rejected, never clamped.

Keep `max_force_n` well below the robot's own collision reflex threshold.
The panel's PRE-FLIGHT sets that to 40 N / 40 Nm, and the reflex watches the
*estimated* external wrench, whose bias at rest eats into the margin.

## Malformed setpoints and bad numbers

- A setpoint with a non-finite position, or a quaternion whose norm is
  outside 0.9 to 1.1, is dropped with a throttled warning. An unset
  orientation (all zeros) is the usual cause; normalising it would give NaN
  and zero the task torque mid-hold. Valid quaternions are normalised.
- A non-finite joint position, velocity or Coriolis vector commands zero
  torque for that cycle (through the rate limiter), so one bad sample never
  lands in the rate limiter's memory.
- A non-finite total torque is replaced by zero.

## Float mode

With `float_mode: true` the controller commands Coriolis plus the joint-limit
wall only. libfranka compensates gravity, so the arm floats and can be moved
by hand. Use it as the first commissioning step: smooth floating proves the
real-time loop and the payload setting.

`float_mode` is live. Switching it **off** re-seeds the equilibrium where the
arm is now and discards every setpoint published before, so going from float
to hold cannot snap the arm back to where it was before it was moved.

## The joint-limit wall and speed brake

Without a wall the only guard at a joint limit is the robot's own reflex,
which stops the arm and, on this stack, takes the driver down. The wall acts
in every mode, float included, and is added before the joint torque clamp and
the rate limiter (so the 12 Nm wrist can still be overpowered by hand, and
the robot's reflex stays the backstop).

- **Wall.** Inside the last `joint_wall_margin_rad` before a limit, a spring
  `joint_wall_k[i] * depth` pushes the joint back out, and damping
  `joint_wall_d[i] * dq` opposes motion in **both** directions (one-way
  damping throws a released joint back out). The damping ramps in over the
  first 0.01 rad of the margin.
- **Speed brake.** The FR3 limits each joint's speed by its distance to the
  end stop: toward a stop the allowed speed is
  `min(v_max, -a + sqrt(c * room))`, with libfranka's published constants.
  A joint moving toward a limit faster than `joint_speed_frac` of that allowed
  speed is braked by `joint_brake_d[i]` times the excess. Far from the limits
  the allowed speed is 2.6 to 5.3 rad/s, so the brake never acts there.
- `joint_wall_margin_rad: 0` turns the wall off; `joint_speed_frac: 0` turns
  the brake off.

The limits default to the FR3 values from `franka_description` and must
match `tracking_joint_lower`/`tracking_joint_upper` in
`fr3_cell/config/fr3_params.yaml`, which the tracking node uses to hold
before a joint reaches the wall.

## Changing parameters

- **Live parameters** (the gains, `nullspace_stiffness`, the slew pair and
  `float_mode`) change while the controller holds the arm. The 1 kHz loop
  adopts them at the start of a cycle.
- **All or nothing.** A parameter set is validated as a whole before anything
  is applied. If any gain in it is out of range, nothing in it is applied.
  Send gain changes with `set_parameters_atomically` (the panel does): plain
  `set_parameters` applies each parameter separately.
- **Configure-time parameters** (`arm_id`, the four ceilings, the joint wall)
  are read at configure only. A live change is rejected with a message. To
  change one, edit the yaml and restart the driver terminal and then the cell
  launch: when the controller is already loaded, the spawner does not
  re-apply the parameter file.

```bash
ros2 param get /cartesian_impedance_stroke_controller k_pos_tool
ros2 param set /cartesian_impedance_stroke_controller setpoint_slew_mps 0.03
```

The cell launch spawns the controller inactive with the shipped yaml. To do
it by hand (the driver's controller manager must find this workspace, see
[installation.md](installation.md)):

```bash
ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
    --inactive --param-file fr3_mating_controllers/config/cartesian_impedance_stroke.yaml
```

Never leave it active alongside `fr3_arm_controller`: both claim the effort
interfaces. The panel swaps the two with `switch_controller` (STRICT).

## Parameters

Ranges are from `GainLimits`, `ConfigLimits` and `WallLimits` in
`impedance_detail.hpp`. An out-of-range value makes configure fail, or a live
set be rejected.

| Parameter | Default | Range | When |
|---|---|---|---|
| `arm_id` | `fr3` | | configure |
| `float_mode` | `false` | | live |
| `k_pos_tool` | `[150, 150, 800]` N/m (tool x, y, z) | each 0 to 3000 | live |
| `k_rot_tool` | `[10, 10, 20]` Nm/rad (tool roll, pitch, yaw) | each 0 to 300 | live |
| `damping_ratio` | `1.0` | 0.1 to 2.0 (keep at or below 1.0) | live |
| `nullspace_stiffness` | `5.0` Nm/rad | 0 to 50 | live |
| `setpoint_slew_mps` | `0.05` m/s | 0.001 to 0.30 | live |
| `setpoint_slew_rps` | `0.5` rad/s | 0.001 to 1.0 | live |
| `max_force_n` | `30.0` N | 1 to 100 | configure |
| `max_torque_nm` | `10.0` Nm | 0.5 to 30 | configure |
| `tau_max_nm` | `[87, 87, 87, 87, 12, 12, 12]` Nm | each 0.1 to the FR3 limit | configure |
| `tau_rate_limit` | `1.0` Nm per cycle | 0.01 to 1.0 | configure |
| `joint_lower` | FR3 lower limits (rad) | finite; `upper - lower` above twice the margin | configure |
| `joint_upper` | FR3 upper limits (rad) | as above | configure |
| `joint_wall_margin_rad` | `0.14` (8 deg) | 0 (off) to 0.35 | configure |
| `joint_wall_k` | `[600, 600, 600, 600, 150, 150, 150]` Nm/rad | each 0 to 2000 | configure |
| `joint_wall_d` | `[20, 20, 20, 20, 3, 3, 1.5]` Nm s/rad | each 0 to 50 | configure |
| `joint_speed_frac` | `0.7` | 0 (off) to 1 | configure |
| `joint_brake_d` | `[120, 120, 100, 100, 6, 6, 3]` Nm s/rad per rad/s | each 0 to 200 | configure |

The panel mirrors `GainLimits` in `fr3_cell/cell/core.py` and refuses an
out-of-range gain before sending it; `fr3_cell/cell/test_cell_pins.py` fails
if the two drift apart, and checks the shipped yaml against `ConfigLimits`.
The tracking node's `tracking_max_force_n` and `tracking_max_torque_nm` must
equal `max_force_n` and `max_torque_nm` (also pinned by that test).

## Tuning procedure

Commission on the real arm in this order, from the panel's torque ladder
(the operator steps are in [operator_guide.md](operator_guide.md)). Change
one value at a time.

1. **Pre-flight.** Set the payload (camera and bracket only; the hand belongs
   to the end-effector configuration in Desk) and the collision thresholds.
   If the estimated external force at rest reads more than a few newtons,
   fix the payload before going on.
2. **Float** (`float_mode: true`). The arm must float smoothly under gentle
   hand pressure (under 10 N). Jitter, kicks or a control success rate under
   about 99 % are a network or real-time problem (run `fr3_preflight`), not
   a gain problem. A slow sag means the payload is wrong.
3. **Hold** (`float_mode: false`). Pressing HOLD must not move the arm. Push
   the TCP: with the defaults it should feel like a soft spring sideways
   (150 N/m: 15 N moves it about 10 cm) and stiffer along tool Z (800 N/m:
   12 N moves it about 1.5 cm). One small overshoot on release is normal;
   ringing for more than a couple of cycles, buzz or drift is not.
4. **Setpoint.** Step the equilibrium 1 to 2 cm **up** first. The arm should
   glide there at the slew limit and settle within a few millimetres.

Then tune:

- **Friction sets a residual.** Joint friction leaves the arm short of the
  equilibrium by about `F_friction / k`. With a breakaway force of up to
  about 6.5 N, that is ~43 mm at 150 N/m and ~4 mm at 1500 N/m. Stiffness is
  what closes it. The tracking node integrates the remaining error away
  itself (see [tracking.md](tracking.md#the-tracking-law)).
- **Lateral stiffness** trades self-alignment against accuracy: too soft and
  a sliding part stalls on friction, too stiff and it fights a chamfer
  instead of sliding in. Adjust by about 50 N/m at a time. Roll and pitch
  likewise.
- **Tool-Z stiffness** sets how hard the arm pushes along the tool axis for a
  given lag (`F = k_z * lag`).
- **Speed** comes from `setpoint_slew_mps`/`rps`, not from stiffness.
- **Damping** stays at 1.0 or below (see above).
- **Nullspace stiffness** holds the elbow posture. Raise it if the arm drifts
  in configuration during a move.

## Gain presets

The panel's preset slider writes one of the sets in
[`fr3_cell/cell/config/gain_presets.yaml`](../fr3_cell/cell/config/gain_presets.yaml)
in one atomic set. `k_xy` goes to `k_pos_tool[0,1]`, `k_z` to
`k_pos_tool[2]`, `k_rp` to `k_rot_tool[0,1]`, `k_yaw` to `k_rot_tool[2]` and
`zeta` to `damping_ratio`.

| Preset | `k_xy` N/m | `k_z` N/m | `k_rp` Nm/rad | `k_yaw` Nm/rad | `zeta` | Note |
|---|---|---|---|---|---|---|
| soft | 100 | 500 | 5 | 10 | 1.0 | placeholder, not tuned on the arm |
| commission | 300 | 800 | 20 | 30 | 1.0 | the default stop |
| firm | 800 | 1200 | 50 | 50 | 0.9 | |
| stiff | 1800 | 1800 | 80 | 80 | 0.8 | |

- **Checked at load.** The panel refuses to start if a preset is outside the
  controller's `GainLimits`, has `zeta` above 1.0, or if any gain is not
  monotonic from soft to stiff.
- **Free space only.** The panel switches presets only while the external
  force and the spring lead are small (`preset_max_force_n` and
  `preset_max_lead_mm` in `fr3_cell/cell/config/settings.yaml`).
- **Where the gains start.** The controller always starts with its yaml gains
  (150/800/10/20, `zeta` 1.0). The panel remembers the last gains you
  applied (in `~/.config/fr3_cell/settings.yaml`) and writes them back at the
  next FLOAT or HOLD.
- **Tracking needs firm gains.** The tracking node refuses gains too soft to
  track, see [tracking.md](tracking.md#gains-during-a-session).

## Tests

```bash
colcon test --packages-select fr3_mating_controllers && colcon test-result --verbose
```

`test/test_impedance_detail.cpp` covers the gain, ceiling and slew
validation, the setpoint handoff (exactly-once delivery, `invalidate`, and
that the 1 kHz side never waits on a held lock), and the joint wall and speed
envelope. The control law itself needs the real arm: the mock hardware in
`franka_ros2` does not integrate torques.
