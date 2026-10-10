# Vision tracking: `tracking_node`

`tracking_node` (package `fr3_tracking`) makes the arm follow a part seen by
the wrist camera. It keeps the camera at a fixed standoff from the part by
streaming the [impedance controller](controller.md)'s equilibrium pose at
50 Hz. The controller does the compliant motion; this node decides where the
equilibrium goes, when it may move, and when it must hold or stop.

The node comes up idle and publishes nothing until an operator starts it
from the panel's TRACK button. Every hold publishes the arm's own pose, so the
arm stays where it is rather than coasting.

Source: [`tracking_node.cpp`](../fr3_tracking/src/tracking_node.cpp) (ROS
plumbing) and [`tracking_law.hpp`](../fr3_tracking/include/fr3_tracking/tracking_law.hpp)
(the pure law, unit-tested). Parameters: the `tracking_*` and `track_*` keys
in [`fr3_cell/config/fr3_params.yaml`](../fr3_cell/config/fr3_params.yaml).

## Contents

- [Interfaces](#interfaces)
- [The goal](#the-goal)
- [START and STOP](#start-and-stop)
- [Gains during a session](#gains-during-a-session)
- [The tracking law](#the-tracking-law)
- [What makes it hold, and what makes it stop](#what-makes-it-hold-and-what-makes-it-stop)
- [Status topic](#status-topic)
- [Tracking log](#tracking-log)
- [Parameters](#parameters)
- [Tests](#tests)

## Interfaces

| Name | Type | Direction | Notes |
|---|---|---|---|
| `~/start_tracking` | `std_srvs/Trigger` | server | Start a session. Blocks for up to ~14 s while it checks the controller and applies the gains. |
| `~/stop_tracking` | `std_srvs/Trigger` | server | Stop. Always succeeds; the message says whether the gains were restored. |
| `~/status` | `diagnostic_msgs/DiagnosticStatus` | publisher | Latched (reliable, transient local). See [Status topic](#status-topic). |
| `equilibrium_topic` | `geometry_msgs/PoseStamped` | publisher | The controller's `~/equilibrium_pose`, in `BASE_FRAME_ID`. |
| `pose_topic` | `geometry_msgs/PoseStamped` | subscriber | `/object/pose`: the filtered part pose ([perception.md](perception.md#the-object-pose-contract)). |
| `tracking_raw_pose_topic` | `geometry_msgs/PoseStamped` | subscriber | `/object/pose_raw`: only its arrival time is used, as proof the detector saw the part just now. |
| `tracking_state_topic` | `franka_msgs/FrankaRobotState` | subscriber | The 1 kHz robot state: `o_t_ee`, joint positions, velocities and torques. |
| `/<impedance_controller>/transition_event` | `lifecycle_msgs/TransitionEvent` | subscriber | Ends tracking if the controller leaves ACTIVE. |
| `/<impedance_controller>/get_parameters`, `set_parameters_atomically` | parameter services | client | Reads and writes the controller's gains. |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | client | START refuses unless the impedance controller is active. |
| TF | | listener | `BASE_FRAME_ID` to the camera frame at the image stamp, and the hand-eye transform `EEF_FRAME_ID` to `tracking_camera_frame`. |

The node must be spun by a `MultiThreadedExecutor` (its `main` does this):
START waits on parameter-service replies that only another executor thread
can deliver. STOP has its own callback group, so it is served even while a
START is in progress.

## The goal

TRACK holds the **camera** where the panel's ALIGN leaves it:

- the camera's optical axis anti-parallel to the part's Z axis (looking at
  the face the marker is on),
- the part `tracking_standoff_m` straight ahead,
- the part's X axis at `tracking_inplane_deg` in the image (0 = right,
  +90 = down).

In the part frame, the camera goal is a flip of pi about X, then a turn of
`-inplane` about the new Z, translated `standoff` along Z. The node converts
it to the pose the controller needs:

```
T_base_tcp_goal = T_base_part * T_part_cam_goal * inverse(T_tcp_cam)
T_base_ee_goal  = T_base_tcp_goal * T_tcp_ee
```

- `T_tcp_cam` is the hand-eye calibration, read from TF (`EEF_FRAME_ID`,
  `fr3_hand_tcp`, to `tracking_camera_frame`).
- `T_tcp_ee` is the fixed offset between `fr3_hand_tcp` and the frame the
  controller holds (`franka::Frame::kEndEffector`). It is **measured** at
  START from TF and `o_t_ee` sampled together, and accepted only when three
  consecutive samples agree within 1 mm and 1 mrad. Nothing assumes the two
  frames coincide.
- `T_base_part` is `/object/pose` transformed into `BASE_FRAME_ID` with TF at
  the image's stamp, so a moving eye-in-hand camera is composed with the arm
  pose of the moment the image was taken. If TF at that stamp is missing it
  falls back to the latest transform, with a warning.

With `tracking_inplane_hold: true` (the panel's in-plane "off"), the in-plane
angle is the one the camera sees at START. If the part is not in view at
START (a blind start), it is taken from the first detection.

## START and STOP

The panel first writes the session goal and box to the node with one
`set_parameters_atomically` (`tracking_standoff_m`, `tracking_inplane_deg` or
`tracking_inplane_hold`, `tracking_over_lead_policy`, `tracking_box_*`), then
calls `~/start_tracking`. START, in order:

1. Refuses unless the impedance controller is **active**
   (`list_controllers`).
2. Reads the goal and the workspace box. These are read **only** here, so a
   write during a session cannot move the goal under the arm.
3. Measures `T_tcp_ee` and reads the hand-eye transform (refuses if either
   cannot be established, for example while the arm is moving).
4. Refuses if the robot state is stale, if `float_mode` is on (a floating arm
   must not be gain-stepped), or if it cannot read the gains it would have
   to restore.
5. Snapshots the controller's `k_pos_tool`, `k_rot_tool`, `damping_ratio`,
   `setpoint_slew_mps` and `setpoint_slew_rps`.
6. Scales the deadband and lead caps to the stiffness in force and refuses
   gains that are too soft (see [Gains during a session](#gains-during-a-session)).
7. Publishes the measured pose as the equilibrium and waits
   `tracking_settle_s`, so any equilibrium error left over is gone **before**
   the gain step (a tenfold stiffness raise on 20 mm of leftover error would
   saturate the 30 N cap at once).
8. Applies the session gains in one `set_parameters_atomically`.
9. Opens the tracking log and starts the 50 Hz loop.

Any refusal leaves the node idle with the reason on `~/status` at ERROR.

STOP (from `~/stop_tracking`, or any automatic stop):

1. Stops the loop. It is serialised with the loop, so the last message on the
   wire is the hold, never a leading equilibrium.
2. Publishes the measured pose once, so the controller holds where the arm is.
3. Closes the log.
4. Restores the snapshot with one `set_parameters_atomically`. If the restore
   fails, STOP still succeeds and says that the gains were **not** restored,
   on the service reply and on `~/status`.

A STOP during START wins: START backs out, and if its gain step landed after
the STOP's restore, it restores again. On Ctrl-C a pre-shutdown hook runs the
same stop while the executor can still deliver the controller's reply.

## Gains during a session

**Operator gains (default, `tracking_use_operator_gains: true`).** TRACK keeps
the stiffness the operator applied. Only these are written at START:

- the profile's slew (`track_setpoint_slew_mps` 0.10 m/s,
  `track_setpoint_slew_rps` 0.5 rad/s);
- `damping_ratio`, only if the operator's is above the profile's
  `track_damping_ratio` (0.5): it is capped at 0.5 for the session. Rotational
  damping acting through the wrist's small inertia caused a ~40 Hz buzz when
  tracking moved the wrist at `zeta` 1.0 and high `k_rot`.

Right after START replies, the panel overwrites the slew with its **TRACK
SPEED** slider: `pct * 0.30` m/s and `pct * 1.0` rad/s
(`track_slew_max_mps` and `slew_max_rps` in
`fr3_cell/cell/config/settings.yaml`). Every launch starts at 10 % (30 mm/s);
the FAST switch, live while tracking, sets 33.3 % (100 mm/s). STOP restores
the snapshot taken at START: the operator's stiffness, damping and the slew
that was in force before TRACK.

**Profile gains (`tracking_use_operator_gains: false`).** START applies the
whole `track_*` profile atomically: `[1500, 1500, 1500]` N/m, `[90, 90, 90]`
Nm/rad, `zeta` 0.5 and the profile slew. The profile is named by
`tracking_gain_profile`, so another profile is a set of `<name>_*` keys in the
yaml, not a rebuild.

**Scaling to the stiffness in force.** The deadband and the lead caps in the
yaml are tuned for the profile's stiffness, because they are really
`F_friction / k` and `15 N / k`. At START they are scaled by
`k_profile / k_in_force`, using the largest diagonal entry of each:

```
deadband_m   = tracking_deadband_m   * max(track_k_pos_tool) / max(k_pos_tool in force)
lead_max_m   = tracking_lead_max_m   * max(track_k_pos_tool) / max(k_pos_tool in force)
(and the same for the rotational pair with k_rot)
lead_max     = min(lead_max, tracking_max_lead / 2)
```

START refuses gains too soft to track: when the scaled deadband is more than
half the over-lead cap, friction alone would park the arm that far off the
part. With the shipped yaml that is a largest `k_pos_tool` entry below
250 N/m, or a largest `k_rot_tool` entry below about 4.9 Nm/rad. It also
refuses a stiffness of zero, and any set that fails the checks below.

**Checks on the configuration.** At construction (with the profile gains) and
again at START (with the gains in force), `validate_config` requires:

- the deadband within `[F/k, 3F/k]`, with `F` = 6.5 N or 0.6 Nm, the worst
  measured friction breakaway;
- `k * lead_max <= 15 N`, so the integrator can never add more than 15 N;
- 15 N below `tracking_max_force_n`, and `k_rot * lead_max_rad` below
  `tracking_max_torque_nm`;
- `tracking_max_lead_m <= 0.060` m and `tracking_max_lead_rad <= 0.35` rad;
- a valid over-lead policy, positive `tracking_ki`, `tracking_buzz_stop_nm`
  and lead caps.

A construction-time failure stops the node: change the track stiffness
without moving the deadband and lead caps and the node refuses to start.

## The tracking law

Every `tracking_period_s` (20 ms), while tracking:

1. **Buzz check** first, whatever vision says (see
   [below](#what-makes-it-hold-and-what-makes-it-stop)).
2. **Freshness.** Hold unless the robot state, the filtered pose and a raw
   detection are all fresh and TF is available.
3. **Goal.** Compute the end-effector goal from the newest pose (above).
4. **Goal glide** (`tracking_goal_glide: true`). The camera gives a new goal
   about 15 times a second, but the loop runs at 50 Hz. Published as steps,
   a fast slew would cover each step and then stop until the next frame.
   Instead each new frame starts a glide from wherever the last glide had got
   to (or from the arm, after a hold) to the new goal. The glide lasts the
   interval between the two frames' stamps, clamped to
   `[tracking_period_s, tracking_raw_timeout_s]`. It only moves between goals
   already seen, never past the newest, and costs up to one frame of delay.
5. **Error.** `e = goal - measured` for position, and the rotation vector of
   `q_goal * q_measured^-1`.
6. **Bounded integrator.** The controller cannot know it is stuck; only
   vision can. The node keeps a *lead* that pushes the equilibrium past the
   goal until the measured error closes:
   - `lead += Ki * e * dt`, then the lead's norm is clamped to `lead_max`.
     This clamp is the whole anti-windup: the lead can never ask for more
     than `k * lead_max` (15 N).
   - Position and rotation are integrated and clamped separately.
   - **Deadband:** the lead freezes while the error is under half the
     deadband, so it does not hunt across the friction band.
   - With `tracking_ki` 0.5 /s, a 10 mm error takes the lead to its 10 mm cap
     in about 2 s.
   - The lead survives holds (a brief occlusion does not lose it, and
     reacquiring does not lurch). START and STOP reset it.
7. **Equilibrium** = the goal pushed out by the lead (position plus
   `lead.pos`; orientation turned by `lead.rot`).
8. **Verdict.** The equilibrium is judged twice: first the one the **newest
   frame** asks for, then the glided one. Judging only the glide would let a
   sudden jump in the pose (a misdetection, say) be chased for several ticks
   before it crossed a cap. See the next section for the checks.
9. **Publish** the equilibrium, or hold (publish the measured pose once), or
   stop.
10. **Log** one line and update `~/status`.

## What makes it hold, and what makes it stop

**Hold** means: publish the arm's measured pose once and keep it there, then
resume by itself as soon as the cause clears. The node stays armed.

| Cause | Check |
|---|---|
| Robot state stale | Last robot state older than `tracking_state_timeout_s` (0.1 s). A frozen state against a moving goal would wind the lead up on stale data. |
| Part pose stale | `/object/pose` older than `vision_timeout_s` (0.6 s), by its header stamp (arrival time if the stamp is zero or more than 50 ms in the future). |
| No fresh raw detection | No `/object/pose_raw` within `tracking_raw_timeout_s` (0.25 s). The filtered pose may coast on its prediction through a dropout; predictions never drive motion. |
| No transform | TF for the pose is unavailable (lookups never wait, so one missing transform cannot stall the loop). |
| Not finite | The equilibrium is not a finite pose. |
| Z floor | The equilibrium would go below `tracking_z_floor_m` (0.10 m, absolute in `fr3_link0`). The camera bracket hangs below the flange. |
| Workspace box | The equilibrium would leave `tracking_box_x_m`, `tracking_box_y_m` or rise above `tracking_box_z_max_m`. The panel writes its drawer box before every START. |
| Over-lead, policy `hold` | The equilibrium would sit more than `tracking_max_lead_m` (60 mm) or `tracking_max_lead_rad` (0.26 rad, ~15 deg) from the arm. Resumes once the arm catches up. |
| Joint limit | A joint within `tracking_joint_margin_rad` (0.14 rad) of its end stop would go further in: either the spring from the arm to the equilibrium pulls it there (more than 0.05 Nm, computed as `J^T` of the tool-frame spring with the gains in force) or it is already moving there faster than 0.05 rad/s. It lets go as soon as the part pulls the joint back out, so the operator backs out by moving the part. |

The floor, the box and a non-finite pose hold under every over-lead policy.

**Over-lead policies** (`tracking_over_lead_policy`, the one live parameter;
the panel's dropdown writes it):

| Policy | When the equilibrium would lead the arm beyond the caps |
|---|---|
| `hold` (default) | Hold where the arm is; resume once the equilibrium is back inside the caps. |
| `stop` | End tracking and restore the gains, as `~/stop_tracking` does. |
| `clamp` | Keep following, with the equilibrium cut back to the caps around the arm (position along the straight line, rotation along the shortest path). The floor and box then judge the clamped pose. |

**Stop** means: end the session, hold where the arm is and restore the gains.

| Cause | Check |
|---|---|
| `~/stop_tracking` | The panel's STOP NOW, END TRACK, and its exit handoff. |
| Controller left ACTIVE | A `transition_event` from the impedance controller to any state but active (a RELEASE, a crash, another client's switch), including during START. |
| Buzz | The loudest joint's measured torque above 20 Hz exceeds `tracking_buzz_stop_nm` (3.5 Nm rms). Each joint's torque goes through a 2nd-order Butterworth high-pass at 20 Hz and a 0.1 s running rms, on every 1 kHz sample. Tracking motion stays mostly below 20 Hz; a gain-induced oscillation does not. |
| Over-lead, policy `stop` | As above. |
| Node shutdown | Ctrl-C or SIGTERM. |

## Status topic

`/tracking_node/status` (`diagnostic_msgs/DiagnosticStatus`), latched, about
5 Hz and at once on every change of state or level. `message` is
`"<state>"` or `"<state> - <reason>"`. `level` is OK while tracking or idle,
WARN while holding or stopping (and after a fault stop), ERROR after a START
refusal.

| Key | Value |
|---|---|
| `state` | `idle`, `starting`, `tracking`, `holding` or `stopping` |
| `reason` | why it holds, stopped or refused (empty otherwise) |
| `policy` | the over-lead policy in force |
| `gains` | `operator` or `profile` |
| `pos_err_mm`, `rot_err_deg` | last tick's error to the goal (`nan` when no goal was computed) |
| `lead_mm`, `lead_deg` | the integrator's lead |
| `marker_age_s`, `raw_age_s` | age of the last filtered pose and raw detection (`nan` before the first) |
| `buzz_nm` | the buzz meter's level (Nm rms above 20 Hz, loudest joint) |
| `standoff_m`, `inplane_deg` | the session goal |

```bash
ros2 topic echo /tracking_node/status
```

## Tracking log

When `tracking_log_dir` is set, each session writes
`<tracking_log_dir>/tracking_YYYYmmdd_HHMMSS.jsonl` (local time), one JSON
object per tick. The cell launch sets the directory to
`$FR3_LOG_DIR/YYYY-MM-DD/` when `FR3_LOG_DIR` is set (`fr3_env.sh` sets it).
The node does not create directories. The file is flushed about once a
second and at STOP.

| Field | Meaning |
|---|---|
| `t` | seconds since the session started (monotonic) |
| `stamp` | image stamp of the pose used, s (`null` if none) |
| `goal` | the end-effector goal (after the glide), `[x, y, z, qx, qy, qz, qw]` in the base frame; `null` on a tick that held before computing a goal |
| `meas` | the measured end-effector pose (`o_t_ee`), same layout |
| `pos_err_mm`, `rot_err_deg` | error from `meas` to `goal` (`null` when no goal) |
| `lead_mm`, `lead_deg` | the lead's size after this tick |
| `buzz_nm` | the buzz meter's level |
| `published` | `true` if an equilibrium went out this tick |
| `policy` | the over-lead policy in force |
| `reason` | why the tick held or stopped, or what a clamp cut; empty otherwise |

Summarise logs (the panel's `cell_*.jsonl` traces too) with:

```bash
python3 fr3_cell/scripts/analyse_trace.py              # newest log
python3 fr3_cell/scripts/analyse_trace.py <file.jsonl>
```

## Parameters

Every key below must be present in the parameter file: a missing key stops
the node with its name, never a silent default. "START" means re-read at
every START (the panel writes it first); "live" means it can change during a
session; everything else is read once at startup.

| Parameter | Default | Meaning |
|---|---|---|
| `pose_topic` | `/object/pose` | Filtered part pose. |
| `tracking_raw_pose_topic` | `/object/pose_raw` | Raw detections (arrival time only). |
| `tracking_state_topic` | `/franka_robot_state_broadcaster/robot_state` | Robot state. |
| `BASE_FRAME_ID` | `fr3_link0` | Frame of the equilibrium and the box. |
| `EEF_FRAME_ID` | `fr3_hand_tcp` | The TCP frame the goal is defined at. |
| `impedance_controller` | `cartesian_impedance_stroke_controller` | Controller whose gains it reads and writes. |
| `equilibrium_topic` | `/cartesian_impedance_stroke_controller/equilibrium_pose` | Where it publishes. |
| `vision_timeout_s` | `0.6` s | Maximum age of `/object/pose`. |
| `tracking_raw_timeout_s` | `0.25` s | Maximum age of the last raw detection; also the longest goal glide. |
| `tracking_state_timeout_s` | `0.1` s | Maximum age of the robot state. |
| `tracking_period_s` | `0.02` s | Loop period (50 Hz). |
| `tracking_camera_frame` | `camera_color_optical_frame` | Camera frame of the goal (START). |
| `tracking_standoff_m` | `0.10` m | Camera to part distance (START). |
| `tracking_inplane_deg` | `90.0` deg | Part X in the image, 0 = right, +90 = down (START). |
| `tracking_inplane_hold` | `false` | `true`: keep the in-plane angle seen at START (START). |
| `tracking_ki` | `0.5` 1/s | Integrator gain. |
| `tracking_lead_max_m` | `0.010` m | Lead cap at the profile stiffness (15 N at 1500 N/m). Scaled at START. |
| `tracking_lead_max_rad` | `0.017` rad | Rotational lead cap (1.5 Nm at 90 Nm/rad). Scaled at START. |
| `tracking_deadband_m` | `0.005` m | 6.5 N / 1500 N/m; the lead freezes under half of it. Scaled at START. |
| `tracking_deadband_rad` | `0.007` rad | 0.6 Nm / 90 Nm/rad. Scaled at START. |
| `tracking_max_lead_m` | `0.060` m | Over-lead cap, position (at most 0.060). |
| `tracking_max_lead_rad` | `0.26` rad | Over-lead cap, rotation (at most 0.35). |
| `tracking_over_lead_policy` | `hold` | `hold`, `stop` or `clamp` (live). |
| `tracking_z_floor_m` | `0.10` m | Absolute floor for the equilibrium in the base frame. Equal to the panel's floor. |
| `tracking_box_x_m` | `[0.20, 0.80]` m | Workspace box, base frame (START). |
| `tracking_box_y_m` | `[-0.45, 0.45]` m | (START) |
| `tracking_box_z_max_m` | `0.80` m | Top of the box; must be above the floor (START). |
| `tracking_joint_lower` | FR3 lower limits | rad, 7 entries; must match the controller's `joint_lower`. |
| `tracking_joint_upper` | FR3 upper limits | rad, 7 entries. |
| `tracking_joint_margin_rad` | `0.14` rad | Joint-limit hold margin, 0 to 0.5. |
| `tracking_buzz_stop_nm` | `3.5` Nm | Buzz level that stops tracking. |
| `tracking_goal_glide` | `true` | Glide between camera frames; `false` = steps. |
| `tracking_use_operator_gains` | `true` | Keep the operator's stiffness; `false` = apply the profile. |
| `tracking_gain_profile` | `track` | Prefix of the profile keys below. |
| `tracking_settle_s` | `1.0` s | Settle time before the gain step at START. |
| `tracking_profile_timeout_s` | `2.0` s | Bound on each parameter and controller-manager call. |
| `tracking_max_force_n` | `30.0` N | Must equal the controller's `max_force_n`. |
| `tracking_max_torque_nm` | `10.0` Nm | Must equal the controller's `max_torque_nm`. |
| `tracking_log_dir` | `''` | Log directory; empty = no log. The launch overrides it. |
| `track_k_pos_tool` | `[1500, 1500, 1500]` N/m | Profile stiffness. Isotropic on purpose: an anisotropic tool-frame K deflects the force off the commanded direction when the tool is tilted. |
| `track_k_rot_tool` | `[90, 90, 90]` Nm/rad | Profile rotational stiffness. |
| `track_damping_ratio` | `0.5` | Profile damping; also the cap on the operator's `zeta` during TRACK. |
| `track_setpoint_slew_mps` | `0.10` m/s | Profile slew, written at START (the panel then sets TRACK SPEED). |
| `track_setpoint_slew_rps` | `0.5` rad/s | |

## Tests

- `fr3_tracking/test/test_tracking_law.cpp` (gtest): the goal geometry, the
  integrator and deadband, the over-lead verdicts and policies, the floor and
  box veto, the joint-limit hold, the goal glide, the buzz meter,
  `validate_config`, and closed-loop runs against a simulated arm.
- `fr3_tracking/test/test_mating_geometry.cpp` (gtest): the pose math in
  `mating_geometry.hpp`. The tracking node uses only its `pose_error`.
- `fr3_cell/sim/tracking_smoke.py`: the real `tracking_node` in a fake cell
  (controller manager, controller parameters, a first-order arm, robot state,
  TF and the `/object/*` topics). It drives START, following, an over-cap
  jump, a raw dropout and STOP, and checks each step. It runs on DDS domain
  87 over loopback only, so it cannot reach a live cell.

```bash
colcon test --packages-select fr3_tracking fr3_cell && colcon test-result --verbose
python3 fr3_cell/sim/tracking_smoke.py          # alone; --gdb for a backtrace
```
