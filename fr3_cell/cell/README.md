# FR3 Cell Control (the panel)

The operator panel for the FR3 cell, in PySide6: ALIGN through MoveIt in
position mode; the impedance ladder (PRE-FLIGHT, FLOAT, HOLD, SETPOINT),
TRACK, and GRIP / PLACE / PLACE AT B with the Franka Hand in torque mode.
It moves nothing on its own: every motion is a button press. STOP NOW is a
software stop with no safety rating; it does not replace the robot's
external stop or enabling device. How to operate it:
[docs/operator_guide.md](../../docs/operator_guide.md).

```bash
source fr3_cell/scripts/fr3_env.sh
fr3_cell                          # the whole cell launch, the panel included
fr3_cell mock:=true               # no robot: the mock cell and the panel, DDS domain 88
python3 fr3_cell/cell/cell.py [--mock]    # the window alone
```

Needs PySide6 (see `requirements.txt`). It runs as the node `cell_panel` and
refuses to start while another `cell_panel` is up, so two panels never drive
one arm. On exit it stops tracking and hands the arm back to
`fr3_arm_controller`.

## What is kept between launches

- **Kept** in `~/.config/fr3_cell/settings.yaml` (`settings_mock.yaml` for the
  mock): the settings drawer, saved on every change, and the last gains you
  applied. The controller starts with its yaml gains, so the panel writes
  yours back at the next FLOAT or HOLD. Delete the file to reset. Taught
  poses are in `poses.yaml` beside it.
- **Reset on every launch**, on purpose: the two speed sliders (motion 20 %,
  TRACK 10 %), FAST (off) and the robot-state gate (on).

Fixed thresholds are in `config/settings.yaml` (PLACEHOLDER = a first guess,
still to tune), the gain presets in `config/gain_presets.yaml`
([docs/controller.md](../../docs/controller.md#gain-presets)).

## GRIP and PLACE

`grip_node.py`, started by the cell launch, runs each sequence on the
impedance controller with the operator's gains:

- **GRIP:** read the cube's pose once from fresh raw detections, open the
  Hand, glide above the cube, descend, grasp, lift.
- **PLACE:** the reverse, back to where the cube was gripped.
- **PLACE AT B:** carry the held cube to target marker B lying flat on the
  table (`target_marker_id`) and set it down in one of B's two slots, 60 mm
  either side of B along its x axis (the pitch is still a placeholder).
  `grip_node` remembers B whenever it is steadily in view.

Every target is checked against the Z floor, the workspace box and the joint
limits before anything moves. A contact force change holds the descent.
`~/stop` holds where the arm is and leaves the Hand as it is. Parameters:
`grip_*` in `fr3_cell/config/fr3_params.yaml`. The pure geometry is
`grip_logic.py`.

## Files

| File | What |
|---|---|
| `cell.py` | Entry point; `LiveBackend` between the window and ROS; the exit handoff |
| `core.py` | Constants, geometry, the shutdown order; no ROS, no Qt |
| `ros_node.py` | `CellNode`: MoveIt moves, controller switches, parameters, the 50 Hz state relay, readback, recovery, heartbeat |
| `logic.py` | Pure decisions: what is enabled and why not, the banner, the next step, the chips, slider and preset maps |
| `actions.py` | The commands, the stops, the recorder (REC) |
| `view.py`, `palette.py`, `choices.py` | The Qt window; colour tokens and text sizes; selector values |
| `persist.py` | The settings drawer, saved and restored |
| `state_relay.py` | The C++ `topic_tools throttle` child that relays the 1 kHz robot state at 50 Hz |
| `grip_node.py`, `grip_logic.py` | GRIP and PLACE |
| `mock_cell.py`, `isolate.py` | The fake cell with fault injection; the re-exec onto DDS domain 88 |
| `mock_smoke.py` | Headless: the real ROS side and commands against the mock cell |
| `visual.py`, `baselines/` | Five scenes rendered offscreen and compared with the baselines |

## Tests

```bash
python3 -m pytest fr3_cell/cell        # from the repo root, workspace sourced
```

- `test_cell_logic.py`, `test_cell_gate.py`: enable rules, banner, presets,
  slider maps; the robot-state gate and PAUSE.
- `test_cell_pins.py`: the numbers the panel shares with the controller,
  `fr3_params.yaml`, the tracking law and the hand-eye file agree.
- `test_cell_view.py`, `test_cell_exit.py`, `test_cell_persist.py`,
  `test_cell_recorder.py`, `test_grip_logic.py`.
- `test_cell_integration.py` runs `visual.py` and `mock_smoke.py`.

Accept an intended change of look with `python3 fr3_cell/cell/visual.py
--update` (the baselines depend on the installed fonts).

## Known gaps

- `/cell_panel/heartbeat` (10 Hz, GUI thread) has no consumer yet: a frozen
  panel does not stop `tracking_node`.
- The stop/release-after-marker-loss options run in the GUI; `tracking_node`
  itself only holds on stale vision.
- `franka_ros2` cannot report the collision thresholds or load in force, so
  the panel infers whether PRE-FLIGHT is still valid.
- On this stack a robot reflex can end `ros2_control_node` (which also serves
  error recovery): restart terminal 1.
