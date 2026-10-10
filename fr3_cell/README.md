# fr3_cell

The FR3 cell itself: the launch file that brings up everything except the
Franka driver, the node parameters, the DDS profile, the hand-eye
calibration and part files, the operator panel with its mock cell, the grip
node, bring-up scripts, and the offline vision tools.

- Bring-up and daily use: [docs/operator_guide.md](../docs/operator_guide.md)
- Installation and calibration: [docs/installation.md](../docs/installation.md),
  [docs/setup_and_calibration.md](../docs/setup_and_calibration.md)
- How the nodes connect, DDS isolation, where files go: [docs/architecture.md](../docs/architecture.md)

## Launch

```bash
source fr3_cell/scripts/fr3_env.sh     # every terminal; defines fr3_cell and fr3_preflight
fr3_cell                               # = ros2 launch fr3_cell fr3_cell.launch.py
fr3_cell mock:=true                    # no robot: the mock cell and the panel, DDS domain 88
```

On the real cell it spawns the impedance controller inactive into the
driver's controller manager, publishes the hand-eye TF, and starts the
vision node, `tracking_node`, `grip_node` and the panel. Nothing moves until
a panel button is pressed. Main arguments:

| Argument | Default | |
|---|---|---|
| `mock` | `false` | `true`: the mock cell and the panel only, on an isolated DDS domain |
| `vision_source` | `realsense` | `standalone` adds the depth estimator and frame recording; `topic` uses a separate camera driver (colour only) |
| `marker_id` / `target_marker_id` | `0` / `1` | ArUco ids (`DICT_6X6_250`) of the part's marker and of target B (`-1` = off) |
| `range_source` | `depth` | Marker distance from depth, or `aruco` alone |
| `object_shadow`, `object_acquire_shadow` | `false` | Depth estimator / marker-free acquisition in shadow mode (standalone only) |
| `object_pose_impl` | `cpp` | Estimator implementation, `cpp` or `python` |
| `capture_fps`, `capture_width`, `capture_height` | `15`, `640`, `480` | D405 capture |
| `capture_preset`, `capture_spatial_filter`, `capture_exposure_us`, `capture_gain` | `High Accuracy`, `true`, `-1`, `-1` | Depth and exposure settings |
| `vision_cpus` | `12-19` | CPUs for the vision process (`''` = not pinned); set for your machine |
| `params_file` | `config/fr3_params.yaml` | Node parameters |
| `handeye_xyz`, `handeye_quat` | from `calib/handeye.yaml` | Try a candidate calibration without editing the file |
| `start_panel` | `true` | Run the panel as part of the launch |

## Contents

| Path | What |
|---|---|
| `launch/fr3_cell.launch.py` | Terminal 2 of the cell |
| `config/fr3_params.yaml` | Parameters of `tracking_node` (`tracking_*`, `track_*`) and `grip_node` (`grip_*`), see [docs/tracking.md](../docs/tracking.md#parameters) |
| `config/cyclonedds_fr3.xml` | CycloneDDS profile: the interfaces DDS may use (never the robot's) and buffer sizes. Edit the interface names for your machine |
| `config/realsense_low_bw.yaml` | Camera driver profile for `vision_source:=topic` |
| `calib/handeye.yaml` | The hand-eye calibration (TCP to camera optical frame) and its record |
| `parts/cube55.yaml` | The part file for the 55 mm test cube, see [docs/perception.md](../docs/perception.md#part-files) |
| `cell/` | The operator panel, `grip_node`, the mock cell and their tests: [cell/README.md](cell/README.md) |
| `scripts/fr3_env.sh` | Environment for every terminal: ROS, `franka_ros2`, this workspace, DDS profile, `FR3_ROBOT_IP`, `FR3_LOG_DIR` |
| `scripts/fr3_preflight.sh` | Read-only checks (real-time kernel, the robot link, DDS isolation, camera bandwidth traps, this workspace and the camera); exit code = number of failures |
| `scripts/analyse_trace.py` | Summarise panel (`cell_*`) and tracking (`tracking_*`) logs; the newest under `$FR3_LOG_DIR` by default |
| `sim/tracking_smoke.py` | The real `tracking_node` in a fake cell, DDS domain 87 |
| `vision/` | Offline vision evaluation on recordings, see [docs/perception.md](../docs/perception.md#offline-evaluation-tools) |
| `setup/99-realsense-no-suspend.rules` | udev rule that stops the D405 from USB-autosuspending (install steps in its header) |

## Tests

```bash
colcon test --packages-select fr3_cell && colcon test-result --verbose
python3 -m pytest fr3_cell            # from the repo root, workspace sourced
```

`colcon test` runs the pytest suites (panel logic and view, persistence, the
exit handoff, grip geometry, the panel against the mock cell, the trace
analyser, the vision tools), the tracking smoke test and the linters. No
robot or camera is used; the tests that start ROS nodes run on their own DDS
domains. `run_tests.sh` at the repo root builds and tests every package.
