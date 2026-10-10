# Installation

This page covers what you need, how to build the workspace natively, the
Docker image for the mock panel, and the failures people hit most often.
For the robot-side setup (network, Desk, camera, calibration) see
[setup_and_calibration.md](setup_and_calibration.md).

## Requirements

### For the mock panel and the tests (no robot)

| Item | Requirement |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble (`ros-humble-desktop`) |
| Tools | `python3-vcstool`, `python3-rosdep`, `python3-colcon-common-extensions` |
| Python | the packages in [`requirements.txt`](../requirements.txt): `numpy<2`, `opencv-contrib-python==4.11.0.86`, `PySide6>=6.5,<6.12`, `pyrealsense2>=2.54` |
| Fonts | `fonts-lato` (the panel's UI font; it falls back to DejaVu Sans without it, and the panel's screenshot test is then skipped) |

Or skip all of this and use [Docker](#docker-mock-panel-only).

### Additionally, for a real FR3

| Item | Requirement |
|---|---|
| Kernel | A **PREEMPT_RT** kernel, set up as described in Franka's documentation ("Setting up the real-time kernel"). The FCI loop must answer the robot every 1 ms. |
| libfranka | From apt, `ros-humble-libfranka` (rosdep installs it). Its version **must match the robot's system version**: check Franka's compatibility table before you update either the robot or the package. |
| Network | A **dedicated wired NIC** for the robot, nothing else on that subnet (for example PC `172.16.0.1/24`, robot `172.16.0.2`). Not Wi-Fi, not through a shared switch. |
| Real-time limits | `rtprio` 99 for your user in `/etc/security/limits.conf` (log in again after editing) |
| Camera | Intel RealSense D405 on USB, with the no-autosuspend udev rule installed (below) |
| Power | A laptop must run on mains power, not battery |
| Gripper | The Franka Hand, for GRIP and PLACE |

## Native build, step by step

### 1. Install the build tools

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions python3-pip
sudo rosdep init      # once per machine; skip if it says it is already initialised
rosdep update
source /opt/ros/humble/setup.bash
```

### 2. Create the workspace and fetch the sources

```bash
mkdir -p ~/fr3_ws/src && cd ~/fr3_ws/src
git clone https://github.com/knnurl/fr3-impedance-tracking.git
vcs import < fr3-impedance-tracking/fr3_impedance_tracking.repos
```

`vcs import` clones the pinned source dependencies next to this repo:
`franka_ros2` at `v2.0.4` and `franka_description` at `1.0.1`.

### 3. Install the system dependencies

```bash
cd ~/fr3_ws
PKGS="fr3_cell franka_bringup franka_fr3_moveit_config franka_gripper"
rosdep install -y --ignore-src --from-paths $(colcon list --packages-up-to $PKGS --paths-only)
```

rosdep reads the `package.xml` of this repo's packages and of the
`franka_ros2` packages they and the FR3 bring-up need, and installs the
matching apt packages: `ros-humble-libfranka`, MoveIt, `ros2_control`,
`cv_bridge`, `rmw_cyclonedds_cpp`, Eigen, pybind11 and the rest.
`franka_ros2` also carries Gazebo and SpaceMouse packages; `--packages-up-to`
leaves them (and their dependencies) out.

### 4. Install the Python packages

```bash
pip3 install -r src/fr3-impedance-tracking/requirements.txt
```

These are not available, or not new enough, as Humble debs. `numpy` stays
below 2 because Humble's `cv_bridge` and the apt OpenCV are built against
numpy 1.x (see [Troubleshooting](#troubleshooting)). OpenCV is pinned to
4.11: 4.10 computes contour hulls differently from the system OpenCV that
`object_pose_cpp` links, which the Python/C++ parity tests catch.

### 5. Build

```bash
colcon build --symlink-install --packages-up-to $PKGS --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- **`Release` matters on the real arm.** The controller runs a 1 kHz loop
  with Eigen; an unoptimised build can miss the FCI deadline in HOLD and
  TRACK while FLOAT still looks fine.
- **`--symlink-install`** makes the Python packages and the panel run from
  the source tree, so a Python edit needs no rebuild. Keep using it: a build
  without it over a symlink install fails (see Troubleshooting).
- `object_pose_cpp` is optional at run time: without it the vision node
  logs that it uses the Python estimator instead, with the same results,
  only slower.

### 6. Source and run the mock panel

```bash
source install/setup.bash
ros2 launch fr3_cell fr3_cell.launch.py mock:=true
```

The mock cell and the panel re-exec themselves onto DDS domain 88 over
loopback only, whatever your shell had set, so they cannot meet a real cell.

### 7. Run the tests

```bash
./src/fr3-impedance-tracking/run_tests.sh
```

It builds the five packages, runs gtest, pytest, the tracking smoke test
and the linters, and prints the results. Set `WS=/path/to/ws` if the repo
is not at `<ws>/src/fr3-impedance-tracking`.

### 8. Real arm only: one-time PC setup

```bash
# The D405 must never USB-autosuspend (see the file's header for why)
sudo cp src/fr3-impedance-tracking/fr3_cell/setup/99-realsense-no-suspend.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=8086

# RT throttling off, permanently
echo 'kernel.sched_rt_runtime_us = -1' | sudo tee /etc/sysctl.d/99-fr3-rt.conf
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```

Then edit `fr3_cell/config/cyclonedds_fr3.xml` for your machine and run
the preflight check: both are in
[setup_and_calibration.md](setup_and_calibration.md).

## Docker (mock panel only)

From the repo root:

```bash
docker build -t fr3-impedance-tracking -f docker/Dockerfile .
./docker/run_mock.sh
```

`run_mock.sh` starts the image `fr3-impedance-tracking` and opens the mock
panel on your desktop through X11. Use the container to try the panel and
run the mock; do not drive a real arm from it.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Spawning the impedance controller fails with `Loader for controller ... not found` | Terminal 1's controller manager cannot find `fr3_mating_controllers`. Build the workspace and source it (`fr3_env.sh` does this) **before** launching `moveit.launch.py` in terminal 1; restart terminal 1. `fr3_env.sh` and `fr3_preflight` both print a `[FAIL]` line when the package does not resolve to this workspace. |
| `Failed to configure controller` from the spawner when you relaunch terminal 2 | Expected when the impedance controller is already **active**: the controller manager refuses to reconfigure it and leaves it running untouched. |
| The arm stops with `communication_constraints_violation`, or libfranka reports `Connection reset by peer` | The 1 kHz FCI loop missed its deadline. Run `fr3_preflight` and fix every FAIL. The usual causes: no PREEMPT_RT kernel; RT throttling on (`/proc/sys/kernel/sched_rt_runtime_us` not `-1`); laptop on battery; DDS traffic on the robot NIC (`CYCLONEDDS_URI` not set, or the XML lists the robot interface); a non-Release build; an open Desk browser tab; image or point-cloud topics on the graph. |
| Reflex while in FLOAT or HOLD at rest | Check the payload: the end-effector profile in Desk must include the camera and mount (PRE-FLIGHT sets the FCI payload to zero). PRE-FLIGHT reports the external force at rest; above about 5 N something is wrong. |
| The D405 stops delivering frames: `frame timeout` forever, no error | The kernel USB-autosuspended the camera and it re-enumerated with a new device number. Install `fr3_cell/setup/99-realsense-no-suspend.rules` (step 8), then replug the camera. `fr3_preflight` warns when the rule is missing. |
| `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`, or crashes importing `cv_bridge` | numpy 2 was installed (often pulled in by another pip package). `pip3 install "numpy<2"`. |
| `python3 cannot import pyrealsense2` | `pip3 install -r requirements.txt` again; the default `vision_source:=realsense` needs it. |
| colcon: `existing path cannot be removed: Is a directory` | A build without `--symlink-install` over a symlink install (or the reverse). `rm -rf build/<pkg> install/<pkg>` and build again with `--symlink-install`. |
| `move_group` never gets `/joint_states` ("no current robot state"), no error anywhere | DDS discovery over loopback ran out of participant slots. Keep `MaxAutoParticipantIndex` at 80 in `cyclonedds_fr3.xml` and the `localhost` peer; if you write your own DDS profile, copy that block. |
| The camera view shows frames, but `/aruco/pose` is silent | The marker is not being detected or has the wrong id. The vision node discards every id but `marker_id` (launch argument, default `0`). Check the dictionary (`DICT_6X6_250`) and the print. |
| The panel will not start: another `cell_panel` is running | Only one panel may drive the arm. Close the other one (it may be in another terminal). |
| After a reflex, the panel's RECOVER finds no server | On FR3 a reflex can take down `ros2_control_node`, which hosts error recovery. Restart terminal 1; the panel reloads the impedance controller by itself about 15 s after the controller manager returns. |
