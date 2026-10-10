# Setup and calibration

One-time setup of a cell: the PC, the network and DDS, the preflight check,
Desk, the camera and its mount, hand-eye calibration, the markers and the
part file. Install the software first ([installation.md](installation.md));
day-to-day operation is in [operator_guide.md](operator_guide.md).

## Why the PC setup matters

`franka_ros2` runs the Franka Control Interface (FCI) loop: a UDP round
trip to the robot every **1 ms**. If that deadline slips, the robot stops
with a `communication_constraints_violation` reflex, or libfranka drops the
connection. Most of this page is about keeping that loop on time:

1. **Do not create image traffic.** The vision node captures the D405
   in-process (`vision_source:=realsense`, the default). Only poses (a few
   KB/s) and a debug image capped at 5 Hz, published only while something
   subscribes, enter ROS.
2. **Give the robot its own wired NIC**, with nothing else on that subnet.
3. **Keep DDS off that NIC** (below).
4. **Real-time hygiene**: PREEMPT_RT kernel, RT priority, no RT throttling,
   performance governor, mains power.
5. **Build the controller in Release.**

## Network

- Connect the robot's control box (the FCI port) to a dedicated wired
  interface on the PC. Give the PC a static address on the robot's subnet,
  for example PC `172.16.0.1/24`, robot `172.16.0.2`.
- If your robot is not at `172.16.0.2`, set `FR3_ROBOT_IP` in
  `fr3_cell/scripts/fr3_env.sh` (or export it before launching).
- No Wi-Fi and no shared switch on the robot link. Ping round trips should
  stay under 1 ms; `fr3_preflight` measures them.

## Real time

| Setting | How |
|---|---|
| PREEMPT_RT kernel | Follow Franka's "Setting up the real-time kernel" guide. `uname -v` must contain `PREEMPT_RT`. |
| RT priority | Add `<your user> - rtprio 99` to `/etc/security/limits.conf`, then log in again. `ulimit -r` should print 99. |
| RT throttling off | `sudo sysctl -w kernel.sched_rt_runtime_us=-1`; make it permanent with `echo 'kernel.sched_rt_runtime_us = -1' \| sudo tee /etc/sysctl.d/99-fr3-rt.conf` |
| CPU governor | `sudo cpupower frequency-set -g performance` |
| Power | Laptops on mains. On battery the governor still reads `performance` but the CPU cannot hold its clocks. |

The vision process is pinned to a set of CPUs with the launch argument
`vision_cpus` (default `12-19`, the efficiency cores of the PC this was
developed on). Set it to cores your PC has, away from the ones the control
loop uses, or `vision_cpus:=''` to not pin it.

## DDS isolation (CycloneDDS)

The cell uses CycloneDDS with the profile
[`fr3_cell/config/cyclonedds_fr3.xml`](../fr3_cell/config/cyclonedds_fr3.xml).
`fr3_env.sh` exports it in every terminal:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://<path to>/fr3_cell/config/cyclonedds_fr3.xml
```

Why it is pinned:

- **Interface isolation.** The profile's `<Interfaces>` block lists only the
  loopback interface `lo`. The robot NIC is never listed, so no ROS topic
  can share the FCI link, however heavy. A Cyclone profile without an
  `<Interfaces>` block tunes buffers but isolates nothing.
- **Buffer tuning.** Larger fragments and socket buffers, so bursts do not
  turn into dropped fragments and retransmit storms.
- **Discovery on loopback.** Loopback is not multicast-capable, so
  discovery is unicast to `localhost`. The profile raises
  `MaxAutoParticipantIndex` to 80: the default of 9 is fewer participants
  than `moveit.launch.py` alone starts, and the ones above the cap are
  silently never discovered (`move_group` then never sees `/joint_states`).

Edit the `<Interfaces>` block for your machine (`ip -br link` lists the
names):

- `lo` always stays.
- Add your Wi-Fi interface only if you need Foxglove or rqt from another
  machine, and do not stream images over it.
- **Never add the robot's interface.**

Only one `CYCLONEDDS_URI` can be active per process: use this one for every
terminal of the cell.

## The preflight check

```bash
source src/fr3-impedance-tracking/fr3_cell/scripts/fr3_env.sh
fr3_preflight            # or: fr3_preflight <robot_ip>
```

It is read-only: it changes nothing on the PC or the robot. Its exit code
is the number of FAILs, so `fr3_preflight && ros2 launch ...` starts the
driver only when everything passed. WARNs do not fail it; fix them before
long runs.

| Group | Check | Result if wrong |
|---|---|---|
| Real time | kernel is PREEMPT_RT | FAIL |
| | `rtprio` limit at least 90 | FAIL |
| | CPU governor is `performance` | WARN |
| | on mains power (when the PC reports an AC adapter) | FAIL |
| | RT throttling off (`sched_rt_runtime_us = -1`) | FAIL |
| Robot link | a route to the robot exists | FAIL |
| | the route is not wireless | FAIL |
| | 20 pings: max under 1 ms (OK), under 2 ms (WARN), else FAIL | WARN / FAIL |
| | no open Desk (HTTPS) connections to the robot | WARN |
| DDS | `RMW_IMPLEMENTATION` is `rmw_cyclonedds_cpp` | WARN |
| | `CYCLONEDDS_URI` set and the file exists | FAIL |
| | the profile does not bind the robot's interface | FAIL |
| | the profile has an `<Interfaces>` block | WARN |
| Topics | no point-cloud topic is live (needs a running graph) | FAIL |
| | `/aruco/debug_image` exists: fine locally, not over Wi-Fi | WARN |
| Workspace | `fr3_mating_controllers` resolves to this workspace | FAIL |
| | `roscam` is a symlink install | WARN |
| | `pyrealsense2` is importable | WARN |
| | the RealSense no-autosuspend udev rule is installed | WARN |

## Desk

Before every session:

1. Unlock the joints.
2. Activate FCI.
3. **Close the Desk browser tab.** Its HTTPS connections share the robot
   link; `fr3_preflight` warns while one is open.

Once, for the cell: **create an end-effector profile that includes the
camera and mount**, and make it the active one. On the cell this was
developed on, the profile "Franka Hand with D405" has a mass of 0.83 kg and
a centre of mass at (-5, -5, 32) mm; weigh and estimate your own.

The payload belongs in Desk only. The panel's PRE-FLIGHT sets the extra
FCI payload to zero, so the load is counted once. PRE-FLIGHT also sets the
collision thresholds (contact 20 N, reflex 40 N) and reports the external
force at rest. If that force is above about 5 N, the profile is wrong: fix
it before FLOAT, or the arm will drift and the bias eats into the reflex
margin.

## The camera and its mount

### Mount

The D405 sits on the wrist in a 3D-printed bracket:
[`hardware/camera_mount/franka_D405_camera_mount.step`](../hardware/camera_mount/).
The model is "Realsense D405 Camera Mount for Franka Robot Arms" by **Lele
Burger**, from
[MakerWorld](https://makerworld.com/en/models/1925363-realsense-d405-camera-mount-for-franka-robot-arms),
licensed [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) and
included unmodified (see [NOTICE](../NOTICE)). It is designed for the Panda
and fits the FR3, which has the same flange.

- Fasteners: 2 × M3 × 10 mm and 1 × M6 × 25 mm socket head.
- Print with plenty of walls (the designer's profile: 0.2 mm layers, 6
  walls, 25 % infill). The bracket is a cantilever; any flex shows up as
  hand-eye error that changes with arm pose and no calibration removes.
- The bracket is the physical hand-eye transform. **Reprint or reseat it,
  and you must calibrate again.** In this mount the camera sits rotated
  about 90° about its optical axis relative to the TCP.

### USB

Plug the D405 into a USB 3 port. Install the udev rule that stops the
kernel autosuspending it; otherwise an idle camera re-enumerates and the
running vision node waits for frames forever:

```bash
sudo cp fr3_cell/setup/99-realsense-no-suspend.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=8086
```

Only one process can own the camera. With the default
`vision_source:=realsense`, do not start a `realsense2_camera` driver or the
RealSense Viewer while the cell runs.

### Capture settings

The launch file captures colour and depth at 640 × 480, 15 fps, with the
D405's "High Accuracy" preset and the SDK spatial filter, and takes the
marker's distance from depth (`range_source:=depth`). These are launch
arguments (`capture_fps`, `capture_width`, `capture_height`,
`capture_preset`, `capture_spatial_filter`, `capture_exposure_us`,
`capture_gain`, `range_source`). Exposure and gain can also be changed live:

```bash
ros2 param set /aruco_pose_publisher capture_exposure_us 4000   # -1 = auto
```

## ArUco markers

| | |
|---|---|
| Dictionary | `DICT_6X6_250` |
| Size | **21 mm** black square (the vision node's `marker_size_m`, 0.021) |
| Tracked part | id **0** (launch argument `marker_id`), centred on the part's top face |
| Place target B | id **1** (launch argument `target_marker_id`, `-1` = off), lying flat on the table |

- Print with scaling disabled, then measure the black square with calipers.
  A 5 % size error is a 5 % range error. The launch file does not expose
  `marker_size_m`; if your print is not 21 mm, add `marker_size_m` (in
  metres) to the vision node's parameters in
  `fr3_cell/launch/fr3_cell.launch.py`.
- Matte paper, flat and rigid.
- The vision node decodes every marker it sees and discards every id other
  than `marker_id` (and `target_marker_id`). A wrong id looks exactly like no
  marker: the camera view shows frames, `/aruco/pose` stays silent.

## Hand-eye calibration

The hand-eye transform is `fr3_hand_tcp` → `camera_color_optical_frame`.
The cell keeps exactly one copy, in
[`fr3_cell/calib/handeye.yaml`](../fr3_cell/calib/handeye.yaml):

- `fr3_cell.launch.py` publishes it as a static TF on every launch, and its
  log line says which values it used;
- the panel uses its rotation for ALIGN and shows its residuals on the
  CALIB chip.

The file in the repo is the calibration of the cell this was developed on.
With the same mount it is a starting point; calibrate your own.

### Collect and solve

1. Fix a marker in the workspace where it cannot move, and bring the cell
   up as usual (terminal 1 and terminal 2).
2. In a third terminal (with `fr3_env.sh` sourced):

   ```bash
   ros2 run roscam handeye_calib --ros-args -p base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp
   ```

   Pass both frames: the tool's defaults are for a different robot. It
   reads `/aruco/pose_raw`, which is in the camera's optical frame and does
   not depend on the current hand-eye TF.
3. Move the arm to 10-20 **diverse** poses with the marker in view: vary
   all three rotations and the distance. In the panel, FLOAT lets you guide
   the arm by hand and HOLD keeps it still. At each pose, with the arm
   still, press **Enter** to record a sample. Samples are saved as you go
   (`handeye_samples.yaml` in the current directory; `-p output:=<file>`
   to change it).
4. Press **s** to solve. The tool runs four solvers (Tsai, Park, Horaud,
   Daniilidis), prints each one's consistency residual, picks the best and
   prints the translation and quaternion. Expect a residual of a few mm
   and a degree or two at most. A large residual means the poses were not diverse enough or
   the marker moved. **q** quits.
5. You can solve again later without ROS:

   ```bash
   ros2 run roscam handeye_calib --solve handeye_samples.yaml
   ```

### Install the result

Edit `fr3_cell/calib/handeye.yaml` and keep every key:

| Key | Set to |
|---|---|
| `parent_frame`, `child_frame` | `fr3_hand_tcp`, `camera_color_optical_frame` (unchanged) |
| `xyz` | the printed translation, metres |
| `quat_xyzw` | the printed quaternion |
| `method`, `poses` | the selected solver, the number of samples |
| `residual_mm`, `residual_deg` | the selected solver's residual |
| `calibrated` | the date, as a quoted string |
| `validated_scatter_mm` | optional: the static-marker scatter from the check below |
| `xyz_aruco_range` | optional: the translation solved from ArUco-only distances, used automatically with `range_source:=aruco`. Delete it after a fresh calibration with depth ranging, unless you solve it too. |

With a symlink install the edit takes effect at the next launch of
terminal 2; no rebuild. To try a candidate without editing the file:

```bash
ros2 launch fr3_cell fr3_cell.launch.py handeye_xyz:="x y z" handeye_quat:="qx qy qz qw"
```

### Check it

With the marker still, move the arm through different viewpoints. The
marker's pose in the base frame (`/aruco/pose`, filtered in `fr3_link0`)
must stay put to within a few millimetres. If it swims as the arm moves,
the calibration is wrong. On the reference cell the scatter across the 21
calibration poses was about 8 mm.

## Part files

The vision node reports the part's pose on `/object/*`. The part is
described by a YAML file in [`fr3_cell/parts/`](../fr3_cell/parts/); the
launch file uses `cube55.yaml`, the 55 mm test cube:

```yaml
name: cube55
mesh:
  box_mm: [55.0, 55.0, 55.0]      # a box, centred on the top face
symmetry:
  axis: z
  order: 4                        # looks the same every 90 deg about Z
T_marker_object:                  # the marker's pose on the part
  xyz_mm: [0.0, 0.0, 0.0]
  rpy_deg: [0.0, 0.0, 0.0]
  measured: false
```

| Key | Meaning |
|---|---|
| `name` | a label; defaults to the file name |
| `mesh.box_mm` | a box of this size (x, y, z in mm). The object frame has its origin at the top-face centre, Z out of the top face. |
| `mesh.stl`, `mesh.scale` | instead of `box_mm`: an STL file next to the part file, and its unit in metres (default `0.001`, i.e. the STL is in mm). Export it with the origin at the point you want to reach and Z out of that face. |
| `symmetry.order` | how many identical poses per turn about Z (1 = none). The depth estimator returns the equivalent pose nearest its prior. |
| `T_marker_object` | the transform from the marker frame to the object frame: offset in mm and roll-pitch-yaw in degrees (applied Z·Y·X). Identity means the marker is taken as centred and square on the top face. With the `marker` source, `/object/*` is the marker pose composed with this transform. |
| `T_marker_object.measured` | a note for people; the code does not read it |

Measure `T_marker_object` with calipers and a square once the sticker is on
the part: it moves the pose the arm acts on.

To use a different part, add its file to `fr3_cell/parts/` and point
`PART_FILE` in `fr3_cell/launch/fr3_cell.launch.py` at it (there is no
launch argument for it).
