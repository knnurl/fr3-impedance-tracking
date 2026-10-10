# roscam

Vision for the FR3 cell, with a wrist-mounted RealSense D405: the ArUco
marker pose, the object pose contract (`/object/*`), a depth and colour
estimator of a known part's pose, and hand-eye calibration. The camera is
captured inside the vision process, so no image topics reach the ROS graph.

Everything about how it works, the topics, parameters, part files and
limitations: [docs/perception.md](../docs/perception.md).

## Executables

| Command | What |
|---|---|
| `ros2 run roscam cam_pub --ros-args -p source:=realsense -p filter_frame:=fr3_link0` | Marker pose from in-process capture (`source:=topic` takes a camera driver's colour topic instead). Node `aruco_pose_publisher`. |
| `ros2 run roscam vision_standalone --ros-args -p object_part:=fr3_cell/parts/cube55.yaml` | The same node, plus the depth estimator (shadow mode, `object_source: depth_checked`, acquisition shadow) and frame recording (`record_dir`). |
| `ros2 run roscam handeye_calib --ros-args -p base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp` | Collect hand-eye samples; `--solve <samples.yaml>` solves offline. See [docs/setup_and_calibration.md](../docs/setup_and_calibration.md). |
| `ros2 run roscam connector_pose` | ICP refinement of a connector STL against depth, seeded by the marker. Not used by the cell launch. |
| `python3 -m roscam.rs_capture --seconds 5 [--depth]` | Camera self-test without ROS: frame rate and jitter. |

On the cell, `fr3_cell.launch.py` starts the vision node with the right
parameters (`vision_source:=realsense|standalone|topic`).

Topics out: `/aruco/pose`, `/aruco/pose_raw`, `/aruco/target_pose_raw`,
`/aruco/debug_image` (only while subscribed), `/object/pose`,
`/object/pose_raw`, `/object/pose_quality`.

Main parameters: `marker_id`, `marker_size_m`, `filter_frame`, `source`,
`range_source`, the `capture_*` settings, `object_part`, `object_source`,
`object_shadow`, `object_acquire_shadow`, `object_pose_impl`. See
[docs/perception.md](../docs/perception.md).

## Modules

| Module | What |
|---|---|
| `cam_pub.py` | `ArucoPosePublisher`: detection, IPPE with depth disambiguation, depth range, Kalman filter, target marker B |
| `vision_standalone.py` | One process owns the camera and runs every pipeline on the same frames; `DepthRunner` |
| `object_contract.py` | The `/object/*` topics, sources and source switching |
| `object_pose.py` | `ObjectPoseEstimator`: refine a part pose from depth and colour edges; `acquire()` with no prior; part files |
| `object_shadow.py` | Shadow mode (`DepthShadow`) and the acquisition shadow (`AcquireShadow`) |
| `depth_checked.py` | The `depth_checked` rules: marker veto, acquisition, object filter |
| `pose_kf.py` | 6-DoF pose Kalman filter with an innovation gate |
| `plane_normal.py` | Depth plane fit: IPPE mirror disambiguation, tilt and range from depth |
| `rs_capture.py` | pyrealsense2 capture outside ROS |
| `frame_recorder.py` | Records colour, depth and the published poses to disk |
| `handeye_calib.py`, `connector_pose.py`, `icp.py` | Hand-eye calibration; ICP refinement |

The estimator, `depth_checked`, the filter and the recorder are ROS-free and
tested without a camera.

## Tests

```bash
colcon test --packages-select roscam && colcon test-result --verbose
python3 -m pytest roscam/test        # from the repo root, workspace sourced
```

`test_object_pose.py` scores the estimator on synthetic scenes;
`test_object_pose_cpp.py` holds the C++ estimator to the same answers (it
skips when `object_pose_cpp` is not built); the rest cover the contract,
`depth_checked`, shadow mode, the filter, the plane fit, range from depth,
capture and recording.
