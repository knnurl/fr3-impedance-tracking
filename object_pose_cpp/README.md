# object_pose_cpp

The part pose estimator of `roscam/object_pose.py`
(`ObjectPoseEstimator.process`) in C++, as a Python extension module
(pybind11, Eigen, OpenCV). It gives the same answers as the Python
reference, several times faster, so the vision loop can estimate the part's
pose on every camera frame.

How the estimator works: [docs/perception.md](../docs/perception.md#the-depth-and-colour-estimator).

## Use

You do not call it directly. `roscam.object_pose.CppObjectPoseEstimator`
wraps it:

- **Shared model.** The Python class prepares the model (mesh samples,
  silhouette edges) and hands it over, so both versions run on identical
  data.
- **Shared gates.** The quality dict and the gates are assembled from the
  C++ results by the same Python code as the reference.
- **Selection.** The vision node uses it when `object_pose_impl` is `cpp`
  (the launch default).
- **Fallback.** If this package is not built, the node logs it and uses the
  Python estimator instead: the same poses, slower.

```python
from roscam.object_pose import CppObjectPoseEstimator, load_part
est = CppObjectPoseEstimator(load_part('fr3_cell/parts/cube55.yaml'))
T, valid, quality = est.process(depth_m, bgr, K, dist, prior_4x4)
```

The offline tools in `fr3_cell/vision/` take `--impl cpp|python`.

## Files

| Path | What |
|---|---|
| `src/estimator.hpp`, `src/estimator.cpp` | The estimator. Its steps, constants and floating-point order follow the Python function by function |
| `src/kdtree.hpp` | Nearest-neighbour search on the mesh samples |
| `src/bindings.cpp` | The `Estimator` class for Python (`process`, `warm`) |
| `object_pose_cpp/__init__.py` | Exposes `Estimator` |

## Build

Needs Eigen 3, OpenCV (core, imgproc, calib3d) and pybind11.

```bash
colcon build --packages-select object_pose_cpp
source install/setup.bash
python3 -c "from object_pose_cpp import Estimator"
```

## Tests

Parity with the Python reference is tested in `roscam`:

```bash
python3 -m pytest roscam/test/test_object_pose_cpp.py
```

The tests check the same pose (to rounding), validity, reason and quality
numbers on synthetic scenes:

- depth only and with colour edges, and the no-contrast fallback;
- a face broken into islands, an angled view, an occluder, junk input;
- priors out of view or behind the camera;
- the symmetry snap, a non-cube box, and bad input that must not crash.

They skip when this package is not built. `colcon test --packages-select
object_pose_cpp` runs the linters.
