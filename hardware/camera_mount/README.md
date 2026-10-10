# Wrist camera mount — Intel RealSense D405 on the Franka FR3

The 3D-printed bracket that holds the D405 on the FR3 wrist. This is the
**physical realisation of the hand-eye transform**, so it is part of the
cell definition, not an accessory: change this part and
`fr3_cell/calib/handeye.yaml` is invalidated.

## Attribution (CC BY 4.0 — attribution is required)

| | |
|---|---|
| Model | Realsense D405 Camera Mount for Franka Robot Arms |
| Author | **Lele Burger** |
| Source | https://makerworld.com/en/models/1925363-realsense-d405-camera-mount-for-franka-robot-arms |
| Licence | [Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/) |
| Released | 2025-10-26 |
| Modifications | None — `franka_D405_camera_mount.step` is the unmodified original |

Designed for the Franka Emika Panda arm; used here on the FR3, which shares
the flange interface.

## Fasteners

- 2 × M3 × 10 mm socket head
- 1 × M6 × 25 mm socket head

## Print settings (designer profile)

0.2 mm layer, 6 walls, 25% infill, ~1.5 h, 1 plate.

Walls matter more than infill here: the bracket is a cantilever carrying the
camera, and any flex between the flange and the camera shows up directly as
hand-eye error that no amount of calibration removes — it is not a constant
offset if it deflects with arm pose.

## Geometry (parsed from the STEP)

Units are **metres**. Bounding box of the mount + camera envelope:

```
X  -0.08 .. 0.05   span 130 mm
Y  -0.02 .. 0.03   span  50 mm
Z  -0.02 .. 0.06   span  80 mm
```

## Relationship to hand-eye calibration

The bracket fixes the TCP -> camera transform that `roscam handeye_calib`
measures and `fr3_cell/calib/handeye.yaml` stores (the launch file publishes
it as a static TF). With this bracket on the reference cell the calibration
came out at |t| = 78 mm with the camera turned ~90 deg about its optical
axis, and the optical axis 0.37 deg off TCP Z. Recalibrate whenever the
bracket is reprinted or reseated (docs/setup_and_calibration.md).

Using the STEP as a check, not a source:

- the CAD origin is the designer's, NOT the robot flange, so the raw
  bounding box above cannot be compared against the hand-eye `xyz` directly —
  only the magnitude is meaningful, and 78 mm suits this envelope;
- deriving the translation from CAD alone would also need the D405's optical
  centre relative to its mounting holes (Intel datasheet) — the optical
  origin is inside the camera body and is not a feature of this part;
- a printed part has real tolerance, so CAD is a prior, not ground truth.
