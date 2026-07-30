# SO-101 Wrist Camera — ROS 2 & Gazebo Integration

**The SO-101 / LeRobot ecosystem has six 3D-printable wrist camera mounts. It has zero ROS 2
integration for any of them.** This repository is that missing half: a URDF/xacro description with a
correctly-derived `camera_link`, a Gazebo Harmonic camera sensor, `ros2_control`, and a one-command
simulation.

```bash
ros2 launch so101_wrist_cam_description sim.launch.py demo:=true
```

Gazebo opens with the arm on a pick bench, the arm runs a scripted sweep, and the wrist camera
publishes to `/wrist_cam/image_raw`.

---

## Why this exists

The upstream [`TheRobotStudio/SO-ARM100`](https://github.com/TheRobotStudio/SO-ARM100) repository
ships excellent hardware and, for simulation, exactly three URDF files. Searching the SO-101 URDF for
`camera` or `sensor` returns **zero matches** — no camera link, no optical frame, no Gazebo sensor,
no launch file, no `ros2_control` configuration.

So six people solved printing the mount. Nobody wired one into ROS 2. That is the gap this fills.

## The camera pose is measured, not estimated

This is the part worth reviewing. Every number came from the official CAD, measured in FreeCAD by
enumerating cylindrical faces — bolt holes *are* cylinders, so that locates them exactly rather than
by eye. Full derivation, including the two approaches that were wrong first, is in
[`docs/VERIFIED-GEOMETRY.md`](docs/VERIFIED-GEOMETRY.md).

| Quantity | Value | Source |
|---|---|---|
| Wrist bolt pattern | 2 × M3, **8.100 mm** apart | Ø3.200 holes, `Wrist_Roll_Follower_SO101.step` |
| Camera pattern | **27.000 × 27.000 mm**, M2 | Ø2.000 holes, official hex-nut mount |
| Mount tilt | **exactly 25.000°** | face axis = `(−sin 25°, −cos 25°, 0)` to 12 s.f. |
| `camera_link` xyz | `0.0025000 0.0640735 0.0027586` m | transform chain via the URDF `<visual>` origin |
| `camera_link` rpy | `0 1.134464014 −1.570796327` | pitch 65.000°, yaw −90.000° |

The mount can bolt on four ways once you account for the 180° flip and the fact that FreeCAD's
cylinder axis sign is arbitrary rather than an outward normal. All four were enumerated and scored by
where the gripper TCP lands — three put it 78°, 102° and 171° off-axis. The surviving one puts it
**9.00° off-axis at 120.1 mm**, inside the ±27.3° vertical field of view. Confirmed live over tf2,
and independently reproduced by a second code path in
[`scripts/build_check_assembly.py`](scripts/build_check_assembly.py), which also writes a FreeCAD
document so the placement can be inspected visually.

## Contents

| Path | What |
|---|---|
| `ros2/so101_wrist_cam_description/` | The ROS 2 package — [its README has the detail](ros2/so101_wrist_cam_description/README.md) |
| `docs/VERIFIED-GEOMETRY.md` | Every measurement, its provenance, and the failed attempts |
| `scripts/` | Re-runnable FreeCAD measurement and verification |
| `cad/reference/` | Official upstream CAD, pinned to one commit |

## Requirements

ROS 2 **Jazzy** · Gazebo **Harmonic** · Ubuntu 24.04

```bash
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
                    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control
```

## Honest limitations

- **Not validated against physical hardware.** No SO-101 was available. Geometry is verified against
  official CAD only; it has never been checked against a real arm.
- **Horizontal FOV is an assumption.** 1.204277 rad (69°) is typical for a 32 × 32 UVC module but is
  not in the CAD and no datasheet was consulted. It is a launch argument for that reason.
- **Camera intrinsics are Gazebo's ideal pinhole** — no distortion, no calibration.
- **Frame rate varies** — 9–14 Hz headless under WSL2 against 30 requested, down to ~2.7 Hz with the
  Gazebo GUI and RViz both running. Host rendering throughput, not a model error.

## Licence

Apache-2.0, matching upstream. See [`LICENSE`](LICENSE) and
[`ros2/so101_wrist_cam_description/NOTICE`](ros2/so101_wrist_cam_description/NOTICE).

The URDF and meshes are derived from `TheRobotStudio/SO-ARM100` at commit
`fda892cba81032c46c40976a48c9ceadbf40a9ca`; the only change is rewriting relative mesh paths to
`package://` URIs. The printable mount modelled here is the work of Conor McGartholl and Philip Fung.
**This does not replace it — print theirs.** This supplies the simulation layer it never had.
