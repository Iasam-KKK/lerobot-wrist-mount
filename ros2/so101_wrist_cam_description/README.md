# so101_wrist_cam_description

**ROS 2 Jazzy + Gazebo Harmonic simulation for the SO-101 wrist camera.**

The SO-101 ecosystem has six 3D-printable wrist camera mounts. It has **zero** ROS 2 integration for
any of them — the upstream repo ships three URDF files and not one contains a `camera`, `sensor`, or
optical frame. This package is that missing half.

```bash
ros2 launch so101_wrist_cam_description sim.launch.py
```

Gazebo Harmonic opens with the arm on a pick bench, RViz shows the live wrist feed, and
`/wrist_cam/image_raw` publishes 640×480 `rgb8` in `wrist_cam_optical_frame`.

---

## Why the camera pose is trustworthy

Nothing here was eyeballed off a photo or a render. Every number was measured from TheRobotStudio's
official CAD in FreeCAD by enumerating cylindrical faces — bolt holes *are* cylinders, so this finds
them exactly. The full derivation, including the checks that failed first, is in
[`../../docs/VERIFIED-GEOMETRY.md`](../../docs/VERIFIED-GEOMETRY.md).

| Quantity | Value | How it was established |
|---|---|---|
| Wrist bolt pattern | 2 × M3, **8.100 mm** apart | Ø3.200 holes in `Wrist_Roll_Follower_SO101.step` |
| Camera pattern | **27.000 × 27.000 mm**, M2 | Ø2.000 holes in the official hex-nut mount |
| Mount tilt | **exactly 25.000°** | face axis = `(-sin 25°, -cos 25°, 0)` to 12 s.f. |
| `camera_link` xyz | `0.0025000 0.0640735 0.0027586` m | transform chain through the URDF `<visual>` origin |
| `camera_link` rpy | `0 1.134464014 -1.570796327` | pitch 65.000°, yaw −90.000° |

The mount can physically bolt on four ways once you account for the 180° flip and the fact that
FreeCAD's cylinder axis sign is arbitrary. All four were enumerated and scored by where the gripper
TCP lands; three put it 78°, 102° and 171° off-axis. The surviving one puts it **9.00° off-axis at
120.1 mm** — inside the ±27.3° vertical FOV. Verified live over tf2, not just on paper.

## Moving the arm

```bash
ros2 launch so101_wrist_cam_description sim.launch.py demo:=true
```

Runs a ~24 s scripted bench sweep — the arm tilts down over the bench, sweeps across the three
objects, descends toward the red cube, rolls the wrist, and returns to neutral, while the gripper
opens/closes/opens. That is the clip worth recording: the wrist feed tracking the scene as the arm
moves is the thing a static URDF cannot show.

Manual control instead:

```bash
ros2 control list_controllers          # arm_controller, gripper_controller, joint_state_broadcaster
ros2 run so101_wrist_cam_description demo_motion.py    # against a running sim
```

**Requires the ros2_control stack, which is not in `ros-jazzy-desktop`:**

```bash
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control
```

Without it, use `control:=false` — a passive arm with a working camera, no controllers.

Controllers are `arm_controller` (the 5 arm joints) and `gripper_controller` (the jaw), both
`JointTrajectoryController` on position interfaces, configured in `config/controllers.yaml`. Limits
in `urdf/so101_control.xacro` are copied from upstream, not invented.

> Upstream also ships legacy ROS 1 `<transmission>` blocks using
> `hardware_interface/PositionJointInterface`. `gz_ros2_control` ignores those and reads
> `<ros2_control>`, so both appear in the expanded URDF. That is expected, not a duplicate.

## Arguments

| Argument | Default | Notes |
|---|---|---|
| `rviz` | `true` | Open RViz with the camera view |
| `gui` | `true` | `false` runs Gazebo headless; images still render |
| `control` | `true` | `false` = passive arm, no ros2_control needed |
| `demo` | `false` | Run the scripted bench sweep |
| `camera_width` / `camera_height` | `640` / `480` | The resolution upstream recommends actually using |
| `camera_fps` | `30` | Requested rate; see the honest note below |
| `camera_tilt` | `25.0` | Degrees off the jaw axis. Change only if you modify the mount |

`display.launch.py` skips Gazebo entirely — RViz plus joint sliders, for checking frames quickly.

## Topics

| Topic | Type |
|---|---|
| `/wrist_cam/image_raw` | `sensor_msgs/msg/Image` |
| `/wrist_cam/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/clock` | `rosgraph_msgs/msg/Clock` |

## Build

```bash
mkdir -p ~/so101_ws/src && cd ~/so101_ws/src
ln -s /path/to/so101_wrist_cam_description .
cd ~/so101_ws && colcon build --symlink-install
source install/setup.bash
```

Requires `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-image`.

## Known limits — read before relying on this

- **Frame rate.** 30 fps is requested; actual depends heavily on what else is rendering:

  | Configuration | Measured |
  |---|---|
  | headless (`gui:=false rviz:=false`) | 9–14 Hz |
  | headless + demo motion | ~11 Hz |
  | Gazebo GUI + RViz + demo | **~2.7 Hz** |

  This is host rendering throughput under WSL2, not a model error, and it varies between identical
  runs. **For recording, run `gui:=true rviz:=false`** — every extra GL consumer costs camera rate.
  Measure on your own hardware before quoting a number to anyone.
- **`/joint_states` publishes at ~450 Hz** — once per 1 ms physics step. `<update_rate>` on
  `gz-sim-joint-state-publisher-system` does **not** throttle it (verified: still 454 Hz when set to
  50). If the TF traffic bothers you, raise `<max_step_size>` in `worlds/pick_bench.sdf`; that trades
  physics fidelity for rate, so it is left at 1 ms by default.
- **`gz_frame_id` logs an SDF warning** — "XML Element[gz_frame_id] ... not defined in SDF, dropping".
  Cosmetic: libsdformat does not recognise the tag but `ros_gz` still reads it, confirmed by
  `/wrist_cam/image_raw` carrying `frame_id: wrist_cam_optical_frame`. Do not "fix" it by removing
  the tag, or the images will come out framed on `wrist_cam_link` instead.
- **No `ros2_control`.** The arm is spawned and posable but not actuated. Joint states are published
  from Gazebo. Adding controllers is the obvious next step.
- **Horizontal FOV is an assumption, not a measurement.** `1.204277 rad` (69°) is typical for this
  class of 32 × 32 UVC module, but it is not in the official CAD and no datasheet was consulted. It
  is a launch argument for exactly this reason — set it from your camera's datasheet.
- **Camera intrinsics are Gazebo's ideal pinhole.** No distortion model, no calibration. For
  sim-to-real work you must calibrate the physical camera.
- **Not validated against physical hardware.** No SO-101 was available. Geometry is verified against
  official CAD; it has never been checked against a real arm.

## Troubleshooting under WSL2

**Gazebo GUI dies at startup** with `D3D12: Removing Device` → `failed to create drisw screen` →
`Failed to create OpenGL context`, taking the whole launch with it (`on_exit_shutdown`):

1. `wsl --shutdown` from PowerShell, then reopen. This has fixed it — the crash was seen on a VM
   whose filesystem had gone read-only, and was not reproducible after a restart.
2. If it persists, run headless and use RViz for visuals — server-side sensor rendering works even
   when the GUI cannot start:
   ```bash
   ros2 launch so101_wrist_cam_description sim.launch.py gui:=false demo:=true
   ```
3. `GALLIUM_DRIVER=d3d12`, `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe` and
   `QT_QPA_PLATFORM=xcb` were all tested against a standalone `gz sim -g`; **all four survived**, so
   this is not a driver-choice problem and swapping renderers is unlikely to help.

**Filesystem suddenly read-only** (`Read-only file system`, `Input/output error` from `mount`):
ext4 is mounted `errors=remount-ro` and tripped. `wsl --shutdown` and reopen. Nothing in this
package is lost — the source lives on the Windows drive and only `build/` and `install/` sit inside
WSL.

**`package not found`** — you did not source the overlay:
```bash
source ~/so101_ws/install/setup.bash
```

## Licence and attribution

Apache-2.0, matching upstream.

`urdf/so101_base.urdf` and everything in `meshes/` are derived from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) at commit
`fda892cba81032c46c40976a48c9ceadbf40a9ca` (2026-02-26). The only change to the URDF is rewriting
relative `assets/` mesh paths to `package://` URIs — no geometry, joint, frame or inertial value was
altered. See `NOTICE`.

The printable mount this models is TheRobotStudio's
`Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module`, credited to Conor McGartholl and Philip
Fung. **This package does not replace it — print theirs.** It supplies the simulation layer it never
had.
