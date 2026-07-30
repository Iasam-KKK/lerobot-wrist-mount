# SO-101 wrist geometry — verified from official CAD

**Every number here was measured, not read off a drawing or a photo.** Method: STEP loaded into
FreeCAD 1.1 via the XML-RPC addon, every cylindrical face enumerated with its radius, axis and
centre. Bolt holes are cylinders, so this finds them exactly rather than by eye.

## Provenance

| | |
|---|---|
| Source repo | `TheRobotStudio/SO-ARM100`, branch `main` |
| Fetched | 2026-07-30 |
| Licence | **Apache 2.0** — permissive, derivatives allowed, commercial use allowed. Attribution + NOTICE required. |
| Local copies | `projects/02-lerobot-wrist-mount/cad/reference/` |
| Measurement script | `projects/02-lerobot-wrist-mount/scripts/` (all re-runnable against the FreeCAD RPC) |

⚠ The repo `CHANGELOG.md` **stops at v0.1.13 (2025-01-27)** and describes only SO-100 work. The
SO-101 files and every camera mount landed *after* the changelog was abandoned, so **there is no
version string for the SO-101 geometry**. The only meaningful revision identifier is the git commit.
Pin it before shipping anything dimensioned.

---

## 1. Wrist interface — `STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step`

Overall: bbox **69.547 × 75.442 × 110.112 mm**, volume 56 619.5 mm³, 237 faces, 1 solid.

**The camera mounting interface is two M3 holes:**

| Feature | Value |
|---|---|
| Hole diameter | **Ø3.200 mm** (M3 clearance, tight — printed part) |
| Centre A | `(-5.000, -20.718, 24.350)` |
| Centre B | `( 3.100, -20.718, 24.350)` |
| **Spacing** | **8.100 mm**, along local X |
| Hole axis | `(0, -1, 0)` — normal to the y = -20.718 face |
| Nut retention | Hex recesses (hexagonal pockets, so they do not appear in a cylinder scan — confirmed from the official README, not measured) |

Other patterns on this part, for context — **not** the camera interface:

- **Servo horn pattern**: 4 × Ø3.200 at z = 5.950, corners `(±4.950, 4.732 / -5.168)` → a
  **9.900 × 9.900 mm** square. Counterbored Ø6.000 from z = -7.536.
- **Motor boss**: 4 × Ø4.000, axis ±Y, at `(-12.600, 9.489, 14.100/34.600)` and
  `(-8.800, -9.925, 14.100/34.600)`.
- Central Ø5.400 bore at `(0, -0.218, 20.446)`, axis -Z.

## 2. Camera interface — `Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/…step`

Overall: bbox **66.097 × 42.597 × 35.000 mm**, volume 10 848.1 mm³, 52 faces, 1 solid.

**Wrist side** — mates to §1 above:

| Feature | Value |
|---|---|
| Hole diameter | **Ø3.300 mm**, counterbored **Ø5.400** |
| Centres | `(-4.000, -8.150, 18.100)` and `(-4.000, -8.150, 10.000)` |
| **Spacing** | **8.100 mm** |

✅ **Interface confirmed.** 8.100 mm on the mount, 8.100 mm on the wrist — exact match, and the
diameters are a correct clearance pair (Ø3.30 through, Ø3.20 into the nut recess). The two STEP files
use different origins, so the match was established by the spacing signature, not by coordinates.

**Camera side** — 32 × 32 mm UVC module:

| Feature | Value |
|---|---|
| Hole diameter | **Ø2.000 mm** (M2, self-tapping into plastic) |
| Pattern | **27.000 × 27.000 mm square** |
| Centres | `(-35.339, -40.014, 4.000)`, `(-35.339, -40.014, 31.000)`, `(-59.809, -28.604, 4.000)`, `(-59.809, -28.604, 31.000)` |
| Face axis | `(-0.422618261741, -0.906307787037, 0)` → **exactly 25.000°** |

That axis is `(-sin 25°, -cos 25°, 0)` to twelve significant figures, so the 25° tilt is a designed
value, not an artefact. Likewise the pattern is exactly 27.000 mm on both axes. Read to 2 dp it
looks like 24.8° — **use 25.000°.**

The 27.000 mm square inside a 32 × 32 mm board is the standard footprint for this whole class of USB
camera module — which is why the official README says "any 32mm x 32mm USB camera module with min
720p / 30fps will likely work".

Other Ø2.000 cylinders in this file sit at z = 3.500 with areas of 5–55 mm²; those are pocket corner
rounds, **not** holes. Do not treat them as a bolt pattern.

**Mount orientation, measured not inferred.** The Ø5.400 counterbore spans X = -4.000…-2.000 and the
Ø3.300 through-hole spans X = -2.000…0.000. The screw head therefore sits on the -X side and the
wrist is on **+X**, so the mount body (which extends to X = -64.097) hangs *away* from the arm. This
single fact fixes the assembly direction; getting it backwards puts the camera on the wrong side of
the wrist.

---

## 2b. Derived: `camera_link` pose in the URDF `gripper_link` frame

⚠ The STEP and the URDF/print STL are **not** the same geometry — bboxes differ
(69.547 × 75.442 × 110.112 vs 65.200 × 52.000 × 105.425 mm). But they **do share an origin**, proven
by probing the mesh along the STEP-measured hole axes: both M3 axes read void for their full depth
while the material between them reads solid. So STEP coordinates transfer into the URDF frame.

Chain: camera M2 pattern → M3 bolt midpoint → wrist mesh frame → `gripper_link` via the upstream
`<visual>` origin `xyz="0 -0.000218214 0.000949706" rpy="-3.14159 0 0"`.

| | |
|---|---|
| `camera_link` xyz | **`0.0025000  0.0640735  0.0027586`** m |
| `camera_link` rpy | **`0  1.134464014  -1.570796327`** (pitch 65.000°, yaw -90.000°) |
| Optical axis | `(0, -0.422618, -0.906308)` — 25.000° off the jaw axis |

**Selection was not by eye.** Four sign combinations exist (the 180° flip about the bolt axis, times
the arbitrary sign of FreeCAD's cylinder `.Axis`, which is *not* an outward normal). All four were
enumerated and scored by where the gripper TCP lands:

| flip | axis sign | TCP off optical axis | verdict |
|---|---|---|---|
| False | +1 | 77.94° | outside FOV |
| False | -1 | 102.06° | outside FOV |
| True | +1 | 171.00° | points backwards |
| **True** | **-1** | **9.00° @ 120.1 mm** | ✅ **inside FOV** |

**Regression check — keep this true.** `gripper_frame_link` (the TCP) must sit ≈9.00° off the
optical axis at ≈120 mm. Confirmed live via tf2: `wrist_cam_link → gripper_frame_link` =
`[0.119, -0.010, 0.016]`, i.e. 9.01° off-axis. If a change breaks this, the camera is aimed at
nothing and the package is worthless.

## 3. Fasteners (from official README, not measured)

- 4 × **M2** — camera to adapter (these ship with the Feetech servos)
- 2 × **M3 × 8 mm** + 2 × **M3 hex nuts** — adapter to wrist. ⚠ **Not** included with the servos.
- Print orientation: as-oriented in the STL, tree supports, **40% infill** to avoid wobble.

## 4. Reproducing this

```bash
# FreeCAD must be open with MCP Addon → Start RPC Server
python projects/02-lerobot-wrist-mount/scripts/measure_step.py \
  "E:\ME\UAV\projects\02-lerobot-wrist-mount\cad\reference\official-step\Wrist_Roll_Follower_SO101.step"
```

## 5. Visual check — `cad/camera_placement_check.FCStd`

Built by `scripts/build_check_assembly.py`. Places the official camera mount onto the official wrist
part using the *same* rotation and translation the URDF encodes, then draws the optical axis. Open it
in FreeCAD to confirm by eye what the numbers claim:

| Object | Colour | Should look like |
|---|---|---|
| `Wrist_Roll_Follower` | grey | the printed wrist part |
| `Camera_Mount_PLACED` | orange | seated flat on the bolt face, hanging outboard |
| `Optical_Axis_130mm` | red line | leaving the sensor, running down past the jaws |
| `Camera_Sensor_Centre` | red ball | the M2 pattern centre |
| `Gripper_TCP` | green ball | the grasp point — **inside the cone, not on the line** |
| `FOV_Cone_vertical` | translucent blue | ±27.27° frame; the green ball must sit inside it |

⚠ **The red line does not touch the green ball, and it must not.** At 8.995° off-axis over 120.08 mm
the TCP is **18.78 mm** to one side, while the ball is only 4 mm in radius — so a line through the
optical axis misses it by roughly 15 mm. That is correct behaviour, not a placement error. The right
question is "is the TCP inside the frame", which is what the cone answers: it lands at **0.33 of the
way from frame centre to frame edge**. An earlier draft of this table said the line should "nearly
hit" the ball; that was wrong and is why the render looks alarming at first.

Independently reproduced there: **TCP 120.08 mm away, 8.995° off the optical axis** — matching the
URDF-derived 120.1 mm / 9.00° by a different code path.

⚠ The script also reports `mount/wrist interference: 410.7 mm³`. **This is a boolean artefact, not a
design clash.** The overlap contains **zero solids** while reporting non-zero volume, and sits in an
8 mm band straddling the mating plane at y = -20.718 — the signature of a boolean across coincident
faces where one body is a tessellated mesh converted with `makeShapeFromMesh`. A real volumetric
interference yields solids. Treat the figure as meaningless rather than alarming.

A definitive clash check is not available: the wrist STEP is a different revision from the printed
part (§2b), so STEP-vs-STEP would compare the wrong bodies. The only conclusive test is a printed
test fit, which needs an SO-101 that is not available here.
