# G1 + TWIST2 head + Dex1/D405 hand — Isaac Sim assets

Self-contained USD/URDF assets for a Unitree G1 fitted with a TWIST2 2-DoF camera head
and Dex1_1 grippers carrying RealSense D405 cameras. Meters, Z-up.

## Contents

### `g1_assembled/` — the full robot (what you load in Isaac Sim)
- `g1_29dof_with_dex1_base_fix1.usd` — Unitree G1 (29-DoF) with:
  - both **Dex1_1 hands** rebuilt with the official **D405 mount** (built-in USB-cam removed),
    all gripper meshes matte black, wrist cameras `left_D405` / `right_D405`.
  - the **TWIST2 2-DoF head** mounted on `head_link` (referenced from `twist2_head.usda`),
    merged into the single robot articulation.
  - Everything is one articulation (root `root_joint`). Meshes are inline except the head,
    which references `./twist2_head.usda` (kept alongside — do not separate them).
- `twist2_head.usda` — the head, referenced by the G1 (also opens standalone).
- `config.yaml`, `configuration/` — original Unitree asset config.

**Head joints to drive:** `joint_yaw` (yaw, centered 23°), `joint_pitch_motor` (pitch, −30°…+90°).
Gears `gear_yaw` / `gear_pitch` couple the rest.

**D405 cameras:** `<root>/<side>_hand_base_link/<side>_D405`, and per-hand optical frames.
Intrinsics: 1280×720, HFOV 87° / VFOV 58°.

### `dex1_d405_hand/` — standalone Dex1_1 hand with the D405 mount
- `dex1_1_d405.urdf` (+ `meshes/`) — for ROS / planners / cuRobo sphere gen.
- `dex1_1_d405.usd` — self-contained USD (D405 camera + intrinsics).
- Built from the official `Dex1_1_Realsense_D405_Camera_Mount_M5010` mount.

### `twist2_head/` — standalone 2-DoF head
- `twist2_head.usda` — yaw (gear-coupled) + pitch (crank/rod linkage), ZED Mini stereo pair.
  Pitch limits −30°…+90°, yaw centered 23°.

## Notes / TODO
- The head's mount transform on the G1 is a **first pass** (registered to the mid360 lidar
  hole via the 4 corner screws, ~4 mm residual). Refine with a camera calibration and update
  the transform on the `twist2_head` reference prim in the G1 USD.
- USD collision meshes still carry the old built-in colliders (invisible); cuRobo uses its own
  sphere set for planning, so regenerate spheres against the new geometry.
