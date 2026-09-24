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
  - Everything is one articulation (root `root_joint`). Meshes are inline except:
    - the head, which references `./twist2_head.usda`;
    - the hand geometry (`<side>_hand_*/visuals`, `collisions`, `<side>_D405`), which references
      `../dex1_d405_hand/dex1_1_d405_{right,left}.usd`.
    Keep the folders together. Hand link prims, joints, drives and masses stay in the G1 file,
    so prim paths and joint names are the same as before.
- `twist2_head.usda` — the head, referenced by the G1 (also opens standalone).
- `config.yaml`, `configuration/` — original Unitree asset config.

**Head joints to drive:** `joint_yaw` (yaw, centered 23°), `joint_pitch_motor` (pitch, −30°…+90°).
Gears `gear_yaw` / `gear_pitch` couple the rest.

**D405 cameras:** `<root>/<side>_hand_base_link/<side>_D405`, and per-hand optical frames.
Intrinsics: 1280×720, HFOV 87° / VFOV 58°.

### `dex1_d405_hand/` — Dex1_1 hand with the D405 mount (single source for the hand)
- `dex1_1_d405.urdf` (+ `meshes/`) — the source of truth for ROS, planners and the USDs below.
- `dex1_1_d405.usd` — articulated standalone hand, URDF names (`base_link`, `Joint1_1`, …),
  camera `base_link/D405`, fixed to the world.
- `dex1_1_d405_right.usd`, `dex1_1_d405_left.usd` — the same hand with `right_hand_` / `left_hand_`
  names and `<side>_D405` cameras; the G1 references their geometry.
- All three have the same frames as the URDF and the G1. `Joint1_1`/`Joint2_1` are prismatic (−0.02…0.0245 m,
  + closes) and `Joint2_1` mimics `Joint1_1` (single motor). The D405 mount and camera are merged into
  `base_link` as geometry. Colliders come from the visual meshes: convex decomposition for the base,
  mount and finger bodies (`Link*_2`), convex hulls for the rest.
- `dex1_1_d405_spheres.yml` — collision spheres for cuRobo/BODex, per URDF link in link frames
  (150 spheres; sphere 0 of `Link1_3`/`Link2_3` is the pad-face contact point).
- Built from the official `Dex1_1_Realsense_D405_Camera_Mount_M5010` mount.

### `tools/` — regenerate the hand assets (needs `usd-core trimesh numpy scipy pyyaml`)
- `build_dex1_1_usd.py` — URDF → the three hand USDs above.
- `g1_use_dex1_hand.py` — point the G1's hand geometry at the per-side hand USDs (re-runnable).
- `usd_spheres_to_yaml.py` — sphere prims placed under each link of a hand USD (e.g. edited in
  Isaac Sim) → `dex1_1_d405_spheres.yml`.

After editing the URDF: run `build_dex1_1_usd.py`; the G1 picks the change up through its references.

### `twist2_head/` — standalone 2-DoF head
- `twist2_head.usda` — yaw (gear-coupled) + pitch (crank/rod linkage), ZED Mini stereo pair.
  Pitch limits −30°…+90°, yaw centered 23°.

## Notes / TODO
- The head's mount transform on the G1 is a **first pass** (registered to the mid360 lidar
  hole via the 4 corner screws, ~4 mm residual). Refine with a camera calibration and update
  the transform on the `twist2_head` reference prim in the G1 USD.
- cuRobo spheres for the G1's hands: take `dex1_d405_hand/dex1_1_d405_spheres.yml` and prefix the
  link names with `left_hand_` / `right_hand_`. `d405_mount`/`d405_camera` spheres are in those links'
  URDF frames; express them in `<side>_hand_base_link` if the G1 URDF has no such links.
