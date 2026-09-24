#!/usr/bin/env python3
"""Build articulated Dex1-1 + D405 USDs from dex1_d405_hand/dex1_1_d405.urdf.

Writes three files next to the URDF, all with identical geometry, frames and colliders:
  dex1_1_d405.usd        standalone hand, plain URDF names (base_link, Joint1_1, ...), camera "D405"
  dex1_1_d405_right.usd  names prefixed "right_hand_", camera "right_D405"  (referenced by the G1)
  dex1_1_d405_left.usd   names prefixed "left_hand_",  camera "left_D405"   (referenced by the G1)

Layout (matches the hand links of g1_assembled/, so the G1 can reference link contents 1:1):
  /<root>                        ArticulationRoot, self-collision off, fixed to the world
    /<p>base_link                rigid body; the D405 mount and camera are merged in as geometry
      visuals/{base_link/mesh, d405_mount, d405_body}
      collisions/{base_link/mesh, d405_mount, d405_body}
      <cam>                      Camera at the URDF d405_optical_frame
    /<p>Link1_1 ... /<p>Link2_3  rigid bodies with visuals/<Link>/mesh and collisions/<Link>/mesh
    /joints                      <p>Joint1_1, <p>Joint2_1 (prismatic), fixed joints, root_joint
Colliders are built from the visual meshes: convex decomposition for the base, mount and finger
bodies (a single hull of Link*_2 would cover the pad face), convex hulls for the rest.

Requires: usd-core, trimesh, numpy, scipy.   Usage: python tools/build_dex1_1_usd.py
"""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
from scipy.spatial.transform import Rotation as R

HAND_DIR = Path(__file__).resolve().parent.parent / "dex1_d405_hand"
# Fixed children of base_link merged into it as geometry, with the G1's prim names.
MERGED = {"d405_mount": "d405_mount", "d405_camera": "d405_body"}
OPTICAL_FRAME = "d405_optical_frame"
DECOMPOSE = {"base_link", "d405_mount", "Link1_2", "Link2_2"}
BODY_COLOR, PAD_COLOR = (0.015, 0.015, 0.015), (0.965, 0.95, 0.95)  # G1 material_CAD1EE / 4C4C4C
PAD_LINKS = {"Link1_3", "Link2_3"}
DRIVE = dict(stiffness=100.0, damping=1.0, max_force=200.0)  # same gains as the G1 hand joints
# D405 intrinsics as authored on the G1 cameras (HFOV 87 deg, 1280x720).
CAMERA = dict(focal_length=1.88, horizontal_aperture=3.5681069, vertical_aperture=2.0842021, clip=(0.02, 10.0))


def T(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    m[:3, 3] = xyz
    return m


def parse_urdf(path):
    root = ET.parse(path).getroot()
    vec = lambda s, d: [float(v) for v in s.split()] if s else d
    links = {}
    for l in root.findall("link"):
        vis, col, mass = l.find("visual/geometry/mesh"), l.find("collision/geometry/mesh"), l.find("inertial/mass")
        links[l.get("name")] = dict(
            visual=vis.get("filename") if vis is not None else None,
            collision=col.get("filename") if col is not None else None,
            mass=float(mass.get("value")) if mass is not None else 0.0,
        )
    joints = {}
    for j in root.findall("joint"):
        o, lim = j.find("origin"), j.find("limit")
        joints[j.get("name")] = dict(
            type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            T=T(vec(o.get("xyz") if o is not None else None, [0, 0, 0]), vec(o.get("rpy") if o is not None else None, [0, 0, 0])),
            axis=vec(j.find("axis").get("xyz") if j.find("axis") is not None else None, [1, 0, 0]),
            lower=float(lim.get("lower")) if lim is not None else None,
            upper=float(lim.get("upper")) if lim is not None else None,
        )
    return links, joints


def quatf(Rm):
    x, y, z, w = R.from_matrix(Rm).as_quat()
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def matrix(M):
    return Gf.Matrix4d(np.asarray(M).T.tolist())  # USD uses the row-vector convention


def build(links, joints, mesh_root, out, prefix, cam_name, root_name, mimic):
    pose = {"base_link": np.eye(4)}  # link poses in base_link at q = 0
    pending = dict(joints)
    while pending:
        for n, j in list(pending.items()):
            if j["parent"] in pose:
                pose[j["child"]] = pose[j["parent"]] @ j["T"]
                del pending[n]

    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = f"/{root_name}"
    rp = UsdGeom.Xform.Define(stage, root).GetPrim()
    stage.SetDefaultPrim(rp)
    UsdPhysics.ArticulationRootAPI.Apply(rp)
    rp.AddAppliedSchema("PhysxArticulationAPI")
    rp.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)

    cache = {}

    def mesh(path, filename, M=None, color=BODY_COLOR, collider=None):
        f = mesh_root / filename
        tm = cache.setdefault(f, trimesh.load(f))
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(tm.vertices.astype(np.float32)))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tm.faces)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(tm.faces.flatten().tolist()))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*tm.bounds[0]), Gf.Vec3f(*tm.bounds[1])]))
        if M is not None:
            m.AddTransformOp().Set(matrix(M))
        if collider:
            m.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(collider)
        else:
            m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    bodies = [l for l in links if l not in MERGED and l != OPTICAL_FRAME]
    for link in bodies:
        path = f"{root}/{prefix}{link}"
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTransformOp().Set(matrix(pose[link]))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        mass = links[link]["mass"] + (sum(links[m]["mass"] for m in MERGED) if link == "base_link" else 0.0)
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
        color = PAD_COLOR if link in PAD_LINKS else BODY_COLOR
        approx = "convexDecomposition" if link in DECOMPOSE else "convexHull"
        mesh(f"{path}/visuals/{link}/mesh", links[link]["visual"], color=color)
        mesh(f"{path}/collisions/{link}/mesh", links[link]["visual"], collider=approx)
    base = f"{root}/{prefix}base_link"
    for link, name in MERGED.items():
        approx = "convexDecomposition" if link in DECOMPOSE else "convexHull"
        mesh(f"{base}/visuals/{name}", links[link]["visual"], M=pose[link])
        mesh(f"{base}/collisions/{name}", links[link]["visual"], M=pose[link], collider=approx)

    # D405: ROS optical frame (+Z forward, +Y down) -> USD camera (looks down -Z, +Y up)
    cam = UsdGeom.Camera.Define(stage, f"{base}/{cam_name}")
    cam.AddTransformOp().Set(matrix(pose[OPTICAL_FRAME] @ np.diag([1, -1, -1, 1])))
    cam.CreateFocalLengthAttr(CAMERA["focal_length"])
    cam.CreateHorizontalApertureAttr(CAMERA["horizontal_aperture"])
    cam.CreateVerticalApertureAttr(CAMERA["vertical_aperture"])
    cam.CreateClippingRangeAttr(Gf.Vec2f(*CAMERA["clip"]))

    fj = UsdPhysics.FixedJoint.Define(stage, f"{root}/joints/root_joint")  # fixed base, like the G1 hands' wrist
    fj.CreateBody1Rel().SetTargets([base])
    for name, j in joints.items():
        if j["child"] in MERGED or j["child"] == OPTICAL_FRAME:
            continue
        path = f"{root}/joints/{prefix}{name}"
        prismatic = j["type"] == "prismatic"
        jt = UsdPhysics.PrismaticJoint.Define(stage, path) if prismatic else UsdPhysics.FixedJoint.Define(stage, path)
        jt.CreateBody0Rel().SetTargets([f"{root}/{prefix}{j['parent']}"])
        jt.CreateBody1Rel().SetTargets([f"{root}/{prefix}{j['child']}"])
        # USD prismatic axes are +X/+Y/+Z; a URDF axis of -X becomes +X with both frames turned 180 deg about Z
        flip = np.eye(3)
        if prismatic:
            ax = np.array(j["axis"])
            assert np.count_nonzero(ax) == 1 and abs(ax[0]) == 1, f"{name}: only +/-X prismatic axes handled"
            if ax[0] < 0:
                flip = R.from_euler("z", math.pi).as_matrix()
        jt.CreateLocalPos0Attr(Gf.Vec3f(*map(float, j["T"][:3, 3])))
        jt.CreateLocalRot0Attr(quatf(j["T"][:3, :3] @ flip))
        jt.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        jt.CreateLocalRot1Attr(quatf(flip))
        if prismatic:
            jt.CreateAxisAttr("X")
            jt.CreateLowerLimitAttr(j["lower"])
            jt.CreateUpperLimitAttr(j["upper"])
            d = UsdPhysics.DriveAPI.Apply(jt.GetPrim(), "linear")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(DRIVE["stiffness"])
            d.CreateDampingAttr(DRIVE["damping"])
            d.CreateMaxForceAttr(DRIVE["max_force"])
            d.CreateTargetPositionAttr(0.0)

    if mimic:  # single-motor gripper: Joint2_1 follows Joint1_1 (PhysX: q + gearing * q_ref + offset = 0)
        m = stage.GetPrimAtPath(f"{root}/joints/{prefix}Joint2_1")
        m.AddAppliedSchema("PhysxMimicJointAPI:transX")
        m.CreateRelationship("physxMimicJoint:transX:referenceJoint").SetTargets([f"{root}/joints/{prefix}Joint1_1"])
        m.CreateAttribute("physxMimicJoint:transX:referenceJointAxis", Sdf.ValueTypeNames.Token).Set("transX")
        m.CreateAttribute("physxMimicJoint:transX:gearing", Sdf.ValueTypeNames.Float).Set(-1.0)
        m.CreateAttribute("physxMimicJoint:transX:offset", Sdf.ValueTypeNames.Float).Set(0.0)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=HAND_DIR / "dex1_1_d405.urdf")
    ap.add_argument("--out-dir", type=Path, default=HAND_DIR)
    ap.add_argument("--no-mimic", action="store_true", help="Leave Joint2_1 independent of Joint1_1.")
    a = ap.parse_args()
    links, joints = parse_urdf(a.urdf)
    for suffix, prefix, cam in (("", "", "D405"), ("_right", "right_hand_", "right_D405"), ("_left", "left_hand_", "left_D405")):
        out = a.out_dir / f"dex1_1_d405{suffix}.usd"
        out.unlink(missing_ok=True)
        build(links, joints, a.urdf.parent, out, prefix, cam, f"dex1_1_d405{suffix}", not a.no_mimic)


if __name__ == "__main__":
    main()
