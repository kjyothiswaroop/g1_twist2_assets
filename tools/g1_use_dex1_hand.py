#!/usr/bin/env python3
"""Point the G1's Dex1-1 hand geometry at dex1_d405_hand/dex1_1_d405_{right,left}.usd.

For each <side>_hand_<link> body in the G1 USD this replaces the inline `visuals`, `collisions`
(and on d405_camera the `<side>_D405` camera) with references to the same prims in the per-side
hand file. The D405 mount and camera links (<side>_hand_d405_mount / _d405_camera), which the
original G1 did not have, are created with their fixed joints, as in the hand file and the URDF. The hand file's materials are referenced as <root>/<side>_hand_Looks and bound to the
visual meshes, so geometry, camera and look all come from the hand builder. Link prims, joints,
drives and masses of the G1 are left as they are, so prim paths, joint names and control
behaviour do not change. Re-running is safe.

Rebuild the hand files first with tools/build_dex1_1_usd.py.
Usage: python tools/g1_use_dex1_hand.py [--g1 g1_assembled/g1_29dof_with_dex1_base_fix1.usd]
"""
import argparse
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

REPO = Path(__file__).resolve().parent.parent
ROOT = "/g1_29dof_with_hand_rev_1_0"
LINKS = ["base_link", "Link1_1", "Link1_2", "Link1_3", "Link2_1", "Link2_2", "Link2_3", "d405_mount", "d405_camera"]
# Links the original G1 lacks -> the hand-file fixed joint that attaches each one
NEW_LINKS = {"d405_mount": "base_to_d405_mount", "d405_camera": "d405_mount_to_camera"}
CAMERA_LINK = "d405_camera"
BODY_MATERIAL, PAD_MATERIAL = "body_anodised", "pad_rubber"  # materials in the hand file's Looks
PAD_LINKS = {"Link1_3", "Link2_3"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--g1", type=Path, default=REPO / "g1_assembled" / "g1_29dof_with_dex1_base_fix1.usd")
    a = ap.parse_args()
    layer = Sdf.Layer.FindOrOpen(str(a.g1))
    g1 = Usd.Stage.Open(layer)

    for side in ("right", "left"):
        hand_file = REPO / "dex1_d405_hand" / f"dex1_1_d405_{side}.usd"
        assert hand_file.exists(), f"missing {hand_file}; run tools/build_dex1_1_usd.py"
        asset = Path("..") / hand_file.relative_to(REPO)  # relative to g1_assembled/
        hand_root = f"/dex1_1_d405_{side}"
        hand = Usd.Stage.Open(str(hand_file))
        base_local = np.array(UsdGeom.Xformable(g1.GetPrimAtPath(f"{ROOT}/{side}_hand_base_link")).GetLocalTransformation()).T
        for link, joint in NEW_LINKS.items():
            add_fixed_link(layer, hand, hand_root, side, link, joint, base_local)
        old_cam = layer.GetPrimAtPath(f"{ROOT}/{side}_hand_base_link/{side}_D405")  # camera used to be on base_link
        if old_cam:
            del layer.GetPrimAtPath(f"{ROOT}/{side}_hand_base_link").nameChildren[f"{side}_D405"]
        for link in LINKS:
            body = layer.GetPrimAtPath(f"{ROOT}/{side}_hand_{link}")
            assert body, f"G1 has no {side}_hand_{link}"
            children = ["visuals", "collisions"] + ([f"{side}_D405"] if link == CAMERA_LINK else [])
            for name in children:
                if name in body.nameChildren:
                    del body.nameChildren[name]
                spec = Sdf.PrimSpec(body, name, Sdf.SpecifierDef, "Camera" if name.endswith("D405") else "Xform")
                spec.referenceList.Prepend(Sdf.Reference(str(asset), f"{hand_root}/{side}_hand_{link}/{name}"))
        looks = f"{ROOT}/{side}_hand_Looks"
        if layer.GetPrimAtPath(looks):
            del layer.GetPrimAtPath(ROOT).nameChildren[f"{side}_hand_Looks"]
        spec = Sdf.PrimSpec(layer.GetPrimAtPath(ROOT), f"{side}_hand_Looks", Sdf.SpecifierDef, "Scope")
        spec.referenceList.Prepend(Sdf.Reference(str(asset), f"{hand_root}/Looks"))
        # Material overrides need the composed prims, so they are authored in a second pass below.

    layer.Save()
    stage = Usd.Stage.Open(str(a.g1))
    for side in ("right", "left"):
        for link in LINKS:
            vis = stage.GetPrimAtPath(f"{ROOT}/{side}_hand_{link}/visuals")
            material = f"{ROOT}/{side}_hand_Looks/" + (PAD_MATERIAL if link in PAD_LINKS else BODY_MATERIAL)
            for p in Usd.PrimRange(vis):
                if p.IsA(UsdGeom.Mesh):
                    over = Sdf.CreatePrimInLayer(layer, p.GetPath())
                    over.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["MaterialBindingAPI"]))
                    rel = Sdf.RelationshipSpec(over, "material:binding", custom=False) if "material:binding" not in over.relationships else over.relationships["material:binding"]
                    rel.targetPathList.explicitItems = [Sdf.Path(material)]
    layer.Save()
    print(f"updated {a.g1}")


def add_fixed_link(layer, hand, hand_root, side, link, joint, base_local):
    """Create <side>_hand_<link> (rigid body at the hand-file pose) and its fixed joint in the G1 layer."""
    hand_link = hand.GetPrimAtPath(f"{hand_root}/{side}_hand_{link}")
    local = base_local @ np.array(UsdGeom.Xformable(hand_link).GetLocalTransformation()).T  # base_link is the hand root
    name = f"{side}_hand_{link}"
    spec = layer.GetPrimAtPath(f"{ROOT}/{name}") or Sdf.PrimSpec(layer.GetPrimAtPath(ROOT), name, Sdf.SpecifierDef, "Xform")
    spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(["PhysicsRigidBodyAPI", "PhysicsMassAPI"]))
    q = Gf.Matrix4d(local.T.tolist()).ExtractRotationQuat()
    values = [
        ("physics:mass", Sdf.ValueTypeNames.Float, hand_link.GetAttribute("physics:mass").Get()),
        ("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*map(float, local[:3, 3]))),
        ("xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(q.GetReal(), q.GetImaginary())),
        ("xformOp:scale", Sdf.ValueTypeNames.Double3, Gf.Vec3d(1, 1, 1)),
        ("xformOpOrder", Sdf.ValueTypeNames.TokenArray, ["xformOp:translate", "xformOp:orient", "xformOp:scale"]),
    ]
    for attr, typ, value in values:
        (spec.attributes.get(attr) or Sdf.AttributeSpec(spec, attr, typ, variability=Sdf.VariabilityUniform if attr == "xformOpOrder" else Sdf.VariabilityVarying)).default = value

    hj = UsdPhysics.FixedJoint(hand.GetPrimAtPath(f"{hand_root}/joints/{side}_hand_{joint}"))
    parent = hj.GetBody0Rel().GetTargets()[0].name  # e.g. right_hand_base_link
    joints = layer.GetPrimAtPath(f"{ROOT}/joints")
    jname = f"{side}_hand_{joint}"
    if jname in joints.nameChildren:
        del joints.nameChildren[jname]
    js = Sdf.PrimSpec(joints, jname, Sdf.SpecifierDef, "PhysicsFixedJoint")
    for rel, target in (("physics:body0", f"{ROOT}/{parent}"), ("physics:body1", f"{ROOT}/{name}")):
        Sdf.RelationshipSpec(js, rel, custom=False).targetPathList.explicitItems = [Sdf.Path(target)]
    for attr, typ in (("physics:localPos0", Sdf.ValueTypeNames.Point3f), ("physics:localRot0", Sdf.ValueTypeNames.Quatf),
                      ("physics:localPos1", Sdf.ValueTypeNames.Point3f), ("physics:localRot1", Sdf.ValueTypeNames.Quatf)):
        Sdf.AttributeSpec(js, attr, typ).default = hj.GetPrim().GetAttribute(attr).Get()


if __name__ == "__main__":
    main()
