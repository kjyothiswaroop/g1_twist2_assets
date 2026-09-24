#!/usr/bin/env python3
"""Point the G1's Dex1-1 hand geometry at dex1_d405_hand/dex1_1_d405_{right,left}.usd.

For each <side>_hand_<link> body in the G1 USD this replaces the inline `visuals`, `collisions`
(and on the base the `<side>_D405` camera) with references to the same prims in the per-side
hand file. The hand file's materials are referenced as <root>/<side>_hand_Looks and bound to the
visual meshes, so geometry, camera and look all come from the hand builder. Link prims, joints,
drives and masses of the G1 are left as they are, so prim paths, joint names and control
behaviour do not change. Re-running is safe.

Rebuild the hand files first with tools/build_dex1_1_usd.py.
Usage: python tools/g1_use_dex1_hand.py [--g1 g1_assembled/g1_29dof_with_dex1_base_fix1.usd]
"""
import argparse
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom

REPO = Path(__file__).resolve().parent.parent
ROOT = "/g1_29dof_with_hand_rev_1_0"
LINKS = ["base_link", "Link1_1", "Link1_2", "Link1_3", "Link2_1", "Link2_2", "Link2_3"]
BODY_MATERIAL, PAD_MATERIAL = "body_anodised", "pad_rubber"  # materials in the hand file's Looks
PAD_LINKS = {"Link1_3", "Link2_3"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--g1", type=Path, default=REPO / "g1_assembled" / "g1_29dof_with_dex1_base_fix1.usd")
    a = ap.parse_args()
    layer = Sdf.Layer.FindOrOpen(str(a.g1))

    for side in ("right", "left"):
        hand_file = REPO / "dex1_d405_hand" / f"dex1_1_d405_{side}.usd"
        assert hand_file.exists(), f"missing {hand_file}; run tools/build_dex1_1_usd.py"
        asset = Path("..") / hand_file.relative_to(REPO)  # relative to g1_assembled/
        hand_root = f"/dex1_1_d405_{side}"
        for link in LINKS:
            body = layer.GetPrimAtPath(f"{ROOT}/{side}_hand_{link}")
            assert body, f"G1 has no {side}_hand_{link}"
            children = ["visuals", "collisions"] + ([f"{side}_D405"] if link == "base_link" else [])
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


if __name__ == "__main__":
    main()
