"""Read sphere prims under each link of a hand USD and write cuRobo/BODex collision_spheres YAML (link frames).

Links with no sphere prims in the USD keep their spheres from --fallback.
Usage: python tools/usd_spheres_to_yaml.py spheres.usd dex1_d405_hand/dex1_1_d405_spheres.yml [--fallback old.yml]
Sphere prims may sit anywhere below a link prim (e.g. <link>/collision_spheres/sNN); scale is applied to the radius.
"""
import argparse, numpy as np, yaml
from pxr import Usd, UsdGeom

ap = argparse.ArgumentParser()
ap.add_argument("usd"); ap.add_argument("out"); ap.add_argument("--fallback")
ap.add_argument("--root", default="/dex1_1_d405")
a = ap.parse_args()
ORDER = ["base_link", "d405_mount", "d405_camera", "Link1_1", "Link1_2", "Link1_3", "Link2_1", "Link2_2", "Link2_3"]
fallback = yaml.safe_load(open(a.fallback))["collision_spheres"] if a.fallback else {}
st = Usd.Stage.Open(a.usd); xc = UsdGeom.XformCache(0)
out, src = {}, {}
for link in ORDER:
    lp = st.GetPrimAtPath(f"{a.root}/{link}")
    inv = np.linalg.inv(np.array(xc.GetLocalToWorldTransform(lp)).T)
    sph = []
    for p in Usd.PrimRange(lp):
        if not p.IsA(UsdGeom.Sphere): continue
        rel = inv @ np.array(xc.GetLocalToWorldTransform(p)).T
        r = UsdGeom.Sphere(p).GetRadiusAttr().Get() * np.linalg.norm(rel[:3, :3], axis=0).max()
        sph.append({"center": [round(float(v), 4) for v in rel[:3, 3]], "radius": round(float(r), 4)})
    out[link], src[link] = (sph, "usd") if sph else (fallback.get(link, []), "fallback")
with open(a.out, "w") as f:
    f.write("collision_spheres:\n")
    for link in ORDER:
        f.write(f"  {link}:  # {src[link]}, {len(out[link])} spheres\n")
        for s in out[link]:
            c = s["center"]; f.write(f'    - "center": [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]\n      "radius": {s["radius"]:.4f}\n')
for link in ORDER: print(f"{link:12s} {src[link]:8s} {len(out[link])}")
