"""Real part geometry from manufacturer CAD (item Industrietechnik, Line 8, 40x40, slot 8).

Switched from Line 5 (20x20) on 2026-11-03: with the 20 mm bracket the G1's Dex1 jaws could not
grasp at the hole (hole ~9 mm above the table, see NOTES.md). The 20x20 files stay in assets/cad/.

assets/cad/item_45281_profile_8_40x40_light_natural_200mm.step
    item 0.0.452.81, Profile 8 40x40 light, natural (clear-anodized aluminium). item's export was the
    full 6 m bar (L=6000); cut once to 200 mm, centred (OpenCascade intersect with a box).
assets/cad/item_67011_bracket_8_40x40_with_cap_free_holes.step
    item 0.0.670.11, Angle Bracket Set 8 40x40 with grey cover cap (die-cast zinc), reduced to the
    bracket + cap with free holes. Made once from item_67011_angle_bracket_set_8_40x40.step (one fused
    solid: bracket, cap, 2 M8 button-head screws, 2 T-slot nuts) with OpenCascade booleans: cut
    everything beyond the outer faces y > 0 / z > 0 (nuts, screw tips, and item's small centring lugs
    -- a simplification), drill both holes (r 4.25 mm through the 5 mm legs, axes z at (0,-20) and
    y at (0,-20)), cut both screw heads (r 7.1 mm, 5 -> 11.6 mm off the inner leg faces). The cap
    cannot be separated in this export and is kept (item ships it with the set); the holes are free
    through the leg into the cap's hollow inside (21 mm deep along the axis).

Meshes keep the CAD coordinate frame (metres): that frame is the object frame of the
6D pose labels. Stable resting poses are computed from the convex hull.
"""
import os
from functools import lru_cache

import numpy as np
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CAD_DIR = os.path.join(ROOT, "assets", "cad")

PARTS = {
    "profile": {"step": "item_45281_profile_8_40x40_light_natural_200mm.step", "item_no": "0.0.452.81",
                "material": "aluminium_anodized"},
    "bracket": {"step": "item_67011_bracket_8_40x40_with_cap_free_holes.step",
                "item_no": "0.0.670.11 (bracket + cap, no fasteners)", "material": "zinc_diecast_ral9006"},
}

# Bracket hole geometry in its CAD frame (m), used by the detector, the fit, the evaluations and the
# grasp: outer faces at y = 0 and z = 0, legs 5 mm thick, two 8.5 mm clearance holes for M8.
BRACKET_HOLE_DIAMETER = 0.0085
BRACKET_OPENINGS = {          # name -> (opening centre, outward normal of the face around it)
    "A_outer": ((0, -0.020, 0.0), (0, 0, 1)), "A_inner": ((0, -0.020, -0.005), (0, 0, -1)),
    "B_outer": ((0, 0.0, -0.020), (0, 1, 0)), "B_inner": ((0, -0.005, -0.020), (0, -1, 0)),
}
BRACKET_WIDTH = 0.040         # m, across the legs (CAD x)
# Rotations mapping the bracket onto itself: identity and 180 deg about (0, 1, 1)/sqrt(2) (swaps the legs)
BRACKET_SYMMETRIES = [np.eye(3), np.array([[-1.0, 0, 0], [0, 0, 1], [0, 1, 0]])]

MIN_POSE_PROB = 0.02          # ignore implausible rest poses (e.g. profile standing on its end)


@lru_cache(maxsize=None)
def load_part(name):
    """Main solid of the part's STEP file as a watertight trimesh, in the CAD frame (metres)."""
    scene = trimesh.load(os.path.join(CAD_DIR, PARTS[name]["step"]))
    mesh = scene.to_geometry()
    mesh.merge_vertices()
    body = max(mesh.split(only_watertight=False), key=lambda m: m.area)   # main solid; drops tiny STEP fragments
    assert body.is_watertight, f"{name}: main solid is not watertight"
    return body


@lru_cache(maxsize=None)
def stable_poses(name):
    """(transforms, probabilities): rotations that put the part at rest on the z=0 plane.

    Slow (~1 min per part), so cached next to the CAD file; delete the .npz if the CAD changes.
    """
    cache = os.path.join(CAD_DIR, f"{name}_stable_poses.npz")
    if not os.path.exists(cache):
        T, p = load_part(name).compute_stable_poses(n_samples=2000, threshold=0.0)
        np.savez(cache, T=T, p=p)
    d = np.load(cache)
    T, p = d["T"], d["p"]
    keep = p >= MIN_POSE_PROB
    return T[keep], p[keep] / p[keep].sum()


if __name__ == "__main__":
    from materials import MATERIALS
    for n in PARTS:
        m = load_part(n)
        rho = MATERIALS[PARTS[n]["material"]]["physical"]["density"]
        T, p = stable_poses(n)
        print(f"{n}: extents (mm) {np.round(m.extents * 1e3, 1)}, faces {len(m.faces)}, "
              f"mass {m.volume * rho * 1e3:.1f} g, rest poses {len(p)} (p = {np.round(p, 2)})")
