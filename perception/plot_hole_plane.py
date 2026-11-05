"""3D plots of the depth points around the bracket's top hole: what the hole detector makes of the hole face.

Three panels side by side: ideal depth + baseline detector, D435 depth + baseline detector, D435 depth +
noise-aware (tuned) detector. Oblique 3D view (rotatable in the interactive window). Only depth points are drawn,
in the true hole's frame (ground truth, as in eval_holes.py; independent of the detector):
  x, y   in the true face plane, origin at the true hole centre [mm]
  h      height off the true face plane [mm] (axis stretched ~4x relative to x, y)
Points within RADIUS of the hole centre and |h| < H_MAX, coloured by what the detector did with them:
  blue     on the detected face containing the hole (the face with most points near the hole)
  orange   on another detected face
  grey     on no face (left over after face peeling, or not an object pixel)
  black    rays of a hole gap the detector accepted, drawn where they meet their face plane
Panel titles: face points near the hole, the detected face's tilt against the true face, and the
detected top hole (distance from the true centre; "wrong" if > 5 mm, as in eval_holes.py).

  python perception/plot_hole_plane.py --seed 8 --run 22                 # D435 model (see camera_model/)
  python perception/plot_hole_plane.py --seed 8 --run 22 --bias 0        # without the depth bias
  python perception/plot_hole_plane.py --seed 8 --run 22 --save          # PNG to output/hole_plane/<condition>/

--k-sigma / --tune-axial set the tuned detector (default k = 2.5, Intel noise). Noise draw r of seed s
is the same as run r in eval_*.py and grasp_hole.py.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
import camera_model  # noqa: E402
from bracket_fit import frame_from_axis  # noqa: E402
from eval_holes import CORRECT_MM, gt_openings, gt_top  # noqa: E402
from frame_io import ideal_frame  # noqa: E402
from hole_detector import detect_holes, rays  # noqa: E402
from hole_detector_tuned import K_SIGMA, detect_holes_tuned  # noqa: E402

RADIUS = 18.0        # mm, in-plane crop around the hole centre (hole 8.5 mm, centre 20 mm from the leg tip)
H_MAX = 5.0          # mm, crop off the plane
COLOURS = {"hole face": "tab:blue", "other face": "tab:orange", "no face": "0.75", "accepted hole gap": "black"}


def panel(depth, K, c, R, det):
    """Points (hole frame, mm) per class, and the panel's summary line."""
    Rays = rays(K, depth.shape)
    to_hole = lambda X: (X - c) @ R * 1e3                                   # noqa: E731
    near = lambda Y: (np.hypot(Y[:, 0], Y[:, 1]) < RADIUS) & (np.abs(Y[:, 2]) < H_MAX)  # noqa: E731
    valid = depth > 0
    Y = to_hole(Rays * depth[..., None])                                    # H x W x 3
    in_crop = valid & near(Y.reshape(-1, 3)).reshape(depth.shape)

    # the hole face = detected face with most pixels in the crop
    counts = [np.count_nonzero(f["mask"] & in_crop) for f in det["faces"]]
    i_hole = int(np.argmax(counts)) if counts and max(counts) > 0 else None
    on_any = np.zeros(depth.shape, bool)
    for f in det["faces"]:
        on_any |= f["mask"]
    hole_face = det["faces"][i_hole]["mask"] if i_hole is not None else np.zeros(depth.shape, bool)
    out = {"hole face": Y[in_crop & hole_face], "other face": Y[in_crop & on_any & ~hole_face],
           "no face": Y[in_crop & ~on_any]}
    gaps = []
    for h in det["all"]:
        f = next(f for f in det["faces"] if f["normal"] is h["axis"])
        r = Rays[h["mask"]]
        G = to_hole(r * (-f["d"] / (r @ f["normal"]))[:, None])
        gaps.append(G[near(G)])
    out["accepted hole gap"] = np.vstack(gaps) if gaps else np.zeros((0, 3))

    line = f"hole face {len(out['hole face'])} pts"
    if i_hole is not None:
        tilt = np.degrees(np.arccos(np.clip(abs(det["faces"][i_hole]["normal"] @ R[:, 2]), 0, 1)))
        line += f", tilt {tilt:.1f}°"
    top = det["top"]
    if top is None:
        line += "\ntop hole: none"
    else:
        e = np.linalg.norm(top["center"] - c) * 1e3
        line += f"\ntop hole: {e:.1f} mm from true" + (" (wrong)" if e > CORRECT_MM else "")
    return out, line


def draw(ax, pts, elev, azim):
    for name, X in pts.items():
        if len(X):
            ax.scatter(*X.T, c=COLOURS[name], s=4 if name != "no face" else 2, depthshade=False, label=name)
    ax.set_xlim(-RADIUS, RADIUS); ax.set_ylim(-RADIUS, RADIUS); ax.set_zlim(-H_MAX, H_MAX)
    ax.set_box_aspect((1, 1, 0.6))
    ax.set_proj_type("ortho")                                # no perspective: edge-on heights read true
    ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.set_zlabel("h [mm]")
    ax.view_init(elev=elev, azim=azim)
    if elev == 90:                                           # face-on: h points at the viewer
        ax.set_zticks([]); ax.set_zlabel("")
    elif elev == 0:                                          # edge-on: y points away from the viewer
        ax.set_yticks([]); ax.set_ylabel("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--run", type=int, default=0, help="noise draw (D435 model)")
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet")
    camera_model.add_args(ap)
    ap.add_argument("--k-sigma", type=float, default=K_SIGMA, help="tuned detector: thresholds in units of sigma_z")
    ap.add_argument("--tune-axial", choices=["intel", "ahn"], default="intel")
    ap.add_argument("--save", nargs="?", const="auto",
                    help="write a PNG instead of opening a window; without a path it goes to "
                         "output/hole_plane/<condition>/seed_XXX_runYY.png")
    args = ap.parse_args()
    args.camera = "d435"                                     # the noisy rows; ideal rows are always shown
    tag = camera_model.tag(args)
    if args.save == "auto":
        args.save = os.path.join(ROOT, "output", "hole_plane", f"{args.backend}_{tag}",
                                 f"seed_{args.seed:03d}_run{args.run:02d}.png")

    f = ideal_frame(args.backend, args.seed)
    if f is None:
        sys.exit(f"no ideal frame for seed {args.seed} ({args.backend}); see README 'Test set and reference data'")
    spec, cam = f["meta"]["spec"], f["meta"]["camera_pose_world_ros"]
    K = np.array(spec["camera"]["K"])
    ops = gt_openings(spec, cam)
    top = gt_top(ops, f["depth"], K)
    if not top:
        sys.exit(f"seed {args.seed}: no visible top hole")
    c, n, _ = ops[top[0]]
    R = frame_from_axis(n, np.array([1.0, 0, 0]))            # columns: x, y in the face, z = face normal

    noisy = camera_model.depth(args, f["depth"], K, np.random.default_rng((args.seed, args.run, 0)))
    tuned = lambda d: detect_holes_tuned(d, K, args.seed, args.k_sigma, args.tune_axial)  # noqa: E731
    rows = [("ideal depth, baseline", f["depth"], lambda d: detect_holes(d, K, seed=args.seed)),
            (f"{tag} draw {args.run}, baseline", noisy, lambda d: detect_holes(d, K, seed=args.seed)),
            (f"{tag} draw {args.run}, tuned k={args.k_sigma:g}", noisy, tuned)]

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(18, 6.5))
    for i, (name, depth, detect) in enumerate(rows):
        pts, line = panel(depth, K, c, R, detect(depth))
        print(f"{name}: {line}".replace("\n", " | "))
        ax = fig.add_subplot(1, len(rows), i + 1, projection="3d")
        draw(ax, pts, 30, -60)
        ax.set_title(f"{name}\n{line}", fontsize=10)
    handles = [plt.Line2D([], [], ls="", marker="o", color=col, label=lab) for lab, col in COLOURS.items()]
    fig.legend(handles=handles, loc="lower center", ncol=4)
    fig.suptitle(f"seed {args.seed} ({args.backend}), true top hole {top[0]}: depth points near the hole, "
                 f"in the true hole frame (h stretched ~4x)")
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    if args.save:
        os.makedirs(os.path.dirname(os.path.abspath(args.save)), exist_ok=True)
        fig.savefig(args.save, dpi=120)
        print("wrote", os.path.relpath(args.save))
    else:
        plt.show()


if __name__ == "__main__":
    main()
