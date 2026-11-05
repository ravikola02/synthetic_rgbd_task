"""Evaluate the bracket fit (hole detection -> 6D bracket pose) against CAD ground truth.

Scenes where no top hole is detected cannot be fitted (the fit starts from the hole).
Success = translation error < SUCCESS_T_MM and symmetry-aware rotation error < SUCCESS_R_DEG.
ADD-S is reported too, but not used for success: the bracket is compact and nearly cube-shaped,
so poses rotated by 90-150 deg still had ADD-S of 2-3 mm (< the usual 10 % of diameter).
Writes overlays of the reconstruction for the first --vis seeds (model points coloured by
their depth residual on top of the depth view) and a PLY with scene + fitted model points.

  python perception/eval_bracket_fit.py [--backend isaac]     # development set config.DEV_SEEDS
  python perception/eval_bracket_fit.py --camera d435 --runs 10   # D435 model, 10 noise draws per scene
  python perception/eval_bracket_fit.py --camera d435 --bias 0 --runs 10   # ablation: no depth bias (see camera_model/)

With the D435 depth bias every position moves ~10 mm along the viewing ray, so the summary also counts
"pose found": rotation < SUCCESS_R_DEG and translation < FOUND_T_MM (bias shift allowed).

Noise draws are seeded by (seed, run) (same as frame 0 in eval_holes), so all model variants see the
same random numbers.
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
import camera_model  # noqa: E402
from bracket_fit import fit_bracket, model_points, pose_errors, visible_model  # noqa: E402
from eval_holes import pose_matrix  # noqa: E402
from frame_io import depth_to_vis, ideal_frame  # noqa: E402
import hole_detector_tuned  # noqa: E402
from config import DEV_SEEDS  # noqa: E402


SUCCESS_T_MM, SUCCESS_R_DEG = 5.0, 10.0
FOUND_T_MM = 15.0       # "pose found": right pose up to the D435 depth-bias shift (+-2 % ~ 10-13 mm here)


def gt_pose(spec, cam_pose):
    br = next(p for p in spec["parts"] if p["name"] == "bracket")
    return np.linalg.inv(pose_matrix(cam_pose["position"], cam_pose["quat_wxyz"])) @ pose_matrix(br["position"], br["quat_wxyz"])


def overlay(depth, K, T, pts, nrm, path):
    """Depth view with the fitted model's visible points: green = on the measured surface, red = off."""
    img = Image.fromarray(depth_to_vis(depth, K)).convert("RGB")
    X, u, v = visible_model(T, pts, nrm, np.asarray(K), depth.shape)
    r = np.abs(depth[v, u] - X[:, 2])
    draw = ImageDraw.Draw(img)
    for uu, vv, rr in zip(u, v, r):
        draw.point((uu, vv), fill=(0, 255, 0) if rr < 0.003 else (255, 0, 0))
    cx, cy = int(np.median(u)), int(np.median(v))
    box = (max(cx - 60, 0), max(cy - 40, 0), max(cx - 60, 0) + 120, max(cy - 40, 0) + 80)
    img.crop(box).resize((480, 320), Image.NEAREST).save(path)


def save_ply(path, scene_pts, model_pts):
    pts = np.vstack([scene_pts, model_pts])
    col = np.vstack([np.tile([150, 150, 150], (len(scene_pts), 1)), np.tile([255, 60, 60], (len(model_pts), 1))])
    with open(path, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\nproperty float x\nproperty float y\n"
                "property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for p, c in zip(pts, col):
            f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {c[0]} {c[1]} {c[2]}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=DEV_SEEDS,
                    help="scene seeds; default: the development set config.DEV_SEEDS (15 scenes with a visible top hole)")
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet")
    camera_model.add_args(ap)
    hole_detector_tuned.add_args(ap)
    ap.add_argument("--runs", type=int, default=1, help="independent noise draws per scene")
    ap.add_argument("--vis", type=int, default=6, help="write overlays / PLY for the first N fitted seeds")
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "bracket_fit"))
    args = ap.parse_args()
    detect, det_tag = hole_detector_tuned.from_args(args)
    out = os.path.join(args.out, (args.backend if args.camera == "ideal" else f"{args.backend}_{camera_model.tag(args)}")
                       + det_tag)
    os.makedirs(out, exist_ok=True)
    pts, nrm = model_points()

    rows, n_vis = [], 0
    for seed in args.seeds:
        f = ideal_frame(args.backend, seed)                    # data/ideal/<backend>/seed_XXX/head
        if f is None:
            continue
        spec, cam_pose = f["meta"]["spec"], f["meta"]["camera_pose_world_ros"]
        K = np.array(spec["camera"]["K"])
        for run in range(args.runs):
            depth = camera_model.depth(args, f["depth"], K, np.random.default_rng((seed, run, 0)))
            row = evaluate(seed, run, depth, K, spec, cam_pose, pts, nrm, out, vis=n_vis < args.vis and run == 0,
                           detect=detect)
            n_vis += row.pop("vis", 0)
            rows.append(row)

    with open(os.path.join(out, f"results_runs{args.runs}.json" if args.runs > 1 else "results.json"), "w") as f:
        json.dump(rows, f, indent=1)
    summary(rows, args, det_tag)


def evaluate(seed, run, depth, K, spec, cam_pose, pts, nrm, out, vis, detect):
    det = detect(depth, K, seed=seed)
    fit = fit_bracket(depth, K, det, seed=seed)
    row = {"seed": seed, "run": run, "hole_detected": det["top"] is not None}
    if fit is not None:
        T_gt = gt_pose(spec, cam_pose)
        t, r, adds = pose_errors(fit["T_cam_obj"], T_gt, pts)
        t0, r0, adds0 = pose_errors(fit["init_T"], T_gt, pts)
        row.update(opening=fit["opening"], t_err_mm=t, r_err_deg=r, adds_mm=adds, success=bool(t < SUCCESS_T_MM and r < SUCCESS_R_DEG),
                   init_adds_mm=adds0, init_r_err_deg=r0, icp_rmse_mm=fit["icp_rmse_mm"],
                   inliers=fit["inliers"], violations=fit["violations"])
        if vis:
            overlay(depth, K, fit["T_cam_obj"], pts, nrm, os.path.join(out, f"seed_{seed:03d}_overlay.png"))
            cloud = det["points"][(depth > 0) & (depth < 1.0)][::4]
            Xm = pts @ fit["T_cam_obj"][:3, :3].T + fit["T_cam_obj"][:3, 3]
            save_ply(os.path.join(out, f"seed_{seed:03d}_reconstruction.ply"), cloud, Xm)
            row["vis"] = 1
    print(json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}))
    return row


def summary(rows, args, det_tag=""):
    fitted = [r for r in rows if "adds_mm" in r]
    ok = [r for r in fitted if r["success"]]
    print(f"\n=== bracket fit from top hole{det_tag}, {camera_model.tag(args)} depth, {args.backend}, {args.runs} run(s) per scene ===")
    print(f"scene runs {len(rows)}, hole candidate found (fit attempted) {len(fitted)}, "
          f"success (t < {SUCCESS_T_MM:.0f} mm and r < {SUCCESS_R_DEG:.0f} deg) {len(ok)}/{len(rows)}")
    found = [r for r in fitted if r["t_err_mm"] < FOUND_T_MM and r["r_err_deg"] < SUCCESS_R_DEG]
    print(f"pose found (t < {FOUND_T_MM:.0f} mm, depth bias shift allowed, and r < {SUCCESS_R_DEG:.0f} deg) "
          f"{len(found)}/{len(rows)}" + (f", translation median {np.median([r['t_err_mm'] for r in found]):.1f} mm, "
                                         f"rotation median {np.median([r['r_err_deg'] for r in found]):.2f} deg" if found else ""))
    if ok:
        print("successful fits:")
        for key, lab in (("adds_mm", "ADD-S mm"), ("t_err_mm", "translation mm"), ("r_err_deg", "rotation deg"),
                         ("init_adds_mm", "ADD-S before ICP mm")):
            a = np.array([r[key] for r in ok])
            print(f"  {lab:20s}: median {np.median(a):.2f}, mean {a.mean():.2f}, max {a.max():.2f}")
    per_seed = {}
    for r in rows:
        per_seed.setdefault(r["seed"], []).append(bool(r.get("success")))
    print("successful runs per seed: " + ", ".join(f"{s}: {sum(v)}/{len(v)}" for s, v in per_seed.items()))


if __name__ == "__main__":
    main()
