"""Evaluate the bracket fit (hole detection -> 6D bracket pose) against CAD ground truth.

Scenes where no top hole is detected cannot be fitted (the fit starts from the hole).
Success = translation error < SUCCESS_T_MM and symmetry-aware rotation error < SUCCESS_R_DEG.
ADD-S is reported too, but not used for success: the bracket is compact and nearly cube-shaped,
so poses rotated by 90-150 deg still had ADD-S of 2-3 mm (< the usual 10 % of diameter).
Writes overlays of the reconstruction for the first --vis seeds (model points coloured by
their depth residual on top of the depth view) and a PLY with scene + fitted model points.

  python perception/eval_bracket_fit.py [--backend isaac]     # fixed test set config.TEST_SEEDS
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [HERE, os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
from bracket_fit import fit_bracket, model_points, pose_errors, visible_model  # noqa: E402
from eval_holes import pose_matrix  # noqa: E402
from frame_io import depth_to_vis, ideal_frame  # noqa: E402
from hole_detector import detect_holes  # noqa: E402
from config import TEST_SEEDS  # noqa: E402


SUCCESS_T_MM, SUCCESS_R_DEG = 5.0, 10.0


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
    ap.add_argument("--seeds", type=int, nargs="*", default=TEST_SEEDS,
                    help="scene seeds; default: the fixed test set config.TEST_SEEDS (15 scenes with a visible top hole)")
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet")
    ap.add_argument("--vis", type=int, default=6, help="write overlays / PLY for the first N fitted seeds")
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "bracket_fit"))
    args = ap.parse_args()
    out = os.path.join(args.out, args.backend)
    os.makedirs(out, exist_ok=True)
    pts, nrm = model_points()

    rows, n_vis = [], 0
    for seed in args.seeds:
        f = ideal_frame(args.backend, seed)                    # data/ideal/<backend>/seed_XXX/head
        if f is None:
            continue
        spec, depth, cam_pose = f["meta"]["spec"], f["depth"], f["meta"]["camera_pose_world_ros"]
        K = np.array(spec["camera"]["K"])
        det = detect_holes(depth, K, seed=seed)
        fit = fit_bracket(depth, K, det, seed=seed)
        row = {"seed": seed, "hole_detected": det["top"] is not None}
        if fit is not None:
            T_gt = gt_pose(spec, cam_pose)
            t, r, adds = pose_errors(fit["T_cam_obj"], T_gt, pts)
            t0, r0, adds0 = pose_errors(fit["init_T"], T_gt, pts)
            row.update(opening=fit["opening"], t_err_mm=t, r_err_deg=r, adds_mm=adds, success=bool(t < SUCCESS_T_MM and r < SUCCESS_R_DEG),
                       init_adds_mm=adds0, init_r_err_deg=r0, icp_rmse_mm=fit["icp_rmse_mm"],
                       inliers=fit["inliers"], violations=fit["violations"])
            if n_vis < args.vis:
                overlay(depth, K, fit["T_cam_obj"], pts, nrm, os.path.join(out, f"seed_{seed:03d}_overlay.png"))
                cloud = det["points"][(depth > 0) & (depth < 1.0)][::4]
                Xm = pts @ fit["T_cam_obj"][:3, :3].T + fit["T_cam_obj"][:3, 3]
                save_ply(os.path.join(out, f"seed_{seed:03d}_reconstruction.ply"), cloud, Xm)
                n_vis += 1
        rows.append(row)
        print(json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}))

    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump(rows, f, indent=1)
    fitted = [r for r in rows if "adds_mm" in r]
    ok = [r for r in fitted if r["success"]]
    print(f"\n=== bracket fit from top hole, ideal depth, {args.backend} ===")
    print(f"scenes {len(rows)}, hole detected (fit attempted) {len(fitted)}, "
          f"success (t < {SUCCESS_T_MM:.0f} mm and r < {SUCCESS_R_DEG:.0f} deg) {len(ok)}/{len(fitted)}")
    if ok:
        print("successful fits:")
        for key, lab in (("adds_mm", "ADD-S mm"), ("t_err_mm", "translation mm"), ("r_err_deg", "rotation deg"),
                         ("init_adds_mm", "ADD-S before ICP mm")):
            a = np.array([r[key] for r in ok])
            print(f"  {lab:20s}: median {np.median(a):.2f}, mean {a.mean():.2f}, max {a.max():.2f}")
    print(f"failed seeds: {[r['seed'] for r in fitted if not r['success']]}")


if __name__ == "__main__":
    main()
