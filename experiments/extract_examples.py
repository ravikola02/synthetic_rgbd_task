"""Example cases from the final sweep (output/final_sweep/) as figures and grasp videos -> extracted_examples/.

  python experiments/extract_examples.py            # all examples (~2 min; videos need the grasp, PyBullet)
  python experiments/extract_examples.py --no-video

Each example recomputes one (seed, noise draw, condition) of the sweep with the same code and the same random
numbers (draw r of seed s = np.random.default_rng((s, r, 0))), so the figures match the sweep's rows. The
cases were picked from output/final_sweep/results/rows_*.csv to show the main points (extracted_examples/README.md).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import textwrap

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, "perception"), os.path.join(ROOT, "scene"),
                os.path.join(ROOT, "scene", "backends")]
import camera_model  # noqa: E402
from bracket_fit import fit_bracket, model_points, visible_model  # noqa: E402
from eval_bracket_fit import gt_pose  # noqa: E402
from eval_holes import gt_openings, gt_top  # noqa: E402
from final_sweep import CONDITIONS, LABELS  # noqa: E402
from frame_io import depth_to_vis, ideal_frame  # noqa: E402
from hole_detector import detect_holes  # noqa: E402

OUT = os.path.join(ROOT, "extracted_examples")
PY = sys.executable
GREEN, RED, ORANGE, BLUE, INK, MUTED = "#1baf7a", "#d62728", "#eb6834", "#2a78d6", "#2b2b2b", "#8a8a85"


def cam_args(cond):
    ap = argparse.ArgumentParser()
    camera_model.add_args(ap)
    return ap.parse_args(CONDITIONS[cond])


def case(seed, run, cond):
    """Depth, detection, fit and ground truth of one sweep row."""
    f = ideal_frame("pybullet", seed)
    spec, pose = f["meta"]["spec"], f["meta"]["camera_pose_world_ros"]
    K = np.array(spec["camera"]["K"])
    depth = camera_model.depth(cam_args(cond), f["depth"], K, np.random.default_rng((seed, run, 0)))
    det = detect_holes(depth, K, seed=seed * 1000)            # as eval_holes (detector stage, frame 0)
    det_fit = detect_holes(depth, K, seed=seed)               # as eval_bracket_fit / grasp_hole (fit stage)
    fit = fit_bracket(depth, K, det_fit, seed=seed)
    ops = gt_openings(spec, pose)
    bracket_id = next(p["instance_id"] for p in spec["parts"] if p["name"] == "bracket")
    return {"depth": depth, "K": K, "det": det, "fit": fit, "ops": ops, "gt": gt_top(ops, f["depth"], K),
            "T_gt": gt_pose(spec, pose), "mask": f["instance"] == bracket_id, "cond": cond, "seed": seed, "run": run}


def matched(c):
    """Name of the true opening nearest to the detector's pick (as eval_holes)."""
    return min(c["ops"], key=lambda n: np.linalg.norm(c["ops"][n][0] - c["det"]["top"]["center"]))


def uv(K, p):
    return K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]


def crop_box(c, pts_uv, pad=30):
    H, W = c["depth"].shape
    us, vs = zip(*pts_uv)
    return (int(max(min(us) - pad, 0)), int(min(max(us) + pad, W)), int(max(min(vs) - pad, 0)), int(min(max(vs) + pad, H)))


def style(ax, title):
    ax.set_title(title, fontsize=9, loc="left", color=INK)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color(MUTED)


def detection_panel(ax, c, box, title):
    u0, u1, v0, v1 = box
    ax.imshow(depth_to_vis(c["depth"], c["K"])[v0:v1, u0:u1], extent=(u0, u1, v1, v0))
    for h in c["det"]["all"]:
        u, v = uv(c["K"], h["center"])
        ax.plot(u, v, "o", ms=5, mfc="none", mec=ORANGE, mew=1.2)
    for name in c["gt"]:
        u, v = uv(c["K"], c["ops"][name][0])
        ax.plot(u, v, "o", ms=13, mfc="none", mec=GREEN, mew=2)
    if c["det"]["top"] is not None:
        u, v = uv(c["K"], c["det"]["top"]["center"])
        ax.plot(u, v, "x", ms=11, color=RED, mew=2.5)
    ax.set_xlim(u0, u1)
    ax.set_ylim(v1, v0)
    style(ax, title)


def fit_panel(ax, c, box, title):
    u0, u1, v0, v1 = box
    vis = depth_to_vis(c["depth"], c["K"]).astype(float).mean(axis=2)
    ax.imshow(vis[v0:v1, u0:u1], cmap="gray", extent=(u0, u1, v1, v0), vmin=0, vmax=255)
    if c["fit"] is not None:
        pts, nrm = model_points()
        X, u, v = visible_model(c["fit"]["T_cam_obj"], pts, nrm, c["K"], c["depth"].shape)
        on = np.abs(c["depth"][v, u] - X[:, 2]) < 0.003
        ax.scatter(u[on] + 0.5, v[on] + 0.5, s=2, c=GREEN, lw=0)
        ax.scatter(u[~on] + 0.5, v[~on] + 0.5, s=2, c=RED, lw=0)
    ax.set_xlim(u0, u1)
    ax.set_ylim(v1, v0)
    style(ax, title)


def row_text(c):
    top = c["det"]["top"]
    if top is None:
        d = "no hole detected"
    else:
        e = min(np.linalg.norm(top["center"] - c["ops"][n][0]) for n in c["gt"]) * 1e3 if c["gt"] else np.nan
        d = f"top pick {e:.0f} mm from the hole" if e > 3 else f"hole found ({e:.1f} mm)"
    if c["fit"] is None:
        return d + "; no fit"
    T, G = c["fit"]["T_cam_obj"], c["T_gt"]
    return d + f"; fit {np.linalg.norm(T[:3, 3] - G[:3, 3]) * 1e3:.1f} mm"


def fig_detect_fit(cases, name, title, note):
    import matplotlib.pyplot as plt
    pts = []
    for c in cases:
        pts += [uv(c["K"], c["ops"][n][0]) for n in c["ops"]]
        pts += [uv(c["K"], h["center"]) for h in c["det"]["all"]]
        vs, us = np.nonzero(c["mask"])
        pts += [(us.min(), vs.min()), (us.max(), vs.max())]
    box = crop_box(cases[0], pts)
    fig, axs = plt.subplots(2, len(cases), figsize=(4.2 * len(cases), 6.4), squeeze=False)
    for j, c in enumerate(cases):
        lab = LABELS[c["cond"]] + ("" if c["cond"] == "0_ideal" else f", draw {c['run']}")
        detection_panel(axs[0, j], c, box, f"{lab}: detection\n{row_text(c).split(';')[0]}")
        fit_panel(axs[1, j], c, box, f"{lab}: CAD fit\n{row_text(c).split('; ')[-1]}")
    width = int(14 * 4.2 * len(cases))
    fig.suptitle(textwrap.fill(title, width // 1.6), x=0.01, ha="left", fontsize=12, color=INK)
    fig.text(0.01, 0.01, textwrap.fill(note, width) + "\n" + textwrap.fill(
             "Green ring = true top hole; orange = hole candidates; red x = detector's pick. "
             "Fit: model points on the measured surface (green) / off it (red).", width), fontsize=8, color=MUTED)
    fig.tight_layout(rect=(0, 0.09, 1, 0.92))
    fig.savefig(os.path.join(OUT, name), dpi=150)
    plt.close(fig)


def fig_edge(seed, name):
    """Edge fattening alone: depth around the hole, ideal vs edge, and a cut through the hole centre."""
    import matplotlib.pyplot as plt
    a, b = case(seed, 0, "0_ideal"), case(seed, 0, "3_edge")
    hc = a["ops"][matched(a)][0]
    u, v = (int(x) for x in uv(a["K"], hc))
    r = 14
    box = (u - r, u + r, v - r, v + r)
    vals = np.concatenate([a["depth"][v - r:v + r, u - r:u + r].ravel(), b["depth"][v - r:v + r, u - r:u + r].ravel()])
    vals = vals[vals > 0]
    lo, hi = np.percentile(vals, [2, 98]) * 1e3
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.6), gridspec_kw={"width_ratios": [1, 1, 1.3]}, layout="constrained")
    for ax, c, lab in ((axs[0], a, "ideal"), (axs[1], b, "edge fattening (R200 value)")):
        im = ax.imshow(c["depth"][v - r:v + r, u - r:u + r] * 1e3, cmap="viridis", vmin=lo, vmax=hi,
                       extent=(u - r, u + r, v + r, v - r), interpolation="nearest")
        ax.axhline(v + 0.5, color="white", lw=0.8, ls="--")
        top = c["det"]["top"]
        n = c["ops"][matched(c)][1]
        style(ax, f"{lab}\nhole seen as {top['diameter'] * 1e3:.1f} mm (true 8.5)\n"
                  f"axis error {np.degrees(np.arccos(min(1, abs(top['axis'] @ n)))):.1f}°")
    fig.colorbar(im, ax=[axs[0], axs[1]], shrink=0.85, label="depth (mm)")
    us = np.arange(u - r, u + r)
    for c, col, lab in ((a, BLUE, "ideal"), (b, ORANGE, "edge fattening")):
        d = c["depth"][v, u - r:u + r] * 1e3
        axs[2].step(us, np.where(d > 0, d, np.nan), where="mid", color=col, lw=2, label=lab)
    axs[2].set_xlabel("image column (px), row through the hole centre")
    axs[2].set_ylabel("depth (mm)")
    axs[2].legend(frameon=False, fontsize=8)
    axs[2].grid(color="#e4e3df")
    for sp in ("top", "right"):
        axs[2].spines[sp].set_visible(False)
    fig.suptitle(f"Seed {seed}: edge fattening alone fills the hole with foreground depth; it is still detected, "
                 "but its axis tilts", x=0.01, ha="left", fontsize=12, color=INK)
    fig.savefig(os.path.join(OUT, name), dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_bias(seed, run, name):
    """Depth bias: side view (camera x vs depth z) of the bracket's points and the fitted / true CAD model."""
    import matplotlib.pyplot as plt
    pts, nrm = model_points()
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.4), sharey=True)
    for ax, cond in zip(axs, ("0_ideal", "5_bias")):
        c = case(seed, 0 if cond == "0_ideal" else run, cond)
        K, D = c["K"], c["depth"]
        vs, us = np.nonzero(c["mask"] & (D > 0))
        z = D[vs, us]
        x = (us + 0.5 - K[0, 2]) / K[0, 0] * z
        ax.scatter(x * 1e3, z * 1e3, s=3, c="#b8b7b0", lw=0, label="measured bracket points")
        for T, col, lab in ((c["T_gt"], GREEN, "true pose"), (c["fit"]["T_cam_obj"], BLUE, "fitted CAD")):
            X, _, _ = visible_model(T, pts, nrm, K, D.shape)
            ax.scatter(X[:, 0] * 1e3, X[:, 2] * 1e3, s=2, c=col, lw=0, label=lab)
        t = np.linalg.norm(c["fit"]["T_cam_obj"][:3, 3] - c["T_gt"][:3, 3]) * 1e3
        sign = "" if cond == "0_ideal" else (" (+2 %)" if np.random.default_rng((seed, run, 0)).choice([-1.0, 1.0]) > 0 else " (−2 %)")
        ax.set_title(f"{LABELS[cond]}{sign}: fit {t:.1f} mm from the true pose", fontsize=10, loc="left", color=INK)
        ax.set_xlabel("camera x (mm)")
        ax.grid(color="#e4e3df")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axs[0].set_ylabel("depth z (mm)")
    axs[0].invert_yaxis()
    axs[1].legend(frameon=False, fontsize=8, markerscale=4, loc="lower right")
    fig.suptitle(f"Seed {seed}, draw {run}: the depth bias moves the whole part along the viewing ray "
                 "(shape and rotation unchanged)", x=0.01, ha="left", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT, name), dpi=150)
    plt.close(fig)


def video(seed, run, cond, name):
    """Grasp video of one sweep row (close view); returns the grasp result row."""
    import json
    if os.path.exists(os.path.join(OUT, name)):                 # already rendered: keep it
        print(f"video {name}: exists, kept")
        return {}
    d = os.path.join(OUT, f"_video_tmp_{name}")
    shutil.rmtree(d, ignore_errors=True)
    subprocess.run([PY, "robot/grasp_hole.py", "--seeds", str(seed), "--run-ids", str(run), "--videos", "1",
                    "--close-view", "--out", d, *CONDITIONS[cond]], cwd=ROOT, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    mp4 = glob.glob(os.path.join(d, "*.mp4"))
    res = glob.glob(os.path.join(d, "results*.json"))
    row = json.load(open(res[0]))[0] if res else {}
    if mp4:
        shutil.move(mp4[0], os.path.join(OUT, name))
    shutil.rmtree(d, ignore_errors=True)
    moved = max([p["bracket_moved_mm"] for p in row.get("phase_log", [])] or [0])
    print(f"video {name}: {row.get('status', 'no fit')}, lift {row.get('lift_mm', float('nan')):.0f} mm, "
          f"bracket moved {moved:.0f} mm" + ("" if mp4 else " (no video: grasp not executed)"))
    return row


def strip(videos, name, title):
    """Frames at the same moments of two grasp videos (ideal vs camera model), one row each."""
    import cv2
    import matplotlib.pyplot as plt
    fr = (0.45, 0.62, 0.80, 0.99)
    fig, axs = plt.subplots(len(videos), len(fr), figsize=(3.3 * len(fr), 2.7 * len(videos)), squeeze=False)
    for i, (mp4, lab) in enumerate(videos):
        cap = cv2.VideoCapture(os.path.join(OUT, mp4))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        for j, f in enumerate(fr):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(f * (n - 1)))
            ok, img = cap.read()
            if ok:
                axs[i, j].imshow(cv2.cvtColor(img[:420], cv2.COLOR_BGR2RGB))
            style(axs[i, j], lab if j == 0 else "")
        cap.release()
    fig.suptitle(title, x=0.01, ha="left", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94), h_pad=2.0)
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)


# executed grasps: (seed, draw, condition, video name, strip label); ideal videos are made next to each case
GRASPS = {
    "01": (243, 2, "2_axial", "axial noise: lifted"),
    "02": (299, 0, "4_edge_axial", "edge + axial: lifted"),
    "03": (351, 0, "3_edge", "edge fattening: lifted"),
    "04": (245, 0, "5_bias", "depth bias +2 %: lifted"),
    "05": (432, 0, "6_full", "full model: lifted"),
}
VIDEO_NAMES = {"2_axial": "axial_noise", "4_edge_axial": "edge_axial", "3_edge": "edge_fattening",
               "5_bias": "depth_bias", "6_full": "full_model"}


def main():
    from concurrent.futures import ThreadPoolExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    # perception: what the camera effects do to the detector and the fit
    fig_detect_fit([case(243, 2, "0_ideal"), case(243, 2, "2_axial")], "01_axial_noise_false_holes.png",
                   "Seed 243, draw 2: axial noise - 11 false holes, the detector picks one 177 mm away",
                   "The noise breaks up the faces and leaves enclosed gaps that pass as holes, mostly on the profile. "
                   "The CAD fit tries every candidate and still finds the bracket (1.1 mm off), and the grasp lifts it "
                   "(video 01).")
    fig_detect_fit([case(299, 0, "0_ideal"), case(299, 0, "2_axial"), case(299, 0, "4_edge_axial")],
                   "02_edge_fattening_amplifies_noise.png",
                   "Seed 299, draw 0: same noise draw - axial alone finds the hole, edge + axial does not",
                   "Edge fattening fills the hole with foreground depth, so with noise the real hole loses to a false one "
                   "163 mm away. The fit still recovers the bracket and the grasp lifts it (video 02).")
    fig_edge(351, "03_edge_fattening_fills_hole.png")
    # pose shift: the bias moves the whole part along the viewing ray
    fig_bias(245, 0, "04_depth_bias_shift.png")
    fig_detect_fit([case(432, 0, "0_ideal"), case(432, 0, "6_full")], "05_full_model_false_hole_still_lifted.png",
                   "Seed 432, draw 0: full D435 model - false hole 278 mm away, pose 10 mm off, the bracket is still lifted",
                   "The detector picks a false hole on the profile, the fit recovers the bracket but shifted by the depth "
                   "bias, and the jaws still close on it and lift it (video 05).")
    # where the pipeline stops
    fig_detect_fit([case(431, 3, "0_ideal"), case(431, 3, "6_full")], "06_full_model_wrong_pose.png",
                   "Seed 431, draw 3: full D435 model - the CAD fit locks onto a false hole: pose 253 mm / 43° off",
                   "Every hole candidate is false, and the best-scoring CAD match is on the wrong part. The grasp "
                   "planner then finds no reachable grasp (IK failed): the robot does not move.")
    fig_detect_fit([case(511, 0, "0_ideal"), case(511, 0, "5_bias")], "07_depth_bias_body_collision.png",
                   "Seed 511: depth bias - the shifted grasp needs an arm motion through the robot's body",
                   "Hole and shape are right, the pose is 10.3 mm off along the viewing ray. Every arm motion that reaches "
                   "the shifted grasp would pass the arm 4.5 mm into the torso, so the planner rejects it "
                   "(body collision, all 5 draws): the robot does not move.")
    fig_detect_fit([case(246, 0, "0_ideal"), case(246, 0, "1_rules")], "08_control_rules_planner_sensitivity.png",
                   "Control, seed 246: datasheet rules alone leave perception unchanged, yet the grasp planner fails (IK)",
                   "Same hole, same pose (0.23 vs 0.25 mm): the grasp search lands on a candidate the arm cannot reach "
                   "(all 5 draws). Part of every grasp drop vs ideal is this planner sensitivity.")
    if a.no_video:
        return
    jobs = []
    for k, (seed, run, cond, _) in GRASPS.items():
        jobs += [(seed, run, "0_ideal", f"{k}_ideal_grasp_seed{seed}.mp4"),
                 (seed, run, cond, f"{k}_{VIDEO_NAMES[cond]}_grasp_seed{seed}_draw{run}.mp4")]
    with ThreadPoolExecutor(len(jobs)) as ex:
        list(ex.map(lambda j: video(*j), jobs))
    for k, (seed, run, cond, lab) in GRASPS.items():
        strip([(f"{k}_ideal_grasp_seed{seed}.mp4", "ideal: lifted"),
               (f"{k}_{VIDEO_NAMES[cond]}_grasp_seed{seed}_draw{run}.mp4", lab)],
              f"{k}_grasp_frames.png", f"Seed {seed} grasp: ideal depth (top) vs {LABELS[cond]}, draw {run} (bottom)")
    print(f"examples in {OUT}")


if __name__ == "__main__":
    main()
