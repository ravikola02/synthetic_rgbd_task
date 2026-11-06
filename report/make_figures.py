"""Figures of the report -> report/figures/ (scene.png, tasks.png, detection.png, hole_plane.png, grasp.png).

  python report/make_figures.py            # ~1 min: renders the grasp video for tasks.png (grasp.png uses extracted_examples/ videos)
  python report/make_figures.py --no-video # scene / detection only (scene.png then needs an existing video)

Uses the same cases and random numbers as experiments/extract_examples.py (draw r of seed s =
np.random.default_rng((s, r, 0))). grasp.png reuses example 05's close-view grasp videos from
extracted_examples/ (made by experiments/extract_examples.py).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "experiments"))
from extract_examples import (CONDITIONS, LABELS, PY, case, crop_box, detection_panel,  # noqa: E402
                              row_text, style, uv)
from frame_io import depth_to_vis, ideal_frame  # noqa: E402

OUT = os.path.join(HERE, "figures")
VID = os.path.join(OUT, "videos")
GRASPS = [(432, 0, "0_ideal")]                          # tasks.png (side view)
EXAMPLES = os.path.join(ROOT, "extracted_examples")     # grasp.png: example 09's close-view videos


def video_name(seed, run, cond):
    return os.path.join(VID, f"grasp_{seed}_{cond}_draw{run}.mp4")


def render_video(job):
    seed, run, cond = job
    dst = video_name(seed, run, cond)
    if os.path.exists(dst):
        return
    tmp = dst + "_tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    subprocess.run([PY, "robot/grasp_hole.py", "--seeds", str(seed), "--run-ids", str(run), "--videos", "1",
                    "--side-view", "--out", tmp, *CONDITIONS[cond]], cwd=ROOT,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    mp4 = glob.glob(os.path.join(tmp, "*.mp4"))
    if mp4:
        shutil.move(mp4[0], dst)
    shutil.rmtree(tmp, ignore_errors=True)


def frame(path, f):
    import cv2
    cap = cv2.VideoCapture(path)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(f * (n - 1)))
    ok, img = cap.read()
    cap.release()
    return cv2.cvtColor(img[:420], cv2.COLOR_BGR2RGB)        # below 420: the video's text overlay


def overview(seed, eye=(1.05, -0.75, 0.55), target=(0.25, 0.0, 0.05)):
    """Still of the robot at the table (PyBullet, observation pose), from the front."""
    import pybullet as p
    import pybullet_backend as pb
    from spec import make_scene
    pb.build(make_scene(seed))
    view = p.computeViewMatrix(list(eye), list(target), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(40, 4 / 3, 0.05, 5.0)
    img = np.reshape(p.getCameraImage(800, 600, view, proj, renderer=p.ER_TINY_RENDERER)[2], (600, 800, 4))
    p.disconnect()
    return img[..., :3]


def fig_scene(seed=432, run=0):
    """Robot at the table, head-camera RGB, ideal depth and full-model depth around the parts."""
    import matplotlib.pyplot as plt
    f = ideal_frame("pybullet", seed)
    K = np.array(f["meta"]["spec"]["camera"]["K"])
    ideal, full = case(seed, run, "0_ideal"), case(seed, run, "6_full")
    vs, us = np.nonzero(f["instance"] >= 3)                      # parts
    u0, u1, v0, v1 = us.min() - 25, us.max() + 25, vs.min() - 25, vs.max() + 25
    fig, axs = plt.subplots(1, 4, figsize=(14, 2.9), gridspec_kw={"width_ratios": [1.1, 1.45, 1, 1]})
    axs[0].imshow(overview(seed))
    style(axs[0], "G1 at the table")
    axs[1].imshow(f["rgb"])
    axs[1].add_patch(plt.Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, ec="white", lw=1, ls="--"))
    style(axs[1], "head camera RGB (848×480)")
    for ax, c, lab in ((axs[2], ideal, "ideal depth"), (axs[3], full, "D435 model depth")):
        ax.imshow(depth_to_vis(c["depth"], K)[v0:v1, u0:u1])
        style(ax, lab)
    fig.tight_layout(w_pad=0.6)
    fig.savefig(os.path.join(OUT, "scene.png"), dpi=200, bbox_inches="tight")
    plt.close(fig)


def fig_detection(seed=299, run=0):
    """Hole detection on one noise draw: ideal, axial noise, edge fattening + axial noise."""
    import matplotlib.pyplot as plt
    cases = [case(seed, run, c) for c in ("0_ideal", "2_axial", "4_edge_axial")]
    pts = []
    for c in cases:
        vs, us = np.nonzero(c["mask"])                           # the bracket only: a false pick far away
        pts += [(us.min(), vs.min()), (us.max(), vs.max())]     # is named in the panel title instead
    box = crop_box(cases[0], pts, pad=6)
    fig, axs = plt.subplots(1, 3, figsize=(7.6, 2.6))
    for ax, c in zip(axs, cases):
        detection_panel(ax, c, box, f"{LABELS[c['cond']]}\n{row_text(c).split(';')[0]}")
    fig.tight_layout(w_pad=1.0)
    fig.savefig(os.path.join(OUT, "detection.png"), dpi=200, bbox_inches="tight")
    plt.close(fig)


def fig_hole_plane(seed=8, run=22):
    """Depth points around the true top hole, in its frame: ideal vs axial noise (as plot_hole_plane.py).
    Seed 8 is a development scene: its noise draw 22 shows the hole face breaking up most clearly."""
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(ROOT, "perception"))
    from bracket_fit import frame_from_axis
    from eval_holes import gt_openings, gt_top
    from hole_detector import detect_holes
    from plot_hole_plane import COLOURS, draw, panel
    from extract_examples import cam_args
    f = ideal_frame("pybullet", seed)
    spec, cam = f["meta"]["spec"], f["meta"]["camera_pose_world_ros"]
    K = np.array(spec["camera"]["K"])
    ops = gt_openings(spec, cam)
    c, n, _ = ops[gt_top(ops, f["depth"], K)[0]]
    R = frame_from_axis(n, np.array([1.0, 0, 0]))
    import camera_model
    rows = [(lab, camera_model.depth(cam_args(cond), f["depth"], K, np.random.default_rng((seed, run, 0))))
            for lab, cond in (("ideal depth", "0_ideal"), ("axial noise", "2_axial"))]
    fig = plt.figure(figsize=(10, 4.6))
    for i, (name, depth) in enumerate(rows):
        pts, line = panel(depth, K, c, R, detect_holes(depth, K, seed=seed))
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        draw(ax, pts, 30, -60)
        ax.set_title(f"{name}\n{line.replace('top hole', 'detected top hole')}", fontsize=10)
    handles = [plt.Line2D([], [], ls="", marker="o", color=col, label=lab) for lab, col in COLOURS.items()]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(os.path.join(OUT, "hole_plane.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def fig_grasp():
    """Seed 473, ideal vs full D435 model (draw 3), close view from extracted_examples/ (example 09)."""
    import matplotlib.pyplot as plt
    rows = (("09_ideal_grasp_seed473.mp4", "ideal depth: gripped squarely, lifted 100 mm"),
            ("09_full_model_grasp_seed473_draw3.mp4", "full D435 model, draw 3: caught by a corner, lifted 53 mm"))
    moments = (0.45, 0.62, 0.80, 0.99)
    fig, axs = plt.subplots(2, 4, figsize=(12, 4.6))
    for i, (mp4, lab) in enumerate(rows):
        for j, fr in enumerate(moments):
            axs[i, j].imshow(frame(os.path.join(EXAMPLES, mp4), fr))
            style(axs[i, j], lab if j == 0 else "")
    fig.tight_layout(w_pad=0.4, h_pad=1.2)
    fig.savefig(os.path.join(OUT, "grasp.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def fig_tasks(seed=432, run=0):
    """The three stages on ideal depth: detected top hole, fitted CAD, grasp (isometric video, rendered if missing)."""
    import matplotlib.pyplot as plt
    from bracket_fit import model_points, visible_model
    c = case(seed, run, "0_ideal")
    vs, us = np.nonzero(c["mask"])
    box = crop_box(c, [(us.min(), vs.min()), (us.max(), vs.max())], pad=8)
    c["det"]["all"] = [h for h in c["det"]["all"]                              # candidates on the bracket only
                       if box[0] <= uv(c["K"], h["center"])[0] <= box[1] and box[2] <= uv(c["K"], h["center"])[1] <= box[3]]
    u0, u1, v0, v1 = box
    vids = {}
    for view in ("iso", "axis"):              # isometric: robot + table; axis: close-up, hole axis drawn
        vids[view] = video_name(seed, run, "0_ideal").replace(".mp4", f"_{view}.mp4")
        if not os.path.exists(vids[view]):
            tmp = vids[view] + "_tmp"
            subprocess.run([PY, "robot/grasp_hole.py", "--seeds", str(seed), "--run-ids", str(run), "--videos", "1",
                            f"--{view}-view", "--out", tmp, *CONDITIONS["0_ideal"]], cwd=ROOT,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            shutil.move(glob.glob(os.path.join(tmp, "*.mp4"))[0], vids[view])
            shutil.rmtree(tmp, ignore_errors=True)
    img = frame(vids["iso"], 0.62)
    close = frame(vids["axis"], 0.42)[50:390, 120:500]
    fig, axs = plt.subplots(1, 3, figsize=(8, 2.6), gridspec_kw={"width_ratios": [1, 1, 1.25]})
    detection_panel(axs[0], c, box, "1  hole detection")
    axs[1].imshow(depth_to_vis(c["depth"], c["K"]).mean(axis=2)[v0:v1, u0:u1], cmap="gray",
                  extent=(u0, u1, v1, v0), vmin=0, vmax=255)
    pts, nrm = model_points()
    _, u, v = visible_model(c["fit"]["T_cam_obj"], pts, nrm, c["K"], c["depth"].shape)
    axs[1].scatter(u + 0.5, v + 0.5, s=1.5, c="#1baf7a", lw=0)
    axs[1].set_xlim(u0, u1)
    axs[1].set_ylim(v1, v0)
    style(axs[1], "2  pose fit (ICP)")
    axs[2].imshow(img[15:, 90:570])
    style(axs[2], "3  grasp along the hole axis")
    ins = axs[2].inset_axes([0.0, 0.0, 0.37, 0.46])                 # corner: approach along the hole axis
    ins.imshow(close)
    ins.set_xticks([])
    ins.set_yticks([])
    for sp in ins.spines.values():
        sp.set_color("#8a8a85")
    fig.tight_layout(w_pad=0.6)
    fig.savefig(os.path.join(OUT, "tasks.png"), dpi=200, bbox_inches="tight")
    plt.close(fig)


def fig_stages(conds=("0_ideal", "2_axial", "4_edge_axial", "5_bias", "6_full")):
    """Success per stage for the main conditions (from output/final_sweep/results/summary.csv), 95 % exact
    intervals; same metrics and colours as the sweep's all_stages.png."""
    import csv
    import matplotlib.pyplot as plt
    with open(os.path.join(ROOT, "output", "final_sweep", "results", "summary.csv")) as f:
        summary = list(csv.DictReader(f))
    metrics = [(("holes", "right_hole"), "#2a78d6", "hole detected (15 mm)"),
               (("fit", "success"), "#eb6834", "fit success (5 mm, 10°)"),
               (("grasp", "lifted"), "#1baf7a", "grasp lifted")]
    x, w = np.arange(len(conds)), 0.8 / len(metrics)
    fig, ax = plt.subplots(figsize=(7, 2.9))
    for i, (m, col, name) in enumerate(metrics):
        rr = {r["condition"]: r for r in summary if (r["stage"], r["metric"]) == m}
        y = np.array([float(rr[c]["rate"]) * 100 for c in conds])
        err = np.array([[y[j] - 100 * float(rr[c]["cp_lo"]), 100 * float(rr[c]["cp_hi"]) - y[j]]
                        for j, c in enumerate(conds)]).T
        pos = x + (i - 1) * w
        ax.bar(pos, y, w * 0.92, color=col, label=name, zorder=2)
        ax.errorbar(pos, y, yerr=err, fmt="none", ecolor="#2b2b2b", elinewidth=1, capsize=2.5, zorder=3)
    ax.set_xticks(x, [LABELS[c] for c in conds])
    ax.set_ylim(0, 105)
    ax.set_ylabel("success (%)")
    ax.grid(axis="y", color="#e4e3df", zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.16))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "stages.png"), dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    from concurrent.futures import ThreadPoolExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    os.makedirs(VID, exist_ok=True)
    if not a.no_video:
        with ThreadPoolExecutor(len(GRASPS)) as ex:
            list(ex.map(render_video, GRASPS))
        fig_grasp()
        fig_tasks()
    fig_scene()
    fig_detection()
    fig_hole_plane()
    fig_stages()
    print(f"figures in {OUT}")


if __name__ == "__main__":
    main()
