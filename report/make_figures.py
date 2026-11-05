"""Figures of the report -> report/figures/ (scene.png, detection.png, hole_plane.png, grasp.png).

  python report/make_figures.py            # ~3 min: renders 4 grasp videos (PyBullet) for grasp.png
  python report/make_figures.py --no-video # scene / detection only (scene.png then needs an existing video)

Uses the same cases and random numbers as experiments/extract_examples.py (draw r of seed s =
np.random.default_rng((s, r, 0))). Grasp videos use grasp_hole.py --side-view, so the hand is not drawn over
the robot's body.
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
GRASPS = [(233, 1, "0_ideal"), (233, 1, "5_bias"), (204, 2, "0_ideal"), (204, 2, "6_full")]


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


def fig_scene(seed=204, run=2):
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
        pts += [uv(c["K"], c["ops"][n][0]) for n in c["ops"]]
        pts += [uv(c["K"], h["center"]) for h in c["det"]["all"]]
        vs, us = np.nonzero(c["mask"])
        pts += [(us.min(), vs.min()), (us.max(), vs.max())]
    box = crop_box(cases[0], pts, pad=12)
    fig, axs = plt.subplots(1, 3, figsize=(10, 4.3))
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
    """Two grasp moments per case; columns: seed 233 ideal / bias, seed 204 ideal / full model."""
    import matplotlib.pyplot as plt
    moments = ((0.62, "closing"), (0.99, "after lift"))
    rows = [("ideal depth", [GRASPS[0], GRASPS[2]]), ("camera model", [GRASPS[1], GRASPS[3]])]
    fig, axs = plt.subplots(2, 4, figsize=(14, 4.9))
    for i, (rlab, jobs) in enumerate(rows):
        for k, (seed, run, cond) in enumerate(jobs):
            for m, (fr, mlab) in enumerate(moments):
                ax = axs[i, 2 * k + m]
                ax.imshow(frame(video_name(seed, run, cond), fr))
                lab = {"0_ideal": "ideal", "5_bias": "depth bias +2 %"}.get(cond, LABELS[cond])
                style(ax, f"seed {seed}, {lab}: {mlab}")
    fig.tight_layout(w_pad=0.4, h_pad=1.6)
    fig.savefig(os.path.join(OUT, "grasp.png"), dpi=170, bbox_inches="tight")
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
    fig_scene()
    fig_detection()
    fig_hole_plane()
    print(f"figures in {OUT}")


if __name__ == "__main__":
    main()
