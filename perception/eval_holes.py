"""Evaluate the top-hole detector on simulated frames against CAD ground truth.

Frames come from the ideal dataset data/ideal/<backend>/seed_XXX/head/ (PyBullet frames that are
missing are rendered on the fly; Isaac frames must be rendered first with scene/render.py). The detector runs on the head camera's depth and is compared
with the bracket's hole openings from the CAD model (scene/parts.py).

Ground truth: the bracket has 2 through-holes, each with 2 openings (outer face, inner face
leg thickness deeper). Valid top openings = those facing the camera whose outward normal is within
MAX_TOP_TILT_DEG of vertical and that are visible: at least 6 of 8 points on a ring 2 mm outside
the hole edge (on its face) match the rendered depth within 2 mm. A detection is correct if it lies
within CORRECT_MM of a valid opening (when the bracket rests on its corner, both holes tilt 45 deg
and are equally "top").

  python perception/eval_holes.py              # fixed test set config.TEST_SEEDS
  python perception/eval_holes.py --backend isaac      # after rendering those seeds
  python perception/eval_holes.py --max-frames 10      # retry on later frames
  python perception/eval_holes.py --camera d435 --runs 10   # D435 model, 10 noise draws per scene
  python perception/eval_holes.py --camera d435 --axial ahn --runs 10   # ablation: Ahn axial noise

Multi-frame: per test scene, run the detector on frame 0, 1, ... and stop at the first frame
with a top-hole detection (the robot cannot know if it is correct), up to --max-frames. The
scene is static: with ideal depth every frame is the same image and only the detector's RANSAC
sampling changes; with the D435 model (--camera d435) every frame gets a new noise draw (frame_depth()).
--runs repeats the whole scene with independent noise draws; draws are seeded by (seed, run, frame),
so all model variants see the same random numbers (paired comparison).
"""
import argparse
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
import camera_model  # noqa: E402
from config import TEST_SEEDS  # noqa: E402
from frame_io import ideal_frame  # noqa: E402
import hole_detector_tuned  # noqa: E402
from hole_detector import MAX_TOP_TILT_DEG  # noqa: E402
from parts import BRACKET_HOLE_DIAMETER  # noqa: E402
from parts import BRACKET_OPENINGS as OPENINGS  # noqa: E402  (bracket hole openings, CAD frame)

CORRECT_MM = 3.0
VIS_RING = BRACKET_HOLE_DIAMETER / 2 + 0.002     # m, visibility ring radius around an opening


def pose_matrix(position, quat_wxyz):
    T = trimesh.transformations.quaternion_matrix(quat_wxyz)
    T[:3, 3] = position
    return T


def gt_openings(spec, cam_pose):
    """Openings in the camera frame: name -> (centre_cam, normal_cam, normal_world)."""
    T_wc = pose_matrix(cam_pose["position"], cam_pose["quat_wxyz"])
    br = next(p for p in spec["parts"] if p["name"] == "bracket")
    T_wo = pose_matrix(br["position"], br["quat_wxyz"])
    T_co = np.linalg.inv(T_wc) @ T_wo
    out = {}
    for name, (c, n) in OPENINGS.items():
        out[name] = ((T_co @ [*c, 1])[:3], T_co[:3, :3] @ n, T_wo[:3, :3] @ n)
    return out


def visible(c, n, depth, K, ring=VIS_RING, tol=0.002):
    """Is the face around an opening seen by the camera (not hidden by other geometry)?"""
    e1 = np.cross(n, [1.0, 0, 0]) if abs(n[0]) < 0.9 else np.cross(n, [0, 1.0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    hits = 0
    for a in np.linspace(0, 2 * np.pi, 8, endpoint=False):
        p = c + ring * (np.cos(a) * e1 + np.sin(a) * e2)
        u, v = int(K[0, 0] * p[0] / p[2] + K[0, 2]), int(K[1, 1] * p[1] / p[2] + K[1, 2])
        if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]:
            hits += abs(depth[v, u] - p[2]) < tol
    return hits >= 6


def gt_top(openings, depth, K):
    """Valid top openings (facing the camera, upward, visible), most upward first."""
    valid = [(n_w[2], name) for name, (c, n_cam, n_w) in openings.items()
             if n_cam @ (-c / np.linalg.norm(c)) > 0.2 and n_w[2] > np.cos(np.radians(MAX_TOP_TILT_DEG))
             and visible(c, n_cam, depth, K)]
    return [name for _, name in sorted(valid, reverse=True)]


def frame_depth(depth, K, args, seed, run, frame_idx):
    """Depth of frame `frame_idx` of a static scene: the ideal depth through the camera model.
    Ideal: identical for every frame. D435 model: a new noise draw per (seed, run, frame)."""
    return camera_model.depth(args, depth, K, np.random.default_rng((seed, run, frame_idx)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=TEST_SEEDS,
                    help="scene seeds; default: the fixed test set config.TEST_SEEDS (15 scenes with a visible top hole)")
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet")
    ap.add_argument("--max-frames", type=int, default=1, help="frames tried per scene until a detection")
    camera_model.add_args(ap)
    hole_detector_tuned.add_args(ap)
    ap.add_argument("--runs", type=int, default=1, help="independent noise draws per scene")
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "hole_detector"))
    args = ap.parse_args()
    args.detect, args.det_tag = hole_detector_tuned.from_args(args)
    os.makedirs(args.out, exist_ok=True)

    def get_frame(seed):
        f = ideal_frame(args.backend, seed)                    # data/ideal/<backend>/seed_XXX/head
        if f is None:
            return None, None
        return f["meta"]["spec"], {"depth": f["depth"], "cam_pose_ros": f["meta"]["camera_pose_world_ros"]}

    rows = []
    for seed in args.seeds:
        spec, cam = get_frame(seed)
        if spec is None:
            print(f"seed {seed}: no {args.backend} frame, skipped")
            continue
        K = np.array(spec["camera"]["K"])
        ops = gt_openings(spec, cam["cam_pose_ros"])
        gt = gt_top(ops, cam["depth"], K)                      # visibility from the ideal depth
        for run in range(args.runs):
            rows.append(evaluate(seed, run, cam["depth"], K, ops, gt, args))

    tag = f"{camera_model.tag(args)}_depth{args.det_tag}_{args.backend}_frames{args.max_frames}" + (f"_runs{args.runs}" if args.runs > 1 else "")
    with open(os.path.join(args.out, f"results_{tag}.json"), "w") as f:
        json.dump(rows, f, indent=1)
    summary(rows, args)


def evaluate(seed, run, depth, K, ops, gt, args):
    """Detect on up to --max-frames frames of one scene and score the top hole against the GT."""
    for f in range(args.max_frames):
        det = args.detect(frame_depth(depth, K, args, seed, run, f), K, seed=seed * 1000 + f)
        if det["top"] is not None:
            break
    row = {"seed": seed, "run": run, "gt_top": gt, "n_candidates": len(det["all"]),
           "detected": det["top"] is not None, "frames_used": f + 1}
    if det["top"] is not None:
        c, a = det["top"]["center"], det["top"]["axis"]
        name, (cg, ng, _) = min(ops.items(), key=lambda kv: np.linalg.norm(kv[1][0] - c))
        row.update(matched=name, pos_err_mm=float(np.linalg.norm(c - cg) * 1e3),
                   axis_err_deg=float(np.degrees(np.arccos(np.clip(abs(a @ ng), 0, 1)))),
                   diameter_mm=float(det["top"]["diameter"] * 1e3), n_px=det["top"]["n_px"])
        row["correct"] = bool(row["pos_err_mm"] < CORRECT_MM and name in gt)
    # candidates not near any bracket opening = false positives (e.g. on the profile)
    row["false_pos"] = int(sum(min(np.linalg.norm(h["center"] - o[0]) for o in ops.values()) * 1e3 > CORRECT_MM
                           for h in det["all"]))
    print(json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}))
    return row


def summary(rows, args):
    has_gt = [r for r in rows if r["gt_top"]]
    correct = [r for r in has_gt if r.get("correct")]
    print(f"\n=== top-hole detector{args.det_tag}, {camera_model.tag(args)} depth, {args.backend}, up to {args.max_frames} frame(s), "
          f"{args.runs} run(s) per scene ===")
    print(f"scene runs {len(rows)}, with a visible upward hole (GT) {len(has_gt)}")
    print(f"detected correctly {len(correct)}/{len(has_gt)} ({100 * len(correct) / max(len(has_gt), 1):.0f}%)")
    print(f"detections on frames without GT top hole: {sum(r['detected'] for r in rows if not r['gt_top'])}")
    print(f"false-positive candidates (not on a bracket opening): {sum(r['false_pos'] for r in rows)}")
    if correct:
        pe = np.array([r["pos_err_mm"] for r in correct]); ae = np.array([r["axis_err_deg"] for r in correct])
        print(f"position error mm : median {np.median(pe):.2f}, mean {pe.mean():.2f}, max {pe.max():.2f}")
        print(f"axis error deg    : median {np.median(ae):.2f}, mean {ae.mean():.2f}, max {ae.max():.2f}")
    print(f"frames used per scene: mean {np.mean([r['frames_used'] for r in rows]):.2f}, "
          f"scenes needing > 1 frame: {sum(r['frames_used'] > 1 for r in rows)}")
    print(f"no detection {sum(not r['detected'] for r in has_gt)}, "
          f"wrong detection {sum(r['detected'] and not r.get('correct') for r in has_gt)}")
    per_seed = {}
    for r in has_gt:
        per_seed.setdefault(r["seed"], []).append(bool(r.get("correct")))
    print("correct runs per seed: " + ", ".join(f"{s}: {sum(v)}/{len(v)}" for s, v in per_seed.items()))


if __name__ == "__main__":
    main()
