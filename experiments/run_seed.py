"""Run one seed through the whole downstream task (detector -> bracket fit -> grasp) for one depth condition.

For inspecting a single case of the final experiment, e.g. a failure in output/final_sweep/: same scripts,
same frozen options and the same noise draw as the sweep, so the numbers match the sweep's row.

  python experiments/run_seed.py --seed 201                               # full model, draw 0, headless
  python experiments/run_seed.py --seed 201 --condition 2_axial --run 3   # axial only, noise draw 3
  python experiments/run_seed.py --seed 201 --gui                         # watch the grasp in the PyBullet GUI
  python experiments/run_seed.py --seed 201 --video                       # headless, grasp video (mp4)
  python experiments/run_seed.py --seed 201 --stages holes fit            # perception only
  python experiments/run_seed.py --seed 201 --backend isaac               # perception on the Isaac frame

Conditions (experiments/final_sweep.py): 0_ideal 1_rules 2_axial 3_edge 4_edge_axial 5_bias 6_full.
--gui opens the PyBullet window for the grasp (real-time stepping, Enter closes it at the end); it only
slows the simulation down, the result is the same as headless. The detector and fit are headless in both
cases and write their overlay (depth view + fitted model) and PLY; eval_bracket_fit draws these for
noise draw 0 only, so they are named *_draw0. The grasp is PyBullet only.
Outputs: output/run_seed/seed_XXX_<condition>_run<R>/  (holes/, fit/, grasp/ with results JSON + logs).
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, "perception"), os.path.join(ROOT, "robot"),
                os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
from final_sweep import CONDITIONS  # noqa: E402

PY = sys.executable


def perception(script, extra, run, out, backend, cam):
    """Run a perception script for draws 0..run (draws are seeded by (seed, run)) and return row `run`."""
    os.makedirs(out, exist_ok=True)
    cmd = [PY, script, "--runs", str(run + 1), "--backend", backend, "--out", out, *extra, *cam]
    with open(os.path.join(out, "log.txt"), "w") as f:
        rc = subprocess.run(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT).returncode
    files = glob.glob(os.path.join(out, "**", "results*.json"), recursive=True)
    if rc or not files:
        sys.exit(f"{script} failed (rc {rc}), see {out}/log.txt")
    with open(files[0]) as f:
        return next(r for r in json.load(f) if r["run"] == run)


def grasp(seed, run, out, cam, gui, video):
    """Grasp in-process (so the GUI can be shown); the scene and pipeline are those of robot/grasp_hole.py."""
    import pybullet as p
    import pybullet_backend as pb
    import grasp_hole
    if gui:
        build, step, disconnect = pb.build, p.stepSimulation, p.disconnect

        def build_gui(spec, gui_=False):
            sim = build(spec, gui=True)                         # parts settle inside build (normal speed)
            p.resetDebugVisualizerCamera(0.9, 50, -30, [0.35, 0.0, 0.0])

            def slow_step(*a, **k):                             # real time for watching only
                r = step(*a, **k)
                time.sleep(1 / 240)
                return r
            p.stepSimulation = slow_step
            return sim

        def disconnect_wait(*a, **k):
            if not a and not k:                                 # the GUI session (IK clients pass their id)
                p.stepSimulation = step
                input("[grasp] done, press Enter to close the window")
            return disconnect(*a, **k)
        pb.build = build_gui
        p.disconnect = disconnect_wait
    os.makedirs(out, exist_ok=True)
    sys.argv = ["grasp_hole.py", "--seeds", str(seed), "--run-ids", str(run), "--videos", "1" if video else "0",
                "--out", out, *cam]
    grasp_hole.main()
    files = glob.glob(os.path.join(out, "results*.json"))
    with open(files[0]) as f:
        return {k: v for k, v in json.load(f)[0].items() if k != "phase_log"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--condition", choices=list(CONDITIONS), default="6_full")
    ap.add_argument("--run", type=int, default=0, help="noise draw (ignored for 0_ideal)")
    ap.add_argument("--stages", nargs="*", choices=["holes", "fit", "grasp"], default=["holes", "fit", "grasp"])
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet",
                    help="renderer of the perception frame (Isaac frame must exist: scene/render.py)")
    ap.add_argument("--gui", action="store_true", help="show the grasp in the PyBullet GUI")
    ap.add_argument("--video", action="store_true", help="record the grasp video (headless)")
    ap.add_argument("--out", default=None, help="default: output/run_seed/seed_XXX_<condition>_run<R>/")
    a = ap.parse_args()
    run = 0 if a.condition == "0_ideal" else a.run
    cam = CONDITIONS[a.condition]
    out = a.out or os.path.join(ROOT, "output", "run_seed", f"seed_{a.seed:03d}_{a.condition}_run{run}"
                                + ("_isaac" if a.backend == "isaac" else ""))
    res = {}
    if "holes" in a.stages:
        res["holes"] = perception("perception/eval_holes.py", ["--seeds", str(a.seed)], run,
                                  os.path.join(out, "holes"), a.backend, cam)
    if "fit" in a.stages:
        res["fit"] = perception("perception/eval_bracket_fit.py", ["--seeds", str(a.seed), "--vis", "1"], run,
                                os.path.join(out, "fit"), a.backend, cam)
        for f in glob.glob(os.path.join(out, "fit", "*", f"seed_{a.seed:03d}_*")):   # eval_bracket_fit draws
            stem, ext = os.path.splitext(f)                                           # the overlay of draw 0 only
            if "_draw0" not in stem:
                os.replace(f, f"{stem}_draw0{ext}")
    if "grasp" in a.stages:
        res["grasp"] = grasp(a.seed, run, os.path.join(out, "grasp"), cam, a.gui, a.video)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(f"\n=== seed {a.seed}, {a.condition}, draw {run}, perception on {a.backend} ===")
    h, ft, g = res.get("holes"), res.get("fit"), res.get("grasp")
    if h:
        print(f"detector: {'correct' if h.get('correct') else 'right hole (15 mm)' if h.get('right_hole') else 'no detection' if not h['detected'] else 'wrong hole'}"
              + (f", position error {h['pos_err_mm']:.1f} mm, axis {h['axis_err_deg']:.1f} deg" if "pos_err_mm" in h else ""))
    if ft:
        found = "t_err_mm" in ft and ft["t_err_mm"] < 15 and ft["r_err_deg"] < 10
        print(f"fit: {'success' if ft.get('success') else 'pose found (15 mm allowance)' if found else 'failed' if 't_err_mm' in ft else 'not attempted (no hole)'}"
              + (f", t {ft['t_err_mm']:.1f} mm, r {ft['r_err_deg']:.1f} deg" if "t_err_mm" in ft else ""))
    if g:
        print(f"grasp: {g.get('status', 'no fit')}" + (f", lift {g['lift_mm']:.0f} mm" if "lift_mm" in g else ""))
    print(f"outputs: {out}")


if __name__ == "__main__":
    main()
