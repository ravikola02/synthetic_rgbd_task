"""Final experiment: 7 depth conditions x 3 stages (detector, bracket fit, grasp) on the held-out test seeds.

Protocol: AI_NOTES.md Q3, "Final test protocol" (frozen, git tag final-experiment, amended 2026-11-05).

  python experiments/final_sweep.py select        # screen seeds from 201 up, keep the first 30 that pass the rule
  python experiments/final_sweep.py run           # all conditions x stages x seeds, in parallel (resumes: done jobs skipped)
  python experiments/final_sweep.py analyse       # merge results, statistics, bar plots
  python experiments/final_sweep.py all           # the three in order

Seed rule (same as DEV_SEEDS, AI_NOTES.md Q3 amendment 3): every ideal-depth stage succeeds - top hole visible,
detector within 3 mm, fit success, grasp lifted. Screening runs the ideal stages on frames kept in output/<sweep>/screening/ideal/ (IDEAL_DIR), so rejected seeds
never enter data/ideal/.

Primary result: 30 seeds (quick to replicate; timings in REPRODUCE.md). Extended result (e.g. overnight,
~1.5-2 h): --n-seeds 100, same rule and protocol; the first 30 of its seeds are the primary ones. It writes
to output/final_sweep_n100/ and keeps its ideal frames there (output/final_sweep_n100/ideal/, via the
IDEAL_DIR environment variable) instead of adding ~140 MB to the tracked data/ideal/.
  python experiments/final_sweep.py all --n-seeds 100

Renderer: --backend pybullet (default) or isaac. The seeds are always screened on PyBullet depth (same
rule, same seeds, so both backends see the same scenes); with isaac the selected seeds are then rendered in
Isaac Sim (one seed per process, sequential: GPU; --isaac-workers to run more at once) and the detector and
the fit run on the Isaac depth. The grasp is PyBullet only (physics + its own head render) in both cases.
Isaac outputs: output/final_sweep_isaac[_n<N>]/; Isaac frames of the 30 seeds go to data/ideal/isaac/.
  OMNI_KIT_ACCEPT_EULA=YES python experiments/final_sweep.py all --backend isaac

Live view: every command starts the web view (experiments/watch.py: progress, running rates, plots) and opens
it in the browser; --no-view skips it, --port sets the port (next free one if taken). When the command ends
in a terminal, it waits for Enter so the page stays up.

The runs call the existing scripts with the frozen command-line options (one process per
condition x stage x seed, numpy single-threaded), so a result is exactly what the script gives.
Noise draws are seeded by (seed, run): splitting the work over processes does not change any number.

Outputs (output/final_sweep[_isaac]/ for 30 seeds, output/final_sweep[_isaac]_n<N>/ otherwise):
  screening/ideal/pybullet/seed_XXX/   ideal frames of the screened seeds (selected ones are copied
                               to <IDEAL_DIR>/pybullet/), screening.json (visible / rejected per seed)
  test_seeds.json              the selected seeds
  raw/<condition>/<stage>/seed_XXX/   per-job results JSON + log.txt
  results/rows_<stage>.csv     every row of every condition (+ bias sign, hole distance / viewing angle)
  results/summary.csv          rate, k/n, 95 % exact (Clopper-Pearson) and seed-bootstrap intervals,
                               exact McNemar p vs ideal and for edge+axial vs axial
  plots/*.png                  bar plots with 95 % error bars
"""
import argparse
import csv
import glob
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "perception"), os.path.join(ROOT, "scene"),
                os.path.join(ROOT, "scene", "backends")]
OUT = os.path.join(ROOT, "output", "final_sweep")
PY = sys.executable

N_SEEDS = 30
FIRST_SEED = 201            # seeds <= 200 were used in development or in the ideal accuracy evaluation
DRAWS = 5                   # noise draws per seed, perception and grasp (ideal: 1, it is deterministic)
WORKERS = os.cpu_count() or 4      # parallel processes (results do not depend on it)
BACKEND = "pybullet"        # renderer of the perception frames: pybullet | isaac
ISAAC_WORKERS = 1
INFEASIBLE = ("IK failed", "body collision", "no table-clear grasp")

# condition -> camera options (AI_NOTES.md Q3, frozen protocol table)
CONDITIONS = {
    "0_ideal":      ["--camera", "ideal"],
    "1_rules":      ["--camera", "d435", "--edge", "none", "--axial", "none", "--bias", "0"],
    "2_axial":      ["--camera", "d435", "--edge", "none", "--bias", "0", "--no-rules"],
    "3_edge":       ["--camera", "d435", "--axial", "none", "--bias", "0", "--no-rules"],
    "4_edge_axial": ["--camera", "d435", "--bias", "0", "--no-rules"],
    "5_bias":       ["--camera", "d435", "--edge", "none", "--axial", "none", "--no-rules"],
    "6_full":       ["--camera", "d435"],
}
LABELS = {"0_ideal": "ideal", "1_rules": "rules", "2_axial": "axial", "3_edge": "edge",
          "4_edge_axial": "edge + axial", "5_bias": "bias ±2 %", "6_full": "full model"}
STAGES = ("holes", "fit", "grasp")

# accuracy: (stage, value, subset metric, label); over the draws that pass the 15 mm criterion (the bias
# shift then shows up as an error, not as a failure), grasp over the executed grasps
ACCURACY = [
    ("holes", "pos_err_mm", "right_hole", "detector: hole position error (mm)"),
    ("holes", "axis_err_deg", "right_hole", "detector: hole axis error (°)"),
    ("fit", "t_err_mm", "found", "fit: translation error (mm)"),
    ("fit", "r_err_deg", "found", "fit: rotation error (°)"),
    ("grasp", "hole_pos_err_mm", "fitted", "grasp: hole position error at grasp time (mm)"),
    ("grasp", "approach_err_deg", "fitted", "grasp: approach direction error (°)"),
]

# (stage, metric) -> label. Detector success = the hole was detected (the detection is on a visible top hole,
# nearest opening within 15 mm, so the depth-bias shift is allowed); how accurately is reported separately
# (position / axis error, and "within 3 mm" as an accuracy threshold)
METRICS = {
    ("holes", "right_hole"): "detector: hole detected (right hole, < 15 mm)",
    ("holes", "correct"): "detector: within 3 mm (accuracy threshold)",
    ("fit", "success"): "fit: success (< 5 mm, < 10°)",
    ("fit", "found"): "fit: pose found (< 15 mm, < 10°)",
    ("grasp", "lifted"): "grasp: lifted (> 50 mm, both jaws)",
}


def env1():
    e = dict(os.environ)
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        e[k] = "1"
    e.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    return e


def sh(cmd, log, ideal_dir=None):
    env = env1()
    if ideal_dir:
        env["IDEAL_DIR"] = ideal_dir
    with open(log, "w") as f:
        return subprocess.run(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, env=env).returncode


# ------------------------------------------------------------------ select
def select():
    from eval_holes import gt_openings, gt_top
    from frame_io import load_frame
    sdir = os.path.join(OUT, "screening", "ideal")                # IDEAL_DIR of the screening runs
    fdir = os.path.join(sdir, "pybullet")
    os.makedirs(fdir, exist_ok=True)
    found, screened, seed = [], {}, FIRST_SEED
    while len(found) < N_SEEDS:
        batch = list(range(seed, seed + WORKERS))
        seed += WORKERS

        def render(s):
            if not os.path.exists(os.path.join(fdir, f"seed_{s:03d}", "head", "meta.json")):
                sh([PY, "scene/render.py", "--backend", "pybullet", "--seed", str(s), "--out", fdir],
                   os.path.join(fdir, f"render_{s:03d}.log"))
        with ThreadPoolExecutor(WORKERS) as ex:
            list(ex.map(render, batch))
        vis = {}
        for s in batch:
            f = load_frame(os.path.join(fdir, f"seed_{s:03d}", "head"))
            spec, K = f["meta"]["spec"], f["K"]
            vis[s] = gt_top(gt_openings(spec, f["meta"]["camera_pose_world_ros"]), f["depth"], K)
        cand = [s for s in batch if vis[s]]
        ideal = {s: {} for s in cand}
        stages = ["holes", "fit", "grasp"]
        with ThreadPoolExecutor(WORKERS) as ex:               # ideal-depth stages of the visible seeds
            for (s, st), row in zip([(s, st) for s in cand for st in stages],
                                    ex.map(lambda a: ideal_row(*a, sdir), [(s, st) for s in cand for st in stages])):
                ideal[s][st] = row
        for s in batch:
            g = ideal.get(s, {})
            screened[s] = {"visible": vis[s],
                           "ideal_detector_correct": g["holes"].get("correct") if "holes" in g else None,
                           "ideal_fit_success": g["fit"].get("success") if "fit" in g else None,
                           "ideal_grasp": g["grasp"].get("status", "no fit") if "grasp" in g else None}
            sc = screened[s]
            ok = bool(vis[s]) and bool(sc["ideal_detector_correct"]) and bool(sc["ideal_fit_success"]) \
                and sc["ideal_grasp"] == "lifted"
            if ok and len(found) < N_SEEDS:
                found.append(s)
    last = found[-1]
    upto = [s for s in screened if s <= last and s not in found]
    rejected = {"no_visible_hole": [s for s in upto if not screened[s]["visible"]],
                "detector_wrong": [s for s in upto if screened[s]["visible"] and screened[s]["ideal_detector_correct"] is False],
                "fit_failed": [s for s in upto if screened[s]["visible"] and screened[s]["ideal_fit_success"] is False],
                "grasp_plan_infeasible": [s for s in upto if screened[s]["ideal_grasp"] in INFEASIBLE],
                "grasp_not_lifted": [s for s in upto if screened[s]["ideal_grasp"] not in (None, "lifted", *INFEASIBLE)]}
    for s in found:                                           # selected frames join the ideal reference set
        dst = os.path.join(IDEAL_DIR, "pybullet", f"seed_{s:03d}")
        if not os.path.exists(dst):
            shutil.copytree(os.path.join(fdir, f"seed_{s:03d}"), dst)
    with open(os.path.join(OUT, "screening", "screening.json"), "w") as f:
        json.dump({"rule": "top hole visible on ideal PyBullet depth (eval_holes.gt_top) and on ideal depth: "
                           "detector within 3 mm, fit success, grasp lifted",
                   "first_seed": FIRST_SEED, "last_seed": last, "selected": found, "rejected": rejected,
                   "screened": {str(s): v for s, v in screened.items()}}, f, indent=1)
    with open(os.path.join(OUT, "test_seeds.json"), "w") as f:
        json.dump(found, f)
    print(f"selected {len(found)} seeds in {FIRST_SEED}-{last}; rejected: "
          + ", ".join(f"{k} {len(v)}" for k, v in rejected.items()) + f"\n{found}")
    if BACKEND == "isaac":
        render_isaac(found)


def ideal_row(s, stage, ideal_dir):
    """Result row of one ideal-depth stage of seed s (screening); frames read from ideal_dir."""
    d = os.path.join(OUT, "screening", stage, f"seed_{s:03d}")
    if not result_files(d):
        os.makedirs(d, exist_ok=True)
        script = {"holes": ["perception/eval_holes.py"], "fit": ["perception/eval_bracket_fit.py", "--vis", "0"],
                  "grasp": ["robot/grasp_hole.py", "--videos", "0"]}[stage]
        sh([PY, *script, "--seeds", str(s), "--out", d], os.path.join(d, "log.txt"), ideal_dir)
    files = result_files(d)
    if not files:
        return {"status": "script failed"}
    with open(files[0]) as f:
        return json.load(f)[0]


def render_isaac(seeds):
    """Ideal Isaac frames of the selected seeds (one Isaac process per seed; skipped if present)."""
    todo = [s for s in seeds if not os.path.exists(os.path.join(IDEAL_DIR, "isaac", f"seed_{s:03d}", "head", "meta.json"))]
    logs = os.path.join(OUT, "screening", "isaac_logs")
    os.makedirs(logs, exist_ok=True)

    def render(s):
        rc = sh([PY, "scene/render.py", "--backend", "isaac", "--seed", str(s)], os.path.join(logs, f"render_{s:03d}.log"))
        print(f"isaac render seed {s}: {'ok' if rc == 0 else f'FAILED (rc {rc})'}", flush=True)
    with ThreadPoolExecutor(ISAAC_WORKERS) as ex:
        list(ex.map(render, todo))
    missing = [s for s in seeds if not os.path.exists(os.path.join(IDEAL_DIR, "isaac", f"seed_{s:03d}", "head", "meta.json"))]
    assert not missing, f"Isaac frames missing for seeds {missing} (logs: {logs})"


# ------------------------------------------------------------------ run
def test_seeds():
    with open(os.path.join(OUT, "test_seeds.json")) as f:
        return json.load(f)


def job_dir(cond, stage, seed):
    return os.path.join(OUT, "raw", cond, stage, f"seed_{seed:03d}")


def result_files(d):
    return glob.glob(os.path.join(d, "**", "results*.json"), recursive=True)


def job(cond, stage, seed):
    d = job_dir(cond, stage, seed)
    if result_files(d):
        return cond, stage, seed, "skipped"
    os.makedirs(d, exist_ok=True)
    n = 1 if cond == "0_ideal" else DRAWS
    cam = CONDITIONS[cond]
    if stage == "holes":
        cmd = [PY, "perception/eval_holes.py", "--seeds", str(seed), "--runs", str(n), "--backend", BACKEND,
               "--out", d, *cam]
    elif stage == "fit":
        cmd = [PY, "perception/eval_bracket_fit.py", "--seeds", str(seed), "--runs", str(n), "--vis", "0",
               "--backend", BACKEND, "--out", d, *cam]
    else:                                                     # PyBullet only (physics), whatever BACKEND
        cmd = [PY, "robot/grasp_hole.py", "--seeds", str(seed), "--run-ids", *map(str, range(n)),
               "--videos", "0", "--out", d, *cam]
    rc = sh(cmd, os.path.join(d, "log.txt"))
    return cond, stage, seed, "ok" if rc == 0 and result_files(d) else f"FAILED (rc {rc}, see {d}/log.txt)"


def run():
    seeds = test_seeds()
    if BACKEND == "isaac":
        render_isaac(seeds)
    jobs = [(c, st, s) for st in ("grasp", "fit", "holes") for c in CONDITIONS for s in seeds]   # slowest first
    failed = []
    with ThreadPoolExecutor(WORKERS) as ex:
        for i, (c, st, s, status) in enumerate(ex.map(lambda a: job(*a), jobs)):
            print(f"[{i + 1}/{len(jobs)}] {c} {st} seed {s}: {status}", flush=True)
            if status.startswith("FAILED"):
                failed.append((c, st, s))
    print(f"done, {len(failed)} failed jobs" + (f": {failed}" if failed else ""))


# ------------------------------------------------------------------ analyse
def load_rows(stage):
    from eval_holes import gt_openings
    from frame_io import ideal_frame
    meta = {}
    rows = []
    for cond in CONDITIONS:
        for s in test_seeds():
            files = result_files(job_dir(cond, stage, s))
            if not files:
                print(f"missing: {cond} {stage} seed {s}")
                continue
            with open(files[0]) as f:
                rr = json.load(f)
            for r in rr:
                r = {k: v for k, v in r.items() if k != "phase_log"}
                r["condition"] = cond
                r["bias_sign"] = int(np.random.default_rng((r["seed"], r["run"], 0)).choice([-1.0, 1.0]))
                if stage == "holes":
                    if not r.get("gt_top"):
                        print(f"note: {cond} seed {s} run {r['run']}: no visible top hole on {BACKEND} depth")
                    r["correct"] = bool(r.get("correct", False))
                    r["right_hole"] = bool(r.get("right_hole", False))
                    if r.get("matched"):                     # distance and viewing angle of the matched opening
                        if s not in meta:
                            fr = ideal_frame(BACKEND, s)
                            meta[s] = gt_openings(fr["meta"]["spec"], fr["meta"]["camera_pose_world_ros"])
                        c, n_cam, _ = meta[s][r["matched"]]
                        ray = c / np.linalg.norm(c)
                        r["hole_dist_mm"] = float(np.linalg.norm(c) * 1e3)
                        r["view_angle_deg"] = float(np.degrees(np.arccos(np.clip(abs(n_cam @ ray), 0, 1))))
                elif stage == "fit":
                    r["success"] = bool(r.get("success", False))
                    r["found"] = bool("t_err_mm" in r and r["t_err_mm"] < 15.0 and r["r_err_deg"] < 10.0)
                else:
                    r["lifted"] = bool(r.get("success", False))
                    if r.get("approach_err_deg") is not None:      # vs the nearest opening's normal; the inner
                        a = r["approach_err_deg"]                   # opening's normal is the same axis reversed
                        r["approach_err_deg"] = min(a, 180.0 - a)
                    r["status"] = r.get("status", "no fit" if not r.get("fitted") else "?")
                rows.append(r)
    return rows


def outcomes(rows, cond, metric, seeds):
    """{(seed, draw): bool}; the deterministic ideal outcome is repeated for every draw (pairing)."""
    d = {(r["seed"], r["run"]): r[metric] for r in rows if r["condition"] == cond}
    if cond == "0_ideal":
        d = {(s, k): d[(s, 0)] for s in seeds for k in range(DRAWS) if (s, 0) in d}
    return d


def clopper_pearson(k, n, a=0.05):
    from scipy.stats import beta
    lo = beta.ppf(a / 2, k, n - k + 1) if k > 0 else 0.0
    hi = beta.ppf(1 - a / 2, k + 1, n - k) if k < n else 1.0
    return lo, hi


def seed_bootstrap(o, seeds, B=2000):
    """95 % interval resampling whole seeds (draws of one seed are not independent)."""
    rng = np.random.default_rng(0)
    per = {s: [v for (ss, _), v in o.items() if ss == s] for s in seeds}
    per = {s: v for s, v in per.items() if v}
    keys = list(per)
    stats = []
    for _ in range(B):
        pick = rng.choice(len(keys), len(keys))
        vals = np.concatenate([per[keys[i]] for i in pick])
        stats.append(vals.mean())
    return tuple(np.percentile(stats, [2.5, 97.5]))


def mcnemar(a, b):
    from scipy.stats import binomtest
    keys = a.keys() & b.keys()
    n01 = sum(a[k] and not b[k] for k in keys)
    n10 = sum(b[k] and not a[k] for k in keys)
    p = binomtest(n01, n01 + n10, 0.5).pvalue if n01 + n10 else 1.0
    return n01, n10, p


def analyse():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    seeds = test_seeds()
    res, pdir = os.path.join(OUT, "results"), os.path.join(OUT, "plots")
    os.makedirs(res, exist_ok=True)
    os.makedirs(pdir, exist_ok=True)
    rows = {st: load_rows(st) for st in STAGES}
    for st, rr in rows.items():
        keys = sorted({k for r in rr for k in r})
        with open(os.path.join(res, f"rows_{st}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, keys)
            w.writeheader()
            w.writerows(rr)

    summary = []
    for (st, m), label in METRICS.items():
        ideal = outcomes(rows[st], "0_ideal", m, seeds)
        for cond in CONDITIONS:
            o = outcomes(rows[st], cond, m, seeds)
            if not o:
                continue
            k, n = sum(o.values()), len(o)
            lo, hi = clopper_pearson(k, n)
            blo, bhi = seed_bootstrap(o, seeds)
            row = {"stage": st, "metric": m, "label": label, "condition": cond, "k": k, "n": n, "rate": k / n,
                   "cp_lo": lo, "cp_hi": hi, "boot_lo": blo, "boot_hi": bhi}
            if cond != "0_ideal":
                row["ideal_only"], row["cond_only"], row["p_vs_ideal"] = mcnemar(ideal, o)
            if cond == "4_edge_axial":
                row["axial_only"], row["edge_axial_only"], row["p_vs_axial"] = \
                    mcnemar(outcomes(rows[st], "2_axial", m, seeds), o)
            summary.append(row)
    keys = ["stage", "metric", "label", "condition", "k", "n", "rate", "cp_lo", "cp_hi", "boot_lo", "boot_hi",
            "ideal_only", "cond_only", "p_vs_ideal", "axial_only", "edge_axial_only", "p_vs_axial"]
    with open(os.path.join(res, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(summary)
    for r in summary:
        extra = f"  p vs ideal {r['p_vs_ideal']:.3g}" if "p_vs_ideal" in r else ""
        extra += f"  p vs axial {r['p_vs_axial']:.3g}" if "p_vs_axial" in r else ""
        print(f"{r['label'][:44]:44s} {LABELS[r['condition']]:12s} {r['k']:4d}/{r['n']:<4d} "
              f"{100 * r['rate']:5.1f} %  [{100 * r['cp_lo']:.0f}-{100 * r['cp_hi']:.0f}]{extra}")

    # plots: bars = rate, error bars = 95 % exact interval; strict vs allowance side by side
    BLUE, ORANGE, INK, MUTED = "#2a78d6", "#eb6834", "#2b2b2b", "#8a8a85"
    conds = list(CONDITIONS)
    x = np.arange(len(conds))

    def bars(ax, metrics, colors, names):
        w = 0.8 / len(metrics)
        for i, (m, col, name) in enumerate(zip(metrics, colors, names)):
            rr = {r["condition"]: r for r in summary if (r["stage"], r["metric"]) == m}
            y = np.array([rr[c]["rate"] * 100 if c in rr else np.nan for c in conds])
            err = np.array([[100 * (rr[c]["rate"] - rr[c]["cp_lo"]), 100 * (rr[c]["cp_hi"] - rr[c]["rate"])]
                            if c in rr else [0, 0] for c in conds]).T
            pos = x + (i - (len(metrics) - 1) / 2) * w
            ax.bar(pos, y, w * 0.92, color=col, label=name, zorder=2)
            ax.errorbar(pos, y, yerr=err, fmt="none", ecolor=INK, elinewidth=1, capsize=3, zorder=3)
        ax.set_xticks(x, [LABELS[c] for c in conds], rotation=20, ha="right")
        ax.set_ylim(0, 105)
        ax.set_ylabel("success rate (%)")
        ax.grid(axis="y", color="#e4e3df", zorder=0)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(MUTED)
        ax.tick_params(colors=INK)

    n_note = f"{BACKEND}: {len(seeds)} test seeds × {DRAWS} draws (ideal: 1 draw, repeated); error bars: 95 % exact interval"
    for st, title in (("holes", "Top-hole detector"), ("fit", "Bracket fit")):
        ms = [k for k in METRICS if k[0] == st]
        fig, ax = plt.subplots(figsize=(8, 4.2))
        bars(ax, ms, [BLUE, ORANGE], ["hole detected", "within 3 mm"] if st == "holes"
             else ["strict (5 mm)", "15 mm allowance"])
        ax.set_title(f"{title}: success by depth condition", loc="left", color=INK)
        ax.legend(frameon=False, loc="lower left")
        fig.text(0.01, 0.01, n_note, fontsize=8, color=MUTED)
        fig.tight_layout(rect=(0, 0.03, 1, 1))
        fig.savefig(os.path.join(pdir, f"{st}_success.png"), dpi=150)
        plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    bars(ax, [("grasp", "lifted")], [BLUE], ["lifted"])
    ax.set_title("Grasp at the hole: lifted by depth condition", loc="left", color=INK)
    fig.text(0.01, 0.01, n_note, fontsize=8, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(pdir, "grasp_lifted.png"), dpi=150)
    plt.close(fig)

    # all stages in one figure: strict criteria (detector, fit) and lifted
    fig, ax = plt.subplots(figsize=(9, 4.2))
    bars(ax, [("holes", "right_hole"), ("fit", "success"), ("grasp", "lifted")], [BLUE, ORANGE, "#1baf7a"],
         ["hole detected", "fit success (5 mm, 10°)", "grasp lifted"])
    ax.set_title("Success at each stage of the downstream task", loc="left", color=INK)
    ax.legend(frameon=False, loc="lower left", fontsize=8)
    fig.text(0.01, 0.01, n_note, fontsize=8, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(pdir, "all_stages.png"), dpi=150)
    plt.close(fig)

    # bias check: hole position error vs distance and vs viewing angle (bias only, right hole found)
    rr = [r for r in rows["holes"] if r["condition"] == "5_bias" and r.get("right_hole") and "hole_dist_mm" in r]
    if rr:
        fig, axs = plt.subplots(1, 2, figsize=(9, 3.8), sharey=True)
        for sign, col, name in ((1, BLUE, "+2 %"), (-1, ORANGE, "−2 %")):
            q = [r for r in rr if r["bias_sign"] == sign]
            axs[0].scatter([r["hole_dist_mm"] for r in q], [r["pos_err_mm"] for r in q], s=24, color=col,
                           edgecolor="white", linewidth=1, label=name, zorder=3)
            axs[1].scatter([r["view_angle_deg"] for r in q], [r["pos_err_mm"] for r in q], s=24, color=col,
                           edgecolor="white", linewidth=1, label=name, zorder=3)
        d = np.linspace(min(r["hole_dist_mm"] for r in rr), max(r["hole_dist_mm"] for r in rr), 2)
        axs[0].plot(d, 0.02 * d, color=MUTED, lw=1.5, ls="--", label="2 % of distance", zorder=2)
        axs[0].set_xlabel("hole distance from camera (mm)")
        axs[1].set_xlabel("viewing angle from hole axis (°)")
        axs[0].set_ylabel("hole position error (mm)")
        axs[0].legend(frameon=False, fontsize=8)
        for ax in axs:
            ax.grid(color="#e4e3df", zorder=0)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        fig.suptitle("Depth bias only: hole position error", x=0.01, ha="left", color=INK)
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "bias_error_vs_distance_angle.png"), dpi=150)
        plt.close(fig)
    acc = accuracy(rows, seeds, res, pdir, plt, (BLUE, INK, MUTED))
    tables(summary, acc, seeds, res)
    table_png(os.path.join(res, "table.html"), os.path.join(pdir, "table.png"))
    print(f"\nresults: {res}\nplots:   {pdir}")


def median_boot(vals_by_seed, B=2000):
    """Median and its 95 % interval, resampling whole seeds."""
    rng = np.random.default_rng(0)
    keys = [k for k, v in vals_by_seed.items() if v]
    meds = []
    for _ in range(B):
        pick = rng.choice(len(keys), len(keys))
        meds.append(np.median(np.concatenate([vals_by_seed[keys[i]] for i in pick])))
    return tuple(np.percentile(meds, [2.5, 97.5]))


def accuracy(rows, seeds, res, pdir, plt, colors):
    BLUE, INK, MUTED = colors
    acc = []
    for st, val, subset, label in ACCURACY:
        for cond in CONDITIONS:
            rr = [r for r in rows[st] if r["condition"] == cond and r.get(subset) and r.get(val) is not None]
            if not rr:
                continue
            v = np.array([r[val] for r in rr], float)
            by_seed = {}
            for r in rr:
                by_seed.setdefault(r["seed"], []).append(r[val])
            lo, hi = median_boot(by_seed)
            acc.append({"stage": st, "value": val, "label": label, "condition": cond, "n": len(v),
                        "median": float(np.median(v)), "q25": float(np.percentile(v, 25)),
                        "q75": float(np.percentile(v, 75)), "p90": float(np.percentile(v, 90)),
                        "max": float(v.max()), "boot_lo": lo, "boot_hi": hi})
    with open(os.path.join(res, "accuracy.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(acc[0]))
        w.writeheader()
        w.writerows(acc)
    print("\naccuracy (median [95 % seed-bootstrap], IQR, n draws):")
    for a in acc:
        print(f"{a['label'][:44]:44s} {LABELS[a['condition']]:12s} {a['median']:6.2f} [{a['boot_lo']:.2f}-{a['boot_hi']:.2f}]"
              f"  IQR {a['q25']:.2f}-{a['q75']:.2f}  n {a['n']}")

    conds = list(CONDITIONS)
    fig, axs = plt.subplots(3, 2, figsize=(10, 9))
    for ax, (st, val, subset, label) in zip(axs.flat, ACCURACY):
        aa = {a["condition"]: a for a in acc if a["value"] == val and a["stage"] == st}
        x = np.arange(len(conds))
        y = np.array([aa[c]["median"] if c in aa else np.nan for c in conds])
        err = np.array([[aa[c]["median"] - aa[c]["boot_lo"], aa[c]["boot_hi"] - aa[c]["median"]] if c in aa else [0, 0]
                        for c in conds]).T
        ax.bar(x, y, 0.7, color=BLUE, zorder=2)
        ax.errorbar(x, y, yerr=err, fmt="none", ecolor=INK, elinewidth=1, capsize=3, zorder=3)
        for xi, c in zip(x, conds):
            if c in aa:
                ax.text(xi, aa[c]["boot_hi"], f"{aa[c]['median']:.1f}", ha="center", va="bottom", fontsize=7, color=MUTED)
        ax.set_xticks(x, [LABELS[c] for c in conds], rotation=25, ha="right", fontsize=8)
        ax.set_title(label, loc="left", fontsize=10, color=INK)
        ax.grid(axis="y", color="#e4e3df", zorder=0)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.text(0.01, 0.005, f"{BACKEND}: median over draws passing the 15 mm criterion (grasp: executed with a fit); "
             "error bars: 95 % seed-bootstrap interval of the median", fontsize=8, color=MUTED)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(os.path.join(pdir, "accuracy.png"), dpi=150)
    plt.close(fig)
    return acc


def table_png(html, png):
    """Picture of results/table.html (headless Chrome screenshot, light theme, trimmed); skipped without Chrome."""
    mac = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
    chrome = (shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
              or (mac if os.path.exists(mac) else None))
    if not chrome:
        print("no Chrome/Chromium found: table.png skipped")
        return
    tmp = png + ".full.png"
    subprocess.run([chrome, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--force-device-scale-factor=2",
                    "--window-size=2600,1400", "--blink-settings=preferredColorScheme=1",
                    f"--screenshot={tmp}", "file://" + os.path.abspath(html)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
    if not os.path.exists(tmp):
        print("table.png: screenshot failed")
        return
    from PIL import Image, ImageChops
    im = Image.open(tmp).convert("RGB")
    bbox = ImageChops.difference(im, Image.new("RGB", im.size, im.getpixel((0, 0)))).getbbox()
    if bbox:
        pad = 40
        im = im.crop((max(bbox[0] - pad, 0), max(bbox[1] - pad, 0), min(bbox[2] + pad, im.width), min(bbox[3] + pad, im.height)))
    im.save(png)
    os.remove(tmp)


def tables(summary, acc, seeds, res):
    """The live view's layout (conditions x metrics) as results/table.{md,csv,html}: success rate with k/n,
    95 % exact interval and McNemar p vs ideal; accuracy as median [95 % seed-bootstrap]."""
    conds, mets, accs = list(CONDITIONS), list(METRICS), [(st, v) for st, v, _, _ in ACCURACY]
    S = {(r["condition"], r["stage"], r["metric"]): r for r in summary}
    A = {(a["condition"], a["stage"], a["value"]): a for a in acc}
    alab = {(st, v): lab for st, v, _, lab in ACCURACY}
    short = {k: v.split(": ", 1)[1] if ": " in v else v for k, v in METRICS.items()}
    note = (f"{BACKEND}, {len(seeds)} test seeds × {DRAWS} noise draws (ideal: 1 draw per seed, repeated for the "
            f"pairing); seeds: every ideal stage succeeds. Success: rate (k/n), [95 % exact interval], p = exact McNemar vs ideal "
            "(paired draws). Accuracy: median [95 % seed-bootstrap interval] over draws passing the 15 mm criterion.")

    def sp(c, m):
        r = S.get((c, *m))
        if not r:
            return "–", ""
        txt = f"{100 * r['rate']:.0f} % ({r['k']}/{r['n']})"
        ci = f"[{100 * r['cp_lo']:.0f}–{100 * r['cp_hi']:.0f}]" + (f", p = {r['p_vs_ideal']:.2g}" if "p_vs_ideal" in r else "")
        return txt, ci

    def ap(c, k):
        a = A.get((c, *k))
        return f"{a['median']:.2f} [{a['boot_lo']:.2f}–{a['boot_hi']:.2f}]" if a else "–"

    md = [f"# Final sweep: {os.path.relpath(OUT, ROOT)}", "", note, "", "## Success by stage", "",
          "| condition | " + " | ".join(f"{METRICS[m].split(':')[0]}: {short[m]}" for m in mets) + " |",
          "|---|" + "---|" * len(mets)]
    for c in conds:
        md.append(f"| {LABELS[c]} | " + " | ".join(" ".join(x for x in sp(c, m) if x) for m in mets) + " |")
    md += ["", "Edge + axial vs axial (paired, exact McNemar): " + "; ".join(
        f"{short[m]} p = {S[('4_edge_axial', *m)]['p_vs_axial']:.2g}" for m in mets if ("4_edge_axial", *m) in S), "",
        "## Accuracy (median [95 % seed-bootstrap])", "",
        "| condition | " + " | ".join(alab[k] for k in accs) + " |", "|---|" + "---|" * len(accs)]
    for c in conds:
        md.append(f"| {LABELS[c]} | " + " | ".join(ap(c, k) for k in accs) + " |")
    with open(os.path.join(res, "table.md"), "w") as f:
        f.write("\n".join(md) + "\n")

    with open(os.path.join(res, "table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["condition"] + [f"{m[0]}_{m[1]}_{x}" for m in mets for x in ("rate", "k", "n", "cp_lo", "cp_hi", "p_vs_ideal")]
                   + [f"{k[0]}_{k[1]}_{x}" for k in accs for x in ("median", "boot_lo", "boot_hi")])
        for c in conds:
            row = [LABELS[c]]
            for m in mets:
                r = S.get((c, *m), {})
                row += [r.get("rate"), r.get("k"), r.get("n"), r.get("cp_lo"), r.get("cp_hi"), r.get("p_vs_ideal")]
            for k in accs:
                a = A.get((c, *k), {})
                row += [a.get("median"), a.get("boot_lo"), a.get("boot_hi")]
            w.writerow(row)

    color = {"right_hole": "var(--s1)", "correct": "var(--s2)", "success": "var(--s1)", "found": "var(--s2)",
             "lifted": "var(--s3)"}

    def cell(c, m):
        r = S.get((c, *m))
        if not r:
            return '<td class="small">–</td>'
        pct = 100 * r["rate"]
        _, ci = sp(c, m)
        return (f'<td><div class="rate" title="{r["k"]}/{r["n"]} draws"><div class="bar"><i style="width:{pct:.1f}%;'
                f'background:{color[m[1]]}"></i></div><span>{pct:.0f} %</span><span class="small">{r["k"]}/{r["n"]}'
                f'</span></div><div class="small">{ci}</div></td>')
    head = "".join(f"<th>{METRICS[m].split(':')[0]}<br>{short[m]}</th>" for m in mets)
    body = "".join(f"<tr><td>{LABELS[c]}</td>" + "".join(cell(c, m) for m in mets) + "</tr>" for c in conds)
    ahead = "".join(f"<th>{alab[k]}</th>" for k in accs)
    abody = "".join(f"<tr><td>{LABELS[c]}</td>" + "".join(f"<td>{ap(c, k)}</td>" for k in accs) + "</tr>" for c in conds)
    import watch
    css = watch.PAGE.split("<style>")[1].split("</style>")[0]
    html = (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" '
            f'content="width=device-width, initial-scale=1"><title>Sweep Results</title><style>{css}'
            f'main{{max-width:none;width:max-content}}.sub{{max-width:1400px}}</style></head>'
            f'<body><main><h1>Final sweep results</h1><div class="sub">{os.path.relpath(OUT, ROOT)} · {note}</div>'
            f'<h2>Success by stage</h2><div class="card scroll"><table>{"<tr><th>condition</th>" + head + "</tr>"}{body}'
            f'</table></div><h2>Accuracy</h2><div class="card scroll"><table>{"<tr><th>condition</th>" + ahead + "</tr>"}'
            f'{abody}</table></div></main></body></html>')
    with open(os.path.join(res, "table.html"), "w") as f:
        f.write(html)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["select", "run", "analyse", "all"])
    ap.add_argument("--n-seeds", type=int, default=N_SEEDS,
                    help="test seeds (default 30 = primary result; e.g. 100 = extended run)")
    ap.add_argument("--workers", type=int, default=WORKERS, help="parallel processes (default: CPU cores)")
    ap.add_argument("--backend", choices=["pybullet", "isaac"], default=BACKEND,
                    help="renderer of the perception frames (grasp is PyBullet in both)")
    ap.add_argument("--isaac-workers", type=int, default=ISAAC_WORKERS, help="Isaac renders at once (GPU)")
    ap.add_argument("--no-view", action="store_true", help="do not start / open the live web view")
    ap.add_argument("--port", type=int, default=8765, help="port of the live web view")
    a = ap.parse_args()
    N_SEEDS, WORKERS, BACKEND, ISAAC_WORKERS = a.n_seeds, a.workers, a.backend, a.isaac_workers
    OUT = os.path.join(ROOT, "output", "final_sweep" + ("_isaac" if BACKEND == "isaac" else "")
                       + ("" if N_SEEDS == 30 else f"_n{N_SEEDS}"))
    if N_SEEDS != 30:                                         # extended run: own outputs and ideal frames
        os.environ["IDEAL_DIR"] = os.path.join(OUT, "ideal")  # read by config.py, also in the subprocesses
    IDEAL_DIR = os.environ.get("IDEAL_DIR") or os.path.join(ROOT, "data", "ideal")
    os.makedirs(OUT, exist_ok=True)
    if not a.no_view:
        import watch
        print(f"live view: {watch.start(OUT, port=a.port)}", flush=True)
    if a.stage in ("select", "all"):
        select()
    if a.stage in ("run", "all"):
        run()
    if a.stage in ("analyse", "all"):
        analyse()
    if not a.no_view and sys.stdin.isatty():
        input("finished - press Enter to stop the live view ")
