# Reproducing the results on a laptop

Everything below runs on Linux, on the CPU with **PyBullet**, no GPU. Isaac Sim (NVIDIA GPU, Linux) is
only an optional renderer cross-check and is not needed for any result.

There are four levels. Pick the one that fits the time you have:

| Level | What it shows | Time on an 8-core laptop |
|---|---|---|
| 1. Re-analyse the published results | tables and plots rebuilt from the per-draw results in the repo | < 1 min |
| 2. One case | one seed / condition / noise draw end to end, same numbers as the sweep | ~1 min |
| 3. Full rerun of the final experiment | all 7 conditions × 30 test seeds × 5 draws × 3 stages | ~1.5–3 h |
| 4. Full rerun including seed selection | as 3, plus re-screening seeds 201–557 for the test set | +20–40 min |

## Setup

```bash
git clone https://github.com/ravikola02/synthetic_rgbd_task.git
cd synthetic_rgbd_task
conda env create -f environment.yml          # Python 3.11, CPU only
conda activate synthetic_rgbd
```

### Check the install

```bash
python camera_model/d435.py                  # prints the D435 noise levels; no rendering
python scene/render.py --seed 432 --out /tmp/frame   # renders one frame (rgb, depth, instance mask, meta)
```

The first run converts the part CAD (STEP) into meshes in `assets/meshes/` (a few seconds, once). All
commands are run from the repo root.

**Optional: Chrome / Chromium** is used only to make `plots/table.png` (a picture of the results table). It is
found on the PATH (`google-chrome`, `chromium` or `chromium-browser`); without it that one picture is
skipped and everything else runs.

## Level 1: re-analyse the published results (< 1 min)

The per-draw results of the final experiment are in the repo (`output/final_sweep/raw/`). Rebuild the
statistics, tables and plots from them:

```bash
python experiments/final_sweep.py analyse --no-view
git diff --stat output/final_sweep/results/  # the tables should be unchanged
```

Results: `output/final_sweep/results/table.md` (also `.csv`, `.html`), `summary.csv`, `accuracy.csv`,
`rows_*.csv`; plots in `output/final_sweep/plots/`.

## Level 2: one case (~1 min)

```bash
python experiments/run_seed.py --seed 432 --condition 6_full --run 0          # headless
python experiments/run_seed.py --seed 432 --condition 6_full --run 0 --gui    # watch the grasp (PyBullet GUI)
python experiments/run_seed.py --seed 432 --condition 0_ideal --video         # grasp video (mp4)
```

Conditions: `0_ideal 1_rules 2_axial 3_edge 4_edge_axial 5_bias 6_full`. `--run` is the noise draw (0–4 in the
sweep). This is example 05 of `extracted_examples/` (under the full D435 model the detector picks a false hole,
the pose is 10 mm off, and the bracket is still lifted). The printout should match the sweep's row for seed 432,
draw 0: "detector: wrong hole, position error 277.8 mm", "fit: pose found ..., t 10.2 mm", "grasp: lifted".
Outputs: `output/run_seed/`.

The example figures and videos can be rebuilt with `python experiments/extract_examples.py` (~5–10 min; add
`--no-video` for figures only, ~2 min).

## Level 3: full rerun of the final experiment (~1.5–3 h)

Keep the published results aside, start a fresh sweep folder with the same 30 test seeds, run, analyse, and
compare:

```bash
mv output/final_sweep output/final_sweep_published
mkdir -p output/final_sweep
cp output/final_sweep_published/test_seeds.json output/final_sweep/
python experiments/final_sweep.py run        # 630 jobs in parallel; opens a live view in the browser
python experiments/final_sweep.py analyse
diff output/final_sweep_published/results/table.md output/final_sweep/results/table.md
```

- **Parallelism.** By default it uses one process per CPU core (`--workers N` to change; fewer if the laptop
  gets hot or runs out of memory: a grasp process peaks at ~0.65 GB). Each noise draw is seeded by (seed, draw), so the
  number of processes does not change any result.
- **Time.** The whole sweep is about 8–10 CPU-hours (per draw: detector ~6 s, fit ~7 s, grasp ~16 s). That is
  ~1.5–2 h on 8 cores, ~3–4 h on 4 cores. Ideal depth runs one draw per seed.
- **Live view.** The sweep starts a local web page (http://127.0.0.1:8765/) with progress, running success
  rates and, after `analyse`, the plots. `--no-view` skips it. It can also be opened separately:
  `python experiments/watch.py --open`.
- **Resuming.** If it is interrupted, run the same command again: finished jobs are skipped.
- **Disk.** ~15 MB of results and logs.

**What to expect.** On the same machine the numbers are identical. On another machine or OS, the perception
results (detector, fit) should match: the noise comes from NumPy's seeded generator, and the detector's RANSAC
is seeded too. The grasp runs a physics simulation, and floating-point differences between platforms can change
the outcome of a few grasps, because the grasp planner is sensitive to ~1 mm changes (see the "rules" control in
the README). A grasp rate a few draws away from the published one is expected; the pattern across conditions
should hold.

## Level 4: including the seed selection (+15–30 min)

```bash
mv output/final_sweep output/final_sweep_published
python experiments/final_sweep.py all        # screen seeds from 201 up, then run and analyse
```

Screening renders seeds from 201 upward and keeps the first 30 for which every ideal-depth stage succeeds (top
hole visible, detector within 3 mm, fit success, grasp lifted with a collision-free arm motion). It should select the same 30 seeds as
`TEST_SEEDS` in `scene/config.py` (seeds 201–557; screening details in `screening/screening.json`). On another
platform an ideal grasp could flip for the reason above and change the list; in that case use Level 3, which
fixes the published seeds.

## Optional: camera model vs real D435 depth (HAMMER)

```bash
python camera_model/validate_hammer.py       # fetches a 45 MB sample once (data/hammer/), then analyses (~1 min)
```

Downloads 108 frames of the HAMMER dataset (real D435 depth + ground truth, CC BY-NC-SA 4.0) by HTTP range
requests from its 170 GB archive, applies the model to the ground truth and compares noise, bias, invalid pixels
and edge behaviour (`output/hammer/summary.md`, `hammer_vs_model.png`; report Section 5).

## Optional: Isaac Sim cross-check (Linux + NVIDIA GPU)

```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
export OMNI_KIT_ACCEPT_EULA=YES
export G1_USD=/path/to/g1_29dof_with_dex1_base_fix1.usd     # G1 + Dex1 USD, not in the repo
python experiments/final_sweep.py all --backend isaac       # output/final_sweep_isaac/
```

The seeds are still screened on PyBullet (same scenes for both renderers). Isaac renders the 30 test seeds one at
a time (~75 s each on the GPU), and the detector and fit then run on the Isaac depth. The grasp is implemented in
PyBullet only, so its numbers are the PyBullet ones in both cases.
