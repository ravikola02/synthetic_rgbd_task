# Synthetic RGB-D of a manufacturing cell with a RealSense D435 depth model

Generates labelled synthetic RGB-D frames of an aluminium profile and an angle bracket on a table in front of a
Unitree G1 humanoid. The frames are seen by the G1's head camera, an Intel RealSense D435 (`d435_link`). The repo also runs a
downstream task on the depth: detect the bracket's top hole in 6D, fit the bracket CAD, and grasp the bracket
with the jaws approaching along the hole axis.

**Status: complete, with final results.** The scene, the ideal-depth reference set, the downstream pipeline and
the analytical D435 depth model (`camera_model/d435.py`) are done. Every script runs on ideal depth by default
and on the D435 model with `--camera d435`. The final experiment (7 depth conditions on 30 held-out test seeds)
is in [Final experiment](#final-experiment), with results in `output/final_sweep/results/` and examples
in `extracted_examples/`. A check of the camera model against real D435 depth (HAMMER dataset):
`camera_model/validate_hammer.py`. Algorithms and math: `docs/pipeline.md`, `docs/camera_model.md`.

## Setup

Step-by-step reproduction on a laptop (Linux or macOS, PyBullet only), from re-analysing the published results to a
full rerun: [REPRODUCE.md](REPRODUCE.md).

```bash
conda env create -f environment.yml      # CPU only; PyBullet backend
conda activate synthetic_rgbd
```

All scripts are run from the repo root. Everything needed for the PyBullet path is in the repo: part CAD,
robot model and the reference frames.

### Isaac Sim (optional)

The Isaac backend renders the same scene with Isaac Sim 5.1 (NVIDIA GPU, Linux). It is only used to
cross-check the PyBullet frames; the two backends agree to 0.00 mm median depth on the parts.

```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
export OMNI_KIT_ACCEPT_EULA=YES
export G1_USD=/path/to/g1_29dof_with_dex1_base_fix1.usd   # G1 + Dex1 USD, not in the repo (77 MB)
```

## Quick start (ideal depth)

```bash
python scene/render.py --seed 4 --out /tmp/frame      # one frame: rgb, depth, instance mask, meta
python scene/render.py --seed 4 --gui                 # look at the scene in the PyBullet GUI
python perception/eval_holes.py                       # hole detector on the 15 development seeds
python perception/eval_bracket_fit.py                 # 6D bracket fit on the 15 development seeds
python robot/grasp_hole.py                            # grasp the 15 development seeds; videos of the first 2
python perception/plot_hole_plane.py --seed 8 --run 22   # 3D: hole face as detected, ideal vs D435 x baseline vs tuned
```

Add `--backend isaac` to the render and the two evaluation scripts to use Isaac frames instead. An Isaac
render handles one seed per process, because Isaac exits the Python process when it closes.

### D435 camera model

`camera_model/d435.py` turns ideal depth into D435-like depth. It works on saved frames, independent of
the simulator. Three effects, each with a published number, applied in the order they arise in the camera:

1. **edge fattening:** at depth steps the foreground grows 2 px into the background, so gaps of 4 px or less
   close. No D435 value exists; the value is the R200's (Keselman et al. 2017 §4.2.3: gaps < 5 px merge) and
   has to be calibrated against real D435 depth;
2. **axial noise:** Intel's σ_z = z²·s/(f·B), s = 0.08 px (tuning guide p. 4), applied as disparity noise;
3. **depth bias:** a scale error at the datasheet's Z-accuracy limit (±2 %, Table 4-15), +2 % or −2 % with
   equal probability per noise draw;

plus the datasheet output rules: left invalid depth band (§4.5), Min-Z 195 mm (Table 4-12), 1 mm depth unit
(tuning guide §8a). These were tested and do not change the downstream results, but keep the frames D435-like.

```bash
python camera_model/d435.py                                     # noise levels and what each step does to a frame
python perception/eval_holes.py --camera d435 --runs 10         # 10 noise draws per development seed
python perception/eval_bracket_fit.py --camera d435 --runs 10
python robot/grasp_hole.py --camera d435 --seeds 45 --run-ids 0 --close-view   # a grasp from D435 depth
```

Ablations switch one part of the model: `--edge none`, `--axial ahn` (Ahn et al. 2019 noise, ~2.7× Intel's at
0.5 m) or `--axial none`, `--bias 0` (or a fixed `--bias 2` / `--bias -2`, in %), `--no-rules`. Noise draw r
of seed s uses the same random numbers, including the bias sign, in every script and model variant.
AI_LOGS.md (Q2, working log) lists the effects left out and why.

With the depth bias every position moves ~10 mm along the viewing ray, so the evaluation summaries also
count the "right hole" / "pose found" with that shift allowed, next to the strict 3 mm / 5 mm criteria.

**Mitigation: noise-aware detector.** `perception/hole_detector_tuned.py` is the same hole detector with its
thresholds scaled to the expected D435 noise (k·σ_z per pixel, k = 2.5 by default). It is reported separately
from the baseline comparison.

```bash
python perception/eval_bracket_fit.py --camera d435 --runs 10 --detector tuned [--k-sigma 3]
python robot/grasp_hole.py --camera d435 --detector tuned --seeds 8 --run-ids 22
```

Results go to `output/` (ignored by git, except the final results in `output/final_sweep/`).

### Expected results

| Step (ideal depth) | Result |
|---|---|
| Hole detector, 15 development seeds | 15/15; centre error median 0.70 mm, axis 0.71° |
| Hole detector, all seeds 0–200 with a visible top hole (74) | 66/74 correct; the 8 misses pick a round gap on the bracket's cover cap. No detections on the 127 scenes without a visible hole |
| Bracket fit, 15 development seeds | 15/15; translation median 0.34 mm, rotation 0.84° |
| Grasp, 15 development seeds | 15/15 lifted ~100 mm |

These are ideal-depth results on the development seeds. The D435 results are in [Final experiment](#final-experiment).

The development set was chosen so that every step succeeds with ideal depth (see below). The 15/15 results are
therefore the baseline for the camera model, not an accuracy measure.

## Final experiment

Does the D435 model change the outcome of hole detection, bracket fit and grasp? 7 depth conditions (ideal,
datasheet rules, axial noise, edge fattening, edge + axial, depth bias ±2 %, full model) on 30 held-out test seeds
(`TEST_SEEDS`: every ideal-depth stage succeeds, with a collision-free arm motion), 5 paired noise draws each. Protocol and hypotheses: AI_LOGS.md Q3 (working log).

```bash
python experiments/final_sweep.py all          # select seeds, run, analyse (see REPRODUCE.md for timings);
                                               # opens a live web view (--no-view to skip)
python experiments/run_seed.py --seed 432 --condition 6_full --run 0 [--gui]   # one case, same numbers
python experiments/extract_examples.py         # example figures + grasp videos -> extracted_examples/
```

Results: `output/final_sweep/results/table.md` (also `.csv`, `.html`, `plots/table.png`), plots in
`output/final_sweep/plots/`, per-draw rows in `results/rows_*.csv`, raw per-job JSON in `raw/`. Examples of
what the camera model does to the task, with grasp videos: `extracted_examples/README.md`.

| Condition | Hole detected | Fit, pose found (15 mm) | Grasp lifted |
|---|---|---|---|
| ideal | 100 % | 100 % | 100 % |
| datasheet rules | 97 % | 100 % | 77 % |
| axial noise | 63 % | 100 % | 70 % |
| edge fattening | 100 % | 100 % | 73 % |
| edge + axial | 39 % | 100 % | 78 % |
| depth bias ±2 % | 100 % | 100 % (~10.7 mm off) | 79 % |
| full model | 54 % | 99 % (~11.1 mm off) | 69 % |

The camera model bites at the detector (axial noise creates false holes; edge fattening makes it worse,
p < 1e-4); the CAD fit absorbs the detection errors, and the depth bias passes through it as a ~11 mm shift.
Every executed grasp lifted the bracket, also 11 mm off: all grasp failures are planning failures (IK, or an arm
motion through the robot's own body, which the planner rejects). The rules-only row is the control for the
grasp: it changes perception by < 1 mm, so its grasp drop (77 %) is the grasp planner's own sensitivity; compare
the camera effects on the grasp with it. The perception results were repeated on Isaac Sim depth
(`output/final_sweep_isaac/plots/`).

## Development set and reference data

`DEV_SEEDS = [4, 8, 23, 26, 27, 32, 37, 38, 45, 69, 73, 86, 92, 97, 112]` in `scene/config.py`. These are the
first 15 seeds where, with ideal depth:

1. the top hole is visible to the camera,
2. the detector finds that hole,
3. the bracket fit succeeds, and
4. the grasp lifts the bracket.

The selection rule and the rejection counts are in the comment above `DEV_SEEDS`.

`data/ideal/<backend>/seed_XXX/head/` holds the ideal frames of the development seeds for both backends. This is
the reference the camera model's depth is compared against. To regenerate it:

```bash
python scene/render.py --dev-seeds                                       # PyBullet, all 15
for s in 4 8 23 26 27 32 37 38 45 69 73 86 92 97 112; do
  python scene/render.py --backend isaac --seed $s; done                  # Isaac
```

Each frame folder contains:

| File | Content |
|---|---|
| `rgb.png` | colour image, 848×480 |
| `depth.npy` | float32 z-depth in metres, 0 = no return |
| `instance.npy` | per-pixel id: 0 background, 1 table, 2 robot, 3+ parts |
| `depth_vis.png` | colour-mapped depth for viewing |
| `meta.json` | intrinsics K, camera pose (ROS optical frame: +z forward, +x right, +y down) and the full scene spec, including the part poses used as ground truth |

## Layout

```
scene/           scene definition shared by both backends
  config.py        robot, table, placement regions, camera, DEV_SEEDS
  parts.py         part CAD, hole geometry, symmetries, stable resting poses
  materials.py     visual / physical properties
  spec.py          seed -> backend-independent scene spec
  frame_io.py      frame format, ideal reference set access
  render.py        CLI: render a seed with either backend
  backends/        pybullet_backend.py, isaac_backend.py
perception/      hole_detector.py, hole_detector_tuned.py (mitigation), bracket_fit.py,
                 eval_holes.py, eval_bracket_fit.py
robot/           grasp_hole.py: IK (with an arm-body collision check), grasp selection and execution in PyBullet
experiments/     final_sweep.py (final experiment), run_seed.py (one case, --gui), watch.py (live view),
                 extract_examples.py (example figures and videos)
extracted_examples/  examples from the final sweep (figures, grasp videos, README)
camera_model/    d435.py: analytical D435 depth model; __init__.py: shared --camera options and ablations;
                 validate_hammer.py: model vs real D435 depth (HAMMER sample, fetched to data/hammer/)
assets/cad/      part CAD (STEP) + cached stable poses
assets/robot/    G1 + Dex1 URDF and meshes
data/ideal/      ideal reference frames of the development and test seeds
docs/            pipeline.md and camera_model.md (algorithms and math); D435 datasheet, depth tuning guide, depth post-processing paper, depth testing methodology,
                 Ahn et al. 2019, Keselman et al. 2017, literature notes
report/          report.tex (the PDF report), make_figures.py (its figures -> report/figures/)
AI_LOGS.md      working log for the AI assistant (decision history, superseded results; not for human readers)
```

## Sources and modifications

**Parts.** item Industrietechnik CAD: profile 8 40×40 light (0.0.452.81) and angle bracket set 8 40×40
(0.0.670.11).
- The profile is cut to 200 mm.
- The nuts and screws are removed from the bracket set, its holes are drilled through at Ø8.5 mm, and its
  locating lugs are removed. The cover cap is kept.

The steps are recorded in the `scene/parts.py` docstring.

**Robot model.** `assets/robot/g1_dex1/` is Unitree's `g1_29dof_mode_15_with_dex1_1.urdf` from
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) (G1 with Dex1 parallel grippers). Changes:
- mesh paths are made relative;
- the `world` link and the rubber-hand links are removed;
- a `*_hand_base_link` frame is added on each gripper base, plus fingertip frames.

The grasp code uses `*_hand_base_link`. The torso camera frame `d435_link` comes from the original URDF.

**Camera.** Specifications, the axial noise model, the depth-bias limit and the output rules are taken from the
Intel RealSense D400 datasheet and the depth tuning guide in `docs/`. Edge fattening uses the R200 value of
Keselman et al., "Intel RealSense Stereoscopic Depth Cameras", CVPRW 2017 (§4.2.3). The axial-noise ablation uses
Ahn et al., "Analysis and Noise Modeling of the Intel RealSense D435 for Mobile Robots", UR 2019 (Eq. 2).
Measured subpixel values: Intel's depth post-processing paper and the online "Projectors for D400" page.
