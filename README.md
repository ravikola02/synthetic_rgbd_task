# Synthetic RGB-D of a manufacturing cell with a RealSense D435 depth model

Generates labelled synthetic RGB-D frames of an aluminium profile and an angle bracket on a table in front of a
Unitree G1 humanoid. The frames are seen by the G1's head camera, an Intel RealSense D435 (`d435_link`). The repo also runs a
downstream task on the depth: detect the bracket's top hole in 6D, fit the bracket CAD, and grasp the bracket
with the jaws approaching along the hole axis.

**Status:** the scene, the ideal-depth reference set, the downstream pipeline and the analytical D435 depth
model (`camera_model/d435.py`) are complete. Every script runs on ideal depth by default and on the D435
model with `--camera d435`.

## Setup

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
python perception/eval_holes.py                       # hole detector on the 15 test seeds
python perception/eval_bracket_fit.py                 # 6D bracket fit on the 15 test seeds
python robot/grasp_hole.py                            # grasp the 15 test seeds; videos of the first 2
```

Add `--backend isaac` to the render and the two evaluation scripts to use Isaac frames instead. An Isaac
render handles one seed per process, because Isaac exits the Python process when it closes.

### D435 camera model

`camera_model/d435.py` turns ideal depth into D435-like depth. It works on saved frames, independent of
the simulator. It only includes effects that Intel's documentation (datasheet + depth tuning guide) gives a
model for:

1. axial noise: Intel's σ_z = z²·s/(f·B), s = 0.08 px (tuning guide §3e), applied as disparity noise;
   `--axial ahn` swaps in the empirical D435 fit of Ahn et al. 2019 (ablation);
2. datasheet rules: left invalid depth band (datasheet §4.5), Min-Z 195 mm (Table 4-12), 1 mm depth unit
   (tuning guide §8a).

```bash
python camera_model/d435.py                                     # noise levels and what each step does to a frame
python perception/eval_holes.py --camera d435 --runs 10         # 10 noise draws per test seed
python perception/eval_bracket_fit.py --camera d435 --runs 10
python perception/eval_bracket_fit.py --camera d435 --axial ahn --runs 10   # ablation: Ahn et al. noise
python robot/grasp_hole.py --camera d435 --seeds 8 --run-ids 22             # a grasp from noisy depth
```

**Mitigation: noise-aware detector.** `perception/hole_detector_tuned.py` is the same hole detector with its
thresholds scaled to the expected D435 noise (k·σ_z per pixel, k = 2.5 by default). It is reported separately
from the baseline comparison.

```bash
python perception/eval_bracket_fit.py --camera d435 --runs 10 --detector tuned [--k-sigma 3]
python robot/grasp_hole.py --camera d435 --detector tuned --seeds 8 --run-ids 22
```

`--axial none` runs the datasheet rules alone and `--no-rules` the axial noise alone. Noise draw r of
seed s uses the same random numbers in every script and model variant. NOTES.md (Q2) lists the effects
left out and why.

Results go to `output/` (ignored by git).

### Expected results

| Step (ideal depth) | Result |
|---|---|
| Hole detector, 15 test seeds | 15/15; centre error median 0.70 mm, axis 0.71° |
| Hole detector, all seeds 0–200 with a visible top hole (74) | 66/74 correct; the 8 misses pick a round gap on the bracket's cover cap. No detections on the 127 scenes without a visible hole |
| Bracket fit, 15 test seeds | 15/15; translation median 0.34 mm, rotation 0.84° |
| Grasp, 15 test seeds | 15/15 lifted ~100 mm |

| Step (D435 model, 15 test seeds × 10 noise draws, PyBullet) | Result |
|---|---|
| Hole detector alone | 90/150 correct; most failures pick a false hole on the profile's flat top face |
| Bracket fit | 142/150; translation median 0.81 mm, rotation 0.96° |
| Bracket fit, Ahn et al. noise without the rules (ablation, `--axial ahn --no-rules`) | 4/150 (most draws: no hole found) |
| Hole detector / bracket fit with the noise-aware detector (mitigation, k = 2.5) | 144/150 / 150/150 |

The test set was chosen so that every step succeeds with ideal depth (see below). The 15/15 results are
therefore the baseline for the camera model, not an accuracy measure.

## Test set and reference data

`TEST_SEEDS = [4, 8, 23, 26, 27, 32, 37, 38, 45, 69, 73, 86, 92, 97, 112]` in `scene/config.py`. These are the
first 15 seeds where, with ideal depth:

1. the top hole is visible to the camera,
2. the detector finds that hole,
3. the bracket fit succeeds, and
4. the grasp lifts the bracket.

The selection rule and the rejection counts are in the comment above `TEST_SEEDS`.

`data/ideal/<backend>/seed_XXX/head/` holds the ideal frames of the test seeds for both backends. This is
the reference the camera model's depth is compared against. To regenerate it:

```bash
python scene/render.py --test-seeds                                       # PyBullet, all 15
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
  config.py        robot, table, placement regions, camera, TEST_SEEDS
  parts.py         part CAD, hole geometry, symmetries, stable resting poses
  materials.py     visual / physical properties
  spec.py          seed -> backend-independent scene spec
  frame_io.py      frame format, ideal reference set access
  render.py        CLI: render a seed with either backend
  backends/        pybullet_backend.py, isaac_backend.py
perception/      hole_detector.py, hole_detector_tuned.py (mitigation), bracket_fit.py,
                 eval_holes.py, eval_bracket_fit.py
robot/           grasp_hole.py: IK, grasp selection and execution in PyBullet
camera_model/    d435.py: analytical D435 depth model; __init__.py: shared --camera/--axial options
assets/cad/      part CAD (STEP) + cached stable poses
assets/robot/    G1 + Dex1 URDF and meshes
data/ideal/      ideal reference frames of the test seeds
docs/            D435 datasheet, depth tuning guide, Ahn et al. 2019, literature notes
NOTES.md         working notes: decisions, results, what did not work
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

**Camera.** Specifications and the noise model are taken from the Intel RealSense D400 datasheet and the depth
tuning guide in `docs/`; the ablation uses Ahn et al., "Analysis and Noise Modeling of the Intel RealSense D435
for Mobile Robots", UR 2019 (Eq. 2).
