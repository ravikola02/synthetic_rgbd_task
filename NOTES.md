# Working notes: answers to task.md, built up as we go

Status legend: ✅ decided · 🟡 proposed, not yet confirmed · ⬜ open

---

## Project structure
```
scene/         config.py (layout, camera, TEST_SEEDS) · parts.py (CAD, hole geometry) · materials.py
               spec.py (scene description) · frame_io.py (frame format, ideal dataset) · render.py (CLI)
               backends/pybullet_backend.py, isaac_backend.py
camera_model/  d435.py (analytical D435 model: simulate()) · __init__.py (--camera/--axial options)
perception/    hole_detector.py · hole_detector_tuned.py (noise-aware thresholds, mitigation)
               bracket_fit.py · eval_holes.py · eval_bracket_fit.py
robot/         grasp_hole.py (G1 + Dex1 grasp normal to the hole, PyBullet)
assets/cad/    item 40×40 STEP + cached stable poses · assets/meshes/ (OBJ exported from CAD)
data/ideal/<backend>/seed_XXX/head/   ideal reference frames of the 15 TEST_SEEDS (PyBullet + Isaac)
output/        evaluation results, overlays, grasp videos (not in git)
docs/          datasheet, tuning guide (BKM), Ahn et al. 2019, literature notes
```
- **Ideal reference set** (user, 2026-11-04): `data/ideal/{pybullet,isaac}/seed_XXX/head/` (rgb, depth, instance, meta) for `TEST_SEEDS = [4, 8, 23, 26, 27, 32, 37, 38, 45, 69, 73, 86, 92, 97, 112]` = first 15 seeds where, with ideal depth, the top hole is visible, the detector alone finds it, the fit succeeds and the grasp lifts ~100 mm (seeds 0–112 checked; rejected: 69 no visible hole, 4 wrong hole, 20 unreachable, 4 pushed/not lifted (12, 42, 54, 63), 1 slipped (20)). Every stage is 15/15 on this set by construction: it is the baseline the camera model degrades. `scene/render.py --backend pybullet --test-seeds` regenerates PyBullet; Isaac one seed per process.

**Results with ideal depth (PyBullet unless stated):**
| Step | Result |
|---|---|
| Top-hole detector alone ("most upward" candidate), all 201 seeds 0–200 (not the test set) | 74 scenes with a visible top hole: correct 66/74 (89 %), never no-detection; the 8 wrong (17, 21, 52, 65, 113, 132, 189, 199) all pick the round gap on the bracket's cap (~22 mm from any opening). 127 scenes without a visible hole: no top hole returned (0 false detections). Correct ones: position error median 0.64 mm (p90 1.22, max 1.84), axis 0.68° (p90 1.11, max 1.47); diameter median 7.3 mm vs true 8.5 (p10–p90 6.0–8.1: edge pixels lost at ~4–6 px across). Results in `output/hole_detector/{all_visible,no_visible_hole}_seeds_0-200/` |
| Bracket fit (tries every upward candidate, keeps the best fit) | test set: 15/15 (by construction); translation median 0.34 mm (max 0.56), rotation 0.84° (max 1.28°), ADD-S 0.50 mm. Detector alone on the test set: 15/15, position 0.70 mm, axis 0.71° |
| Same on the Isaac reference set | detector 15/15 (position 0.70 mm, axis 0.72°), fit 15/15 (translation 0.32 mm, rotation 0.65°); Isaac − PyBullet depth on part pixels: per-seed median 0.00 mm for all 15 |
| Grasp at the hole, approach normal to it (default URDF TCP, squeeze 4 mm) | test set 15/15 lifted 99–102 mm (by construction). Over all 40 candidates in 0–112 that passed detection + fit: 15 lifted, 20 unreachable (IK), 4 pushed/not lifted (12, 42: a gripper part not covered by the finger model pushes the bracket ~25 mm on approach; also 54, 63), 1 slipped at 51 mm (20, 3–4 N). Jaw gain 1.0 instead of 0.1: no change. Finger swept-volume collision check added (did not catch 12/42) |

## Q1. Reference camera and pipeline
**Camera:** ✅ Intel RealSense **D435** (active IR stereo), the head camera of the Unitree G1 (`d435_link` in the unitree_ros G1 URDF: on torso_link at xyz (0.058, 0.018, 0.430) m, pitched 47.6° down; checked in `assets/robot/g1_dex1/g1_dex1.urdf`).
- Why: it is the camera on the real robot used as the platform (the G1 carries a D435i in its head: Weston Robot G1 user guide, westonrobot.github.io/docs-legacy/robot_user_guide/unitree/g1.html; Unitree sells it as the G1 depth camera), so the setup is defensible. Intel's datasheet and tuning guide document it. Its stereo failure modes (texture, specularity, occlusion) are what metal parts provoke.
- Key specs (docs/): B = 50 mm; depth FOV H87°/V58° (HD); OV9282 imagers, 1280×800, **global shutter**; wide 850 nm VCSEL projector (H90°/V63°, 360 mW); recommended depth resolution 848×480 (f ≈ 447 px); Min-Z 195 mm @848×480.
- Expected RMS depth noise (z²·0.08/(f·B), 848×480): 0.6 mm @0.4 m, 0.9 mm @0.5 m, 1.3 mm @0.6 m, 3.6 mm @1.0 m.

**Renderer / simulator:** ✅ **NVIDIA Isaac Sim 5.1** (RTX rendering, PhysX) and **PyBullet**, sharing one scene description.
- Isaac: physically based rendering of metal, physics + robot/gripper models, standard in industrial robotics research. Cost: needs an RTX GPU and a large install, so reviewers may not run it on a laptop.
- PyBullet: CPU only, ~2 s per frame, same scene and output format; lets reviewers run everything on a laptop. Limitation: TinyRenderer has no PBR or reflections (no material effects). Development in PyBullet, confirmation in Isaac: the two agree on part depth to 0.00 mm (median, all 15 test seeds).
- ✅ The camera model works on saved frames (`camera_model/d435.py`), so it does not depend on the simulator.
- Environment: conda env at `/mnt/shared_data/envs/synthetic_rgbd` (Python 3.11, `isaacsim[all,extscache]==5.1.0` from pypi.nvidia.com, installed with uv; Isaac needs `OMNI_KIT_ACCEPT_EULA=YES`). `environment.yml` for the CPU / PyBullet path.
- Structure: `scene/spec.py` makes a plain-data scene description per seed; `scene/materials.py` is one material table; backends in `scene/backends/` turn the spec into a frame; `scene/frame_io.py` defines one output format. CLI: `scene/render.py --backend pybullet|isaac --seed N`.
  - Parts placed analytically at rest and kept static by default (`--settle` for physics): the same seed gives an identical scene in both backends.
  - Mass from mesh volume × density: profile 355 g (aluminium 2700 kg/m³), bracket 182 g (zinc 6600 kg/m³; overestimate, the cap is plastic).
- Alternatives considered: BlenderProc (one Python script, CPU, no robot), pyrender + trimesh (simplest, no realistic metal).

**Labels produced:** ✅ per frame: RGB, ideal z-depth (m), instance ids (0 background, 1 table, 2 robot, 3+ parts), and meta.json with intrinsics K, camera pose (ROS optical frame) and the 6D part poses (ground truth).

**Scene:**
- Unitree G1 (fixed base) with Dex1 parallel grippers; D435 at `d435_link`, 848×480, HFOV 87°.
- Table sized to G1 reach: 0.40 × 0.60 m, top 0.75 m above the floor, centre x = 0.34 m in front of the pelvis. From an IK reachability map (both arms, 9 approach directions, TCP at part height): best reach at table height 0.75 m and x = 0.20–0.30 m (44–67 % of directions reachable; x ≥ 0.35 m only 11–33 %; tables at 0.80 / 0.85 m not better). Bracket placed in x 0.24–0.30 m, y ±0.12 m; profile further out (x 0.37–0.41 m) to stay in view. Parts at 0.47–0.68 m from the camera.
- ✅ Observation / ready arm pose (`OBSERVE_ARM_POSE`): arms raised out to the sides (shoulder pitch 0.3, roll ±0.8, yaw ±0.5, elbow 0), grippers ~24 cm above the table beside it; from a grid search: no robot pixels in the head camera, no self-collision. A real robot looks before it reaches.
- ✅ Parts: **real manufacturer CAD from item (Line 8, 40×40, slot 8)**, STEP files in `assets/cad/`, loaded with trimesh + cascadio (OpenCascade), exported once to OBJ for both backends:
  - Profile **item 0.0.452.81** (Profile 8 40×40 light, natural = clear-anodized aluminium; item's export was the full 6 m bar, cut to 200 mm): 40 × 40 × 200 mm, 355 g.
  - Bracket **item 0.0.670.11** (Angle Bracket Set 8 40×40 with grey cover cap), reduced to **bracket + cap with free holes**. item's STEP fuses bracket, cap, 2 M8 button-head screws and 2 T-slot nuts into one solid. Changed once with OpenCascade booleans (no script kept; steps recorded in `scene/parts.py`): cut everything beyond the outer faces (nuts, screw tips, item's small centring lugs: a simplification), drill both Ø8.5 holes through the 5 mm legs (open into the cap's hollow, 21 mm deep), cut both screw heads; cap kept (not separable in the export). Result: one closed solid, 40 × 42.2 × 42.2 mm. Original STEP kept unchanged. **Not aluminium**: die-cast zinc, powder-coated RAL 9006 (satin), so expect it to be easier for the D435 than the anodized profile.
  - The CAD frame is the object frame for the 6D pose labels. Leg-swap symmetry (180° about (0, 1, 1)/√2) holds to 0.4 mm.
  - Resting poses from `trimesh.compute_stable_poses` (convex hull), sampled by probability per seed; poses below p = 0.02 dropped (e.g. profile standing on its end). Profile: 4 side poses (p = 0.25 each). Bracket: 5 poses (p = 0.32, 0.23, 0.23, 0.11, 0.11).
  - What didn't work: (1) the part sampler looped forever with a large minimum part distance (fixed by resampling both parts jointly); (2) stable-pose computation takes ~1 min per part, so the results are cached as `.npz` next to the STEP files.
- Materials: `scene/materials.py` (visual: rgba / metallic / roughness, used by Isaac's UsdPreviewSurface; physical: density, friction). Candidate NVIDIA MDL materials for Isaac: Base/Metals `Aluminum_Anodized`, `Aluminum_Polished` (not used yet). Lighting: dome + overhead area light.
- Simplification: convex-hull colliders (fine for resting parts, approximate for grasp contacts).

## Q2. Camera imperfections: included / left out / why
**Open issue (depth origin):** the D435 reports depth from the **left IR imager**, 17.5 mm from the centreline of the tripod hole (¼-20) (datasheet Table 4-20), and from the front cover glass (−4.2 mm, Table 4-17). Our camera sits at the URDF `d435_link`; to check which point that is (a 17.5 mm lateral offset is about 13 px at 0.6 m; it decides on which side occlusion shadows appear).

**Literature notes:** `docs/literature_d435_simulation.md` (online: https://claude.ai/artifact/NDdykcHzKFRfK1Si1WQWwF) (physics-based IR stereo sims: Zhang et al. T-RO 2023 / SAPIEN, DREDS; analytic: Ahn et al. 2019 D435; Isaac Sim 5.1 `SingleViewDepthSensor`; robot-learning noise stacks; validation data: HAMMER (real D435 + GT on reflective objects)).

**Approach:** ✅ (user, 2026-11-04) **analytical noise models**: corrupt the ideal depth with closed-form models, one effect at a time. Why (user): analytical models are **testable** and transparent: each effect can be switched on alone and its effect on the downstream step measured, and the models come ready from the camera documentation, so we do not have to study, develop and verify a simulator or a network first. Rejected:
- **Physics-based IR stereo simulation** (render the IR pair with the projector pattern, run stereo matching; Zhang et al. T-RO 2023 / SAPIEN, DREDS): the most faithful option, but a project of its own (IR materials, projector pattern, a D4-like matcher whose 40+ parameters Intel does not document, BKM §4a, §4c), and it would itself need validation against real D435 data we do not have. Out of scope and beyond what can be understood and verified in this assignment.
- **Learned noise models**: need real D435 depth of similar scenes to train or validate on, which we do not have. Checked 2026-11-04 whether a ready D435 one exists: **no released learned D435 noise generator.**
  - Closest: ByteDance Seed, "Manipulation as in Simulation" (arXiv 2509.02530, 2025): per-camera (incl. D435) *hole* model (DINOv2 + DPT, valid/missing per pixel from RGB) and *value* model (Depth Anything V2, affine-invariant, plus a guided filter for scale and hand-crafted rules for high-frequency noise), learned from their real ByteCameraDepth data (RGB + raw depth, no ground truth). Released: only the Camera Depth Models (denoisers: RGB + raw D435 depth → clean metric depth, weights CC BY-NC 4.0; github.com/ByteDance-Seed/manip-as-in-sim-suite has inference code only). The noise generators are not released.
  - Other learned work is not a D435 model: Sweeney, Izatt, Tedrake ICRA 2019 (CNN predicting dropout pixels only, structured-light (Kinect-type) data); Planche & Singh ICCV 2021 (differentiable simulation of structured-light sensors, Kinect V1). DREDS/SwinDRNet and ActiveZero restore or estimate depth rather than corrupt it.

**Scope of the camera model ✅ (user, 2026-11-04): only effects that the D435 documentation lets us model. The documentation = datasheet + Intel's tuning guide (BKM), which the user counts as part of the datasheet** (task.md: "its datasheet and published information are sufficient"). Full pass through the datasheet by the AI agent, checked against the PDF text (`docs/RealSense-D400-Series-Datasheet-Mar-2026.pdf`, doc 337029-017; D43x / D430 columns; page numbers are the document's own). Everything else in the table below is left out, with the reason "no datasheet model".

| # | Item | Datasheet reference | D435 value | Kind | In the model? |
|---|---|---|---|---|---|
| A1 | Left invalid depth band (non-overlapping views; left imager is the matching reference) | §4.5, Fig. 4-2, p. 82–83 | width = B·F/Z px → 45 px at 0.5 m (848×480) | **analytical formula** | include (no effect here: parts are central) |
| A2 | Depth FOV at distance Z (same geometry as A1) | §4.4, p. 81 | Depth FOV = HFOV/2 + atan(tan(HFOV/2) − B/Z) | **analytical formula** | covered by A1 (same effect) |
| B1 | Min-Z | Table 4-12, p. 84 | 195 mm at 848×480 | rule (cut-off) | include (no effect: parts at 0.47–0.68 m) |
| B2 | Depth = Z perpendicular to the imager plane, not range | §2.3, p. 18; Fig. 2-2, p. 19 | — | definition | already so (z-depth) |
| B3 | Depth origin = left imager centre | §4.8.1, Table 4-20, p. 91 | 17.5 mm from the tripod-hole (¼-20) centreline (D43x) | geometry | needed to place A1 and occlusion correctly (open issue: check `d435_link`) |
| B4 | Depth start point (Z = 0 plane) | §4.8, p. 86; Table 4-17, p. 89 | front cover glass, Z' = −4.2 mm (D435/D435i) | geometry | check against `d435_link` (4.2 mm offset in z if wrong) |
| B5 | Rectified, undistorted depth | §3.3.1 "image rectification", p. 23–24; imager distortion ≤ 1.5 %, Table 3-13, p. 40–41 | — | rule | justifies the ideal pinhole K, no lens distortion |
| B6 | Global shutter depth imagers (OV9282, 1280×800) | Table 3-13, p. 40 | global | rule | justifies leaving out rolling shutter / motion |
| B7 | Baseline, FOV | Table 3-11, p. 39; Table 3-49, p. 65 (D435 camera) | B = 50 mm; depth FOV H87° / V58° (HD; specified at 2 m, ±3° tolerance: Table 3-49 notes) | parameters | used (f = 447 px from 87°) |
| C1 | Z-accuracy (absolute error) | Table 4-15, p. 85 | ± 2 % | **limit only** | not modelled; quote as a limit (optional sensitivity sweep = own assumption) |
| C2 | RMS error / spatial noise vs best-fit plane | Tables 4-13, 4-15, p. 84–85 | ≤ 2 % | **limit only** | modelled by the Intel BKM formula (0.9 mm ≪ 10 mm limit) |
| C3 | Temporal noise | Table 4-15, p. 85 | ≤ 1 % | **limit only** | not modelled (single frames) |
| C4 | Fill rate | Table 4-15, p. 85 | ≥ 99 % | **limit only** | not modelled (no rule for where invalid pixels occur) |
| — | KPI test conditions (apply to C1–C4) | Table 4-15 notes, p. 85–86 | ≤ 2 m, central 80 % ROI, HD resolution, texture-less white target, laser 150 mW, auto exposure, factory-validated | context | the limits are for a flat white wall, not metal parts |
| D1 | Repetitive patterns "may cause false depth"; specular reflections "may cause image saturation" (D400 vs D400f) | Table 3-50, p. 66 | qualitative | **qualitative only** | not modelled (no datasheet model); relevant to the profile's T-slots and metal |
| D2 | Laser power (0–360 mW, 30 mW steps), exposure, gain, preset | Table 4-23, p. 92–93; projector 360 mW, 850 nm, H90°/V63°, Table 3-17, p. 42 | settings | settings | state the assumed defaults (laser 150 mW, auto exposure, default preset), no effect model |

Full pass through the tuning guide by the AI agent (`docs/BKM-Tuning-D400-Depth-Cameras.pdf`, Grunnet-Jepsen, Sweetser, Woodfill, Rev 2.0; section numbers are the guide's own):

| # | Item | BKM reference | D435 value | Kind | In the model? |
|---|---|---|---|---|---|
| T1 | Axial RMS depth error | §3e | σ_z = z²·s/(f·B); s < 0.1 px on a well-textured target (towards 0.05), > ~0.2 px means recalibrate (§3c); ~30 % smaller s with the laser off | **analytical formula** | ✅ main model, s = 0.08 (`camera_model/d435.py`) |
| T2 | Min-Z from the disparity search range | §5b | MinZ = f·B/126 → disparities > 126 px are invalid; BKM: ~168 mm at 848×480 (datasheet Table 4-12: 195 mm) | **analytical formula** | include as the disparity cut-off (no effect here) |
| T3 | Depth unit (quantisation) | §8a–b | 16-bit depth, 1 mm unit by default | rule | include (round to 1 mm; ~0.3 mm RMS) |
| T4 | Disparity shift (range window) | §5c | default 0 → MinZ to infinity | rule | default 0, nothing to model |
| T5 | No post-processing by default | §7a | raw stream; filters (decimation, temporal, edge-preserving, hole filling) are left to the app | rule | justifies modelling the raw depth |
| T6 | Multiple D4xx cameras do not interfere | §6b | — | rule | justifies leaving out interference (one camera anyway) |
| T7 | Focus | §8b-i | focused 50 cm to infinity, defocus well below 20 cm | rule | no effect (parts at 0.47–0.68 m) |
| T8 | "Egg carton" bumps (spatially correlated noise) | §3e | no correlation length | **qualitative only** | not modelled; noise is per-pixel independent |
| T9 | Presets: High Density (fewer holes) vs High Accuracy (only high-confidence depth, "false depth worse than no depth") | §4a–c | 40+ parameters, "no future descriptions" | **qualitative only** | not modelled; state the default preset as the assumption |
| T10 | Close range: left and right cameras see different sides of an object → no depth | §8b-i | — | **qualitative only** | the mechanism behind occlusion shadows; not a datasheet model |
| T11 | Repetitive structures → ambiguous match, zero depth via DSSecondPeakThreshold | §14 | — | **qualitative only** | not modelled (T-slots of the profile) |
| T12 | Exposure / gain, laser power (150 mW nominal), laser speckle, glare, sunlight, glossy white backgrounds | §2, §3a, §9e, §12, §15 | — | **qualitative only** | not modelled (indoor cell; assume good exposure, 150 mW) |

Inconsistencies in the two documents: imager HFOV 90° (datasheet Table 3-13) vs 91.2° (BKM table) vs depth FOV 87° (datasheet Table 3-11); Min-Z 195 mm (datasheet Table 4-12; also BKM §11c: "about 19.5 cm") vs ~168 mm (BKM §5b, formula f·B/126 with f ≈ 424 px, i.e. HFOV ≈ 90°). None of them matters at our 0.47–0.68 m.

**Modelled effects (all others left out: no model in the D435 documentation):** T1 axial noise (main result), plus the deterministic rules A1 left invalid band, B1/T2 Min-Z, T3 1 mm quantisation, with geometry B2–B5 (z-depth, left-imager origin, front-glass start point, rectified pinhole). Ablation outside the documentation: Ahn et al. 2019.
Side note: the §4.4 formula with the imager HFOV of 90° (Table 3-13) gives a depth FOV of 87.0° at 0.5 m and 89.3° at 2 m, while Table 3-11 gives 87° "at 2 m"; the difference is within the ±3° tolerance. We use 87° (f = 447 px; 90° would give f = 424 px, i.e. ~5 % more σ_z).

**Effects left out and why** (2026-11-04; rule: an effect is modelled only if the D435 documentation (datasheet + BKM) gives a model for it, see the scope tables above). Two reasons:

*(a) Not important for this scene or the depth-only downstream pipeline*
| Effect | Why it does not matter here | Source |
|---|---|---|
| Lateral noise (per-pixel jitter of image position) | Moves points *within* a face plane, so the plane fits (table, faces) and the hole centre (mean of the gap pixels) are barely affected; it only matters at edges (see edge effects below). Ahn's lateral value (41–43 mm, 90th percentile of edge scatter, no trend) describes edge corruption, not a jitter, and would erase an 8.5 mm hole, so it is unusable at our scale anyway | Ahn et al. 2019 §V |
| Temporal noise | Single frames; each run already gets an independent noise draw | DS Table 4-15 (≤ 1 %) |
| Rolling shutter, motion blur | Depth imagers are global shutter; static scene | DS Table 3-13 |
| Ambient IR / sunlight, glare, thermal drift | Indoor cell, short sessions | BKM §13, §15 |
| Multi-camera interference | One camera; D4xx cameras do not interfere | BKM §6b |
| Defocus | Focused 50 cm to infinity; parts at 0.47–0.68 m | BKM §8b-i |
| RGB sensor effects, colour-to-depth misregistration | Downstream task uses depth only; RGB rendered ideal and pre-registered | DS Table 3-49, see above |
| Post-processing filters | Not applied by default; we model the raw stream | BKM §7a |

*(b) No reliable source for an analytical model* — final (user, 2026-11-04): these stay in this table as deliberate omissions (no D435 model in the datasheet or tuning guide), not as a gap to close.
| Effect | What is known | Why not modelled |
|---|---|---|
| Incidence-angle dependence | Ahn Eq. 2 angle term; Ahn Fig. 9 data: flat to ~60°, huge by 75° | Coefficients hand-set ("heuristically obtained") and inconsistent with the paper's own data in radians and in degrees; not in Intel's documentation. Our hole faces are seen at 20–72°, so it may matter; left out by the scope rule |
| Edge fattening / flying pixels at depth steps | Mechanism: the matching window spans the edge. Keselman et al. 2017 (arXiv 1705.05548) quantify it only for the older R200 (7×7 Census window, gaps < 5 px merged); for the D400 they state a change to semi-global matching with edge-preserving aggregation, without numbers. Robot-learning noise stacks tune it by hand | No D400/D435 value published. May matter for 6–40 px holes; left out by the scope rule |
| Occlusion shadows inside the image | Mechanism described (DS §4.5 non-overlap, BKM §8b-i) | Intel gives a model only for the frame-edge band (included); per-pixel occlusion would be our own derivation |
| Specular holes / outliers on metal, material dependence | Qualitative: "specular reflections may cause image saturation" (DS Table 3-50); physics-based sims (Zhang 2023, DREDS); forum reports | No analytical D435 model; also not the focus (bracket is matte powder-coated) |
| Repetitive-structure false depth (T-slots) | Qualitative (DS Table 3-50, BKM §14) | No model |
| Spatially correlated noise ("egg carton") | Qualitative (BKM §3e) | No correlation length; noise is per-pixel independent (stated as an assumption) |
| Random dropouts / invalid patches | Fill rate ≥ 99 % on a white wall (DS Table 4-15) | A limit only; no rule for where invalid pixels occur |
| Depth bias / scale error | Z-accuracy ± 2 % (DS Table 4-15) | A limit only; no model of the error's form (scale, offset, field-dependent) |
| Preset behaviour (holes vs density), laser power, exposure | Qualitative (BKM §2–4) | Intel does not describe the parameters; assumed: default preset, 150 mW, good exposure |

**Axial / lateral noise: Ahn et al. 2019 vs Intel BKM** (2026-11-04; Ahn full text in `docs/793350771-ANM.pdf`, read and summarised by the AI agent)
- Ahn Eq. 1 is Intel's formula: σ_z = z²·s/(f·B). Its Fig. 2 shows measured RMS close to it up to ~1 m.
- Intel BKM: a theoretical floor with one parameter, the subpixel s, for a localised plane fit on a well-textured target. s < 0.1 (down to 0.05) is good, > ~0.2 px means recalibrate. The noise looks like an "egg carton", i.e. spatially correlated bumps. No angle term, no lateral model.
- Ahn Eq. 2 (D435, 848×480, default preset after Dynamic Calibration, laser 150 mW, white planar target, 0.4–2.1 m, 0–75° at 15° steps, 100 frames each): σ_z = 0.001063 + 0.0007278 z + 0.003949 z² + 0.022 z^{3/2} θ/(π/2 − θ)² (m). Axial = per-pixel deviation from a plane fitted to a central patch of the target.
- At our z (0.47–0.56 m), θ = 0: Intel (s = 0.08) 0.8–1.1 mm; Ahn 2.3–2.7 mm, i.e. ~2.7× Intel, equivalent to s ≈ 0.2 px. Ahn's constant 1.06 mm dominates at short range; the two converge by ~2 m.
- Ahn's angle term is hand-set and does not reproduce its own Fig. 9. Read with θ in radians it predicts 16 mm at 45° and 42 mm at 60° (z = 0.6 m), whereas Fig. 9 shows ~3–5 mm, flat from 0° to 60°. Read in degrees it stays < 3 mm even at 75°, whereas Fig. 9 is off-scale (> 0.1 m) at 75°. Usable fact from the data: flat to ~60°, very large by 75°, nothing measured in between. Our hole faces are seen at 20–72°, i.e. inside that gap.
- Lateral (Ahn): σ_L = 43.2 mm, σ_R = 40.7 mm (90th percentile of edge-pixel scatter about the target's vertical edges). Not Gaussian, no trend with distance or angle, includes the left shadowing. It describes edge corruption, not a per-pixel jitter, and is ~30 px at 0.6 m, so it is unusable for 8.5 mm holes. Lateral effects are to be modelled as edge effects (fattening, flying pixels, shadow) instead.
- Ahn Fig. 8: at 0.3–0.6 m the error histograms are spiky, consistent with the 1 mm depth unit (BKM §8), so quantisation is visible at our range.

**Magnitudes for our setup:** D435 at 848×480, f = 447 px, B = 50 mm, bracket at z = 0.47–0.56 m, profile 0.50–0.68 m (15 test seeds), so disparity ≈ 33–48 px. The 40 mm bracket is ~35 px wide (1,600–2,050 px area); 1 px ≈ 1.1–1.5 mm on the parts. The visible top hole (Ø8.5) covers 6–40 px, seen at 20–72° from its axis (10 of 15 seeds ≥ 50°). Occlusion band beside a 40 mm step: B·f·Δz/z² ≈ 3.6 px at 0.5 m.

## Q3. Experiment: does the camera model change a downstream outcome?
**Robot task framing:** ✅ (user) perception task = **6D pose of the top hole** of the bracket from depth only (hole = where a screw/pin goes in assembly). Pose = opening centre + hole axis; rotation about the axis is undefined for a round hole. Then fit the bracket CAD from the hole and grasp it with the jaws approaching normal to the hole.

**Why a geometric (RANSAC) pipeline and not a neural pose estimator** (user, 2026-11-04): FoundationPose (model-based 6D pose from RGB-D + CAD) was considered and skipped. It needs a CUDA PyTorch stack, compiled extensions and pretrained weights, which goes against the task's "runs on your own laptop" requirement. The geometric pipeline is a deliberate surrogate: it relies on the same depth cues a neural network would use, whether for detection/pose estimation or inside a robot policy (planar faces, depth edges, small holes a few pixels wide, surface normals). If the camera model destroys or shifts these cues, a learned model that depends on them is affected too. The surrogate is also transparent, so a failure can be traced to the effect that caused it. Limitation for the report: a network trained on noisy depth may learn to tolerate some effects, so the surrogate shows how sensitive the cues are, not how a particular network would perform.

**Detector** (`perception/hole_detector.py`, simple geometry, numpy/scipy only): depth → points (range < 1 m drops the floor) → table = largest RANSAC plane → object pixels > 1.5 mm above it → connected clusters → peel planar faces with RANSAC → hole = gap enclosed by a face (binary hole filling) with equivalent diameter 0.6–1.6 × 8.5 mm, ≥ 3 px and round (principal spread ratio ≤ 2, rejects slot fragments) → centre = gap rays intersected with the face plane, axis = face normal → "top" = normal closest to the table normal (≤ 50°).
- Resolution limit: the 8.5 mm hole is ~7 px wide face-on at 0.5 m; on the test seeds the visible hole is 6–40 px, smallest at steep viewing angles.
- Thresholds set on ideal depth: object height 1.5 mm, plane inliers 1 mm. With D435 noise (σ ≈ 1 mm) these sit at the noise level: an expected camera-model effect.
- Ground truth (`eval_holes.py`): 4 hole openings from the CAD (2 holes × outer/inner face). Valid = facing the camera, normal within 50° of vertical, and visible (≥ 6/8 ring points around the opening match the ideal depth). Correct = within 3 mm of a valid opening.

**Bracket fit** (`perception/bracket_fit.py`, `perception/eval_bracket_fit.py`): fits the CAD bracket (full 6D pose) starting from a detected upward hole. The hole fixes 5 DoF; search 4 openings × 36 rotations about the axis (10°), score each by visible model points vs measured depth (+ match within 3 mm, −2× where the camera sees through the model), refine with ICP on the bracket's depth points; every upward candidate is tried and the best-scoring fit kept. Symmetry-aware errors (leg-swap symmetry). Success = translation < 5 mm and rotation < 10°; ADD-S not used for success because 90–150° wrong poses still had ADD-S of 2–3 mm (compact, near-cubic part). Outputs: overlay images and PLY (scene + fitted model).

**Grasp at the hole** (`robot/grasp_hole.py`, PyBullet only): perception (hole → bracket fit, no ground truth) → Dex1 jaws approach **normal to the hole** (along the detected axis, into the face) and close across the bracket's 40 mm width (fitted CAD x) → ready pose → pre-grasp 10 cm out → straight-line approach → close → lift 10 cm. Success = lifted > 5 cm and touching both jaws after a 1 s hold.
- IK: damped least squares on the 7 arm joints in a separate kinematics client, both arms, restarts; nothing executed if any waypoint misses by > 5 mm.
- Grasp point: antipodal on the fitted CAD. Search depth along the approach and shift along the face near the hole; keep candidates whose jaws stay clear of the table and whose fingers do not hit the part on the straight approach (swept volume); take the one with the most CAD surface facing each pad. (A plain "material between the pads" count picked the leg's thin edge.)
- Jaws close slowly at equal speed to the grasped width minus a 4 mm squeeze (not fully shut), so they stall on the part.
- TCP check (user: "I don't think it's at the tip"): correct, the URDF tip frames used as TCP are 11–14 mm behind the jaw tips. `--tcp-offset-mm` / `--tcp-side-mm` move the TCP; with the 40×40 bracket the default TCP lifts more often (4/5 vs 3/5 on the first seeds), so it is kept.
- Jaw geometry taken from the finger collision meshes (tip at 143.3 mm, plate 29 mm tall), not from `getAABB`, which includes the collision margin.
- Execution diagnostics per phase (`phase_log`: TCP tracking error, jaw positions, jaw–bracket contacts and force, jaw–table contacts, bracket displacement). Arm tracking 0.1–0.3 mm.
- With `--camera d435 --run-ids r` the grasp runs on the D435 model's depth (noise draw r).

**Hypothesis:** ⬜
**Conditions:** ✅ ideal depth vs the D435 model (Intel axial noise + datasheet rules); ablations: axial noise alone, rules alone, Ahn et al. noise; mitigation: noise-aware detector thresholds.
**Metrics:** ✅ detector alone (correct / no detection / wrong hole), bracket fit success and pose error (translation, rotation), grasp outcome (lifted, push, lift height) on example draws.
**Results** (2026-11-04). PyBullet, 15 test seeds × 10 independent noise draws. Same random numbers for both models. Per-pixel independent Gaussian, no angle term, no edge effects. Model: `camera_model/d435.py`, `simulate(depth, K, rng, axial="intel"|"ahn"|None, rules=True|False)`; command line (`eval_holes.py`, `eval_bracket_fit.py`, `grasp_hole.py`): Intel = `--camera d435 --no-rules`, Ahn = `--camera d435 --axial ahn --no-rules`, rules only = `--camera d435 --axial none`, Intel + rules (complete model) = `--camera d435`, Ahn + rules = `--camera d435 --axial ahn`; add `--runs 10`.

| Depth | σ_z on the bracket | Detector alone: correct / none / wrong | Fit success | Fit t / r (median, successes) |
|---|---|---|---|---|
| ideal | 0 | 15 / 0 / 0 (of 15) | 15/15 | 0.34 mm / 0.84° |
| Intel BKM, s = 0.08 px | 0.9 mm | 78 / 0 / 72 (52 %) | 143/150 | 0.76 mm / 0.92° |
| Ahn et al. 2019, θ = 0 | 2.4 mm | 1 / 140 / 9 (1 %) | 4/150 | 2.2 mm / 1.4° |
| datasheet rules only (left band, Min-Z, 1 mm quantisation) | 0 (0.3 mm RMS rounding) | 14 / 0 / 1 (of 15) | 15/15 | 0.38 mm / 0.57° |
| **Intel + datasheet rules (complete model)** | 0.9 mm | 90 / 3 / 57 (60 %) | 142/150 | 0.81 mm / 0.96° |

- Intel: every detector failure is a *wrong* pick, spread evenly over the seeds (2–7 of 10), unrelated to hole size or viewing angle. The picks are false holes on the **profile's horizontal top face** (~180 mm from the bracket, cos_up = 1.000): the noise leaves enclosed 10–20 px gaps above the 1 mm face threshold, these pass the size (≥ 5.1 mm) and roundness tests, and "most upward" prefers them over the real hole, which is tilted ~45°. The fit tries every candidate and keeps the best CAD match, so it rejects them: 143/150. Seed 8 (hole seen face-on, 20°) is the weak one at 6/10; cause not yet checked.
- Ahn: the detector finds no hole in 140/150. Cause: thresholds tuned on ideal depth. At σ = 2.4 mm only ~32 % of face pixels fall within the 1 mm face threshold (Intel: ~73 %), so faces break up and the hole is no longer enclosed by a face. ~27 % of table pixels rise above the 1.5 mm object height (~4,200 speckle clusters per frame, 246 ≥ 15 px), which makes runs ~3× slower.
- Datasheet rules (2026-11-04, `camera_model/d435.py`, `rules=True`): the left invalid band is 7–19 px wide in our frames (the left image edge sees the floor 1–3 m away; the datasheet's 45 px is for a scene at 0.5 m) and never reaches a part; Min-Z (195 mm) removes nothing (nearest depth 0.47 m). Only the 1 mm quantisation acts: alone it makes seed 112's detector pick the known false hole, the round 9.3 mm gap on a horizontal surface of the bracket's cap (cos_up = 1.000), which the fit still rejects (15/15). On top of Intel noise (same 150 draws): detector 78 → 90 correct, 3 runs with no detection (seed 8), fit 143 → 142 (seed 8: 6/10 → 3/10), accuracy unchanged (0.76 → 0.81 mm, 0.92 → 0.96°). The detector difference is not significant (paired draws: 28 correct only without rules, 40 only with rules; exact McNemar p = 0.18). Conclusion: the datasheet rules make the frames D435-like but do not change the outcome; the axial noise does.
- **Grasp with the camera model (example of a downstream effect, 2026-11-04).** `robot/grasp_hole.py --camera d435 --run-ids …` (noise draw r of seed s = same random numbers as run r in the evaluations). The grasp scene is settled by physics (~1 mm different from the static evaluation frames), so the evaluation's failing draws did not reproduce there (seed 8 draws 5/7/8, seed 45 draw 4: correct hole, 0.7–1.5 mm). Screening 30 draws × seeds 8, 45, 37, 97 on the settled scenes (perception only): 24/120 draws gave no fit or a fit started from a wrong hole: either on the bracket but 6–18 mm off (the cap gap), or on the profile 130–270 mm away. Grasps on 9 of them:
  - 8/9 never moved: the planner's own checks stopped them ("unreachable (IK)" 5, all four profile cases among them; "no table-clear grasp" 3). The pipeline fails safe, but the robot does not grasp.
  - Seed 8, draw 22 (fit started 13.7 mm from the true hole, grasp point 19 mm along the face): the jaws push the bracket 11 mm on approach and 22 mm while closing, then lift it only 83 mm (ideal depth: ~100 mm, no push). Counted as "lifted" by the > 50 mm rule, but degraded. Video: `output/grasp/wrong_hole/grasp_seed_008_d435_intel_run22_tcp+0mm_side0mm.mp4` (ideal-depth counterpart: `output/grasp/grasp_seed_008_tcp+0mm_side0mm.mp4`).
  - Also seen: with the hole found correctly (0.7–1.5 mm), 3 of 5 draws were "unreachable (IK)" although the same seeds lift with ideal depth: ~1 mm of pose change switches the grasp search to a candidate the arm cannot reach.
- Interpretation: the 2.7× spread between the two published axial models at 0.5 m decides between "the fit survives" and "the pipeline collapses". The detector's fixed mm thresholds turn noise level directly into failure. **Threshold tuning (user, 2026-11-04):** retuning the detector thresholds to the noise (e.g. a few σ_z) may be tried, but only as a *mitigation* result (a fix for noisy depth), not as part of the baseline comparison; the baseline detector stays tuned on ideal depth.
**Mitigation step: noise-aware detector thresholds** ✅ (user, 2026-11-04; a fix for noisy depth, reported separately from the baseline comparison). `perception/hole_detector_tuned.py`: the baseline detector with each threshold (plane inliers 1 mm, object height 1.5 mm) replaced per pixel by max(baseline value, k·σ_z(z)), σ_z from the camera model, i.e. the detector is adapted to the camera's noise level. The baseline detector is unchanged (refactor checked: identical results). Options `--detector tuned --k-sigma k [--tune-axial ahn]` in `eval_holes.py`, `eval_bracket_fit.py`, `grasp_hole.py`. Same 150 noise draws as the baseline runs.

| Detector | **Ideal depth** (15 seeds): detector correct · hole axis error · fit | **D435 depth** (Intel + rules, 15 seeds × 10 draws): detector correct / none / wrong · hole axis error · fit · fit t / r |
|---|---|---|
| baseline | 15/15 · 0.71° · 15/15 (0.34 mm / 0.84°) | 90 / 3 / 57 (60 %) · 0.87° · 142/150 · 0.81 mm / 0.96° |
| tuned, k = 2 | – | 114 / 0 / 36 (76 %) · 1.15° · – |
| **tuned, k = 2.5** | 14/15 · 1.75° · 15/15 (0.40 mm / 0.80°) | **144 / 0 / 6 (96 %)** · 1.63° · **150/150** · 0.81 mm / 1.12° |
| tuned, k = 3 | 15/15 · 3.33° · 15/15 (0.48 mm / 0.89°) | 149 / 1 / 0 (99 %) · 2.50° · 147/150 · 0.87 mm / 1.22° |
| tuned, k = 4 | – | 106 / 40 / 4 (71 %) · 4.58° · – |
| Ahn noise + rules, baseline | | 2 / 140 / 8 (1 %) · 1.37° · 4/150 · 2.07 mm / 1.84° |
| Ahn noise + rules, tuned to Ahn (k = 2.5) | | 54 / 47 / 49 (36 %) · 9.46° · 67/150 · 2.78 mm / 2.63° |

- Success rate back to the ideal level (fit 150/150 at k = 2.5; detector 96–99 %): the noise no longer leaves enclosed gaps outside the wider face band, so the false holes on the profile's top face disappear. Accuracy is not: fit translation 0.81 mm vs 0.34 mm ideal, the same as with the baseline detector on D435 depth, i.e. it comes from the noisy points the fit uses, not from the detector. Tuning removes the wrong-hole failures, not the precision loss.
- Trade-off: wider bands let each face fit take in nearby off-face pixels, which tilts the detected hole axis, even on ideal depth (0.71° → 1.75° → 3.33° for baseline / k = 2.5 / 3); the CAD fit's ICP removes most of it. Too wide (k = 4): the object-height threshold (~4–5 mm) reaches the 5 mm leg thickness, 40 runs without detection.
- Limit: tuned to Ahn's noise (2.4 mm) the detector reaches only 36 % (baseline 1 %), the fit 67/150 (baseline 4/150); the bands needed exceed the bracket's 5 mm legs.
- Caveats: k chosen on the same 15 test seeds it is reported on; the detector is given the true noise level (σ_z of the model). Best case for threshold tuning.
- **Grasp with the tuned detector (k = 2.5) on the 9 draws whose baseline perception started from a wrong hole** (seed 8 draws 9, 10, 15, 18, 22, 24; seed 45 draws 0, 16; seed 97 draw 9): all 9 now find the true hole (0.4–0.8 mm); **8/9 lift ~100 mm** (baseline: 8 never moved — IK / table clearance refused — and 1 lifted only 83 mm after pushing the bracket 22 mm). Seed 8 draw 22, side by side:
  - baseline detector: fit started 13.7 mm off the hole, bracket pushed 11 mm on approach and 22 mm while closing, lifted 83 mm. Video `output/grasp/wrong_hole/grasp_seed_008_d435_intel_run22_tcp+0mm_side0mm.mp4`.
  - tuned detector: hole 0.56 mm off, bracket moved 1.1 mm while closing, 6–8 N per jaw, lifted 99.5 mm. Video `output/grasp/mitigation/grasp_seed_008_d435_intel_run22_tuned2.5_tcp+0mm_side0mm.mp4`.
  - The one failure (seed 8 draw 10) is in execution, not perception: hole 0.5 mm off and the same grasp point as draw 22, but the jaws did not open on the way to the pre-grasp (−16 / −5 mm instead of +4), so a finger pushed the bracket 26 mm on approach (like the ideal-depth push failures of seeds 12, 42).
- **Interpretation (user):** the baseline thresholds play the role of parameters adapted to the data they were developed on. A perception module or robot policy trained only on ideal synthetic depth adapts its parameters to that data, like the baseline detector, and breaks on the real camera's noise. Collecting the training data through the camera model adapts the parameters to the noise (here: the tuned thresholds), so the result is more realistically deployable. Most sim-to-real approaches handle this with domain randomization, which brings its own problems and challenges for training; a camera model instead gives targeted data quality: the noise the deployed camera will actually produce.
  - Note from the results: the adaptation is only as good as the camera model. The two published D435 noise levels differ by 2.7× (Intel vs Ahn), and parameters adapted to the lower one meet the higher one in the same way the ideal-tuned baseline met the Intel noise. Tuning on the wrong noise level is the risk a camera model carries and domain randomization hedges against.

**What did not work:**
- **False holes on the cap.** With ideal depth, the detector's 8 wrong picks on seeds 0–200 are all the round gap on a horizontal surface of the bracket's cap ("most upward" prefers it over the real, tilted hole). Trying every upward candidate in the fit and keeping the best CAD match fixes this for the fit (seed 13: the grasp drove a jaw 5 mm into the part before that change).
- **Bug: half-pixel offset in the PyBullet backend.** TinyRenderer samples column c at u = c and row r at v = r + 1, while K assumes pixel centres at +0.5. Detected holes were systematically offset by (+0.40, −0.54) px. Fixed with an off-centre projection matrix; offset now (+0.07, 0.00) px.
- Isaac backend bugs found while testing headless (all fixed):
  - Camera API setters write focal length / aperture in stage units (m → focal 0.11); the RTX renderer ignores such values and renders with its default 50 mm lens (~25° FOV) while `get_intrinsics_matrix()` still reports the requested K. Measured with a 1 m target at 2 m: 848 px wide instead of 223. Fixed by setting the USD attributes at mm scale (aperture 20.955, focal 11.04) → 224 px.
  - `SimulationApp.close()` ends the Python process, so frames were never saved; close is now called after saving.
  - RGB key is `rgb` in Isaac 5.1 (not `rgba`).
  - The G1 asset's soft joint drives (stiffness 100) let the waist sag under gravity, so the head camera moved while frames rendered: holes offset by (0.0, −1.05) px and −1.2 mm in depth. Fixed with stiff drives (kp 1e6, kd 1e4); camera drift now 0.003 mm and its pose equals the PyBullet one.
- Isaac renders the grey table almost white (lighting intensities were guesses): cosmetic, open.
**Known limitation:** evaluation is synthetic-only → shows sensitivity to the model, not that the model is realistic. Validation against real data is not required by the task (user, 2026-11-04).
**Possible future work:** check the model's error statistics against HAMMER (real D435 depth with ground truth from 3D scans, objects grouped diffuse / reflective / transparent; see `docs/literature_d435_simulation.md`). A layout-independent comparison script (real vs model per depth bin: invalid rate, bias, robust spread; edge pixels separately) was written and self-tested, then removed (user, 2026-11-04: stick with the basic analytical model; the HAMMER download server did not respond from this machine). Licence CC BY-NC-SA 4.0. Neither HAMMER paper (Jung et al., arXiv 2205.04565; CVPR 2023, arXiv 2303.14840) fits a sensor noise model or reports raw D435-vs-GT error: their tables give errors of depth networks *trained on* each sensor (e.g. active stereo RMSE 72.2 mm), which cannot be compared with a noise model; distance dependence only qualitative; working distances not stated. GT accuracy: 0.80 mm point RMSE for robot-labelled objects, background meshes (hand-held scanner) less accurate, no number. So a comparison needs the raw data; on objects at ~0.5 m the GT error (0.8 mm) is as large as the predicted noise (0.9 mm), so subtract it in quadrature.
**Open items (user, 2026-11-04):**
- Test set: may be increased beyond 15 seeds later.
- Seed 8 (the weakest seed under the D435 model: fit 6/10 → 3/10 with the rules, no detection in 3 draws; hole seen nearly face-on, 20°): to be cleaned up and explained once the conclusions are settled.

## Q4. Riskiest sim-to-real assumption + 6-month PhD plan (≤1 page)
⬜ (collect ideas here as they come up)
- (user, 2026-11-04) Training data from ideal synthetic depth adapts a model's parameters to noise-free data (cf. the baseline detector's thresholds); data generated through a camera model adapts them to the deployed sensor (cf. the tuned detector, Q3 mitigation). Alternative to broad domain randomization: targeted data quality. Risk: the camera model's own error (published D435 noise levels differ 2.7×).

## Q5. AI-tool use note
- **How the AI was used:** I did not read the papers or documents in full myself. I relied on the AI agent (Claude Code) to find sources, read and summarise them (datasheet, tuning guide, Ahn et al., HAMMER, learned-model papers), look up specific values and page references, and proofread my notes and the README. I made the decisions from its summaries, and asked it to check claims against the source text (PDFs in `docs/`, the robot URDF, saved result files) rather than from memory; that check found and fixed wrong page numbers, a misattributed citation (Keselman et al. 2017 values are for the R200, not the D435) and a miscounted result.
- Used Claude Code to read the task, structure the problem, find and extract camera specs from the datasheet and tuning guide, write and run the pipeline code under the user's decisions.
- Corrections / disagreements with the tool:
  - **Main example: edge fattening. I disagreed with the AI's suggestion to model it.**
    - *What the AI suggested:* while reviewing the approach, the AI argued the camera model was "generic, not specific to the parts". It is z² axial noise plus three datasheet rules that have no effect in our scene, while the task asks for imperfections "on these parts". It recommended adding edge fattening at the hole, with the width swept in pixels: the 8.5 mm hole is only 6–40 px wide, so fattening by 1–2 px could close it. It had ranked edge fattening as the most important effect from the start.
    - *What I decided and why:* I did not model it. No D435 value exists for the stereo matching-window width (not in the datasheet, not in the tuning guide). A swept, invented width would break the scope rule: only effects that Intel's D435 documentation gives a model for, so every included effect is traceable to the manufacturer. Edge fattening stays in the "effects left out and why" table (no reliable source for an analytical model), as a deliberate omission rather than a gap to close.
    - *Why it's a good example:* the AI favoured coverage of the task wording; I favoured traceability and a single consistent rule, and stated the reason.
  - Simulator: the AI recommended BlenderProc for simplicity and reviewer reproducibility; the user chose Isaac Sim instead (physics/robot integration, industrial relevance) and accepted the reproducibility cost, mitigated by decoupling the camera model from the renderer.

---

## Decision log
| Date | Decision | Alternatives considered | Reason |
|---|---|---|---|
| 2026-11-03 | Core of the work = camera depth model; downstream task kept simple | Full assembly simulation | Task asks for one downstream step; assembly sim doesn't test the camera model |
| 2026-11-03 | Camera: RealSense D435 (G1 head camera) | Other RealSense models, ToF cameras (Azure Kinect / Femto), Zivid | The G1 carries a D435; documented by Intel's datasheet and tuning guide; stereo failure modes fit metal parts |
| 2026-11-03 | Table sized to G1 reach (0.40 × 0.60 m, top 0.75 m, centre x 0.34 m) | Larger table further out | Parts must be graspable by the G1 (IK reachability map) |
| 2026-11-03 | Parts: item Line 8 40×40 profile + angle bracket (manufacturer CAD) | Smaller bracket sizes | The bracket and its holes must suit a hole-normal grasp with the Dex1 jaws |
| 2026-11-03 | Keep PyBullet as alternative backend | Isaac only | GPU busy; reviewer reproducibility on a laptop |
| 2026-11-03 | Simulator: Isaac Sim | BlenderProc (my recommendation), pyrender, PyBullet | User's choice: physics/robot integration and industrial relevance outweigh the install and reproducibility cost |
| 2026-11-04 | Test set re-selected: all stages succeed with ideal depth (seeds 4, 8, 23, 26, 27, 32, 37, 38, 45, 69, 73, 86, 92, 97, 112), confirmed on Isaac (detection + fit) | Keep perception/grasp failures in the test set | User's choice: clean ideal baseline, so every camera-model failure is caused by the camera model. Accuracy of the ideal pipeline is reported separately on all seeds 0–200 |
| 2026-11-04 | Camera model: analytical noise first; learned models read, then decided; IR stereo simulation out of scope | IR stereo simulation (Zhang et al. 2023 / SAPIEN, DREDS), learned noise | User's choice: analytical effects can be tested and understood one at a time; IR stereo is beyond what is feasible and understandable now |
| 2026-11-04 | Axial noise: Intel BKM model (σ_z = z²·s/(f·B), s = 0.08 px) is the main camera model; Ahn et al. 2019 (θ = 0) kept as an ablation | Ahn et al. as the main model (empirical fit on a real D435, ~2.7× Intel at 0.5 m) | User's choice: the Intel model comes from the manufacturer. Ahn shows how sensitive the result is to the noise level (Intel: fit 143/150; Ahn: 4/150) |
| 2026-11-04 | RGB rendered ideal and pre-registered to depth; colour-sensor imperfections left out | Render RGB from the colour sensor's pose and intrinsics; model RGB noise / rolling shutter / misregistration | Downstream task uses depth only; study focus is the D435's depth imperfections |
| 2026-11-04 | Camera model limited to effects the D435 documentation lets us model (Q2, "Scope of the camera model" table, with datasheet references) | Literature models (Ahn angle term, edge fattening, dropout patterns) | User's choice: every included effect is traceable to the manufacturer. Datasheet formulas: left invalid band / depth FOV; rules: Min-Z, z-depth, origin, rectification, global shutter; limits only: accuracy, spatial / temporal noise, fill rate. Axial noise from the BKM (Intel). The BKM counts as part of the datasheet (user); BKM adds Min-Z = f·B/126 and 1 mm quantisation |
| 2026-11-04 | Review responses: omitted effects (edge fattening, angle term, occlusion) stay omitted; no validation (HAMMER = future work); threshold tuning only as a mitigation result; test set may grow beyond 15; seed 8 explained later | Close the omitted effects; validate on HAMMER now; retune thresholds in the baseline | User's choice: scope rule (documented D435 models only); validation not required by the task; keep the baseline detector tuned on ideal depth so the camera model's effect is not hidden |
| 2026-11-04 | Keep the basic analytical model; HAMMER comparison script removed | Validate on HAMMER; adopt a learned D435 model | No released learned D435 noise generator (ByteDance's is unreleased); HAMMER papers give no raw D435 error numbers and the data could not be downloaded; validation is not required by the task |
| 2026-11-04 | Downstream perception stays geometric (RANSAC planes/holes + CAD fit); FoundationPose not integrated | FoundationPose on the ideal RGB-D (5-seed test) | Needs a GPU stack and weights (not laptop-runnable); the geometric pipeline is a transparent surrogate for the depth cues a network (detector or robot policy) relies on |
| 2026-11-04 | Noise-aware detector thresholds (k·σ_z, k = 2.5) as a mitigation step, reported separately from the baseline | Retune the baseline detector; domain randomization | Shows that adapting the perception parameters to the camera model's noise recovers the success rate (fit 150/150, grasp 8/9 on the wrong-hole draws); baseline stays tuned on ideal depth so the camera model's effect is not hidden |
