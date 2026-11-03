# Working notes: answers to task.md, built up as we go

Status legend: ✅ decided · 🟡 proposed, not yet confirmed · ⬜ open

---

## Project structure (2026-11-03, after switching to item Line 8 40×40 parts)
```
scene/       config.py (layout, camera, TEST_SEEDS) · parts.py (CAD, hole geometry) · materials.py
             spec.py (scene description) · frame_io.py (frame format, ideal dataset) · render.py (CLI)
             backends/pybullet_backend.py, isaac_backend.py
perception/  hole_detector.py · bracket_fit.py · eval_holes.py · eval_bracket_fit.py
robot/       grasp_hole.py (G1 + Dex1 grasp normal to the hole, PyBullet)
camera_model/d435.py (D435 model: interface + constants, effects not yet chosen)
assets/cad/  item 40×40 STEP (+ archive_line5_20x20/) · assets/meshes/ (OBJ exported from CAD)
data/ideal/<backend>/seed_XXX/head/   ideal reference frames of the 15 TEST_SEEDS (PyBullet + Isaac)
output/      evaluation results, overlays, grasp videos
docs/        datasheet, tuning BKM, literature notes
```
- **Ideal reference set** (user, 2026-11-03): `data/ideal/{pybullet,isaac}/seed_XXX/head/` (rgb, depth, instance, meta) for `TEST_SEEDS = [4, 8, 12, 17, 20, 21, 23, 26, 27, 32, 37, 38, 42, 45, 52]`. This is what the camera model's depth will be compared with. `scene/render.py --backend pybullet --test-seeds` regenerates PyBullet; Isaac one seed per process.
- Cleanup: unused constants and helpers removed (camera-link pose, arm-reach estimates, aperture/focal, `ros_pose_from_link`); hole geometry in one place (`parts.py`); evaluations read the ideal dataset; stale outputs of the 20×20 parts deleted; 20×20 CAD archived.

**Current results (40×40 parts, ideal depth, PyBullet, 15 test seeds):**
| Step | Result |
|---|---|
| Top-hole detector alone ("most upward" candidate) | 12/15; misses 17, 21, 52 pick a round gap on the bracket's cap; position error median 0.66 mm, axis 0.71° |
| Bracket fit (tries every upward candidate, keeps the best fit) | 15/15 (by construction of the test set); translation median 0.37 mm (max 0.70), rotation 0.66° (max 1.28°), ADD-S 0.50 mm |
| Same on the Isaac reference set | detector 12/15 (same seeds), fit 15/15 (translation 0.36 mm, rotation 0.58°); Isaac − PyBullet depth on shared part pixels: per-seed median 0.00 mm |
| Grasp at the hole, approach normal to it (default URDF TCP, squeeze 4 mm) | 13/15 lifted ~100 mm; seeds 12, 42: a gripper part not covered by the finger model hits the bracket during the approach and pushes it ~25 mm (fit itself correct: ≤ 0.5 mm, ≤ 1°); seed 20 slips at 51 mm (3–4 N). Jaw gain 1.0 instead of 0.1: no change (reverted). Finger swept-volume collision check added (did not catch 12/42) |

## Q1. Reference camera and pipeline
**Camera:** ✅ Intel RealSense **D435** (active IR stereo), the head camera of the Unitree G1 (`d435_link` in unitree_ros G1 URDF: on torso_link at xyz (0.058, 0.018, 0.420) m, pitched ≈47.6° down).
- Why: matches the real robot used as the platform (G1 ships with a D435i), so the setup is defensible. Same datasheet + tuning paper cover it. It has >2× the depth noise of the D415 at equal range (BKM), so camera effects on the downstream step should be more visible. Stereo failure modes (texture, specularity, occlusion) are what shiny aluminium provokes.
- Changed from the D415 (first choice) after finding the G1 carries a D435; also, at the G1 head pose the table may be closer than the D415's 450 mm Min-Z.
- Key specs (docs/): B = 50 mm; depth FOV H87°/V58° (HD); OV9282 imagers, 1280×800, **global shutter**; wide 850 nm VCSEL projector (H90°/V63°, 360 mW); recommended depth resolution 848×480 (f ≈ 447 px); Min-Z 195 mm @848×480, 280 mm @1280×720.
- Expected RMS depth noise (Z²·0.08/(f·B), 848×480): 0.6 mm @0.4 m, 1.3 mm @0.6 m, 2.3 mm @0.8 m, 3.6 mm @1.0 m. Left invalid band ≈ 37 px @0.6 m.
- Noted inconsistency: HFOV given as 87° (depth FOV, datasheet), 90° (imager table, and used in BKM example), 91.2° (older BKM table). Using 87° depth FOV.

**Renderer / simulator:** ✅ **NVIDIA Isaac Sim** (RTX ray-traced rendering, Replicator for labels, PhysX).
- Why: physically based rendering of metal, built-in labels and randomisation, physics + robot/gripper models (executed grasps become possible), standard in industrial robotics research.
- Accepted cost: needs an RTX GPU, ≥32 GB RAM and a large install, so reviewers may not be able to run it on a laptop; steeper API.
- 🟡 Mitigation: keep the camera model and the experiment as plain Python (numpy/OpenCV) that read Isaac's saved outputs (depth, normals, masks, poses) and ship a small generated dataset, so reviewers can run everything except the rendering.
- Environment: conda env at `/mnt/shared_data/envs/synthetic_rgbd` (Python 3.11, `isaacsim[all,extscache]==5.1.0` from pypi.nvidia.com, installed with uv). Placed on /mnt/shared_data because the home disk is 96% full. Activate with `conda activate /mnt/shared_data/envs/synthetic_rgbd`.
- ✅ **Common scene and camera model for both backends.** Develop in PyBullet, test in Isaac. `scene/spec.py` makes a plain-data scene description per seed. `scene/materials.py` is one material table (visual, physical, optical). Backends in `scene/backends/` turn the spec into a frame. `scene/frame_io.py` defines one output format (rgb, z-depth, common instance ids 0 = bg, 1 = table, 2 = robot, 3+ = parts, meta.json). `camera_model/d435.py` works only on saved frames, so it doesn't depend on the simulator. CLI: `scene/render.py --backend pybullet|isaac --seed N`.
  - Parts placed analytically at rest and kept static by default (`--settle` for physics). Reason: the same seed gives an identical scene in both backends; PyBullet settling also leaves parts ≈1 mm above the table (collision margin).
  - Mass from mesh volume × 2700 kg/m³: profile 134 g (0.67 kg/m; real 20×20 slot-6 profiles are ≈0.4–0.5 kg/m, so the simplified section is too solid), bracket 5.6 g.
- ✅ **PyBullet kept as an alternative backend** (`scene/build_scene_pybullet.py`): CPU only, ~2 s per frame, same config/parts/output layout (rgb, depth, instance ids, meta.json). Gives reviewers a way to run everything on a laptop and lets work continue while the GPU is busy. Limitation: TinyRenderer has no PBR or reflections, so it can't render material effects (specular aluminium). Depth, masks and poses match Isaac/preview (parts at 0.52–0.64 m in both).
- Alternatives considered: BlenderProc (one Python script, runs on CPU, no robot), pyrender+trimesh (simplest, no realistic metal), PyBullet (crude rendering).

**Labels produced:** 🟡 ideal z-depth, RGB, surface normals, instance masks, 6D part poses, camera intrinsics + pose (`scene/build_scene.py`).

**Scene:** 🟡 written, not yet run (blocked on RAM: about 2 GB free, Xorg using about 80 GB).
- Unitree G1, fixed base, Dex1 grippers (local Unitree asset). D435 = Isaac camera attached to `d435_link`, 848×480, HFOV 87°.
- Table sized to G1 reach: 0.40 × 0.60 m, top 0.75 m above floor, centre 0.35 m in front of pelvis (spans x 0.15–0.55). G1 arm reach (from asset): shoulder at (0, ±0.10, 0.29) m, arm ≈0.54 m fully extended → ≈0.43 m horizontal reach on the table plane. Parts within ±6 cm (x) / ±12 cm (y) of table centre.
  - First version (0.8 × 1.2 m table at 0.60 m) was out of reach; CPU preview showed parts at 0.70–0.81 m and the bracket only ≈11 px wide. New layout: parts 0.50–0.64 m from camera, bracket ≈15 px, expected noise ≈1.0–1.5 mm.
  - ✅ Observation arm pose (`OBSERVE_ARM_POSE` in config, applied by both backends): elbows +90° (forearms down), shoulder roll ±0.20 rad (0.15 is the smallest value without hand-hip collision in PyBullet). Result: 0 robot pixels in view, parts fully visible. Justification: a real robot looks before it reaches.
  - Tried to replace the zinc bracket with an aluminium one: the second download was byte-identical to item 42502 (same zinc bracket), so the bracket is still zinc.
  - (Before) In the default pose the G1's forearms and grippers are in view (≈8% of the image, PyBullet render), at the left and right of the table. With the current placement region they frame the parts but don't cover them; parts placed near y = ±0.12 could be partly hidden. Options: accept (realistic clutter and occlusion), or define an "observation" arm pose.
- ✅ Parts: **real manufacturer CAD from item (Line 5, 20×20, slot 6)**, STEP files in `assets/cad/`, loaded with trimesh + cascadio (OpenCascade) and exported once to OBJ for both backends:
  - Profile **item 0.0.370.03** (Profile 5 20×20 natural, cut to 200 mm): clear-anodized aluminium, 99 g (0.50 kg/m).
  - Bracket **item 0.0.425.02** (Angle Bracket Set 5 20×20), reduced to the **bare bracket with free holes**. item's STEP fuses the bracket with its 2 M5 button-head screws and 2 T-slot nuts into one solid (plus the plastic cap as a second solid). Removed once with OpenCascade booleans (via gmsh, no script kept; steps recorded in `scene/parts.py`): drop the cap, cut everything beyond the outer faces (nuts, screw tips), re-drill both Ø5.3 holes, cut both screw heads (their side faces were complete, so no bracket material is lost). Result: one closed solid, exactly 20×20×20 mm, 2.36 cm³ → 15.6 g at zinc density (item's ~23 g is likely the set with fasteners). Original STEP kept unchanged. Note: **not aluminium**; a matte painted metal surface, so expect it to be easier for the D435 than the anodized profile.
  - The CAD frame is kept as the object frame for 6D pose labels.
  - Resting poses come from `trimesh.compute_stable_poses` (convex hull), sampled by probability per seed; poses below p = 0.02 are dropped (e.g. profile standing on its end, p = 0.003). Profile: 4 side poses. Bracket (cleaned): 5 poses (p = 0.30, 0.25, 0.25, 0.10, 0.10).
  - Replaced the procedural placeholders (which had an unrealistic 0.67 kg/m profile).
  - What didn't work: (1) part sampler looped forever after raising MIN_PART_DIST to 0.14 m (fixed by resampling both parts jointly); (2) stable-pose computation takes ~1 min per part on the loaded CPU, so the results are cached as `.npz` next to the STEP files.
- Material: currently UsdPreviewSurface aluminium (metallic 1, roughness 0.3). 🟡 Candidates from NVIDIA's free MDL library (public S3): Base/Metals `Aluminum_Anodized`, `Aluminum_Anodized_Black`, `Aluminum_Cast`, `Aluminum_Polished`; vMaterials_2/Metal `Aluminum` (Polished, Glossy_Fingerprints) and `Aluminum_Anodized` (Shiny, Scratched, Smudges, Dirty). Lighting: dome + overhead area light.
- Simplifications: simplified slot profile (no chamfers or grooves); bracket always rests on its horizontal leg; convex-hull colliders (fine for resting parts, wrong for grasp contacts in slots).

## Q2. Camera imperfections: included / left out / why
**Wrist cameras: tried and dropped (2026-11-03).** Added a D435 on each wrist using Unitree's own mount (`unitree_sim_isaaclab`, `tasks/common_config/camera_configs.py`, Dex1 wrist camera: parent `*_hand_base_link`, pos (±0.0254, 0.045, 0.135), rot (−0.342, 0.940, 0, 0), ROS convention). Removed by user decision because it complicates the scene. Reasons on record: real G1 wrist setups use a RealSense D405 (close range, from about 7 cm), while a D435 is blind closer than 195 mm; the wrist cameras need a different arm pose than the head camera (arms down: they see only legs; arms forward: grippers cover the head view). The camera list in `config.py` still supports more cameras if revisited.

**Open issue (depth origin):** the D435 reports depth from the **left IR imager**, 17.5 mm from the camera centre (tripod hole), datasheet Table 4-20. Our camera sits at the URDF `d435_link`; need to check which point that is and render ideal depth from the left imager (a 17.5 mm lateral offset is about 13 px at 0.6 m, and it decides on which side occlusion shadows appear).

**Literature notes:** `docs/literature_d435_simulation.md` (online: https://claude.ai/artifact/NDdykcHzKFRfK1Si1WQWwF) (physics-based IR stereo sims: Zhang et al. T-RO 2023 / SAPIEN, DREDS; analytic: Ahn et al. 2019 D435; Isaac Sim 5.1 `SingleViewDepthSensor`; robot-learning noise stacks; validation data: HAMMER (real D435 + GT on reflective objects)).

**Approach:** ⬜ Level 1 (corrupt ideal depth) vs Level 2 (render IR stereo pair + projector, run stereo matching).

Magnitudes for our setup: D435 at 848×480, f = 447 px, B = 50 mm, parts at z = 0.50–0.62 m, so disparity ≈ 36–45 px. A part is ~15–16 px wide; 1 px ≈ 1.1–1.4 mm on the part.

| Effect | Stage | Magnitude here | Relevance | Status | Justification / source |
|---|---|---|---|---|---|
| Edge fattening / flying pixels | matching window spans the part edge | 1–3 px per edge = 1.2–3.7 mm; a 20 mm-high part on the table gives a 20 mm depth step at every edge; edge pixels are a large share of a 16 px-wide part | High | 🟡 include | Keselman 2017 (window aggregation); DPL / Parkour edge dropout; HAMMER (qualitative) |
| Specular holes and outliers on metal | surface reflection | unknown size; outliers sit behind the surface (mirrored table); depends on material and angle (clear anodized = satin, bare = shiny) | High (the core question) | 🟡 include, rule-based | Zhang 2023, DREDS, HAMMER, forum reports |
| Depth bias / scale error | calibration | spec ±2 % → up to ±11 mm at 0.55 m; well-calibrated ~1.5 mm (≈5 mm at 1 m, scales with z²) | High for grasp tolerance, invisible to segmentation | 🟡 include as per-sequence scale | Datasheet Table 4-15; Keselman 2017 |
| Random noise ∝ z² | subpixel matching | σ = 0.9–1.4 mm (0.08 px), 2.2–3.4 mm if subpixel degrades to 0.2 px on shiny parts | Medium (ICP averages over many points) | 🟡 include, subpixel per material | BKM §3e; Ahn 2019 |
| Incidence-angle term | surface + matching | noise rises steeply near grazing; side faces of the profile are seen at ~40–50° | Medium | 🟡 include (Nguyen-style term) | Ahn 2019; arXiv 2412.15040 |
| Occlusion shadows | one imager blocked | ≈1.5 px band beside each part (20 mm step) | Low–medium | 🟡 include (geometric) | stereo geometry |
| T-slot detail | matching resolution, interreflection | 6 mm slot ≈ 4–5 px → smoothed or holes | Low for pose (profile faces are symmetric) | 🟡 emerges from edge model | — |
| Disparity quantisation | output | ~0.35–0.54 mm per 1/32 px step; 1 mm depth unit | Low | 🟡 include (cheap) | Keselman 2017 (subpixel bits) |
| RGB-depth misregistration | separate RGB sensor | depends on pipeline | Medium only if RGB masks feed depth-based pose | ⬜ decide with pipeline | Datasheet (RGB sensor offset) |
| Left invalid band | stereo geometry | ≈37 px at left image edge; parts are central | None here | 🟡 include (free) or leave out | Datasheet §4.5 |
| Min-Z / Max-Z | range limits | 0.195 m / ~3 m; parts at 0.5–0.62 m | None here | 🟡 include (free) | Datasheet Table 4-12 |
| Temporal noise | sensor | ≤1 %; single frames | Covered by random noise | 🟡 leave out | Datasheet Table 4-15 |
| Rolling shutter, motion blur | sensor | D435 depth imagers are global shutter; static scene | None | 🟡 leave out | Datasheet Table 3-13 |
| Ambient IR / sunlight, thermal drift, multi-camera interference | environment | indoor cell, one camera | Low | 🟡 leave out | BKM |
| Post-processing filters, presets | firmware / SDK | we model the raw stream; preset choice decides holes vs outliers | Choice to state | ⬜ decide preset | BKM §4 |

## Q3. Experiment: does the camera model change a downstream outcome?
**Robot task framing:** ✅ (2026-11-03, user) perception task = **6D pose of the top hole** of the bracket from depth only (hole = where a screw/pin goes in assembly). Pose = opening centre + hole axis; rotation about the axis is undefined for a round hole.
- Earlier option (not chosen): 6D pose of the whole part via segment + ICP to CAD.

**Detector** (`perception/hole_detector.py`, simple geometry, numpy/scipy only): depth → points (range < 1 m drops the floor) → table = largest RANSAC plane → object pixels > 1.5 mm above it → connected clusters → peel planar faces with RANSAC → hole = gap enclosed by a face (binary hole filling) with equivalent diameter 0.6–1.6 × 5.3 mm, ≥ 3 px and round (principal spread ratio ≤ 2, rejects slot fragments) → centre = gap rays intersected with the face plane, axis = face normal → "top" = normal closest to the table normal (≤ 50°).
- Resolution limit: a 5.3 mm hole is ~4 px wide at 0.55 m (848×480), so a hole is only 3–14 px.
- Thresholds are for ideal depth: a bracket leg lying flat is 2 mm thick, so table/object thresholds must stay < 2 mm. With D435 noise (σ ≈ 1 mm) this becomes hard: an expected camera-model effect.

**Fixed test set** (user, 2026-11-03): `config.TEST_SEEDS = [0, 1, 2, 8, 9, 11, 12, 13, 17, 19, 20, 23, 26, 27, 31]`, used by all evaluations: the 15 scenes among seeds 0–29 with a visible top hole, with seed 14 (pocket hole, missed by the detector) replaced by seed 31. **Caveat for the report:** removing a failure case makes ideal-depth success on this set optimistic (hole detector 15/15).

**Bracket reconstruction** (`perception/bracket_fit.py`, `perception/eval_bracket_fit.py`): fits the CAD bracket (full 6D pose) starting from the detected top hole. The hole fixes 5 DoF; search 4 openings × 36 rotations about the axis (10°), score each by visible model points vs measured depth (+ match within 3 mm, −2× where the camera sees through the model), refine with ICP on the bracket's depth points. Symmetry-aware errors (the bracket maps onto itself under a 180° turn about (0, 1, 1)/√2; checked on the CAD: 0.17 mm). Success = translation < 5 mm and rotation < 10°; ADD-S not used for success because 90–150° wrong poses still had ADD-S of 2–3 mm (compact, near-cubic part). Outputs: overlay images and PLY (scene + fitted model).
- Result, ideal depth, PyBullet, seeds 0–29 + 31 (before the test set was fixed): 16 fits, 14 successful; successful fits ADD-S median 0.28 mm, translation 0.38 mm, rotation 1.7°. Failures: seed 3 (fit started from the detector's one false hole) and **seed 8** (wrong rotation about the hole axis, 90° off; in the test set). Not run on Isaac (user: keep the script, no full run).

**Grasp at the hole** (`robot/grasp_hole.py`, PyBullet only, user request 2026-11-03): perception (hole → bracket fit, no GT) → grasp with the Dex1 jaws approaching **normal to the hole** (along the detected axis, into the face), jaws closing across the bracket's 20 mm width (fitted CAD x) → ready pose → pre-grasp 10 cm out → straight-line approach → close → lift 10 cm. IK: damped least squares on 7 arm joints in a separate kinematics client, both arms, restarts; not executed if IK misses by > 5 mm. Success = lifted > 5 cm and touching both jaws.
- New ready / observation pose (`OBSERVE_ARM_POSE`): arms raised out to the sides (shoulder pitch 0.3, roll ±0.8, yaw ±0.5, elbow 0), grippers ~24 cm above the table beside it; grid search: no robot pixels in the head camera, no self-collision.
- **Result, first 5 test seeds: 0/5 lifted.** Perception fine (approach within ~2° of the true hole normal in all 5); IK < 0.1 mm in 4/5 (seed 0: no reachable solution, 41 mm). Cause: in all 5 the bracket rests on its corner (legs at 45°), holes ~10 mm above the table. Approaching normal to the hole tilts the gripper 43°; the Dex1 jaw reaches 14 mm past the pad centre and is 35 mm tall, so its lowest corner is ~22 mm below the pad centre → the pads would have to be ≥ 26 mm above the table. Even with the grasp point slid 15 mm up the face, the pads stay ~11 mm outside the face and close on air (video `output/grasp/grasp_seed_001.mp4`).
- First attempt also had: jaws opened to 95 mm (now 30 mm), and the arm executing an unreachable IK solution that knocked the bracket off the table (now refused).
- **Layout change (user, 2026-11-03): table and parts moved into the reachable region.** IK reachability map (both arms, 9 approach directions, TCP at part height): best at table height 0.75 m and x = 0.20–0.30 m (44–67 % of directions reachable); x ≥ 0.35 m only 11–33 %; tables at 0.80 / 0.85 m not better. Table centre x 0.35 → 0.34; bracket placed in x 0.22–0.30, y ±0.12; profile further out (x 0.38–0.42) to stay in view and on the table.
- **New fixed test set:** `TEST_SEEDS = [2, 4, 5, 12, 13, 21, 23, 26, 27, 30, 31, 39, 42, 43, 45]` = first 15 seeds passing: top hole visible, detected correctly, grasp reachable (IK < 5 mm). Seeds 0–45 checked; rejected: 21 no visible top hole, 4 not detected, 3 wrong hole, 2 unreachable, 1 no fit. Ideal-depth detector results on this set are 15/15 by construction. Earlier detector / fit / Isaac results used the old layout and old seeds.
- **Grasp result on the new layout, first 5 test seeds: 0/5 lifted**, although all are now reachable (IK error ≤ 0.4 mm) and the approach is within 1.9–2.5° of the true hole normal. Same cause as before: the grasp point must back off 10–11 mm outside the face to keep the jaws off the table (bracket on its corner, approach tilted 43°). Videos: `output/grasp/grasp_seed_002.mp4`, `grasp_seed_004.mp4`.
- **TCP check (user: "I don't think it's at the tip"):** correct. The URDF tip frames (`*_thumb_tip` / `*_index_tip`, used as TCP) are at 132 mm along the jaw from `*_hand_base_link`; the jaw tips are at 143 mm (visual mesh) / 146 mm (collision AABB), i.e. the TCP is 11–14 mm behind the tip. Added `--tcp-offset-mm` (TCP shift along the jaw). Sweep on the 5 seeds, offsets 0 / 4 / 8 / 11 mm: grasp depth −11 / −7 / −3 / 0 mm (1 mm per mm of offset), **still 0/5 lifted at every offset**. Reason (seeds 2, 4; holes 8.7–8.8 mm above the table): even with the TCP 3 mm from the tip, keeping the jaws ≥ 4 mm off the table needs the TCP shifted 14–16 mm up the face, but the leg tip is only 10 mm from the hole, so the pads close beyond the bracket. The binding constraint is the jaw's 35 mm height (±12 mm vertically at the 43° approach) against a hole < 9 mm above the table, not the TCP position.
- **Sideways TCP offset (user idea): solves the table clearance.** TCP moved towards the jaw plate's lower edge (perpendicular to the approach; closing direction flipped so the plate points up): with +8 mm along the jaw and 11 mm sideways the pads reach 4 mm into the part with the jaws ≥ 4 mm above the table in 4/5 seeds (seed 5 then unreachable by IK).
- **Jaw model corrected** from the finger collision meshes (placed in the link's inertial frame): tip at 143.3 mm (not 146), plate 29 mm tall (±14.5, not ±17.5), pad face x 92–143 mm, pads 29.8 mm apart at the open setting. The earlier numbers came from `getAABB`, which includes the collision margin; with them the 15 mm side offset had put the TCP 0.5 mm outside the plate.
- **Grasp point choice: antipodal on the fitted CAD.** Search depth/shift near the hole; among table-clear candidates take the one with the most CAD surface facing each pad (normals along ±closing axis, weaker side counts). A plain "material between the pads" count picked the leg's thin edge.
- **Execution diagnostics** logged per phase (`phase_log`: TCP tracking error, jaw positions, jaw–bracket contacts and normal force, jaw–table contacts, bracket displacement). Arm tracking is 0.1–0.3 mm.
- **Still 0/5 lifted** (tip +8 mm, side 11 mm). Jaws now reach and touch the bracket; with slow equal-speed closing (10 mm/s) they stop on it (−15 to −18 mm) with the bracket moving < 1 mm, but the normal force stays 1–6 N, often on one jaw only, and during a 0.5 s squeeze the jaws slide on to fully shut (−20 mm, 5.8 mm gap). Pad friction 2 / 5 / 10: no change. So the pads are not on the bracket's 20 mm-wide parallel side faces but on edges they slide past; friction is not the limiting factor. Open: verify where the pads actually touch (contact positions on the CAD), e.g. with a close-up render.
- **Jaws closed only to the grasped width (user: "don't fully shut")**: target = CAD width between the antipodal faces (19.9–20.0 mm) minus a squeeze of 2 / 4 / 6 mm (jaw target −13.9 / −14.9 / −15.9 mm instead of −20). Still 0/4 executed scenes lifted. The jaws reach their target every time (they never stall on a 20 mm body) with 0–3.5 N contact force. Contact points in the bracket's CAD frame: both jaws at x = ±9–10 mm (the right faces, the 20 mm width) but only at y ≈ 0, z ≈ 0, i.e. the outer corner ridge of the L, with ~1 mm overlap. Cause: the bracket rests on its leg tips with the corner up (inverted V); the side faces the pads could clamp lie below the ridge, towards the table, and the table clearance keeps the jaw plate above them, so the pads only pinch the rounded ridge, which slips out.
- **Switched to item Line 8 40×40 parts (user, 2026-11-03)** because the 20 mm bracket was too small for the Dex1 jaws. Profile **item 0.0.452.81** (Profile 8 40×40 light natural; export was the full 6 m bar, cut to 200 mm): 355 g. Bracket **item 0.0.670.11** (Angle Bracket Set 8 40×40 with grey cover cap, zinc): one fused solid with cap, 2 M8 button-head screws, 2 T-slot nuts → cut everything beyond the outer faces (nuts, screw tips, and item's small centring lugs, a simplification), drilled both Ø8.5 holes through the 5 mm legs, cut both screw heads; **cap kept** (not separable in the export; item ships it with the set). 40 × 42.2 × 42.2 mm, 182 g at zinc density (overestimate: the cap is plastic). Leg-swap symmetry holds (0.4 mm). Hole geometry now in one place (`scene/parts.py`: diameter, openings, width, symmetries) used by detector, fit, evaluations and grasp. Jaws open to 53.8 mm; placement regions and spacing enlarged.
- **New fixed test set:** `TEST_SEEDS = [4, 8, 13, 20, 23, 24, 26, 27, 32, 35, 43, 45, 48, 69, 92]` (seeds 0–92 checked; rejected: 56 no visible top hole, 18 unreachable, 4 wrong hole).
- **Grasp result, 40×40, first 5 test seeds, default URDF TCP (no offsets), squeeze 4 mm: 4/5 lifted ~100 mm** (seeds 4, 8, 20, 23; approach tilt ~45°, pads ~10 mm into the part, jaws stall on the 39.7–39.8 mm width with 5–8 N per jaw). With TCP offsets +8 mm / 11 mm: 3/5 (seed 20 unreachable), so the default TCP is kept.
- **Failure, seed 13: false hole.** The detector finds both real holes (0.5–0.6 mm, 0.5–0.7°) plus a round 9.6 mm gap on a horizontal surface of the bracket/cap; the "most upward" rule picks the gap, the fit starts there (83° off) and a jaw is driven 5 mm into the part. The seed passed selection on the static scene, which differs from the settled grasp scene by 1 mm.
- Finding for the report (20×20 bracket): a hole-normal grasp with this gripper is geometrically infeasible for the bracket's commonest rest pose; perception accuracy is not the limiting factor here. PyBullet, ideal depth, CAD ground truth (4 hole openings: 2 holes × outer/inner face). Valid GT = openings facing the camera, normal within 50° of vertical, and visible (≥ 6/8 ring points around the opening match rendered depth). Correct = within 3 mm of a valid opening.

**Result, ideal depth, 60 seeds (2026-11-03):** 28 frames with a visible upward hole; **24/28 correct (86%)**; position error median 0.33 mm (max 0.79); axis error median 1.7° (max 3.3°); 1 wrong detection on a frame without a visible hole.
**Hypothesis:** ⬜
**Conditions:** 🟡 ideal depth vs simple noise vs full camera model
**Metrics:** 🟡 pose error (mm, °, ADD-S), grasp success from estimated pose, "assembly-ready" rate
**Results:** ⬜
**What did not work:**
- Hole detector, first version: 25% correct. Causes: (1) scoring counted the second of two equally tilted holes as wrong (bracket resting on its corner, both holes at 45°); (2) 4 mm object threshold > 2 mm leg thickness, so flat legs were classed as table; (3) slot fragments on the profile passed the size test; (4) holes hidden in the bracket's pocket were expected by the GT. Fixed: accept any valid opening, 1.5 mm thresholds, roundness and ≥ 3 px tests, GT visibility check → 86%.
- Multi-frame retry (user idea, 2026-11-03): try up to 10 frames per scene and stop at the first top-hole detection. Ideal depth, PyBullet, 60 seeds: correct 24/28 → 24/28 (no change); detections on scenes without a visible hole 1 → 2 (worse); 6.1 frames used on average. Reason: static scene + ideal depth = identical frames, only RANSAC sampling changes, so occluded holes stay occluded and repeated tries on empty scenes eventually produce a spurious hole. Expected to matter only with per-frame camera noise; then confirm a hole over ≥ 2 frames instead of stopping at the first detection.
- Remaining failures (all 5 cases): hole at the bottom of the bracket's pocket (leg lying flat, inner face up) seen at the G1 camera's ~42° viewing angle, partly hidden by the side walls. The detector either misses it or picks a 3–5 px gap on a 45° inner face (~10 mm, ~45° wrong).
- **Bug found: half-pixel offset in the PyBullet backend.** TinyRenderer samples column c at u = c and row r at v = r + 1, while K assumes pixel centres at +0.5. Detected holes were systematically offset by (+0.40, −0.54) px (std 0.15) → 0.87 mm median error. Fixed with an off-centre projection matrix; offset now (+0.07, 0.00) px and median error 0.33 mm. All ground truth from earlier PyBullet frames had this offset. **To check: Isaac's pixel-centre convention.**
- Isaac backend bugs found while testing headless (all fixed):
  - Camera API setters write focal length / aperture in stage units (m → focal 0.11); the RTX renderer ignores such values and renders with its default 50 mm lens (~25° FOV) while `get_intrinsics_matrix()` still reports the requested K. Measured with a 1 m target at 2 m: 848 px wide instead of 223. Fixed by setting the USD attributes at mm scale (aperture 20.955, focal 11.04) → 224 px.
  - `SimulationApp.close()` ends the Python process, so frames were never saved; close is now called after saving.
  - RGB key is `rgb` in Isaac 5.1 (not `rgba`).
  - The G1 asset's soft joint drives (stiffness 100) let the waist sag under gravity, so the head camera moved while frames rendered: holes were offset by (0.0, −1.05) px and −1.2 mm in depth (median position error 2.09 mm vs 0.33 mm in PyBullet). Fixed with stiff drives (kp 1e6, kd 1e4); camera drift during rendering now 0.003 mm and its pose equals the PyBullet one.
- **Isaac vs PyBullet after the fixes (ideal depth, same 30 seeds):** both 14/15 correct (93%), same missed seed (14, pocket hole), position error median 0.34 / max 1.00 mm in both, axis error median 2.0° (Isaac) vs 1.9° (PyBullet). Camera drift in Isaac ≤ 0.0033 mm. The two backends now agree on geometry, so results from PyBullet development carry over to Isaac.
- Isaac renders the grey table almost white (lighting intensities were guesses): cosmetic, open.
**Known limitation:** evaluation is synthetic-only → shows sensitivity to the model, not that the model is realistic.

## Q4. Riskiest sim-to-real assumption + 6-month PhD plan (≤1 page)
⬜ (collect ideas here as they come up)

## Q5. AI-tool use note
- Used Claude Code to read the task, structure the problem, find and extract camera specs from the datasheet and tuning paper.
- Corrections / disagreements with the tool:
  - Camera: the user stated the G1 has a D415 by default; checking the G1 URDF showed it is a D435 (`d435_link`). Decided to switch the reference camera to the D435 (fact-check of a user assumption, not a tool error).
  - Simulator: the AI recommended BlenderProc for simplicity and reviewer reproducibility; the user chose Isaac Sim instead (physics/robot integration, industrial relevance) and accepted the reproducibility cost, mitigated by decoupling the camera model from the renderer.

---

## Decision log
| Date | Decision | Alternatives considered | Reason |
|---|---|---|---|
| 2026-11-03 | Core of the work = camera depth model; downstream task kept simple | Full assembly simulation | Task asks for one downstream step; assembly sim doesn't test the camera model |
| 2026-11-03 | Camera: RealSense D415 | D435, Azure Kinect / Femto (ToF), Zivid | Best published noise characterisation; lower noise than D435; stereo failure modes fit aluminium |
| 2026-11-03 | Camera changed D415 → D435 | Keep D415 at the G1 head mount | G1 really carries a D435; D415 Min-Z (450 mm) risky at G1 head-to-table distance; D435 noise is larger, so effects are easier to see |
| 2026-11-03 | Table shrunk to G1 reach (0.40 × 0.60 m at x = 0.35 m) | Original 0.8 × 1.2 m at 0.60 m | Parts must be graspable by the G1; side effect: parts closer to the camera (bigger, less noise) |
| 2026-11-03 | Keep PyBullet as alternative backend | Isaac only | GPU busy; reviewer reproducibility on a laptop |
| 2026-11-03 | Simulator: Isaac Sim | BlenderProc (my recommendation), pyrender, PyBullet | User's choice: physics/robot integration and industrial relevance outweigh the install and reproducibility cost |
