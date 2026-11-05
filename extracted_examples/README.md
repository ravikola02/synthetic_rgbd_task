# Extracted examples: where the D435 model breaks the downstream task

Cases taken from the final sweep (`output/final_sweep/`, 30 test seeds where every ideal-depth stage succeeds,
5 noise draws per seed). Each one is recomputed with the same code and the same random numbers as its sweep row
(`python experiments/extract_examples.py`; `--no-video` for figures only). Every seed here lifts the bracket with
ideal depth. The grasp examples (04–06) also lift with the datasheet rules alone (the control condition), so their
failures are not just the grasp planner's sensitivity (see 08).

Legend of the detection / fit figures: green ring = true top hole, orange = hole candidates, red × = the detector's
pick; fit = CAD model points on the measured surface (green) or off it (red).

## Perception failures

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 01 | 204, 1 | axial noise | The noise breaks up the bracket's faces, so the hole is no longer a gap enclosed by a face: **no hole candidate**, so the CAD fit cannot start and the pipeline stops (no pose, no grasp). | `01_axial_noise_no_hole.png` |
| 02 | 299, 0 | axial vs edge + axial | **Same noise draw**: axial alone finds the hole (1.0 mm), but with edge fattening the detector picks a false hole on the profile 163 mm away. Edge fattening amplifies the noise effect (sweep: 56 % → 39 % detected, p = 0.005). | `02_edge_fattening_amplifies_noise.png` |
| 03 | 380 | edge fattening alone | The hole is filled with foreground depth (a plug ~6 mm in front of the face instead of a 4 mm deep gap). It is still detected (edge alone: 100 %), but it looks 10.0 mm wide instead of 6.7 mm and its axis tilts from 0.8° to 3.0°: an accuracy effect, not a detection failure. | `03_edge_fattening_fills_hole.png` |
| 07 | 431, 3 | full model | Every hole candidate is false and the best-scoring CAD match lands on the **wrong part: pose 253 mm / 43° off**. The grasp planner then finds no reachable grasp (IK failed), so the robot does not move. | `07_full_model_wrong_pose.png` |

## Grasp failures (ideal and rules-only both lift)

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 04 | 204 (all draws) | edge fattening alone | Perception barely changes (hole 2.0 mm, fit 0.4 mm), yet the grasp point moves and the jaws **push the bracket 39 mm** instead of lifting it. Edge fattening has no randomness, so it fails on every draw. | `04_edge_fattening_grasp_fails.png`, `04_edge_fattening_grasp_frames.png`, `04_ideal_grasp_seed204.mp4`, `04_edge_fattening_grasp_seed204.mp4` |
| 05 | 233, 1 | depth bias (+2 %) | The hole is found and the shape and rotation are right, but the whole part sits **10.7 mm further along the viewing ray** (2 % of ~0.53 m). The jaws close off the part and **knock it over** (pushed 64 mm). | `05_depth_bias_shift.png` (side view: measured points, true and fitted CAD), `05_depth_bias_grasp_frames.png`, `05_ideal_grasp_seed233.mp4`, `05_depth_bias_grasp_seed233_draw1.mp4` |
| 06 | 204, 2 | full model | **Every stage degrades**: the detector picks a false hole on the profile (161 mm away), the fit recovers the bracket but 11.6 mm off (bias), and the jaws close off-centre and **tip the bracket over** (pushed 25 mm). Same scene as 04. | `06_full_model_grasp_fails.png`, `06_full_model_grasp_frames.png`, `04_ideal_grasp_seed204.mp4` (ideal), `06_full_model_grasp_seed204_draw2.mp4` |

## Control: the grasp planner's own sensitivity

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 08 | 246 (all draws) | datasheet rules alone | Perception is unchanged (same hole, fit 0.2 vs 0.3 mm), yet the grasp **fails (IK)**: the 1 mm depth rounding nudges the pose and the grasp search lands on a candidate the arm cannot reach. In the sweep the rules alone drop the grasp from 100 % to 77 % (7 of 30 seeds; rules have no randomness, so each seed fails or lifts on all 5 draws). Part of every grasp drop vs ideal is this planner sensitivity: judge the camera effects on the grasp against the rules row (77 %), not only against ideal (100 %). | `08_control_rules_planner_sensitivity.png` |

## Points for the report

1. **The detector is where the camera model bites.** Axial noise creates false holes (01, 02, 07); edge fattening
   on its own does not stop detection but makes the noise effect worse (02) and degrades the hole's geometry (03).
2. **The CAD fit absorbs most perception errors** (pose found in 99–100 % of draws under every condition), but not
   all: with no candidate on the bracket it cannot start (01), and with only false candidates it can lock onto the
   wrong part (07).
3. **The depth bias is a pure shift along the viewing ray** (05): shape and rotation stay right, the position is
   off by 2 % of the distance (~11 mm). The grasp tolerates this often (bias: 66 % lifted vs 77 % for the rules
   control) but not always (05, 06).
4. **Small perception changes can flip the grasp** (04, 08): the grasp planner is sensitive to ~1 mm pose changes,
   which is why the rules-only control is needed to interpret the grasp column.
