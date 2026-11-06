# Extracted examples: what the D435 model does to the downstream task

Cases taken from the final sweep (`output/final_sweep/`, 30 test seeds where every ideal-depth stage succeeds,
5 noise draws per seed). Each one is recomputed with the same code and the same random numbers as its sweep row
(`python experiments/extract_examples.py`; `--no-video` for figures only). Grasps are planned with the arm-body
collision check (the arm stays at least 10 mm from the robot's own body along the whole motion), so every motion
shown is one the G1 can make.

Legend of the detection / fit figures: green ring = true top hole, orange = hole candidates, red × = the detector's
pick; fit = CAD model points on the measured surface (green) or off it (red).

## Perception: where the camera model bites, and how the fit absorbs it

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 01 | 243, 2 | axial noise | The noise breaks up the faces and leaves enclosed gaps that pass as holes: **11 false holes**, the detector picks one on the profile 177 mm away. The CAD fit tries every candidate and still finds the bracket (1.1 mm off), and the grasp lifts it. | `01_axial_noise_false_holes.png`, `01_grasp_frames.png`, `01_ideal_grasp_seed243.mp4`, `01_axial_noise_grasp_seed243_draw2.mp4` |
| 02 | 299, 0 | axial vs edge + axial | **Same noise draw**: axial alone finds the hole (1.0 mm); with edge fattening the hole is filled and a false hole on the profile 163 mm away wins. Edge fattening amplifies the noise effect (sweep: 63 % → 39 % detected, p = 6.5e-5). The fit and the grasp still succeed. | `02_edge_fattening_amplifies_noise.png`, `02_grasp_frames.png`, `02_ideal_grasp_seed299.mp4`, `02_edge_axial_grasp_seed299_draw0.mp4` |
| 03 | 351 | edge fattening alone | The hole is filled with foreground depth (a plug in front of the face instead of a gap). It is still detected (edge alone: 100 %), but it looks 8.5 mm wide instead of 7.1 mm and its axis tilts from 0.4° to 1.7°: an accuracy effect, not a detection failure. | `03_edge_fattening_fills_hole.png`, `03_grasp_frames.png`, `03_ideal_grasp_seed351.mp4`, `03_edge_fattening_grasp_seed351_draw0.mp4` |
| 04 | 245, 0 | depth bias (+2 %) | Shape and rotation are right, but the whole part sits **10.5 mm further along the viewing ray** (2 % of ~0.52 m). The jaws are wide enough to absorb this: the bracket is lifted. | `04_depth_bias_shift.png` (side view: measured points, true and fitted CAD), `04_grasp_frames.png`, `04_ideal_grasp_seed245.mp4`, `04_depth_bias_grasp_seed245_draw0.mp4` |
| 05 | 432, 0 | full model | **Every perception stage degrades and the grasp still succeeds**: the detector picks a false hole on the profile 278 mm away, the fit recovers the bracket but 10.2 mm off (bias), and the jaws still close on it and lift it. | `05_full_model_false_hole_still_lifted.png`, `05_grasp_frames.png`, `05_ideal_grasp_seed432.mp4`, `05_full_model_grasp_seed432_draw0.mp4` |

## Where the pipeline stops (the robot does not move, so no video)

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 06 | 431, 3 | full model | Every hole candidate is false and the best-scoring CAD match lands on the **wrong part: pose 253 mm / 43° off** (the only draw in 150 under the full model without a pose). The grasp planner then finds no reachable grasp (IK failed). | `06_full_model_wrong_pose.png` |
| 07 | 511 (all draws) | depth bias | Hole and shape are right, the pose is 10.3 mm off. Every arm motion that reaches the shifted grasp would pass the arm 4.5 mm into the torso, so the planner rejects it (**body collision**). The bias moves the grasp target, and that alone can make it unreachable. | `07_depth_bias_body_collision.png` |

## Control: the grasp planner's own sensitivity

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 08 | 246 (all draws) | datasheet rules alone | Perception is unchanged (same hole, fit 0.23 vs 0.25 mm), yet the grasp **fails (IK)**: the 1 mm depth rounding nudges the pose and the grasp search lands on a candidate the arm cannot reach. In the sweep the rules alone drop the grasp from 100 % to 77 % (7 of 30 seeds; rules have no randomness, so each seed fails or lifts on all 5 draws). Judge the camera effects on the grasp against this row (77 %), not only against ideal (100 %). | `08_control_rules_planner_sensitivity.png` |

## Lifted, but misaligned

| # | Seed, draw | Condition | What happens | Files |
|---|---|---|---|---|
| 09 | 473, 3 | full model | Counted as **lifted** in the sweep, but the grasp is off: the pose is 8.6 mm off, the jaws catch the bracket by one corner, and it swings in the grip during the lift (it moves 37 mm relative to the hand between closing and holding; ideal: < 2 mm) and hangs tilted, lifted 53 mm instead of 100 mm. With ideal depth the jaws grip it squarely. A grasp that would not survive a placement or an insertion. | `09_grasp_frames.png`, `09_ideal_grasp_seed473.mp4`, `09_full_model_grasp_seed473_draw3.mp4` |

## Points for the report

1. **The detector is where the camera model bites.** Axial noise creates false holes (01, 06); edge fattening on its
   own does not stop detection but makes the noise effect worse (02) and degrades the hole's geometry (03).
2. **The CAD fit absorbs almost all of it** (pose found in 99–100 % of draws under every condition): it tries every
   candidate and keeps the one that matches the bracket (01, 02, 05). It fails only when every candidate is false
   and a wrong part matches better (06).
3. **The depth bias is a pure shift along the viewing ray** (04): shape and rotation stay right, the position is
   off by 2 % of the distance (~11 mm).
4. **Every executed grasp lifts the bracket**, even 10–11 mm off (04, 05): the open jaws (54 mm for the 40 mm
   bracket) absorb the shift. The grasp drops (69–79 % vs 77 % for the rules control) are all planning failures:
   a millimetre change moves the grasp target to one the arm cannot reach (08) or can reach only through its own
   body (07).
