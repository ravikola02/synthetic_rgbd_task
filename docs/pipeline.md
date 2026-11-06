# Pipeline: algorithms and math

From a random seed to a lifted bracket, and how the experiment measures each step. The depth camera model in
the middle of the pipeline has its own document: [`camera_model.md`](camera_model.md).

```
seed ──► scene spec ──► render (PyBullet / Isaac Sim) ──► ideal RGB-D + instance mask + CAD poses
                                                                 │
                                         camera model (D435) ◄───┘   (or ideal depth)
                                                 │
                     depth ──► 1. top-hole detector ──► 2. bracket CAD fit ──► 3. grasp at the hole (PyBullet)
                                       │                       │                       │
                                       └──── evaluated against CAD ground truth ───────┘
```

Conventions: camera frame = ROS optical frame (+Z forward, +X right, +Y down). Poses are 4×4 homogeneous
transforms $T_{ab}$ mapping frame $b$ into frame $a$. Depth is z-depth in metres (0 = invalid).

## 1. Scene generation (`scene/spec.py`, `scene/config.py`, `scene/parts.py`)

A seed fully determines the scene. Every backend builds the same plain-data spec, so seed $s$ is the same scene
in PyBullet and Isaac Sim.

- **Robot and camera.** Unitree G1 with Dex1 grippers, arms in a fixed "observe" pose. The camera is the G1's
  torso D435 (`d435_link`), with an 848×480 depth stream and $f = 447$ px.
- **Table.** 0.40 × 0.60 m, top 0.75 m above the floor, in front of the robot.
- **Parts.** Two item Line 8 parts from their CAD (STEP): a 40×40×200 mm aluminium profile and a 40×40 bracket
  with two 8.5 mm (M8) through-holes.
- **Pose sampling.**
  1. *Stable resting pose.* trimesh computes the part's stable poses on a plane and their probabilities
     (`compute_stable_poses`: the faces of the convex hull, weighted by a Monte Carlo of 2000 random drops; cached). One is drawn by its probability, so the
     common rest poses (e.g. the bracket on its 45° corner) appear as often as they would physically.
  2. *Yaw* $\psi \sim \mathcal U(0, 2\pi)$ about the vertical.
  3. *Position* uniform in a per-part region of the table; profile and bracket are resampled until their
     centres are $> 0.16$ m apart, so they never overlap.

  The CAD frame's world pose is
  $T = \mathrm{Trans}(x, y, z_{\text{table}})\,R_z(\psi)\,\mathrm{Trans}(-\mathbf c)\,T_{\text{rest}}$, where
  $\mathbf c$ centres the rested part's footprint at the origin with its lowest point on $z = 0$.
- For perception the parts are placed at rest and kept static (identical in both backends). For the grasp they
  are dynamic and settle under physics first (about 1 mm different from the static frames).

## 2. Rendering and ground truth (`scene/backends/`, `scene/frame_io.py`)

Both backends return RGB, metric z-depth, an instance mask (0 background, 1 table, 2 robot, 3+ parts) and the
exact part and camera poses.

- **PyBullet** (TinyRenderer) returns an OpenGL depth buffer $b \in [0,1]$ with near/far planes $n, F$, converted
  to metric z-depth by
  $$Z = \frac{F\,n}{F - (F - n)\,b} .$$
  The projection matrix is built from $K$, with a half-pixel shift: TinyRenderer samples pixel $(c, r)$ at
  $(c, r+1)$, while $K$ puts pixel centres at $+\tfrac12$. Without the shift every image was off by half a pixel,
  which the hole detector exposed as a systematic offset against CAD ground truth.
- **Isaac Sim 5.1** (RTX) uses its `distance_to_image_plane` annotator, which is z-depth directly.
- **Ground truth** comes from the CAD and the exact poses. The bracket's hole *openings* are 2 holes × 2 faces
  (outer, and inner one leg thickness deeper), each with a centre and an outward normal in the CAD frame. They
  are transformed into the camera frame, $\mathbf c_{\text{cam}} = T_{cw}T_{wo}\,\mathbf c_{\text{obj}}$.

## 3. Top-hole detector (`perception/hole_detector.py`)

Input: depth and $K$ only (no RGB, no learning). Output: hole poses in the camera frame. A round hole's pose is
its opening centre $\mathbf c$ and axis $\mathbf n$ (5 DoF; the rotation about the axis is undefined).

1. **Back-projection.** $\mathbf p(u,v) = Z(u,v)\,\mathbf r(u,v)$ with $\mathbf r = ((u+\frac12-c_x)/f,\
   (v+\frac12-c_y)/f,\ 1)$. Only pixels with $0 < Z < 1$ m are kept, which drops the floor.
2. **Table plane: RANSAC.** On 5000 sampled points, 300 times: take 3 random points, form the plane
   $\mathbf n = \frac{(\mathbf p_2-\mathbf p_1)\times(\mathbf p_3-\mathbf p_1)}{\lVert\cdot\rVert}$,
   $d = -\mathbf n\cdot\mathbf p_1$, and count the inliers $|\mathbf n\cdot\mathbf p + d| < \tau$. The best
   plane is refined by least squares: the inliers' centroid $\mathbf m$ and the normal = the right singular
   vector of $(\mathbf p_i - \mathbf m)$ with the smallest singular value, oriented towards the camera.
3. **Objects.** The height above the table is $h = \mathbf n_t\cdot\mathbf p + d_t$; object pixels have
   $h > 1.5$ mm. Connected components (8-neighbourhood) give one cluster per part.
4. **Face peeling.** In each cluster, up to 4 planar faces are found one after another by RANSAC
   (inliers $< 1$ mm, at least 15 px). Each face keeps its largest connected patch and is removed before the next.
5. **Holes = enclosed gaps.** Binary hole filling of the face mask, minus the face, gives the regions *enclosed
   by* the face but *not on* it. Each region is a hole candidate if it passes three tests:
   - *Size.* Each gap pixel's ray is intersected with the face plane,
     $t = -d_f / (\mathbf r\cdot\mathbf n_f)$, $\mathbf X = t\,\mathbf r$. A pixel's footprint on the plane is
     $$A_{\text{px}} = \frac{Z^2\,\hat r_z}{f_x f_y\,|\hat{\mathbf r}\cdot\mathbf n_f|}$$
     (pixel solid angle × distance², divided by the obliquity). The equivalent diameter
     $D = \sqrt{4\sum A_{\text{px}}/\pi}$ must lie in $[0.6, 1.6] \times 8.5$ mm, and the region needs at least 3 px.
   - *Roundness.* The two in-plane eigenvalues $\lambda_1 \le \lambda_2$ of the covariance of the points
     $\mathbf X$ must satisfy $\sqrt{\lambda_2/\lambda_1} \le 2$. This rejects slot fragments.
   - The candidate's pose is centre $\mathbf c = \overline{\mathbf X}$ and axis $\mathbf n = \mathbf n_f$.
6. **Top hole.** The candidate whose axis is closest to the table normal (largest $\mathbf n\cdot\mathbf n_t$),
   if within 50° of it.

The thresholds (1 mm plane inliers, 1.5 mm object height) are tuned on ideal depth. With D435 noise
($\sigma_Z \approx 1$ mm) they sit at the noise level, which is the camera model's main effect: faces break
up, noise leaves enclosed gaps on flat faces, and those pass as holes. A noise-aware variant
(`hole_detector_tuned.py`, a mitigation outside the baseline comparison) sets each threshold to
$\max(\text{baseline},\ k\,\sigma_Z(Z))$ with $k = 2.5$.

**Evaluation** (`perception/eval_holes.py`).
- *Valid top openings:* the opening faces the camera, $\mathbf n_{\text{cam}}\cdot(-\mathbf c/\lVert\mathbf c\rVert) > 0.2$;
  its normal is within 50° of vertical; and it is *visible*: at least 6 of 8 points on a ring 2 mm outside the
  hole edge match the ideal depth within 2 mm.
- *Hole detected* = the pick is within 15 mm of a valid opening of a visible top hole. 15 mm allows the
  depth-bias shift.
- *Within 3 mm* = an accuracy threshold.
- *Errors:* position = distance to the nearest opening; axis error = $\arccos|\mathbf n\cdot\mathbf n_{\text{gt}}|$.

## 4. Bracket CAD fit (`perception/bracket_fit.py`)

The hole fixes 5 of the bracket's 6 DoF. What's left is *which* of the 4 openings was seen, and the rotation
$\varphi$ about the hole axis.

1. **Hypotheses.** For opening $o$ (CAD centre $\mathbf c_o$, normal $\mathbf n_o$) and $\varphi \in \{0°, 10°,
   \dots, 350°\}$: build a frame on the opening, $T_{\text{obj,open}} = [R(\mathbf n_o), \mathbf c_o]$, and the
   matching frame on the detected hole, $T_{\text{cam,open}} = [R(\mathbf n)\,R_z(\varphi), \mathbf c]$. Here
   $R(\mathbf z)$ has $\mathbf z$ as its third column, and its first column is a reference axis projected off
   $\mathbf z$ (Gram–Schmidt). The bracket pose is
   $$T_{\text{cam,obj}} = T_{\text{cam,open}}\,T_{\text{obj,open}}^{-1}.$$
   That is $4 \times 36 = 144$ hypotheses per hole candidate.
2. **Score** (depth only). 4000 surface samples of the CAD (with normals) are posed. A point is *visible* if
   its normal faces the camera ($\mathbf n\cdot(-\mathbf X) > 0$) and it is in front in a z-buffer over its
   pixel $(u,v) = (fX/Z + c_x,\ fY/Z + c_y)$. For each visible point with residual $r = Z_{\text{obs}}(u,v) - Z_{\text{model}}$:
   $$\text{inlier}: |r| < 3\ \text{mm}, \qquad \text{violation}: r > 5\ \text{mm (the camera sees through the model)},$$
   $$\text{score} = \#\text{inliers} - 2\,\#\text{violations}.$$
3. **ICP refinement** of the best hypothesis on the depth points of the hole's cluster (30 iterations):
   visible model points $\mathbf x_i$ are matched to their nearest observed points $\mathbf y_i$ (k-d tree),
   keeping pairs closer than 4 mm. Then
   $$\min_{R,\mathbf t}\sum_i \lVert R\mathbf x_i + \mathbf t - \mathbf y_i\rVert^2$$
   is solved in closed form (Kabsch/Umeyama): with centred sets, $H = \sum(\mathbf x_i - \bar{\mathbf x})
   (\mathbf y_i - \bar{\mathbf y})^\top = U\Sigma V^\top$,
   $R = V\,\mathrm{diag}(1, 1, \det VU^\top)\,U^\top$, $\mathbf t = \bar{\mathbf y} - R\bar{\mathbf x}$.
   The update is $T \leftarrow [R, \mathbf t]\,T$.
4. **Every upward candidate is tried** (axis within 50° of up), and the fit with the highest score wins. Taking
   only the detector's top pick failed when a round gap on the bracket's cap passed as a hole. This is also why
   the fit survives most false detections: a false hole on the profile gives a poor CAD match.

**Pose errors** (`pose_errors`). The bracket maps onto itself under a 180° turn $S$ about $(0,1,1)/\sqrt2$, which
swaps the legs. So
$$e_t = \lVert\mathbf t_{\text{est}} - \mathbf t_{\text{gt}}\rVert, \qquad
e_R = \min_{S\in\{I, S\}} \arccos\frac{\mathrm{tr}\big((R_{\text{gt}}S)^\top R_{\text{est}}\big) - 1}{2}.$$
- *Success:* $e_t < 5$ mm and $e_R < 10°$.
- *Pose found:* $e_t < 15$ mm and $e_R < 10°$ (allows the depth-bias shift).
- ADD-S is reported but not used: the compact, near-cubic bracket gives ADD-S of 2–3 mm even for 90–150° wrong poses.

## 5. Grasp at the hole (`robot/grasp_hole.py`, PyBullet)

Perception output only (detected hole + fitted CAD), no ground truth for control.

1. **Grasp frame.**
   - *Approach* (hand $x$): into the face, normal to the hole, $\mathbf a = -R_{wc}\mathbf n$.
   - *Closing* (hand $y$): the bracket's 40 mm width axis from the fitted CAD, made perpendicular to the approach,
     $\mathbf b = \frac{\mathbf x_{\text{obj}} - (\mathbf x_{\text{obj}}\cdot\mathbf a)\mathbf a}{\lVert\cdot\rVert}$.
     It's flipped if needed so that the hand $z = \mathbf a\times\mathbf b$ points up (the jaws are symmetric).
2. **Grasp point search.** The TCP is placed at $\mathbf c + \delta\,\mathbf a + s\,\mathbf u$:
   - $\delta \in [-3, 12]$ mm is the depth behind the opening;
   - $s \in [-25, 25]$ mm is a shift along the face's uphill direction $\mathbf u$;
   - both in 1 mm steps.

   Every candidate must:
   - keep all jaw corners at least 4 mm above the table;
   - have no CAD point in the fingers' volume swept along the straight 10 cm approach (no collision on the way in).

   The candidate kept is the most *antipodal*: CAD surface samples inside the closing volume whose normal is along
   $\pm\mathbf b$ ($|\mathbf n\cdot\mathbf b| > 0.9$) are counted per pad, and the weaker pad counts. At least 15 points are
   required. The grasped width is the median distance between the two pad-facing surfaces. The jaws are
   commanded to that width minus 4 mm, so they stall on the part and press.
3. **Inverse kinematics.** Damped least squares on the 7 arm joints, in a separate kinematics-only PyBullet
   client. With the pose error $\mathbf e = (\mathbf p^* - \mathbf p,\ \tfrac12\,\theta\hat{\boldsymbol\omega})$,
   where $\theta\hat{\boldsymbol\omega}$ is the axis-angle of $R^* R^\top$, and the geometric Jacobian $J$ (6×7):
   $$\Delta\mathbf q = J^\top\left(JJ^\top + \lambda^2 I\right)^{-1}\mathbf e, \qquad \lambda = 0.02,$$
   - each step is clipped to 0.2 rad and to the joint limits;
   - up to 300 iterations;
   - up to 8 attempts: from the current arm pose, then from 7 perturbed ones ($\mathcal N(0, 0.5)$ rad);
   - both arms are tried, and the better one is used.

   Waypoints: pre-grasp 10 cm out along $-\mathbf a$, a straight-line approach in 10 steps, close, squeeze,
   a straight lift of 10 cm in 10 steps, then a 1 s hold. Nothing is executed if any waypoint misses by more
   than 5 mm ("IK failed").
   - **Body collision check.** PyBullet runs without self-collision, so the planner checks it: along the whole
     motion (the move to the pre-grasp at 20 joint-space steps, every approach and lift waypoint), the grasping
     arm from the upper arm to the jaws must stay at least 10 mm from the rest of the robot (pelvis, legs,
     torso, head, other arm), measured with `getClosestPoints` on the collision meshes. An attempt that fails
     this is discarded and the next one tried; if no attempt of either arm is clear, nothing is executed
     ("body collision"). The clearance during execution is logged too (`body_clearance_exec_mm`).
4. **Execution** with physics at 240 Hz: position-controlled joints, jaws closing slowly at equal speed, and
   rubber-like pad friction. **Success** = the bracket is lifted more than 5 cm and touches both jaws after the
   hold. Statuses: lifted / executed, not lifted / IK failed / body collision / no table-clear grasp / no fit.

## 6. Experiment and statistics (`experiments/final_sweep.py`)

- **Conditions** (7): ideal; datasheet rules; axial noise; edge fattening; edge + axial; depth bias ±2 %; full
  model. Each one switches parts of the camera model on or off.
- **Test seeds.** The first 30 seeds from 201 up for which every ideal-depth stage succeeds (hole visible,
  detector within 3 mm, fit success, grasp lifted). This is the same rule as the development seeds, so a
  failure under a condition is caused by the camera model.
- **Draws.** Each seed gets 5 noise draws, seeded by (seed, draw). Every condition sees the same random numbers
  (see `camera_model.md` §7), so outcomes are **paired** per draw. Ideal depth is deterministic: one run per seed,
  repeated for the pairing. The jobs run in parallel processes, which does not change any number.
- **Success rates** $\hat p = k/n$ with the exact (Clopper–Pearson) 95 % interval:
  $$p_{\text{lo}} = \mathrm{Beta}^{-1}\!\left(\tfrac{\alpha}{2};\,k,\,n-k+1\right),\quad
  p_{\text{hi}} = \mathrm{Beta}^{-1}\!\left(1-\tfrac{\alpha}{2};\,k+1,\,n-k\right),\quad \alpha = 0.05.$$
- **Paired comparison** (exact McNemar test). Over paired draws, let $n_{01}$ count draws that succeed only in
  condition A and $n_{10}$ those that succeed only in B. Under $H_0$ (no difference),
  $n_{01} \sim \mathrm{Bin}(n_{01} + n_{10}, \tfrac12)$, and the p-value is the two-sided binomial test. This is
  used for every condition vs ideal, and for edge + axial vs axial (does edge fattening amplify the noise?).
- **Seed bootstrap.** The 5 draws of a seed share the scene, so they are not independent. Resampling whole
  seeds (2000×) gives a 95 % interval for rates and medians that respects this.
- **Accuracy:** medians (with seed-bootstrap intervals) of the position and axis errors, fit translation and
  rotation, and the hole position and approach errors at grasp time. They're computed over draws that pass the
  15 mm criterion, so the bias shows up as error, not as failure.
- **Control.** The rules change perception by < 1 mm, yet lower the grasp from 100 % to 77 %: the grasp
  planner's own sensitivity (its candidate search switches on ~1 mm pose changes). Camera effects on the grasp
  should be read against this row as well as against ideal.

```bash
python experiments/final_sweep.py all        # select seeds, run, analyse; live web view
python experiments/run_seed.py --seed 204 --condition 6_full --run 2 --gui
python experiments/extract_examples.py       # extracted_examples/
```
