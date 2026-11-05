# D435 depth camera model: algorithms and math

Code: [`camera_model/d435.py`](../camera_model/d435.py) (`simulate(depth, K, rng, edge, axial, bias, rules)`).
Choices and sources: NOTES.md Q2. Experiment that uses it: NOTES.md Q3 and [`pipeline.md`](pipeline.md).

The model turns an **ideal rendered depth image** into the depth a RealSense D435 would report. It is
analytical, so it runs on any renderer's output (PyBullet and Isaac Sim share it). It applies three effects with
a published number, plus the output rules from the datasheet, in the order they arise in the camera.

| Step | Effect | Kind | Source of the number |
|---|---|---|---|
| 1 | Edge fattening | deterministic | Keselman et al. 2017 §4.2.3 (R200: gaps < 5 px merge) |
| 2 | Axial noise | random (per pixel) | Intel BKM tuning guide §3e (subpixel RMS 0.08 px) |
| 3 | Depth bias | random (sign per draw) | Datasheet Table 4-15 (Z-accuracy ±2 %) |
| 4 | Output rules: left invalid band, Min-Z, 1 mm unit | deterministic | Datasheet §4.5, Table 4-12; BKM §8a |

## 1. Camera geometry

**Image and intrinsics.** The 848×480 depth stream (the datasheet's recommended resolution) with depth field of
view $\mathrm{HFOV} = 87°$ gives the focal length in pixels:

$$f = \frac{W}{2\tan(\mathrm{HFOV}/2)} = \frac{848}{2\tan 43.5°} \approx 447\ \text{px}, \qquad
K = \begin{pmatrix} f & 0 & c_x \\ 0 & f & c_y \\ 0 & 0 & 1 \end{pmatrix},\quad (c_x, c_y) = (424, 240).$$

**Depth is z-depth.** The D435 reports $Z$, the distance along the optical axis, not the range along the ray
(datasheet §2.3). The renderers produce the same quantity. A pixel $(u, v)$ with depth $Z$ back-projects to

$$\mathbf{p} = Z\,\mathbf{r}(u, v), \qquad \mathbf{r}(u,v) = \left(\frac{u + \tfrac12 - c_x}{f},\ \frac{v + \tfrac12 - c_y}{f},\ 1\right)^\top .$$

**Stereo.** The D435 matches the image of its left IR imager against the right one, with baseline
$B = 50$ mm (datasheet Table 3-11). A point at depth $Z$ appears shifted between the two images by the disparity

$$d = \frac{f B}{Z}\ \ \text{[px]}, \qquad Z = \frac{f B}{d}.$$

At the parts' distance (0.47–0.57 m) $d \approx 39$–$48$ px. Several effects are simplest in disparity, so
the model converts $Z \to d$, applies them, and converts back (`to_disparity`, `to_depth`; $Z = 0$ marks
invalid pixels and stays invalid).

## 2. Edge fattening (`edge_fattening`, `edge="r200"`)

**Mechanism.** A stereo matcher compares *windows* of pixels, not single pixels. At a depth step the window
straddles both sides, and the foreground (larger disparity) dominates the match. So the
foreground grows into the background, and narrow gaps such as a hole seen from the side close up.

**Number.** No D435 value is published. The only measured value is for Intel's older R200 (Keselman et al. 2017
§4.2.3): it "incorrectly merg[es] gaps between edges of less than 5 pixels" (7×7 Census transform with 7×7 box
aggregation). Intel's D400 post-processing paper implies an x-y resolution of the same order (sub-sampling by
< 3 px has "fairly minimal impact"). Gaps $\le 4$ px closing corresponds to the foreground growing
$g = 2$ px from each side.

**Algorithm.** In disparity $d(u,v)$:

1. *Rim pixels* (foreground side of a step): pixels closer than some 3×3 neighbour by more than a threshold,
   $$\text{rim}(u,v) = \Big[d(u,v) - \min_{3\times3} d > \delta\Big], \qquad \delta = 0.5\ \text{px}.$$
   The threshold separates real steps from slanted faces. A plane seen at angle $\theta$ changes its depth by
   $Z\tan\theta / f$ per pixel, so its disparity by $\frac{fB}{Z^2}\cdot\frac{Z\tan\theta}{f} = \frac{B\tan\theta}{Z}$
   per pixel: $\approx 0.29$–$0.37$ px at $\theta = 75°$ and $Z = 0.5$–$0.65$ m, below $\delta$, so it is not an edge.
2. *Foreground disparity near each pixel:* the mean disparity of the rim pixels in a $(2g+1)\times(2g+1) = 5\times5$
   window (box filters of the rim mask and of the masked disparity),
   $$d_{\text{fg}}(u,v) = \frac{\sum_{w} \text{rim}\cdot d}{\sum_{w} \text{rim}} \quad\text{(where any rim pixel is in the window).}$$
3. *Growth:* a pixel that lies behind that foreground takes its disparity,
   $$d'(u,v) = \begin{cases} d_{\text{fg}}(u,v) & \text{if } d_{\text{fg}}(u,v) - d(u,v) > \delta \\ d(u,v) & \text{otherwise.}\end{cases}$$

So every background pixel within 2 px of a foreground rim is pulled to the rim's depth. The filled pixels sit at
the rim's depth, not on the face plane behind: a hole in a slanted face becomes a "plug" a few mm in front of
the face (`extracted_examples/03_edge_fattening_fills_hole.png`). Checked on synthetic gaps: 3–4 px gaps close,
a 5 px gap keeps 1 px, and a 75° plane is unchanged.

**Limits.** The growth width is the R200's and the fill rule is ours. Both should be calibrated against real
D435 depth. The effect is deterministic: the same frame always gives the same result.

## 3. Axial noise (`axial_intel`, `axial="intel"`)

**Mechanism.** The matcher finds the disparity to a fraction of a pixel. Its error $\varepsilon_d$ (the
*subpixel* RMS $s$, px) propagates to depth through $Z = fB/d$:

$$\frac{\partial Z}{\partial d} = -\frac{fB}{d^2} = -\frac{Z^2}{fB}
\quad\Longrightarrow\quad \sigma_Z \approx \frac{Z^2}{fB}\, s .$$

This is Intel's formula (BKM §3e). The error grows with the **square** of the distance.

**Number.** $s = 0.08$ px, the value of the BKM's D435 curve (Intel measures 0.07–0.11 px on good targets, up to
0.32 px on a black shiny surface). With $f = 447$ px and $B = 0.05$ m:

| $Z$ | 0.40 m | 0.50 m | 0.55 m | 0.60 m | 1.0 m |
|---|---|---|---|---|---|
| $\sigma_Z$ (Intel) | 0.57 mm | 0.89 mm | 1.08 mm | 1.29 mm | 3.6 mm |

**Algorithm.** Applied where it arises, in disparity, independently per pixel:

$$d' = d + s\,\xi, \quad \xi \sim \mathcal N(0, 1), \qquad Z' = \frac{fB}{d'} .$$

Adding the noise to the disparity (not to the depth) keeps the stereo behaviour: to first order it is Gaussian
in depth with the $\sigma_Z$ above, with the slight asymmetry of $1/d$ for larger errors.

**Ablation `axial="ahn"`** (not in the final model). Ahn et al. 2019 (Eq. 2) fitted the axial noise of a real
D435 at 848×480 as $\sigma_Z(Z) = 0.001063 + 0.0007278\,Z + 0.003949\,Z^2$ (m), applied directly in depth:
2.41 mm at 0.5 m, about 2.7× Intel's value. The paper's angle term is left out (hand-set, not consistent with its
own Fig. 9). On the development seeds this noise level made the baseline detector collapse (1/150 detected).

**Limits.** Real D435 noise is spatially correlated (the BKM's "egg carton" pattern) with no published size;
here it is independent per pixel.

## 4. Depth bias (`depth_bias`, `bias="random"`)

**Number.** The datasheet gives the absolute depth accuracy only as a limit: $|\Delta Z| \le 2\,\%$ of the
distance (Table 4-15; white wall, $\le 2$ m). It gives no model of the error's form.

**Model.** At the limit, as a scale error, with a random sign per noise draw (a unit's calibration error could
be either way):

$$Z' = (1 + b)\,Z, \qquad b = \pm 0.02 \ \text{with probability } \tfrac12 \text{ each}.$$

**Effect on 3D points.** Because back-projection is linear in $Z$, $\mathbf p' = Z'\mathbf r = (1+b)\,\mathbf p$:
every point moves **along its own viewing ray** by $b\,\lVert\mathbf p\rVert$. The scene is scaled about the
camera centre. Shapes and orientations stay the same, and the whole part shifts by ~2 % of its distance
(~10.7 mm at 0.53 m in the final sweep, with an unchanged rotation error). The shift does not depend on the
viewing angle.

`bias=<float>` fixes $b$ (e.g. `0.02`), and `bias=0` switches it off.

## 5. Datasheet output rules (`rules=True`)

**Left invalid band** (`left_invalid_band`, datasheet §4.5). The left imager is the reference: a pixel at
column $x$ with disparity $d$ has its match at $x - d$ in the right image. If that falls off the right image's
edge there is no depth:

$$x + \tfrac12 < d(x) = \frac{fB}{Z} \ \Rightarrow\ Z' = 0 .$$

At 0.5 m the band is $fB/Z \approx 45$ px wide. In our frames the left edge sees the floor 1–3 m away, so the
band is 7–19 px and never reaches a part.

**Min-Z** (`min_z`, datasheet Table 4-12): no depth closer than 195 mm at 848×480, $Z < 0.195 \Rightarrow Z' = 0$.
The BKM's formula $Z_{\min} = fB/126$ (disparity search range of 126 px) gives ~168 mm. The nearest depth here is
0.47 m, so nothing is removed.

**Quantisation** (`quantise`, BKM §8a): 16-bit depth in the default 1 mm unit,
$Z' = 1\,\text{mm}\cdot\mathrm{round}(Z / 1\,\text{mm})$. For a uniformly distributed rounding error the
RMS is $1\,\text{mm}/\sqrt{12} \approx 0.29$ mm.

These rules are kept so the frames look like real D435 output. In the experiments they had no significant effect
on perception. They do nudge the fitted pose by a fraction of a millimetre, which is enough to change the
grasp planner's choice on some seeds (the "rules" control in the final sweep).

## 6. Lateral error

Intel publishes no lateral (in-plane) noise model. In a stereo camera, lateral error arises from two sources,
both already in the model:

1. **Back-projection of the axial noise.** The pixel grid is fixed, so depth noise moves each point along its
   ray. In camera $X$ this is $\sigma_X = |X/Z|\,\sigma_Z$: zero on the optical axis, about $\sigma_Z$ at the
   edge of the field of view. This is geometry, not a separate noise source.
2. **Edge fattening**, the one mechanism that genuinely shifts depth sideways (at depth steps).

Ahn et al. measure "lateral noise" only as the scatter of edge pixels about a target's vertical edges (§IV–V)
and attribute its non-Gaussian form to shadowing. That supports treating lateral error as an edge effect.
Calling the combination "lateral noise" is our synthesis, not Ahn's statement.

## 7. Randomness and pairing

`simulate` takes a NumPy generator. The sweep seeds it with `np.random.default_rng((seed, draw, 0))`, and it is
used in a fixed order:

1. the bias sign is drawn first, always (even when the bias is off);
2. then the axial noise, one standard normal per pixel.

So **draw $r$ of seed $s$ has the same bias sign and the same noise in every condition**. Comparisons between
conditions are therefore paired draw by draw. Edge fattening and the rules use no randomness, so a condition
made only of them (e.g. "rules only", "edge only") gives the same depth for every draw.

## 8. Parameters

| Constant | Value | Source |
|---|---|---|
| `BASELINE_M` | 0.050 m | Datasheet Table 3-11 (D430 module) |
| $f$ (`FX` in `scene/config.py`) | 447 px | 848 px, HFOV 87° (datasheet) |
| `SUBPIXEL_PX` | 0.08 px | BKM §3e |
| `EDGE_GROW_PX` | 2 px | Keselman 2017 §4.2.3 (R200) |
| `EDGE_STEP_PX` | 0.5 px | ours: above a 75° slope (0.29–0.37 px/px at 0.5–0.65 m) |
| `BIAS_LIMIT` | 0.02 | Datasheet Table 4-15 |
| `MIN_Z_M` | 0.195 m | Datasheet Table 4-12 |
| `DEPTH_UNIT_M` | 0.001 m | BKM §8a |
| `AHN_ABC` (ablation) | 0.001063, 0.0007278, 0.003949 | Ahn et al. 2019 Eq. 2 |

Effects left out (temporal noise, correlated noise, incidence angle, specular and dark-surface dropout,
occlusion shadow, multi-path and others), with the reason for each: NOTES.md Q2, "Final list of effects".

```bash
python camera_model/d435.py     # prints the noise levels and what each step does to a test frame
```
