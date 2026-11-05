"""Analytical RealSense D435 depth model, independent of the simulator.

    simulate(depth, K, rng)  ->  depth as the D435 would report it (m, 0 = invalid)

Input: ideal z-depth of a frame (scene/frame_io.py; 848x480, K = pinhole intrinsics). Three effects, each
with a published number, plus the datasheet output rules. Applied in the order they arise in the camera:

1. EDGE FATTENING (deterministic), edge="r200"
   A stereo matcher compares windows of pixels; at a depth step the window straddles both sides and the
   foreground (larger disparity) wins, so the foreground grows into the background and narrow gaps close.
   No D435 value is published. The only number is for the older R200 (Keselman et al. 2017,
   docs/1705.05548v2.pdf, §4.2.3): it "incorrectly merg[es] gaps between edges of less than 5 pixels"
   (7x7 Census + 7x7 box aggregation, §3.2). Intel's D400 post-processing paper (docs/Intel-RealSense-
   Depth-PostProcess.pdf) implies an x-y resolution of the same order (sub-sampling < 3 px: "fairly
   minimal impact"). Model: at depth steps (adjacent pixels differ by > EDGE_STEP_PX disparity) the
   foreground grows EDGE_GROW_PX = 2 px into the background, so gaps <= 4 px close; the filled pixels take
   the mean rim disparity in the window (our rule). To be calibrated against real D435 depth.

2. AXIAL NOISE (random), axial="intel"
   The D435 measures disparity d (px) between its two IR imagers: z = f*B/d. The matcher finds d to a
   fraction of a pixel; this subpixel error s (px RMS) propagates to depth as (BKM §3e)
       sigma_z = z^2 * s / (f * B)
   s = 0.08 px (BKM's D435 curve; Intel measures 0.07-0.11 px on good targets, up to 0.32 px on a black
   shiny surface). Applied where it arises: disparity + N(0, s) per pixel. Here (f = 447 px, B = 50 mm):
   0.89 mm at 0.50 m, 1.08 mm at 0.55 m.
   axial="ahn": ablation, Ahn et al. 2019 Eq. 2 (docs/793350771-ANM.pdf), fitted on a real D435 at
   848x480: sigma_z = 0.001063 + 0.0007278 z + 0.003949 z^2 (m, angle term left out: hand-set and not
   consistent with the paper's Fig. 9). Applied in depth. 2.41 mm at 0.50 m (~2.7x Intel).
   Both Gaussian and independent per pixel. The real noise is spatially correlated ("egg carton",
   BKM §3e) with no published size; not modelled.

3. DEPTH BIAS (random sign), bias="random"
   Datasheet Table 4-15 (p. 85) gives Z-accuracy (absolute error) only as a limit: +-2 % of distance
   (white wall, <= 2 m). Modelled at the limit as a scale error along the viewing ray, depth * (1 + b),
   with b = +2 % or -2 % at equal probability per noise draw (the sign of a unit's calibration error is
   unknown). 10 mm at 0.5 m. bias=<float> fixes b (e.g. 0.02); bias=0 switches it off.
   The sign is always drawn first, so draw r has the same sign and the same noise in every variant.

4. DATASHEET RULES (deterministic), rules=True
   a. left invalid depth band (datasheet §4.5, p. 82-83): depth is matched with the LEFT imager as
      reference; a pixel at column x with disparity d = f*B/z has its match at x - d in the right image,
      so x < d -> no depth.
   b. Min-Z (datasheet Table 4-12, p. 84): no depth closer than 195 mm at 848x480.
   c. quantisation (BKM §8a): 16-bit depth in 1 mm units -> round to 1 mm.
   Tested: no significant effect on the downstream task here; kept so the frames are D435-like.

Effects left out and why: NOTES.md Q2 ("Final list of effects").

    python camera_model/d435.py     # noise levels and what each step does to a test frame
"""
import numpy as np

BASELINE_M = 0.050      # datasheet Table 3-11 (D430 module)
SUBPIXEL_PX = 0.08      # BKM §3e: typical subpixel RMS (value used for its D435 curve)
AHN_ABC = (0.001063, 0.0007278, 0.003949)   # Ahn et al. 2019 Eq. 2 (m, 1, 1/m)
EDGE_GROW_PX = 2        # Keselman 2017 §4.2.3 (R200): gaps < 5 px merge -> 2 px growth from each side
EDGE_STEP_PX = 0.5      # disparity step that counts as an edge; a plane seen at 75 deg changes by
                        # B*tan(75)/z ~ 0.2-0.3 px per pixel here, so slanted faces are not edges
BIAS_LIMIT = 0.02       # datasheet Table 4-15: Z-accuracy +-2 %
MIN_Z_M = 0.195         # datasheet Table 4-12, 848x480
DEPTH_UNIT_M = 0.001    # BKM §8a, default depth unit


def to_disparity(depth, K, baseline=BASELINE_M):
    return np.where(depth > 0, K[0, 0] * baseline / np.maximum(depth, 1e-9), 0.0)


def to_depth(disp, K, baseline=BASELINE_M):
    return np.where(disp > 0, K[0, 0] * baseline / np.maximum(disp, 1e-9), 0.0)


# ------------------------------------------------------------------ 1. edge fattening
def edge_fattening(depth, K, grow=EDGE_GROW_PX, step=EDGE_STEP_PX):
    """Foreground grows `grow` px into the background at depth steps."""
    from scipy.ndimage import minimum_filter, uniform_filter
    disp = to_disparity(depth, K)
    rim = disp - minimum_filter(disp, size=3) > step                   # foreground rim: closer than a neighbour
    n = 2 * grow + 1
    count = uniform_filter(rim.astype(float), size=n)
    fg = np.where(count > 0, uniform_filter(np.where(rim, disp, 0.0), size=n) / np.maximum(count, 1e-12), 0.0)
    out = np.where(fg - disp > step, fg, disp)                         # background pixels take the rim's disparity
    return to_depth(out, K).astype(depth.dtype)


# ------------------------------------------------------------------ 2. axial noise
def sigma_intel(z, f_px, subpixel=SUBPIXEL_PX, baseline=BASELINE_M):
    """Axial RMS depth error (m) at depth z (m), Intel BKM."""
    return z ** 2 * subpixel / (f_px * baseline)


def sigma_ahn(z):
    """Axial RMS depth error (m) at depth z (m), Ahn et al. 2019, surface facing the camera."""
    a, b, c = AHN_ABC
    return a + b * z + c * z ** 2


def axial_intel(depth, K, rng, subpixel=SUBPIXEL_PX):
    """Gaussian subpixel error added to the disparity."""
    disp = to_disparity(depth, K) + subpixel * rng.standard_normal(depth.shape)
    return np.where(depth > 0, to_depth(disp, K), 0.0).astype(depth.dtype)


def axial_ahn(depth, rng):
    """Gaussian error with Ahn's sigma_z(z) added to the depth."""
    noisy = depth + sigma_ahn(depth) * rng.standard_normal(depth.shape)
    return np.where(depth > 0, noisy, 0.0).astype(depth.dtype)


# ------------------------------------------------------------------ 3. depth bias
def depth_bias(depth, b):
    """Scale error along the viewing ray: depth * (1 + b); 0 (invalid) stays 0."""
    return (depth * (1.0 + b)).astype(depth.dtype)


# ------------------------------------------------------------------ 4. datasheet rules
def left_invalid_band(depth, K):
    """Pixels whose match would fall left of the right image's edge -> 0."""
    x = np.arange(depth.shape[1]) + 0.5                                # pixel-centre column, left edge = 0
    return np.where(x[None, :] < to_disparity(depth, K), 0.0, depth).astype(depth.dtype)


def min_z(depth, min_z_m=MIN_Z_M):
    """Closer than Min-Z -> 0."""
    return np.where(depth < min_z_m, 0.0, depth).astype(depth.dtype)


def quantise(depth, unit=DEPTH_UNIT_M):
    """Round to the output depth unit."""
    return (np.round(depth / unit) * unit).astype(depth.dtype)


# ------------------------------------------------------------------ model
def simulate(depth, K, rng, edge="r200", axial="intel", bias="random", rules=True):
    """Ideal z-depth (m, 0 = invalid) -> D435 depth.

    edge:  "r200" (default) or None
    axial: "intel" (default), "ahn" (ablation) or None
    bias:  "random" (default: +-BIAS_LIMIT, 50/50 per draw), a fixed fraction (e.g. 0.02) or 0
    rules: datasheet output rules (left invalid band, Min-Z, 1 mm quantisation)
    """
    K = np.asarray(K)
    sign = rng.choice([-1.0, 1.0])               # always drawn first: draw r has the same sign and noise in every variant
    if edge == "r200":
        depth = edge_fattening(depth, K)
    elif edge is not None:
        raise ValueError(f"unknown edge model {edge!r}")
    if axial == "intel":
        depth = axial_intel(depth, K, rng)
    elif axial == "ahn":
        depth = axial_ahn(depth, rng)
    elif axial is not None:
        raise ValueError(f"unknown axial model {axial!r}")
    if bias == "random":
        bias = BIAS_LIMIT * sign
    if bias:
        depth = depth_bias(depth, bias)
    if rules:
        depth = quantise(min_z(left_invalid_band(depth, K)))
    return depth


if __name__ == "__main__":
    import os
    import sys
    ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    sys.path[:0] = [os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
    from frame_io import ideal_frame
    f = ideal_frame("pybullet", 4)                                     # test seed 4
    d, K, ins = f["depth"], np.asarray(f["K"]), f["instance"]
    m = ins == 4                                                       # bracket pixels

    e = edge_fattening(d, K)
    changed = np.abs(e - d) > 1e-6
    print(f"1. edge fattening: {changed.sum()} px changed ({(changed & m).sum()} on the bracket)")

    print("2. axial noise sigma_z (mm):\n  z (m)   intel   ahn")
    for z in (0.4, 0.5, 0.55, 0.6, 1.0, 2.0):
        print(f"  {z:4.2f}   {sigma_intel(z, K[0, 0]) * 1e3:5.2f}  {sigma_ahn(z) * 1e3:5.2f}")
    for name, s in (("intel", sigma_intel(d[m], K[0, 0])), ("ahn", sigma_ahn(d[m]))):
        diff = simulate(d, K, np.random.default_rng(0), edge=None, axial=name, bias=0, rules=False)[m] - d[m]
        print(f"  seed 4 bracket ({m.sum()} px): {name} added std {diff.std() * 1e3:.2f} mm, "
              f"model {np.sqrt(np.mean(s ** 2)) * 1e3:.2f} mm")

    signs = [np.sign(np.median(simulate(d, K, np.random.default_rng((4, r, 0)), edge=None, axial=None,
                                        rules=False)[m] / d[m] - 1)) for r in range(10)]
    print(f"3. depth bias: +-{BIAS_LIMIT:.0%}, signs of draws 0-9: {' '.join('+' if s > 0 else '-' for s in signs)}"
          f" ({BIAS_LIMIT * np.median(d[m]) * 1e3:.1f} mm on the bracket)")

    band = (left_invalid_band(d, K) == 0) & (d > 0)
    print(f"4. left invalid band: {band.sum(1).min()}-{band.sum(1).max()} px wide per row, "
          f"part pixels in it: {(band & (ins >= 3)).sum()}; Min-Z: {((d > 0) & (d < MIN_Z_M)).sum()} px closer than "
          f"{MIN_Z_M * 1e3:.0f} mm; quantisation RMS {(quantise(d)[d > 0] - d[d > 0]).std() * 1e3:.2f} mm")
