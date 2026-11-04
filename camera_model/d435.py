"""Analytical RealSense D435 depth model, independent of the simulator.

    simulate(depth, K, rng, axial="intel", rules=True)  ->  depth as the D435 would report it (m, 0 = invalid)

Input: ideal z-depth of a frame (scene/frame_io.py; 848x480, K = pinhole intrinsics). Only effects that
the D435 documentation gives a model for are included (scope, references and the effects left out:
NOTES.md Q2). Steps, in this order:

1. AXIAL NOISE (random)
   axial="intel"  main model, Intel tuning guide (BKM §3e)
       The D435 measures disparity d (px) between its two IR imagers: z = f*B/d. The matcher finds d
       to a fraction of a pixel; this subpixel error s (px RMS) propagates to depth as
           sigma_z = z^2 * s / (f * B)
       BKM: s < 0.1 px on a well-textured target (towards 0.05); > ~0.2 px means recalibrate. Applied
       where the error arises: depth -> disparity -> + N(0, s) per pixel -> depth.
       Here (f = 447 px, B = 50 mm, s = 0.08): 0.89 mm at 0.50 m, 1.08 mm at 0.55 m.
   axial="ahn"    ablation, Ahn et al. 2019 (docs/793350771-ANM.pdf, Eq. 2), fitted on a real D435
       at 848x480 (default preset, laser 150 mW, white planar target, 0.4-2.1 m):
           sigma_z = 0.001063 + 0.0007278 z + 0.003949 z^2           (m; theta = 0)
       The paper's angle term 0.022 z^1.5 theta/(pi/2 - theta)^2 is left out: its coefficients were set
       by hand and do not reproduce the paper's own Fig. 9. Applied in depth (the domain it was fitted in).
       Here: 2.41 mm at 0.50 m (~2.7x Intel).
   axial=None     no noise (datasheet rules only)
   Both: Gaussian, independent per pixel. The real noise is spatially correlated ("egg carton", BKM
   §3e) with no published correlation length; not modelled.

2. DATASHEET RULES (deterministic), rules=True
   a. left invalid depth band  (datasheet §4.5, p. 82-83): depth is matched with the LEFT imager as
      reference; a left pixel at column x with disparity d = f*B/z has its match at x - d in the right
      image, so x < d -> no depth. (The datasheet's band width B*F/Z px is this for constant depth Z.)
   b. Min-Z  (datasheet Table 4-12, p. 84): no depth closer than 195 mm at 848x480.
   c. quantisation  (BKM §8a): 16-bit depth in 1 mm units -> round to 1 mm.

Sources: docs/RealSense-D400-Series-Datasheet-Mar-2026.pdf (datasheet), docs/BKM-Tuning-D400-Depth-Cameras.pdf
(BKM, counted as part of the datasheet), docs/793350771-ANM.pdf (Ahn et al. 2019).

    python camera_model/d435.py     # sigma_z of both axial models and what each step does to a frame
"""
import numpy as np

BASELINE_M = 0.050      # datasheet Table 3-11 (D430 module)
SUBPIXEL_PX = 0.08      # BKM §3e: typical subpixel RMS (value used for its D435 curve)
AHN_ABC = (0.001063, 0.0007278, 0.003949)   # Ahn et al. 2019 Eq. 2 (m, 1, 1/m)
MIN_Z_M = 0.195         # datasheet Table 4-12, 848x480
DEPTH_UNIT_M = 0.001    # BKM §8a, default depth unit


# ------------------------------------------------------------------ 1. axial noise
def sigma_intel(z, f_px, subpixel=SUBPIXEL_PX, baseline=BASELINE_M):
    """Axial RMS depth error (m) at depth z (m), Intel BKM."""
    return z ** 2 * subpixel / (f_px * baseline)


def sigma_ahn(z):
    """Axial RMS depth error (m) at depth z (m), Ahn et al. 2019, surface facing the camera."""
    a, b, c = AHN_ABC
    return a + b * z + c * z ** 2


def axial_intel(depth, K, rng, subpixel=SUBPIXEL_PX, baseline=BASELINE_M):
    """Gaussian subpixel error added to the disparity."""
    f_px = K[0, 0]
    valid = depth > 0
    disparity = np.zeros_like(depth)
    disparity[valid] = f_px * baseline / depth[valid]                       # px
    disparity += subpixel * rng.standard_normal(depth.shape)
    noisy = np.zeros_like(depth)
    noisy[valid] = f_px * baseline / disparity[valid]
    return noisy.astype(depth.dtype)


def axial_ahn(depth, rng):
    """Gaussian error with Ahn's sigma_z(z) added to the depth."""
    noisy = depth + sigma_ahn(depth) * rng.standard_normal(depth.shape)
    return np.where(depth > 0, noisy, 0.0).astype(depth.dtype)


# ------------------------------------------------------------------ 2. datasheet rules
def left_invalid_band(depth, K, baseline=BASELINE_M):
    """Pixels whose match would fall left of the right image's edge -> 0."""
    x = np.arange(depth.shape[1]) + 0.5                        # pixel-centre column, left edge = 0
    disparity = np.where(depth > 0, K[0, 0] * baseline / np.maximum(depth, 1e-9), 0.0)
    return np.where(x[None, :] < disparity, 0.0, depth).astype(depth.dtype)


def min_z(depth, min_z_m=MIN_Z_M):
    """Closer than Min-Z -> 0."""
    return np.where(depth < min_z_m, 0.0, depth).astype(depth.dtype)


def quantise(depth, unit=DEPTH_UNIT_M):
    """Round to the output depth unit."""
    return (np.round(depth / unit) * unit).astype(depth.dtype)


# ------------------------------------------------------------------ model
def simulate(depth, K, rng, axial="intel", rules=True):
    """Ideal z-depth (m, 0 = invalid) -> D435 depth. axial: "intel" (main), "ahn" (ablation) or None."""
    K = np.asarray(K)
    if axial == "intel":
        depth = axial_intel(depth, K, rng)
    elif axial == "ahn":
        depth = axial_ahn(depth, rng)
    elif axial is not None:
        raise ValueError(f"unknown axial model {axial!r}")
    if rules:
        depth = quantise(min_z(left_invalid_band(depth, K)))
    return depth


if __name__ == "__main__":
    import os
    import sys
    ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    sys.path[:0] = [os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
    from frame_io import ideal_frame
    f = ideal_frame("pybullet", 4)                                    # test seed 4
    d, K, ins = f["depth"], f["K"], f["instance"]

    print("axial noise sigma_z (mm):\n  z (m)   intel   ahn")
    for z in (0.4, 0.5, 0.55, 0.6, 1.0, 2.0):
        print(f"  {z:4.2f}   {sigma_intel(z, K[0, 0]) * 1e3:5.2f}  {sigma_ahn(z) * 1e3:5.2f}")
    m = ins == 4                                                     # bracket pixels
    for name, s in (("intel", sigma_intel(d[m], K[0, 0])), ("ahn", sigma_ahn(d[m]))):
        diff = simulate(d, K, np.random.default_rng(0), axial=name, rules=False)[m] - d[m]
        print(f"seed 4 bracket ({m.sum()} px): {name} added std {diff.std() * 1e3:.2f} mm, "
              f"model {np.sqrt(np.mean(s ** 2)) * 1e3:.2f} mm")

    band = (left_invalid_band(d, K) == 0) & (d > 0)
    print(f"left invalid band: {band.sum()} px, {band.sum(1).min()}-{band.sum(1).max()} px wide per row "
          f"(datasheet B*F/Z at 0.5 m: {K[0, 0] * BASELINE_M / 0.5:.1f} px); part pixels in it: {(band & (ins >= 3)).sum()}")
    print(f"Min-Z: pixels closer than {MIN_Z_M * 1e3:.0f} mm: {((d > 0) & (d < MIN_Z_M)).sum()} "
          f"(nearest {d[d > 0].min():.3f} m)")
    q = quantise(d)[d > 0] - d[d > 0]
    print(f"quantisation to {DEPTH_UNIT_M * 1e3:.0f} mm: error RMS {q.std() * 1e3:.2f} mm, max {np.abs(q).max() * 1e3:.2f} mm")
