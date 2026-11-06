"""Noise-aware top-hole detector: the baseline detector (hole_detector.py) with its thresholds scaled to the
D435's expected axial noise. A mitigation for noisy depth, not part of the baseline comparison.

The baseline thresholds were set on ideal depth (1 mm plane inliers, 1.5 mm object height). With D435
noise (sigma_z ~ 0.9 mm at 0.5 m, Intel model) they sit at the noise level: faces break up and noise
leaves enclosed gaps that pass as holes. Here each threshold becomes K_SIGMA times the expected noise at
the pixel's depth, never below the baseline value:
    plane inliers (table, faces):  max(1 mm,   K_SIGMA * sigma_z(z))
    object height above table:     max(1.5 mm, K_SIGMA * sigma_z(z))
sigma_z comes from the camera model (camera_model/d435.py, Intel BKM: z^2 s / (f B)), i.e. the detector
knows the camera's noise level, as one would after measuring it on the real camera.
Upper limit: the bracket's legs are 5 mm thick, so the object height must stay well below that
(K_SIGMA = 2.5 at 0.5-0.7 m gives 2.2-4.4 mm).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "..")]
from camera_model.d435 import sigma_ahn, sigma_intel  # noqa: E402
from hole_detector import (DIAM_RANGE, FACE_THRESH, MAX_ELONGATION, MIN_HEIGHT, MIN_HOLE_PX,  # noqa: E402
                           TABLE_THRESH, detect_holes)

K_SIGMA = 2.5           # thresholds in units of the expected noise (2.5 sigma keeps ~99 % of a Gaussian)


def thresholds(K, k_sigma=K_SIGMA, axial="intel", limits=None):
    """Per-pixel thresholds (functions of the depth image) for detect_holes; limits: optional hole-shape
    limits {"min_hole_px", "diam_range", "max_elongation"} (default: the baseline values)."""
    sigma = (lambda z: sigma_intel(z, K[0, 0])) if axial == "intel" else sigma_ahn
    return {"table": lambda d: np.maximum(TABLE_THRESH, k_sigma * sigma(d)),
            "face": lambda d: np.maximum(FACE_THRESH, k_sigma * sigma(d)),
            "min_height": lambda d: np.maximum(MIN_HEIGHT, k_sigma * sigma(d)), **(limits or {})}


def detect_holes_tuned(depth, K, seed=0, k_sigma=K_SIGMA, axial="intel", limits=None):
    return detect_holes(depth, K, seed=seed, thresholds=thresholds(np.asarray(K), k_sigma, axial, limits))


# ------------------------------------------------------------------ command-line options (eval scripts)
def add_args(ap):
    ap.add_argument("--detector", choices=["baseline", "tuned"], default="baseline",
                    help="baseline: thresholds tuned on ideal depth; tuned: thresholds scaled to the D435 noise")
    ap.add_argument("--k-sigma", type=float, default=K_SIGMA, help="tuned detector: thresholds in units of sigma_z")
    ap.add_argument("--tune-axial", choices=["intel", "ahn"], default="intel",
                    help="tuned detector: noise model its thresholds assume")
    ap.add_argument("--min-hole-px", type=int, default=MIN_HOLE_PX, help="tuned detector: smallest hole (pixels)")
    ap.add_argument("--diam-range", type=float, nargs=2, default=list(DIAM_RANGE),
                    help="tuned detector: accepted hole diameter, fractions of the true diameter")
    ap.add_argument("--max-elongation", type=float, default=MAX_ELONGATION, help="tuned detector: roundness limit")


def from_args(args):
    """(detect function(depth, K, seed), name suffix for outputs)."""
    if args.detector == "baseline":
        return detect_holes, ""
    limits = {"min_hole_px": args.min_hole_px, "diam_range": tuple(args.diam_range),
              "max_elongation": args.max_elongation}
    suffix = f"_tuned{args.k_sigma:g}" + ("" if args.tune_axial == "intel" else f"{args.tune_axial}")
    if limits != {"min_hole_px": MIN_HOLE_PX, "diam_range": tuple(DIAM_RANGE), "max_elongation": MAX_ELONGATION}:
        suffix += f"_px{args.min_hole_px}_d{args.diam_range[0]:g}-{args.diam_range[1]:g}_e{args.max_elongation:g}"
    return (lambda d, K, seed=0: detect_holes_tuned(d, K, seed, args.k_sigma, args.tune_axial, limits)), suffix
