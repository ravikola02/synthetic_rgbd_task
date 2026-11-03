"""RealSense D435 depth camera model, independent of any simulator.

Input : an ideal frame from either backend (scene/frame_io.ideal_frame / load_frame), i.e. the
        reference set data/ideal/<backend>/seed_XXX/head/ for config.TEST_SEEDS:
        depth (m), instance ids, K, materials from the scene spec.
Output: depth as the D435 would report it (m, 0 = invalid).

Which effects to model is still open (see NOTES.md Q2); `simulate` is the single entry
point so the experiment code does not change when effects are added or removed.

Sources: docs/RealSense-D400-Series-Datasheet-Mar-2026.pdf (DS),
         docs/BKM-Tuning-D400-Depth-Cameras.pdf (BKM).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class D435Params:
    baseline_m: float = 0.050          # DS Table 3-11 (D430 module)
    hfov_deg: float = 87.0             # DS Table 3-11, depth FOV at 1280x720 / 848x480
    min_z_m: float = 0.195             # DS Table 4-12, at 848x480
    max_z_m: float = 3.0               # practical working range (not a hard DS spec)
    subpixel_rms: float = 0.08         # BKM: typical 0.05-0.1 px on a well-textured target


def simulate(depth, instance, K, materials=None, params=D435Params(), rng=None):
    """Return the D435-like depth for one ideal frame. No effects implemented yet."""
    raise NotImplementedError("camera effects not chosen yet - see NOTES.md Q2")
