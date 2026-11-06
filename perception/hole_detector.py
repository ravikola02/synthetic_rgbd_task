"""Depth-only detector for the 6D pose of the top hole on a part (simple geometric logic).

Input : one depth image (metres, 0 = invalid) and the camera matrix K (ROS optical frame:
        +Z forward, +X right, +Y down). No RGB, no learning.
Output: hole poses in the camera frame. The pose of a round hole is its opening centre and
        its axis (z = face normal, pointing out of the face). Rotation about the axis is
        undefined for a round hole; x is fixed by convention (camera x projected on the face).

Steps
  1. back-project depth to points; keep the workspace (range < MAX_RANGE) -> drops the floor
  2. table = largest plane (RANSAC); object pixels = more than MIN_HEIGHT above it
  3. connected object clusters (one per part)
  4. per cluster, peel planar faces one by one with RANSAC
  5. hole = pixels enclosed by a face but not on it (binary hole filling), whose equivalent
     diameter on the face plane matches the expected hole size and which is round (not a slot)
  6. centre = mean of those pixels' rays intersected with the face plane; axis = face normal
  7. "top" hole = face normal closest to the table normal

Thresholds (MIN_HEIGHT, TABLE_THRESH, FACE_THRESH) are tuned on ideal depth. detect_holes(...,
thresholds=...) can replace them by per-pixel values (functions of depth); perception/hole_detector_tuned.py
uses that to scale them with the D435 noise. The defaults are this baseline detector.
"""
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scene"))
from parts import BRACKET_HOLE_DIAMETER as HOLE_DIAMETER  # noqa: E402  (item Line 8 bracket: 8.5 mm, M8)
MAX_RANGE = 1.0               # m, workspace limit (table at 0.4-0.75 m, floor beyond ~1.3 m)
# Thresholds set on ideal depth, well below the bracket's 5 mm leg thickness. With D435 noise (~1 mm)
# they sit at the noise level (AI_LOGS.md Q3); hole_detector_tuned.py scales them to the noise (mitigation).
MIN_HEIGHT = 0.0015           # m above the table to count as an object
TABLE_THRESH = 0.001          # m, RANSAC inlier distance for the table
FACE_THRESH = 0.001           # m, RANSAC inlier distance for part faces
MIN_FACE_PX = 15
MIN_HOLE_PX = 3               # 1-2 px gaps are not measurable (the 8.5 mm hole is ~7 px wide face-on at 0.5 m)
MAX_FACES = 4
DIAM_RANGE = (0.6, 1.6)       # accepted equivalent diameter, as a fraction of HOLE_DIAMETER
MAX_ELONGATION = 2.0          # ratio of in-plane principal spreads; rejects slot fragments
MAX_TOP_TILT_DEG = 50.0       # a "top" hole's axis must be within this angle of the table normal


def rays(K, shape):
    """Unnormalised viewing ray (x/z, y/z, 1) for every pixel centre."""
    H, W = shape
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    return np.dstack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones((H, W))])


def ransac_plane(P, thresh, rng, iters=300):
    """Plane n.p + d = 0 with the most inliers (thresh: scalar or one per point), refined by least squares;
    n faces the camera."""
    best_count, best = -1, None
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n /= norm
        count = np.count_nonzero(np.abs(P @ n - n @ s[0]) < thresh)
        if count > best_count:
            best_count, best = count, (n, -n @ s[0])
    n, d = best
    inl = np.abs(P @ n + d) < thresh
    m = P[inl].mean(0)
    n = np.linalg.svd(P[inl] - m)[2][2]                 # least-squares normal of the inliers
    if n @ -m < 0:                                       # orient towards the camera (at the origin)
        n = -n
    return n, -n @ m


def per_pixel(value, depth):
    """Threshold image: a constant, or a function of the depth image."""
    return np.broadcast_to(value(depth) if callable(value) else value, depth.shape)


def detect_holes(depth, K, seed=0, thresholds=None):
    """thresholds: optional {"min_height", "table", "face"} -> metres or function(depth image) -> metres,
    and hole-shape limits {"min_hole_px", "diam_range", "max_elongation"} (defaults: the module constants)."""
    th = {"min_height": MIN_HEIGHT, "table": TABLE_THRESH, "face": FACE_THRESH, "min_hole_px": MIN_HOLE_PX,
          "diam_range": DIAM_RANGE, "max_elongation": MAX_ELONGATION, **(thresholds or {})}
    th_height, th_table, th_face = (per_pixel(th[k], depth) for k in ("min_height", "table", "face"))
    rng = np.random.default_rng(seed)
    R = rays(K, depth.shape)
    P = R * depth[..., None]                             # points, camera frame
    valid = (depth > 0) & (depth < MAX_RANGE)

    # 1-2. table plane and object pixels
    idx = np.flatnonzero(valid)
    sample = rng.choice(idx, min(5000, len(idx)), replace=False)
    n_t, d_t = ransac_plane(P.reshape(-1, 3)[sample], th_table.reshape(-1)[sample], rng)
    height = P @ n_t + d_t
    objects = valid & (height > th_height)

    # 3. clusters
    labels, n_clusters = ndimage.label(objects, structure=np.ones((3, 3)))
    holes, faces = [], []
    for c in range(1, n_clusters + 1):
        remaining = labels == c
        # 4. peel faces
        for _ in range(MAX_FACES):
            if remaining.sum() < MIN_FACE_PX:
                break
            n_f, d_f = ransac_plane(P[remaining], th_face[remaining], rng, iters=200)
            on_plane = valid & (np.abs(P @ n_f + d_f) < th_face) & (labels == c)
            if on_plane.sum() < MIN_FACE_PX:
                break
            face_lab, nf = ndimage.label(on_plane)
            face = face_lab == (np.argmax(np.bincount(face_lab[on_plane])))   # largest connected patch
            remaining &= ~on_plane
            faces.append({"mask": face, "normal": n_f, "d": d_f, "cluster": c})

            # 5. enclosed gaps in the face
            gaps, ng = ndimage.label(ndimage.binary_fill_holes(face) & ~face)
            for g in range(1, ng + 1):
                gm = gaps == g
                if gm.sum() < th["min_hole_px"]:
                    continue
                r = R[gm]
                t = -d_f / (r @ n_f)                     # ray-plane intersection
                X = r * t[:, None]
                rhat = r / np.linalg.norm(r, axis=1, keepdims=True)
                # pixel footprint on the plane: z^2 cos(alpha) / (fx fy |n.rhat|)
                area = np.sum(X[:, 2] ** 2 * rhat[:, 2] / (K[0, 0] * K[1, 1] * np.abs(rhat @ n_f)))
                diam = np.sqrt(4 * area / np.pi)
                if not (th["diam_range"][0] * HOLE_DIAMETER <= diam <= th["diam_range"][1] * HOLE_DIAMETER):
                    continue
                if len(X) >= 3:                          # roundness: principal spreads within the plane
                    ev = np.sort(np.linalg.eigvalsh(np.cov((X - X.mean(0)).T)))[1:]
                    if np.sqrt(ev[1] / max(ev[0], 1e-12)) > th["max_elongation"]:
                        continue
                holes.append({"center": X.mean(0), "axis": n_f, "diameter": diam, "n_px": int(gm.sum()),
                              "cos_up": float(n_f @ n_t), "cluster": c, "mask": gm})

    # 7. top hole
    holes.sort(key=lambda h: -h["cos_up"])
    top = holes[0] if holes and holes[0]["cos_up"] > np.cos(np.radians(MAX_TOP_TILT_DEG)) else None
    return {"top": top, "all": holes, "faces": faces, "table_normal": n_t, "table_d": d_t, "labels": labels, "points": P}


def hole_pose(hole):
    """4x4 pose of the hole in the camera frame (z = axis out of the face)."""
    z = hole["axis"]
    x = np.array([1.0, 0, 0]) - z[0] * z
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], axis=1)
    T[:3, 3] = hole["center"]
    return T
