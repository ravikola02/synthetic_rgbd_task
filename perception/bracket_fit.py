"""Fit the bracket's CAD model into the scene, starting from a detected hole.

The hole pose fixes 5 of the bracket's 6 degrees of freedom (opening centre + axis). Left:
which of the 4 openings was seen (hole A/B x outer/inner face) and the rotation about the
hole axis. All 4 x 36 hypotheses (10 deg steps) are scored against the depth image, then the
best one is refined with ICP on the bracket's depth points.

Hypothesis score (camera frame, depth only):
  model points visible from the camera (z-buffer over the posed CAD surface samples),
  compared with the measured depth at their pixel:
    + inlier     |d_obs - z_model| < INLIER_TOL
    - violation  d_obs > z_model + FREE_TOL  (the camera sees through where the model is)
  score = inliers - 2 * violations

Symmetry: the bracket maps onto itself under a 180 deg turn about (0, 1, 1)/sqrt(2)
(swaps the legs; holds to 0.4 mm on the CAD mesh).
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scene"))
from parts import BRACKET_OPENINGS, BRACKET_SYMMETRIES, load_part  # noqa: E402

OPENINGS, SYMMETRIES = BRACKET_OPENINGS, BRACKET_SYMMETRIES   # see scene/parts.py
N_MODEL_POINTS = 4000
ANGLE_STEP_DEG = 10
INLIER_TOL = 0.003            # m
FREE_TOL = 0.005              # m
ICP_ITERS = 30
ICP_MAX_DIST = 0.004          # m, correspondence cut-off


def model_points(seed=0):
    """Surface samples (points, normals) of the bracket in its CAD frame."""
    import trimesh
    mesh = load_part("bracket")
    pts, face = trimesh.sample.sample_surface(mesh, N_MODEL_POINTS, seed=seed)
    return np.asarray(pts), mesh.face_normals[face]


def frame_from_axis(z, x_hint):
    z = z / np.linalg.norm(z)
    x = x_hint - (x_hint @ z) * z
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def visible_model(T, pts, nrm, K, shape):
    """Posed model points the camera sees: facing it and in front in a z-buffer."""
    X = pts @ T[:3, :3].T + T[:3, 3]
    N = nrm @ T[:3, :3].T
    keep = (np.einsum("ij,ij->i", N, -X) > 0) & (X[:, 2] > 0.05)
    X = X[keep]
    u = np.floor(K[0, 0] * X[:, 0] / X[:, 2] + K[0, 2]).astype(int)
    v = np.floor(K[1, 1] * X[:, 1] / X[:, 2] + K[1, 2]).astype(int)
    inside = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    X, u, v = X[inside], u[inside], v[inside]
    zbuf = np.full(shape, np.inf)
    np.minimum.at(zbuf, (v, u), X[:, 2])
    front = X[:, 2] <= zbuf[v, u] + 0.001
    return X[front], u[front], v[front]


def score_pose(T, pts, nrm, depth, K):
    X, u, v = visible_model(T, pts, nrm, K, depth.shape)
    d = depth[v, u]
    ok = d > 0
    r = d[ok] - X[ok, 2]
    inl, viol = np.count_nonzero(np.abs(r) < INLIER_TOL), np.count_nonzero(r > FREE_TOL)
    return inl - 2 * viol, inl, viol


def umeyama(A, B):
    """Rigid transform (R, t) minimising |R A + t - B|."""
    ma, mb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mb - R @ ma


def icp(T, pts, nrm, obs, K, shape):
    """Refine T so the visible model points match the observed bracket points."""
    tree_obs = cKDTree(obs)
    for it in range(ICP_ITERS):
        X, _, _ = visible_model(T, pts, nrm, K, shape)
        if len(X) < 20:
            break
        dist, idx = tree_obs.query(X)
        m = dist < ICP_MAX_DIST
        if m.sum() < 20:
            break
        R, t = umeyama(X[m], obs[idx[m]])
        dT = np.eye(4)
        dT[:3, :3], dT[:3, 3] = R, t
        T = dT @ T
        if np.linalg.norm(t) < 1e-6 and np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)) < 1e-5:
            break
    X, _, _ = visible_model(T, pts, nrm, K, shape)
    dist = tree_obs.query(X)[0]
    return T, float(np.sqrt(np.mean(np.minimum(dist, ICP_MAX_DIST) ** 2)))


def _fit_from_hole(depth, K, det, hole, pts, nrm):
    """Best of the 4 openings x 36 rotations for one hole candidate, refined with ICP."""
    obs = det["points"][det["labels"] == hole["cluster"]]           # depth points of the part with the hole
    best = None
    for name, (c, n) in OPENINGS.items():
        T_obj_open = np.eye(4)
        T_obj_open[:3, :3] = frame_from_axis(np.array(n, float), np.array([1.0, 0, 0]) if abs(n[0]) < 0.9
                                              else np.array([0, 1.0, 0]))
        T_obj_open[:3, 3] = c
        for phi in np.radians(np.arange(0, 360, ANGLE_STEP_DEG)):
            Rz = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0], [0, 0, 1]])
            T_cam_open = np.eye(4)
            T_cam_open[:3, :3] = frame_from_axis(hole["axis"], np.array([1.0, 0, 0])) @ Rz
            T_cam_open[:3, 3] = hole["center"]
            T = T_cam_open @ np.linalg.inv(T_obj_open)
            sc = score_pose(T, pts, nrm, depth, K)
            if best is None or sc[0] > best[0][0]:
                best = (sc, T, name)
    T, rmse = icp(best[1], pts, nrm, obs, K, depth.shape)
    score, inl, viol = score_pose(T, pts, nrm, depth, K)
    return {"T_cam_obj": T, "opening": best[2], "score": score, "inliers": inl, "violations": viol,
            "icp_rmse_mm": rmse * 1e3, "init_T": best[1], "hole": hole}


def fit_bracket(depth, K, det, seed=0):
    """6D pose of the bracket (4x4, CAD frame -> camera frame) from the detected holes.

    Every upward hole candidate (axis within the detector's top-tilt limit) is tried and the fit
    that explains the depth best (highest score) wins; its hole is returned as fit["hole"]. Taking
    simply the most upward candidate failed when a round gap on the bracket/cap passed as a hole.
    """
    from hole_detector import MAX_TOP_TILT_DEG
    cands = [h for h in det["all"] if h["cos_up"] > np.cos(np.radians(MAX_TOP_TILT_DEG))]
    if not cands:
        return None
    K = np.asarray(K)
    pts, nrm = model_points(seed)
    return max((_fit_from_hole(depth, K, det, h, pts, nrm) for h in cands), key=lambda f: f["score"])


def pose_errors(T_est, T_gt, pts):
    """Translation (mm), symmetry-aware rotation (deg) and ADD-S (mm)."""
    t_err = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3]) * 1e3
    r_err = min(np.degrees(np.arccos(np.clip((np.trace((T_gt[:3, :3] @ S).T @ T_est[:3, :3]) - 1) / 2, -1, 1)))
                for S in SYMMETRIES)
    Pe = pts @ T_est[:3, :3].T + T_est[:3, 3]
    Pg = pts @ T_gt[:3, :3].T + T_gt[:3, 3]
    adds = cKDTree(Pg).query(Pe)[0].mean() * 1e3
    return t_err, r_err, adds
