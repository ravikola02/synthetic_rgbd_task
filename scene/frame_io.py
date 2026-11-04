"""Common frame format written by every backend and read by the camera model.

Ideal frames of the test seeds live in data/ideal/<backend>/seed_XXX/<camera>/ (config.IDEAL_DIR):
the reference that the camera model's depth is compared with.

<frame_dir>/<camera>/          (camera = head)
  rgb.png         uint8 HxWx3
  depth.npy       float32 HxW, ideal z-depth in metres (along the optical axis), 0 = no hit
  instance.npy    uint8 HxW, common instance ids (see spec.py)
  depth_vis.png   for viewing only
  meta.json       scene spec + camera K + camera pose (ROS optical frame: +Z forward,
                  +X right, +Y down) + part poses as rendered
"""
import json
import os

import numpy as np
from PIL import Image


VIS_RANGE = (0.40, 0.80)   # m: the workspace (table and parts at 0.45-0.75 m); farther = grey


def depth_to_vis(depth, K, rng=VIS_RANGE):
    """Viewing aid only: turbo colour over the workspace depth range, modulated by shading from
    the surface normals so 40 mm parts and their edges stand out from the table. Black = no
    depth, grey = beyond the workspace (floor)."""
    import matplotlib
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    K = np.asarray(K)
    P = np.dstack([(u - K[0, 2]) / K[0, 0] * depth, (v - K[1, 2]) / K[1, 1] * depth, depth])
    n = np.cross(np.gradient(P, axis=1), np.gradient(P, axis=0))
    n /= np.linalg.norm(n, axis=2, keepdims=True) + 1e-12
    light = np.array([-0.4, -0.5, -0.77])                        # from upper left, towards the camera
    shade = np.clip(np.abs(n @ (light / np.linalg.norm(light))), 0, 1)
    t = np.clip((depth - rng[0]) / (rng[1] - rng[0]), 0, 1)
    rgb = matplotlib.colormaps["turbo"](t)[..., :3] * (0.35 + 0.65 * shade[..., None])
    rgb[depth > rng[1]] = 0.5 * (0.6 + 0.4 * shade[depth > rng[1], None])
    rgb[depth <= 0] = 0
    return (255 * np.clip(rgb, 0, 1)).astype(np.uint8)


def save_frame(out_dir, rgb, depth, instance, spec, cam_pose_ros, part_poses, backend):
    """Write one camera's frame; meta.json records which camera via the folder name."""
    os.makedirs(out_dir, exist_ok=True)
    depth = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    Image.fromarray(rgb.astype(np.uint8)).save(os.path.join(out_dir, "rgb.png"))
    np.save(os.path.join(out_dir, "depth.npy"), depth)
    np.save(os.path.join(out_dir, "instance.npy"), instance.astype(np.uint8))

    Image.fromarray(depth_to_vis(depth, spec["camera"]["K"])).save(os.path.join(out_dir, "depth_vis.png"))

    meta = {"backend": backend, "camera_name": os.path.basename(os.path.normpath(out_dir)), "camera_pose_world_ros": cam_pose_ros, "part_poses": part_poses, "spec": spec}
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)


def load_frame(frame_dir):
    with open(os.path.join(frame_dir, "meta.json")) as f:
        meta = json.load(f)
    return {
        "rgb": np.array(Image.open(os.path.join(frame_dir, "rgb.png"))),
        "depth": np.load(os.path.join(frame_dir, "depth.npy")),
        "instance": np.load(os.path.join(frame_dir, "instance.npy")),
        "K": np.array(meta["spec"]["camera"]["K"]),
        "meta": meta,
    }


def ideal_frame_dir(backend, seed, camera="head"):
    from config import IDEAL_DIR
    return os.path.join(IDEAL_DIR, backend, f"seed_{seed:03d}", camera)


def ideal_frame(backend, seed, camera="head"):
    """Load an ideal frame of the dataset. PyBullet frames that are missing are rendered and saved;
    Isaac frames must be rendered beforehand with scene/render.py (returns None if missing)."""
    d = ideal_frame_dir(backend, seed, camera)
    if not os.path.exists(os.path.join(d, "meta.json")):
        if backend != "pybullet":
            return None
        from pybullet_backend import render
        from spec import make_scene
        spec = make_scene(seed)
        out = render(spec)
        for cam_name, f in out["cameras"].items():
            save_frame(ideal_frame_dir(backend, seed, cam_name), f["rgb"], f["depth"], f["instance"], spec,
                       f["cam_pose_ros"], out["part_poses"], backend)
    return load_frame(d)

