"""Backend-independent scene description.

make_scene(seed) returns plain data (JSON-serialisable). Every backend builds exactly
this scene, so a seed gives the same layout in PyBullet and Isaac Sim.

Part poses are full 6D poses of the CAD frame in the world: a stable resting pose
(sampled by its probability), a random yaw, and a position near the table centre.

Instance ids are common to all backends:
    0 = background / floor, 1 = table, 2 = robot, 3.. = parts in the order listed.
"""
import os

import numpy as np
import trimesh

from config import (CAMERAS, FLOOR_Z, FX, G1_PRIM, G1_URDF, G1_USD, OBSERVE_ARM_POSE, RES, TABLE_SIZE,
                    TABLE_TOP_Z, TABLE_X, sample_part_poses)
from materials import MATERIALS
from parts import PARTS, ROOT, load_part, stable_poses

MESH_DIR = os.path.join(ROOT, "assets", "meshes")

INSTANCE_BACKGROUND, INSTANCE_TABLE, INSTANCE_ROBOT, INSTANCE_FIRST_PART = 0, 1, 2, 3


def _mesh_file(name):
    """OBJ of the CAD solid (CAD frame), written once; both backends load this file."""
    path = os.path.join(MESH_DIR, f"{name}.obj")
    if not os.path.exists(path):
        os.makedirs(MESH_DIR, exist_ok=True)
        load_part(name).export(path)
    return path


def _resting_pose(name, x, y, yaw, rng):
    """4x4 world pose of the part's CAD frame: random stable pose, yaw, centred at (x, y) on the table."""
    T_rest, prob = stable_poses(name)
    T = T_rest[rng.choice(len(prob), p=prob)]
    rested = load_part(name).copy().apply_transform(T)
    lo, hi = rested.bounds
    centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    T = trimesh.transformations.translation_matrix(-centre) @ T                     # footprint centre at origin, resting on z=0
    T = trimesh.transformations.rotation_matrix(yaw, [0, 0, 1]) @ T
    return trimesh.transformations.translation_matrix([x, y, TABLE_TOP_Z]) @ T


def make_scene(seed=0, profile_material=None, settle=False):
    rng = np.random.default_rng(seed)
    xy_yaw = dict(zip(("profile", "bracket"), sample_part_poses(rng)))

    parts = []
    for i, (name, (x, y, yaw)) in enumerate(xy_yaw.items()):
        material = profile_material if (name == "profile" and profile_material) else PARTS[name]["material"]
        T = _resting_pose(name, x, y, yaw, rng)
        parts.append({
            "name": name,
            "item_no": PARTS[name]["item_no"],
            "instance_id": INSTANCE_FIRST_PART + i,
            "mesh": _mesh_file(name),
            "position": T[:3, 3].tolist(),
            "quat_wxyz": trimesh.transformations.quaternion_from_matrix(T).tolist(),
            "material": material,
            "mass": float(load_part(name).volume * MATERIALS[material]["physical"]["density"]),
        })

    return {
        "seed": seed,
        "settle": settle,                     # False: parts placed at rest and kept static (identical across backends)
        "floor_z": FLOOR_Z,
        "robot": {"urdf": G1_URDF, "usd": G1_USD, "prim": G1_PRIM, "base_position": [0.0, 0.0, 0.0],
                  "joint_positions": OBSERVE_ARM_POSE},          # all other joints at 0
        "cameras": {n: {"link": c["link"], "pos": list(c["pos"]), "quat_wxyz": list(c["quat_wxyz"])}
                    for n, c in CAMERAS.items()},       # pose of the ROS optical frame in the parent link
        "camera": {"model": "RealSense D435 depth stream (all cameras)", "resolution": list(RES),
                   "K": [[FX, 0.0, RES[0] / 2], [0.0, FX, RES[1] / 2], [0.0, 0.0, 1.0]],
                   "near": 0.01, "far": 10.0},
        "table": {"size": list(TABLE_SIZE), "position": [TABLE_X, 0.0, TABLE_TOP_Z - TABLE_SIZE[2] / 2],
                  "material": "table_grey"},
        "parts": parts,
        "materials": MATERIALS,
    }
