"""Scene constants shared by every backend and script (numpy only)."""
import math
import os

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
IDEAL_DIR = os.path.join(ROOT, "data", "ideal")      # ideal frames: data/ideal/<backend>/seed_XXX/<camera>/

# ---------------------------------------------------------------- robot
G1_USD = "/mnt/shared_data/unitree_sim_assets/assets/robots/g1-29dof-dex1-base-fix-usd/g1_29dof_with_dex1_base_fix1.usd"
G1_URDF = "/mnt/shared_data/unitree_sim_assets/urdf/g1_dex1.urdf"
G1_PRIM = "/World/G1"

# The G1 base is fixed to the world at the origin (pelvis frame), so we keep the robot
# there and place floor/table relative to it instead of moving the robot.
PELVIS_HEIGHT = 0.793                       # pelvis above the soles, from the asset bounds
FLOOR_Z = -PELVIS_HEIGHT

# Arm pose while the camera looks at the table, and the start of every grasp trajectory
# ("ready" pose): arms raised out to the sides, grippers pointing forward ~24 cm above the table
# and just beside it (|y| = 0.44 m). Chosen from a grid search over shoulder pitch/roll/yaw and
# elbow: no robot pixels in the head camera, no self-collision, grippers >= 5 cm above the table,
# closest to a nominal ready point (0.20, +-0.25, table + 0.15 m). Replaced "arms hanging down"
# (elbow 90 deg), which kept the camera view clear but put the grippers below table height.
OBSERVE_ARM_POSE = {"left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
                    "left_shoulder_roll_joint": 0.8, "right_shoulder_roll_joint": -0.8,
                    "left_shoulder_yaw_joint": 0.5, "right_shoulder_yaw_joint": -0.5,
                    "left_elbow_joint": 0.0, "right_elbow_joint": 0.0}

# ---------------------------------------------------------------- table and parts
# An IK reachability map (both arms, 9 approach directions: vertical + 43 deg tilt at 8 yaws, TCP at
# part height) gave the best reach at table height 0.75 m and x = 0.20-0.30 m (44-67 % of directions,
# across y = +-0.25 m); x >= 0.35 m dropped to 11-33 %; tables at 0.80 / 0.85 m were not better. So
# the bracket (the part to grasp) is placed in x 0.24-0.30 m and the 200 mm profile further out
# (x 0.37-0.41 m), so it stays in the camera view and on the table. The camera sees the table from
# x ~ 0.19 m.
TABLE_HEIGHT = 0.75                         # table top above the floor
TABLE_TOP_Z = FLOOR_Z + TABLE_HEIGHT
TABLE_SIZE = (0.40, 0.60, 0.04)             # x (depth from robot), y (width), thickness
TABLE_X = 0.34                              # centre; table spans x 0.14-0.54, y +-0.30
PLACE_REGIONS = {                           # part -> (centre x, centre y), (half x, half y)
    "profile": ((0.39, 0.0), (0.02, 0.12)),     # 40x40x200: ends + half-width stay on the table
    "bracket": ((0.27, 0.0), (0.03, 0.12)),     # 40x40 bracket: footprint radius ~0.03 m, stays in view
}
MIN_PART_DIST = 0.16    # profile half-length (0.10) + half-width (0.02) + bracket footprint radius (~0.03) + margin

# ---------------------------------------------------------------- camera
# Pose of each camera's ROS optical frame (+Z forward, +X right, +Y down) in its parent link frame.
# Only the G1's torso-mounted D435 (d435_link: +X forward / +Z up). Wrist cameras were tried and
# dropped (see NOTES.md); the dict keeps that easy to revisit.
CAMERAS = {
    "head": {"link": "d435_link", "pos": (0.0, 0.0, 0.0), "quat_wxyz": (0.5, -0.5, 0.5, -0.5)},
}

# D435 depth stream (datasheet: depth FOV 87 x 58 deg; recommended 848x480)
RES = (848, 480)
HFOV_DEG = 87.0
FX = RES[0] / (2 * math.tan(math.radians(HFOV_DEG) / 2))   # focal length in pixels (447)


def sample_part_poses(rng):
    """(x, y, yaw) for the profile and the bracket in their placement regions, non-overlapping."""
    def sample(name):
        (cx, cy), (hx, hy) = PLACE_REGIONS[name]
        return np.array([cx, cy]) + rng.uniform([-hx, -hy], [hx, hy])
    while True:                     # resample both until the parts are far enough apart
        p, b = sample("profile"), sample("bracket")
        if np.linalg.norm(b - p) > MIN_PART_DIST:
            break
    return (p[0], p[1], rng.uniform(0, 2 * np.pi)), (b[0], b[1], rng.uniform(0, 2 * np.pi))


# ---------------------------------------------------------------- fixed test set
# Used by all evaluations. Selected 2026-11-03 for the item Line 8 40x40 parts: the first 15 seeds,
# in order, that pass all of
#   1. the bracket's top hole is visible to the head camera (ideal PyBullet depth, CAD ground truth)
#   2. the hole used by the bracket fit (perception/bracket_fit.py) is that hole (within 3 mm)
#   3. the grasp poses (pre-grasp, approach, lift) are reachable by one arm (IK error < 5 mm)
# Because perception failures are excluded by construction, ideal-depth perception results on this
# set are 15/15 by definition; report them as such, not as accuracy.
# Seeds 0-52 were checked; 38 rejected: 30 no visible top hole, 8 grasp unreachable.
TEST_SEEDS = [4, 8, 12, 17, 20, 21, 23, 26, 27, 32, 37, 38, 42, 45, 52]
