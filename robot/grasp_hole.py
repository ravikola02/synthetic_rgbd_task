"""G1 + Dex1 grasps the bracket at its top hole, approaching normal to the hole (PyBullet only).

Pipeline per scene (no ground truth used for control):
  1. build the scene with dynamic parts (settled by physics), robot in the ready pose
  2. head camera -> depth (ideal, or through the D435 model) -> top-hole detection -> bracket 6D fit
  3. grasp pose: approach direction = into the face along the detected hole axis (normal to the
     hole); jaws close across the bracket's 40 mm width (fitted CAD x-axis, made perpendicular to
     the approach). The TCP position near the hole is searched over depth along the approach
     (<= GRASP_DEPTH) and shift along the face (+-MAX_FACE_SHIFT): keep candidates whose jaws stay
     TABLE_CLEARANCE above the table and whose fingers do not hit the part along the straight
     approach (fitted CAD vs the fingers' swept volume), and pick the best antipodal one: the most fitted-CAD surface
     facing each pad (normal along +-closing axis, weaker side counts) inside the pad region. A
     plain "material between the pads" count picked the leg's thin edge and the jaws pushed the
     part away; a fixed depth missed the triangular side walls. The 35 mm-tall jaws otherwise hit the table when the hole is on a 45 deg
     leg ~9 mm above it (bracket resting on its corner, the commonest pose); --tcp-offset-mm and
     --tcp-side-mm move the TCP towards the jaw tip / the plate's lower edge to make room.
  4. trajectory: ready pose -> pre-grasp (PRE_GRASP_DIST out along the hole axis, jaws open,
     joint-space move) -> straight-line approach -> close jaws -> straight-line lift (LIFT_HEIGHT)
     IK: damped least squares on the arm's 7 joints only, in a separate kinematics-only client,
     both arms tried with restarts; nothing is executed if any waypoint misses by > MAX_IK_ERR, or if the
     arm comes closer than BODY_CLEARANCE to the robot's own body anywhere along the motion ("body collision")
  5. success = bracket lifted by > 5 cm and touching both jaws after a 1 s hold

  python robot/grasp_hole.py                  # all 15 development seeds, videos of the first 2
  python robot/grasp_hole.py --camera d435 --seeds 8 --run-ids 22   # D435 model, noise draw 22
  python robot/grasp_hole.py --camera d435 --detector tuned --seeds 8 --run-ids 22   # + noise-aware detector

--camera d435 puts the rendered depth through the D435 model (camera_model/: edge fattening, axial noise,
depth bias, datasheet rules; ablation options there). --close-view records the video close to the bracket,
--side-view from the grasping arm's side (report figures).
Noise draw r of seed s uses the
same random numbers as run r in perception/eval_*.py. The grasp scene is settled by physics (~1 mm
different from the static evaluation frames), so the same draw need not give the same detection.

Result (2026-11-04, item 40x40 bracket, ideal depth, default TCP): 15/15 development seeds lifted ~100 mm
(development set selected on that). Failures seen while selecting: 12, 42 (a gripper part not covered by
the finger model pushes the bracket ~25 mm on approach), 54, 63 not lifted, 20 slips at 51 mm.
See AI_NOTES.md.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [ROOT, os.path.join(ROOT, "perception"), os.path.join(ROOT, "scene"), os.path.join(ROOT, "scene", "backends")]
import camera_model  # noqa: E402
import hole_detector_tuned  # noqa: E402
import pybullet_backend as pb  # noqa: E402
from bracket_fit import OPENINGS, fit_bracket, model_points  # noqa: E402
from config import ROOT, TABLE_TOP_Z, DEV_SEEDS  # noqa: E402
from eval_holes import pose_matrix  # noqa: E402
from hole_detector import detect_holes  # noqa: E402
from spec import make_scene  # noqa: E402

GRASP_DEPTH = 0.012           # m, max TCP depth behind the hole opening, along the approach
PRE_GRASP_DIST = 0.10         # m
LIFT_HEIGHT = 0.10            # m
TABLE_CLEARANCE = 0.004       # m, lowest jaw corner above the table
MAX_FACE_SHIFT = 0.025        # m, TCP shift along the face (hole is 20 mm from the leg tip on the 40x40 bracket)
MAX_IK_ERR = 0.005            # m
BODY_CLEARANCE = 0.010        # m, min distance of the grasping arm (upper arm to jaws) from the rest of the
                              # robot along the whole planned motion; PyBullet runs without self-collision,
                              # so without this check IK solutions could pass the arm through the torso
IK_RESTARTS = 8               # IK restarts per arm, from perturbed arm poses, to find a collision-free one
PAD_LEN = 0.015               # m, pad length behind the TCP counted as contact region
MIN_PAD_POINTS = 15           # CAD surface samples (of 4000) facing EACH pad (antipodal grasp)
MODEL_PTS, MODEL_NRM = model_points()
# Jaw geometry in *_hand_base_link (x = along the jaws), measured on the finger collision meshes
# (dex1_col_*.stl, placed in the link's inertial frame): the URDF tip frames (*_thumb_tip /
# *_index_tip) are at x = 132 mm, the jaw tips at 143.3 mm (same as the visual mesh); the jaw plate
# is 29 mm tall (hand z = +-14.5 mm) and its inner pad face runs from x = 92 to 143 mm. (getAABB
# gives 146 mm / 35 mm because it includes the collision margin; that was used first, wrongly.)
# --tcp-offset-mm moves the TCP towards the tip, --tcp-side-mm towards the plate's lower edge.
URDF_TCP_X, JAW_TIP_X, JAW_HALF_HEIGHT = 0.132, 0.1433, 0.0145
TCP_IN_HAND = np.array([URDF_TCP_X, 0.0, 0.0])
JAW_BOX = None                                     # jaw extent around the TCP at JAW_OPEN (hand frame)


def set_tcp_offset(offset, side=0.0):
    """Move the TCP along the jaw by `offset` m (+ = towards the tip) and sideways by `side` m towards
    the jaw plate's lower edge (hand -z; grasp_pose keeps hand +z pointing up). The jaw plate is
    35 mm tall (hand z), so a side offset keeps the plate above the grasp point instead of half below.
    Jaw box (around the TCP): tip beyond the TCP, plate height shifted by `side`, 22 mm fingers.
    Keep the TCP on the plate: side < JAW_HALF_HEIGHT and offset < JAW_TIP_X - URDF_TCP_X."""
    global TCP_IN_HAND, JAW_BOX
    TCP_IN_HAND = np.array([URDF_TCP_X + offset, 0.0, -side])
    half_w = PAD_GAP_OPEN / 2 + 0.022                  # outer face of the 22 mm-thick fingers
    JAW_BOX = (np.array([-0.058 - offset, -half_w, -JAW_HALF_HEIGHT + side]),
               np.array([JAW_TIP_X - TCP_IN_HAND[0], half_w, JAW_HALF_HEIGHT + side]))

JAW_OPEN, JAW_SHUT, JAW_FORCE = 0.004, -0.02, 40.0      # joint limits: -0.02 = fully shut, +0.0245 = open


def pad_gap(q):
    """Pad gap (m) at jaw joint q: measured 29.8 mm at q = -8 mm, each jaw moves 1 mm per mm of joint."""
    return 0.0298 + 2 * (q + 0.008)


PAD_GAP_OPEN = pad_gap(JAW_OPEN)                         # 53.8 mm: room for the 40 mm bracket
set_tcp_offset(0.0)
SQUEEZE = 0.004               # m, jaws are commanded to (grasped width - SQUEEZE), not fully shut; the
                              # interference makes the position-controlled jaws stall on the part and press
JAW_SPEED = 0.01                                        # m/s per jaw while closing
PAD_FRICTION = 2.0            # jaw pads (rubber-like); PyBullet multiplies it with the part's friction
DT = 1.0 / 240
VIDEO_EVERY = 8                                    # sim steps per video frame (30 fps)


def arm_joints(robot, side):
    names = [f"{side}_{n}_joint" for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                            "wrist_roll", "wrist_pitch", "wrist_yaw")]
    idx = {p.getJointInfo(robot, j)[1].decode(): j for j in range(p.getNumJoints(robot))}
    return [idx[n] for n in names], [idx[f"{side}_dex1_finger_joint_1"], idx[f"{side}_dex1_finger_joint_2"]]


def body_link_sets(robot, side, client=0):
    """Links of the grasping arm (upper arm to jaws) and the rest of the robot that it must not touch
    (pelvis, legs, torso, head, the other arm); the arm's shoulder pitch/roll links sit on the torso."""
    names = {j: p.getJointInfo(robot, j, physicsClientId=client)[12].decode()
             for j in range(p.getNumJoints(robot, physicsClientId=client))}
    has_col = [j for j in names if p.getCollisionShapeData(robot, j, physicsClientId=client)]
    arm = [j for j in has_col if names[j].startswith(side + "_") and
           any(t in names[j] for t in ("shoulder_yaw", "elbow", "wrist", "dex1"))]
    body = [-1] + [j for j in has_col if j not in arm and not names[j].startswith(side + "_shoulder")]
    return arm, body


class Kinematics:
    """IK on a kinematics-only copy of the robot (separate PyBullet client)."""

    def __init__(self, urdf, side):
        self.c = p.connect(p.DIRECT)
        self.r = p.loadURDF(urdf, useFixedBase=True, physicsClientId=self.c)
        self.movable = [j for j in range(p.getNumJoints(self.r, physicsClientId=self.c))
                        if p.getJointInfo(self.r, j, physicsClientId=self.c)[2] != p.JOINT_FIXED]
        names = {p.getJointInfo(self.r, j, physicsClientId=self.c)[12].decode(): j
                 for j in range(p.getNumJoints(self.r, physicsClientId=self.c))}
        self.hand = names[f"{'L' if side == 'left' else 'R'}_hand_base_link"]
        jn = {p.getJointInfo(self.r, j, physicsClientId=self.c)[1].decode(): j for j in self.movable}
        self.arm = [jn[f"{side}_{n}_joint"] for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                                       "wrist_roll", "wrist_pitch", "wrist_yaw")]
        self.cols = [self.movable.index(j) for j in self.arm]
        self.lim = np.array([p.getJointInfo(self.r, j, physicsClientId=self.c)[8:10] for j in self.arm])
        self.arm_links, self.body_links = body_link_sets(self.r, side, self.c)

    def clearance(self, q_all, limit=0.1):
        """Smallest distance (m, capped at limit) between the arm links and the rest of the robot."""
        self.set(q_all)
        d = [c[8] for a in self.arm_links for b in self.body_links
             for c in p.getClosestPoints(self.r, self.r, limit, a, b, physicsClientId=self.c)]
        return min(d, default=limit)

    def set(self, q_all):
        for j, q in q_all.items():
            p.resetJointState(self.r, j, q, physicsClientId=self.c)

    def tcp_pose(self):
        pos, quat = p.getLinkState(self.r, self.hand, computeForwardKinematics=True, physicsClientId=self.c)[4:6]
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        return np.asarray(pos) + R @ TCP_IN_HAND, R

    def solve(self, q_start, pos, R_goal, iters=300, damping=0.02):
        """Arm joints reaching TCP pose (pos, R_goal); returns (q_arm, pos_err_m, rot_err_deg)."""
        q_all = dict(q_start)
        self.set(q_all)
        for _ in range(iters):
            cur, R = self.tcp_pose()
            e_pos = pos - cur
            dR = R_goal @ R.T
            angle = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
            axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
            e_rot = axis / (2 * np.sin(angle)) * angle if angle > 1e-6 else np.zeros(3)
            if np.linalg.norm(e_pos) < 1e-4 and angle < 1e-3:
                break
            qs = [q_all[j] for j in self.movable]
            Jl, Ja = p.calculateJacobian(self.r, self.hand, TCP_IN_HAND.tolist(), qs, [0.0] * len(qs),
                                         [0.0] * len(qs), physicsClientId=self.c)
            J = np.vstack([np.asarray(Jl)[:, self.cols], np.asarray(Ja)[:, self.cols]])
            e = np.concatenate([e_pos, 0.5 * e_rot])
            dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
            for k, j in enumerate(self.arm):
                q_all[j] = float(np.clip(q_all[j] + np.clip(dq[k], -0.2, 0.2), *self.lim[k]))
            self.set(q_all)
        cur, R = self.tcp_pose()
        rot = np.degrees(np.arccos(np.clip((np.trace(R_goal @ R.T) - 1) / 2, -1, 1)))
        return [q_all[j] for j in self.arm], float(np.linalg.norm(pos - cur)), float(rot)


def grasp_pose(T_wc, det, fit):
    """TCP position and orientation (world) for the grasp; approach = hand x, closing = hand y."""
    hole = fit["hole"]                                               # the hole the best fit started from
    c = T_wc[:3, :3] @ hole["center"] + T_wc[:3, 3]
    approach = -(T_wc[:3, :3] @ hole["axis"])                       # into the face, normal to the hole
    x_obj = (T_wc @ fit["T_cam_obj"])[:3, 0]                        # bracket width direction (40 mm)
    close = x_obj - (x_obj @ approach) * approach
    close /= np.linalg.norm(close)
    if np.cross(approach, close)[2] < 0:                            # keep hand +z (jaw plate height) upward;
        close = -close                                               # the jaws are symmetric, so this is free
    R = np.stack([approach, close, np.cross(approach, close)], axis=1)
    corners = np.array([[x, y, z] for x in (JAW_BOX[0][0], JAW_BOX[1][0])
                        for y in (JAW_BOX[0][1], JAW_BOX[1][1]) for z in (JAW_BOX[0][2], JAW_BOX[1][2])])
    up = np.array([0, 0, 1.0]) - approach[2] * approach              # uphill direction within the face
    up = up / np.linalg.norm(up) if np.linalg.norm(up) > 1e-6 else np.zeros(3)

    def clear(t):
        return (corners @ R.T + t)[:, 2].min() >= TABLE_TOP_Z + TABLE_CLEARANCE

    # Material between the pads, from the fitted CAD model (perception output, not ground truth):
    # posed bracket surface samples inside the closing volume (hand frame, relative to the TCP):
    # along the jaw from PAD_LEN behind the TCP to the tip, across the open gap, within the plate height.
    T_wo = T_wc @ fit["T_cam_obj"]
    P = MODEL_PTS @ T_wo[:3, :3].T + T_wo[:3, 3]
    n_close = (MODEL_NRM @ T_wo[:3, :3].T) @ close                  # surface normal along the closing axis
    lo = np.array([-PAD_LEN, -PAD_GAP_OPEN / 2, JAW_BOX[0][2]])
    hi = np.array([JAW_BOX[1][0], PAD_GAP_OPEN / 2, JAW_BOX[1][2]])

    # fingers' volume swept along the straight approach (hand frame, relative to the final TCP):
    # each finger from the open pad face outwards by its 22 mm thickness, plate height, and along
    # the jaw from 58 mm behind the TCP to the tip, extended back by the approach distance
    f_lo = np.array([JAW_BOX[0][0] - PRE_GRASP_DIST, PAD_GAP_OPEN / 2, JAW_BOX[0][2]])
    f_hi = np.array([JAW_BOX[1][0], PAD_GAP_OPEN / 2 + 0.022, JAW_BOX[1][2]])

    def finger_hits(local):
        y = np.abs(local[:, 1])
        return np.count_nonzero((local[:, 0] > f_lo[0]) & (local[:, 0] < f_hi[0]) & (y > f_lo[1]) & (y < f_hi[1])
                                & (local[:, 2] > f_lo[2]) & (local[:, 2] < f_hi[2]))

    best = None
    for depth in np.arange(GRASP_DEPTH, -0.004, -0.001):                # TCP behind the opening
        for shift in np.arange(-MAX_FACE_SHIFT, MAX_FACE_SHIFT + 1e-9, 0.001):
            tcp = c + depth * approach + shift * up
            if not clear(tcp):
                continue
            local = (P - tcp) @ R                                     # into the hand frame
            if finger_hits(local):                                    # a finger would hit the part on the way in
                continue
            inside = np.all((local > lo) & (local < hi), axis=1)
            # antipodal: surface facing each pad (normals along +-closing axis), take the weaker side
            pos_face, neg_face = inside & (n_close > 0.9), inside & (n_close < -0.9)
            n = int(min(np.count_nonzero(pos_face), np.count_nonzero(neg_face)))
            if best is None or n > best[0]:
                width = (np.median(local[pos_face, 1]) - np.median(local[neg_face, 1])) if n else None
                best = (n, tcp, depth, shift, abs(width) if width is not None else None)
    if best is None or best[0] < MIN_PAD_POINTS:
        return None, R, approach, None, None, None
    return best[1], R, approach, best[2], best[3], best[4]


def closest_in_cad(robot, body, link):
    """Closest point of a jaw on the part, in the part's CAD frame (mm), and its distance (mm)."""
    pts = p.getClosestPoints(robot, body, 0.1, linkIndexA=link)
    if not pts:
        return None
    c = min(pts, key=lambda c: c[8])
    pos, q = p.getBasePositionAndOrientation(body)
    R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
    return np.round(np.r_[(np.array(c[6]) - pos) @ R, c[8]] * 1e3, 1).tolist()


def jaw_target(width):
    """Jaw joint target that closes the pads to the grasped width minus SQUEEZE (not fully shut)."""
    return float(np.clip(JAW_OPEN + (width - SQUEEZE - PAD_GAP_OPEN) / 2, JAW_SHUT, JAW_OPEN))  # inverse of pad_gap


def plan_arm(urdf, side, robot, tcp, R, approach, jaw_close=None):
    """IK for all waypoints with one arm; returns (plan, max pos err m, max rot err deg, min body clearance m).

    IK restarts from perturbed arm poses until a plan reaches every waypoint and keeps the arm BODY_CLEARANCE
    away from the robot's body along the whole motion (the move to the pre-grasp is checked at joint-space
    steps, as it is executed); otherwise the plan with the best pre-grasp IK is returned with its clearance."""
    kin = Kinematics(urdf, side)
    q_now = {j: p.getJointState(robot, j)[0] for j in kin.movable}
    jc = JAW_SHUT if jaw_close is None else jaw_close
    fallback = None
    for restart in range(IK_RESTARTS):
        q0 = dict(q_now)
        if restart:
            rng = np.random.default_rng(restart)
            for k, j in enumerate(kin.arm):
                q0[j] = float(np.clip(q0[j] + rng.normal(0, 0.5), *kin.lim[k]))
        q_pre, e_pre, r_pre = kin.solve(q0, tcp - PRE_GRASP_DIST * approach, R)
        if e_pre >= 1e-3 and fallback is not None:
            continue
        plan, errs, configs = [("to pre-grasp", q_pre, JAW_OPEN, 360, tcp - PRE_GRASP_DIST * approach)], \
            [(e_pre, r_pre)], []
        q_start = np.array([q_now[j] for j in kin.arm])
        for t in np.linspace(0, 1, 21)[1:]:                             # executed as a joint-space ramp
            configs.append(q_start + t * (np.array(q_pre) - q_start))
        q_prev = {**q_now, **dict(zip(kin.arm, q_pre))}
        for k in range(1, 11):                                           # straight-line approach
            qk, e, r = kin.solve(q_prev, tcp - PRE_GRASP_DIST * approach * (1 - k / 10), R)
            q_prev.update(zip(kin.arm, qk)); errs.append((e, r)); configs.append(np.array(qk))
            plan.append(("approach", qk, JAW_OPEN, 24, tcp - PRE_GRASP_DIST * approach * (1 - k / 10)))
        plan.append(("close", qk, jc, 240, tcp))
        plan.append(("squeeze", qk, jc, 120, tcp))                # let the grip force build before lifting
        for k in range(1, 11):                                           # straight-line lift
            qk, e, r = kin.solve(q_prev, tcp + np.array([0, 0, LIFT_HEIGHT * k / 10]), R)
            q_prev.update(zip(kin.arm, qk)); errs.append((e, r)); configs.append(np.array(qk))
            plan.append(("lift", qk, jc, 24, tcp + np.array([0, 0, LIFT_HEIGHT * k / 10])))
        plan.append(("hold", qk, jc, 240, tcp + np.array([0, 0, LIFT_HEIGHT])))
        e_max, r_max = max(e for e, _ in errs), max(r for _, r in errs)
        clear = min(kin.clearance({**q_now, **dict(zip(kin.arm, q))}) for q in configs)
        result = (plan, e_max, r_max, clear)
        if e_max <= MAX_IK_ERR and clear >= BODY_CLEARANCE:
            p.disconnect(kin.c)
            return result
        if fallback is None or (e_pre < 1e-3 <= fallback[0]):
            fallback = (e_pre, result)
    p.disconnect(kin.c)
    return fallback[1]


def run(seed, video_path=None, args=None, run_idx=0):
    spec = make_scene(seed, settle=True)                            # dynamic parts for grasping
    sim = pb.build(spec)
    robot, bracket = sim["robot"], sim["bodies"]["bracket"]
    cam = pb.render_cameras(sim, spec)["cameras"]["head"]
    K = np.array(spec["camera"]["K"])
    camera = camera_model.tag(args) if args is not None else "ideal"
    detector = "tuned detector" if args is not None and args.detector == "tuned" else "baseline detector"
    depth = (camera_model.depth(args, cam["depth"], K, np.random.default_rng((seed, run_idx, 0)))
             if args is not None else cam["depth"])
    detect = hole_detector_tuned.from_args(args)[0] if args is not None else detect_holes
    det = detect(depth, K, seed=seed)
    fit = fit_bracket(depth, K, det, seed=seed)
    row = {"seed": seed, "camera": camera, "run": run_idx, "hole_detected": det["top"] is not None,
           "fitted": fit is not None}
    if fit is None:
        p.disconnect()
        return row

    T_wc = pose_matrix(cam["cam_pose_ros"]["position"], cam["cam_pose_ros"]["quat_wxyz"])
    tcp, R, approach, depth, shift, width = grasp_pose(T_wc, det, fit)

    # ground truth only for reporting: true axis of the opening nearest to the detection
    pos, q = p.getBasePositionAndOrientation(bracket)
    T_wo = pose_matrix(pos, [q[3], q[0], q[1], q[2]])
    c_det = T_wc[:3, :3] @ fit["hole"]["center"] + T_wc[:3, 3]
    gt = min(OPENINGS.values(), key=lambda o: np.linalg.norm(T_wo[:3, :3] @ o[0] + T_wo[:3, 3] - c_det))
    gt_axis = T_wo[:3, :3] @ np.array(gt[1], float)
    row["hole_pos_err_mm"] = float(np.linalg.norm(T_wo[:3, :3] @ np.array(gt[0]) + T_wo[:3, 3] - c_det) * 1e3)
    row.update(approach_err_deg=float(np.degrees(np.arccos(np.clip(-approach @ gt_axis, -1, 1)))),
               approach_tilt_from_vertical_deg=float(np.degrees(np.arccos(np.clip(-approach[2], -1, 1)))))
    if tcp is None:
        row.update(status="no table-clear grasp")
        p.disconnect()
        return row
    jaw_close = jaw_target(width)
    row.update(grasp_depth_mm=depth * 1e3, face_shift_mm=shift * 1e3, grasp_width_mm=width * 1e3,
               jaw_close_mm=jaw_close * 1e3)

    plans = {sd: plan_arm(spec["robot"]["urdf"], sd, robot, tcp, R, approach, jaw_close) for sd in ("left", "right")}
    ok = [sd for sd in plans if plans[sd][1] <= MAX_IK_ERR and plans[sd][3] >= BODY_CLEARANCE]
    reach = [sd for sd in plans if plans[sd][1] <= MAX_IK_ERR]
    side = min(ok or reach or plans, key=lambda sd: plans[sd][1])
    plan, e_max, r_max, clear = plans[side]
    arm, fingers = arm_joints(robot, side)
    row.update(side=side, ik_pos_err_max_mm=e_max * 1e3, ik_rot_err_max_deg=r_max,
               body_clearance_plan_mm=clear * 1e3)
    if not ok:
        row.update(status="body collision" if reach else "IK failed")
        p.disconnect()
        return row
    arm_links, body_links = body_link_sets(robot, side)
    exec_clear = 0.1

    # execute with physics
    for f in fingers:
        p.changeDynamics(robot, f, lateralFriction=PAD_FRICTION, spinningFriction=0.01)
    writer = None
    if video_path:
        writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 480))
        eye, target = np.array([0.95, -0.55 if side == "left" else 0.55, 0.30]), np.array([0.33, 0.0, -0.05])
        if args is not None and getattr(args, "close_view", False):        # same direction, aimed at the bracket
            target = np.array(p.getBasePositionAndOrientation(bracket)[0]) + [0, 0, 0.05]
            eye = target + 0.4 * (eye - [0.33, 0.0, -0.05]) / np.linalg.norm(eye - [0.33, 0.0, -0.05])
        if args is not None and getattr(args, "side_view", False):   # from the grasping arm's side: the body
            target = np.array(p.getBasePositionAndOrientation(bracket)[0]) + [0.02, 0, 0.09]   # is beside the hand,
            d = np.array([0.15, 1.0 if side == "left" else -1.0, 0.35])                    # not behind it
            eye = target + 0.62 * d / np.linalg.norm(d)
        view = p.computeViewMatrix(eye.tolist(), target.tolist(), [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(45, 640 / 480, 0.05, 5.0)
    z0 = p.getBasePositionAndOrientation(bracket)[0][2]
    q_cmd = np.array([p.getJointState(robot, j)[0] for j in arm])
    step = 0
    hand = sim["link_index"][f"{'L' if side == 'left' else 'R'}_hand_base_link"]
    table = sim["table"]
    b_start = np.array(p.getBasePositionAndOrientation(bracket)[0])
    phase_log = []
    for n_ph, (phase, q_goal, jaw, n_steps, tcp_goal) in enumerate(plan):
        q_from = q_cmd.copy()
        for i in range(n_steps):
            q_cmd = q_from + (np.array(q_goal) - q_from) * min(1.0, (i + 1) / (0.8 * n_steps))
            p.setJointMotorControlArray(robot, arm, p.POSITION_CONTROL, targetPositions=q_cmd.tolist(),
                                        forces=[200.0] * 7, positionGains=[0.3] * 7)
            for f in fingers:                                        # equal, slow jaw speed: neither jaw
                p.setJointMotorControl2(robot, f, p.POSITION_CONTROL, targetPosition=jaw,   # shoves the part
                                        force=JAW_FORCE, maxVelocity=JAW_SPEED)              # before the other

            p.stepSimulation()
            if step % 24 == 0:                                       # check: the executed motion stays clear too
                exec_clear = min([exec_clear] + [c[8] for a in arm_links for b in body_links
                                                 for c in p.getClosestPoints(robot, robot, exec_clear, a, b)])
            if writer is not None and step % VIDEO_EVERY == 0:
                img = np.reshape(p.getCameraImage(640, 480, view, proj, renderer=p.ER_TINY_RENDERER)[2], (480, 640, 4))
                bgr = cv2.cvtColor(img[..., :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
                cv2.putText(bgr, f"seed {seed}  {camera} depth, draw {run_idx}, {detector}", (12, 440),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
                cv2.putText(bgr, phase, (12, 466), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
                writer.write(bgr)
            step += 1
        if n_ph + 1 == len(plan) or plan[n_ph + 1][0] != phase:     # end of a phase: diagnostics
            Th = pb.link_pose(robot, hand)
            tcp_now = Th[:3, :3] @ TCP_IN_HAND + Th[:3, 3]
            phase_log.append({
                "phase": phase, "tcp_err_mm": round(float(np.linalg.norm(tcp_now - tcp_goal)) * 1e3, 1),
                "jaw_mm": [round(p.getJointState(robot, f)[0] * 1e3, 1) for f in fingers],
                "jaw_bracket_contacts": [len(p.getContactPoints(robot, bracket, linkIndexA=f)) for f in fingers],
                "jaw_bracket_force_N": [round(sum(c[9] for c in p.getContactPoints(robot, bracket, linkIndexA=f)), 2)
                                        for f in fingers],
                "jaw_table_contacts": sum(len(p.getContactPoints(robot, table, linkIndexA=f)) for f in fingers),
                "jaw_bracket_dist_mm": [round(min([c[8] for c in p.getClosestPoints(robot, bracket, 0.1, linkIndexA=f)]
                                                  or [0.1]) * 1e3, 1) for f in fingers],
                "jaw_closest_on_bracket_cad_mm": [closest_in_cad(robot, bracket, f) for f in fingers],
                "bracket_in_tcp_frame_mm": np.round((np.array(p.getBasePositionAndOrientation(bracket)[0]) - tcp_now)
                                                    @ Th[:3, :3] * 1e3, 1).tolist(),
                "bracket_moved_mm": round(float(np.linalg.norm(np.array(p.getBasePositionAndOrientation(bracket)[0])
                                                               - b_start)) * 1e3, 1)})
    row["phase_log"] = phase_log
    if writer is not None:
        writer.release()

    lift = p.getBasePositionAndOrientation(bracket)[0][2] - z0
    touching = [len(p.getContactPoints(robot, bracket, linkIndexA=f)) > 0 for f in fingers]
    row.update(body_clearance_exec_mm=exec_clear * 1e3)
    row.update(lift_mm=lift * 1e3, touches_both_jaws=all(touching), success=bool(lift > 0.05 and all(touching)),
               status="lifted" if lift > 0.05 and all(touching) else "executed, not lifted")
    p.disconnect()
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=DEV_SEEDS)
    ap.add_argument("--videos", type=int, default=2, help="record videos of the first N seeds")
    ap.add_argument("--close-view", action="store_true", help="video camera close to the bracket")
    ap.add_argument("--side-view", action="store_true", help="video camera beside the bracket, on the grasping arm's side "
                    "(the hand is not drawn over the robot's body; used for the report figures)")
    camera_model.add_args(ap)
    hole_detector_tuned.add_args(ap)
    ap.add_argument("--run-ids", type=int, nargs="*", default=[0], help="noise draws to run per seed")
    ap.add_argument("--tcp-offset-mm", type=float, nargs="*", default=[0.0],
                    help="TCP shift along the jaw from the URDF tip frames (+ = towards the tip); several = sweep")
    ap.add_argument("--tcp-side-mm", type=float, default=0.0,
                    help="TCP shift towards the jaw plate's lower edge, perpendicular to the approach")
    ap.add_argument("--pad-friction", type=float, default=PAD_FRICTION)
    ap.add_argument("--squeeze-mm", type=float, default=SQUEEZE * 1e3,
                    help="command the jaws this far inside the grasped width (they stall on the part)")
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "grasp"))
    args = ap.parse_args()
    globals()["PAD_FRICTION"] = args.pad_friction
    globals()["SQUEEZE"] = args.squeeze_mm / 1e3
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for off in args.tcp_offset_mm:
        set_tcp_offset(off / 1000, args.tcp_side_mm / 1000)
        for i, seed in enumerate(args.seeds):
            for r in args.run_ids:
                cam_tag = ("" if args.camera == "ideal" else f"_{camera_model.tag(args)}_run{r}") + \
                          hole_detector_tuned.from_args(args)[1]
                video = (os.path.join(args.out, f"grasp_seed_{seed:03d}{cam_tag}_tcp{off:+.0f}mm_side{args.tcp_side_mm:.0f}mm.mp4")
                         if i < args.videos else None)
                row = {"tcp_offset_mm": off, "tcp_side_mm": args.tcp_side_mm, **run(seed, video, args, r)}
                rows.append(row)
                print(json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()
                                  if k != "phase_log"}), flush=True)
    det_tag = hole_detector_tuned.from_args(args)[1]
    name = "results.json" if args.camera == "ideal" and not det_tag else f"results_{camera_model.tag(args)}{det_tag}.json"
    with open(os.path.join(args.out, name), "w") as f:
        json.dump(rows, f, indent=1)
    print(f"\n=== grasp at top hole, {camera_model.tag(args)} depth{det_tag}, PyBullet ===")
    for off in args.tcp_offset_mm:
        rr = [r for r in rows if r["tcp_offset_mm"] == off]
        print(f"TCP offset {off:+5.1f} mm: lifted {sum(r.get('success', False) for r in rr)}/{len(rr)}, "
              f"grasp depth mm {[round(r['grasp_depth_mm']) for r in rr if 'grasp_depth_mm' in r]}")


if __name__ == "__main__":
    main()
