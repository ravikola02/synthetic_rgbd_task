"""PyBullet backend: CPU only (TinyRenderer). Builds a scene spec, returns one ideal frame.

render(spec) = build(spec) + render_cameras(sim, spec) + disconnect. Callers that keep simulating
afterwards (e.g. the grasp run) use build / render_cameras directly.

Limitation: no physically based shading, reflections or environment light. Materials
only set colour and a specular strength, so RGB is plausible but not physically
correct. Depth, instance masks and poses are exact.
"""

import numpy as np
import pybullet as p
import pybullet_data

import trimesh

from spec import INSTANCE_BACKGROUND, INSTANCE_ROBOT, INSTANCE_TABLE


def _xyzw(q_wxyz):
    return [q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]


def build(spec, gui=False):
    """Create the scene in a new PyBullet session; returns ids needed later."""
    mats = spec["materials"]
    p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # ---- static world
    p.loadURDF("plane.urdf", basePosition=[0, 0, spec["floor_z"]])
    robot = p.loadURDF(spec["robot"]["urdf"], basePosition=spec["robot"]["base_position"], useFixedBase=True)
    joints = [j for j in range(p.getNumJoints(robot)) if p.getJointInfo(robot, j)[2] != p.JOINT_FIXED]
    targets = [spec["robot"]["joint_positions"].get(p.getJointInfo(robot, j)[1].decode(), 0.0) for j in joints]
    for j, q in zip(joints, targets):
        p.resetJointState(robot, j, q)
    p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, targetPositions=targets,
                                forces=[200.0] * len(joints))

    t = spec["table"]
    half = np.array(t["size"]) / 2
    tm = mats[t["material"]]
    table = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
                              p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=tm["visual"]["rgba"]),
                              basePosition=t["position"])
    p.changeDynamics(table, -1, lateralFriction=tm["physical"]["friction"], restitution=tm["physical"]["restitution"])

    # ---- parts
    bodies = {}
    for part in spec["parts"]:
        m = mats[part["material"]]
        spec_strength = 0.2 + 0.8 * m["visual"]["metallic"] * (1 - m["visual"]["roughness"])
        body = p.createMultiBody(
            baseMass=part["mass"] if spec["settle"] else 0.0,        # static unless settling
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_MESH, fileName=part["mesh"]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_MESH, fileName=part["mesh"],
                                                     rgbaColor=m["visual"]["rgba"],
                                                     specularColor=[spec_strength] * 3),
            basePosition=part["position"], baseOrientation=_xyzw(part["quat_wxyz"]))
        p.changeDynamics(body, -1, lateralFriction=m["physical"]["friction"], restitution=m["physical"]["restitution"])
        bodies[part["name"]] = body

    for _ in range(480 if spec["settle"] else 1):
        p.stepSimulation()
    link_index = {p.getJointInfo(robot, j)[12].decode(): j for j in range(p.getNumJoints(robot))}
    return {"robot": robot, "table": table, "bodies": bodies, "link_index": link_index}


def link_pose(robot, link):
    """4x4 world pose of a robot link."""
    pos, quat = p.getLinkState(robot, link, computeForwardKinematics=True)[4:6]
    T = np.eye(4)
    T[:3, :3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
    T[:3, 3] = pos
    return T


def render_cameras(sim, spec):
    """Ideal frames of all spec cameras in the current state, plus part poses."""
    robot, table, bodies, link_index = sim["robot"], sim["table"], sim["bodies"], sim["link_index"]
    W, H = spec["camera"]["resolution"]
    K = np.array(spec["camera"]["K"])
    near, far = spec["camera"]["near"], spec["camera"]["far"]
    # TinyRenderer samples column c at u = c and row r at v = r + 1 (top-left image coords), while K
    # (and Isaac) put pixel centres at +0.5. Shift the frustum so pixel (c, r) sees the ray through
    # (c + 0.5, r + 0.5) under K; otherwise every image is off by half a pixel (found via the
    # hole detector: systematic (+0.4, -0.5) px offset against CAD ground truth).
    cx_tr, cy_tr = K[0, 2] - 0.5, K[1, 2] + 0.5
    proj = p.computeProjectionMatrix(left=-cx_tr * near / K[0, 0], right=(W - cx_tr) * near / K[0, 0],
                                     bottom=-(H - cy_tr) * near / K[1, 1], top=cy_tr * near / K[1, 1],
                                     nearVal=near, farVal=far)
    cameras = {}
    for cam_name, cam in spec["cameras"].items():
        T_link = link_pose(robot, link_index[cam["link"]])
        T_off = trimesh.transformations.quaternion_matrix(cam["quat_wxyz"])
        T_off[:3, 3] = cam["pos"]
        T = T_link @ T_off                                               # world <- optical frame
        eye, fwd, down = T[:3, 3], T[:3, 2], T[:3, 1]
        view = p.computeViewMatrix(eye, eye + fwd, -down)
        _, _, rgba, zbuf, seg = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER, shadow=1,
                                                 lightDirection=[0.3, 0.2, 1.0])
        zbuf = np.reshape(zbuf, (H, W))
        depth = far * near / (far - (far - near) * zbuf)               # OpenGL z-buffer -> metric z-depth
        depth[zbuf >= 1.0] = 0.0
        seg = np.reshape(seg, (H, W))
        instance = np.full((H, W), INSTANCE_BACKGROUND, np.uint8)
        instance[seg == table] = INSTANCE_TABLE
        instance[seg == robot] = INSTANCE_ROBOT
        for part in spec["parts"]:
            instance[seg == bodies[part["name"]]] = part["instance_id"]
        cameras[cam_name] = {"rgb": np.reshape(rgba, (H, W, 4))[..., :3], "depth": depth, "instance": instance,
                             "cam_pose_ros": {"position": T[:3, 3].tolist(),
                                              "quat_wxyz": trimesh.transformations.quaternion_from_matrix(T).tolist()}}

    part_poses = {}
    for name, body in bodies.items():
        pos, q = p.getBasePositionAndOrientation(body)
        part_poses[name] = {"position": list(pos), "quat_wxyz": [q[3], q[0], q[1], q[2]]}
    return {"cameras": cameras, "part_poses": part_poses}


def render(spec, gui=False):
    sim = build(spec, gui)
    frame = render_cameras(sim, spec)
    if gui:
        input("[pybullet] press Enter to close the window")
    p.disconnect()
    return frame
