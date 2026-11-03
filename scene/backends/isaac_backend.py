"""Isaac Sim 5.1 backend (RTX). Builds the same scene spec, returns one ideal frame.

Materials use UsdPreviewSurface (rgba/metallic/roughness from materials.py) so the
two backends share one material definition. Set use_mdl=True to use the NVIDIA MDL
material named in materials.py instead (downloaded from the public asset server).

Must be the first thing that starts Isaac in the process (SimulationApp is created here).
SimulationApp.close() ends the whole Python process, so render() leaves the app open:
save the returned frame first, then call close(). Run with OMNI_KIT_ACCEPT_EULA=YES.
"""
import math

import numpy as np

from spec import INSTANCE_BACKGROUND, INSTANCE_ROBOT, INSTANCE_TABLE

NUCLEUS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/NVIDIA/"


_APP = None


def close():
    """Shut Isaac down (this exits the Python process)."""
    if _APP is not None:
        _APP.close()


def render(spec, gui=False, use_mdl=False):
    global _APP
    from isaacsim import SimulationApp
    app = _APP = SimulationApp({"headless": not gui})

    import omni.kit.commands
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    import trimesh

    mats = spec["materials"]
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 60)
    stage = world.stage
    world.scene.add_default_ground_plane(z_position=spec["floor_z"])

    # ---- materials (visual + physics), one prim per entry in materials.py
    looks = {}
    for name, m in mats.items():
        path = f"/World/Looks/{name}"
        if use_mdl and m["visual"]["mdl"]:
            mdl = NUCLEUS + m["visual"]["mdl"]
            omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url=mdl,
                                      mtl_name=mdl.split("/")[-1][:-4], mtl_path=path)
            mat = UsdShade.Material(stage.GetPrimAtPath(path))
        else:
            mat = UsdShade.Material.Define(stage, path)
            sh = UsdShade.Shader.Define(stage, path + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*m["visual"]["rgba"][:3]))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(m["visual"]["metallic"])
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(m["visual"]["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        phys.CreateStaticFrictionAttr(m["physical"]["friction"])
        phys.CreateDynamicFrictionAttr(m["physical"]["friction"])
        phys.CreateRestitutionAttr(m["physical"]["restitution"])
        looks[name] = mat

    def bind(prim, mat_name):
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        api.Bind(looks[mat_name])
        api.Bind(looks[mat_name], UsdShade.Tokens.weakerThanDescendants, "physics")

    # ---- robot
    add_reference_to_stage(spec["robot"]["usd"], spec["robot"]["prim"])

    # ---- table (static box with collider)
    t = spec["table"]
    cube = UsdGeom.Cube.Define(stage, "/World/Table")
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*t["position"]))
    xf.AddScaleOp().Set(Gf.Vec3f(*t["size"]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    bind(cube.GetPrim(), t["material"])

    # ---- parts (meshes loaded from the shared OBJ files)
    for part in spec["parts"]:
        mesh = trimesh.load(part["mesh"], force="mesh")
        root = f"/World/Parts/{part['name']}"
        pxf = UsdGeom.Xform.Define(stage, root)
        pxf.AddTranslateOp().Set(Gf.Vec3d(*part["position"]))
        w, x, y, z = part["quat_wxyz"]
        pxf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        um = UsdGeom.Mesh.Define(stage, root + "/mesh")
        um.CreatePointsAttr([Gf.Vec3f(*v) for v in mesh.vertices.astype(float)])
        um.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
        um.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist())
        um.CreateSubdivisionSchemeAttr("none")
        um.CreateNormalsAttr([Gf.Vec3f(*n) for n in np.repeat(mesh.face_normals, 3, axis=0)])
        um.SetNormalsInterpolation("faceVarying")              # sharp CAD edges
        prim = pxf.GetPrim()
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(not spec["settle"])        # static unless settling
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(part["mass"])
        UsdPhysics.CollisionAPI.Apply(um.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(um.GetPrim()).CreateApproximationAttr("convexHull")
        bind(um.GetPrim(), part["material"])

    # ---- lights
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr(400)
    rect = UsdLux.RectLight.Define(stage, "/World/Lights/Overhead")
    rect.CreateIntensityAttr(15000); rect.CreateWidthAttr(1.0); rect.CreateHeightAttr(1.0)
    UsdGeom.Xformable(rect).AddTranslateOp().Set(Gf.Vec3d(t["position"][0], 0.0, spec["floor_z"] + 2.5))

    # ---- cameras: each = ROS optical frame at a fixed offset under a robot link (follows the link)
    W, H = spec["camera"]["resolution"]
    K = np.array(spec["camera"]["K"])
    cams = {n: Camera(prim_path=f"{spec['robot']['prim']}/{c['link']}/D435_{n}", resolution=(W, H))
            for n, c in spec["cameras"].items()}
    robot = SingleArticulation(prim_path=spec["robot"]["prim"], name="g1")
    world.reset()
    robot.initialize()
    q = np.array([spec["robot"]["joint_positions"].get(n, 0.0) for n in robot.dof_names])
    robot.set_joint_positions(q)
    # The asset's drives (stiffness 100) let the waist sag under gravity, so the torso-mounted
    # camera kept moving while frames rendered (1 px / 1.2 mm systematic hole offset). Stiff
    # drives hold the posture like a real robot standing still.
    dof = len(robot.dof_names)
    robot.get_articulation_controller().set_gains(kps=np.full(dof, 1e6), kds=np.full(dof, 1e4))
    robot.apply_action(ArticulationAction(joint_positions=q))       # drives hold the observation pose
    # Intrinsics: set the USD camera attributes directly, at the usual millimetre scale. The
    # Camera API setters write them in stage units (metres -> e.g. focal 0.11), which the RTX
    # renderer ignores: it renders with the default 50 mm lens while get_intrinsics_matrix()
    # still reports the requested K (measured: a 1 m target at 2 m covered 848 px instead of 223).
    aperture_mm = 20.955
    for n, cam in cams.items():
        cam.initialize()
        cam.set_local_pose(translation=np.array(spec["cameras"][n]["pos"]),
                           orientation=np.array(spec["cameras"][n]["quat_wxyz"]), camera_axes="ros")
        cam.prim.GetAttribute("horizontalAperture").Set(aperture_mm)
        cam.prim.GetAttribute("verticalAperture").Set(aperture_mm * H / W)
        cam.prim.GetAttribute("focalLength").Set(aperture_mm * K[0, 0] / W)
        cam.set_clipping_range(spec["camera"]["near"], spec["camera"]["far"])
        cam.add_distance_to_image_plane_to_frame()
        cam.add_instance_id_segmentation_to_frame()

    for _ in range(240 if spec["settle"] else 1):
        world.step(render=False)
    pose0 = {n: np.asarray(cam.get_world_pose(camera_axes="ros")[0]) for n, cam in cams.items()}
    for _ in range(30):                                          # renderer warm-up / convergence
        world.step(render=True)

    # ---- read back
    def data_of(v):
        return v["data"] if isinstance(v, dict) else v

    def common_instances(seg_raw):
        seg = np.squeeze(data_of(seg_raw)).astype(np.int64)
        id_to_path = seg_raw.get("info", {}).get("idToLabels", {}) if isinstance(seg_raw, dict) else {}
        instance = np.full((H, W), INSTANCE_BACKGROUND, np.uint8)
        for sid, path in id_to_path.items():
            path = str(path)
            if path.startswith("/World/Table"):
                cid = INSTANCE_TABLE
            elif path.startswith(spec["robot"]["prim"]):
                cid = INSTANCE_ROBOT
            else:
                cid = next((pt["instance_id"] for pt in spec["parts"]
                            if path.startswith(f"/World/Parts/{pt['name']}")), None)
            if cid is not None:
                instance[seg == int(sid)] = cid
        return instance

    cameras = {}
    for n, cam in cams.items():
        fr = cam.get_current_frame()
        pos, qc = cam.get_world_pose(camera_axes="ros")
        cameras[n] = {"rgb": np.asarray(cam.get_rgb()), "depth": data_of(fr["distance_to_image_plane"]),
                      "instance": common_instances(fr["instance_id_segmentation"]),
                      "cam_pose_ros": {"position": np.asarray(pos).tolist(), "quat_wxyz": np.asarray(qc).tolist(),
                                       "drift_during_render_mm": float(np.linalg.norm(np.asarray(pos) - pose0[n]) * 1e3)}}

    part_poses = {}
    for part in spec["parts"]:
        mtx = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Parts/{part['name']}")).ComputeLocalToWorldTransform(0)
        qp = mtx.ExtractRotationQuat()
        part_poses[part["name"]] = {"position": list(mtx.ExtractTranslation()),
                                    "quat_wxyz": [qp.GetReal(), *qp.GetImaginary()]}

    if gui:
        while app.is_running():
            world.step(render=True)
    return {"cameras": cameras, "part_poses": part_poses}
