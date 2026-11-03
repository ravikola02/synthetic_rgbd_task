"""Render one ideal frame of the common scene with either backend.

  python scene/render.py --backend pybullet --seed 0
  OMNI_KIT_ACCEPT_EULA=YES python scene/render.py --backend isaac --seed 0

Same seed -> same scene in both backends; output in data/ideal/<backend>/seed_XXX/<camera>/
(the ideal reference set; format: see frame_io.py).
  python scene/render.py --backend pybullet --test-seeds     # all config.TEST_SEEDS (PyBullet only)
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "backends"))

from config import TEST_SEEDS  # noqa: E402
from frame_io import ideal_frame_dir, save_frame  # noqa: E402
from materials import MATERIALS  # noqa: E402
from spec import make_scene  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=["pybullet", "isaac"], default="pybullet")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--test-seeds", action="store_true", help="PyBullet: render all config.TEST_SEEDS")
parser.add_argument("--profile-material", choices=[k for k in MATERIALS if k.startswith("aluminium")],
                    default=None, help="override the profile material (default: anodized, as in the item datasheet)")
parser.add_argument("--settle", action="store_true", help="drop parts with physics instead of placing them at rest")
parser.add_argument("--mdl", action="store_true", help="Isaac only: use NVIDIA MDL materials")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--out", default=None, help="default: data/ideal/<backend>/")
args = parser.parse_args()



def render_one(seed):
    spec = make_scene(seed, profile_material=args.profile_material, settle=args.settle)
    if args.backend == "pybullet":
        from pybullet_backend import render
        frame = render(spec, gui=args.gui)
    else:
        from isaac_backend import render
        frame = render(spec, gui=args.gui, use_mdl=args.mdl)
    print(f"[render:{args.backend}] seed {seed}")
    for cam_name, f in frame["cameras"].items():                 # one sub-folder per camera
        out = os.path.join(args.out, f"seed_{seed:03d}", cam_name) if args.out else ideal_frame_dir(args.backend, seed, cam_name)
        save_frame(out, f["rgb"], f["depth"], f["instance"], spec, f["cam_pose_ros"], frame["part_poses"], args.backend)
        d, inst = f["depth"], f["instance"]
        parts = inst >= 3
        rng_txt = f"{d[parts].min():.3f}-{d[parts].max():.3f} m" if parts.any() else "not visible"
        print(f"  {cam_name:12s} part pixels {int(parts.sum()):5d}, part depth {rng_txt}, "
              f"robot pixels {int((inst == 2).sum())}  -> {out}")


if args.test_seeds:
    assert args.backend == "pybullet", "Isaac starts once per process: loop over seeds in the shell"
    for s in TEST_SEEDS:
        render_one(s)
else:
    render_one(args.seed)
if args.backend == "isaac":
    from isaac_backend import close
    close()                                                        # exits the process (Isaac)
