"""D435 camera model (d435.py) and the command-line options shared by the evaluation and grasp scripts.

  --camera ideal         ideal rendered depth (reference)
  --camera d435          analytical D435 model: Intel axial noise + datasheet rules (main)
           --axial ahn   Ahn et al. 2019 axial noise instead of Intel's (ablation)
           --axial none  no axial noise (datasheet rules only)
           --no-rules    axial noise only
"""
from .d435 import simulate  # noqa: F401


def add_args(ap):
    ap.add_argument("--camera", choices=["ideal", "d435"], default="ideal", help="depth: ideal or the D435 model")
    ap.add_argument("--axial", choices=["intel", "ahn", "none"], default="intel",
                    help="D435 axial noise model: intel (main), ahn (ablation), none")
    ap.add_argument("--no-rules", action="store_true", help="D435 model without the datasheet rules")


def tag(args):
    """Short name of the depth condition, used in output file names and printouts."""
    if args.camera == "ideal":
        return "ideal"
    return f"d435_{args.axial}" + ("_norules" if args.no_rules else "")


def depth(args, ideal_depth, K, rng):
    """The depth a script works on: ideal, or the ideal depth through the D435 model."""
    if args.camera == "ideal":
        return ideal_depth
    return simulate(ideal_depth, K, rng, axial=None if args.axial == "none" else args.axial,
                    rules=not args.no_rules)
