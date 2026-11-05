"""D435 camera model (d435.py) and the command-line options shared by the evaluation and grasp scripts.

  --camera ideal            ideal rendered depth (reference)
  --camera d435             D435 model: edge fattening + Intel axial noise + depth bias (+-2 %, random sign)
                            + datasheet rules
Ablations of the D435 model (each switches one part):
           --edge none      no edge fattening
           --axial ahn      Ahn et al. 2019 axial noise instead of Intel's; --axial none: no axial noise
           --bias 0         no depth bias; --bias 2 / --bias -2: fixed sign (%)
           --no-rules       no datasheet rules
"""
from .d435 import simulate  # noqa: F401


def add_args(ap):
    ap.add_argument("--camera", choices=["ideal", "d435"], default="ideal", help="depth: ideal or the D435 model")
    ap.add_argument("--edge", choices=["r200", "none"], default="r200", help="D435 edge fattening (R200 value) or none")
    ap.add_argument("--axial", choices=["intel", "ahn", "none"], default="intel",
                    help="D435 axial noise: intel (main), ahn (ablation), none")
    ap.add_argument("--bias", default="random",
                    help="D435 depth bias: random (+-2 %%, 50/50 per draw), a fixed value in %% (e.g. 2, -2) or 0")
    ap.add_argument("--no-rules", action="store_true", help="D435 model without the datasheet rules")


def _bias(args):
    return "random" if args.bias == "random" else float(args.bias) / 100


def tag(args):
    """Short name of the depth condition for file names and printouts: 'd435' = full model, plus the ablations."""
    if args.camera == "ideal":
        return "ideal"
    parts = ["d435"]
    if args.edge != "r200":
        parts.append(f"edge-{args.edge}")
    if args.axial != "intel":
        parts.append(f"axial-{args.axial}")
    if args.bias != "random":
        parts.append(f"bias{float(args.bias):+g}" if float(args.bias) else "bias0")
    if args.no_rules:
        parts.append("norules")
    return "_".join(parts)


def depth(args, ideal_depth, K, rng):
    """The depth a script works on: ideal, or the ideal depth through the D435 model."""
    if args.camera == "ideal":
        return ideal_depth
    return simulate(ideal_depth, K, rng, edge=None if args.edge == "none" else args.edge,
                    axial=None if args.axial == "none" else args.axial, bias=_bias(args), rules=not args.no_rules)
