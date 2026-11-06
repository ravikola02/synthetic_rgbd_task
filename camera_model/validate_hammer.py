"""Test the D435 model against real D435 depth: HAMMER dataset (Jung et al. 2022/2023).

HAMMER records 13 indoor scenes with a RealSense D435 (projector on, 848x480) and gives, for every frame,
ground-truth depth rendered into the same D435 view from 3D scans (objects 0.8 mm RMSE; walls / table from
a hand-held scanner, less accurate). The model is applied to that ground truth with HAMMER's intrinsics, and
the same statistics are computed on the real and the simulated depth:

  invalid     share of pixels with no depth (away from depth edges)
  bias        per-frame scale error, median(depth / GT - 1), on the walls (flat, diffuse)
  pixel noise trimmed RMS of horizontal neighbour differences of the error / sqrt(2): the uncorrelated
              part, comparable with the model's per-pixel noise
  plane RMS   RMS residual of a plane fitted to the depth in 21 x 21 px patches (no GT involved; includes
              correlated error up to the patch size); Intel's own depth-quality metric is of this kind
  spread      MAD of the error after removing the frame's median error: includes correlated
              ("egg carton") error and GT error (subtract ~0.8 mm in quadrature on objects)
  fattening   share of background pixels k px from a >= 30 mm closer pixel that read the foreground depth
              (and the share that are invalid); GT-to-sensor misalignment of ~1 px also shows up here

Regions by HAMMER's instance labels: walls (200), table (180), objects (the rest). Statistics are taken
4 px away from depth edges, except the fattening.

  python camera_model/validate_hammer.py            # fetch the sample (~45 MB, once) and analyse
  python camera_model/validate_hammer.py fetch      # only download
  python camera_model/validate_hammer.py analyse    # only analyse (outputs in output/hammer/)

The full archive is 170 GB; `fetch` reads only the needed members with HTTP range requests: 8 evenly spaced
frames of the first trajectory of each scene (d435/_gt, d435/depth_proj, d435/_instance, intrinsics).
Data licence: CC BY-NC-SA 4.0 (https://github.com/Junggy/HAMMER-dataset); the data is not in the repo.
"""
import csv
import io
import os
import sys
import urllib.request
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import d435  # noqa: E402

URL = "https://www.campar.in.tum.de/public_datasets/2022_arxiv_jung/_dataset_processed.zip"
DATA = os.path.join(ROOT, "data", "hammer")
OUT = os.path.join(ROOT, "output", "hammer")
FRAMES_PER_SCENE = 8
BINS = [(400, 600), (600, 800), (800, 1000), (1000, 1400)]       # GT depth bins (mm)
STEP = 30.0                                                      # mm, depth step that counts as an edge
EDGE_MARGIN = 4                                                  # px kept away from edges for the statistics
REGIONS = {"walls": lambda ins: ins == 200, "table": lambda ins: ins == 180,
           "objects": lambda ins: (ins != 200) & (ins != 180)}
SOURCES = ("real", "model")
FAT_SOURCES = ("real", "model", "model, no edge fattening")      # the last one = measurement floor


# ------------------------------------------------------------------ fetch
class _HTTPFile(io.RawIOBase):
    """Seekable read-only view of a remote file through HTTP range requests."""

    def __init__(self, url):
        self.url, self.pos = url, 0
        head = urllib.request.urlopen(urllib.request.Request(url, method="HEAD"), timeout=60)
        self.size = int(head.headers["Content-Length"])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def readinto(self, b):
        if self.pos >= self.size or not len(b):
            return 0
        end = min(self.pos + len(b), self.size) - 1
        req = urllib.request.Request(self.url, headers={"Range": f"bytes={self.pos}-{end}"})
        data = urllib.request.urlopen(req, timeout=120).read()
        b[:len(data)] = data
        self.pos += len(data)
        return len(data)


def fetch():
    print(f"reading the archive's directory ({URL}) ...", flush=True)
    z = zipfile.ZipFile(io.BufferedReader(_HTTPFile(URL), buffer_size=1 << 20))
    frames = {}
    for n in z.namelist():
        parts = n.split("/")
        if len(parts) == 4 and parts[0].endswith("traj1_1") and parts[1:3] == ["d435", "_gt"] and n.endswith(".png"):
            frames.setdefault(parts[0], []).append(parts[3])
    want = []
    for scene, fs in sorted(frames.items()):
        fs.sort()
        want.append(f"{scene}/d435/intrinsics.txt")
        for i in range(FRAMES_PER_SCENE):
            f = fs[int(i * (len(fs) - 1) / (FRAMES_PER_SCENE - 1))]
            want += [f"{scene}/d435/{k}/{f}" for k in ("_gt", "depth_proj", "_instance")]
    todo = [n for n in want if not os.path.exists(os.path.join(DATA, n))]
    print(f"{len(frames)} scenes, {len(want)} files, {len(todo)} to download", flush=True)
    for i, n in enumerate(todo):
        dst = os.path.join(DATA, n)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst + ".part", "wb") as f:
            f.write(z.read(n))
        os.replace(dst + ".part", dst)
        if (i + 1) % 25 == 0:
            print(f"  {i + 1}/{len(todo)}", flush=True)
    print(f"data in {DATA}")


# ------------------------------------------------------------------ analyse
def _trimmed_rms(x, keep=0.95):
    """RMS of the central `keep` share of |x| (robust to outliers, fine below the 1 mm quantisation step)."""
    if x.size <= 50:
        return np.nan
    a = np.sort(np.abs(x))[:int(keep * x.size)]
    return float(np.sqrt(np.mean(a ** 2)))


def plane_noise(d, mask, win=21):
    """Sensor noise without the GT: RMS residual of a least-squares plane fitted to the depth in win x win
    patches fully inside the mask (valid pixels only, >= 80 % valid); median over patches (mm)."""
    h, w = d.shape
    vv, uu = np.mgrid[0:win, 0:win]
    res = []
    for v0 in range(0, h - win, win):
        for u0 in range(0, w - win, win):
            m = mask[v0:v0 + win, u0:u0 + win]
            z = d[v0:v0 + win, u0:u0 + win]
            ok = m & (z > 0)
            if m.mean() < 0.999 or ok.mean() < 0.8:
                continue
            A = np.c_[uu[ok], vv[ok], np.ones(ok.sum())]
            coef, *_ = np.linalg.lstsq(A, z[ok], rcond=None)
            res.append(np.sqrt(np.mean((z[ok] - A @ coef) ** 2)))
    return float(np.median(res)) if len(res) >= 3 else np.nan, len(res)


def _mad(x):
    return 1.4826 * float(np.median(np.abs(x - np.median(x)))) if x.size > 50 else np.nan


def _frames():
    import glob
    for gp in sorted(glob.glob(os.path.join(DATA, "*", "d435", "_gt", "*.png"))):
        base, name = os.path.dirname(os.path.dirname(gp)), os.path.basename(gp)
        paths = [gp, os.path.join(base, "depth_proj", name), os.path.join(base, "_instance", name)]
        if all(os.path.exists(p) for p in paths):
            yield gp.split(os.sep)[-4], name, paths, np.loadtxt(os.path.join(base, "intrinsics.txt"))


def region_stats(d, g, region):
    """Statistics of depth d against GT g (mm) on a region mask (already away from edges)."""
    valid = region & (d > 0)
    s = {"px": int(region.sum()), "invalid": float(1 - valid.sum() / max(region.sum(), 1)),
         "gt_mm": float(np.median(g[region])) if region.any() else np.nan,
         "bias_pct": np.nan, "pix_noise_mm": np.nan, "spread_mm": np.nan, "plane_rms_mm": np.nan}
    if valid.sum() > 50:
        e = np.where(valid, d - g, np.nan)
        pair = valid[:, 1:] & valid[:, :-1]
        s["bias_pct"] = float(np.median(d[valid] / g[valid] - 1) * 100)
        s["pix_noise_mm"] = _trimmed_rms((e[:, 1:] - e[:, :-1])[pair]) / np.sqrt(2)
        s["spread_mm"] = _mad(e[valid] - np.median(e[valid]))
        s["plane_rms_mm"] = plane_noise(d, region)[0]
    return s


def fattening(d, g, kmax=6):
    """Per distance k (px) from a >= STEP closer pixel: (background px, share reading the foreground among
    valid ones, share invalid)."""
    from scipy.ndimage import minimum_filter
    out, prev = [], np.zeros(g.shape, bool)
    for k in range(1, kmax + 1):
        fg = minimum_filter(g, size=2 * k + 1)
        near = (g - fg > STEP) & (g > 0)
        ring, prev = near & ~prev, near
        v = ring & (d > 0)
        fat = v & (np.abs(d - fg) < np.abs(d - g))
        out.append((int(ring.sum()), fat.sum() / max(v.sum(), 1), (ring & (d == 0)).sum() / max(ring.sum(), 1)))
    return out


def analyse():
    import cv2
    from scipy.ndimage import binary_dilation, minimum_filter
    os.makedirs(OUT, exist_ok=True)
    rows, fat = [], {s: [] for s in FAT_SOURCES}
    for fi, (scene, name, (gp, dp, ip), K) in enumerate(_frames()):
        g = cv2.imread(gp, -1).astype(float)
        real = cv2.imread(dp, -1).astype(float)
        ins = cv2.imread(ip, -1)
        ins = ins[..., 0] if ins.ndim == 3 else ins
        model = d435.simulate((g / 1000).astype(np.float32), K, np.random.default_rng((fi, 0, 0))) * 1000.0
        edges = (g - minimum_filter(g, size=3) > STEP) | (g <= 0)
        away = ~binary_dilation(edges, iterations=EDGE_MARGIN)
        for src, d in (("real", real), ("model", model)):
            for rname, rfun in REGIONS.items():
                for lo, hi in BINS:
                    reg = rfun(ins) & away & (g >= lo) & (g < hi)
                    if reg.sum() >= 500:
                        rows.append({"src": src, "scene": scene, "frame": name, "region": rname, "bin": f"{lo}-{hi}",
                                     **region_stats(d, g, reg)})
            fat[src].append(fattening(d, g))
        fat["model, no edge fattening"].append(fattening(
            d435.simulate((g / 1000).astype(np.float32), K, np.random.default_rng((fi, 0, 0)), edge=None) * 1000.0, g))
    if not rows:
        sys.exit(f"no frames in {DATA}: run `python camera_model/validate_hammer.py fetch` first")
    keys = list(rows[0])
    with open(os.path.join(OUT, "rows.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    n_frames = len({(r["scene"], r["frame"]) for r in rows})
    lines = [f"# D435 model vs real D435 (HAMMER, {n_frames} frames, {len({r['scene'] for r in rows})} scenes)", "",
             "Medians over frames; statistics 4 px away from depth edges. Model = camera_model/d435.py applied to the "
             "HAMMER ground truth with HAMMER's intrinsics (f = 423 px).", "",
             "| region | GT depth (mm) | frames | invalid real / model | pixel noise real / model (mm) | "
             "plane-fit RMS real / model (mm) | spread vs GT real / model (mm) | Intel sigma_z (mm) |",
             "|---|---|---|---|---|---|---|---|"]
    summary = []
    for rname in REGIONS:
        for lo, hi in BINS:
            sel = {s: [r for r in rows if r["src"] == s and r["region"] == rname and r["bin"] == f"{lo}-{hi}"]
                   for s in SOURCES}
            if not sel["real"]:
                continue
            med = {s: {k: float(np.nanmedian([r[k] for r in sel[s]])) for k in
                       ("gt_mm", "invalid", "pix_noise_mm", "plane_rms_mm", "spread_mm", "bias_pct")} for s in SOURCES}
            z = med["real"]["gt_mm"] / 1000
            sig = d435.sigma_intel(z, 423.0) * 1000
            summary.append({"region": rname, "bin": f"{lo}-{hi}", "frames": len(sel["real"]), "intel_sigma_mm": sig,
                            **{f"{s}_{k}": v for s in SOURCES for k, v in med[s].items()}})
            lines.append(f"| {rname} | {lo}-{hi} | {len(sel['real'])} | {med['real']['invalid']:.0%} / "
                         f"{med['model']['invalid']:.0%} | {med['real']['pix_noise_mm']:.2f} / "
                         f"{med['model']['pix_noise_mm']:.2f} | {med['real']['plane_rms_mm']:.2f} / "
                         f"{med['model']['plane_rms_mm']:.2f} | {med['real']['spread_mm']:.2f} / "
                         f"{med['model']['spread_mm']:.2f} | {sig:.2f} |")
    with open(os.path.join(OUT, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    # per-frame scale bias on the walls
    lines += ["", "## Scale bias (per frame, walls)", ""]
    bias = {}
    for s in SOURCES:
        per = {}
        for r in rows:
            if r["src"] == s and r["region"] == "walls" and not np.isnan(r["bias_pct"]):
                per.setdefault((r["scene"], r["frame"]), []).append(r["bias_pct"])
        b = np.array([np.median(v) for v in per.values()])
        bias[s] = b
        lines.append(f"- {s}: median {np.median(b):+.2f} %, middle 50 % {np.percentile(b, 25):+.2f} .. "
                     f"{np.percentile(b, 75):+.2f} %, |bias| > 2 % in {np.mean(np.abs(b) > 2):.0%} of {b.size} frames")
    by_scene = {}
    for r in rows:
        if r["src"] == "real" and r["region"] == "walls" and not np.isnan(r["bias_pct"]):
            by_scene.setdefault(r["scene"], []).append(r["bias_pct"])
    lines.append("- real, per scene (median %): " + ", ".join(f"{k.split('_')[0]} {np.median(v):+.1f}"
                                                           for k, v in sorted(by_scene.items(), key=lambda x: int(x[0].split('_')[0][5:]))))
    # edge fattening
    lines += ["", "## Edge fattening", "",
              "Background pixels k px from a >= 30 mm closer pixel: share reading the foreground depth (of the valid "
              "ones) / share invalid. The model without edge fattening shows the measurement floor (noise, slanted "
              "surfaces).", "", "| k (px) | " + " | ".join(FAT_SOURCES) + " |", "|---|---|---|---|"]
    fat_curves = {}
    for s in FAT_SOURCES:
        a = np.array(fat[s])                                       # frames x k x 3
        tot = a[:, :, 0].sum(0)
        fat_curves[s] = ((a[:, :, 1] * a[:, :, 0]).sum(0) / tot, (a[:, :, 2] * a[:, :, 0]).sum(0) / tot)
    for k in range(len(fat_curves["real"][0])):
        lines.append(f"| {k + 1} | " + " | ".join(f"{fat_curves[s][0][k]:.0%} / {fat_curves[s][1][k]:.0%}"
                                                  for s in FAT_SOURCES) + " |")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(OUT, "summary.md"), "w") as f:
        f.write(text)
    print(text)
    plot(summary, bias, fat_curves)
    print(f"outputs in {OUT}")


def plot(summary, bias, fat_curves):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, real_c, model_c, ref_c = "#2e2c27", "#2a6fdb", "#d9822b", "#8a8a85"
    fig, axs = plt.subplots(1, 4, figsize=(16, 3.6))
    zz = np.linspace(0.4, 1.4, 50)
    # 1 noise vs depth (walls + objects)
    ax = axs[0]
    for rname, mk in (("walls", "o"), ("objects", "s")):
        rr = [r for r in summary if r["region"] == rname]
        z = [r["real_gt_mm"] / 1000 for r in rr]
        ax.plot(z, [r["real_pix_noise_mm"] for r in rr], mk + "-", color=real_c, label=f"real, {rname}")
        ax.plot(z, [r["model_pix_noise_mm"] for r in rr], mk + "--", color=model_c, label=f"model, {rname}")
    ax.plot(zz, d435.sigma_intel(zz, 423.0) * 1000, ":", color=ref_c, label="Intel $\\sigma_z$ (0.08 px)")
    ax.plot(zz, d435.sigma_ahn(zz) * 1000, "-.", color=ref_c, label="Ahn et al. 2019")
    ax.set(xlabel="GT depth (m)", ylabel="mm", title="pixel noise (uncorrelated)")
    ax.legend(fontsize=7, frameon=False)
    # 2 spread
    ax = axs[1]
    for rname, mk in (("walls", "o"), ("objects", "s")):
        rr = [r for r in summary if r["region"] == rname]
        z = [r["real_gt_mm"] / 1000 for r in rr]
        ax.plot(z, [r["real_plane_rms_mm"] for r in rr], mk + "-", color=real_c, label=f"real, {rname}")
        ax.plot(z, [r["model_plane_rms_mm"] for r in rr], mk + "--", color=model_c, label=f"model, {rname}")
    ax.set(xlabel="GT depth (m)", ylabel="mm", title="plane-fit RMS, 21 px patches (no GT)")
    ax.legend(fontsize=7, frameon=False)
    # 3 bias histogram
    ax = axs[2]
    bins = np.arange(-8, 8.5, 0.5)
    ax.hist(bias["real"], bins=bins, color=real_c, alpha=0.8, label="real (per frame)")
    ax.hist(bias["model"], bins=bins, color=model_c, alpha=0.6, label="model (per draw)")
    for x in (-2, 2):
        ax.axvline(x, color=ref_c, ls=":")
    ax.set(xlabel="scale bias on walls (%)", ylabel="frames", title="depth bias (datasheet limit ±2 %)")
    ax.legend(fontsize=7, frameon=False)
    # 4 edge fattening
    ax = axs[3]
    k = np.arange(1, len(fat_curves["real"][0]) + 1)
    for s, c in (("real", real_c), ("model", model_c), ("model, no edge fattening", ref_c)):
        ax.plot(k, fat_curves[s][0] * 100, "o-", color=c, label=f"{s}: reads foreground")
        if s != "model, no edge fattening":
            ax.plot(k, fat_curves[s][1] * 100, "x:", color=c, label=f"{s}: invalid")
    ax.set(xlabel="distance from depth step (px)", ylabel="% of background px", title="edge fattening")
    ax.legend(fontsize=7, frameon=False)
    for ax in axs:
        ax.title.set_color(ink)
        ax.title.set_horizontalalignment("left")
        ax.title.set_x(0.0)
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "hammer_vs_model.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd in ("fetch", "all"):
        fetch()
    if cmd in ("analyse", "all"):
        analyse()
