"""Live web view of a running (or finished) final_sweep: progress per condition x stage, running success rates,
and the plots once `analyse` has written them.

experiments/final_sweep.py starts this view itself and opens it in the browser (--no-view to skip); run it on
its own to look at a sweep that is running elsewhere or finished:
  python experiments/watch.py                                   # output/final_sweep/, http://127.0.0.1:8765
  python experiments/watch.py --out output/final_sweep_n100     # the extended run
  python experiments/watch.py --port 8800

Read-only: it only reads output/<sweep>/test_seeds.json and raw/<condition>/<stage>/seed_XXX/results*.json,
so it can be started and stopped at any time while the sweep runs. The page refreshes every 10 s.
Rates are raw running counts (no ideal pairing, no statistics); the final numbers come from
`final_sweep.py analyse`.
"""
import argparse
import glob
import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
from final_sweep import CONDITIONS, LABELS, STAGES  # noqa: E402

METRICS = [("holes", "right_hole", "hole detected"), ("holes", "correct", "detector within 3 mm"),
           ("fit", "success", "fit success (5 mm, 10°)"), ("fit", "found", "fit pose found (15 mm)"),
           ("grasp", "success", "grasp lifted")]


def outcome(stage, metric, r):
    if stage == "fit" and metric == "found":
        return "t_err_mm" in r and r["t_err_mm"] < 15.0 and r["r_err_deg"] < 10.0
    return bool(r.get(metric, False))


def status(out):
    try:
        with open(os.path.join(out, "test_seeds.json")) as f:
            seeds = json.load(f)
    except OSError:
        return {"waiting": f"{os.path.relpath(out, ROOT)}: selecting the test seeds (screening) - "
                           "progress appears when the seed list is written", "time": time.strftime("%H:%M:%S")}
    now = time.time()
    jobs = {"done": 0, "running": 0, "total": len(CONDITIONS) * len(STAGES) * len(seeds)}
    progress, rates, starts, ends = {}, {}, [], []
    for c in CONDITIONS:
        for st in STAGES:
            done = running = 0
            for s in seeds:
                d = os.path.join(out, "raw", c, st, f"seed_{s:03d}")
                files = glob.glob(os.path.join(d, "**", "results*.json"), recursive=True)
                log = os.path.join(d, "log.txt")
                if os.path.exists(log):
                    starts.append(os.path.getmtime(d))
                if files:
                    done += 1
                    ends.append(os.path.getmtime(files[0]))
                    try:
                        with open(files[0]) as f:
                            rows = json.load(f)
                    except (OSError, ValueError):
                        continue
                    for stg, m, _ in METRICS:
                        if stg == st:
                            k, n = rates.get((c, st, m), (0, 0))
                            rates[(c, st, m)] = (k + sum(outcome(st, m, r) for r in rows), n + len(rows))
                elif os.path.exists(log):
                    running += 1
            progress[f"{c}|{st}"] = {"done": done, "running": running, "total": len(seeds)}
            jobs["done"] += done
            jobs["running"] += running
    eta = None
    if ends and jobs["done"] < jobs["total"]:
        elapsed = max(ends) - min(starts)
        eta = elapsed / jobs["done"] * (jobs["total"] - jobs["done"]) if jobs["done"] else None
    return {"out": os.path.relpath(out, ROOT), "n_seeds": len(seeds), "jobs": jobs, "eta_s": eta,
            "last_result_age_s": now - max(ends) if ends else None,
            "conditions": [[c, LABELS[c]] for c in CONDITIONS], "stages": list(STAGES),
            "metrics": [[st, m, lab] for st, m, lab in METRICS], "progress": progress,
            "rates": {f"{c}|{st}|{m}": {"k": k, "n": n} for (c, st, m), (k, n) in rates.items()},
            "time": time.strftime("%H:%M:%S"),
            "plots": sorted(os.path.basename(f) + f"?t={int(os.path.getmtime(f))}"
                            for f in glob.glob(os.path.join(out, "plots", "*.png")))}


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sweep Monitor</title>
<style>
:root{--bg:#fbfaf7;--surface:#ffffff;--ink:#2b2b2b;--muted:#77766f;--line:#e4e3df;--track:#efeee9;
  --s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;--warn:#b26b00}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#121211;--surface:#1a1a19;--ink:#f0efec;
  --muted:#a8a79e;--line:#2e2e2c;--track:#262624;--s1:#3987e5;--s2:#d95926;--s3:#199e70;--warn:#e0a030}}
:root[data-theme="dark"]{--bg:#121211;--surface:#1a1a19;--ink:#f0efec;--muted:#a8a79e;--line:#2e2e2c;--track:#262624;
  --s1:#3987e5;--s2:#d95926;--s3:#199e70;--warn:#e0a030}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding:20px 16px 40px}
h1{font-size:20px;margin:0 0 2px} h2{font-size:15px;margin:28px 0 8px}
.sub{color:var(--muted);font-size:13px}
.card{background:var(--surface);border:1px solid var(--line);border-radius:10px;padding:14px 16px;margin-top:12px}
.kpis{display:flex;flex-wrap:wrap;gap:24px}
.kpi b{display:block;font-size:22px;font-variant-numeric:tabular-nums}
.kpi span{color:var(--muted);font-size:12px}
.bar{height:8px;background:var(--track);border-radius:4px;overflow:hidden;position:relative}
.bar i{display:block;height:100%;background:var(--s1);border-radius:4px}
.bar i.run{position:absolute;top:0;background:var(--s1);opacity:.35;border-radius:0}
.scroll{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}
th,td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
th{font-weight:600;color:var(--muted);font-size:12px}
td.cell{min-width:120px}
.small{font-size:12px;color:var(--muted)}
.legend{display:flex;gap:16px;flex-wrap:wrap;font-size:12px;color:var(--muted);margin-bottom:8px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px;vertical-align:-1px}
.rate{display:flex;align-items:center;gap:8px}
.rate .bar{flex:1;min-width:60px}
.err{color:var(--warn)}
#plots img{max-width:100%;display:block;margin-top:12px;border-radius:6px;background:#fff}
</style></head><body><main>
<h1>Sweep monitor</h1>
<div class="sub" id="sub">loading…</div>
<div class="card kpis" id="kpis"></div>
<h2>Running success rates</h2>
<div class="legend"><span><i style="background:var(--s1)"></i>strict</span><span><i style="background:var(--s2)"></i>15 mm allowance</span><span><i style="background:var(--s3)"></i>grasp lifted</span></div>
<div class="card scroll"><table id="rates"></table></div>
<p class="small">Raw counts so far (k/n draws); no pairing or intervals — final statistics: <code>final_sweep.py analyse</code>.</p>
<h2>Progress by condition and stage</h2>
<div class="card scroll"><table id="prog"></table></div>
<div id="plots"></div>
</main>
<script>
const color={correct:"var(--s1)",success:"var(--s1)",right_hole:"var(--s2)",found:"var(--s2)"};
function fmt(s){if(s==null)return"–";s=Math.round(s);const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?`${h} h ${m} min`:`${m} min ${s%60} s`}
function cell(r,col){if(!r||!r.n)return'<span class="small">–</span>';const p=100*r.k/r.n;
 return `<div class="rate" title="${r.k}/${r.n} draws"><div class="bar"><i style="width:${p}%;background:${col}"></i></div><span>${p.toFixed(0)} %</span><span class="small">${r.k}/${r.n}</span></div>`}
async function tick(){
 let d;try{d=await (await fetch("status.json",{cache:"no-store"})).json()}catch(e){document.getElementById("sub").innerHTML='<span class="err">server not reachable</span>';return}
 if(d.waiting){document.getElementById("sub").textContent=`${d.waiting} · ${d.time}`;return}
 const j=d.jobs,pct=100*j.done/j.total;
 document.getElementById("sub").textContent=`${d.out} · ${d.n_seeds} test seeds · updated ${d.time} (every 10 s)`;
 document.getElementById("kpis").innerHTML=
  `<div class="kpi" style="flex:1;min-width:220px"><b>${j.done} / ${j.total}</b><span>jobs done (${pct.toFixed(0)} %)</span><div class="bar" style="margin-top:6px"><i style="width:${pct}%"></i></div></div>`+
  `<div class="kpi"><b>${j.running}</b><span>jobs in progress</span></div>`+
  `<div class="kpi"><b>${j.done===j.total?"done":fmt(d.eta_s)}</b><span>${j.done===j.total?"all jobs finished":"remaining (rough)"}</span></div>`+
  `<div class="kpi"><b>${fmt(d.last_result_age_s)}</b><span>since last result</span></div>`;
 let h="<tr><th>condition</th>"+d.metrics.map(m=>`<th>${m[2]}</th>`).join("")+"</tr>";
 for(const [c,lab] of d.conditions){h+=`<tr><td>${lab}</td>`+d.metrics.map(m=>`<td class="cell">${cell(d.rates[c+"|"+m[0]+"|"+m[1]],m[0]==="grasp"?"var(--s3)":color[m[1]])}</td>`).join("")+"</tr>"}
 document.getElementById("rates").innerHTML=h;
 let g="<tr><th>condition</th>"+d.stages.map(s=>`<th>${s}</th>`).join("")+"</tr>";
 for(const [c,lab] of d.conditions){g+=`<tr><td>${lab}</td>`+d.stages.map(s=>{const p=d.progress[c+"|"+s],a=100*p.done/p.total,b=100*p.running/p.total;
  return `<td class="cell" title="${p.done} done, ${p.running} in progress, of ${p.total} seeds"><div class="rate"><div class="bar"><i style="width:${a}%"></i><i class="run" style="left:${a}%;width:${b}%"></i></div><span class="small">${p.done}/${p.total}</span></div></td>`}).join("")+"</tr>"}
 document.getElementById("prog").innerHTML=g;
 const pl=document.getElementById("plots"),key=d.plots.join("|");
 if(pl.dataset.key!==key){pl.dataset.key=key;pl.innerHTML=d.plots.length?"<h2>Plots (final_sweep.py analyse)</h2>"+
   d.plots.map(f=>`<div class="card"><img src="plots/${f}" alt="${f.split("?")[0]}"></div>`).join(""):""}}
tick();setInterval(tick,10000);
</script></body></html>"""


def start(out, host="127.0.0.1", port=8765, open_browser=True):
    """Serve the view of sweep folder `out` in a background thread (first free port from `port`); returns the URL."""
    out = os.path.abspath(out)

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            if path.startswith("/status.json"):
                body, ctype = json.dumps(status(out)).encode(), "application/json"
            elif path in ("/", "/index.html"):
                body, ctype = PAGE.encode(), "text/html; charset=utf-8"
            elif path.startswith("/plots/") and path.endswith(".png") and "/" not in path[7:]:
                try:
                    with open(os.path.join(out, "plots", path[7:]), "rb") as f:
                        body, ctype = f.read(), "image/png"
                except OSError:
                    self.send_error(404)
                    return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    for p in range(port, port + 20):
        try:
            srv = ThreadingHTTPServer((host, p), H)
            break
        except OSError:
            continue
    else:
        raise OSError(f"no free port in {port}-{port + 19}")
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://{host}:{p}/"
    if open_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    return url


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "final_sweep"), help="sweep output folder")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1", help="127.0.0.1 = this machine only")
    ap.add_argument("--open", action="store_true", help="also open it in the browser")
    a = ap.parse_args()
    url = start(a.out, a.host, a.port, a.open)
    print(f"watching {os.path.abspath(a.out)}\nopen {url}  (Ctrl+C to stop)")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
