"""Tune the shoe filter on court walks: training walks only, all walks of a setting in parallel.

Objective: geometric mean over the training walks of the RMS position error at the court corners (corners are
the known truth; see court_eval.py). The test walks are never run here.

Run:  python scripts/court_tune.py recordings/imu_20261025_041004.csv results/court_20261025/walk_windows.json
          --train A1 B1 C D E --out results/court_20261025/tuning
"""
import argparse
import itertools
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import court_eval as ce  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402

W = {}


def init(phone_csv, windows, walks):
    d = ce.load(phone_csv, Path(phone_csv).with_suffix(".400hz.npz"))
    t = d["t"]
    G = np.column_stack([d["gx"], d["gy"], d["gz"]])
    A = np.column_stack([d["ax"], d["ay"], d["az"]])
    still = ce.still_mask(G, A)
    W.update(t=t, G=G, A=A, gap=d["gap"], seg={c: ce.segment(t, still, *windows[c]) for c in walks})


def evaluate(job):
    code, over = job
    cfg = PRESETS["shoe"].with_(**over)
    s0, s1, e = W["seg"][code]
    r = ce.run_walk(W["t"], W["G"], W["A"], W["gap"], s0, s1, e, cfg)
    st = ce.stances(r)
    court = ce.court_from_data(r, st, ce.SHAPE[code])
    turns = ce.find_turns(st, court.size, court.turn)
    q = court.align(r["p"])
    corners = ce.match_corners(q, r, st, turns, court)
    if not corners:
        return code, None
    s = ce.score(r, q, corners)
    err = np.array(s["corner_err"])
    return code, dict(rms=float(np.sqrt(np.mean(err ** 2))), worst=float(err.max()), closure=s["closure"],
                      tier1=s["tier1"], tier2=s["tier2"]["pct"] if s["tier2"] else None,
                      stretch=[x["pct"] for x in s["stretches"]], corners=len(corners),
                      head=float(np.mean(np.abs(s["leg_heading_change_deg"]))))


class Tuner:
    def __init__(self, pool, train, log):
        self.pool, self.train, self.log, self.cache = pool, train, log, {}

    def run(self, settings):
        """settings: list of override dicts -> list of (objective, per-walk metrics), all walks in parallel"""
        keys = [json.dumps(s, sort_keys=True) for s in settings]
        todo = [(c, s) for k, s in zip(keys, settings) if k not in self.cache for c in self.train]
        res = self.pool.map(evaluate, todo, chunksize=1) if todo else []
        fresh = {}
        for (c, s), (_, m) in zip(todo, res):
            fresh.setdefault(json.dumps(s, sort_keys=True), {})[c] = m
        for k, per in fresh.items():
            ok = all(per.get(c) for c in self.train)
            obj = float(np.exp(np.mean([np.log(per[c]["rms"]) for c in self.train]))) if ok else float("inf")
            self.cache[k] = (obj, per)
            self.log.write(json.dumps(dict(settings=json.loads(k), objective=obj, walks=per)) + "\n")
            self.log.flush()
        return [self.cache[k] for k in keys]


def line(name, obj, per, train):
    st = [x for c in train for x in per[c]["stretch"]]
    return (f"{name:46s} obj {obj:5.2f} m | rms " + " ".join(f"{c}:{per[c]['rms']:4.1f}" for c in train)
            + f" | stretch med {np.median(st):4.1f} % in3 {100 * np.mean(np.array(st) <= 3):3.0f} %"
            + f" | head/leg {np.mean([per[c]['head'] for c in train]):4.1f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("phone_csv")
    ap.add_argument("windows")
    ap.add_argument("--train", nargs="+", default=["A1", "B1", "C", "D", "E"])
    ap.add_argument("--out", default="results/court_20261025/tuning")
    ap.add_argument("--stage", default="hypotheses", choices=["hypotheses", "search", "refine", "ablate", "lowo"])
    ap.add_argument("--start", default="{}", help="JSON overrides to start the search from")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    windows = json.load(open(a.windows))
    log = open(out / f"{a.stage}.jsonl", "a")
    with Pool(16, initializer=init, initargs=(a.phone_csv, windows, a.train)) as pool:
        tu = Tuner(pool, a.train, log)
        if a.stage == "hypotheses":
            trials = {
                "untuned": {},
                "no ZARU": dict(use_zaru=False),
                "ZARU only below 0.10 rad/s": dict(zaru_gyro_thr=0.10),
                "ZARU only below 0.05 rad/s": dict(zaru_gyro_thr=0.05),
                "ZARU only below 0.02 rad/s": dict(zaru_gyro_thr=0.02),
                "stricter stance (gyro < 0.15)": dict(zupt_gyro_thr=0.15),
                "stricter stance + ZARU < 0.05": dict(zupt_gyro_thr=0.15, zaru_gyro_thr=0.05),
                "no gravity update": dict(use_gravity=False),
            }
            tic = time.time()
            res = tu.run(list(trials.values()))
            for (name, _), (obj, per) in zip(trials.items(), res):
                print(line(name, obj, per, a.train))
            print(f"({time.time() - tic:.0f} s)")
        elif a.stage == "lowo":
            # leave one training walk out: tune the chosen settings on the other walks, score the one left out
            grid = {"zupt_acc_var_thr": [0.5, 1.0, 2.0, 4.0], "sigma_zupt": [0.05, 0.15, 0.3, 0.5],
                    "sigma_g": [1.7e-4, 5e-4, 1.5e-3, 4e-3], "sigma_zaru": [0.001, 0.003, 0.01]}
            rows = []
            for held in a.train:
                rest = [c for c in a.train if c != held]
                t4 = Tuner(pool, rest, log)
                best = {}
                obj0, _ = t4.run([best])[0]
                for _ in range(2):
                    for k, vals in grid.items():
                        cands = [dict(best, **{k: v}) for v in vals]
                        res = t4.run(cands)
                        i = int(np.argmin([o for o, _ in res]))
                        if res[i][0] < obj0 - 1e-3:
                            best, obj0 = cands[i], res[i][0]
                (_, untuned), (_, tuned) = pool.map(evaluate, [(held, {}), (held, best)])
                rows.append((held, untuned["rms"], tuned["rms"], best))
                print(f"  {held} left out: tuned on {' '.join(rest)} -> {json.dumps(best)}\n"
                      f"      {held} RMS corner error: untuned {untuned['rms']:.2f} m -> {tuned['rms']:.2f} m "
                      f"(stretches within 3 %: {100 * np.mean(np.array(untuned['stretch']) <= 3):.0f} % -> "
                      f"{100 * np.mean(np.array(tuned['stretch']) <= 3):.0f} %)", flush=True)
            g = lambda i: float(np.exp(np.mean([np.log(r[i]) for r in rows])))
            print(f"\nleft-out walks, geometric mean RMS corner error: untuned {g(1):.2f} m -> tuned without them {g(2):.2f} m")
        elif a.stage == "ablate":
            # fewest changes from the preset: put each setting back in turn (costliest last) and keep it at the
            # preset if that costs < 2 %; what is left are the changes the training walks really need
            best = json.loads(a.start)
            obj0, per0 = tu.run([best])[0]
            print(line("start", obj0, per0, a.train))
            preset = PRESETS["shoe"]
            while True:
                keys = list(best)
                res = tu.run([{k2: v for k2, v in best.items() if k2 != k} for k in keys])
                for k, (o, _) in zip(keys, res):
                    print(f"  back to preset {k:18s} ({getattr(preset, k)}): {o:.3f}  ({100 * (o / obj0 - 1):+.1f} %)")
                i = int(np.argmin([o for o, _ in res]))
                if not keys or res[i][0] > 1.02 * obj0:
                    break
                print(f"  -> {keys[i]} stays at the preset value", flush=True)
                best.pop(keys[i])
                obj0, per0 = res[i]
            print(line("minimal " + json.dumps(best), obj0, per0, a.train))
            json.dump(dict(settings=best, objective=obj0, walks=per0), open(out / "best_minimal.json", "w"), indent=1)
        elif a.stage == "refine":
            # past the edges the search stopped at; keep a change only if it gains > 3 % (8 settings on 5 walks
            # can overfit), then show how flat the optimum is around each setting
            best = json.loads(a.start)
            edges = {
                "zupt_gyro_thr": [0.8, 1.0], "zupt_acc_var_thr": [4.0, 8.0], "sigma_zupt": [0.3, 0.5],
                "sigma_g": [4e-3], "sigma_a": [2e-2], "zaru_gyro_thr": [0.01], "sigma_zaru": [0.001],
            }
            obj0, per0 = tu.run([best])[0]
            print(line("start", obj0, per0, a.train))
            for k, vals in edges.items():
                cands = [dict(best, **{k: v}) for v in vals]
                res = tu.run(cands)
                i = int(np.argmin([o for o, _ in res]))
                verdict = "keep" if res[i][0] < 0.97 * obj0 else "no"
                if verdict == "keep":
                    best, obj0, per0 = cands[i], res[i][0], res[i][1]
                print(f"  beyond the edge: {k:18s} " + "  ".join(f"{v}: {o:.3f}" for v, (o, _) in zip(vals, res))
                      + f"   -> {verdict}", flush=True)
            print(line("refined " + json.dumps(best), obj0, per0, a.train))
            sens = {"zupt_gyro_thr": [0.45, 0.6, 0.8], "zupt_acc_var_thr": [1.0, 2.0, 4.0], "sigma_zupt": [0.05, 0.15, 0.3],
                    "sigma_g": [5e-4, 1.5e-3, 4e-3], "sigma_a": [2e-3, 8e-3, 2e-2], "zaru_gyro_thr": [0.01, 0.02, 0.05],
                    "sigma_zaru": [0.001, 0.003, 0.01], "use_gravity": [True]}
            print("\nsensitivity (objective, m; best = %.3f):" % obj0)
            for k, vals in sens.items():
                res = tu.run([dict(best, **{k: v}) for v in vals])
                print(f"  {k:18s} " + "  ".join(f"{v}: {o:.3f}" for v, (o, _) in zip(vals, res)), flush=True)
            json.dump(dict(settings=best, objective=obj0, walks=per0), open(out / "best_refined.json", "w"), indent=1)
        else:
            best = json.loads(a.start)
            grid = {
                "use_gravity": [False, True],
                "sigma_grav": [0.05, 0.2, 0.5, 1.0, 2.0],          # only matters with the gravity update on
                "zupt_gyro_thr": [0.2, 0.3, 0.45, 0.6],
                "zupt_acc_var_thr": [0.2, 0.5, 1.0, 2.0],
                "zupt_win": [0.04, 0.08, 0.15],
                "sigma_zupt": [0.01, 0.02, 0.05, 0.15],
                "zupt_period": [0.01, 0.02, 0.05],
                "sigma_g": [5e-5, 1.7e-4, 5e-4, 1.5e-3],
                "sigma_a": [5e-4, 2e-3, 8e-3],
                "zaru_gyro_thr": [0.02, 0.05, float("inf")],
                "sigma_zaru": [0.003, 0.01, 0.03],
            }
            obj0, per0 = tu.run([best])[0]
            print(line("start " + json.dumps(best), obj0, per0, a.train))
            for sweep in range(2):
                for k, vals in grid.items():
                    cands = [dict(best, **{k: v}) for v in vals]
                    res = tu.run(cands)
                    i = int(np.argmin([o for o, _ in res]))
                    if res[i][0] < obj0 - 1e-3:
                        best, obj0, per0 = cands[i], res[i][0], res[i][1]
                    print(f"  sweep {sweep + 1} {k:18s} -> {best.get(k, '(preset)')!s:8s}  obj {obj0:.3f}", flush=True)
            print(line("best " + json.dumps(best), obj0, per0, a.train))
            json.dump(dict(settings=best, objective=obj0, walks=per0), open(out / "best.json", "w"), indent=1)


if __name__ == "__main__":
    main()
