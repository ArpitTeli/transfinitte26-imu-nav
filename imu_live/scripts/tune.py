"""Grid-search Config fields on TUNING sequences only.

Examples
  python scripts/tune.py --synthetic-seeds 1 2 3 --mode ins \
      --grid "zupt_res_thr=0.2,0.3,0.4" "gate_prob=0.99,0.999,0.9999"
  python scripts/tune.py --euroc data/euroc/MH_01_easy data/euroc/MH_02_easy --mode ins --grid ...

The objective is the mean position RMSE over the tuning sequences. Held-out
sequences are never loaded here; evaluate them with run_compare.py afterwards.
"""
import argparse
import itertools
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.io import load_euroc  # noqa: E402
from imunav.metrics import position_metrics  # noqa: E402
from imunav.pipeline import PRESETS, Config, run_sequence  # noqa: E402
from imunav.synth import make_foot_walk, make_walk  # noqa: E402


def parse_grid(items):
    grid = {}
    for it in items:
        k, v = it.split("=")
        grid[k] = [type(getattr(Config(), k))(x) if not isinstance(getattr(Config(), k), bool) else x == "True"
                   for x in v.split(",")]
    return grid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="ins")
    ap.add_argument("--synthetic-seeds", type=int, nargs="*", default=[])
    ap.add_argument("--foot-seeds", type=int, nargs="*", default=[], help="shoe-mounted synthetic seeds")
    ap.add_argument("--preset", help="start from a named preset in imunav.pipeline.PRESETS")
    ap.add_argument("--euroc", nargs="*", default=[])
    ap.add_argument("--grid", nargs="+", required=True)
    ap.add_argument("--out", default="results/tuning.csv")
    a = ap.parse_args()

    seqs = ([make_walk(seed=s) for s in a.synthetic_seeds] + [make_foot_walk(seed=s) for s in a.foot_seeds]
            + [load_euroc(d) for d in a.euroc])
    grid = parse_grid(a.grid)
    keys = list(grid)
    rows = []
    for combo in itertools.product(*grid.values()):
        base = PRESETS[a.preset] if a.preset else Config(mode=a.mode)
        cfg = base.with_(**dict(zip(keys, combo)))
        ms = [position_metrics(r.t, r.p, s.gt_p) for s in seqs for r in [run_sequence(s, cfg)]]
        rows.append({**dict(zip(keys, combo)),
                     "rmse_m": np.mean([m["rmse_m"] for m in ms]),
                     "final_m": np.mean([m["final_error_m"] for m in ms]),
                     "drift_pct": np.mean([m["drift_pct"] for m in ms])})
        print(rows[-1], flush=True)
    df = pd.DataFrame(rows).sort_values("rmse_m")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(a.out, index=False)
    print("\nbest:\n", df.head(5).to_string(index=False))


if __name__ == "__main__":
    main()
