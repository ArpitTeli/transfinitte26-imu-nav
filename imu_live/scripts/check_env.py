"""Check that the environment can run the pipeline. Run: python scripts/check_env.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    for mod in ("numpy", "scipy", "pandas", "matplotlib"):
        m = __import__(mod)
        print(f"{mod:11s} {m.__version__}")
    from imunav.metrics import position_metrics
    from imunav.pipeline import Config, run_sequence
    from imunav.synth import make_walk

    seq = make_walk(seed=0)
    r = run_sequence(seq, Config(mode="ins"))
    m = position_metrics(r.t, r.p, seq.gt_p)
    print(f"smoke test: final error {m['final_error_m']:.2f} m, drift {m['drift_pct']:.2f} %")
    print("environment OK")


if __name__ == "__main__":
    main()
