import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.recording import COLS, filled_stretches, load_recording  # noqa: E402


def _noisy(n, fs=400.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / fs
    X = np.column_stack([rng.normal(0, 0.01, (n, 3)), rng.normal(0, 0.05, (n, 3)) + [0, 0, 9.8]])
    return t, X


def test_old_recording_gets_its_filled_holes_back(tmp_path):
    t, X = _noisy(4000)
    a, b = 1000, 1400                         # 1 s that the old recorder filled with a straight line
    X[a:b + 1] = np.linspace(X[a], X[b], b - a + 1)
    pd.DataFrame(np.column_stack([t, X]), columns=COLS).to_csv(tmp_path / "old.csv", index=False)
    assert filled_stretches(X)                # the straight stretch is detectable
    df, info = load_recording(tmp_path / "old.csv")
    assert info["format"].startswith("laptop (old")
    assert info["n_gaps"] == 1 and info["gap_time"] == pytest.approx(1.0, abs=0.01)
    assert len(df) == 4000 - (b - a - 1)      # the invented samples are gone
    assert df.gap.iloc[a + 1] == pytest.approx(1.0, abs=0.01)


def test_clean_old_recording_has_no_gaps(tmp_path):
    t, X = _noisy(2000, seed=1)
    pd.DataFrame(np.column_stack([t, X]), columns=COLS).to_csv(tmp_path / "clean.csv", index=False)
    df, info = load_recording(tmp_path / "clean.csv")
    assert info["n_gaps"] == 0 and len(df) == 2000


def test_phone_recording_is_aligned(tmp_path):
    t, X = _noisy(2000, seed=2)
    lines = ["# IMU Stream recording v1; device=test; accel_range=78.4; gyro_kind=uncalibrated", "type,t_ns,x,y,z"]
    for k in range(len(t)):
        if 600 <= k < 800:                    # 0.5 s missing even on the phone (e.g. sensor hiccup)
            continue
        ns = 10**12 + int(t[k] * 1e9)
        lines.append(f"1,{ns},{X[k, 3]},{X[k, 4]},{X[k, 5]}")
        lines.append(f"2,{ns + 700_000},{X[k, 0]},{X[k, 1]},{X[k, 2]}")
    (tmp_path / "phone.csv").write_text("\n".join(lines) + "\n")
    df, info = load_recording(tmp_path / "phone.csv")
    assert info["format"] == "phone" and info["meta"]["device"] == "test"
    assert list(df.columns) == COLS + ["gap"]
    assert info["n_gaps"] == 1 and info["gap_time"] == pytest.approx(0.5, abs=0.01)
    assert np.median(np.diff(df.t)) == pytest.approx(1 / 400, rel=1e-6)
