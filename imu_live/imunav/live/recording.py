"""Load a recording into one table: t, gx, gy, gz, ax, ay, az, gap (s of data missing before the row).

Three formats are accepted:
  * laptop recording, new:  t,gx,gy,gz,ax,ay,az,gap   (written by Receiver)
  * laptop recording, old:  t,gx,gy,gz,ax,ay,az       (before gap marking: lost packets were filled with
                            straight lines; those stretches are found and turned back into gaps)
  * phone recording:        '#' header lines, then type,t_ns,x,y,z  (raw events saved by IMU Stream itself;
                            nothing is lost to Wi-Fi). Aligned onto the same grid as the live path.
"""
import numpy as np
import pandas as pd

from .align import StreamAligner

COLS = ["t", "gx", "gy", "gz", "ax", "ay", "az"]


def filled_stretches(X, tol=1e-4, min_len=8):
    """(start, end) row ranges where every channel lies on a straight line: interpolated, not measured.
    Real sensor data at 100+ Hz always has noise, so `min_len` straight samples in a row means a hole."""
    lin = np.r_[False, np.abs(np.diff(X, 2, axis=0)).max(1) < tol, False]
    e = np.flatnonzero(np.diff(lin.astype(int)))
    return [(a, b + 1) for a, b in zip(e[::2], e[1::2]) if b - a >= min_len]


def _from_old(df):
    X = df[COLS[1:]].to_numpy(float)
    t = df.t.to_numpy(float)
    keep = np.ones(len(df), bool)
    gap = np.zeros(len(df))
    for a, b in filled_stretches(X):
        keep[a + 1:b] = False                           # rows a and b are the real samples around the hole
        if b < len(df):
            gap[b] += t[b] - t[a]
    out = df.loc[keep, COLS].copy()
    out["gap"] = gap[keep]
    return out.reset_index(drop=True)


def _from_phone(path, fs, max_gap):
    meta, n_hdr = {}, 0
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            n_hdr += 1
            for kv in line[1:].split(";"):
                if "=" in kv:
                    k, v = kv.split("=", 1); meta[k.strip()] = v.strip()
    ev = pd.read_csv(path, skiprows=n_hdr).to_numpy(float)
    al = StreamAligner(fs, max_gap)
    rows = []
    for typ, t_ns, x, y, z in ev:
        al.push(int(typ), t_ns * 1e-9, (x, y, z))
    for t, g, a, gap in al.pop_ready_gaps():
        rows.append((t, *g, *a, gap))
    out = pd.DataFrame(rows, columns=COLS + ["gap"])
    out.attrs["meta"] = meta
    return out


def load_recording(path, fs=400.0, max_gap=0.05):
    """Returns (DataFrame with COLS + gap, info dict)."""
    with open(path) as f:
        first = f.readline()
    if first.startswith("#") or first.strip().startswith("type"):
        df, fmt = _from_phone(path, fs, max_gap), "phone"
    else:
        df = pd.read_csv(path)
        if "gap" in df.columns:
            df, fmt = df[COLS + ["gap"]].copy(), "laptop"
        else:
            df, fmt = _from_old(df), "laptop (old, gaps recovered)"
    g = df.gap.to_numpy()
    info = dict(format=fmt, duration=float(df.t.iloc[-1] - df.t.iloc[0]) if len(df) else 0.0,
                n_gaps=int((g > 0).sum()), gap_time=float(g.sum()), longest_gap=float(g.max()) if len(g) else 0.0,
                meta=df.attrs.get("meta", {}))
    return df, info
