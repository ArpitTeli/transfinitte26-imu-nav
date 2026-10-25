"""Load a recording into one table: t, gx, gy, gz, ax, ay, az, gap (s of data missing before the row), and
mx, my, mz: the newest raw magnetometer reading at that row (uT; NaN when there is none).

Three formats are accepted:
  * laptop recording, new:  t,gx,gy,gz,ax,ay,az,gap[,mx,my,mz]   (written by Receiver)
  * laptop recording, old:  t,gx,gy,gz,ax,ay,az       (before gap marking: lost packets were filled with
                            straight lines; those stretches are found and turned back into gaps)
  * phone recording:        '#' header lines, then type,t_ns,x,y,z  (raw events saved by IMU Stream itself;
                            nothing is lost to Wi-Fi; type 3 = magnetometer). Aligned onto the same grid as the
                            live path, each row with the newest magnetometer reading, as live.
"""
import os

import numpy as np
import pandas as pd

from .align import StreamAligner

COLS = ["t", "gx", "gy", "gz", "ax", "ay", "az"]
MAG_COLS = ["mx", "my", "mz"]
MAG_STALE = 0.25          # s, as live (receiver.py)


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
    is_mag = ev[:, 0] == 3
    mag = ev[is_mag]
    mag = mag[np.argsort(mag[:, 1], kind="stable")]
    al = StreamAligner(fs, max_gap)
    rows = []
    for typ, t_ns, x, y, z in ev[~is_mag]:
        al.push(int(typ), t_ns * 1e-9, (x, y, z))
    for t, g, a, gap in al.pop_ready_gaps():
        rows.append((t, *g, *a, gap))
    out = pd.DataFrame(rows, columns=COLS + ["gap"])
    out[MAG_COLS] = np.nan
    if len(mag) and len(out):
        tm = mag[:, 1] * 1e-9
        t = out.t.to_numpy()
        i = np.searchsorted(tm, t, side="right") - 1
        ok = (i >= 0) & (t - tm[np.clip(i, 0, None)] < MAG_STALE)
        out.loc[ok, MAG_COLS] = mag[i[ok], 2:5]
    out.attrs["meta"] = meta
    return out


def load_recording(path, fs=400.0, max_gap=0.05):
    """Returns (DataFrame with COLS + gap, info dict). A file that is not a recording raises ValueError saying so."""
    name = os.path.basename(path)
    with open(path) as f:
        first = f.readline()
    if not first.strip():
        raise ValueError(f"{name} is empty.")
    if first.startswith("#") or first.strip().startswith("type"):
        try:
            df, fmt = _from_phone(path, fs, max_gap), "phone"
        except (ValueError, IndexError, pd.errors.ParserError) as e:
            raise ValueError(f"{name} starts like an IMU Stream recording but cannot be read ({e}).") from e
    else:
        df = pd.read_csv(path)
        missing = [c for c in COLS if c not in df.columns]
        if missing:
            raise ValueError(f"{name} is not an IMU recording. It needs the columns {', '.join(COLS)} (IMU Live) or "
                             f"type,t_ns,x,y,z (IMU Stream); its columns are {', '.join(map(str, df.columns[:8]))}.")
        if "gap" in df.columns:
            df, fmt = df[COLS + ["gap"] + [c for c in MAG_COLS if c in df.columns]].copy(), "laptop"
        else:
            df, fmt = _from_old(df), "laptop (old, gaps recovered)"
        for c in MAG_COLS:
            if c not in df.columns:
                df[c] = np.nan
    g = df.gap.to_numpy()
    info = dict(format=fmt, has_mag=bool(np.isfinite(df[MAG_COLS].to_numpy()).any()), duration=float(df.t.iloc[-1] - df.t.iloc[0]) if len(df) else 0.0,
                n_gaps=int((g > 0).sum()), gap_time=float(g.sum()), longest_gap=float(g.max()) if len(g) else 0.0,
                meta=df.attrs.get("meta", {}))
    return df, info
