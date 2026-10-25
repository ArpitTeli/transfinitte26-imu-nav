"""Score court walks against the badminton court's known geometry (the only ground truth we have).

Protocol (docs/data_collection.md): the walker stands still >= 4 s on a start corner, walks laps of the
doubles outline (13.40 x 6.10 m) or one long side there and back, and stands still >= 4 s at the end.
Known truth: every corner position, the line being walked, and the return to the start corner. Corners are
found from the turns in the track, because the walker did not always pause at them.

Run:  python scripts/court_eval.py recordings/imu_20261025_041004.csv results/court_20261025/walk_windows.json
      [--set zupt_gyro_thr=0.4 ...] [--out results/court_20261025/untuned]
walk_windows.json: {"A1": [t_first, t_last], ...} phone-clock times of each walk (from the laptop's per-walk
files, which mark when n was pressed). Outputs: <walk>.png, summary.png, scores.json, printed tables.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.recording import load_recording  # noqa: E402
from imunav.live.session import COURT_L, COURT_W, PROTOCOL  # noqa: E402
from imunav.pipeline import PRESETS, navigator_from_static  # noqa: E402
from imunav.quat import yaw_of  # noqa: E402

SHAPE = {code: shape for code, _, shape in PROTOCOL}
LONG_STILL = 4.0      # s: start/end stills are >= 4 s; corner pauses are 1-3 s
TIER1_S, TIER2_M = 30.0, 50.0


# ---------------------------------------------------------------- data

def load(phone_csv, cache):
    if Path(cache).exists():
        d = np.load(cache)
        return {k: d[k] for k in d.files}
    df, _ = load_recording(phone_csv)                    # the live app's 400 Hz grid
    d = {c: df[c].to_numpy() for c in df.columns}
    np.savez_compressed(cache, **d)
    return d


def still_mask(G, A, k=40):
    """both feet planted: low rotation and gravity-only acceleration over 0.1 s (k samples at 400 Hz)"""
    sm = lambda x: np.convolve(x, np.ones(k) / k, "same")
    gn, an = np.linalg.norm(G, axis=1), np.linalg.norm(A, axis=1)
    return (sm(gn) < 0.2) & (np.abs(sm(an) - 9.81) < 0.6) & (sm(np.abs(an - sm(an))) < 0.4)


def still_runs(t, mask, i0, i1, min_s, merge_s=0.3):
    """[(first, last+1)] sample ranges of stillness >= min_s, merging runs split by a wobble < merge_s"""
    e = np.flatnonzero(np.diff(np.r_[0, mask[i0:i1].astype(int), 0])) + i0
    runs = []
    for a, b in zip(e[::2], e[1::2]):
        if runs and t[a] - t[runs[-1][1] - 1] < merge_s:
            runs[-1][1] = b
        else:
            runs.append([a, b])
    return [(a, b) for a, b in runs if t[b - 1] - t[a] >= min_s]


def segment(t, still, w0, w1):
    """The walk runs from the end of the last long still that begins before n + 30 s (the start corner) to the
    start of the first long still that begins after the next n - 30 s (the finish). Returns sample indices
    (start-still first, walk start, walk end)."""
    i0, i1 = np.searchsorted(t, [w0 - 60, w1 + 30])
    runs = still_runs(t, still, i0, i1, LONG_STILL)
    before = [r for r in runs if t[r[0]] <= w0 + 30]
    start = before[-1]
    after = [r for r in runs if t[r[0]] >= w1 - 30 and r[0] > start[1]]
    return start[0], start[1] - 1, after[0][0]


def run_walk(t, G, A, gap, s0, s1, e, cfg):
    """Calibrate on the stillest 2 s of the start still (as the live app would), then step to the end."""
    win = cfg.init_window
    best = None
    for c0 in range(s0, s1, 40):
        c1 = np.searchsorted(t, t[c0] + win)
        if c1 > s1:
            break
        score = G[c0:c1].std(0).max()
        if best is None or score < best[0]:
            best = (score, c0, c1)
    _, c0, c1 = best
    cfg = cfg.with_(fs=1.0 / float(np.median(np.diff(t[c0:c1]))))
    nav, cal = navigator_from_static(t[c0:c1], G[c0:c1], A[c0:c1], cfg)
    j1 = np.searchsorted(t, t[e] + 1.0)
    n = j1 - c1
    P, Y, S = np.zeros((n, 3)), np.zeros(n), np.zeros(n, bool)
    for m, j in enumerate(range(c1, j1)):
        if gap[j] > 0:
            nav.gap(gap[j])
        else:
            nav.step(G[j], A[j], t[j] - t[j - 1])
        P[m], Y[m], S[m] = nav.p, yaw_of(nav.q), nav.still
    return dict(t=t[c1:j1], p=P, yaw=np.unwrap(Y), still=S, t_walk=t[s1], t_end=t[e], cal=cal,
                cal_std=float(best[0]), rejected=dict(nav.ins.n_rejected))


# ---------------------------------------------------------------- turns and truth

def stances(r):
    """one row per foot-flat period: mid time, position, median yaw"""
    s = r["still"].astype(int)
    e = np.flatnonzero(np.diff(np.r_[0, s, 0]))
    out = []
    for a, b in zip(e[::2], e[1::2]):
        m = (a + b) // 2
        out.append((r["t"][m], *r["p"][m, :2], float(np.median(r["yaw"][a:b]))))
    return np.array(out)


def find_turns(st, size, sign, min_gap=2.5):
    """Corner turns: the yaw change from stance k-1 reaches 60 % of the expected turn within 3 s.
    sign +1 / -1: turns one way only (laps); 0: either way (turning round at the end of a line).
    Returns (stance before the turn, stance after it) index pairs into st."""
    T, Y = st[:, 0], st[:, 3]
    turns, k = [], 1
    while k < len(st) - 1:
        j, best = k, None
        while j < len(st) and T[j] - T[k] <= 3.0:
            dy = Y[j] - Y[k - 1]
            d = sign * dy if sign else abs(dy)
            if d >= 0.6 * size and (best is None or d > best[1]):
                best = (j, d)
            j += 1
        if best is not None and (not turns or T[k] - T[turns[-1][0]] >= min_gap):
            turns.append((k - 1, best[0]))
            k = best[0] + 1
        else:
            k += 1
    return turns


def turn_point(r, st, before, after):
    """sample index half-way through a turn (yaw half-way between the stances either side): the corner's vertex,
    whether the walker stopped on the corner and turned there or rounded it"""
    a, b = np.searchsorted(r["t"], [st[before, 0], st[after, 0]])
    y0, y1 = st[before, 3], st[after, 3]
    frac = (r["yaw"][a:b + 1] - y0) / (y1 - y0)
    return int(a + np.argmax(frac >= 0.5))


class Court:
    """The loop as walked, in a frame with the start corner at the origin and the court in the first quadrant.
    The walk's turn direction and first leg come from the data, not the protocol labels: on 25 Oct every walk
    started from one corner (bottom left with the court lying flat), so left-turning walks went along a long
    side (+x) first and right-turning walks up a short side (+y) first. The world frame is z-up (yaw grows
    anticlockwise), so turn > 0 is a left turn."""

    def __init__(self, turn, first_long):
        self.turn = turn                                   # +1 left turns, -1 right turns, 0 line (turn round)
        a, b = (COURT_L, COURT_W) if first_long else (COURT_W, COURT_L)
        if turn == 0:
            self.loop, self.first_dir = [(COURT_L, 0.0), (0.0, 0.0)], 0.0
        elif turn > 0:
            self.loop, self.first_dir = [(a, 0.0), (a, b), (0.0, b), (0.0, 0.0)], 0.0
        else:
            self.loop, self.first_dir = [(0.0, a), (b, a), (b, 0.0), (0.0, 0.0)], np.pi / 2
        self.loop = [np.array(c) for c in self.loop]
        ext = np.max(self.loop, axis=0)
        self.box = np.array([[0, 0], [ext[0], 0], [ext[0], COURT_W if turn == 0 else ext[1]],
                             [0, COURT_W if turn == 0 else ext[1]], [0, 0]])
        long_x = (self.box[:, 0].max() > self.box[:, 1].max())
        h = self.box[:, :].max(axis=0)
        # the net crosses the middle of both long sides
        self.net = np.array([[h[0] / 2, 0], [h[0] / 2, h[1]]]) if long_x else np.array([[0, h[1] / 2], [h[0], h[1] / 2]])
        self.period = 2 if turn == 0 else 4
        self.size = np.pi if turn == 0 else np.pi / 2

    def truth(self, n):
        """true positions of the first n corners after the start, with the true path length to each"""
        prev, dist, out = np.zeros(2), 0.0, []
        for i in range(n):
            p = self.loop[i % len(self.loop)]
            dist += float(np.linalg.norm(p - prev))
            out.append((p, dist))
            prev = p
        return out

    def align(self, p, align_m=3.0):
        """start at the origin, rotated so the first align_m metres run along the first leg (as the live view)"""
        q = p[:, :2] - p[0, :2]
        d = np.linalg.norm(q, axis=1)
        i = int(np.argmax(d >= align_m))
        a = self.first_dir - np.arctan2(q[i, 1], q[i, 0])
        c, s = np.cos(a), np.sin(a)
        return np.column_stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]])


def court_from_data(r, st, shape):
    """turn direction from the total yaw change; first leg long or short from the distance to the first turn"""
    if shape == "line":
        return Court(0, True)
    turn = 1 if r["yaw"][-1] > r["yaw"][0] else -1
    k = find_turns(st, np.pi / 2, turn)
    first = np.linalg.norm(st[k[0][0], 1:3] - st[0, 1:3]) if k else COURT_L
    return Court(turn, first > (COURT_L + COURT_W) / 2)


def match_corners(q, r, st, turns, court, skip_cost=0.3, end_bit_m=4.0):
    """Choose which detected turns are court corners: the subsequence whose estimated leg lengths best fit the
    court's alternating legs, with whole laps (whole there-and-backs for the line). A stray turn, e.g. at the
    net, splits a leg and gets dropped. The end of the walk is also a candidate, for a walker who stopped on
    the start corner without turning. Returns [(sample index at the corner, true corner, true path length)]."""
    cand = [turn_point(r, st, k0, k1) for k0, k1 in turns]
    if not cand or np.linalg.norm(q[-1] - q[cand[-1]]) > 0.5:
        cand.append(len(q) - 1)
    M = len(cand)
    truth = court.truth(M)
    exp = np.diff(np.r_[0.0, [d for _, d in truth]])
    f, back = np.full((M, M), np.inf), np.full((M, M), -1, int)
    for j in range(M):
        f[0, j] = ((np.linalg.norm(q[cand[j]]) - exp[0]) / exp[0]) ** 2 + skip_cost * j
    for k in range(1, M):
        for j in range(k, M):
            for i in range(k - 1, j):
                if np.isfinite(f[k - 1, i]):
                    L = np.linalg.norm(q[cand[j]] - q[cand[i]])
                    c = f[k - 1, i] + ((L - exp[k]) / exp[k]) ** 2 + skip_cost * (j - i - 1)
                    if c < f[k, j]:
                        f[k, j], back[k, j] = c, i
    best = None
    for K in range(court.period, M + 1, court.period):
        for j in range(K - 1, M):
            if np.isfinite(f[K - 1, j]) and np.linalg.norm(q[-1] - q[cand[j]]) < end_bit_m:
                c = f[K - 1, j] + skip_cost * (M - 1 - j)
                if best is None or c < best[0]:
                    best = (c, K, j)
    if best is None:
        return []
    _, K, j = best
    picked = [j]
    for k in range(K - 1, 0, -1):
        j = back[k, j]
        picked.append(j)
    picked = picked[::-1]
    return [(cand[j], truth[k][0], truth[k][1]) for k, j in enumerate(picked)]


# ---------------------------------------------------------------- scoring

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def seg_dist(p, a, b):
    """distance from points p to the segment a-b"""
    ab = b - a
    u = np.clip(((p - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(p - (a + u[:, None] * ab), axis=1)


def score(r, q, corners):
    t = r["t"] - r["t_walk"]
    i_start = int(np.searchsorted(r["t"], r["t_walk"]))
    idx = np.array([c[0] for c in corners])
    C = np.array([c[1] for c in corners])
    D = np.array([c[2] for c in corners])
    err = np.linalg.norm(q[idx] - C, axis=1)
    # legs: start -> corner 1 -> ...; heading error of each leg (estimated vs true direction), length error
    P0 = np.vstack([q[i_start], q[idx]])
    T0 = np.vstack([np.zeros(2), C])
    v, u = np.diff(P0, axis=0), np.diff(T0, axis=0)
    th = wrap(np.arctan2(v[:, 1], v[:, 0]) - np.arctan2(u[:, 1], u[:, 0]))
    dth = np.diff(np.r_[0.0, th])                     # heading error picked up at the turn + along the leg
    lerr = (np.linalg.norm(v, axis=1) - np.linalg.norm(u, axis=1)) / np.linalg.norm(u, axis=1)
    long_leg = np.isclose(np.linalg.norm(u, axis=1), COURT_L)
    # tier 1: what is known in the first 30 s: the distance to the line being walked, and corner errors
    in30 = (t >= 0) & (t <= TIER1_S)
    leg_of = np.searchsorted(idx, np.arange(len(q)), side="left")
    xt = np.full(len(q), np.nan)
    for k in range(len(C)):
        m = in30 & (leg_of == k)
        if m.any():
            xt[m] = seg_dist(q[m], T0[k], T0[k + 1])
    c30 = err[t[idx] <= TIER1_S]
    x30 = float(np.nanmax(xt)) if np.isfinite(xt).any() else 0.0
    tier1 = max(x30, float(c30.max()) if len(c30) else 0.0)
    walked30 = float(np.linalg.norm(np.diff(q[in30], axis=0), axis=1).sum())
    # tier 2 from the start: first corner >= 50 m of true path
    k50 = int(np.argmax(D >= TIER2_M)) if (D >= TIER2_M).any() else None
    tier2 = None if k50 is None else dict(dist=float(D[k50]), err=float(err[k50]), pct=float(100 * err[k50] / D[k50]))
    # 50 m stretches restarted at every corner: true position, heading from the leg just walked
    stretches = []
    starts = [(i_start, np.zeros(2), 0.0, 0.0)] + [(idx[k], C[k], D[k], -th[k]) for k in range(len(C))]
    for i0, c0, d0, rot in starts:
        m = np.flatnonzero(D >= d0 + TIER2_M)
        if not len(m):
            continue
        m = m[0]
        cr, sr = np.cos(rot), np.sin(rot)
        dv = q[idx[m]] - q[i0]
        est = c0 + np.array([cr * dv[0] - sr * dv[1], sr * dv[0] + cr * dv[1]])
        e = float(np.linalg.norm(est - C[m]))
        stretches.append(dict(dist=float(D[m] - d0), err=e, pct=float(100 * e / (D[m] - d0))))
    return dict(corner_t=t[idx].tolist(), corner_d=D.tolist(), corner_err=err.tolist(),
                leg_heading_err_deg=np.degrees(th).tolist(), leg_heading_change_deg=np.degrees(dth).tolist(),
                leg_len_err_pct=(100 * lerr).tolist(), leg_long=long_leg.tolist(),
                tier1=tier1, tier1_corner=float(c30.max()) if len(c30) else None, tier1_cross=x30, walked30=walked30,
                tier2=tier2, closure=float(err[-1]), closure_pct=float(100 * err[-1] / D[-1]), true_dist=float(D[-1]),
                end_from_start=float(np.linalg.norm(q[-1])), stretches=stretches)


def plot_walk(code, r, q, corners, court, s, path, label):
    t = r["t"] - r["t_walk"]
    idx = [c[0] for c in corners]
    C = np.array([c[1] for c in corners])
    fig, ax = plt.subplots(2, 2, figsize=(14, 9.5), gridspec_kw=dict(width_ratios=[1.35, 1]))
    a = ax[0, 0]
    a.plot(court.box[:, 0], court.box[:, 1], "-", lw=7, color="#d9dee3", solid_capstyle="round", label="court lines")
    a.plot(court.net[:, 0], court.net[:, 1], color="#8d96a0", lw=1.2, ls="--", label="net")
    w = t >= -1
    a.plot(q[w, 0], q[w, 1], lw=1.1, color="tab:blue", label="estimated foot track")
    for k, i in enumerate(idx):
        a.plot([q[i, 0], C[k, 0]], [q[i, 1], C[k, 1]], color="tab:red", lw=0.8, alpha=0.7)
    a.plot(q[idx, 0], q[idx, 1], "o", ms=4, color="tab:red", label="estimate at each corner")
    a.plot(0, 0, "ko", ms=8, label="start")
    a.set(aspect="equal", xlabel="x [m]", ylabel="y [m]", title=f"{code}: track on the court ({label})")
    a.grid(alpha=0.3)
    a.legend(fontsize=7.5, loc="best")
    D, E = np.array(s["corner_d"]), np.array(s["corner_err"])
    a = ax[0, 1]
    a.plot(D, E, "o-", color="tab:red", ms=4, label="error at each corner")
    dd = np.linspace(0, D.max() * 1.02, 50)
    a.plot(dd, 0.03 * dd, "--", color="tab:green", label="3 % of distance (Tier 2 rate)")
    a.axhline(1.0, color="tab:orange", ls=":", label="1 m (Tier 1)")
    a.set(xlabel="true distance walked [m]", ylabel="position error [m]", title="Error vs distance")
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
    a = ax[1, 0]
    h = np.array(s["leg_heading_change_deg"])
    lg = np.array(s["leg_long"])
    k = np.arange(1, len(h) + 1)
    a.bar(k[lg], h[lg], color="tab:purple", label="long side (crosses the net)")
    a.bar(k[~lg], h[~lg], color="tab:gray", label="short side")
    a.plot(k, s["leg_heading_err_deg"], "k.-", lw=0.8, label="heading error so far")
    a.axhline(0, color="k", lw=0.5)
    a.set(xlabel="leg number", ylabel="degrees", title="Heading error picked up on each leg")
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
    a = ax[1, 1]
    a.plot(s["corner_t"], E, "o-", color="tab:red", ms=4)
    a.axvline(TIER1_S, color="tab:orange", ls=":", label="first 30 s")
    a.set(xlabel="time since setting off [s]", ylabel="position error at corner [m]", title="Error vs time")
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_tracks(tracks, results, out, label):
    """every walk's track on its court, one panel each"""
    n = len(tracks)
    cols = 4
    rows = (n + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(4.6 * cols, 3.6 * rows))
    for a in axs.flat[n:]:
        a.axis("off")
    for a, (code, (q, court, idx)) in zip(axs.flat, tracks.items()):
        s = results[code]
        a.plot(court.box[:, 0], court.box[:, 1], "-", lw=5, color="#d9dee3", solid_capstyle="round")
        a.plot(court.net[:, 0], court.net[:, 1], color="#8d96a0", lw=1, ls="--")
        a.plot(q[:, 0], q[:, 1], lw=0.8, color="tab:blue")
        a.plot(q[idx, 0], q[idx, 1], "o", ms=2.5, color="tab:red")
        a.plot(0, 0, "ko", ms=5)
        a.set(aspect="equal", title=f"{code}: {s['walked'].split(' (')[0]}, {s['laps']:g} {'lengths' if s['shape'] == 'line' else 'laps'}\n"
                                     f"return to start {s['closure']:.1f} m, worst corner {max(s['corner_err']):.1f} m")
        a.title.set_fontsize(9)
        a.tick_params(labelsize=7)
        a.grid(alpha=0.3)
    fig.suptitle(f"All walks on the court ({label}); dashed line = net, red dots = estimate at each corner", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "tracks.png", dpi=110)
    plt.close(fig)


def report(results, out, label):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for code, s in results.items():
        ax.plot(s["corner_d"], s["corner_err"], "o-", ms=3, lw=1.2, label=code)
    dmax = max(max(s["corner_d"]) for s in results.values())
    ax.plot([0, dmax], [0, 0.03 * dmax], "k--", lw=1, label="3 % of distance")
    ax.set(xlabel="true distance walked [m]", ylabel="position error at corners [m]",
           title=f"Error at every court corner, all walks ({label})")
    ax.grid(alpha=0.3)
    ax.legend(ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "summary.png", dpi=110)
    plt.close(fig)

    print(f"\n{'walk':5s} {'laps':>4s} {'walk s':>6s} {'true m':>6s} {'est m':>6s} {'Tier1 m':>8s} {'(line/corner)':>13s} "
          f"{'Tier2 from start':>17s} {'return to start':>17s} {'worst corner':>13s}")
    for code, s in results.items():
        t2 = s["tier2"]
        t2s = f"{t2['pct']:5.1f} % @{t2['dist']:.0f} m" if t2 else "-"
        print(f"{code:5s} {s['laps']:4g} {s['walk_s']:6.0f} {s['true_dist']:6.0f} {s['est_dist']:6.0f} {s['tier1']:8.2f} "
              f"{s['tier1_cross']:6.2f}/{s['tier1_corner'] or 0:5.2f}  {t2s:>17s} "
              f"{s['closure']:6.2f} m {s['closure_pct']:5.1f} % {max(s['corner_err']):10.2f} m")
    allst = np.array([x["pct"] for s in results.values() for x in s["stretches"]])
    print(f"\n50 m stretches restarted at every corner (true position, heading from the leg just walked): {len(allst)} stretches, "
          f"median {np.median(allst):.1f} %, 90th pct {np.percentile(allst, 90):.1f} %, worst {allst.max():.1f} %, "
          f"within 3 %: {100 * np.mean(allst <= 3):.0f} %")
    for code, s in results.items():
        x = np.array([y["pct"] for y in s["stretches"]])
        print(f"   {code}: {len(x)} stretches, median {np.median(x):5.1f} %, worst {x.max():5.1f} %, within 3 %: {100 * np.mean(x <= 3):3.0f} %")
    lap = [s for s in results.values() if s["shape"] != "line"]
    lc = np.concatenate([np.array(s["leg_heading_change_deg"])[np.array(s["leg_long"])] for s in lap])
    sc = np.concatenate([np.array(s["leg_heading_change_deg"])[~np.array(s["leg_long"])] for s in lap])
    ll = np.concatenate([np.array(s["leg_len_err_pct"])[np.array(s["leg_long"])] for s in lap])
    sl = np.concatenate([np.array(s["leg_len_err_pct"])[~np.array(s["leg_long"])] for s in lap])
    print("\nNet effect, lap walks, per leg (the turn at its start plus the leg itself):")
    print(f"   long sides (cross the net): {len(lc)} legs, heading error picked up: mean |{np.mean(np.abs(lc)):.1f}| deg, "
          f"90th pct {np.percentile(np.abs(lc), 90):.1f} deg; length error mean |{np.mean(np.abs(ll)):.1f}| %")
    print(f"   short sides (no net):       {len(sc)} legs, heading error picked up: mean |{np.mean(np.abs(sc)):.1f}| deg, "
          f"90th pct {np.percentile(np.abs(sc), 90):.1f} deg; length error mean |{np.mean(np.abs(sl)):.1f}| %")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("phone_csv")
    ap.add_argument("windows")
    ap.add_argument("--set", nargs="*", default=[], help="override filter settings, e.g. zupt_gyro_thr=0.4")
    ap.add_argument("--out", default="results/court_20261025/untuned")
    ap.add_argument("--label", default="untuned filter")
    ap.add_argument("--walks", nargs="*", help="only these walks")
    a = ap.parse_args()
    cfg = PRESETS["shoe"]
    for kv in a.set:
        k, v = kv.split("=")
        cfg = cfg.with_(**{k: type(getattr(cfg, k))(v)})
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    d = load(a.phone_csv, Path(a.phone_csv).with_suffix(".400hz.npz"))
    t, gap = d["t"], d["gap"]
    G = np.column_stack([d["gx"], d["gy"], d["gz"]])
    A = np.column_stack([d["ax"], d["ay"], d["az"]])
    still = still_mask(G, A)
    windows = json.load(open(a.windows))
    results, tracks = {}, {}
    for code, (w0, w1) in windows.items():
        if a.walks and code not in a.walks:
            continue
        shape = SHAPE[code]
        s0, s1, e = segment(t, still, w0, w1)
        r = run_walk(t, G, A, gap, s0, s1, e, cfg)
        st = stances(r)
        court = court_from_data(r, st, shape)
        turns = find_turns(st, court.size, court.turn)
        q = court.align(r["p"])
        corners = match_corners(q, r, st, turns, court)
        s = score(r, q, corners)
        n = len(corners)
        walked = {1: "anticlockwise (left turns)", -1: "clockwise (right turns)", 0: "there and back"}[court.turn]
        s.update(walk=code, shape=shape, walked=walked, first_leg_m=float(np.linalg.norm(court.loop[0])),
                 laps=(n / 4 if shape != "line" else n), turns_found=len(turns), corners=n,
                 walk_s=float(r["t_end"] - r["t_walk"]), est_dist=float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()),
                 start_still_s=float(t[s1] - t[s0]), cal_gyro_std=r["cal_std"], rejected=r["rejected"])
        results[code] = s
        tracks[code] = (q, court, [c[0] for c in corners])
        plot_walk(code, r, q, corners, court, s, out / f"{code}.png", a.label)
        np.savez_compressed(out / f"{code}_track.npz", t=r["t"], q=q, still=r["still"], yaw=r["yaw"],
                            corners=np.array([c[0] for c in corners]), t_walk=r["t_walk"], t_end=r["t_end"])
        print(f"{code}: walked {walked}, first leg {s['first_leg_m']:.1f} m, {len(turns)} turns found, "
              f"{n} matched to corners", flush=True)
    json.dump(dict(settings=a.set, walks=results), open(out / "scores.json", "w"), indent=1)
    plot_tracks(tracks, results, out, a.label)
    report(results, out, a.label)


if __name__ == "__main__":
    main()
