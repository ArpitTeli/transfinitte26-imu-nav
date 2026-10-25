"""Live dashboard: receive the phone's IMU stream over Wi-Fi and draw the path as you walk.

Run:  python scripts/live.py            (then press Start in the phone app with this laptop's address)
      python scripts/live.py --start B1 (continue the court protocol from walk B1)
Keys: n = next protocol walk (new file; press it before the walker presses Start on the phone)
      m = marker (time + note into the walk's _markers.csv)
      r = recalibrate (hold the shoe still)   c = set current position as origin   q = quit
Every walk is recorded to its own file in results/live/ (installed app: Documents/IMU Live), named
<date_time>_<walk>.csv; the phone also keeps its own complete copy (Download/IMU Stream).
Windows will ask to allow Python through the firewall the first time: allow it on Private networks.
"""
import argparse
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np
# literal backend names: PyInstaller bundles the backends it sees passed to matplotlib.use()
if "--headless" in sys.argv:
    matplotlib.use("Agg")
else:
    matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.receiver import Receiver, local_ip  # noqa: E402
from imunav.live.session import ALIGN_M, COURT_L, LAP_M, PROTOCOL, Session, heading_up  # noqa: E402
from imunav.live.tracker import TRACKING, LiveTracker  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402


FROZEN = getattr(sys, "frozen", False)  # running as the packaged IMU Live.exe


def default_out_dir():
    """Installed app: Documents/IMU Live (Program Files is not writable). Source tree: results/live."""
    if FROZEN:
        return Path.home() / "Documents" / "IMU Live"
    return Path("results/live")


def show_error(title, msg):
    if FROZEN:
        import tkinter
        from tkinter import messagebox
        root = tkinter.Tk()
        root.withdraw()
        messagebox.showerror(title, msg)
        try:
            root.destroy()
        except tkinter.TclError:              # already gone; destroying it again raised a second error dialog
            pass
    else:
        print(f"{title}: {msg}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fs", type=float, default=400.0, help="resampling rate for the filter (Hz)")
    ap.add_argument("--preset", default="shoe", choices=sorted(PRESETS))
    ap.add_argument("--start", choices=[p[0] for p in PROTOCOL], help="first protocol walk for the n key")
    ap.add_argument("--headless", type=float, metavar="SECONDS",
                    help="no window: run this long, then save the dashboard to <out>/dashboard.png "
                         "(with --start, that walk begins immediately)")
    ap.add_argument("--out", type=Path, default=default_out_dir(), help="folder for recordings")
    a = ap.parse_args()

    tracker = LiveTracker(PRESETS[a.preset])
    try:
        rx = Receiver(tracker, a.port, a.fs)
    except OSError as e:
        show_error("IMU Live", f"Cannot listen on UDP port {a.port} ({e}).\n"
                               "Is IMU Live already running? Close it, or start with --port <other>.")
        return
    session = Session(rx, tracker, a.out, start=a.start)
    rx.start()
    ip = local_ip()
    print(f"Listening on {ip}:{a.port}. Enter this address in the phone app.")
    print(f"Recording to {a.out.resolve()}  (press n to start walk {session.next_code})")

    fig = plt.figure(figsize=(10, 9))
    fig.canvas.manager.set_window_title("IMU live tracking")
    ax = fig.add_axes([0.08, 0.11, 0.88, 0.67])
    path_line, = ax.plot([], [], "-", lw=2, color="tab:blue", zorder=2, label="foot track")
    start_dot, = ax.plot([], [], "o", ms=9, color="k", zorder=3, label="start")
    now_dot, = ax.plot([], [], "o", ms=12, color="tab:gray", zorder=4,
                       label="phone (grey = calibrating, green = foot still)")
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
    ax.grid(alpha=0.3)
    fig.legend(loc="lower center", bbox_to_anchor=(0.5, 0.005), ncol=3, fontsize=8.5, frameon=False)  # off the track
    title = fig.text(0.02, 0.975, "", va="top", fontsize=15, weight="bold")
    state = fig.text(0.02, 0.935, "", va="top", fontsize=11)
    link = fig.text(0.02, 0.900, "", va="top", fontsize=11, family="monospace", weight="bold")
    status = fig.text(0.02, 0.865, "", va="top", family="monospace", fontsize=9.5)

    def update(_):
        s = tracker.snapshot()
        p = s["p"]
        code, desc, shape = session.walk
        aligned = heading_up(p) if len(p) else None
        if aligned is not None:
            q = aligned
        elif len(p):
            q = p[:, :2] - p[0, :2]                # under ALIGN_M walked: not turned to point up yet
        else:
            q = np.zeros((1, 2))                   # calibrating: the phone waits at the start
        path_line.set_data(q[:, 0], q[:, 1])
        start_dot.set_data([0], [0])
        now_dot.set_data([q[-1, 0]], [q[-1, 1]])
        now_dot.set_color("tab:gray" if s["state"] != TRACKING else "tab:green" if s["still"] else "tab:orange")
        pts = q
        lo, hi = pts.min(0), pts.max(0)
        cx, cy = (lo + hi) / 2
        half = max(3.0, 0.55 * max(hi - lo) + 1.0)
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

        title.set_text(f"WALK {code}: {desc}")
        if s["state"] == TRACKING:
            note = "" if aligned is not None else f"   (the track turns to point up after {ALIGN_M:.0f} m)"
            state.set_text(f"TRACKING  {s['message']}{note}")
            state.set_color("#1b5e20")
        else:
            gl, al = s["still_limits"]
            m = s["motion"]
            if m is None:
                how = "waiting for data"
            else:
                how = f"shoe motion {m[0]:.3f} rad/s (needs < {gl:.2f})"
                if m[1] >= al:
                    how += f", accel {m[1]:.2f} (needs < {al:.1f})"
                elif m[0] < gl:
                    how += "  holding still..."
            state.set_text(f"CALIBRATING  stand still for 2 s:  {how}")
            state.set_color("#b26a00")
        age = time.monotonic() - rx.last_rx if rx.last_rx else None
        if age is None:
            link.set_text(f"LINK: waiting for the phone   (enter {ip}:{a.port} in IMU Stream)"); link.set_color("#b26a00")
        elif age < 1.0:
            link.set_text(f"LINK OK   {rx.acc_rate:4.0f} Hz   lost packets {rx.lost}   data gaps {s['gaps']} "
                          f"({s['gap_time']:.1f} s)"); link.set_color("#1b5e20")
        else:
            link.set_text(f"LINK LOST for {age:.0f} s   (normal between walks while the phone is stopped; "
                          "mid-walk, the phone keeps its own copy)"); link.set_color("#b3263a")
        laps = (f"   laps ~{s['distance'] / LAP_M:4.1f}" if shape in ("cw", "ccw")
                else f"   lengths ~{s['distance'] / COURT_L:4.1f}" if shape == "line" else "")
        lines = [
            f"Steps {s['steps']:4d}   Distance {s['distance']:6.1f} m{laps}   Speed {s['speed']:4.2f} m/s"
            + (f"   Rejected ZUPTs {s['rejected'].get('zupt', 0)}" if s["state"] == TRACKING else ""),
            f"Recording {Path(session.path).name}  ({rx.rec_rows} rows, {session.markers} markers)",
            f"Keys: n = start walk {session.next_code}   m = marker   r = recalibrate   c = reset origin   q = quit",
        ]
        status.set_text("\n".join(lines))
        return path_line, start_dot, now_dot, title, state, link, status

    def on_key(ev):
        if ev.key == "n":
            code, desc, _ = session.next_walk()
            print(f"walk {code} started: {desc} -> {session.path}")
        elif ev.key == "m":
            session.marker()
            print(f"marker {session.markers} at phone time {rx.last_t}")
        elif ev.key == "r":
            tracker.reset()
        elif ev.key == "c":
            tracker.clear()
        elif ev.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    try:
        if a.headless:
            if a.start:
                session.next_walk()
            time.sleep(a.headless)
            update(None)
            out = a.out / "dashboard.png"
            fig.savefig(out, dpi=100)
            print("saved", out)
            return
        anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
        plt.show()
    finally:
        rx.stop()
        session.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # a windowed exe has no console to print to
        import traceback
        show_error("IMU Live crashed", "".join(traceback.format_exception(e))[-1500:])
        if not FROZEN:
            raise
        sys.exit(1)        # re-raising in the packaged app shows a second, raw "unhandled exception" dialog
