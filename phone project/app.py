"""Phone IMU viewer - desktop app.
Receives accelerometer / gyroscope / magnetometer from the phone (Sensor Logger app, HTTP push), shows the raw IMU
live, and runs the project's fusion filter (filter/fusion.c: Madgwick tilt + EKF + ZUPT + magnetometer gating).
Run:  python app.py"""
import math, os, queue, subprocess, sys, time, tkinter as tk
from tkinter import filedialog, messagebox, ttk
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from engine import Engine, MAPPINGS
from receiver import Receiver, local_ips

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 8000
REFRESH_MS = 80
PARAMS = {"mag": 1, "zupt": 1}          # filter settings; see filter/fusion_dll.c fl_set() for every key


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Phone IMU Viewer")
        self.geometry("1280x820")
        self.q = queue.Queue()
        self.rx = None
        self.sim = None
        self.engine = Engine(PARAMS)
        self._build()
        self.after(REFRESH_MS, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.toggle_server()

    # ---------------------------------------------------------------- layout
    def _build(self):
        top = ttk.Frame(self, padding=6); top.pack(fill="x")
        self.btn_srv = ttk.Button(top, text="Stop receiver", command=self.toggle_server); self.btn_srv.pack(side="left")
        ttk.Label(top, text="  Port").pack(side="left")
        self.port_var = tk.StringVar(value=str(PORT)); ttk.Entry(top, textvariable=self.port_var, width=6).pack(side="left")
        ttk.Label(top, text="  Phone orientation").pack(side="left")
        self.map_var = tk.StringVar(value=next(iter(MAPPINGS)))
        cb = ttk.Combobox(top, textvariable=self.map_var, values=list(MAPPINGS), state="readonly", width=38)
        cb.pack(side="left"); cb.bind("<<ComboboxSelected>>", lambda e: self.engine.set_mapping(self.map_var.get()))
        ttk.Button(top, text="Reset filter", command=self.reset).pack(side="left", padx=6)
        ttk.Button(top, text="Demo (simulated phone)", command=self.toggle_demo).pack(side="left")
        ttk.Button(top, text="Save CSV...", command=self.save).pack(side="left", padx=6)
        self.freeze = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Pause plots", variable=self.freeze).pack(side="left")

        self.info = tk.StringVar(); self.status = tk.StringVar()
        ttk.Label(self, textvariable=self.info, padding=(8, 0), font=("Segoe UI", 10, "bold")).pack(anchor="w")
        ttk.Label(self, textvariable=self.status, padding=(8, 0, 8, 4), foreground="#555").pack(anchor="w")

        nb = ttk.Notebook(self); nb.pack(fill="both", expand=True)
        # ---- raw IMU tab
        self.f1 = Figure(figsize=(10, 7), tight_layout=True)
        self.ax1 = [self.f1.add_subplot(4, 1, i + 1) for i in range(4)]
        specs = [("Accelerometer (m/s^2)", ("ax", "ay", "az")), ("Gyroscope (rad/s)", ("gx", "gy", "gz")),
                 ("Magnetometer (uT)", ("mx", "my", "mz")), ("|accel| (m/s^2)", ("anorm",))]
        self.lines1 = []
        for ax, (title, keys) in zip(self.ax1, specs):
            ax.set_title(title, loc="left", fontsize=9); ax.grid(alpha=.3)
            ls = [ax.plot([], [], lw=1, label=k[-1] if len(keys) > 1 else "")[0] for k in keys]
            if len(keys) > 1: ax.legend(loc="upper right", ncol=3, fontsize=7)
            self.lines1.append((ax, keys, ls))
        self.ax1[-1].set_xlabel("time since start (s)")
        c1 = FigureCanvasTkAgg(self.f1, nb); nb.add(c1.get_tk_widget(), text="Raw IMU"); self.c1 = c1

        # ---- filter tab
        self.f2 = Figure(figsize=(10, 7), tight_layout=True)
        gs = self.f2.add_gridspec(3, 2)
        self.axT = self.f2.add_subplot(gs[:, 0]); self.axT.set_aspect("equal", adjustable="datalim")
        self.axT.set_title("Filtered track (metres) - IMU-only dead reckoning drifts", loc="left", fontsize=9)
        self.axT.grid(alpha=.3); (self.lnT,) = self.axT.plot([], [], lw=1.2); (self.dotT,) = self.axT.plot([], [], "ro")
        self.axH = self.f2.add_subplot(gs[0, 1]); self.axH.set_title("Heading (deg)", loc="left", fontsize=9)
        self.axA = self.f2.add_subplot(gs[1, 1]); self.axA.set_title("Roll / pitch (deg)", loc="left", fontsize=9)
        self.axV = self.f2.add_subplot(gs[2, 1]); self.axV.set_title("Speed estimate (m/s)", loc="left", fontsize=9)
        for a in (self.axH, self.axA, self.axV): a.grid(alpha=.3)
        (self.lnH,) = self.axH.plot([], [], lw=1)
        (self.lnR,) = self.axA.plot([], [], lw=1, label="roll"); (self.lnP,) = self.axA.plot([], [], lw=1, label="pitch")
        self.axA.legend(loc="upper right", fontsize=7, ncol=2)
        (self.lnV,) = self.axV.plot([], [], lw=1)
        c2 = FigureCanvasTkAgg(self.f2, nb); nb.add(c2.get_tk_widget(), text="Filtered (fusion.c)"); self.c2 = c2
        self.nb = nb

    # ---------------------------------------------------------------- actions
    def toggle_server(self):
        if self.rx:
            self.rx.stop(); self.rx = None; self.btn_srv.config(text="Start receiver"); return
        try:
            self.rx = Receiver(self.q, int(self.port_var.get())); self.rx.start()
            self.btn_srv.config(text="Stop receiver")
        except OSError as e:
            self.rx = None; messagebox.showerror("Receiver", f"Cannot listen on port {self.port_var.get()}: {e}")

    def reset(self):
        self.engine.reset()
        while not self.q.empty(): self.q.get_nowait()

    def toggle_demo(self):
        if self.sim and self.sim.poll() is None:
            self.sim.terminate(); self.sim = None; return
        if not self.rx:
            self.toggle_server()
        self.reset()
        self.sim = subprocess.Popen([sys.executable, os.path.join(HERE, "simulate_phone.py"), self.port_var.get(), "600"],
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def save(self):
        if not self.engine.log:
            messagebox.showinfo("Save", "No data yet."); return
        p = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="phone_imu.csv")
        if p: self.engine.save_csv(p)

    def _close(self):
        if self.sim and self.sim.poll() is None: self.sim.terminate()
        if self.rx: self.rx.stop()
        self.destroy()

    # ---------------------------------------------------------------- update loop
    def _tick(self):
        t_end = time.time() + 0.05           # never block the UI for more than ~50 ms
        while time.time() < t_end:
            try:
                self.engine.feed(*self.q.get_nowait())
            except queue.Empty:
                break
        self._header()
        if not self.freeze.get() and self.engine.buf["t"]:
            self._draw_current_tab()
        self.after(REFRESH_MS, self._tick)

    def _header(self):
        e = self.engine; o = e.last
        ips = ", ".join(f"http://{ip}:{self.port_var.get()}/data" for ip in local_ips()) or "(no network found)"
        live = self.rx and time.time() - self.rx.last_rx < 2
        self.info.set(f"Phone URL: {ips}      Stream: {'LIVE' if live else 'no data'}      Filter: {e.state}"
                      + (f"   WARNING: {e.warn}" if e.warn else ""))
        if o:
            self.status.set(
                f"{e.rate:5.1f} Hz | heading {math.degrees(o['yaw']):7.1f} deg | roll {math.degrees(o['roll']):6.1f} pitch {math.degrees(o['pitch']):6.1f} "
                f"| speed {o['v']:5.2f} m/s | pos ({o['x']:.1f}, {o['y']:.1f}) m | gyro bias {o['bg']:+.4f} rad/s | "
                f"stationary {'yes' if o['still'] else 'no'} | mag ok/rej {o['mag_ok']:.0f}/{o['mag_rej']:.0f} | ZUPT ok/rej {o['zupt_ok']:.0f}/{o['zupt_rej']:.0f}")
        else:
            self.status.set("Waiting for samples... (point the phone app at the URL above, or press Demo)")

    def _draw_current_tab(self):
        b = self.engine.buf
        t = np.fromiter(b["t"], float)
        if self.nb.index(self.nb.select()) == 0:
            for ax, keys, ls in self.lines1:
                for k, l in zip(keys, ls):
                    l.set_data(t, np.fromiter(b[k], float))
                ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 5)); ax.relim(); ax.autoscale_view(scalex=False)
            self.c1.draw_idle()
        else:
            xs, ys = self.engine.track
            self.lnT.set_data(xs, ys); self.dotT.set_data(xs[-1:], ys[-1:])
            self.axT.relim(); self.axT.autoscale_view()
            for ln, k, ax in ((self.lnH, "yaw", self.axH), (self.lnR, "roll", self.axA),
                              (self.lnP, "pitch", self.axA), (self.lnV, "v", self.axV)):
                ln.set_data(t, np.fromiter(b[k], float))
                ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 5)); ax.relim(); ax.autoscale_view(scalex=False)
            self.c2.draw_idle()


if __name__ == "__main__":
    App().mainloop()
