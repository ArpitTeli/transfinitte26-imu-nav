"""Fusion Player: replay an NCLT dataset through the IMU / wheel / magnetometer fusion filter and watch it unfold.
Open a dataset (sensor archive + ground-truth CSV); every condition is computed once, then played back at the robot's
own pace (1x) or faster. Run:  python app.py
"""
import os, sys, glob, json, threading
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import engine

pg.setConfigOptions(antialias=False, background="w", foreground="#30363d")
LINE_W, GT_W = 1, 1      # wide pens are slow to draw in Qt's software renderer
GT_PEN = pg.mkPen((15, 15, 15), width=GT_W)
GT_FULL_PEN = pg.mkPen((190, 195, 200), width=1)
COLORS = ["#7d6fb0", "#e8a33d", "#2f8fa3", "#d1495b", "#4c9a5f", "#8a5a44"]
SPEEDS = [0.5, 1, 2, 5, 10, 30, 60, 120, 300, 600]
WINDOWS = [("Last 2 min", 2), ("Last 10 min", 10), ("Last 30 min", 30), ("Whole run", None)]
MAX_TRAJ_PTS = 3000
SERIES_BUCKETS = 1500                          # time series: min/max envelope over this many buckets


def dec(a, i):
    """first i samples, thinned to <= MAX_TRAJ_PTS, always ending on sample i-1"""
    if i <= 0: return a[:0]
    s = max(1, -(-i // MAX_TRAJ_PTS))          # ceil, so the result stays within the budget
    return a[:i:s] if (i - 1) % s == 0 else np.r_[a[:i:s], a[i - 1]]


def visible(x, k, lo, hi):
    """index range [a, b) of the first k samples of sorted x that fall in [lo, hi], plus one sample either side"""
    a = max(0, int(np.searchsorted(x, lo)) - 1); b = min(k, int(np.searchsorted(x, hi, side="right")) + 1)
    return a, max(a, b)


def envelope(x, y, a, b, buckets=SERIES_BUCKETS):
    """x[a:b], y[a:b] reduced to the min and max of each bucket (keeps spikes), in time order"""
    n = b - a
    if n <= 2 * buckets: return x[a:b], y[a:b]
    s = -(-n // buckets); m = (n // s) * s
    xs, ys = x[a:a + m].reshape(-1, s), y[a:a + m].reshape(-1, s); r = np.arange(len(ys))
    i0, i1 = ys.argmin(1), ys.argmax(1); lo, hi = np.minimum(i0, i1), np.maximum(i0, i1)
    X = np.column_stack([xs[r, lo], xs[r, hi]]).ravel(); Y = np.column_stack([ys[r, lo], ys[r, hi]]).ravel()
    return (np.r_[X, x[a + m:b]], np.r_[Y, y[a + m:b]]) if m < n else (X, Y)


def wrap_breaks(x, y):
    """angles in degrees wrapped to +-180, with gaps (NaN) where they wrap so no vertical lines are drawn"""
    y = (y + 180) % 360 - 180
    j = np.flatnonzero(np.abs(np.diff(y)) > 180) + 1
    return np.insert(x.astype(float), j, np.nan), np.insert(y, j, np.nan)


def fmt_time(s):
    s = max(0, int(s)); return f"{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}" if s >= 3600 else f"{s // 60}:{s % 60:02d}"


# ---------------------------------------------------------------- background processing
class Worker(QtCore.QObject):
    """Runs engine.process on a daemon thread; the signals arrive queued on the GUI thread.
    A daemon thread (not a QThread) so closing the window mid-processing exits cleanly instead of aborting
    with 'QThread destroyed while running' (engine.process cannot be interrupted)."""
    progress = QtCore.Signal(str)
    done = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, args):
        super().__init__(); self.args = args

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self.done.emit(engine.process(*self.args, cb=self.progress.emit))
        except Exception as e:
            self.failed.emit(str(e))


class OpenDialog(QtWidgets.QDialog):
    def __init__(self, parent, settings_store):
        super().__init__(parent)
        self.setWindowTitle("Open dataset"); self.setMinimumWidth(620); self.store = settings_store
        lay = QtWidgets.QFormLayout(self)
        self.tar = self._file_row(lay, "Sensor archive", "NCLT sensor archive (*.tar.gz *.tgz *.tar *.gz);;All files (*)", "last_tar")
        self.gt = self._file_row(lay, "Ground truth", "Ground-truth CSV (*.csv);;All files (*)", "last_gt")
        self.profile = QtWidgets.QComboBox()
        for p in engine.settings_list():
            with open(p) as fh:
                self.profile.addItem(json.load(fh).get("name", os.path.basename(p)), p)
        lay.addRow("Filter settings", self.profile)
        self.align = QtWidgets.QComboBox()
        self.align.addItem("Fit on the first 60 s of motion", "fit60"); self.align.addItem("True pose when motion starts", "start")
        lay.addRow("Align to ground truth", self.align)
        note = QtWidgets.QLabel("Use the files exactly as downloaded from the NCLT site. The first run of a dataset takes "
                                "10-30 s; after that it opens from the cache.")
        note.setWordWrap(True); note.setStyleSheet("color:#5a6672"); lay.addRow(note)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.button(QtWidgets.QDialogButtonBox.Ok).setText("Process"); bb.accepted.connect(self._accept); bb.rejected.connect(self.reject)
        lay.addRow(bb)

    def _file_row(self, lay, label, filt, key):
        w = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(w); h.setContentsMargins(0, 0, 0, 0)
        e = QtWidgets.QLineEdit(self.store.value(key, "")); b = QtWidgets.QPushButton("Browse…")
        def pick():
            f, _ = QtWidgets.QFileDialog.getOpenFileName(self, label, os.path.dirname(e.text()) or "", filt)
            if f: e.setText(f)
        b.clicked.connect(pick); h.addWidget(e, 1); h.addWidget(b); lay.addRow(label, w); e._key = key
        return e

    def _accept(self):
        for e in (self.tar, self.gt):
            if not os.path.isfile(e.text()):
                QtWidgets.QMessageBox.warning(self, "File not found", f"Choose an existing file for {e._key[5:]}."); return
            self.store.setValue(e._key, e.text())
        self.accept()

    def values(self):
        return self.tar.text(), self.gt.text(), self.profile.currentData(), self.align.currentData()


# ---------------------------------------------------------------- main window
class Player(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fusion Player"); self.resize(1500, 920)
        self.store = QtCore.QSettings("Transfinitte26", "FusionPlayer")
        self.d = None; self.t_play = 0.0; self.playing = False; self.worker = None
        self.clock = QtCore.QElapsedTimer(); self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick)
        self._build()

    # ---------- layout
    def _build(self):
        tb = self.addToolBar("Main"); tb.setMovable(False); tb.setIconSize(QtCore.QSize(18, 18))
        act = QtGui.QAction("Open dataset…", self); act.setShortcut("Ctrl+O"); act.triggered.connect(self.open_dialog); tb.addAction(act)
        self.recent_btn = QtWidgets.QToolButton(); self.recent_btn.setText("Recent"); self.recent_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.recent_menu = QtWidgets.QMenu(self); self.recent_menu.aboutToShow.connect(self._fill_recent); self.recent_btn.setMenu(self.recent_menu)
        tb.addWidget(self.recent_btn); tb.addSeparator()
        self.title = QtWidgets.QLabel("No dataset loaded"); self.title.setStyleSheet("font-weight:600; padding:0 8px"); tb.addWidget(self.title)

        central = QtWidgets.QWidget(); v = QtWidgets.QVBoxLayout(central); v.setContentsMargins(6, 6, 6, 6)
        self.tabs = QtWidgets.QTabWidget(); self.tabs.currentChanged.connect(lambda _: self.refresh()); v.addWidget(self.tabs, 1)
        v.addWidget(self._transport())
        self.setCentralWidget(central)

        dock = QtWidgets.QDockWidget("Live", self); dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        w = QtWidgets.QWidget(); dl = QtWidgets.QVBoxLayout(w)
        self.clock_lbl = QtWidgets.QLabel("—"); self.clock_lbl.setStyleSheet("font-size:20px; font-weight:600; font-family:Consolas,monospace")
        self.dist_lbl = QtWidgets.QLabel(""); self.dist_lbl.setStyleSheet("color:#5a6672")
        dl.addWidget(self.clock_lbl); dl.addWidget(self.dist_lbl)
        self.live = QtWidgets.QTableWidget(0, 4); self.live.setHorizontalHeaderLabels(["Condition", "Error now", "RMSE so far", "Max so far"])
        self.live.verticalHeader().setVisible(False); self.live.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.live.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch); self.live.setMinimumWidth(420)
        dl.addWidget(self.live); dl.addStretch(1)
        self.follow = QtWidgets.QCheckBox("Trajectories follow the robot"); self.follow.toggled.connect(lambda _: self.refresh())
        self.span = QtWidgets.QSpinBox(); self.span.setRange(10, 5000); self.span.setValue(150); self.span.setSuffix(" m view")
        self.full_gt = QtWidgets.QCheckBox("Show the whole ground-truth path faintly"); self.full_gt.setChecked(True)
        self.full_gt.toggled.connect(self._toggle_full_gt)
        self.logerr = QtWidgets.QCheckBox("Log scale for position error"); self.logerr.setChecked(True); self.logerr.toggled.connect(self._toggle_log)
        self.window = QtWidgets.QComboBox()
        for label, mins in WINDOWS: self.window.addItem(label, mins)
        self.window.setCurrentIndex(1); self.window.currentIndexChanged.connect(lambda _: self.refresh())
        wl = QtWidgets.QHBoxLayout(); wl.addWidget(QtWidgets.QLabel("Heading and speed time axis")); wl.addWidget(self.window, 1)
        for x in (self.follow, self.span, self.full_gt, self.logerr): dl.addWidget(x)
        dl.addLayout(wl)
        dock.setWidget(w); self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
        self._placeholder()

    def _transport(self):
        bar = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(bar); h.setContentsMargins(0, 4, 0, 0)
        self.play_btn = QtWidgets.QPushButton("Play"); self.play_btn.setFixedWidth(80); self.play_btn.clicked.connect(self.toggle_play)
        self.play_btn.setShortcut("Space")
        rst = QtWidgets.QPushButton("Restart"); rst.clicked.connect(self.restart)
        self.speed = QtWidgets.QComboBox()
        for s in SPEEDS: self.speed.addItem(f"{s:g}× " + ("(real time)" if s == 1 else ""), s)
        self.speed.setCurrentIndex(SPEEDS.index(1))
        # valueChanged, not sliderMoved: clicking the track (page step) must seek too; refresh() blocks the signal itself
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(0, 10000); self.slider.valueChanged.connect(self.seek)
        self.time_lbl = QtWidgets.QLabel("0:00 / 0:00"); self.time_lbl.setStyleSheet("font-family:Consolas,monospace")
        h.addWidget(self.play_btn); h.addWidget(rst); h.addWidget(QtWidgets.QLabel("Speed")); h.addWidget(self.speed)
        h.addWidget(self.slider, 1); h.addWidget(self.time_lbl)
        for w in (self.play_btn, rst, self.speed, self.slider): w.setEnabled(False)
        self._transport_widgets = (self.play_btn, rst, self.speed, self.slider)
        return bar

    def _placeholder(self):
        self.tabs.clear()
        lbl = QtWidgets.QLabel("Open a dataset to start:  Open dataset…  (Ctrl+O)\n\nPick the NCLT sensor archive (…_sen.tar.gz) "
                               "and its groundtruth_….csv.")
        lbl.setAlignment(QtCore.Qt.AlignCenter); lbl.setStyleSheet("color:#5a6672; font-size:15px")
        self.tabs.addTab(lbl, "Start")

    # ---------- opening datasets
    def _fill_recent(self):
        self.recent_menu.clear()
        files = sorted(glob.glob(os.path.join(engine.CACHE, "*.npz")), key=os.path.getmtime, reverse=True)
        for f in files[:15]:
            try:
                with np.load(f) as z: m = json.loads(str(z["meta"]))
            except Exception: continue
            a = self.recent_menu.addAction(f"{m['name']}  ·  {m['settings'].get('name','')}  ·  "
                                           f"{'60 s fit' if m['align'] == 'fit60' else 'start pose'}")
            a.triggered.connect(lambda _=False, f=f: self._open_cached(f))
        if not files: self.recent_menu.addAction("No processed datasets yet").setEnabled(False)

    def _open_cached(self, f):
        try:
            d = engine.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Could not open", f"{os.path.basename(f)} is unreadable ({e}). "
                                           "Open the dataset again to rebuild it."); return
        self.set_dataset(d)

    def open_dialog(self):
        dlg = OpenDialog(self, self.store)
        if dlg.exec() != QtWidgets.QDialog.Accepted: return
        self.pause()
        self.prog = QtWidgets.QProgressDialog("Starting…", None, 0, 0, self); self.prog.setWindowTitle("Processing dataset")
        self.prog.setWindowModality(QtCore.Qt.WindowModal); self.prog.setMinimumWidth(420); self.prog.show()
        self.worker = Worker(dlg.values())
        self.worker.progress.connect(self.prog.setLabelText)
        self.worker.done.connect(self._processed); self.worker.failed.connect(self._failed)
        self.worker.start()

    def _processed(self, d):
        self.prog.close(); self.set_dataset(d)

    def _failed(self, msg):
        self.prog.close(); QtWidgets.QMessageBox.critical(self, "Could not process the dataset", msg)

    # ---------- building the plots for a dataset
    def set_dataset(self, d):
        self.pause(); self.d = d; m = d["meta"]; T = d["t"]
        self.conds = m["conditions"]; self.nc = len(self.conds); self.t0, self.t1 = float(T[0]), float(T[-1])
        self.tm = (T - T[0]) / 60.0                                  # minutes from start, for time axes
        self.title.setText(f"{m['name']}   ·   {m['distance']/1000:.2f} km   ·   {m['duration']/60:.0f} min   ·   "
                           f"{m['settings'].get('name','')}")
        self.updaters = []; self.tabs.blockSignals(True); self.tabs.clear()
        self._tab_traj(); self._tab_error(); self._tab_pair("Heading", "h", "heading (deg)", np.degrees)
        self._tab_pair("Speed", "v", "speed (m/s)", lambda x: x); self._tab_sensors(); self._tab_summary()
        self.tabs.blockSignals(False)
        self.live.setRowCount(self.nc)
        for i, c in enumerate(self.conds):
            it = QtWidgets.QTableWidgetItem(c); it.setForeground(QtGui.QColor(COLORS[i % len(COLORS)])); self.live.setItem(i, 0, it)
            for j in (1, 2, 3): self.live.setItem(i, j, QtWidgets.QTableWidgetItem("—"))
        self.err2 = [np.cumsum(d[f"c{i}_err"] ** 2) for i in range(self.nc)]
        self.errmax = [np.maximum.accumulate(d[f"c{i}_err"]) for i in range(self.nc)]
        for w in self._transport_widgets: w.setEnabled(True)
        self.t_play = self.t0; self.refresh()

    def _grid(self, n):
        cols = 2 if n > 1 else 1; return [(i // cols, i % cols) for i in range(n)]

    def _tab_traj(self):
        d = self.d; gl = pg.GraphicsLayoutWidget(); self.tabs.addTab(gl, "Trajectories")
        gx, gy = d["gt_x"], d["gt_y"]; s = max(1, len(gx) // 3000)
        lo, hi = np.array([gx.min(), gy.min()]), np.array([gx.max(), gy.max()]); pad = 0.08 * (hi - lo).max()
        self.traj_plots, self.full_items = [], []
        first = None
        for i, (r, c) in enumerate(self._grid(self.nc)):
            p = gl.addPlot(row=r, col=c, title=self.conds[i]); p.setAspectLocked(True); p.showGrid(x=True, y=True, alpha=0.15)
            p.setLabel("bottom", "x (m)"); p.setLabel("left", "y (m)")
            full = p.plot(gx[::s], gy[::s], pen=GT_FULL_PEN); self.full_items.append(full)
            gt = p.plot(pen=GT_PEN, name="ground truth"); est = p.plot(pen=pg.mkPen(COLORS[i % len(COLORS)], width=LINE_W))
            gdot = pg.ScatterPlotItem(size=11, pen=pg.mkPen("k", width=2), brush=pg.mkBrush(255, 255, 255, 230))
            edot = pg.ScatterPlotItem(size=11, pen=pg.mkPen("w", width=1.5), brush=pg.mkBrush(COLORS[i % len(COLORS)]))
            p.addItem(gdot); p.addItem(edot)
            p.setXRange(lo[0] - pad, hi[0] + pad, padding=0); p.setYRange(lo[1] - pad, hi[1] + pad, padding=0)
            if first is None: first = p
            else: p.setXLink(first); p.setYLink(first)
            self.traj_plots.append(p)
            ex, ey = d[f"c{i}_x"], d[f"c{i}_y"]
            def upd(k, gt=gt, est=est, gdot=gdot, edot=edot, ex=ex, ey=ey, p=p, i=i):
                gt.setData(dec(gx, k), dec(gy, k)); est.setData(dec(ex, k), dec(ey, k))
                j = max(k - 1, 0); gdot.setData([gx[j]], [gy[j]]); edot.setData([ex[j]], [ey[j]])
                p.setTitle(f"{self.conds[i]}   <span style='color:#5a6672'>error {self.d[f'c{i}_err'][j]:.1f} m</span>")
            self.updaters.append((0, upd))
        def follow(k):
            if self.follow.isChecked() and k > 0:
                j = k - 1; h = self.span.value() / 2
                first.setXRange(gx[j] - h, gx[j] + h, padding=0); first.setYRange(gy[j] - h, gy[j] + h, padding=0)
        self.updaters.append((0, follow))

    def _cursor(self, p):
        line = pg.InfiniteLine(angle=90, pen=pg.mkPen((90, 90, 90, 120), width=1, style=QtCore.Qt.DashLine)); p.addItem(line); return line

    def _tab_error(self):
        d = self.d; gl = pg.GraphicsLayoutWidget(); idx = self.tabs.addTab(gl, "Position error")
        pt = gl.addPlot(row=0, col=0, title="Position error vs time"); pd_ = gl.addPlot(row=1, col=0, title="Position error vs distance travelled")
        pt.setLabel("bottom", "time (min)"); pt.setLabel("left", "error (m)"); pd_.setLabel("bottom", "distance (km)"); pd_.setLabel("left", "error (m)")
        for p in (pt, pd_): p.showGrid(x=True, y=True, alpha=0.2); p.addLegend(offset=(10, 10))
        self.err_plots = (pt, pd_); cur = self._cursor(pt)
        pt.setXRange(0, self.tm[-1], padding=0.01); pd_.setXRange(0, d["dist"][-1] / 1000, padding=0.01)
        km = d["dist"] / 1000
        items = []
        for i, c in enumerate(self.conds):
            pen = pg.mkPen(COLORS[i % len(COLORS)], width=LINE_W)
            items.append((pt.plot(pen=pen, name=c), pd_.plot(pen=pen, name=c), np.maximum(d[f"c{i}_err"], 1e-3)))
        def upd(k):
            (t0, t1), (d0, d1) = pt.viewRange()[0], pd_.viewRange()[0]
            ta, tb = visible(self.tm, k, t0, t1); da, db = visible(km, k, d0, d1)
            for a, b, e in items: a.setData(*envelope(self.tm, e, ta, tb)); b.setData(*envelope(km, e, da, db))
            cur.setValue(self.tm[max(k - 1, 0)])
        self.updaters.append((idx, upd)); self._toggle_log(self.logerr.isChecked())

    def _tab_pair(self, name, key, ylabel, conv):
        """one plot per condition, ground truth drawn in each; time axis set by the 'Heading and speed time axis' control"""
        d = self.d; gl = pg.GraphicsLayoutWidget(); idx = self.tabs.addTab(gl, name)
        heading = key == "h"; gtv = conv(d[f"gt_{key}"]); items = []; first = None
        if heading: ylo, yhi = -185, 185
        else:
            lo, hi = np.percentile(gtv, 0.5), np.percentile(gtv, 99.5); padv = 0.15 * (hi - lo + 1e-6); ylo, yhi = min(lo, 0) - padv, hi + padv
        for i, (r, c) in enumerate(self._grid(self.nc)):
            p = gl.addPlot(row=r, col=c, title=f"{self.conds[i]}: {name.lower()}"); p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("bottom", "time (min)"); p.setLabel("left", ylabel); p.addLegend(offset=(10, 10))
            g = p.plot(pen=GT_PEN, name="ground truth", connect="finite")
            e = p.plot(pen=pg.mkPen(COLORS[i % len(COLORS)], width=LINE_W), name=self.conds[i], connect="finite")
            p.setYRange(ylo, yhi, padding=0)
            if heading: p.getAxis("left").setTicks([[(v, f"{v}°") for v in (-180, -90, 0, 90, 180)]])
            if first is None: first = p
            else: p.setXLink(first)
            ev = conv(d[f"c{i}_{key}"])
            if not heading:                       # IMU-only speed runs away: pin it to the plot edge and say so
                off = np.mean((ev < ylo) | (ev > yhi))
                ev = np.clip(ev, ylo, yhi)
                if off > 0.2: p.setTitle(f"{self.conds[i]}: {name.lower()}   <span style='color:#b3263a'>"
                                         f"(off scale {100 * off:.0f}% of the time, pinned to the edge)</span>")
            items.append((g, e, ev, self._cursor(p)))
        def upd(k):
            now = self.tm[max(k - 1, 0)]; mins = self.window.currentData()
            x0, x1 = (now - mins, now + 0.02 * mins) if mins else (0, self.tm[-1])
            first.setXRange(x0, x1, padding=0); a, b = visible(self.tm, k, x0, x1)
            for g, e, ev, cur in items:
                for item, y in ((g, gtv), (e, ev)):
                    if heading:
                        s = max(1, -(-(b - a) // (2 * SERIES_BUCKETS)))
                        item.setData(*wrap_breaks(self.tm[a:b:s], y[a:b:s]), connect="finite")
                    else:
                        item.setData(*envelope(self.tm, y, a, b))
                cur.setValue(now)
        self.updaters.append((idx, upd))

    def _tab_sensors(self):
        d = self.d; gl = pg.GraphicsLayoutWidget(); idx = self.tabs.addTab(gl, "Sensors (live)")
        W = 60.0; ts = d["t"] - self.t0
        specs = [("Gyroscope, yaw rate", "deg/s", [("s_gz", "#2f8fa3", "gyro z")]),
                 ("Accelerometer, forward axis", "m/s²", [("s_ax", "#d1495b", "accel x")]),
                 ("Wheel speeds", "m/s", [("s_vl", "#e8a33d", "left"), ("s_vr", "#4c9a5f", "right"), ("gt_v", (25, 25, 25), "ground-truth speed")])]
        items = []
        for r, (title, unit, series) in enumerate(specs):
            p = gl.addPlot(row=r, col=0, title=f"{title}  (last {W:.0f} s)"); p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("left", unit); p.setLabel("bottom", "time (s)"); p.addLegend(offset=(10, 10))
            v = np.concatenate([d[k] for k, _, _ in series]); lo, hi = np.percentile(v, 0.2), np.percentile(v, 99.8)
            p.setYRange(lo - 0.1 * (hi - lo + 1e-6), hi + 0.1 * (hi - lo + 1e-6), padding=0)
            for k, col, nm in series: items.append((p, p.plot(pen=pg.mkPen(col, width=LINE_W), name=nm), d[k]))
        def upd(k):
            if k <= 0: return
            t_now = ts[k - 1]; a = np.searchsorted(ts, t_now - W)
            for p, it, y in items: it.setData(ts[a:k], y[a:k])
            for p in {id(x[0]): x[0] for x in items}.values(): p.setXRange(t_now - W, t_now, padding=0)
        self.updaters.append((idx, upd))

    def _tab_summary(self):
        m = self.d["meta"]; tb = QtWidgets.QTextBrowser(); self.tabs.addTab(tb, "Summary")
        rows = "".join(f"<tr><td style='color:{COLORS[i % len(COLORS)]}'><b>{s['name']}</b></td><td align=right>{s['rmse']:.1f}</td>"
                       f"<td align=right>{s['max']:.1f}</td><td align=right>{s['final']:.1f}</td><td align=right>{s['pct']:.2f} %</td></tr>"
                       for i, s in enumerate(m["stats"]))
        c = m["calib"]; logs = "".join(f"<li><b>{n}</b>: {l}</li>" for n, l in zip(m["conditions"], m["logs"]))
        same = [(m["conditions"][i], m["conditions"][j]) for i in range(self.nc) for j in range(i + 1, self.nc)
                if np.array_equal(self.d[f"c{i}_err"], self.d[f"c{j}_err"])]
        if same:
            rows += "".join(f"<tr><td colspan=5 style='color:#5a6672'>{a} and {b} give identical results on this dataset, "
                            f"so their lines overlap in the graphs.</td></tr>" for a, b in same)
        tb.setHtml(f"""<h2>{m['name']}</h2>
        <p>{m['distance']/1000:.2f} km of ground-truth travel over {m['duration']/60:.1f} min.
        Alignment: {'rigid fit on the first 60 s of motion' if m['align'] == 'fit60' else 'true pose when motion starts'}.</p>
        <h3>Whole-run accuracy</h3><table cellpadding=6 border=0>
        <tr><th align=left>Condition</th><th>RMSE (m)</th><th>Max (m)</th><th>Final (m)</th><th>Final / distance</th></tr>{rows}</table>
        <h3>Self-calibration (from sensor data only)</h3>
        <p>Still period: first {c['still_s']:.1f} s ({c['still_rows']} samples); motion starts at {c['t_move']:.1f} s.<br>
        Track width: {c['track']:.3f} m. Magnetometer hard-iron offset: {', '.join(f'{x:.4f}' for x in c['moff'])}.</p>
        <h3>Filter settings: {m['settings'].get('name','')}</h3><p style='font-family:Consolas,monospace'>{m['settings']['args']}</p>
        <h3>Update counts (accepted / rejected)</h3><ul>{logs}</ul>
        <p>Files: {m['tar']}<br>{m['gt']}</p>""")

    # ---------- playback
    def toggle_play(self):
        if self.playing: self.pause()
        else:
            if self.t_play >= self.t1: self.t_play = self.t0
            self.playing = True; self.play_btn.setText("Pause"); self.clock.start(); self.timer.start(33)

    def pause(self):
        self.playing = False; self.timer.stop(); self.play_btn.setText("Play")

    def restart(self):
        self.t_play = self.t0; self.refresh()

    def seek(self, v):
        if self.d is None: return
        self.t_play = self.t0 + (self.t1 - self.t0) * v / 10000; self.refresh()

    def tick(self):
        dt = self.clock.restart() / 1000.0
        self.t_play += dt * self.speed.currentData()
        if self.t_play >= self.t1: self.t_play = self.t1; self.pause()
        self.refresh()

    def refresh(self):
        if self.d is None: return
        T = self.d["t"]; k = int(np.searchsorted(T, self.t_play, side="right")); cur = self.tabs.currentIndex()
        for tab, f in self.updaters:
            if tab == cur: f(k)
        j = max(k - 1, 0)
        self.clock_lbl.setText(fmt_time(self.t_play - self.t0)); self.dist_lbl.setText(f"{self.d['dist'][j]/1000:.3f} km travelled")
        self.time_lbl.setText(f"{fmt_time(self.t_play - self.t0)} / {fmt_time(self.t1 - self.t0)}")
        self.slider.blockSignals(True); self.slider.setValue(int(10000 * (self.t_play - self.t0) / max(self.t1 - self.t0, 1e-9))); self.slider.blockSignals(False)
        for i in range(self.nc):
            e = self.d[f"c{i}_err"][j]; rm = np.sqrt(self.err2[i][j] / (j + 1)); mx = self.errmax[i][j]
            for col, val in ((1, e), (2, rm), (3, mx)): self.live.item(i, col).setText(f"{val:,.1f} m")

    def _toggle_full_gt(self, on):
        for it in getattr(self, "full_items", []): it.setVisible(on)

    def _toggle_log(self, on):
        for p in getattr(self, "err_plots", ()): p.setLogMode(y=on); p.enableAutoRange(axis="y")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion")
    w = Player(); w.show()
    if len(sys.argv) > 2:      # python app.py sen.tar.gz groundtruth.csv  -> process straight away with the first profile
        w.set_dataset(engine.process(sys.argv[1], sys.argv[2], engine.settings_list()[0]))
    sys.exit(app.exec())
