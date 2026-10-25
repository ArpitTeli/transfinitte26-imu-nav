"""Fusion Player: watch IMU navigation unfold, from recorded data or live from the phone.

Start page, two ways in:
  * Recorded data: a phone walk recording (.csv from IMU Stream or IMU Live) through the foot filter, or an NCLT robot
    dataset (sensor archive + ground truth) through the IMU / wheel / magnetometer filter. Computed once, then played
    back at its own pace (1x) or faster.
  * Live from the phone: the IMU Stream app's Wi-Fi stream through the foot filter, as the walker moves.
Run:  python app.py  [walk.csv | sen.tar.gz groundtruth.csv | --live]
"""
import os, sys, glob, json, threading
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import engine
import foot
import livemode
from imunav.live.session import heading_up
from imunav.live.tracker import TRACKING

pg.setConfigOptions(antialias=False, background="w", foreground="#30363d")
LINE_W, GT_W = 1, 1      # wide pens are slow to draw in Qt's software renderer
GT_PEN = pg.mkPen((15, 15, 15), width=GT_W)
GT_FULL_PEN = pg.mkPen((190, 195, 200), width=1)
COLORS = ["#7d6fb0", "#e8a33d", "#2f8fa3", "#d1495b", "#4c9a5f", "#8a5a44"]
PATH_COLOR = "#2f6fb0"
FLAT, SWING, WAIT = "#2e9e4f", "#e8a33d", "#9aa3ad"     # current-position dot: foot flat, swinging, calibrating
SPEEDS = [0.5, 1, 2, 5, 10, 30, 60, 120, 300, 600]
WINDOWS = [("Last 2 min", 2), ("Last 10 min", 10), ("Last 30 min", 30), ("Whole run", None)]
MAX_TRAJ_PTS = 3000
SERIES_BUCKETS = 1500                          # time series: min/max envelope over this many buckets
LIVE_PORT = 5555


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


def fit_view(p, xy, min_half=3.0):
    """square view around the points with a margin; never smaller than min_half metres each side of centre"""
    lo, hi = xy.min(0), xy.max(0); cx, cy = (lo + hi) / 2; h = max(min_half, 0.55 * max(hi - lo) + 1.0)
    p.setXRange(cx - h, cx + h, padding=0); p.setYRange(cy - h, cy + h, padding=0)


# ---------------------------------------------------------------- background processing
class Worker(QtCore.QObject):
    """Runs fn(*args, cb=...) on a daemon thread; the signals arrive queued on the GUI thread.
    A daemon thread (not a QThread) so closing the window mid-processing exits cleanly instead of aborting
    with 'QThread destroyed while running' (processing cannot be interrupted)."""
    progress = QtCore.Signal(str)
    done = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, fn, args):
        super().__init__(); self.fn, self.args = fn, args

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self.done.emit(self.fn(*self.args, cb=self.progress.emit))
        except Exception as e:
            self.failed.emit(str(e))


class OpenDialog(QtWidgets.QDialog):
    """Recorded data: a phone walk (.csv) or an NCLT robot dataset (sensor archive + ground truth)"""

    def __init__(self, parent, settings_store):
        super().__init__(parent)
        self.setWindowTitle("Open recorded data"); self.setMinimumWidth(640); self.store = settings_store
        v = QtWidgets.QVBoxLayout(self)
        self.kind_walk = QtWidgets.QRadioButton("Phone walk recording (.csv from IMU Stream or IMU Live)")
        self.kind_nclt = QtWidgets.QRadioButton("NCLT robot dataset (sensor archive + ground truth)")
        (self.kind_nclt if self.store.value("last_kind", "walk") == "nclt" else self.kind_walk).setChecked(True)
        v.addWidget(self.kind_walk); v.addWidget(self.kind_nclt)
        self.pages = QtWidgets.QStackedWidget(); v.addWidget(self.pages)

        pw = QtWidgets.QWidget(); fw = QtWidgets.QFormLayout(pw)
        self.walk = self._file_row(fw, "Recording", "Walk recordings (*.csv);;All files (*)", "last_walk")
        note = QtWidgets.QLabel("The phone's own copy (Download/IMU Stream) or IMU Live's (Documents/IMU Live). The walk "
                                "must start with the shoe still for 2 s. Long recordings take a minute or two the first "
                                "time; after that they open from the cache.")
        note.setWordWrap(True); note.setStyleSheet("color:#5a6672"); fw.addRow(note)
        self.pages.addWidget(pw)

        pn = QtWidgets.QWidget(); fn = QtWidgets.QFormLayout(pn)
        self.tar = self._file_row(fn, "Sensor archive", "NCLT sensor archive (*.tar.gz *.tgz *.tar *.gz);;All files (*)", "last_tar")
        self.gt = self._file_row(fn, "Ground truth", "Ground-truth CSV (*.csv);;All files (*)", "last_gt")
        self.profile = QtWidgets.QComboBox()
        for p in engine.settings_list():
            with open(p) as fh:
                self.profile.addItem(json.load(fh).get("name", os.path.basename(p)), p)
        fn.addRow("Filter settings", self.profile)
        self.align = QtWidgets.QComboBox()
        self.align.addItem("Fit on the first 60 s of motion", "fit60"); self.align.addItem("True pose when motion starts", "start")
        fn.addRow("Align to ground truth", self.align)
        note = QtWidgets.QLabel("Use the files exactly as downloaded from the NCLT site. The first run of a dataset takes "
                                "10-30 s; after that it opens from the cache.")
        note.setWordWrap(True); note.setStyleSheet("color:#5a6672"); fn.addRow(note)
        self.pages.addWidget(pn)

        self.kind_walk.toggled.connect(lambda on: self.pages.setCurrentIndex(0 if on else 1))
        self.pages.setCurrentIndex(0 if self.kind_walk.isChecked() else 1)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.button(QtWidgets.QDialogButtonBox.Ok).setText("Process"); bb.accepted.connect(self._accept); bb.rejected.connect(self.reject)
        v.addWidget(bb)

    def _file_row(self, lay, label, filt, key):
        w = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(w); h.setContentsMargins(0, 0, 0, 0)
        e = QtWidgets.QLineEdit(self.store.value(key, "")); b = QtWidgets.QPushButton("Browse…")
        def pick():
            f, _ = QtWidgets.QFileDialog.getOpenFileName(self, label, os.path.dirname(e.text()) or "", filt)
            if f: e.setText(f)
        b.clicked.connect(pick); h.addWidget(e, 1); h.addWidget(b); lay.addRow(label, w); e._key = key; e._label = label
        return e

    def _accept(self):
        fields = (self.walk,) if self.kind_walk.isChecked() else (self.tar, self.gt)
        for e in fields:
            if not os.path.isfile(e.text()):
                QtWidgets.QMessageBox.warning(self, "File not found", f"Choose an existing file for {e._label.lower()}."); return
            self.store.setValue(e._key, e.text())
        self.store.setValue("last_kind", "walk" if self.kind_walk.isChecked() else "nclt")
        self.accept()

    def job(self):
        """(function, args) that processes the chosen data"""
        if self.kind_walk.isChecked():
            return foot.process, (self.walk.text(),)
        return engine.process, (self.tar.text(), self.gt.text(), self.profile.currentData(), self.align.currentData())


class StartPage(QtWidgets.QWidget):
    def __init__(self, on_recorded, on_live):
        super().__init__()
        v = QtWidgets.QVBoxLayout(self); v.setContentsMargins(60, 40, 60, 40); v.addStretch(1)
        t = QtWidgets.QLabel("Fusion Player"); t.setStyleSheet("font-size:28px; font-weight:600"); v.addWidget(t, 0, QtCore.Qt.AlignHCenter)
        s = QtWidgets.QLabel("IMU navigation, from recorded data or live from the phone on the shoe")
        s.setStyleSheet("color:#5a6672; font-size:15px"); v.addWidget(s, 0, QtCore.Qt.AlignHCenter); v.addSpacing(28)
        row = QtWidgets.QHBoxLayout(); row.addStretch(1)
        for title, text, fn in (
                ("Recorded data", "Play back a phone walk (.csv from IMU Stream or IMU Live) or an NCLT robot dataset, "
                                  "at real time or faster.", on_recorded),
                ("Live from the phone", "Stream from the IMU Stream app over Wi-Fi and draw the walker's path as they "
                                        "move.", on_live)):
            card = QtWidgets.QVBoxLayout()
            b = QtWidgets.QPushButton(title); b.setMinimumSize(300, 110); b.clicked.connect(fn)
            b.setStyleSheet("font-size:20px; font-weight:600")
            d = QtWidgets.QLabel(text); d.setWordWrap(True); d.setFixedWidth(300); d.setStyleSheet("color:#5a6672")
            card.addWidget(b); card.addWidget(d); card.addStretch(1); row.addLayout(card); row.addSpacing(32)
        row.addStretch(1); v.addLayout(row); v.addStretch(2)


# ---------------------------------------------------------------- main window
class Player(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fusion Player"); self.resize(1500, 920)
        self.store = QtCore.QSettings("Transfinitte26", "FusionPlayer")
        self.d = None; self.t_play = 0.0; self.playing = False; self.worker = None; self.live_run = None
        self.clock = QtCore.QElapsedTimer(); self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick)
        self.live_timer = QtCore.QTimer(self); self.live_timer.timeout.connect(self.live_tick)
        self._build()
        self.go_home()

    # ---------- layout
    def _build(self):
        tb = self.addToolBar("Main"); tb.setMovable(False); tb.setIconSize(QtCore.QSize(18, 18))
        home = QtGui.QAction("Home", self); home.triggered.connect(self.go_home); tb.addAction(home)
        act = QtGui.QAction("Open recorded data…", self); act.setShortcut("Ctrl+O"); act.triggered.connect(self.open_dialog); tb.addAction(act)
        self.recent_btn = QtWidgets.QToolButton(); self.recent_btn.setText("Recent"); self.recent_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.recent_menu = QtWidgets.QMenu(self); self.recent_menu.aboutToShow.connect(self._fill_recent); self.recent_btn.setMenu(self.recent_menu)
        tb.addWidget(self.recent_btn)
        live = QtGui.QAction("Live from the phone", self); live.setShortcut("Ctrl+L"); live.triggered.connect(self.start_live); tb.addAction(live)
        tb.addSeparator()
        self.title = QtWidgets.QLabel(""); self.title.setStyleSheet("font-weight:600; padding:0 8px"); tb.addWidget(self.title)

        central = QtWidgets.QWidget(); v = QtWidgets.QVBoxLayout(central); v.setContentsMargins(6, 6, 6, 6)
        self.tabs = QtWidgets.QTabWidget(); self.tabs.currentChanged.connect(lambda _: self.refresh()); v.addWidget(self.tabs, 1)
        self.transport = self._transport(); v.addWidget(self.transport)
        self.setCentralWidget(central)

        self.dock = QtWidgets.QDockWidget("", self); self.dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        self.side = QtWidgets.QStackedWidget(); self.side.addWidget(self._side_playback()); self.side.addWidget(self._side_live())
        self.dock.setWidget(self.side); self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.dock)

    def _side_playback(self):
        w = QtWidgets.QWidget(); dl = QtWidgets.QVBoxLayout(w)
        self.clock_lbl = QtWidgets.QLabel("—"); self.clock_lbl.setStyleSheet("font-size:20px; font-weight:600; font-family:Consolas,monospace")
        self.dist_lbl = QtWidgets.QLabel(""); self.dist_lbl.setStyleSheet("color:#5a6672")
        dl.addWidget(self.clock_lbl); dl.addWidget(self.dist_lbl)
        self.live = QtWidgets.QTableWidget(0, 4)
        self.live.verticalHeader().setVisible(False); self.live.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.live.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch); self.live.setMinimumWidth(420)
        dl.addWidget(self.live); dl.addStretch(1)
        self.follow = QtWidgets.QCheckBox("Path view follows the current position"); self.follow.toggled.connect(lambda _: self.refresh())
        self.span = QtWidgets.QSpinBox(); self.span.setRange(4, 5000); self.span.setValue(150); self.span.setSuffix(" m view")
        self.full_gt = QtWidgets.QCheckBox("Show the whole ground-truth path faintly"); self.full_gt.setChecked(True)
        self.full_gt.toggled.connect(self._toggle_full_gt)
        self.logerr = QtWidgets.QCheckBox("Log scale for position error"); self.logerr.setChecked(True); self.logerr.toggled.connect(self._toggle_log)
        self.window = QtWidgets.QComboBox()
        for label, mins in WINDOWS: self.window.addItem(label, mins)
        self.window.setCurrentIndex(1); self.window.currentIndexChanged.connect(lambda _: self.refresh())
        wl = QtWidgets.QHBoxLayout(); wl.addWidget(QtWidgets.QLabel("Heading and speed time axis")); wl.addWidget(self.window, 1)
        for x in (self.follow, self.span, self.full_gt, self.logerr): dl.addWidget(x)
        dl.addLayout(wl)
        return w

    def _side_live(self):
        w = QtWidgets.QWidget(); dl = QtWidgets.QVBoxLayout(w); w.setMinimumWidth(420)
        big = "font-size:20px; font-weight:600"
        self.lv_state = QtWidgets.QLabel(); self.lv_state.setStyleSheet(big)
        self.lv_help = QtWidgets.QLabel(); self.lv_help.setWordWrap(True)
        self.lv_link = QtWidgets.QLabel(); self.lv_link.setStyleSheet("font-weight:600; font-family:Consolas,monospace")
        self.lv_addr = QtWidgets.QLabel(); self.lv_addr.setWordWrap(True); self.lv_addr.setStyleSheet("color:#5a6672")
        self.lv_stats = QtWidgets.QLabel(); self.lv_stats.setStyleSheet("font-size:15px; font-family:Consolas,monospace")
        self.lv_rec = QtWidgets.QLabel(); self.lv_rec.setWordWrap(True); self.lv_rec.setStyleSheet("color:#5a6672")
        for x in (self.lv_state, self.lv_help): dl.addWidget(x)
        dl.addSpacing(10)
        for x in (self.lv_link, self.lv_addr): dl.addWidget(x)
        dl.addSpacing(10); dl.addWidget(self.lv_stats); dl.addSpacing(10)
        grid = QtWidgets.QGridLayout()
        buttons = [("Recalibrate", "R", "The walker stands still: calibrate again and start a new path", lambda: self.live_run.tracker.reset()),
                   ("Reset origin", "C", "Keep tracking, but make the current position the start", lambda: self.live_run.tracker.clear()),
                   ("Next labelled walk", "N", "Data collection: a new file for the next protocol walk", self._live_next_walk),
                   ("Marker", "M", "Data collection: note the time of something worth knowing later", lambda: self.live_run.session.marker())]
        for i, (text, key, tip, fn) in enumerate(buttons):
            b = QtWidgets.QPushButton(f"{text}   ({key})"); b.setShortcut(key); b.setToolTip(tip); b.clicked.connect(fn)
            grid.addWidget(b, i // 2, i % 2)
        dl.addLayout(grid)
        stop = QtWidgets.QPushButton("Stop live"); stop.clicked.connect(self.go_home); stop.setStyleSheet("font-weight:600")
        dl.addWidget(stop); dl.addSpacing(10); dl.addWidget(self.lv_rec); dl.addStretch(1)
        return w

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
        self._transport_widgets = (self.play_btn, rst, self.speed, self.slider)
        return bar

    # ---------- modes: start page, playback, live
    def go_home(self):
        self.stop_live(); self.pause(); self.d = None
        self.tabs.blockSignals(True); self.tabs.clear()
        self.tabs.addTab(StartPage(self.open_dialog, self.start_live), "Start"); self.tabs.blockSignals(False)
        self.title.setText(""); self.transport.hide(); self.dock.hide()

    # ---------- opening recorded data
    def _fill_recent(self):
        self.recent_menu.clear()
        files = sorted(glob.glob(os.path.join(engine.CACHE, "*.npz")), key=os.path.getmtime, reverse=True)
        for f in files[:15]:
            try:
                with np.load(f) as z: m = json.loads(str(z["meta"]))
            except Exception: continue
            if m.get("kind") == "walk":
                label = f"Walk  ·  {m['name']}  ·  {m['duration'] / 60:.1f} min, {m['distance']:.0f} m"
            else:
                label = (f"NCLT  ·  {m['name']}  ·  {m['settings'].get('name', '')}  ·  "
                         f"{'60 s fit' if m['align'] == 'fit60' else 'start pose'}")
            a = self.recent_menu.addAction(label); a.triggered.connect(lambda _=False, f=f: self._open_cached(f))
        if not files: self.recent_menu.addAction("Nothing processed yet").setEnabled(False)

    def _open_cached(self, f):
        try:
            d = engine.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Could not open", f"{os.path.basename(f)} is unreadable ({e}). "
                                           "Open the data again to rebuild it."); return
        self.set_dataset(d)

    def open_dialog(self):
        dlg = OpenDialog(self, self.store)
        if dlg.exec() != QtWidgets.QDialog.Accepted: return
        self.process(*dlg.job())

    def process(self, fn, args):
        self.pause()
        self.prog = QtWidgets.QProgressDialog("Starting…", None, 0, 0, self); self.prog.setWindowTitle("Processing")
        self.prog.setWindowModality(QtCore.Qt.WindowModal); self.prog.setMinimumWidth(420); self.prog.show()
        self.worker = Worker(fn, args)
        self.worker.progress.connect(self.prog.setLabelText)
        self.worker.done.connect(self._processed); self.worker.failed.connect(self._failed)
        self.worker.start()

    def _processed(self, d):
        self.prog.close(); self.set_dataset(d)

    def _failed(self, msg):
        self.prog.close(); QtWidgets.QMessageBox.critical(self, "Could not process the data", msg)

    # ---------- building the plots for a dataset
    def set_dataset(self, d):
        self.stop_live(); self.pause(); self.d = d; m = d["meta"]; T = d["t"]
        self.walk = m.get("kind") == "walk"
        self.conds = m["conditions"]; self.nc = len(self.conds); self.t0, self.t1 = float(T[0]), float(T[-1])
        self.tm = (T - T[0]) / 60.0                                  # minutes from start, for time axes
        if self.walk:
            self.title.setText(f"Walk  ·  {m['name']}   ·   {m['distance']:.0f} m   ·   {m['duration'] / 60:.1f} min   ·   "
                               f"{m['stats'][0]['steps']} steps")
        else:
            self.title.setText(f"NCLT  ·  {m['name']}   ·   {m['distance']/1000:.2f} km   ·   {m['duration']/60:.0f} min   ·   "
                               f"{m['settings'].get('name','')}")
        self.updaters = []; self.tabs.blockSignals(True); self.tabs.clear()
        if self.walk:
            self._tab_walk_path()
        else:
            self._tab_traj(); self._tab_error()
        self._tab_pair("Heading", "h", "heading (deg)", np.degrees)
        self._tab_pair("Speed", "v", "walking speed (m/s)" if self.walk else "speed (m/s)", lambda x: x)
        self._tab_sensors(); self._tab_summary()
        self.tabs.blockSignals(False)
        cols = ["Condition", "Distance", "Steps", "Walking speed"] if self.walk else ["Condition", "Error now", "RMSE so far", "Max so far"]
        self.live.setHorizontalHeaderLabels(cols); self.live.setRowCount(self.nc)
        for i, c in enumerate(self.conds):
            it = QtWidgets.QTableWidgetItem(c); it.setForeground(QtGui.QColor(PATH_COLOR if self.walk else COLORS[i % len(COLORS)]))
            self.live.setItem(i, 0, it)
            for j in (1, 2, 3): self.live.setItem(i, j, QtWidgets.QTableWidgetItem("—"))
        if not self.walk:
            self.err2 = [np.cumsum(d[f"c{i}_err"] ** 2) for i in range(self.nc)]
            self.errmax = [np.maximum.accumulate(d[f"c{i}_err"]) for i in range(self.nc)]
        self.full_gt.setVisible(not self.walk); self.logerr.setVisible(not self.walk)
        self.span.setValue(20 if self.walk else 150)
        self.side.setCurrentIndex(0); self.dock.setWindowTitle("Playback"); self.dock.show(); self.transport.show()
        for w in self._transport_widgets: w.setEnabled(True)
        self.t_play = self.t0; self.refresh()

    def _grid(self, n):
        cols = 2 if n > 1 else 1; return [(i // cols, i % cols) for i in range(n)]

    def _tab_walk_path(self):
        """phone walk: one map, the path drawn as it happens; the dot shows whether the foot is flat"""
        d = self.d; gl = pg.GraphicsLayoutWidget(); self.tabs.addTab(gl, "Path")
        x, y, flat = d["c0_x"], d["c0_y"], d["c0_flat"]
        p = gl.addPlot(title="Foot path (start at the origin, first 3 m pointing up)"); p.setAspectLocked(True)
        p.showGrid(x=True, y=True, alpha=0.15); p.setLabel("bottom", "x (m)"); p.setLabel("left", "y (m)")
        full = p.plot(x[::max(1, len(x) // 3000)], y[::max(1, len(x) // 3000)], pen=GT_FULL_PEN); self.full_items = [full]
        path = p.plot(pen=pg.mkPen(PATH_COLOR, width=2))
        start = pg.ScatterPlotItem([0], [0], size=10, pen=pg.mkPen("w", width=1.5), brush=pg.mkBrush("#1f2328"))
        now = pg.ScatterPlotItem(size=14, pen=pg.mkPen("w", width=1.5)); p.addItem(start); p.addItem(now)
        fit_view(p, np.column_stack([x, y]))
        self.traj_plots = [p]
        def upd(k):
            path.setData(dec(x, k), dec(y, k)); j = max(k - 1, 0)
            now.setData([x[j]], [y[j]], brush=pg.mkBrush(FLAT if flat[j] > 0.5 else SWING))
            if self.follow.isChecked() and k > 0:
                h = self.span.value() / 2; p.setXRange(x[j] - h, x[j] + h, padding=0); p.setYRange(y[j] - h, y[j] + h, padding=0)
            p.setTitle(f"Foot path   <span style='color:#5a6672'>{d['dist'][j]:.1f} m walked, {int(d['steps'][j])} steps</span>")
        self.updaters.append((0, upd))

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
        """one plot per condition, ground truth drawn in each when there is one; time axis set by the 'Heading and
        speed time axis' control"""
        d = self.d; gl = pg.GraphicsLayoutWidget(); idx = self.tabs.addTab(gl, name)
        heading = key == "h"; has_gt = f"gt_{key}" in d; items = []; first = None
        gtv = conv(d[f"gt_{key}"]) if has_gt else None
        if heading: ylo, yhi = -185, 185
        else:
            ref = gtv if has_gt else conv(d[f"c0_{key}"])
            lo, hi = np.percentile(ref, 0.5), np.percentile(ref, 99.5); padv = 0.15 * (hi - lo + 1e-6); ylo, yhi = min(lo, 0) - padv, hi + padv
        for i, (r, c) in enumerate(self._grid(self.nc)):
            p = gl.addPlot(row=r, col=c, title=f"{self.conds[i]}: {name.lower()}"); p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("bottom", "time (min)"); p.setLabel("left", ylabel); p.addLegend(offset=(10, 10))
            g = p.plot(pen=GT_PEN, name="ground truth", connect="finite") if has_gt else None
            col = PATH_COLOR if self.walk else COLORS[i % len(COLORS)]
            e = p.plot(pen=pg.mkPen(col, width=LINE_W), name=self.conds[i], connect="finite")
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
                    if item is None: continue
                    if heading:
                        s = max(1, -(-(b - a) // (2 * SERIES_BUCKETS)))
                        item.setData(*wrap_breaks(self.tm[a:b:s], y[a:b:s]), connect="finite")
                    else:
                        item.setData(*envelope(self.tm, y, a, b))
                cur.setValue(now)
        self.updaters.append((idx, upd))

    def _tab_sensors(self):
        d = self.d; gl = pg.GraphicsLayoutWidget(); idx = self.tabs.addTab(gl, "Sensors (live)")
        W = 30.0 if self.walk else 60.0; ts = d["t"] - self.t0
        if self.walk:
            specs = [("Gyroscope, rotation rate (peak per 0.05 s)", "deg/s", [("s_g", "#2f8fa3", "|gyro|")]),
                     ("Accelerometer, magnitude (peak per 0.05 s)", "m/s²", [("s_a", "#d1495b", "|accel|")]),
                     ("Foot flat on the ground (zero-velocity updates apply)", "share", [("s_flat", FLAT, "foot flat")])]
        else:
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
        if self.walk:
            s, c, g = m["stats"][0], m["calib"], m["gaps"]
            rej = ", ".join(f"{k} {v}" for k, v in m["rejected"].items()) or "none"
            tb.setHtml(f"""<h2>{m['name']}</h2>
            <p>{m['duration'] / 60:.1f} min of recording ({m['format']} format), {s['distance']:.0f} m walked
            (estimated), {s['steps']} steps, {s['speed']:.2f} m/s on average.</p>
            <p>There is no ground truth for a free walk, so this view shows the path and how the filter saw it.
            Court walks are scored against the court with <code>imu_live/scripts/court_eval.py</code>.</p>
            <h3>Calibration at the start (from the sensor data only)</h3>
            <p>Still window: {c['still_from']:.1f} to {c['still_to']:.1f} s into the recording.
            Gyro bias: {', '.join(f'{x:.4f}' for x in c['gyro_bias'])} rad/s. Gravity: {c['gravity']:.3f} m/s².
            Filter rate: {c['rate']:.0f} Hz.</p>
            <h3>Data gaps</h3><p>{g['n']} gaps, {g['seconds']:.1f} s in all (longest {g['longest']:.1f} s).
            Nothing is integrated across a gap: the filter restarts at the next flat foot.</p>
            <h3>Filter settings: {m['settings']['name']}</h3><p style='font-family:Consolas,monospace'>{m['settings']['args']}</p>
            <h3>Rejected updates (outlier checks)</h3><p>{rej}</p>
            <p>File: {m['file']}</p>""")
            return
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
        if self.d is None: return
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
        self.clock_lbl.setText(fmt_time(self.t_play - self.t0))
        self.time_lbl.setText(f"{fmt_time(self.t_play - self.t0)} / {fmt_time(self.t1 - self.t0)}")
        self.slider.blockSignals(True); self.slider.setValue(int(10000 * (self.t_play - self.t0) / max(self.t1 - self.t0, 1e-9))); self.slider.blockSignals(False)
        if self.walk:
            self.dist_lbl.setText(f"{self.d['dist'][j]:.1f} m walked")
            vals = (f"{self.d['dist'][j]:.1f} m", f"{int(self.d['steps'][j])}", f"{self.d['c0_v'][j]:.2f} m/s")
            for col, val in zip((1, 2, 3), vals): self.live.item(0, col).setText(val)
            return
        self.dist_lbl.setText(f"{self.d['dist'][j]/1000:.3f} km travelled")
        for i in range(self.nc):
            e = self.d[f"c{i}_err"][j]; rm = np.sqrt(self.err2[i][j] / (j + 1)); mx = self.errmax[i][j]
            for col, val in ((1, e), (2, rm), (3, mx)): self.live.item(i, col).setText(f"{val:,.1f} m")

    def _toggle_full_gt(self, on):
        for it in getattr(self, "full_items", []): it.setVisible(on)

    def _toggle_log(self, on):
        for p in getattr(self, "err_plots", ()): p.setLogMode(y=on); p.enableAutoRange(axis="y")

    # ---------- live from the phone
    def start_live(self):
        if self.live_run is not None: return
        self.pause(); self.d = None
        try:
            self.live_run = livemode.LiveRun(LIVE_PORT)
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, "Cannot listen for the phone",
                                           f"UDP port {LIVE_PORT} is busy ({e}). Close IMU Live or any other copy of "
                                           "Fusion Player and try again."); return
        self.title.setText("Live from the phone"); self.transport.hide()
        self.tabs.blockSignals(True); self.tabs.clear(); self._live_tabs(); self.tabs.blockSignals(False)
        self.side.setCurrentIndex(1); self.dock.setWindowTitle("Live"); self.dock.show()
        self.live_timer.start(50)

    def _live_tabs(self):
        gl = pg.GraphicsLayoutWidget(); self.tabs.addTab(gl, "Path")
        p = gl.addPlot(title="Foot path (start at the origin, first 3 m pointing up)"); p.setAspectLocked(True)
        p.showGrid(x=True, y=True, alpha=0.15); p.setLabel("bottom", "x (m)"); p.setLabel("left", "y (m)")
        self.lv_path = p.plot(pen=pg.mkPen(PATH_COLOR, width=2))
        p.addItem(pg.ScatterPlotItem([0], [0], size=10, pen=pg.mkPen("w", width=1.5), brush=pg.mkBrush("#1f2328")))
        self.lv_now = pg.ScatterPlotItem(size=16, pen=pg.mkPen("w", width=1.5)); p.addItem(self.lv_now); self.lv_plot = p
        gl2 = pg.GraphicsLayoutWidget(); self.tabs.addTab(gl2, "Sensors (live)")
        self.lv_strips = []
        for r, (title, unit, col) in enumerate((("Gyroscope, rotation rate", "deg/s", "#2f8fa3"),
                                                ("Accelerometer, magnitude", "m/s²", "#d1495b"),
                                                ("Foot flat on the ground (zero-velocity updates apply)", "", FLAT))):
            q = gl2.addPlot(row=r, col=0, title=f"{title}  (last {livemode.STRIP_S:.0f} s)"); q.showGrid(x=True, y=True, alpha=0.2)
            q.setLabel("left", unit); q.setLabel("bottom", "time (s)")
            if r == 1: q.addItem(pg.InfiniteLine(9.81, angle=0, pen=pg.mkPen((120, 120, 120), style=QtCore.Qt.DashLine)))
            if r == 2: q.setYRange(-0.1, 1.1, padding=0)
            self.lv_strips.append((q, q.plot(pen=pg.mkPen(col, width=LINE_W))))

    def live_tick(self):
        if self.live_run is None: return
        s = self.live_run.snapshot(); P = s["p"]
        q = heading_up(P) if len(P) else None
        if q is None: q = P[:, :2] - P[0, :2] if len(P) else np.zeros((1, 2))
        tracking = s["state"] == TRACKING
        if self.tabs.currentIndex() == 0:
            self.lv_path.setData(q[:, 0], q[:, 1])
            self.lv_now.setData([q[-1, 0]], [q[-1, 1]], brush=pg.mkBrush(WAIT if not tracking else FLAT if s["still"] else SWING))
            fit_view(self.lv_plot, q)
        else:
            a = self.live_run.tracker.strips()
            if len(a):
                t = a[:, 0] - a[-1, 0]
                for (p, it), col in zip(self.lv_strips, (1, 2, 3)):
                    it.setData(t[::2], a[::2, col]); p.setXRange(-livemode.STRIP_S, 0, padding=0)
        if tracking:
            self.lv_state.setText("TRACKING"); self.lv_state.setStyleSheet("font-size:20px; font-weight:600; color:#1b7a3a")
            self.lv_help.setText("Go. The path turns to point up after 3 m. Press R to start over, C to make the current "
                                 "position the start.")
        else:
            gl, al = s["still_limits"]; m = s["motion"]
            if m is None:
                how = "waiting for data"
            else:
                how = f"shoe motion {m[0]:.3f} rad/s (needs below {gl:.2f} for 2 s)"
                if m[1] >= al: how += f", accel spread {m[1]:.2f} (needs below {al:.1f})"
                elif m[0] < gl: how += ": holding still…"
            self.lv_state.setText("CALIBRATING"); self.lv_state.setStyleSheet("font-size:20px; font-weight:600; color:#b26a00")
            self.lv_help.setText(f"The walker stands still with the phone strapped to the shoe. {how}")
        age = s["link_age"]
        if age is None:
            self.lv_link.setText("Waiting for the phone"); self.lv_link.setStyleSheet("font-weight:600; color:#b26a00")
        elif age < 1.0:
            self.lv_link.setText(f"LINK OK   {s['rate']:.0f} Hz   lost packets {s['lost']}   data gaps {s['gaps']} ({s['gap_time']:.1f} s)")
            self.lv_link.setStyleSheet("font-weight:600; font-family:Consolas,monospace; color:#1b7a3a")
        else:
            self.lv_link.setText(f"LINK LOST for {age:.0f} s (the phone keeps its own copy)")
            self.lv_link.setStyleSheet("font-weight:600; font-family:Consolas,monospace; color:#b3263a")
        self.lv_addr.setText(f"In IMU Stream, enter {self.live_run.ip} (port {self.live_run.port}), choose Fastest, press Start.")
        self.lv_stats.setText(f"Steps     {s['steps']}\nDistance  {s['distance']:.1f} m\nSpeed     {s['walk_speed']:.2f} m/s")
        code = self.live_run.session.walk[0]
        self.lv_rec.setText(f"Recording to Documents/IMU Live/{s['file']}  ({s['rows']} rows"
                            + (f", walk {code}" if code != "free" else "") + f", {self.live_run.session.markers} markers)")

    def _live_next_walk(self):
        self.live_run.session.next_walk()

    def stop_live(self):
        if self.live_run is None: return
        self.live_timer.stop(); run, self.live_run = self.live_run, None; run.stop()

    def closeEvent(self, e):
        self.stop_live(); super().closeEvent(e)


def show_crash(kind, value, tb):
    """the installed app has no console: show an unexpected error instead of failing silently"""
    import traceback
    QtWidgets.QMessageBox.critical(None, "Fusion Player error", "".join(traceback.format_exception(kind, value, tb))[-1500:])


if __name__ == "__main__":
    if engine.FROZEN: sys.excepthook = show_crash
    app = QtWidgets.QApplication(sys.argv)
    app.styleHints().setColorScheme(QtCore.Qt.ColorScheme.Light)   # light like the plots, even when Windows is dark
    app.setStyle("Fusion")
    w = Player(); w.show()
    a = sys.argv[1:]
    if a == ["--live"]:
        w.start_live()
    elif len(a) == 1:            # python app.py walk.csv  -> process a phone walk straight away
        w.process(foot.process, (a[0],))
    elif len(a) >= 2:            # python app.py sen.tar.gz groundtruth.csv  -> NCLT with the first profile
        w.process(engine.process, (a[0], a[1], engine.settings_list()[0]))
    sys.exit(app.exec())
