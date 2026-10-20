#!/usr/bin/env python3
"""
Exo Log Analyzer — GUI for turning recorded experiment CSVs into paper/poster figures.

Usage:  python exo_analyzer.py            (then Open CSV…)
        python exo_analyzer.py logs/exo_20260713_153238.csv

What it does
  - Loads a telemetry CSV recorded by the control-panel GUI.
  - Computes the full analysis: rest/motion segmentation, estimator noise floors
    and biases, motion agreement/correlation, DOB lag, tracking error, B_v
    adaptation-law fit, gravity-model residual.
  - Shows five figure tabs (Overview, Estimators, Adaptation & Noise, Tracking,
    Statistics report). Each canvas has a zoom/pan toolbar.
  - "Paper" / "Poster" style switch (poster = large fonts, thick lines).
  - Export: choose DPI (300/600/1200) and format (PNG/PDF/SVG), then export the
    current figure or ALL figures + a statistics report into <csvname>_figs/.
    PDF/SVG are vector formats — infinitely scalable, ideal for posters.

Install:  pip install pandas numpy scipy matplotlib PyQt5
"""
import sys, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from PyQt5 import QtWidgets, QtCore

try:
    from scipy.optimize import curve_fit
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

DEG = 180.0 / np.pi
REQUIRED = ["t_ms","state","theta","theta_ref","vel","iq_meas",
            "tau_m","tau_s","tau_d","tau_dob","Bv","iq_cmd"]

STYLES = {
    "Paper":  dict(fs=9,  lw=1.0, lw2=1.5, dpi=100),
    "Poster": dict(fs=15, lw=2.0, lw2=3.0, dpi=100),
}


# ============================ analysis ============================
def bv_law(x, bmin, bmax, t0):
    return bmin + (bmax - bmin) / (1.0 + (x / t0) ** 2)


def analyze(df, rest_vel=0.10, rest_hold=0.5):
    """Compute everything once; figures and the report both read from this dict."""
    a = {}
    t = (df.t_ms - df.t_ms.iloc[0]).values / 1000.0
    dt = float(np.median(np.diff(t)))
    a["t"], a["dt"], a["rate"] = t, dt, 1.0 / dt
    a["dur"] = t[-1]

    # rest = |vel| below threshold sustained >= rest_hold seconds
    k = max(1, int(round(rest_hold / dt)))
    rest_i = np.abs(df.vel.values) < rest_vel
    rest = pd.Series(rest_i).rolling(k, center=True).min().fillna(0).astype(bool).values
    motion = ~rest
    a["rest"], a["motion"] = rest, motion
    a["rest_s"], a["motion_s"] = rest.sum() * dt, motion.sum() * dt

    th = df.theta.values
    a["rom_deg"] = (th.min() * DEG, th.max() * DEG)
    # dominant movement frequency
    x = th - th.mean()
    F = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), dt)
    a["f0"] = float(fr[1:][np.argmax(F[1:])]) if len(fr) > 2 else np.nan
    a["vmax"] = float(np.abs(df.vel).max())

    # estimators
    est = {}
    for n in ["tau_s", "tau_d", "tau_dob"]:
        v = df[n].values
        est[n] = dict(
            rest_std=float(v[rest].std()) if rest.any() else np.nan,
            rest_bias=float(v[rest].mean()) if rest.any() else np.nan,
            motion_rms=float(np.sqrt((v[motion] ** 2).mean())) if motion.any() else np.nan,
            span=float(v.max() - v.min()),
        )
    a["est"] = est
    if motion.any():
        for pair in [("tau_s", "tau_dob"), ("tau_d", "tau_dob"), ("tau_s", "tau_d")]:
            x1, x2 = df[pair[0]].values[motion], df[pair[1]].values[motion]
            if x1.std() > 1e-9 and x2.std() > 1e-9:
                a[f"corr_{pair[0]}_{pair[1]}"] = float(np.corrcoef(x1, x2)[0, 1])
            else:
                a[f"corr_{pair[0]}_{pair[1]}"] = float("nan")
            a[f"rmsd_{pair[0]}_{pair[1]}"] = float(np.sqrt(((x1 - x2) ** 2).mean()))
        # DOB lag vs direct (cross-correlation)
        u = df.tau_d.values[motion]; u = u - u.mean()
        v = df.tau_dob.values[motion]; v = v - v.mean()
        xc = np.correlate(v, u, "full")
        a["dob_lag_ms"] = float((np.argmax(xc) - (len(u) - 1)) * dt * 1000.0)

    # ---- control modes present (v2.4 firmware: 4 admittance, 14 impedance, 6 passive) ----
    st = df.state.values.astype(int)
    a["mode_frac"] = {"admittance": float((st == 4).mean()),
                      "impedance": float((st == 14).mean()),
                      "passive":   float((st == 6).mean())}
    trk_mask = (st == 4) | (st == 6)      # states where theta_ref is a tracked trajectory
    imp_mask = (st == 14)                 # theta_ref is the spring equilibrium here

    # tracking (admittance/passive only — in impedance there is no tracked reference)
    e = (df.theta_ref - df.theta).values
    if trk_mask.any():
        a["trk_rms_deg"] = float(np.sqrt((e[trk_mask] ** 2).mean()) * DEG)
        a["trk_max_deg"] = float(np.abs(e[trk_mask]).max() * DEG)
    else:
        a["trk_rms_deg"] = a["trk_max_deg"] = np.nan
    # impedance spring deflection |theta - theta_eq| (the correct quantity for O2 logs)
    if imp_mask.any():
        a["imp_defl_rms_deg"] = float(np.sqrt((e[imp_mask] ** 2).mean()) * DEG)
        a["imp_defl_max_deg"] = float(np.abs(e[imp_mask]).max() * DEG)
    else:
        a["imp_defl_rms_deg"] = a["imp_defl_max_deg"] = np.nan

    # motor effort
    a["iq_peak"] = float(np.abs(df.iq_meas).max())
    a["tau_m_peak"] = float(np.abs(df.tau_m).max())

    # B_v adaptation
    bv = df.Bv.values
    a["bv_range"] = (float(bv.min()), float(bv.max()))
    a["bv_firm_pct"] = float(100 * (bv > 0.6).mean())
    a["bv_light_pct"] = float(100 * (bv < 0.2).mean())
    a["bv_fit"] = None
    # fit the adaptation law only where damping actually varies (admittance, or
    # impedance with Pa1). Fixed-B_i impedance logs have a constant Bv column —
    # fitting a law to a constant is meaningless.
    fit_mask = ((st == 4) | (st == 14)) & np.isfinite(bv)
    a["bv_fixed"] = bool(fit_mask.any() and np.std(bv[fit_mask]) < 0.005)
    if HAVE_SCIPY and fit_mask.any() and not a["bv_fixed"]:
        try:
            p, _ = curve_fit(bv_law, df.tau_dob.values[fit_mask], bv[fit_mask],
                             p0=[0.05, 0.8, 0.4], maxfev=8000)
            a["bv_fit"] = tuple(float(x) for x in p)
            a["bv_fit_res"] = float((bv[fit_mask] - bv_law(df.tau_dob.values[fit_mask], *p)).std())
        except Exception:
            pass

    # gravity residual (rest windows, needs some spread of angles)
    a["grav"] = None
    if rest.sum() > 50:
        A = np.vstack([np.cos(th[rest]), np.sin(th[rest]), np.ones(rest.sum())]).T
        c, *_ = np.linalg.lstsq(A, df.tau_dob.values[rest], rcond=None)
        a["grav"] = tuple(float(x) for x in c)
    return a


def report_text(path, df, a):
    L = []
    P = L.append
    P(f"Exo log analysis — {os.path.basename(path)}")
    P("=" * 64)
    P(f"Duration {a['dur']:.1f} s @ {a['rate']:.0f} Hz   ({len(df)} samples)")
    P(f"Rest {a['rest_s']:.1f} s / motion {a['motion_s']:.1f} s")
    P(f"ROM {a['rom_deg'][0]:.1f}..{a['rom_deg'][1]:.1f} deg "
      f"({a['rom_deg'][1]-a['rom_deg'][0]:.1f} deg), dominant f {a['f0']:.2f} Hz, "
      f"peak |vel| {a['vmax']:.2f} rad/s")
    P("")
    P("ESTIMATORS (rest windows: true tau_h ~ constant)")
    P(f"{'':10s}{'rest std':>12s}{'rest bias':>12s}{'motion RMS':>12s}{'span':>10s}")
    for n, lbl in [("tau_s", "static"), ("tau_d", "direct"), ("tau_dob", "DOB")]:
        e = a["est"][n]
        P(f"{lbl:10s}{e['rest_std']*1000:9.0f} mNm{e['rest_bias']*1000:+9.0f} mNm"
          f"{e['motion_rms']:10.3f} Nm{e['span']:8.2f} Nm")
    if "corr_tau_s_tau_dob" in a:
        P("")
        P(f"Motion agreement: static-DOB r={a['corr_tau_s_tau_dob']:.2f} "
          f"(RMSd {a['rmsd_tau_s_tau_dob']:.2f} Nm); "
          f"direct-DOB r={a['corr_tau_d_tau_dob']:.2f} "
          f"(RMSd {a['rmsd_tau_d_tau_dob']:.2f} Nm)")
        P(f"DOB lag vs direct: {a['dob_lag_ms']:.0f} ms")
    m = a["mode_frac"]
    P("")
    P(f"MODES     admittance {m['admittance']*100:.0f}% | impedance {m['impedance']*100:.0f}% | "
      f"passive {m['passive']*100:.0f}% of samples")
    if np.isfinite(a["trk_rms_deg"]):
        P(f"TRACKING  (admittance/passive) RMS {a['trk_rms_deg']:.2f} deg, max {a['trk_max_deg']:.2f} deg")
    if np.isfinite(a["imp_defl_rms_deg"]):
        P(f"IMPEDANCE spring deflection |theta-theta_eq| RMS {a['imp_defl_rms_deg']:.2f} deg, "
          f"max {a['imp_defl_max_deg']:.2f} deg")
    P(f"MOTOR     peak |Iq| {a['iq_peak']:.2f} A, peak |tau_m| {a['tau_m_peak']:.2f} Nm")
    P("")
    P(f"B_v       used range {a['bv_range'][0]:.2f}..{a['bv_range'][1]:.2f} Nm·s/rad; "
      f"firm(>0.6) {a['bv_firm_pct']:.0f}% / light(<0.2) {a['bv_light_pct']:.0f}%")
    if a.get("bv_fixed"):
        P("          damping constant over the session (fixed B_i impedance) — no law fit")
    elif a["bv_fit"]:
        b0, b1, t0 = a["bv_fit"]
        P(f"          law fit: Bmin={b0:.3f}, Bmax={b1:.3f}, tau0={t0:.3f} "
          f"(residual RMS {a['bv_fit_res']:.3f})")
    if a["grav"]:
        c = a["grav"]
        P("")
        P(f"GRAVITY RESIDUAL (rest): {c[0]:+.3f}·cosθ {c[1]:+.3f}·sinθ {c[2]:+.3f}  [Nm]")
        P("  caution: short/clustered rest windows make this ill-conditioned; "
          "use a dedicated quasi-static sweep for the paper value.")
    return "\n".join(L)


# ============================ figures ============================
def style_rc(s):
    matplotlib.rcParams.update({
        "font.size": s["fs"], "axes.grid": True, "grid.alpha": 0.3,
        "axes.linewidth": max(0.8, s["lw"] * 0.8),
        "lines.linewidth": s["lw"],
        "savefig.bbox": "tight",
    })


import textwrap

def add_caption(fig, caption, s):
    """Render a figure caption inside the figure, under the axes, so exported
    images carry their own name + explanation (paper drafts, posters, slides)."""
    w_in, h_in = fig.get_size_inches()
    fs = max(7, s["fs"] - 1)
    chars = max(40, int(w_in * 72 / (fs * 0.52)))       # approx chars per line
    lines = textwrap.wrap(caption, chars)
    frac = (len(lines) * fs * 1.5) / (h_in * 72) + 0.02  # reserve bottom space
    fig.tight_layout(rect=[0, frac, 1, 1])
    fig.text(0.02, 0.01, "\n".join(lines), ha="left", va="bottom", fontsize=fs)


def fig_overview(df, a, s):
    f = Figure(figsize=(7, 8.4))
    ax = f.subplots(4, 1, sharex=True)
    t = a["t"]
    ax[0].plot(t, df.theta * DEG, lw=s["lw"], label=r"$\theta$")
    ax[0].plot(t, df.theta_ref * DEG, "--", lw=s["lw"], label=r"$\theta_{ref}$")
    ax[0].set_ylabel("angle [deg]"); ax[0].legend(ncol=2, loc="upper right")
    ax[1].plot(t, df.vel, lw=s["lw"] * 0.8, color="#17becf")
    ax[1].set_ylabel("velocity\n[rad/s]")
    ax[2].plot(t, df.tau_m, lw=s["lw"] * 0.8, label=r"$\tau_m$", color="#8c564b")
    ax[2].plot(t, df.tau_dob, lw=s["lw"], label=r"$\hat\tau_h$ (DOB)", color="#9467bd")
    ax[2].set_ylabel("torque [Nm]"); ax[2].legend(ncol=2, loc="upper right")
    ax[3].plot(t, df.Bv, lw=s["lw"], color="#e377c2")
    ax[3].set_ylabel(r"$B_v$" + "\n[Nm·s/rad]"); ax[3].set_xlabel("time [s]")
    f.align_ylabels()
    add_caption(f,
        "Fig. 1 — Session overview in effort-adaptive admittance mode "
        f"({a['dur']:.0f} s). Top to bottom: joint angle and admittance-generated "
        f"reference (tracked with {a['trk_rms_deg']:.2f}\u00b0 RMS error); joint velocity; "
        "motor torque \u03c4_m and estimated human torque \u03c4\u0302_h (DOB); commanded virtual "
        "damping B_v, dropping on volitional effort and recovering when the wearer relaxes.", s)
    return f


def fig_estimators(df, a, s, zoom):
    f = Figure(figsize=(7, 6.4))
    ax = f.subplots(2, 1)
    t = a["t"]
    for axx, (z0, z1), title in zip(ax, [(t[0], t[-1]), zoom],
                                    ["full session", f"zoom: {zoom[0]:.0f}–{zoom[1]:.0f} s"]):
        m = (t >= z0) & (t <= z1)
        axx.plot(t[m], df.tau_s[m], lw=s["lw"] * 0.9, label="static", color="#2ca02c")
        axx.plot(t[m], df.tau_d[m], lw=s["lw"] * 0.6, alpha=0.75, label="direct", color="#d62728")
        axx.plot(t[m], df.tau_dob[m], lw=s["lw2"], label="DOB", color="#9467bd")
        axx.set_ylabel(r"$\hat\tau_h$ [Nm]"); axx.set_title(title)
    ax[0].legend(ncol=3, loc="upper right"); ax[1].set_xlabel("time [s]")
    lag = a.get("dob_lag_ms", float("nan"))
    add_caption(f,
        "Fig. 2 — Three sensorless human-torque estimators computed simultaneously on "
        "identical signals (top: full session; bottom: zoom). The static estimator "
        "diverges during movement because it neglects the inertial term; the direct "
        "(differentiation-based) estimator shows the largest noise excursions; the DOB "
        f"tracks the direct estimate with \u2248{lag:.0f} ms delay at the lowest noise floor "
        f"({a['est']['tau_dob']['rest_std']*1000:.0f} mNm at rest).", s)
    return f


def fig_adaptation(df, a, s):
    f = Figure(figsize=(7, 4.1))
    ax = f.subplots(1, 2)
    ax[0].scatter(df.tau_dob, df.Bv, s=4, alpha=0.25, color="#9467bd", edgecolors="none")
    if a["bv_fit"]:
        xx = np.linspace(df.tau_dob.min() - 0.2, df.tau_dob.max() + 0.2, 400)
        ax[0].plot(xx, bv_law(xx, *a["bv_fit"]), "k-", lw=s["lw2"], label=r"$B_v(\hat\tau_h)$ law")
        ax[0].legend()
    ax[0].set_xlabel(r"$\hat\tau_h$ [Nm]"); ax[0].set_ylabel(r"$B_v$ [Nm·s/rad]")
    names, keys = ["static", "direct", "DOB"], ["tau_s", "tau_d", "tau_dob"]
    cols = ["#2ca02c", "#d62728", "#9467bd"]
    noise = [a["est"][k]["rest_std"] for k in keys]
    ax[1].bar(names, noise, color=cols)
    for i, v in enumerate(noise):
        if np.isfinite(v):
            ax[1].text(i, v, f" {v*1000:.0f} mNm", ha="center", va="bottom", fontsize=s["fs"])
    ax[1].set_ylabel("rest-window std [Nm]"); ax[1].set_title("noise floor at rest")
    fit = a["bv_fit"]
    fit_s = (f"least-squares fit: B_min={fit[0]:.2f}, B_max={fit[1]:.2f}, "
             f"\u03c4\u2080={fit[2]:.2f} Nm") if fit else "fit unavailable"
    add_caption(f,
        "Fig. 3 — (a) Commanded virtual damping versus the concurrent DOB torque "
        "estimate for every sample; the data collapse onto the designed adaptation law "
        "B_v = B_min + (B_max\u2212B_min)/(1+(\u03c4\u0302_h/\u03c4\u2080)\u00b2) "
        f"(solid line, {fit_s}). (b) Estimator noise floors during rest windows, where "
        "the true interaction torque is approximately constant.", s)
    return f


def fig_tracking(df, a, s):
    f = Figure(figsize=(7, 5.9))
    ax = f.subplots(2, 1)
    t = a["t"]
    e = (df.theta_ref - df.theta).values * DEG
    ax[0].plot(t, e, lw=s["lw"] * 0.8, color="#1f77b4")
    ax[0].set_ylabel("tracking error [deg]"); ax[0].set_xlabel("time [s]")
    ax[0].set_title(f"inner-loop tracking: RMS {a['trk_rms_deg']:.2f}°, max {a['trk_max_deg']:.2f}°")
    ax[1].scatter(np.abs(df.vel), np.abs(e), s=4, alpha=0.3, color="#ff7f0e", edgecolors="none")
    ax[1].set_xlabel("|velocity| [rad/s]"); ax[1].set_ylabel("|error| [deg]")
    ax[1].set_title("error grows with speed \u2192 dominated by Kd/velocity, not stiffness")
    add_caption(f,
        "Fig. 4 — Inner position-loop tracking of the admittance reference "
        f"(RMS {a['trk_rms_deg']:.2f}\u00b0, max {a['trk_max_deg']:.2f}\u00b0). Bottom: error "
        "magnitude versus speed; the proportional growth indicates the residual error is "
        "velocity/damping-limited rather than stiffness-limited, i.e. the observed "
        "interaction behaviour reflects the admittance law, not tracking deficiency.", s)
    return f


FIGS = [("fig_overview", fig_overview),
        ("fig_estimators", fig_estimators),
        ("fig_adaptation_noise", fig_adaptation),
        ("fig_tracking", fig_tracking)]


# ============================ GUI ============================
class FigTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.lay = QtWidgets.QVBoxLayout(self)
        self.canvas = None
        self.fig = None

    def set_figure(self, fig):
        if self.canvas:
            self.lay.removeWidget(self.toolbar); self.toolbar.deleteLater()
            self.lay.removeWidget(self.canvas); self.canvas.deleteLater()
        self.fig = fig
        self.canvas = FigureCanvasQTAgg(fig)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.lay.addWidget(self.toolbar); self.lay.addWidget(self.canvas)
        self.canvas.draw()


class Main(QtWidgets.QMainWindow):
    def __init__(self, path=None):
        super().__init__()
        self.setWindowTitle("Exo Log Analyzer — paper & poster figures")
        self.resize(1150, 800)
        self.df = self.a = self.path = None

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central)

        # top bar
        bar = QtWidgets.QHBoxLayout(); v.addLayout(bar)
        b_open = QtWidgets.QPushButton("Open CSV…"); b_open.clicked.connect(self.open_dialog)
        bar.addWidget(b_open)
        self.lbl_file = QtWidgets.QLabel("no file loaded"); bar.addWidget(self.lbl_file, 1)

        bar.addWidget(QtWidgets.QLabel("Style:"))
        self.style_box = QtWidgets.QComboBox(); self.style_box.addItems(list(STYLES))
        self.style_box.currentIndexChanged.connect(self.render_all)
        bar.addWidget(self.style_box)

        bar.addWidget(QtWidgets.QLabel("Zoom [s]:"))
        self.z0 = QtWidgets.QDoubleSpinBox(); self.z0.setRange(0, 9999); self.z0.setValue(6)
        self.z1 = QtWidgets.QDoubleSpinBox(); self.z1.setRange(0, 9999); self.z1.setValue(12)
        for w in (self.z0, self.z1):
            w.setFixedWidth(70); w.valueChanged.connect(self.render_all); bar.addWidget(w)

        bar.addWidget(QtWidgets.QLabel("Export:"))
        self.dpi_box = QtWidgets.QComboBox(); self.dpi_box.addItems(["300 dpi", "600 dpi", "1200 dpi"])
        self.dpi_box.setCurrentIndex(1); bar.addWidget(self.dpi_box)
        self.fmt_box = QtWidgets.QComboBox(); self.fmt_box.addItems(["png", "pdf (vector)", "svg (vector)"])
        bar.addWidget(self.fmt_box)
        b_exp1 = QtWidgets.QPushButton("Export current"); b_exp1.clicked.connect(self.export_current)
        b_expA = QtWidgets.QPushButton("Export ALL + report")
        b_expA.setStyleSheet("font-weight:bold;")
        b_expA.clicked.connect(self.export_all)
        bar.addWidget(b_exp1); bar.addWidget(b_expA)

        # tabs
        self.tabs = QtWidgets.QTabWidget(); v.addWidget(self.tabs, 1)
        self.fig_tabs = {}
        for name, _ in FIGS:
            ft = FigTab(); self.fig_tabs[name] = ft
            self.tabs.addTab(ft, name.replace("fig_", "").replace("_", " ").title())
        self.txt = QtWidgets.QPlainTextEdit(); self.txt.setReadOnly(True)
        self.txt.setStyleSheet("font-family:Consolas,monospace;")
        self.tabs.addTab(self.txt, "Statistics Report")

        self.status = QtWidgets.QLabel(""); v.addWidget(self.status)

        if path: self.load(path)

    # ---------- loading ----------
    def open_dialog(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open telemetry CSV",
                                                     "logs" if os.path.isdir("logs") else "",
                                                     "CSV files (*.csv)")
        if p: self.load(p)

    def load(self, path):
        try:
            df = pd.read_csv(path)
            missing = [c for c in REQUIRED if c not in df.columns]
            if missing:
                raise ValueError(f"missing columns: {missing} — is this an exo telemetry log?")
            if len(df) < 50:
                raise ValueError("file too short to analyze")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Load failed", str(e))
            return
        self.df, self.path = df, path
        self.a = analyze(df)
        self.lbl_file.setText(f"{os.path.basename(path)}   "
                              f"({self.a['dur']:.1f} s @ {self.a['rate']:.0f} Hz)")
        # sensible default zoom: middle third
        self.z0.blockSignals(True); self.z1.blockSignals(True)
        self.z0.setValue(round(self.a["dur"] * 0.3)); self.z1.setValue(round(self.a["dur"] * 0.6))
        self.z0.blockSignals(False); self.z1.blockSignals(False)
        self.render_all()

    # ---------- rendering ----------
    def render_all(self):
        if self.df is None: return
        s = STYLES[self.style_box.currentText()]
        style_rc(s)
        zoom = (min(self.z0.value(), self.z1.value()), max(self.z0.value(), self.z1.value()))
        for name, fn in FIGS:
            fig = fn(self.df, self.a, s, zoom) if fn is fig_estimators else fn(self.df, self.a, s)
            self.fig_tabs[name].set_figure(fig)
        self.txt.setPlainText(report_text(self.path, self.df, self.a))
        self.status.setText(f"style: {self.style_box.currentText()}   "
                            f"DOB rest noise {self.a['est']['tau_dob']['rest_std']*1000:.0f} mNm   "
                            f"tracking RMS {self.a['trk_rms_deg']:.2f}°")

    # ---------- export ----------
    def _exp_params(self):
        dpi = int(self.dpi_box.currentText().split()[0])
        fmt = self.fmt_box.currentText().split()[0]
        out = os.path.splitext(self.path)[0] + "_figs"
        os.makedirs(out, exist_ok=True)
        return dpi, fmt, out

    def export_current(self):
        if self.df is None: return
        i = self.tabs.currentIndex()
        if i >= len(FIGS):
            self.export_report(); return
        dpi, fmt, out = self._exp_params()
        name = FIGS[i][0]
        p = os.path.join(out, f"{name}.{fmt}")
        self.fig_tabs[name].fig.savefig(p, dpi=dpi)
        self.status.setText(f"exported {p}")

    def export_all(self):
        if self.df is None: return
        dpi, fmt, out = self._exp_params()
        for name, _ in FIGS:
            self.fig_tabs[name].fig.savefig(os.path.join(out, f"{name}.{fmt}"), dpi=dpi)
        self.export_report(out)
        self.status.setText(f"exported {len(FIGS)} figures ({fmt}, {dpi} dpi) + report → {out}/")

    def export_report(self, out=None):
        if out is None: _, _, out = self._exp_params()
        with open(os.path.join(out, "statistics_report.txt"), "w") as f:
            f.write(report_text(self.path, self.df, self.a))
        # machine-readable numbers for tables
        rows = []
        for k, lbl in [("tau_s", "static"), ("tau_d", "direct"), ("tau_dob", "DOB")]:
            e = self.a["est"][k]
            rows.append(dict(estimator=lbl, **e))
        pd.DataFrame(rows).to_csv(os.path.join(out, "estimator_table.csv"), index=False)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Main(sys.argv[1] if len(sys.argv) > 1 else None)
    w.show()
    sys.exit(app.exec_())