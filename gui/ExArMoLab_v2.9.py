#!/usr/bin/env python3
"""
ExArMo Lab v2.9 — experiment runner for the Rehab firmware v2.9 (full device, arm support on).

This is the lab companion of ExArMoGui_v2.9.py. It speaks ONLY the firmware-v2.9 serial
protocol (H, O0/O1, E, A, G, V, D, L, P<letter>, M<SimpleFOC>) and automates the
experiments the thesis still needs on the finished device:

  Gravity   (E-13a)  hold the arm at several angles with nobody touching it → τ_m(θ) at rest
                     → g, φ of G(θ) = g·cos(θ+φ) fitted from MEASURED current, hysteresis per
                     station, and the no-load noise floor σ₀ of all three estimators.
  Plant ID  (E-13b)  motor-driven passive sweeps at several speeds/accelerations, nobody
                     touching → least-squares fit of τ_m = J·α + B·ω + g·cos(θ+φ) + τ_f·sgn(ω)
                     → J, B, τ_f, g, φ with standard errors, plus the "motor moving, no human"
                     bias and noise of static / direct / DOB (E8 condition d).
  Estimator (E8)     live statistics of the three estimators (noise, agreement, DOB lag) and,
                     in RUN, the interaction-torque fit τ_m − G(θ) = B_int·ω + C·sgn(ω) with
                     the net motor power — the transparency / assistance number. A mode
                     protocol steps the assist gain through a list with markers.
  Thermal            (1) IDLE phantom-current check: T_est must FALL in IDLE (nothing is
                     heating). (2) Stall test at a set current (arm blocked by the E9 strut or a
                     hard stop) → T_est against the model's own prediction (C_th, R_th, T_amb).

Every experiment records its own rows, writes logs/lab_<exp>_<time>.json with the results,
and can push the fitted values to the board (Pg Pf Pj Pb Pc). Manual recording writes the
same 13-column CSV + .meta.txt as the rehab control panel, so analyze_exo_session.py and
ExArMoLogAnalyzer work on both.

Install:  pip install pyserial pyqtgraph PyQt5 numpy
Run:      python ExArMoLab_v2.9.py
"""
import sys, os, csv, time, json, math, re, threading, collections
import numpy as np
import serial
from serial.tools import list_ports
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

GUI_VERSION = "Lab v2.9 (18 Oct 2026)"
BAUD = 921600
FIELDS = ["t_ms","state","theta","theta_ref","vel","iq_meas",
          "tau_m","tau_s","tau_d","tau_dob","Bv","iq_cmd","T_est"]
COL = {f: i for i, f in enumerate(FIELDS)}
N_FIELDS_OLD = 12
STATE_NAMES  = {0:"IDLE",1:"HOMING · LIFT ↑",2:"HOMING · LOWER ↓",3:"PARKING",4:"RUN · ADMITTANCE",5:"FAULT",6:"PASSIVE",7:"LAB · VELOCITY"}
STATE_COLORS = {0:"#666666",1:"#c77d00",2:"#c77d00",3:"#c77d00",4:"#1a8a3c",5:"#c40000",6:"#6a5acd",7:"#0e7c86"}
ST_IDLE, ST_RUN, ST_FAULT, ST_PASSIVE, ST_LAB = 0, 4, 5, 6, 7
HISTORY_S = 15.0
DEG = 180.0/np.pi

# firmware v2.9 thermal model constants (main.cpp) — used to PREDICT, never sent
TH_C, TH_RTH, TH_AMB = 14.0, 7.8, 28.0
TH_DERATE_LO, TH_DERATE_HI = 70.0, 80.0

# label, firmware letter, min, max, step, default, decimals, tooltip, (optional) scale sent = value*scale
PARAMS = [
    ("Gravity coeff g [Nm]",      "g", 0.0, 5.0,   0.01, 0.60, 2, "G(θ) = g·cos(θ+φ). The Gravity tab fits this from measured current."),
    ("Gravity phase φ [rad]",     "f", -3.2, 3.2,  0.01, 0.00, 2, "Phase of the gravity model."),
    ("Inertia J [kg·m²]",         "j", 0.001, 0.2, 0.001, 0.015, 3, "Reflected inertia at the joint (incl. rotor × N²). Plant-ID tab fits it."),
    ("Damping B [Nm·s/rad]",      "b", 0.0, 1.0,   0.005, 0.05, 3, "Viscous term of the joint model."),
    ("Coulomb friction τ_f [Nm]", "c", 0.0, 0.5,   0.01, 0.15, 2, "Smooth-sign friction term (estimators + passive feedforward)."),
    ("DOB bandwidth ω_c [rad/s]", "w", 5.0, 150.0, 1.0, 30.0, 0, "DOB Q-filter cutoff."),
    ("Dead-zone τ_dz [Nm]",       "z", 0.0, 1.0,   0.01, 0.15, 2, "Admittance dead-zone (does not affect the logged estimates)."),
    ("K_t,j joint gain [Nm/A]",   "K", 0.2, 2.0,   0.005, 0.775, 3, "τ_m = K_t,j·I_q (E9: 0.775)."),
    ("Passive speed [rad/s]",     "v", 0.05, 1.5,  0.05, 0.50, 2, "Cruise speed of G moves (Plant-ID sets it per sweep)."),
    ("Passive accel [rad/s²]",    "a", 0.05, 5.0,  0.05, 1.20, 2, "Profile acceleration of G moves (Plant-ID sets it per sweep)."),
    ("Position Kp [Nm/rad]",      "p", 1.0, 100.0, 1.0, 20.0, 1, "Inner-loop stiffness."),
    ("Position Kd [Nm·s/rad]",    "d", 0.0, 5.0,   0.05, 0.5, 2, "Inner-loop damping."),
    ("Phase resistance R [Ω]",    "r", 0.05, 5.0,  0.005, 0.47, 3, "E01: 0.472 / 0.496 / 0.485 Ω. Used by the thermal model (P = 1.5·I²·R); re-tunes the current loop when sent."),
    ("Iq limit [A]",              "i", 0.5, 15.0,  0.5, 6.0, 1, "Absolute current clamp (thermal derate goes below it)."),
    ("Continuous Iq [A]",         "u", 0.5, 8.0,   0.1, 3.0, 1, "Thermal derate floor."),
    ("Park angle [rad]",          "k", 0.0, 2.6,   0.01, 0.20, 2, "Where experiments park and release the arm at the end."),
]


# =====================================================================================
#  analysis helpers (pure numpy — also importable from scripts / tests)
# =====================================================================================
def _lstsq_se(A, y):
    """Least squares with standard errors, R² and RMSE."""
    A = np.asarray(A, float); y = np.asarray(y, float)
    c, res, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ c; r = y - yhat
    n, p = A.shape
    rmse = float(np.sqrt(np.mean(r**2)))
    ss_tot = float(np.sum((y - y.mean())**2))
    r2 = 1.0 - float(np.sum(r**2))/ss_tot if ss_tot > 0 else float("nan")
    try:
        cov = np.linalg.inv(A.T @ A) * (np.sum(r**2)/max(n - p, 1))
        se = np.sqrt(np.maximum(np.diag(cov), 0))
    except np.linalg.LinAlgError:
        se = np.full(p, np.nan)
    return c, se, r2, rmse


def fit_gravity_stations(stations):
    """stations: list of dicts with theta (rad), tau_m, tau_s, tau_d, tau_dob (means) and sd_* .
    Fits τ_m(θ) at rest = a·cosθ + b·sinθ  →  g = √(a²+b²), φ = atan2(−b, a)."""
    th = np.array([s["theta"] for s in stations]); tm = np.array([s["tau_m"] for s in stations])
    A = np.c_[np.cos(th), np.sin(th)]
    (a, b), se, r2, rmse = _lstsq_se(A, tm)
    g = math.hypot(a, b); phi = math.atan2(-b, a)
    # hysteresis: stations visited from both directions share (almost) the same angle
    hyst = []
    used = set()
    for i, s in enumerate(stations):
        for j in range(i+1, len(stations)):
            if j in used or i in used: continue
            if abs(stations[j]["theta"] - s["theta"]) < 0.03:
                hyst.append(0.5*abs(stations[j]["tau_m"] - s["tau_m"])); used.update((i, j)); break
    return {"g": g, "phi": phi, "a": a, "b": b, "se_a": float(se[0]), "se_b": float(se[1]),
            "r2": r2, "rmse": rmse,
            "hysteresis_half_Nm": float(np.mean(hyst)) if hyst else float("nan"),
            "sigma0_dob": float(np.mean([s["sd_tau_dob"] for s in stations])),
            "sigma0_static": float(np.mean([s["sd_tau_s"] for s in stations])),
            "sigma0_direct": float(np.mean([s["sd_tau_d"] for s in stations])),
            "bias_dob": float(np.mean([s["tau_dob"] for s in stations]))}


def _smooth(x, n):
    n = max(int(n), 1)
    if n <= 1 or len(x) < n: return np.asarray(x, float)
    k = np.ones(n)/n
    return np.convolve(np.asarray(x, float), k, mode="same")


def fit_plant(rows, smooth_ms=50.0):
    """rows: array (N×13) of telemetry taken while the MOTOR moves the arm and nobody touches it.
    Fits τ_m = J·α + B·ω + a·cosθ + b·sinθ + τ_f·sgn(ω)  (τ_h = 0)."""
    a = np.asarray(rows, float)
    t = a[:, COL["t_ms"]]/1000.0
    if len(a) < 50: raise ValueError("not enough samples")
    dt = float(np.median(np.diff(t))) or 0.0035
    n = max(int(round(smooth_ms/1000.0/dt)), 1)
    vel = _smooth(a[:, COL["vel"]], n)
    acc = _smooth(np.gradient(vel, t), n)
    th = a[:, COL["theta"]]; tm = a[:, COL["tau_m"]]
    sg = np.clip(vel/0.05, -1, 1)                       # same smooth sign as the firmware
    A = np.c_[acc, vel, np.cos(th), np.sin(th), sg]
    c, se, r2, rmse = _lstsq_se(A, tm)
    J, B, ca, cb, tf = c
    moving = np.abs(a[:, COL["vel"]]) > 0.05
    out = {"J": float(J), "B": float(B), "g": math.hypot(ca, cb), "phi": math.atan2(-cb, ca),
           "tau_f": float(tf), "se_J": float(se[0]), "se_B": float(se[1]), "se_tau_f": float(se[4]),
           "r2": r2, "rmse": rmse, "n": int(len(a)), "dt_ms": dt*1000,
           "acc_rms": float(np.sqrt(np.mean(acc**2))), "vel_rms": float(np.sqrt(np.mean(vel**2)))}
    for key in ("tau_s", "tau_d", "tau_dob"):
        x = a[moving, COL[key]] if moving.any() else a[:, COL[key]]
        out[f"noload_bias_{key}"] = float(np.mean(x)); out[f"noload_sd_{key}"] = float(np.std(x))
    return out


def estimator_stats(rows, g=None, phi=None):
    """Live E8 numbers on a window of rows; interaction fit when the window is in RUN."""
    a = np.asarray(rows, float)
    if len(a) < 30: return None
    t = a[:, COL["t_ms"]]/1000.0; dt = float(np.median(np.diff(t))) or 0.0035
    out = {"n": int(len(a)), "rate_hz": 1.0/dt}
    for key in ("tau_s", "tau_d", "tau_dob"):
        out[f"noise_{key}"] = float(np.std(np.diff(a[:, COL[key]])))
    for x, y in (("tau_s", "tau_d"), ("tau_s", "tau_dob"), ("tau_d", "tau_dob")):
        out[f"rmse_{x}_{y}"] = float(np.sqrt(np.mean((a[:, COL[x]] - a[:, COL[y]])**2)))
    s = a[:, COL["tau_s"]] - a[:, COL["tau_s"]].mean(); d = a[:, COL["tau_dob"]] - a[:, COL["tau_dob"]].mean()
    if np.std(s) > 1e-6 and np.std(d) > 1e-6:
        best = (-2, 0)
        for L in range(-40, 41):
            x = s[max(0, L):len(s)+min(0, L)]; y = d[max(0, -L):len(d)+min(0, -L)]
            if len(x) > 10:
                r = float(np.corrcoef(x, y)[0, 1])
                if r > best[0]: best = (r, L)
        out["lag_dob_ms"] = best[1]*dt*1000; out["corr_dob_static"] = best[0]
    st = a[:, COL["state"]]
    if (st == ST_RUN).mean() > 0.9 and g is not None:
        th = a[:, COL["theta"]]; vel = a[:, COL["vel"]]
        ex = a[:, COL["tau_m"]] - g*np.cos(th + (phi or 0.0))
        if np.std(vel) > 0.05:
            A = np.c_[vel, np.sign(vel), np.ones(len(a))]
            c, se, r2, rmse = _lstsq_se(A, ex)
            out.update({"B_int": float(c[0]), "C_int": float(c[1]), "r2_int": r2,
                        "power_W": float(np.mean(ex*vel)),
                        "track_rms_deg": float(np.sqrt(np.mean((th - a[:, COL["theta_ref"]])**2)))*DEG,
                        "speed_p95_deg_s": float(np.quantile(np.abs(vel), 0.95))*DEG})
    return out


def thermal_predict(t, T0, I, R):
    """Firmware model: dT/dt = (1.5·I²·R − (T−T_amb)/R_th)/C_th."""
    Tss = TH_AMB + 1.5*I*I*R*TH_RTH
    tau = TH_RTH*TH_C
    return Tss + (T0 - Tss)*np.exp(-np.asarray(t, float)/tau), Tss, tau


def fit_thermal(rows):
    """Fit T_est(t) = Tss + (T0−Tss)·exp(−t/τ) by grid over τ (linear in Tss, T0)."""
    a = np.asarray(rows, float)
    t = (a[:, COL["t_ms"]] - a[0, COL["t_ms"]])/1000.0; T = a[:, COL["T_est"]]
    best = None
    for tau in np.linspace(20, 400, 191):
        e = np.exp(-t/tau); A = np.c_[1 - e, e]
        c, _, r2, rmse = _lstsq_se(A, T)
        if best is None or rmse < best[3]: best = (tau, c[0], c[1], rmse, r2)
    tau, Tss, T0, rmse, r2 = best
    return {"tau_s": float(tau), "T_ss": float(Tss), "T0": float(T0), "rmse": rmse, "r2": r2,
            "iq_mean": float(np.mean(np.abs(a[:, COL["iq_meas"]]))), "duration_s": float(t[-1]),
            "T_start": float(T[0]), "T_end": float(T[-1])}


# =====================================================================================
#  serial
# =====================================================================================
class SerialWorker(QtCore.QThread):
    batch     = QtCore.pyqtSignal(list)
    textline  = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.port = None; self.ser = None; self.running = True
        self.tx_lock = threading.Lock(); self._rx = b""

    def open_port(self, port): self.port = port
    def close_port(self): self.port = None

    def run(self):
        pending, last_emit = [], 0.0
        while self.running:
            if self.port is None:
                if self.ser:
                    try: self.ser.close()
                    except Exception: pass
                    self.ser = None; self.connected.emit(False)
                time.sleep(0.1); continue
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, BAUD, timeout=0.02)
                    self._rx = b""; self.connected.emit(True)
                    self.textline.emit(f"✓ connected to {self.port}")
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    self._rx += data
                    *lines, self._rx = self._rx.split(b"\n")
                    for raw in lines:
                        line = raw.decode(errors="replace").strip()
                        if not line: continue
                        if line.startswith("D,"):
                            parts = line.split(",")[1:]
                            if len(parts) == N_FIELDS_OLD: parts = parts + ["nan"]
                            if len(parts) == len(FIELDS):
                                try: pending.append([float(p) for p in parts])
                                except ValueError: pass
                        else:
                            self.textline.emit(line)
                now = time.monotonic()
                if pending and (now - last_emit > 0.05 or len(pending) > 300):
                    self.batch.emit(pending); pending, last_emit = [], now
            except (serial.SerialException, OSError) as e:
                self.textline.emit(f"✗ serial error: {e}")
                try:
                    if self.ser: self.ser.close()
                except Exception: pass
                self.ser = None; self.connected.emit(False); time.sleep(1.0)

    def send(self, cmd):
        with self.tx_lock:
            if self.ser:
                try: self.ser.write((cmd.strip() + "\n").encode())
                except Exception: pass

    def stop(self):
        self.running = False; self.wait(1000)
        if self.ser:
            try: self.ser.close()
            except Exception: pass


class ParamRow(QtWidgets.QWidget):
    """Slider + spinbox; sends P<letter><value> 300 ms after the last change."""
    def __init__(self, label, letter, lo, hi, step, default, decimals, tip, send_fn, scale_out=1.0):
        super().__init__()
        self.label, self.letter, self.send_fn, self.decimals, self.scale_out = label, letter, send_fn, decimals, scale_out
        self.setToolTip(tip)
        lay = QtWidgets.QGridLayout(self); lay.setContentsMargins(0, 2, 0, 2)
        lay.addWidget(QtWidgets.QLabel(label), 0, 0, 1, 2)
        self.scale = 10 ** decimals if decimals > 0 else 1
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(int(lo*self.scale), int(hi*self.scale))
        self.spin = QtWidgets.QDoubleSpinBox(); self.spin.setRange(lo, hi); self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals); self.spin.setValue(default); self.spin.setFixedWidth(90)
        self.slider.setValue(int(default*self.scale))
        lay.addWidget(self.slider, 1, 0); lay.addWidget(self.spin, 1, 1)
        self._deb = QtCore.QTimer(self); self._deb.setSingleShot(True); self._deb.setInterval(300); self._deb.timeout.connect(self._emit)
        self._syncing = False
        self.slider.valueChanged.connect(self._from_slider); self.spin.valueChanged.connect(self._from_spin)

    def _from_slider(self, v):
        if self._syncing: return
        self._syncing = True; self.spin.setValue(v/self.scale); self._syncing = False; self._deb.start()

    def _from_spin(self, v):
        if self._syncing: return
        self._syncing = True; self.slider.setValue(int(round(v*self.scale))); self._syncing = False; self._deb.start()

    def value(self): return self.spin.value()

    def set_value(self, v, send=True):
        """Programmatic update (e.g. 'apply fitted g'). Sends immediately when send=True."""
        self._syncing = True
        self.spin.setValue(v); self.slider.setValue(int(round(v*self.scale)))
        self._syncing = False
        if send: self._emit()

    def _emit(self):
        v = self.spin.value()*self.scale_out
        txt = f"{v:.{self.decimals}f}" if self.scale_out == 1.0 else f"{v:.6g}"
        self.send_fn(f"P{self.letter}{txt}")

    def push(self): self._emit()


# =====================================================================================
#  main window
# =====================================================================================
class Main(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"ExArMo {GUI_VERSION} — experiments for firmware v2.9")
        self.resize(1500, 920)
        pg.setConfigOptions(antialias=True, background="w", foreground="k")

        self.rows = collections.deque(maxlen=6000)
        self.t0 = None; self.last_t_ms = None; self.last_row = None
        self.recording = False; self.csv_file = self.csv_writer = None; self.n_recorded = 0; self.last_log = None
        self.last_status = ""; self.fw_ok = False; self.lim_deg = (3.0, 127.0); self.iq_lim_board = float("nan")
        self.markers = []; self.param = {}
        self.exp = None                      # running experiment dict
        self.exp_rows = []                   # rows captured by the running experiment
        self.results = {}                    # last result per experiment (goes into the meta file)

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # ---------------- left panel ----------------
        panel = QtWidgets.QWidget(); panel.setFixedWidth(420)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setFixedWidth(440); scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left = QtWidgets.QVBoxLayout(panel); root.addWidget(scroll, 0)

        g = QtWidgets.QGroupBox("Connection"); left.addWidget(g); gl = QtWidgets.QHBoxLayout(g)
        self.port_box = QtWidgets.QComboBox(); gl.addWidget(self.port_box, 1)
        b = QtWidgets.QToolButton(); b.setText("⟳"); b.clicked.connect(self.scan_ports); gl.addWidget(b)
        self.btn_conn = QtWidgets.QPushButton("Connect"); self.btn_conn.clicked.connect(self.toggle_conn); gl.addWidget(self.btn_conn)

        self.lbl_state = QtWidgets.QLabel("NOT CONNECTED"); self.lbl_state.setAlignment(QtCore.Qt.AlignCenter)
        self._state_style("#666"); left.addWidget(self.lbl_state)
        self.lbl_readout = QtWidgets.QLabel("θ = —   τ̂_DOB = —   I_q = —"); self.lbl_readout.setAlignment(QtCore.Qt.AlignCenter)
        f = self.lbl_readout.font(); f.setPointSize(11); self.lbl_readout.setFont(f); left.addWidget(self.lbl_readout)
        self.lbl_temp = QtWidgets.QLabel("T_est = —"); self.lbl_temp.setAlignment(QtCore.Qt.AlignCenter); left.addWidget(self.lbl_temp)
        self.lbl_fw = QtWidgets.QLabel("firmware: —"); self.lbl_fw.setAlignment(QtCore.Qt.AlignCenter); left.addWidget(self.lbl_fw)

        g = QtWidgets.QGroupBox("Workflow"); left.addWidget(g); gl = QtWidgets.QGridLayout(g)
        self.btn_home = QtWidgets.QPushButton("1. Home"); self.btn_home.setMinimumHeight(32); self.btn_home.clicked.connect(lambda: self.send("H")); gl.addWidget(self.btn_home, 0, 0)
        self.btn_stat = QtWidgets.QPushButton("Status (L)"); self.btn_stat.clicked.connect(lambda: self.send("L")); gl.addWidget(self.btn_stat, 0, 1)
        self.btn_idle = QtWidgets.QPushButton("STOP / Idle (O0)"); self.btn_idle.setMinimumHeight(32)
        self.btn_idle.setStyleSheet("color:#c40000; font-weight:bold;"); self.btn_idle.clicked.connect(self.stop_all); gl.addWidget(self.btn_idle, 0, 2)
        b = QtWidgets.QPushButton("Hold: RUN with A = 0"); b.setToolTip("Gravity-compensated hold (admittance with assist 0): the arm stays where it is, nobody touching → watch τ̂ drift.")
        b.clicked.connect(lambda: (self.send("A0"), self.send("O1"))); gl.addWidget(b, 1, 0, 1, 2)
        b = QtWidgets.QPushButton("Go to park (G)"); b.clicked.connect(lambda: self.send(f"G{self.pv('k'):.3f}")); gl.addWidget(b, 1, 2)
        # [fw v2.9z] manual zero + full auto homing
        b = QtWidgets.QPushButton("0 here (Z)"); b.setStyleSheet("font-weight:bold;")
        b.setToolTip("Stops the motor, then the arm's CURRENT position becomes 0°.\nPut the arm on the bottom stop by hand first, then press 1. Home (lift only).")
        b.clicked.connect(lambda: (self.send("O0"), self.send("Z"))); gl.addWidget(b, 2, 0)
        b = QtWidgets.QPushButton("Full auto home (H1)"); b.setToolTip("Push down to the bottom stop, then up. Replaces the manual zero.")
        b.clicked.connect(lambda: self.send("H1")); gl.addWidget(b, 2, 1, 1, 2)

        self.tabs = QtWidgets.QTabWidget(); left.addWidget(self.tabs, 1)
        self.tabs.addTab(self._tab_gravity(), "Gravity")
        self.tabs.addTab(self._tab_plant(), "Plant ID")
        self.tabs.addTab(self._tab_estimator(), "Estimator")
        self.tabs.addTab(self._tab_thermal(), "Thermal")
        self.tabs.addTab(self._tab_params(), "Params")

        g = QtWidgets.QGroupBox("Recording"); left.addWidget(g); gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("Rate:"), 0, 0)
        self.rate_box = QtWidgets.QComboBox()
        self.rates = [("500 Hz (D2) — board delivers ≈280 Hz", "2"), ("250 Hz (D4)", "4"), ("100 Hz (D10)", "10"), ("Off (D0)", "0")]
        self.rate_box.addItems([r[0] for r in self.rates]); self.rate_box.currentIndexChanged.connect(lambda i: self.send("D" + self.rates[i][1]))
        gl.addWidget(self.rate_box, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Note:"), 1, 0); self.note = QtWidgets.QLineEdit(); self.note.setPlaceholderText("motor serial / arm support / subject / what this is"); gl.addWidget(self.note, 1, 1)
        self.btn_rec = QtWidgets.QPushButton("●  Record"); self.btn_rec.setStyleSheet("color:#c40000; font-weight:bold;"); self.btn_rec.clicked.connect(self.toggle_record); gl.addWidget(self.btn_rec, 2, 0, 1, 2)
        self.lbl_rec = QtWidgets.QLabel(""); gl.addWidget(self.lbl_rec, 3, 0, 1, 2)
        b = QtWidgets.QPushButton("Add marker"); b.clicked.connect(lambda: self.add_marker(self.note.text() or "marker")); gl.addWidget(b, 4, 0, 1, 2)

        # ---------------- right: plots + console ----------------
        right = QtWidgets.QVBoxLayout(); root.addLayout(right, 1)
        grid = pg.GraphicsLayoutWidget(); right.addWidget(grid, 1)
        p1 = grid.addPlot(row=0, col=0, title="Joint angle / deg"); p1.addLegend(offset=(10, 10)); p1.showGrid(x=True, y=True, alpha=0.3)
        self.c_theta = p1.plot(pen=pg.mkPen('#1f77b4', width=2), name="θ"); self.c_ref = p1.plot(pen=pg.mkPen('#ff7f0e', width=2, style=QtCore.Qt.DashLine), name="θ_ref")
        p2 = grid.addPlot(row=0, col=1, title="τ̂_h / N·m"); p2.addLegend(offset=(10, 10)); p2.showGrid(x=True, y=True, alpha=0.3)
        self.c_ts = p2.plot(pen=pg.mkPen('#2ca02c', width=1.2), name="static"); self.c_td = p2.plot(pen=pg.mkPen('#d62728', width=0.8), name="direct"); self.c_tdob = p2.plot(pen=pg.mkPen('#9467bd', width=2.2), name="DOB")
        p3 = grid.addPlot(row=1, col=0, title="vel / rad·s⁻¹  ·  τ_m / N·m"); p3.addLegend(offset=(10, 10)); p3.showGrid(x=True, y=True, alpha=0.3)
        self.c_vel = p3.plot(pen=pg.mkPen('#17becf', width=1.5), name="vel"); self.c_taum = p3.plot(pen=pg.mkPen('#8c564b', width=1.5), name="τ_m")
        p4 = grid.addPlot(row=1, col=1, title="I_q / A  ·  T_est / °C ÷ 100"); p4.addLegend(offset=(10, 10)); p4.showGrid(x=True, y=True, alpha=0.3)
        self.c_iqm = p4.plot(pen=pg.mkPen('#7f7f7f', width=1.2), name="I_q meas"); self.c_iqc = p4.plot(pen=pg.mkPen('#000000', width=1.0, style=QtCore.Qt.DashLine), name="I_q cmd")
        self.c_T = p4.plot(pen=pg.mkPen('#c40000', width=1.5), name="T_est/100")
        for p in (p2, p3, p4): p.setXLink(p1)

        self.console = QtWidgets.QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumHeight(140)
        self.console.setStyleSheet("font-family:Consolas,Menlo,monospace; font-size:11px;"); right.addWidget(self.console)
        h = QtWidgets.QHBoxLayout(); right.addLayout(h); h.addWidget(QtWidgets.QLabel("Raw command:"))
        self.entry = QtWidgets.QLineEdit(); self.entry.setPlaceholderText("e.g.  L   Pw45   V5   MLC3   O0"); self.entry.returnPressed.connect(self.on_entry); h.addWidget(self.entry)

        self.worker = SerialWorker(); self.worker.batch.connect(self.on_batch); self.worker.textline.connect(self.on_text)
        self.worker.connected.connect(self.on_connected); self.worker.start()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(100)
        self.exp_timer = QtCore.QTimer(self); self.exp_timer.timeout.connect(self.exp_tick); self.exp_timer.start(100)
        self.scan_ports(); self.set_controls_enabled(False)

    # ---------------------------------------------------------------- tabs
    def _tab_gravity(self):
        w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
        info = QtWidgets.QLabel("E-13a · Nobody touches the arm. The firmware holds it (PASSIVE, G) at each station, "
                                "ascending then descending, and the measured holding torque τ_m = K_t,j·I_q is fitted:\n"
                                "τ_m(θ) = g·cos(θ+φ). Also gives hysteresis per station and the at-rest noise floor σ₀.")
        info.setWordWrap(True); vl.addWidget(info)
        gl = QtWidgets.QGridLayout(); vl.addLayout(gl)
        gl.addWidget(QtWidgets.QLabel("Stations [deg]"), 0, 0); self.gr_st = QtWidgets.QLineEdit("15, 35, 55, 75, 95, 115"); gl.addWidget(self.gr_st, 0, 1, 1, 3)
        gl.addWidget(QtWidgets.QLabel("Settle [s]"), 1, 0); self.gr_settle = QtWidgets.QDoubleSpinBox(); self.gr_settle.setRange(0.5, 20); self.gr_settle.setValue(3.0); gl.addWidget(self.gr_settle, 1, 1)
        gl.addWidget(QtWidgets.QLabel("Dwell [s]"), 1, 2); self.gr_dwell = QtWidgets.QDoubleSpinBox(); self.gr_dwell.setRange(2, 120); self.gr_dwell.setValue(10.0); gl.addWidget(self.gr_dwell, 1, 3)
        self.gr_both = QtWidgets.QCheckBox("visit ascending then descending (hysteresis)"); self.gr_both.setChecked(True); vl.addWidget(self.gr_both)
        row = QtWidgets.QHBoxLayout(); vl.addLayout(row)
        self.btn_gr = QtWidgets.QPushButton("▶ Run gravity stations"); self.btn_gr.clicked.connect(self.gravity_start); row.addWidget(self.btn_gr)
        b = QtWidgets.QPushButton("Apply fitted g, φ"); b.clicked.connect(self.gravity_apply); row.addWidget(b)
        self.gr_table = QtWidgets.QTableWidget(0, 6); self.gr_table.setHorizontalHeaderLabels(["θ°", "τ_m", "τ̂_s", "τ̂_dob", "σ_dob", "I_q"])
        self.gr_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch); self.gr_table.setMinimumHeight(180); vl.addWidget(self.gr_table)
        self.gr_out = QtWidgets.QLabel("—"); self.gr_out.setWordWrap(True); self.gr_out.setStyleSheet("font-family:Consolas,Menlo,monospace;"); vl.addWidget(self.gr_out)
        vl.addStretch(1); return w

    def _tab_plant(self):
        w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
        info = QtWidgets.QLabel("E-13b · Nobody touches the arm. Motor-driven passive sweeps at several speeds and accelerations; "
                                "least-squares on the measured torque:\nτ_m = J·α + B·ω + g·cos(θ+φ) + τ_f·sgn(ω) → J, B, τ_f, g, φ.\n"
                                "Also the no-human bias and noise of all three estimators while moving (E8 condition d).")
        info.setWordWrap(True); vl.addWidget(info)
        gl = QtWidgets.QGridLayout(); vl.addLayout(gl)
        gl.addWidget(QtWidgets.QLabel("Min / Max [deg]"), 0, 0)
        self.pl_min = QtWidgets.QDoubleSpinBox(); self.pl_min.setRange(0, 150); self.pl_min.setValue(15); gl.addWidget(self.pl_min, 0, 1)
        self.pl_max = QtWidgets.QDoubleSpinBox(); self.pl_max.setRange(0, 150); self.pl_max.setValue(110); gl.addWidget(self.pl_max, 0, 2)
        gl.addWidget(QtWidgets.QLabel("Speeds [rad/s]"), 1, 0); self.pl_speeds = QtWidgets.QLineEdit("0.3, 0.6, 1.0"); gl.addWidget(self.pl_speeds, 1, 1, 1, 2)
        gl.addWidget(QtWidgets.QLabel("Accels [rad/s²]"), 2, 0); self.pl_accs = QtWidgets.QLineEdit("1.2, 3.0"); gl.addWidget(self.pl_accs, 2, 1, 1, 2)
        gl.addWidget(QtWidgets.QLabel("Reps per combo"), 3, 0); self.pl_reps = QtWidgets.QSpinBox(); self.pl_reps.setRange(1, 20); self.pl_reps.setValue(3); gl.addWidget(self.pl_reps, 3, 1)
        row = QtWidgets.QHBoxLayout(); vl.addLayout(row)
        self.btn_pl = QtWidgets.QPushButton("▶ Run plant-ID sweeps"); self.btn_pl.clicked.connect(self.plant_start); row.addWidget(self.btn_pl)
        b = QtWidgets.QPushButton("Apply J, B, τ_f, g, φ"); b.clicked.connect(self.plant_apply); row.addWidget(b)
        self.pl_out = QtWidgets.QLabel("—"); self.pl_out.setWordWrap(True); self.pl_out.setStyleSheet("font-family:Consolas,Menlo,monospace;"); vl.addWidget(self.pl_out)
        vl.addStretch(1); return w

    def _tab_estimator(self):
        w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
        info = QtWidgets.QLabel("E8 · Live numbers over the last window. In RUN: interaction torque τ_m − G(θ) = B_int·ω + C·sgn(ω) "
                                "and net motor power (negative = the user works against the device). The mode protocol steps the assist "
                                "gain with markers; a result table is written at the end.")
        info.setWordWrap(True); vl.addWidget(info)
        gl = QtWidgets.QGridLayout(); vl.addLayout(gl)
        gl.addWidget(QtWidgets.QLabel("Window [s]"), 0, 0); self.es_win = QtWidgets.QDoubleSpinBox(); self.es_win.setRange(2, 60); self.es_win.setValue(10); gl.addWidget(self.es_win, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Estimator"), 0, 2); self.es_sel = QtWidgets.QComboBox(); self.es_sel.addItems(["Static (E0)", "Direct (E1)", "DOB (E2)"]); self.es_sel.setCurrentIndex(2)
        self.es_sel.currentIndexChanged.connect(lambda i: self.send(f"E{i}")); gl.addWidget(self.es_sel, 0, 3)
        self.es_live = QtWidgets.QLabel("—"); self.es_live.setStyleSheet("font-family:Consolas,Menlo,monospace;"); self.es_live.setWordWrap(True); vl.addWidget(self.es_live)
        gl = QtWidgets.QGridLayout(); vl.addLayout(gl)
        gl.addWidget(QtWidgets.QLabel("Mode protocol  A : seconds"), 0, 0, 1, 2)
        self.es_prot = QtWidgets.QLineEdit("0:30, 1.0:30, -0.5:30, 1.12:30"); gl.addWidget(self.es_prot, 1, 0, 1, 2)
        self.btn_es = QtWidgets.QPushButton("▶ Run mode protocol (needs RUN + user)"); self.btn_es.clicked.connect(self.modes_start); gl.addWidget(self.btn_es, 2, 0, 1, 2)
        self.es_out = QtWidgets.QLabel("—"); self.es_out.setWordWrap(True); self.es_out.setStyleSheet("font-family:Consolas,Menlo,monospace;"); vl.addWidget(self.es_out)
        vl.addStretch(1); return w

    def _tab_thermal(self):
        w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
        info = QtWidgets.QLabel(f"Firmware model: C_th {TH_C} J/K · R_th {TH_RTH} K/W · T_amb {TH_AMB} °C · P = 1.5·I²·R · derate {TH_DERATE_LO:.0f}–{TH_DERATE_HI:.0f} °C.\n"
                                "1) IDLE check — with the driver off nothing heats, so T_est must fall toward T_amb. If it RISES, the current "
                                "sense is reading phantom current in IDLE (driver high-Z) and the integrator runs away (seen 18 Oct: 26 000 °C).\n"
                                "2) Stall test — block the arm (E9 strut, or let it rest on a hard stop) and hold a set current in LAB "
                                "velocity mode; T_est is compared with the model's own prediction.")
        info.setWordWrap(True); vl.addWidget(info)
        row = QtWidgets.QHBoxLayout(); vl.addLayout(row)
        row.addWidget(QtWidgets.QLabel("IDLE check [s]")); self.th_idle_s = QtWidgets.QSpinBox(); self.th_idle_s.setRange(5, 300); self.th_idle_s.setValue(20); row.addWidget(self.th_idle_s)
        self.btn_th_idle = QtWidgets.QPushButton("▶ IDLE phantom-current check"); self.btn_th_idle.clicked.connect(self.thermal_idle_start); row.addWidget(self.btn_th_idle)
        gl = QtWidgets.QGridLayout(); vl.addLayout(gl)
        gl.addWidget(QtWidgets.QLabel("Stall current [A]"), 0, 0); self.th_I = QtWidgets.QDoubleSpinBox(); self.th_I.setRange(0.5, 6.0); self.th_I.setValue(2.0); self.th_I.setSingleStep(0.5); gl.addWidget(self.th_I, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Duration [s]"), 0, 2); self.th_dur = QtWidgets.QSpinBox(); self.th_dur.setRange(10, 900); self.th_dur.setValue(120); gl.addWidget(self.th_dur, 0, 3)
        gl.addWidget(QtWidgets.QLabel("Push dir"), 1, 0); self.th_dir = QtWidgets.QComboBox(); self.th_dir.addItems(["toward lower stop (V+)", "toward upper stop (V−)"]); gl.addWidget(self.th_dir, 1, 1, 1, 3)
        self.btn_th_stall = QtWidgets.QPushButton("▶ Stall test (arm must be blocked)"); self.btn_th_stall.setStyleSheet("color:#c77d00; font-weight:bold;")
        self.btn_th_stall.clicked.connect(self.thermal_stall_start); vl.addWidget(self.btn_th_stall)
        self.th_out = QtWidgets.QLabel("—"); self.th_out.setWordWrap(True); self.th_out.setStyleSheet("font-family:Consolas,Menlo,monospace;"); vl.addWidget(self.th_out)
        vl.addStretch(1); return w

    def _tab_params(self):
        w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
        for spec in PARAMS:
            pr = ParamRow(*spec[:8], self.send, *(spec[8:] or ())); self.param[spec[1]] = pr; vl.addWidget(pr)
        b = QtWidgets.QPushButton("Send all parameters to board"); b.clicked.connect(self.push_all); vl.addWidget(b)
        vl.addStretch(1); return w

    # ---------------------------------------------------------------- helpers
    def pv(self, letter): return self.param[letter].value()

    def _state_style(self, color):
        self.lbl_state.setStyleSheet(f"background:{color}; color:white; font-size:20px; font-weight:bold; padding:10px; border-radius:6px;")

    def scan_ports(self):
        cur = self.port_box.currentText(); self.port_box.clear()
        for p in list_ports.comports(): self.port_box.addItem(f"{p.device}  —  {p.description}", p.device)
        i = self.port_box.findText(cur)
        if i >= 0: self.port_box.setCurrentIndex(i)

    def toggle_conn(self):
        if self.btn_conn.text() == "Connect":
            if self.port_box.currentIndex() < 0: self.log("✗ no serial port selected"); return
            self.worker.open_port(self.port_box.currentData())
        else: self.worker.close_port()

    def on_connected(self, ok):
        self.btn_conn.setText("Disconnect" if ok else "Connect"); self.set_controls_enabled(ok)
        if ok: QtCore.QTimer.singleShot(600, lambda: (self.send("L"), self.send("D" + self.rates[self.rate_box.currentIndex()][1])))
        else: self.lbl_state.setText("NOT CONNECTED"); self._state_style("#666")

    def set_controls_enabled(self, ok):
        for w in (self.btn_home, self.btn_idle, self.btn_rec, self.rate_box, self.btn_gr, self.btn_pl, self.btn_es, self.btn_th_idle, self.btn_th_stall): w.setEnabled(ok)

    def on_batch(self, rows):
        if self.t0 is None: self.t0 = rows[0][0]
        self.rows.extend(rows); self.last_t_ms = rows[-1][0]; self.last_row = rows[-1]
        if self.recording: self.csv_writer.writerows(rows); self.n_recorded += len(rows)
        if self.exp is not None and self.exp.get("capture"): self.exp_rows.extend(rows)

    def on_text(self, s):
        self.log(s)
        if s.startswith("L state="):
            self.last_status = s
            m = re.search(r"limits_deg=\[([-\d.]+),([-\d.]+)\]", s)
            if m: self.lim_deg = (float(m.group(1)), float(m.group(2)))
            m = re.search(r"iq_lim=([-\d.]+)", s)
            if m: self.iq_lim_board = float(m.group(1))
            self.fw_ok = "fw=v2.9" in s
            self.lbl_fw.setText(("firmware v2.9 ✓" if self.fw_ok else "⚠ firmware is NOT v2.9") + f"   limits {self.lim_deg[0]:.1f}–{self.lim_deg[1]:.1f}°   iq_lim {self.iq_lim_board:.2f} A")
            self.lbl_fw.setStyleSheet("color:#1a8a3c;" if self.fw_ok else "color:#c40000; font-weight:bold;")
        elif s.startswith("EV,THERM"): self.log("⚠ thermal derate event: " + s)
        elif s.startswith("EV,HOMED"): QtCore.QTimer.singleShot(300, lambda: self.send("L"))
        elif s.startswith("EV,ZERO"): QtCore.QTimer.singleShot(200, lambda: self.send("L"))

    def refresh(self):
        if not self.rows: return
        a = np.asarray(self.rows, dtype=float); t = (a[:, 0] - self.t0)/1000.0
        a = a[t > t[-1] - HISTORY_S]; tt = (a[:, 0] - self.t0)/1000.0; c = COL
        self.c_theta.setData(tt, a[:, c["theta"]]*DEG); self.c_ref.setData(tt, a[:, c["theta_ref"]]*DEG)
        self.c_ts.setData(tt, a[:, c["tau_s"]]); self.c_td.setData(tt, a[:, c["tau_d"]]); self.c_tdob.setData(tt, a[:, c["tau_dob"]])
        self.c_vel.setData(tt, a[:, c["vel"]]); self.c_taum.setData(tt, a[:, c["tau_m"]])
        self.c_iqm.setData(tt, a[:, c["iq_meas"]]); self.c_iqc.setData(tt, a[:, c["iq_cmd"]]); self.c_T.setData(tt, a[:, c["T_est"]]/100.0)
        last = a[-1]; st = int(last[c["state"]])
        self.lbl_state.setText(STATE_NAMES.get(st, str(st)) + (f"   ·   {self.exp['name']} {self.exp.get('status','')}" if self.exp else "")); self._state_style(STATE_COLORS.get(st, "#666"))
        self.lbl_readout.setText(f"θ = {last[c['theta']]*DEG:6.1f}°    τ̂_DOB = {last[c['tau_dob']]:+.3f} N·m    I_q = {last[c['iq_meas']]:+.2f} A    τ_m = {last[c['tau_m']]:+.3f}")
        T = last[c["T_est"]]
        if math.isnan(T): self.lbl_temp.setText("T_est = — (firmware < v2.9)")
        else:
            col = "#1a8a3c" if T < 60 else "#c77d00" if T < 70 else "#c40000"
            self.lbl_temp.setText(f"T_est = {T:.1f} °C" + ("   ⚠ DERATING" if T >= TH_DERATE_LO else "") + ("   ✗ IMPOSSIBLE — see Thermal tab" if T > 250 or T < 0 else ""))
            self.lbl_temp.setStyleSheet(f"color:{col}; font-weight:bold;")
        if self.recording: self.lbl_rec.setText(f"● recording… {self.n_recorded} samples")
        # live estimator stats
        if self.tabs.currentIndex() == 2:
            win = a[tt > tt[-1] - self.es_win.value()]
            s = estimator_stats(win, self.pv("g"), self.pv("f"))
            if s:
                txt = (f"rate {s['rate_hz']:.0f} Hz  n={s['n']}\nnoise SD  static {s['noise_tau_s']:.3f}  direct {s['noise_tau_d']:.3f}  DOB {s['noise_tau_dob']:.3f} N·m\n"
                       f"RMSE  s–d {s['rmse_tau_s_tau_d']:.3f}  s–DOB {s['rmse_tau_s_tau_dob']:.3f}  d–DOB {s['rmse_tau_d_tau_dob']:.3f}")
                if "lag_dob_ms" in s: txt += f"\nDOB lag vs static {s['lag_dob_ms']:+.0f} ms (r={s['corr_dob_static']:.2f})"
                if "B_int" in s: txt += (f"\nRUN: τ_m−G = {s['B_int']:+.3f}·ω {s['C_int']:+.3f}·sgn  R²={s['r2_int']:.2f}\n"
                                        f"net motor power {s['power_W']:+.2f} W   track RMS {s['track_rms_deg']:.2f}°   |ω| p95 {s['speed_p95_deg_s']:.0f}°/s")
                self.es_live.setText(txt)

    def send(self, cmd): self.worker.send(cmd); self.log(f"> {cmd}")
    def push_all(self):
        for r in self.param.values(): r.push()
    def stop_all(self):
        if self.exp: self.exp_finish(aborted=True)
        self.send("O0")
    def on_entry(self):
        c = self.entry.text().strip()
        if c: self.send(c)
        self.entry.clear()
    def log(self, s):
        self.console.appendPlainText(s); sb = self.console.verticalScrollBar(); sb.setValue(sb.maximum())
    def add_marker(self, text):
        t = self.last_t_ms if self.last_t_ms is not None else float("nan")
        self.markers.append((t, text)); self.log(f"◆ marker t_ms={t:.0f}: {text}")
    def state_now(self): return int(self.last_row[COL["state"]]) if self.last_row else -1
    def theta_deg(self): return self.last_row[COL["theta"]]*DEG if self.last_row else float("nan")
    def vel_now(self): return self.last_row[COL["vel"]] if self.last_row else float("nan")

    def _require(self, homed_state_ok, msg):
        if self.exp: self.log("✗ an experiment is already running — STOP first"); return False
        if self.last_row is None: self.log("✗ no telemetry yet"); return False
        if not self.fw_ok: self.log("⚠ firmware status not v2.9 (press Status) — continuing anyway")
        if self.state_now() not in homed_state_ok: self.log("✗ " + msg); return False
        return True

    def _parse_list(self, s):
        return [float(x) for x in re.split(r"[,\s]+", s.strip()) if x]

    # ---------------------------------------------------------------- experiment engine
    def exp_start(self, name, steps, on_finish):
        """steps: list of callables(exp) returning True when the step is complete (polled at 10 Hz)."""
        self.exp = {"name": name, "steps": steps, "i": -1, "on_finish": on_finish, "capture": False, "t_step": time.monotonic(), "status": "", "data": {}}
        self.exp_rows = []
        self.add_marker(f"{name} start"); self.log(f"▶ {name}")
        self.exp_next()

    def exp_next(self):
        e = self.exp; e["i"] += 1; e["t_step"] = time.monotonic()
        if e["i"] >= len(e["steps"]): self.exp_finish(); return
        step = e["steps"][e["i"]]
        if hasattr(step, "enter"): step.enter(e)

    def exp_tick(self):
        e = self.exp
        if not e or e["i"] >= len(e["steps"]): return
        if self.state_now() == ST_FAULT: self.log("✗ FAULT — experiment aborted"); self.exp_finish(aborted=True); return
        try:
            if e["steps"][e["i"]](e): self.exp_next()
        except Exception as ex:
            self.log(f"✗ experiment error: {ex}"); self.exp_finish(aborted=True)

    def exp_finish(self, aborted=False):
        e = self.exp; self.exp = None
        if e is None: return
        self.add_marker(f"{e['name']} {'ABORTED' if aborted else 'done'}")
        if not aborted:
            try: e["on_finish"](e)
            except Exception as ex: self.log(f"✗ result calculation failed: {ex}")
        self.log(f"■ {e['name']} {'aborted' if aborted else 'finished'}")

    # --- step factories ---------------------------------------------------------------
    def st_send(self, *cmds):
        def step(e):
            for c in cmds: self.send(c)
            return True
        return step

    def st_wait(self, secs, status=""):
        def step(e):
            e["status"] = status or f"wait {secs - (time.monotonic() - e['t_step']):.0f} s"
            return time.monotonic() - e["t_step"] >= secs
        return step

    def st_goto(self, deg, settle_s, tol_deg=1.5, timeout=25.0):
        """Send G<deg> and wait until the arm has been within tol for settle_s (PASSIVE hold)."""
        def step(e):
            if "sent" not in e["data"].get(("goto", deg), {}):
                e["data"][("goto", deg)] = {"sent": True, "arr": None}
                if self.state_now() == ST_RUN: self.send("O0")
                self.send(f"G{deg/DEG:.4f}")
            d = e["data"][("goto", deg)]
            e["status"] = f"→ {deg:.0f}°  (θ {self.theta_deg():.1f}°)"
            ok = abs(self.theta_deg() - deg) < tol_deg and abs(self.vel_now()) < 0.03
            if ok and d["arr"] is None: d["arr"] = time.monotonic()
            if not ok: d["arr"] = None
            if d["arr"] is not None and time.monotonic() - d["arr"] >= settle_s:
                del e["data"][("goto", deg)]; return True
            if time.monotonic() - e["t_step"] > timeout:
                self.log(f"⚠ goto {deg:.0f}° timed out (θ={self.theta_deg():.1f}°) — using current angle")
                del e["data"][("goto", deg)]; return True
            return False
        return step

    def st_capture(self, secs, label, store_key=None):
        """Capture telemetry for secs, store the rows under e['data'][store_key] (list of lists)."""
        def step(e):
            if not e["capture"]:
                e["capture"] = True; self.exp_rows = []; e["t_step"] = time.monotonic(); self.add_marker(f"{label} capture")
            e["status"] = f"{label} {secs - (time.monotonic() - e['t_step']):.0f} s"
            if time.monotonic() - e["t_step"] >= secs:
                e["capture"] = False
                if store_key: e["data"].setdefault(store_key, []).append(list(self.exp_rows))
                return True
            return False
        return step

    def st_park_release(self):
        """Park via G then release (O0) once settled — same end as the firmware's homing."""
        park = self.pv("k")*DEG
        inner = self.st_goto(park, 1.0, tol_deg=2.0, timeout=15.0)
        def step(e):
            if inner(e): self.send("O0"); return True
            return False
        return step

    # ---------------------------------------------------------------- gravity
    def gravity_start(self):
        if not self._require((ST_IDLE, ST_PASSIVE, ST_RUN), "home first, then start from IDLE / PASSIVE / RUN"): return
        st = [s for s in self._parse_list(self.gr_st.text()) if self.lim_deg[0] + 2 <= s <= self.lim_deg[1] - 2]
        if len(st) < 3: self.log("✗ need ≥3 stations inside the homed limits"); return
        order = sorted(st) + (sorted(st, reverse=True) if self.gr_both.isChecked() else [])
        steps = [self.st_send("A0")]
        for deg in order:
            steps += [self.st_goto(deg, self.gr_settle.value()), self.st_capture(self.gr_dwell.value(), f"station {deg:.0f}°", "stations")]
        steps.append(self.st_park_release())
        self.gr_table.setRowCount(0)
        self.exp_start("Gravity stations", steps, self.gravity_finish)

    def gravity_finish(self, e):
        stations = []
        for rows in e["data"].get("stations", []):
            a = np.asarray(rows, float)
            if len(a) < 10: continue
            s = {"theta": float(a[:, COL["theta"]].mean()), "theta_deg": float(a[:, COL["theta"]].mean()*DEG),
                 "tau_m": float(a[:, COL["tau_m"]].mean()), "iq": float(a[:, COL["iq_meas"]].mean()), "T_est": float(a[:, COL["T_est"]].mean()), "n": int(len(a))}
            for k in ("tau_s", "tau_d", "tau_dob"): s[k] = float(a[:, COL[k]].mean()); s[f"sd_{k}"] = float(a[:, COL[k]].std())
            stations.append(s)
        if len(stations) < 3: self.log("✗ too few valid stations"); return
        fit = fit_gravity_stations(stations)
        self.gr_table.setRowCount(len(stations))
        for i, s in enumerate(stations):
            for j, v in enumerate((s["theta_deg"], s["tau_m"], s["tau_s"], s["tau_dob"], s["sd_tau_dob"], s["iq"])):
                self.gr_table.setItem(i, j, QtWidgets.QTableWidgetItem(f"{v:.3f}" if j else f"{v:.1f}"))
        g_old, f_old = self.pv("g"), self.pv("f")
        res_old = [s["tau_m"] - g_old*math.cos(s["theta"] + f_old) for s in stations]
        txt = (f"fit τ_m(θ) = g·cos(θ+φ):  g = {fit['g']:.3f} N·m   φ = {fit['phi']:+.3f} rad ({fit['phi']*DEG:+.1f}°)   R² {fit['r2']:.4f}   RMSE {fit['rmse']:.3f}\n"
               f"board model g={g_old:.2f} φ={f_old:+.2f}: residual RMS {np.sqrt(np.mean(np.square(res_old))):.3f} N·m, max {np.max(np.abs(res_old)):.3f}\n"
               f"hysteresis half-width {fit['hysteresis_half_Nm']:.3f} N·m   at-rest σ₀: static {fit['sigma0_static']:.3f}  direct {fit['sigma0_direct']:.3f}  DOB {fit['sigma0_dob']:.3f}   DOB bias {fit['bias_dob']:+.3f}")
        self.gr_out.setText(txt); self.log(txt.replace("\n", " | "))
        self.results["gravity"] = {"fit": fit, "stations": stations, "board_g": g_old, "board_phi": f_old, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        self._save_result("gravity", self.results["gravity"])

    def gravity_apply(self):
        r = self.results.get("gravity")
        if not r: self.log("✗ no gravity result yet"); return
        self.param["g"].set_value(round(r["fit"]["g"], 3)); self.param["f"].set_value(round(r["fit"]["phi"], 3))
        self.log(f"applied Pg{r['fit']['g']:.3f} Pf{r['fit']['phi']:.3f}")

    # ---------------------------------------------------------------- plant ID
    def plant_start(self):
        if not self._require((ST_IDLE, ST_PASSIVE, ST_RUN), "home first; start from IDLE / PASSIVE / RUN"): return
        lo, hi = self.pl_min.value(), self.pl_max.value()
        if not (self.lim_deg[0] + 2 <= lo < hi <= self.lim_deg[1] - 2): self.log("✗ min/max outside the homed limits"); return
        speeds, accs = self._parse_list(self.pl_speeds.text()), self._parse_list(self.pl_accs.text())
        steps = [self.st_send("A0"), self.st_goto(lo, 1.0)]
        for v in speeds:
            for acc in accs:
                steps.append(self.st_send(f"Pv{v:.2f}", f"Pa{acc:.2f}"))
                for _ in range(self.pl_reps.value()):
                    steps += [self._sweep_leg(hi), self._sweep_leg(lo)]
        steps += [self.st_send(f"Pv{self.pv('v'):.2f}", f"Pa{self.pv('a'):.2f}"), self.st_park_release()]
        self.exp_start("Plant ID sweeps", steps, self.plant_finish)

    def _sweep_leg(self, target_deg):
        goto = self.st_goto(target_deg, 0.3, tol_deg=2.0, timeout=30.0)
        def step(e):
            e["capture"] = True
            done = goto(e)
            if done:
                e["data"].setdefault("sweep", []).extend(self.exp_rows); self.exp_rows = []
            return done
        return step

    def plant_finish(self, e):
        rows = e["data"].get("sweep", [])
        e["capture"] = False
        if len(rows) < 200: self.log("✗ too little sweep data"); return
        r = fit_plant(rows)
        self.results["plant"] = {"fit": r, "board": {k: self.pv(k) for k in ("j", "b", "c", "g", "f")}, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        txt = (f"n={r['n']} @ {1000/r['dt_ms']:.0f} Hz   R² {r['r2']:.3f}   RMSE {r['rmse']:.3f} N·m   α_rms {r['acc_rms']:.2f}  ω_rms {r['vel_rms']:.2f}\n"
               f"J  = {r['J']:.4f} ± {r['se_J']:.4f} kg·m²   (board {self.pv('j'):.3f})\n"
               f"B  = {r['B']:.3f} ± {r['se_B']:.3f} N·m·s/rad (board {self.pv('b'):.3f})\n"
               f"τ_f = {r['tau_f']:.3f} ± {r['se_tau_f']:.3f} N·m    (board {self.pv('c'):.2f})\n"
               f"g  = {r['g']:.3f} N·m  φ = {r['phi']:+.3f} rad      (board {self.pv('g'):.2f} / {self.pv('f'):+.2f})\n"
               f"no-human while moving — bias / SD:  static {r['noload_bias_tau_s']:+.3f}/{r['noload_sd_tau_s']:.3f}   "
               f"direct {r['noload_bias_tau_d']:+.3f}/{r['noload_sd_tau_d']:.3f}   DOB {r['noload_bias_tau_dob']:+.3f}/{r['noload_sd_tau_dob']:.3f}")
        if r["acc_rms"] < 0.5: txt += "\n⚠ low acceleration content — J is poorly determined; raise the accel list"
        self.pl_out.setText(txt); self.log(txt.replace("\n", " | "))
        self._save_result("plant", self.results["plant"], rows)

    def plant_apply(self):
        r = self.results.get("plant")
        if not r: self.log("✗ no plant-ID result yet"); return
        f = r["fit"]
        self.param["j"].set_value(round(max(f["J"], 0.001), 4)); self.param["b"].set_value(round(max(f["B"], 0.0), 3))
        self.param["c"].set_value(round(min(max(f["tau_f"], 0.0), 0.5), 3)); self.param["g"].set_value(round(f["g"], 3)); self.param["f"].set_value(round(f["phi"], 3))
        self.log("applied Pj Pb Pc Pg Pf from plant ID")

    # ---------------------------------------------------------------- mode protocol
    def modes_start(self):
        if not self._require((ST_RUN,), "press RUN (O1) with the user in the arm first"): return
        prot = []
        for tok in re.split(r"[,\s]+", self.es_prot.text().strip()):
            if ":" in tok: A, s = tok.split(":"); prot.append((float(A), float(s)))
        if not prot: self.log("✗ protocol empty — format  A:seconds, A:seconds"); return
        steps = []
        for A, s in prot:
            steps += [self.st_send(f"A{A:.2f}"), self.st_wait(2.0, "settle"), self.st_capture(s, f"A={A:+.2f}", "modes")]
        steps.append(self.st_send(f"A0"))
        self.exp = None
        self.exp_start("Mode protocol", steps, self.modes_finish)
        self.exp["data"]["prot"] = prot

    def modes_finish(self, e):
        out = []
        for (A, s), rows in zip(e["data"]["prot"], e["data"].get("modes", [])):
            st = estimator_stats(rows, self.pv("g"), self.pv("f")) or {}
            st["A"] = A; out.append(st)
        lines = ["A      B_int   C_int   P[W]  track°  DOBnoise"]
        for st in out:
            lines.append(f"{st['A']:+5.2f}  {st.get('B_int', float('nan')):+6.3f}  {st.get('C_int', float('nan')):+6.3f}  {st.get('power_W', float('nan')):+5.2f}  {st.get('track_rms_deg', float('nan')):5.2f}  {st.get('noise_tau_dob', float('nan')):.3f}")
        txt = "\n".join(lines); self.es_out.setText(txt); self.log(txt.replace("\n", " | "))
        self.results["modes"] = {"stages": out, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        self._save_result("modes", self.results["modes"])

    # ---------------------------------------------------------------- thermal
    def thermal_idle_start(self):
        if not self._require((ST_IDLE,), "must be in IDLE (O0)"): return
        self.exp_start("IDLE thermal check", [self.st_capture(self.th_idle_s.value(), "IDLE", "idle")], self.thermal_idle_finish)

    def thermal_idle_finish(self, e):
        a = np.asarray(e["data"]["idle"][0], float)
        t = (a[:, COL["t_ms"]] - a[0, COL["t_ms"]])/1000.0; T = a[:, COL["T_est"]]
        slope = float(np.polyfit(t, T, 1)[0]) if len(a) > 5 else float("nan")
        expected = -(T[0] - TH_AMB)/(TH_RTH*TH_C)
        iq = a[:, COL["iq_meas"]]
        verdict = ("OK — T_est is relaxing toward T_amb as the model predicts" if slope <= expected*0.5 + 0.02 and T[-1] < 150
                   else "✗ T_est is NOT cooling in IDLE → phantom current in the thermal integrator (see docs/lab_procedures.md, fix in controlUpdate)")
        txt = (f"IDLE {t[-1]:.0f} s: T_est {T[0]:.1f} → {T[-1]:.1f} °C   slope {slope:+.3f} K/s   model expects {expected:+.3f} K/s\n"
               f"|I_q meas| mean {np.mean(np.abs(iq)):.3f} A  max {np.max(np.abs(iq)):.3f} A  (driver is high-Z: anything ≫ 0 is phantom)\n{verdict}")
        self.th_out.setText(txt); self.log(txt.replace("\n", " | "))
        self.results["thermal_idle"] = {"slope": slope, "expected": expected, "T0": float(T[0]), "T1": float(T[-1]), "iq_abs_mean": float(np.mean(np.abs(iq))), "verdict": verdict, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        self._save_result("thermal_idle", self.results["thermal_idle"])

    def thermal_stall_start(self):
        if not self._require((ST_IDLE,), "must be in IDLE (O0) with the arm blocked"): return
        I, dur = self.th_I.value(), self.th_dur.value()
        Tpred, Tss, tau = thermal_predict([dur], float(self.last_row[COL["T_est"]]), I, self.pv("r") if "r" in self.param else 0.47)
        ans = QtWidgets.QMessageBox.question(self, "Stall test",
            f"The motor will push at up to {I:.1f} A for {dur} s in LAB velocity mode (no joint limits!).\n"
            f"The arm MUST be blocked (E9 strut or resting on the hard stop in the push direction).\n"
            f"Model predicts T_est → {float(Tpred[0]):.0f} °C after {dur} s (steady state {Tss:.0f} °C, τ {tau:.0f} s).\n"
            f"E05 reference: 3 A plateau ≈ 77 °C; 6 A reaches 80 °C in 25 s.\n\nProceed?", QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes: return
        sign = +1 if self.th_dir.currentIndex() == 0 else -1
        steps = [self.st_send(f"Pi{I:.2f}", f"MLC{I:.2f}", f"V{sign*5:.1f}"),
                 self.st_capture(dur, f"stall {I:.1f} A", "stall"),
                 self.st_send("O0", f"Pi{self.pv('i'):.2f}", f"MLC{self.pv('i'):.2f}")]
        self.exp_start("Thermal stall test", steps, self.thermal_stall_finish)
        self.exp["data"]["I"] = I

    def thermal_stall_finish(self, e):
        rows = e["data"]["stall"][0]; I = e["data"]["I"]
        r = fit_thermal(rows)
        a = np.asarray(rows, float); t = (a[:, COL["t_ms"]] - a[0, COL["t_ms"]])/1000.0
        pred, Tss, tau = thermal_predict(t, r["T_start"], r["iq_mean"], self.pv("r") if "r" in self.param else 0.47)
        dev = float(np.sqrt(np.mean((a[:, COL["T_est"]] - pred)**2)))
        txt = (f"stall {r['duration_s']:.0f} s at |I_q| {r['iq_mean']:.2f} A (set {I:.1f}):  T_est {r['T_start']:.1f} → {r['T_end']:.1f} °C\n"
               f"fitted: τ {r['tau_s']:.0f} s, T_ss {r['T_ss']:.0f} °C (R² {r['r2']:.3f})   model: τ {tau:.0f} s, T_ss {Tss:.0f} °C   RMS deviation {dev:.2f} °C\n"
               f"→ compare T_ss with the E05 thermistor plateau at this current (3 A: 77 °C); the model has no sensor on the rehab board")
        if abs(r["iq_mean"] - I) > 0.3: txt += f"\n⚠ measured current differs from the set point by {r['iq_mean']-I:+.2f} A (arm not blocked, or derate active)"
        self.th_out.setText(txt); self.log(txt.replace("\n", " | "))
        self.results["thermal_stall"] = {"fit": r, "model_tau": float(tau), "model_Tss": float(Tss), "rms_dev": dev, "I_set": I, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        self._save_result("thermal_stall", self.results["thermal_stall"], rows)

    # ---------------------------------------------------------------- files
    def _save_result(self, name, result, rows=None):
        os.makedirs("logs", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S"); base = f"logs/lab_{name}_{stamp}"
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump({"gui": GUI_VERSION, "firmware_status": self.last_status, "note": self.note.text(), "result": result}, f, indent=1, default=float)
        if rows:
            with open(base + ".csv", "w", newline="") as f:
                w = csv.writer(f); w.writerow(FIELDS); w.writerows(rows)
        self.log(f"saved {base}.json" + (" + .csv" if rows else ""))

    def toggle_record(self):
        if not self.recording:
            os.makedirs("logs", exist_ok=True)
            fname = time.strftime("logs/lab_%Y%m%d_%H%M%S.csv")
            self.csv_file = open(fname, "w", newline=""); self.csv_writer = csv.writer(self.csv_file); self.csv_writer.writerow(FIELDS)
            self.n_recorded = 0; self.recording = True; self.last_log = fname; self.markers = []
            self.send("L"); self.btn_rec.setText("■  Stop recording"); self.log(f"● recording → {fname}")
        else:
            self.recording = False; self.csv_file.close(); self.write_meta()
            self.btn_rec.setText("●  Record"); self.lbl_rec.setText(f"saved {self.n_recorded} samples → {self.last_log}")
            self.log(f"■ saved {self.n_recorded} samples (+ .meta.txt)")

    def write_meta(self):
        try:
            with open(self.last_log.replace(".csv", ".meta.txt"), "w", encoding="utf-8") as f:
                f.write(f"log: {self.last_log}\nsaved: {time.strftime('%Y-%m-%d %H:%M:%S')}\ngui: {GUI_VERSION}\nfirmware status: {self.last_status}\nnote: {self.note.text()}\n")
                f.write(f"estimator: {['static','direct','DOB'][self.es_sel.currentIndex()]}\ntelemetry: {self.rates[self.rate_box.currentIndex()][0]}\nsamples: {self.n_recorded}\n")
                f.write("parameters (GUI values at stop):\n")
                for r in self.param.values(): f.write(f"  P{r.letter}  {r.label}: {r.value():.{r.decimals}f}\n")
                f.write("markers (t_ms, text):\n")
                for t, s in self.markers: f.write(f"  {t:.0f}, {s}\n")
                if self.results:
                    f.write("experiment results (this session):\n")
                    f.write(json.dumps(self.results, indent=1, default=float) + "\n")
        except Exception as e: self.log(f"✗ meta file failed: {e}")

    def closeEvent(self, ev):
        if self.exp: self.exp_finish(aborted=True)
        if self.recording: self.toggle_record()
        self.worker.stop(); ev.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion")
    w = Main(); w.show(); sys.exit(app.exec_())
