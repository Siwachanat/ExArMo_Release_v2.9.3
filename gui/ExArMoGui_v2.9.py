#!/usr/bin/env python3
"""
ExArMo Rehab Control Panel v2.9 — GUI for the Effort-Adaptive Admittance firmware v2.9.

No command-line arguments and no typed serial commands needed:
  1. Pick the serial port from the dropdown and press Connect.
  2. Press "1. Home"  — the arm finds the capstan hard stops by itself.
  3. Press "2. Admittance" — push the arm to feel it (or "Passive" sweeps).
  4. Choose the estimator and drag the Assist slider (right = assist, left = resistance).
  5. Press "Record" before an experiment, "Stop" after → CSV in ./logs plus a
     .meta.txt with every parameter, then "Analyze last recording".

v2.9 (16 Oct 2026) — matched to firmware v2.9, built for the §4.8 / E08 re-run:
  - "Lab constants" tab: K_t,j (0.775 N·m/A, E9), dead-zone (0.15 N·m), continuous
    current (3 A, E05), phase R (0.47 Ω, E01) and L — every number from the lab.
  - Impedance tab REMOVED: firmware v2.8/v2.9 has no impedance mode, and its
    letters collided with firmware parameters (Pm0.15 switched the torque
    controller to VOLTAGE mode, Pa0/1 overwrote the passive acceleration,
    Pe set the passive integral gain). "Send all parameters" was unsafe in v2.4.
  - Passive tab with goto / sweep sequencer (the firmware's 'G' command).
  - Telemetry field 13 (T_est, estimated winding temperature) + 500 Hz (D2) logging.
  - E08 protocol timer (rest / motion stages) written as markers to the .meta.txt.
  - Record writes <log>.meta.txt (firmware status line, all parameters, estimator,
    assist, note, stage markers) so a log can be re-analysed with the constants it
    was taken with.

Install:  pip install pyserial pyqtgraph PyQt5 numpy pandas matplotlib
Run:      python ExArMoGui_v2.9.py
"""
import sys, os, csv, time, threading, collections, subprocess, math
import numpy as np
import serial
from serial.tools import list_ports
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

GUI_VERSION = "v2.9 (16 Oct 2026)"
BAUD = 921600
FIELDS = ["t_ms","state","theta","theta_ref","vel","iq_meas",
          "tau_m","tau_s","tau_d","tau_dob","Bv","iq_cmd","T_est"]
N_FIELDS_OLD = 12                       # firmware ≤ v2.8 sends 12 fields; T_est is padded
STATE_NAMES  = {0:"IDLE",1:"HOMING · LIFT ↑",2:"HOMING · LOWER ↓",3:"PARKING",4:"RUN · ADMITTANCE",5:"FAULT",6:"PASSIVE",7:"LAB · VELOCITY"}
STATE_COLORS = {0:"#666666",1:"#c77d00",2:"#c77d00",3:"#c77d00",4:"#1a8a3c",5:"#c40000",6:"#6a5acd",7:"#0e7c86"}
HISTORY_S = 15.0
DEG = 180.0/np.pi

# label, firmware letter, min, max, step, default, decimals, tooltip, (optional) scale sent = value*scale
PARAM_TABS = {
    "Model": [
        ("Gravity coeff g [Nm]",   "g", 0.0, 5.0,  0.01, 0.60, 2, "Peak gravity torque of arm support + forearm. Tune until the arm stays put at every angle (Admittance, Assist = 0, dead-zone 0).\nE9 note: the drawing value for the bare cradle (0.185 Nm) was 2.5x too high — tune it, do not compute it."),
        ("Gravity phase φ [rad]",  "f", -3.2, 3.2, 0.01, 0.00, 2, "Shift so gravity torque is max when the arm is horizontal."),
        ("Inertia J [kg·m²]",      "j", 0.001, 0.2, 0.001, 0.015, 3, "Reflected inertia at the joint. Used by the direct estimator and the DOB."),
        ("Damping B [Nm·s/rad]",   "b", 0.0, 1.0,  0.005, 0.05, 3, "Viscous friction of the joint model."),
        ("Coulomb friction [Nm]",  "c", 0.0, 0.5,  0.01, 0.15, 2, "Smooth-sign friction term in all three estimators and the passive feedforward.\nE9 measured 0.02–0.13 Nm of drive hysteresis (half-width): 0.10–0.15 is the right range."),
    ],
    "Admittance": [
        ("Virtual inertia J_v",    "J", 0.002, 0.2, 0.002, 0.02, 3, "How heavy the virtual arm feels to accelerate."),
        ("B_min (active, light)",  "n", 0.01, 1.0, 0.01, 0.05, 2, "Damping when the user pushes hard — smaller = lighter."),
        ("B_max (passive, firm)",  "x", 0.05, 3.0, 0.05, 0.80, 2, "Damping when the user is passive — larger = steadier."),
        ("Effort scale τ₀ [Nm]",   "t", 0.05, 2.0, 0.05, 0.40, 2, "Torque level where damping transitions from firm to light."),
        ("Dead-zone τ_dz [Nm]",    "z", 0.0, 1.0,  0.01, 0.15, 2, "|τ̂_h| below this is treated as zero effort by the admittance loop (telemetry still shows the raw estimate).\nE9: noise floor σ₀ = 0.018 Nm, latched drive hysteresis up to 0.13 Nm → 0.15 Nm. Set 0 while tuning gravity."),
    ],
    "Estimator": [
        ("DOB bandwidth ω_c [rad/s]", "w", 5.0, 150.0, 1.0, 30.0, 0, "Sweep 10–100 for the paper's g-sweep figure. Higher = faster but noisier."),
    ],
    "Passive": [
        ("Passive speed [rad/s]",  "v", 0.05, 1.5, 0.05, 0.50, 2, "Cruise speed of 'G' goto moves."),
        ("Passive accel [rad/s²]", "a", 0.05, 5.0, 0.05, 1.20, 2, "Acceleration of the trapezoidal profile — lower = softer."),
        ("Passive integral Ki",    "e", 0.0, 40.0, 0.5, 12.0, 1, "Integral gain of the passive tracking loop (absorbs friction that varies along the stroke)."),
    ],
    "Inner loop": [
        ("Position Kp [Nm/rad]",   "p", 1.0, 100.0, 1.0, 20.0, 1, "Stiffness of the reference-tracking loop."),
        ("Position Kd [Nm·s/rad]", "d", 0.0, 5.0,  0.05, 0.5, 2, "Damping of the reference-tracking loop."),
    ],
    "Lab constants": [
        ("K_t,j joint gain [Nm/A]", "K", 0.2, 2.0, 0.005, 0.775, 3, "Joint torque per REPORTED ampere, τ_m = K_t,j·I_q.\nE9 load-cell ground truth, 15 Oct 2026, motor R4001: 0.775 ± 0.001 (E7 6 Sep: 0.79 ± 0.04).\nReplaces N·Kt = 9.0 × 0.0955 = 0.86 of firmware v2.8 (ignored the 83 % capstan efficiency)."),
        ("Phase resistance R [Ω]",  "r", 0.05, 5.0, 0.005, 0.47, 3, "E01 four-wire Kelvin, 25 °C: R4001 0.472 / R4002 0.496 / R4003 0.485 Ω. Re-tunes the current loop when changed."),
        ("Phase inductance L [mH]", "l", 0.01, 10.0, 0.01, 0.25, 2, "Set explicitly so the boot-time motor characterisation never runs (lab fix v2-E). Sent in henry.", 1e-3),
        ("Continuous Iq [A]",       "u", 0.5, 8.0, 0.1, 3.0, 1, "E05: 3 A keeps the end-winding below 80 °C in still air (plateau 77 °C). The thermal model derates the Iq limit to this value between 70 and 80 °C estimated."),
    ],
    "Safety": [
        ("Iq limit [A]",           "i", 0.5, 15.0, 0.5, 6.0, 1, "Absolute current clamp (E05: 6 A reaches 80 °C in 25 s — the thermal model derates before that)."),
        ("Homing current [A]",     "h", 0.2, 4.0,  0.1, 1.2, 1, "Push current while finding the hard stops."),
        ("Park angle [rad]",       "k", 0.0, 2.6,  0.01, 0.20, 2, "Where homing gently parks and releases the arm. Set ≈ the natural rest angle so releasing causes no drop."),
        ("Park ramp speed [rad/s]","y", 0.05, 1.0, 0.05, 0.40, 2, "Speed of the post-homing ramp to the park angle."),
    ],
}

E08_STAGES = [("rest · park angle", 30), ("rest · hold mid-range by hand", 30), ("rest · hold near max", 30),
              ("voluntary flexion–extension ≈ 0.3 Hz", 120), ("rest · park angle", 30)]

COL = {f: i for i, f in enumerate(FIELDS)}


class SerialWorker(QtCore.QThread):
    batch     = QtCore.pyqtSignal(list)   # a list of parsed telemetry rows (emitted ~20x/s max)
    textline  = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.port = None
        self.ser = None
        self.running = True
        self.tx_lock = threading.Lock()
        self._rx = b""

    def open_port(self, port):
        self.port = port

    def close_port(self):
        self.port = None

    def run(self):
        pending, last_emit = [], 0.0
        while self.running:
            if self.port is None:
                if self.ser:
                    try: self.ser.close()
                    except Exception: pass
                    self.ser = None
                    self.connected.emit(False)
                time.sleep(0.1); continue
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, BAUD, timeout=0.05)
                    self._rx = b""
                    self.connected.emit(True)
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
                            if len(parts) == N_FIELDS_OLD:           # firmware ≤ v2.8
                                parts = parts + ["nan"]
                            if len(parts) == len(FIELDS):
                                try: pending.append([float(p) for p in parts])
                                except ValueError: pass
                        else:
                            self.textline.emit(line)
                now = time.monotonic()
                if pending and (now - last_emit > 0.05 or len(pending) > 300):
                    self.batch.emit(pending)
                    pending, last_emit = [], now
            except (serial.SerialException, OSError) as e:
                self.textline.emit(f"✗ serial error: {e}")
                try:
                    if self.ser: self.ser.close()
                except Exception: pass
                self.ser = None
                self.connected.emit(False)
                time.sleep(1.0)

    def send(self, cmd):
        with self.tx_lock:
            if self.ser:
                try: self.ser.write((cmd.strip() + "\n").encode())
                except Exception: pass

    def stop(self):
        self.running = False
        self.wait(1000)
        if self.ser:
            try: self.ser.close()
            except Exception: pass


class ParamRow(QtWidgets.QWidget):
    """Slider + spinbox pair; auto-sends 'P<letter><value>' 300 ms after the last change."""
    def __init__(self, label, letter, lo, hi, step, default, decimals, tip, send_fn, scale_out=1.0):
        super().__init__()
        self.label, self.letter, self.send_fn, self.decimals, self.scale_out = label, letter, send_fn, decimals, scale_out
        self.setToolTip(tip)
        lay = QtWidgets.QGridLayout(self); lay.setContentsMargins(0, 2, 0, 2)
        lbl = QtWidgets.QLabel(label); lbl.setToolTip(tip)
        lay.addWidget(lbl, 0, 0, 1, 2)
        self.scale = 10 ** decimals if decimals > 0 else 1
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(int(lo*self.scale), int(hi*self.scale))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(lo, hi); self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals); self.spin.setValue(default)
        self.spin.setFixedWidth(90)
        self.slider.setValue(int(default*self.scale))
        lay.addWidget(self.slider, 1, 0); lay.addWidget(self.spin, 1, 1)
        self._debounce = QtCore.QTimer(self); self._debounce.setSingleShot(True)
        self._debounce.setInterval(300); self._debounce.timeout.connect(self._emit)
        self._syncing = False
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def _from_slider(self, v):
        if self._syncing: return
        self._syncing = True; self.spin.setValue(v/self.scale); self._syncing = False
        self._debounce.start()

    def _from_spin(self, v):
        if self._syncing: return
        self._syncing = True; self.slider.setValue(int(round(v*self.scale))); self._syncing = False
        self._debounce.start()

    def value(self): return self.spin.value()

    def _emit(self):
        v = self.spin.value() * self.scale_out
        txt = f"{v:.{self.decimals}f}" if self.scale_out == 1.0 else f"{v:.6g}"
        self.send_fn(f"P{self.letter}{txt}")

    def push(self):     # re-send current value (e.g. after reconnect)
        self._emit()


class Main(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"ExArMo Rehab Control Panel {GUI_VERSION} — firmware v2.9")
        self.resize(1480, 900)
        pg.setConfigOptions(antialias=True, background="w", foreground="k")

        self.rows = collections.deque(maxlen=8000)   # ~16 s at 500 Hz — only what we plot
        self.t0 = None
        self.last_t_ms = None
        self.recording = False
        self.csv_file = self.csv_writer = None
        self.n_recorded = 0
        self.last_log = None
        self.last_status = ""
        self.param_rows = []
        self.markers = []                            # (t_ms, text) for the meta file
        self.sweep_targets = []; self.sweep_i = 0; self.sweep_running = False
        self.e08_i = -1; self.e08_t_end = 0.0

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # ================= LEFT: control panel =================
        panel = QtWidgets.QWidget(); panel.setFixedWidth(360)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setFixedWidth(380); scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left = QtWidgets.QVBoxLayout(panel)
        root.addWidget(scroll, 0)

        # --- connection ---
        g = QtWidgets.QGroupBox("Connection"); left.addWidget(g)
        gl = QtWidgets.QHBoxLayout(g)
        self.port_box = QtWidgets.QComboBox(); gl.addWidget(self.port_box, 1)
        b_refresh = QtWidgets.QToolButton(); b_refresh.setText("⟳")
        b_refresh.setToolTip("Rescan serial ports"); b_refresh.clicked.connect(self.scan_ports)
        gl.addWidget(b_refresh)
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_conn.clicked.connect(self.toggle_conn); gl.addWidget(self.btn_conn)

        # --- big state indicator ---
        self.lbl_state = QtWidgets.QLabel("NOT CONNECTED")
        self.lbl_state.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_state.setStyleSheet("background:#666; color:white; font-size:20px; "
                                     "font-weight:bold; padding:10px; border-radius:6px;")
        left.addWidget(self.lbl_state)
        self.lbl_readout = QtWidgets.QLabel("θ = —      τ̂_h = —")
        self.lbl_readout.setAlignment(QtCore.Qt.AlignCenter)
        f = self.lbl_readout.font(); f.setPointSize(11); self.lbl_readout.setFont(f)
        left.addWidget(self.lbl_readout)
        self.lbl_temp = QtWidgets.QLabel("T_est = —")
        self.lbl_temp.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_temp.setToolTip("Estimated end-winding temperature (software model fitted to E05).\n"
                                 "70–80 °C: Iq limit derates to the continuous rating. Let it cool below 60 °C between trials.")
        left.addWidget(self.lbl_temp)

        # --- workflow ---
        g = QtWidgets.QGroupBox("Workflow"); left.addWidget(g)
        gl = QtWidgets.QVBoxLayout(g)
        # [fw v2.9z] manual zero: put the arm on the bottom stop by hand → "0 here" → Home lifts only
        hrow = QtWidgets.QHBoxLayout(); gl.addLayout(hrow)
        self.btn_zero = QtWidgets.QPushButton("0  here")
        self.btn_zero.setMinimumHeight(36); self.btn_zero.setStyleSheet("font-weight:bold;")
        self.btn_zero.setToolTip("Manual zero (Z): stops the motor, then the arm's CURRENT position becomes 0°.\n"
                                 "Put the arm on the bottom stop by hand first, then press Home —\n"
                                 "homing then only lifts to find the top (no push-down).")
        self.btn_zero.clicked.connect(lambda: (self.send("O0"), self.send("Z")))
        hrow.addWidget(self.btn_zero, 1)
        self.btn_home = QtWidgets.QPushButton("1.  Home  (find capstan limits)")
        self.btn_home.setMinimumHeight(36)
        self.btn_home.setToolTip("H: after '0 here' → lift only; otherwise full automatic homing.")
        self.btn_home.clicked.connect(lambda: self.send("H"))
        hrow.addWidget(self.btn_home, 3)
        self.btn_home_full = QtWidgets.QPushButton("Full auto")
        self.btn_home_full.setMinimumHeight(36)
        self.btn_home_full.setToolTip("H1: full automatic homing (push down to the bottom stop, then up). Replaces the manual zero.")
        self.btn_home_full.clicked.connect(lambda: self.send("H1"))
        hrow.addWidget(self.btn_home_full, 1)
        row = QtWidgets.QHBoxLayout(); gl.addLayout(row)
        self.btn_run = QtWidgets.QPushButton("2. Admittance"); self.btn_run.setMinimumHeight(36)
        self.btn_run.setStyleSheet("font-weight:bold;")
        self.btn_run.setToolTip("Run admittance control (O1): estimated torque → motion reference → position loop.\nAssist slider active in this mode.")
        self.btn_run.clicked.connect(lambda: self.send("O1")); row.addWidget(self.btn_run)
        self.btn_idle = QtWidgets.QPushButton("Idle / Stop"); self.btn_idle.setMinimumHeight(36)
        self.btn_idle.setStyleSheet("color:#c40000; font-weight:bold;")
        self.btn_idle.clicked.connect(self.stop_all); row.addWidget(self.btn_idle)
        self.btn_stat = QtWidgets.QPushButton("Status")
        self.btn_stat.setToolTip("Print firmware version, constants (K_t,j, N, R, L, dead-zone), sensor angle, homed limits.\n"
                                 "Rotate the arm by hand: raw_motor_angle must change.")
        self.btn_stat.clicked.connect(lambda: self.send("L")); row.addWidget(self.btn_stat)

        # --- estimator ---
        g = QtWidgets.QGroupBox("Active estimator (feeds the admittance)"); left.addWidget(g)
        gl = QtWidgets.QHBoxLayout(g)
        self.est_group = QtWidgets.QButtonGroup(self)
        for i, name in enumerate(["Static", "Direct", "DOB"]):
            rb = QtWidgets.QRadioButton(name)
            if i == 2: rb.setChecked(True)
            self.est_group.addButton(rb, i); gl.addWidget(rb)
        self.est_group.idClicked.connect(lambda i: self.send(f"E{i}"))

        # --- assist slider ---
        g = QtWidgets.QGroupBox("Mode: resistance  ←  |  →  assist"); left.addWidget(g)
        gl = QtWidgets.QVBoxLayout(g)
        self.assist = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.assist.setRange(-200, 200); self.assist.setValue(100)
        self.assist.setTickPosition(QtWidgets.QSlider.TicksBelow); self.assist.setTickInterval(50)
        gl.addWidget(self.assist)
        self.lbl_assist = QtWidgets.QLabel("Assist gain A = 1.00")
        self.lbl_assist.setAlignment(QtCore.Qt.AlignCenter); gl.addWidget(self.lbl_assist)
        self._assist_deb = QtCore.QTimer(self); self._assist_deb.setSingleShot(True)
        self._assist_deb.setInterval(300)
        self._assist_deb.timeout.connect(lambda: self.send(f"A{self.assist.value()/100:.2f}"))
        self.assist.valueChanged.connect(self._assist_changed)
        row = QtWidgets.QHBoxLayout(); gl.addLayout(row)
        for txt, val in [("Resist −0.5", -50), ("Gravity-comp 0", 0), ("Assist 1.0", 100)]:
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(lambda _, v=val: self.assist.setValue(v)); row.addWidget(b)

        # --- parameter tabs ---
        tabs = QtWidgets.QTabWidget(); left.addWidget(tabs, 1)
        for tab_name, rows in PARAM_TABS.items():
            w = QtWidgets.QWidget(); vl = QtWidgets.QVBoxLayout(w)
            for spec in rows:
                pr = ParamRow(*spec[:8], self.send, *(spec[8:] or ()))
                self.param_rows.append(pr); vl.addWidget(pr)
            if tab_name == "Passive":
                vl.addWidget(self._build_passive_box())
            if tab_name == "Safety":
                vl.addWidget(self._build_advanced_box())
            vl.addStretch(1)
            tabs.addTab(w, tab_name)
        b_sync = QtWidgets.QPushButton("Send all parameters to board")
        b_sync.setToolTip("Use after a reconnect or board reset. Safe in v2.9 (every letter matches firmware v2.9).")
        b_sync.clicked.connect(self.push_all)
        left.addWidget(b_sync)

        # --- recording ---
        g = QtWidgets.QGroupBox("Recording (for the paper)"); left.addWidget(g)
        gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("Rate:"), 0, 0)
        self.rate_box = QtWidgets.QComboBox()
        self.rates = [("500 Hz (D2) — §4.8 logging", "2"), ("250 Hz (D4)", "4"), ("100 Hz (D10)", "10"), ("50 Hz (D20)", "20"), ("Off (D0)", "0")]
        self.rate_box.addItems([r[0] for r in self.rates])
        self.rate_box.setToolTip("500 Hz halves the logging downsample of the 1 kHz loop (v2.4 logged at 100 Hz, which was the\n"
                                 "largest scatter term in the B_v(τ̂) figure). ~48 kB/s at 921600 baud.")
        self.rate_box.currentIndexChanged.connect(lambda i: self.send("D" + self.rates[i][1]))
        gl.addWidget(self.rate_box, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Note:"), 1, 0)
        self.note = QtWidgets.QLineEdit(); self.note.setPlaceholderText("subject / motor serial / arm support / what this trial is")
        gl.addWidget(self.note, 1, 1)
        self.btn_rec = QtWidgets.QPushButton("●  Record")
        self.btn_rec.setStyleSheet("color:#c40000; font-weight:bold;")
        self.btn_rec.setMinimumHeight(32)
        self.btn_rec.clicked.connect(self.toggle_record)
        gl.addWidget(self.btn_rec, 2, 0, 1, 2)
        self.lbl_rec = QtWidgets.QLabel(""); gl.addWidget(self.lbl_rec, 3, 0, 1, 2)
        self.btn_mark = QtWidgets.QPushButton("Add marker to log")
        self.btn_mark.setToolTip("Writes the current telemetry time + the note text to the .meta.txt (e.g. 'start push', 'relax').")
        self.btn_mark.clicked.connect(lambda: self.add_marker(self.note.text() or "marker"))
        gl.addWidget(self.btn_mark, 4, 0, 1, 2)
        self.btn_an = QtWidgets.QPushButton("Analyze last recording → figures")
        self.btn_an.setEnabled(False)
        self.btn_an.clicked.connect(self.analyze_last)
        gl.addWidget(self.btn_an, 5, 0, 1, 2)

        # --- E08 protocol timer ---
        g = QtWidgets.QGroupBox("E08 protocol timer (§4.8 estimator comparison)"); left.addWidget(g)
        g.setToolTip("Stages: " + " → ".join(f"{n} {s} s" for n, s in E08_STAGES) +
                     "\nStart recording first. Each stage change is written as a marker so the analysis can split rest and motion.")
        gl = QtWidgets.QVBoxLayout(g)
        self.lbl_e08 = QtWidgets.QLabel("not running"); self.lbl_e08.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_e08.setStyleSheet("font-size:14px; font-weight:bold; padding:4px;")
        gl.addWidget(self.lbl_e08)
        row = QtWidgets.QHBoxLayout(); gl.addLayout(row)
        self.btn_e08 = QtWidgets.QPushButton("Start protocol"); self.btn_e08.clicked.connect(self.e08_start); row.addWidget(self.btn_e08)
        b = QtWidgets.QPushButton("Next stage"); b.clicked.connect(self.e08_next); row.addWidget(b)
        b = QtWidgets.QPushButton("Abort"); b.clicked.connect(self.e08_abort); row.addWidget(b)
        self.e08_timer = QtCore.QTimer(self); self.e08_timer.timeout.connect(self.e08_tick)

        # ================= RIGHT: plots + console =================
        right = QtWidgets.QVBoxLayout(); root.addLayout(right, 1)
        grid = pg.GraphicsLayoutWidget(); right.addWidget(grid, 1)

        p1 = grid.addPlot(row=0, col=0, title="Joint angle θ / deg")
        p1.addLegend(offset=(10, 10)); p1.showGrid(x=True, y=True, alpha=0.3)
        self.c_theta = p1.plot(pen=pg.mkPen('#1f77b4', width=2), name="θ")
        self.c_ref   = p1.plot(pen=pg.mkPen('#ff7f0e', width=2, style=QtCore.Qt.DashLine), name="θ_ref")

        p2 = grid.addPlot(row=0, col=1, title="Human torque τ̂_h / N·m")
        p2.addLegend(offset=(10, 10)); p2.showGrid(x=True, y=True, alpha=0.3)
        self.c_ts   = p2.plot(pen=pg.mkPen('#2ca02c', width=1.5), name="static")
        self.c_td   = p2.plot(pen=pg.mkPen('#d62728', width=1.0), name="direct")
        self.c_tdob = p2.plot(pen=pg.mkPen('#9467bd', width=2.2), name="DOB")

        p3 = grid.addPlot(row=1, col=0, title="Velocity / rad·s⁻¹   ·   Motor torque τ_m / N·m")
        p3.addLegend(offset=(10, 10)); p3.showGrid(x=True, y=True, alpha=0.3)
        self.c_vel  = p3.plot(pen=pg.mkPen('#17becf', width=1.5), name="vel")
        self.c_taum = p3.plot(pen=pg.mkPen('#8c564b', width=1.5), name="τ_m")

        p4 = grid.addPlot(row=1, col=1, title="Adaptive damping B_v   ·   I_q cmd / A")
        p4.addLegend(offset=(10, 10)); p4.showGrid(x=True, y=True, alpha=0.3)
        self.c_bv = p4.plot(pen=pg.mkPen('#e377c2', width=2), name="B_v")
        self.c_iq = p4.plot(pen=pg.mkPen('#7f7f7f', width=1.5), name="I_q")
        for p in (p2, p3, p4): p.setXLink(p1)

        self.console = QtWidgets.QPlainTextEdit(); self.console.setReadOnly(True)
        self.console.setMaximumHeight(120)
        self.console.setStyleSheet("font-family:Consolas,Menlo,monospace; font-size:11px;")
        right.addWidget(self.console)
        h = QtWidgets.QHBoxLayout(); right.addLayout(h)
        h.addWidget(QtWidgets.QLabel("Advanced command:"))
        self.entry = QtWidgets.QLineEdit()
        self.entry.setPlaceholderText("optional raw command, e.g.  L   or  Pw45   or  MVP0.25")
        self.entry.returnPressed.connect(self.on_entry)
        h.addWidget(self.entry)

        # serial + refresh timers
        self.worker = SerialWorker()
        self.worker.batch.connect(self.on_batch)
        self.worker.textline.connect(self.on_text)
        self.worker.connected.connect(self.on_connected)
        self.worker.start()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.scan_ports()
        self.set_controls_enabled(False)

    # ---------------- sub-panels ----------------
    def _build_passive_box(self):
        g = QtWidgets.QGroupBox("Passive moves ('G' goto — arm must be homed)")
        gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("Min [deg]"), 0, 0); self.sw_min = QtWidgets.QDoubleSpinBox(); self.sw_min.setRange(0, 150); self.sw_min.setValue(15); gl.addWidget(self.sw_min, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Max [deg]"), 0, 2); self.sw_max = QtWidgets.QDoubleSpinBox(); self.sw_max.setRange(0, 150); self.sw_max.setValue(90); gl.addWidget(self.sw_max, 0, 3)
        gl.addWidget(QtWidgets.QLabel("Reps"), 1, 0); self.sw_reps = QtWidgets.QSpinBox(); self.sw_reps.setRange(1, 50); self.sw_reps.setValue(5); gl.addWidget(self.sw_reps, 1, 1)
        gl.addWidget(QtWidgets.QLabel("Dwell [s]"), 1, 2); self.sw_dwell = QtWidgets.QDoubleSpinBox(); self.sw_dwell.setRange(0, 30); self.sw_dwell.setValue(1.0); gl.addWidget(self.sw_dwell, 1, 3)
        b = QtWidgets.QPushButton("Go to min"); b.clicked.connect(lambda: self.goto_deg(self.sw_min.value())); gl.addWidget(b, 2, 0, 1, 2)
        b = QtWidgets.QPushButton("Go to max"); b.clicked.connect(lambda: self.goto_deg(self.sw_max.value())); gl.addWidget(b, 2, 2, 1, 2)
        self.btn_sweep = QtWidgets.QPushButton("▶ Sweep min ↔ max"); self.btn_sweep.clicked.connect(self.sweep_toggle); gl.addWidget(self.btn_sweep, 3, 0, 1, 4)
        self.sweep_timer = QtCore.QTimer(self); self.sweep_timer.timeout.connect(self.sweep_tick)
        return g

    def _build_advanced_box(self):
        g = QtWidgets.QGroupBox("Advanced (rarely needed)")
        gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("Torque controller"), 0, 0)
        self.tmode = QtWidgets.QComboBox(); self.tmode.addItems(["foc_current (Pm2)", "dc_current (Pm1)", "voltage (Pm0)"])
        self.tmode.setToolTip("foc_current is the mode every lab constant was measured in. Voltage mode only if the current sense is dead.")
        self.tmode.currentIndexChanged.connect(lambda i: self.send("Pm" + ["2", "1", "0"][i]))
        gl.addWidget(self.tmode, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Measured-Iq sign"), 1, 0)
        self.iqsign = QtWidgets.QComboBox(); self.iqsign.addItems(["auto (homing)", "force +1 (Pq1)", "force −1 (Pq-1)"])
        self.iqsign.setToolTip("Homing auto-calibrates the sign of the measured q-current. Override only if 'L' shows the wrong sign\n(symptom: τ̂ ≈ 2× gravity at rest).")
        self.iqsign.currentIndexChanged.connect(lambda i: i and self.send(["", "Pq1", "Pq-1"][i]))
        gl.addWidget(self.iqsign, 1, 1)
        return g

    # ---------------- connection ----------------
    def scan_ports(self):
        cur = self.port_box.currentText()
        self.port_box.clear()
        for p in list_ports.comports():
            self.port_box.addItem(f"{p.device}  —  {p.description}", p.device)
        i = self.port_box.findText(cur)
        if i >= 0: self.port_box.setCurrentIndex(i)

    def toggle_conn(self):
        if self.btn_conn.text() == "Connect":
            if self.port_box.currentIndex() < 0:
                self.log("✗ no serial port selected"); return
            self.worker.open_port(self.port_box.currentData())
        else:
            self.worker.close_port()

    def on_connected(self, ok):
        self.btn_conn.setText("Disconnect" if ok else "Connect")
        self.set_controls_enabled(ok)
        if ok:
            QtCore.QTimer.singleShot(600, lambda: (self.send("L"), self.send("D" + self.rates[self.rate_box.currentIndex()][1])))
        else:
            self.lbl_state.setText("NOT CONNECTED")
            self.lbl_state.setStyleSheet("background:#666; color:white; font-size:20px; "
                                         "font-weight:bold; padding:10px; border-radius:6px;")

    def set_controls_enabled(self, ok):
        for w in (self.btn_home, self.btn_run, self.btn_idle, self.assist,
                  self.btn_rec, self.rate_box, self.btn_sweep):
            w.setEnabled(ok)

    # ---------------- data ----------------
    def on_batch(self, rows):
        if self.t0 is None: self.t0 = rows[0][0]
        self.rows.extend(rows)
        self.last_t_ms = rows[-1][0]
        if self.recording:
            self.csv_writer.writerows(rows)
            self.n_recorded += len(rows)

    def on_text(self, s):
        self.log(s)
        if s.startswith("L state="):
            self.last_status = s
            if "fw=v2.9" not in s:
                self.log("⚠ firmware is not v2.9 — the Lab-constants tab and T_est need firmware v2.9")
        elif s.startswith("EV,THERM"):
            self.log("⚠ thermal derate: " + s)

    def refresh(self):
        if not self.rows: return
        a = np.asarray(self.rows, dtype=float)
        t = (a[:, 0] - self.t0) / 1000.0
        a = a[t > t[-1] - HISTORY_S]
        tt = (a[:, 0] - self.t0) / 1000.0
        c = COL
        self.c_theta.setData(tt, a[:, c["theta"]]*DEG)
        self.c_ref.setData(tt, a[:, c["theta_ref"]]*DEG)
        self.c_ts.setData(tt, a[:, c["tau_s"]])
        self.c_td.setData(tt, a[:, c["tau_d"]])
        self.c_tdob.setData(tt, a[:, c["tau_dob"]])
        self.c_vel.setData(tt, a[:, c["vel"]])
        self.c_taum.setData(tt, a[:, c["tau_m"]])
        self.c_bv.setData(tt, a[:, c["Bv"]])
        self.c_iq.setData(tt, a[:, c["iq_cmd"]])
        last = a[-1]
        st = int(last[c["state"]])
        self.lbl_state.setText(STATE_NAMES.get(st, str(st)))
        self.lbl_state.setStyleSheet(
            f"background:{STATE_COLORS.get(st,'#666')}; color:white; font-size:20px; "
            "font-weight:bold; padding:10px; border-radius:6px;")
        est = ["static", "direct", "DOB"][self.est_group.checkedId()]
        tau = {"static": "tau_s", "direct": "tau_d", "DOB": "tau_dob"}[est]
        self.lbl_readout.setText(
            f"θ = {last[c['theta']]*DEG:6.1f}°      τ̂_h ({est}) = {last[c[tau]]:+.3f} N·m")
        T = last[c["T_est"]]
        if math.isnan(T):
            self.lbl_temp.setText("T_est = — (firmware < v2.9)")
        else:
            col = "#1a8a3c" if T < 60 else "#c77d00" if T < 70 else "#c40000"
            self.lbl_temp.setText(f"estimated winding T = {T:.0f} °C" + ("   ⚠ DERATING" if T >= 70 else ""))
            self.lbl_temp.setStyleSheet(f"color:{col}; font-weight:bold;")
        if self.recording:
            self.lbl_rec.setText(f"● recording… {self.n_recorded} samples")

    # ---------------- actions ----------------
    def send(self, cmd):
        self.worker.send(cmd)
        self.log(f"> {cmd}")

    def push_all(self):
        for r in self.param_rows: r.push()
        self.send(f"A{self.assist.value()/100:.2f}")
        self.send(f"E{self.est_group.checkedId()}")

    def stop_all(self):
        self.sweep_running = False; self.sweep_timer.stop(); self.btn_sweep.setText("▶ Sweep min ↔ max")
        self.send("O0")

    def _assist_changed(self, v):
        self.lbl_assist.setText(f"Assist gain A = {v/100:.2f}")
        self._assist_deb.start()

    def on_entry(self):
        c = self.entry.text().strip()
        if c: self.send(c)
        self.entry.clear()

    def log(self, s):
        self.console.appendPlainText(s)
        sb = self.console.verticalScrollBar(); sb.setValue(sb.maximum())

    # ---------------- passive goto / sweep ----------------
    def goto_deg(self, deg):
        self.send(f"G{deg/DEG:.4f}")

    def sweep_toggle(self):
        if self.sweep_running:
            self.stop_all(); return
        lo, hi, n = self.sw_min.value(), self.sw_max.value(), self.sw_reps.value()
        self.sweep_targets = [lo, hi] * n + [lo]
        self.sweep_i = 0; self.sweep_running = True; self.sweep_arrived_at = None
        self.btn_sweep.setText("■ Stop sweep")
        self.add_marker(f"sweep start {lo:.0f}-{hi:.0f} deg x{n}")
        self.goto_deg(self.sweep_targets[0]); self.sweep_timer.start(100)

    def sweep_tick(self):
        if not self.sweep_running or not self.rows: return
        last = self.rows[-1]
        th, v = last[COL["theta"]]*DEG, last[COL["vel"]]
        tgt = self.sweep_targets[self.sweep_i]
        if abs(th - tgt) < 2.0 and abs(v) < 0.05:
            if self.sweep_arrived_at is None: self.sweep_arrived_at = time.monotonic()
            if time.monotonic() - self.sweep_arrived_at >= self.sw_dwell.value():
                self.sweep_i += 1; self.sweep_arrived_at = None
                if self.sweep_i >= len(self.sweep_targets):
                    self.sweep_running = False; self.sweep_timer.stop()
                    self.btn_sweep.setText("▶ Sweep min ↔ max"); self.add_marker("sweep done"); return
                self.goto_deg(self.sweep_targets[self.sweep_i])
        else:
            self.sweep_arrived_at = None

    # ---------------- E08 protocol timer ----------------
    def e08_start(self):
        if not self.recording:
            self.log("⚠ start recording first (the stage markers go into the .meta.txt)")
        self.e08_i = -1; self.e08_next(); self.e08_timer.start(250)

    def e08_next(self):
        self.e08_i += 1
        if self.e08_i >= len(E08_STAGES):
            self.e08_abort(); self.add_marker("E08 protocol complete"); self.lbl_e08.setText("complete"); return
        name, secs = E08_STAGES[self.e08_i]
        self.e08_t_end = time.monotonic() + secs
        self.add_marker(f"stage {self.e08_i+1}: {name}")
        QtWidgets.QApplication.beep()

    def e08_tick(self):
        if self.e08_i < 0 or self.e08_i >= len(E08_STAGES): return
        name, secs = E08_STAGES[self.e08_i]
        rem = self.e08_t_end - time.monotonic()
        self.lbl_e08.setText(f"{self.e08_i+1}/{len(E08_STAGES)}  {name}   {max(rem,0):4.0f} s")
        if rem <= 0: self.e08_next()

    def e08_abort(self):
        self.e08_timer.stop(); self.e08_i = -1; self.lbl_e08.setText("not running")

    # ---------------- recording ----------------
    def add_marker(self, text):
        t = self.last_t_ms if self.last_t_ms is not None else float("nan")
        self.markers.append((t, text)); self.log(f"◆ marker t_ms={t:.0f}: {text}")

    def toggle_record(self):
        if not self.recording:
            os.makedirs("logs", exist_ok=True)
            fname = time.strftime("logs/exo_%Y%m%d_%H%M%S.csv")
            self.csv_file = open(fname, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(FIELDS)
            self.n_recorded = 0; self.recording = True; self.last_log = fname; self.markers = []
            self.send("L")                                   # fresh status line for the meta file
            self.btn_rec.setText("■  Stop recording")
            self.log(f"● recording → {fname}")
        else:
            self.recording = False
            self.csv_file.close()
            self.write_meta()
            self.btn_rec.setText("●  Record")
            self.lbl_rec.setText(f"saved {self.n_recorded} samples → {self.last_log}")
            self.btn_an.setEnabled(True)
            self.log(f"■ saved {self.n_recorded} samples (+ .meta.txt)")

    def write_meta(self):
        try:
            with open(self.last_log.replace(".csv", ".meta.txt"), "w", encoding="utf-8") as f:
                f.write(f"log: {self.last_log}\nsaved: {time.strftime('%Y-%m-%d %H:%M:%S')}\ngui: {GUI_VERSION}\n")
                f.write(f"firmware status: {self.last_status}\n")
                f.write(f"note: {self.note.text()}\n")
                f.write(f"estimator: {['static','direct','DOB'][self.est_group.checkedId()]}\nassist A: {self.assist.value()/100:.2f}\n")
                f.write(f"telemetry: {self.rates[self.rate_box.currentIndex()][0]}\nsamples: {self.n_recorded}\n")
                f.write("parameters (GUI values at stop):\n")
                for r in self.param_rows: f.write(f"  P{r.letter}  {r.label}: {r.value():.{r.decimals}f}\n")
                f.write("markers (t_ms, text):\n")
                for t, s in self.markers: f.write(f"  {t:.0f}, {s}\n")
        except Exception as e:
            self.log(f"✗ meta file failed: {e}")

    def analyze_last(self):
        if not self.last_log: return
        here = os.path.dirname(os.path.abspath(__file__))
        cands = [os.path.join(here, "analyze_log.py"), os.path.join(here, "..", "tools", "analyze_log.py"),
                 os.path.join(here, "..", "tools", "ExArMoLogAnalyzer_v2.4.py")]
        script = next((c for c in cands if os.path.isfile(c)), None)
        if not script:
            self.log("✗ analyze_log.py / ExArMoLogAnalyzer not found next to the GUI or in ../tools"); return
        self.log(f"analyzing {self.last_log} with {os.path.basename(script)} …")
        try:
            if "Analyzer" in script:
                subprocess.Popen([sys.executable, script, self.last_log])   # interactive window
            else:
                out = subprocess.run([sys.executable, script, self.last_log], capture_output=True, text=True, timeout=120)
                self.log(out.stdout.strip() or out.stderr.strip())
        except Exception as e:
            self.log(f"✗ analysis failed: {e}")

    def closeEvent(self, ev):
        if self.recording: self.toggle_record()
        self.worker.stop()
        ev.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Main(); w.show()
    sys.exit(app.exec_())
