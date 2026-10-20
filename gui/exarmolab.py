#!/usr/bin/env python3
"""
ExArMo Lab — Motor Tester GUI  (v2.3 · g7-C · 18 Oct 2026: reads the rehab v2.9.2 thermistor · merge of Mac g6-D1 (E9 drive-linearity, tab 11 E10 ratio + hand sweep) and PC g7-A (rehab firmware v2.9 support) · light theme, paper-ready)
====================================================================================================
Companion to the DEDICATED lab firmware `src/main.cpp` v2.7.1-R, and — since
g7-A — ALSO to the ExArMo REHAB firmware v2.9 on the finished device. All
quantities are MOTOR-SHAFT units from the device link.

[g7-B] MERGE (18 Oct 2026)
--------------------------
    Two forks grew from g5-A (20 Sep): the Mac copy got g6-A…g6-D1 (13–15 Oct: tab 11
    E10 kinematic ratio N with motor-driven and hand sweeps, velocity-PID fields, runaway
    guard; E9 drive-linearity readout per load direction; g6-D1 abs-% / negative-K sign
    warning) and the PC copy got g7-A (18 Oct: rehab firmware v2.9 support, below). g7-B
    is a 3-way merge of both; no tab or feature of either fork was dropped.

[g7-A] REHAB FIRMWARE v2.9 SUPPORT (18 Oct 2026)
-------------------------------------------------
    The program detects which firmware answers (13-field telemetry or
    `fw=v2.9` in the `L` status line = rehab; 11 fields / `EV,BOOT` = lab) and,
    on the rehab firmware, translates every lab command it would have sent:

      lab command       rehab v2.9 equivalent
      C<A>  current     EMULATED: LAB velocity mode `V±8` with the current limit
                        (`MLC` + `Pi`) ramped to |A| at the `Pr` rate. With the
                        shaft BLOCKED (E9 strut, load cell, hard stop) the velocity
                        PID saturates at the limit, so the measured current equals
                        the limit — a constant-current stall. A FREE shaft would
                        spin, so a runaway guard (joint speed > 0.6 rad/s for
                        0.3 s) sends O0 and raises a fault, like the lab firmware's
                        RUNAWAY trip. Telemetry `iq_cmd` is 0 in LAB mode, so the
                        emulated ramp level is published as `iqc` instead.
      C0                ramp the limit to 0 (state stays CURRENT at 0 A)
      O0                O0 (then the limit is restored to the cap)
      V<rad/s>          V (native; the firmware re-tunes the velocity PID on
                        every entry from IDLE, so MVP/MVI/MVF are re-sent after it)
      H  hold           G<θ now>: PASSIVE hold at the current joint angle
                        (needs the rehab homing first — Diagnostics → HOME)
      Pr<A/s> ramp      kept on this side (rehab `Pr` is PHASE RESISTANCE — never
                        forwarded)
      Ps±1 direction    kept on this side (sign of the emulated current)
      Pi<A> limit       the cap of the emulation (and the rehab Iq limit)
      Z, A              not available on the rehab firmware — blocked (rehab `A`
                        is the ASSIST GAIN)
      K  keepalive      dropped (no watchdog on the rehab firmware)
      D, L, M…          native

    Telemetry mapping (rehab → this program):
      angle/velocity    joint × 9.05 × DIR_SIGN(−1)  → motor-shaft rad, rad/s
      state             0 IDLE→0 · 7 LAB→3 CURRENT while emulating, else 1
                        VELOCITY · 6 PASSIVE→2 HOLDING · 1/2/3 homing→4 ·
                        4 RUN→5 · 5 FAULT→6
      flags             synthesised (FOC init, current sense, Vbus set; motor
                        enabled/driving while not IDLE; thermistor CLEAR)
      temperature       the rehab board has NO thermistor. Its software thermal
                        MODEL `T_est` is offered as source "model" and labelled
                        as such everywhere — it is the firmware's own estimate,
                        never a measurement. The 18 Oct log showed that estimate
                        running away in IDLE (26 000 °C): Diagnostics shows it
                        live so you can see whether it is sane before trusting a
                        thermal tab.
    What this does NOT change: every tab, fit, log format and the arm sensor
    board. Through the capstan the "KT" tabs measure K_t,j (joint torque per
    reported ampere, E9: 0.775), not the bare-motor K_t.

WHAT CHANGED FROM v1 (and why nothing showed up before)
-------------------------------------------------------
[g2-A] `dev["last"]` was the ONLY channel for firmware messages, and
       `dev_send()` overwrote it with "> K" every single keepalive. So
       EV,FOCFAIL / EV,WATCHDOG / ACK lines were clobbered within a second of
       arriving and the footer banner could essentially never fire. TX and RX
       are now separate, faults are STICKY, and there is a real scrolling
       console on the Diagnostics tab.
[g2-B] The step engine assumed the arm reached the load cell after a fixed
       ~2 s "engage" wait. It never checked. If the arm was short of the cell
       (or pushing the wrong way) every step recorded 0 g with no complaint.
       Engagement now WAITS FOR CONTACT — a real rise in grams — and aborts
       with a clear message if contact never happens.
[g2-C] No way to verify push DIRECTION before a sweep. A direction toggle
       ('Ps±1') and a manual jog live on the Diagnostics tab now.
[g2-D] Telemetry gained id, voltage.q and a health/flags word, so the GUI can
       distinguish "no current commanded" from "current commanded but not
       measured" from "FOC never initialised".
[g2-E] Connect now resets the board (O0), sets the telemetry rate and asks for
       a status dump, instead of trusting whatever state the board was left in.
[g2-F] Keepalive raised to 2 Hz against the firmware's 3 s watchdog.

Tabs:
 0  Diagnostics       console, live flags, jog, direction, self-test
 1  KV · Back-EMF     spin at speed steps, type multimeter V, live KV fit
 2  KT · Add weight   freewheel → rotate arm horizontal → HOLD; add weights,
                      record averaged Iq per step; fit τ vs Iq → Kt
 3  KT · Loadcell     Iq steps; per step wait `settle` (discard) then average
                      the loadcell robustly; fit Kt             [sensor board]
 4  Heat over force   constant Iq; log loadcell+temperature every 1 s with a
                      high-temperature auto-stop                [sensor board]
 5  Capstan torque    same engine as 3 through the capstan; output Kt and
                      optional efficiency vs the motor Kt from tab 3
 6  Thermal rise      [g3-A] hold one current, log the temperature curve, and
                      fit the motor's thermal model: tau, steady-state rise,
                      thermal resistance, and the continuous current that
                      corresponds to an allowed winding rise
 10 E9 ground truth  [g5-A] the E9 protocol: a load cell in a PINNED STRUT
                      holds the arm, the motor ramps current 0->Imax->0 over
                      60 s, and the strut's force becomes the truth the
                      firmware's torque estimate is judged against. The moment
                      arm is the perpendicular from the joint to the line
                      through both pins (NOT F x lever), the arm's weight is
                      never tared out, and the run reports slope +/- SE,
                      intercept, R2, RMSE, residual vs angle, sigma_0 and the
                      friction read off the hysteresis, against the
                      acceptance criteria registered before the run
 9  Arm calibrate    [g4-B] calibrate that board FROM THE GUI, against its
                      raw counts: zero + known masses -> counts/gram, and an
                      IMU zero (and, if the axes are not parallel, a span
                      gain). Kept in labdata/arm_cal.json, so it survives a
                      reset of the board and a restart of this program
 8  Arm angle & force [g4-A] the rehab-arm bench: joint ANGLE from the WIT
                      IMU and FORCE at the load cell, both from ONE ESP32 on
                      ONE clock, so the two can honestly be plotted against
                      each other. Live view, a per-sample CSV log, averaged
                      captures, and a gravity fit tau = M*cos(th-phi) + c
                                                                 [3rd serial]

[g4-A] ARM SENSOR BOARD, 3rd serial, 115200 baud.
    The ESP32 sketch prints one line per 250 ms:
      "Roll_deg: 1.23 | Pitch_deg: -4.56 | Yaw_deg: 0.00 | LoadCell_raw: 936249"
    LoadCell_raw appears instead of LoadCell_g until that board has been tared
    AND calibrated, and any field it cannot vouch for is the literal "nan".
    The board's own tare/scale ('t' and 'c') live in ITS RAM and die on its
    next reset, so tab 9 calibrates from the raw counts on this side and
    keeps the result in labdata/arm_cal.json. Either way, lc_cal is NOT
    involved: tabs 3-6 keep their own load cell, untouched.

[g3-A] TEMPERATURE NOW COMES FROM THE DRIVER BOARD.
    Lab firmware v2.7.1 reads a 100k NTC on GPIO3/PA2 and publishes it as an 11th
    telemetry field, sampled on the SAME clock as Iq. Until now temperature
    arrived on the load-cell Arduino's own serial port, on its own timebase,
    which is fine for "did it get hot" and useless for "how fast did it get
    hot" — two devices, two clocks, no common t=0. The load-cell stream is
    still read and is still the automatic fallback when the board has no bead
    fitted; temp_src() says which one is live, and the fallback is never
    silent, because a thermal number whose provenance you cannot state is not
    a measurement.

Device telemetry (11 fields):
    D,t_ms,state,angle,vel,iq_meas,iq_cmd,id_meas,vq,flags,temp_C
    flags bit5 = thermistor healthy. temp_C is 0 when that bit is CLEAR — an
    unread sensor must never be mistaken for a cold motor, so this program
    gates on the flag and never on the value.
[g4-E] ONE SENSOR BOARD. The second Arduino — a load cell on its own port,
    with its own calibration file, and a temperature bead on its own clock —
    is gone. The ESP32 arm board carries the cell AND the IMU and answers on
    one port at 115200; see [g4-A] below for its line format. Tabs 3-7 read it
    through the same lc_* names they always used, now a shim over that one
    stream, so there is exactly one load cell and one calibration in the
    program. Its calibration is tab 9's, in labdata/arm_cal.json; the old
    labdata/loadcell_cal.json belonged to the old cell and is not read.
    Temperature comes from the driver board's thermistor or not at all.

Run:  pip install pyserial   →   python ExarmoLabGui.py
"""

import tkinter as tk
import tkinter.ttk as ttk
import tkinter.messagebox as messagebox
import threading, time, math, csv, os, re, json, collections
from datetime import datetime
import serial, serial.tools.list_ports

BAUD_DEV = 921600
BAUD_ARM = 115200          # [g4-A] arm sensor board (ESP32: IMU + HX711)
TELEM_MS = 20
KEEPALIVE_S = 0.5          # [g2-F] 2 Hz against the firmware's 3 s watchdog
G_ACC    = 9.80665

# ── light theme (paper-friendly) ───────────────────────────────
BG      = "#f2f4f8"
SURFACE = "#ffffff"
FIELD   = "#eef1f6"
BORDER  = "#d5dbe6"
TEXT    = "#1c2433"
MUTED   = "#5c6a80"
FAINT   = "#8b96a8"
BLUE    = "#2563eb"
GREEN   = "#118a52"
RED     = "#d3455b"
AMBER   = "#b45309"
PURPLE  = "#7c3aed"

F_TITLE = ("Segoe UI", 24, "bold")
F_H     = ("Segoe UI", 13, "bold")
F_LBL   = ("Segoe UI", 13)
F_SM    = ("Segoe UI", 11)
F_BTN   = ("Segoe UI", 13, "bold")
F_MONO  = ("Consolas", 16)
F_BIG   = ("Consolas", 40, "bold")

STATE_NAMES = {0: "IDLE (free)", 1: "VELOCITY", 2: "HOLDING", 3: "CURRENT",
               4: "HOMING (rehab)", 5: "RUN · admittance (rehab)", 6: "FAULT (rehab)"}   # [g7-A]

# ── [g7-A] firmware kind + rehab-v2.9 adapter state ────────────────────────
N_CAPSTAN   = 9.05       # kinematic ratio (E10) — joint → motor shaft
DIR_SIGN    = -1.0       # rehab main.cpp: theta = DIR_SIGN·(motor − off)/N
EMU_V       = 8.0        # [rad/s motor] velocity target used to stall at the current limit
EMU_RUNAWAY = 0.6        # [rad/s joint] sustained for EMU_RUNAWAY_S → abort (shaft was free)
EMU_RUNAWAY_S = 0.3
fw = dict(kind=None,            # None (unknown yet) | "lab" | "rehab"
          homed=False, lim=(None, None), T_est=None, iq_lim=None, rstate=0,
          emu=False,            # emulated current mode active (rehab LAB mode)
          emu_target=0.0, emu_level=0.0, emu_rate=20.0, emu_cap=4.0, emu_dir=1,
          emu_sent=None, emu_t_run=None, pid=[], hold_theta=0.0, fw_line="")
fw_lock = threading.Lock()

FLAG_BITS = [(0x01, "FOC init"), (0x02, "motor enabled"),
             (0x04, "FOC armed (driving)"), (0x08, "current sense"), (0x10, "Vbus"),
             (0x20, "thermistor (GPIO3)")]        # [g3-A]

running = True

# ── device link (lab firmware) ─────────────────────────────────
dev = dict(ser=None, lock=threading.Lock())
# [g2-A] three SEPARATE channels. v1 had one string that the keepalive echo
# overwrote twice a second, which is why firmware faults were never visible.
link = dict(tx="", rx="", fault="", fw="")
console = collections.deque(maxlen=500)
console_seq = [0]          # monotonic — len() saturates once the deque is full
console_lock = threading.Lock()

T = dict(state=0, ang=0.0, vel=0.0, iq=0.0, iqc=0.0, idq=0.0, vq=0.0,
         flags=-1, t_rx=0.0, nfields=0,
         temp=None, temp_ok=False)              # [g3-A] driver-board thermistor
# Producer/consumer discipline: the two reader THREADS append to these while
# the Tk thread and the experiment worker threads read them. A bounded deque
# evicts from the left on every append, so a concurrent read raises
# "deque mutated during iteration". That race has been here all along — it used
# to kill the tick chain outright and look like "the numbers just stopped".
# Never touch these directly; go through snap_hist() / snap_lc().
hist_lock = threading.Lock()
iq_hist  = collections.deque(maxlen=800)     # (t, iq_meas)
iqc_hist = collections.deque(maxlen=800)     # (t, iq_cmd from firmware)
rpm_hist = collections.deque(maxlen=800)     # (t, rpm)
ang_hist = collections.deque(maxlen=800)     # (t, shaft angle rad, accumulated)
# [g3-A] Temperature moves in minutes, not milliseconds, so this one is sized
# for a whole thermal run rather than for a 60 s plot window: 12000 samples at
# the 20 ms telemetry rate is only 4 minutes, but the experiment below decimates
# to its own log period and keeps its own rows — this deque is for the live
# chart and the short averages.
temp_hist = collections.deque(maxlen=12000)  # (t, degC) from the DRIVER board


def snap_hist(dq):
    with hist_lock:
        return list(dq)



def hist_at(dq, t):
    """A (t, v) history, read AT time t, linearly interpolated.

    This is what lets a load-cell sample be paired with the current that
    actually caused it. The cell converts at 10-80 Hz and the driver's
    telemetry arrives at 50 Hz; if a row pairs the force with whatever Iq
    happens to be current when the row is built, the two can be a whole cell
    period apart — 0.2 A of skew at 2 A/s, which tilts a fitted slope by a
    fifth and looks exactly like a real result.
    """
    pts = snap_hist(dq)
    if not pts:
        return None
    if t <= pts[0][0]:
        return pts[0][1]
    if t >= pts[-1][0]:
        return pts[-1][1]
    lo, hi = 0, len(pts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if pts[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, v0), (t1, v1) = pts[lo], pts[hi]
    if t1 <= t0:
        return v0
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def clog(text, kind="rx"):
    with console_lock:
        console.append((time.strftime("%H:%M:%S"), kind, text))
        console_seq[0] += 1


def _dev_write(cmd, quiet=False):
    with dev["lock"]:
        s = dev["ser"]
        if s is None or not s.is_open:
            link["fault"] = "device not connected"
            return False
        try:
            s.write((cmd + "\n").encode())
            link["tx"] = cmd
            if cmd != "K" and not quiet:          # don't spam the console
                clog(cmd, "tx")
            return True
        except Exception as e:
            link["fault"] = f"tx error: {e}"
            clog(f"tx error: {e}", "err")
            return False


def dev_send(cmd, raw=False):
    """Send a LAB-firmware command. On the rehab firmware it is translated
    ([g7-A], see the header); raw=True sends the text as typed."""
    if raw or fw["kind"] != "rehab":
        return _dev_write(cmd)
    return rehab_send(cmd)


def rehab_send(cmd):
    """[g7-A] lab command → rehab v2.9 command(s)."""
    c = cmd.strip()
    if not c:
        return False
    k = c[0]
    ok = True
    if k == "K":
        return True                                            # no watchdog to feed
    if k == "C":
        try:
            a = float(c[1:])
        except ValueError:
            clog(f"bad current command {c}", "err"); return False
        with fw_lock:
            fw["emu_target"] = max(-fw["emu_cap"], min(fw["emu_cap"], a))
            starting = not fw["emu"]
            fw["emu"] = True
            fw["emu_t_run"] = None
        clog(f"{c}  ⇒ emulated current {fw['emu_target']:+.3f} A (limit ramp @ {fw['emu_rate']:g} A/s, V{EMU_V:g})", "tx")
        if starting:
            if fw["rstate"] not in (0, 5, 7):
                _dev_write("O0")                               # V only enters LAB from IDLE/FAULT
            _dev_write("MLC0.00"); _dev_write("Pi0.00")
            fw["emu_level"] = 0.0; fw["emu_sent"] = None
        _emu_push_velocity()
        return True
    if k == "O":
        with fw_lock:
            was = fw["emu"]; fw["emu"] = False; fw["emu_level"] = 0.0; fw["emu_target"] = 0.0; fw["emu_sent"] = None
        ok = _dev_write(c)
        if was or fw["rstate"] == 7:
            _dev_write(f"Pi{fw['emu_cap']:.2f}"); _dev_write(f"MLC{fw['emu_cap']:.2f}")
        T["iqc"] = 0.0
        return ok
    if k == "V":
        try:
            v = float(c[1:])
        except ValueError:
            return _dev_write(c)
        from_idle = fw["rstate"] in (0, 5)
        if from_idle:
            _dev_write(f"Pi{fw['emu_cap']:.2f}"); _dev_write(f"MLC{fw['emu_cap']:.2f}")
        ok = _dev_write(f"V{v:.3f}")
        if from_idle and fw["pid"]:                            # doLabVel() overwrote the PID on entry
            for pc in fw["pid"]:
                _dev_write(pc)
        return ok
    if k == "H":
        if not fw["homed"]:
            link["fault"] = ("REHAB firmware: HOLD needs the joint homed first — "
                             "Diagnostics → 'HOME (rehab H)' then try again")
            clog("H ⇒ blocked: not homed", "err")
            return False
        th = fw["hold_theta"]
        clog(f"H  ⇒ G{th:.4f}  (passive hold at the current joint angle)", "tx")
        if fw["rstate"] == 7 or fw["emu"]:
            rehab_send("O0")
        return _dev_write(f"G{th:.4f}")
    if k == "P" and len(c) >= 2:
        sub = c[1]
        try:
            v = float(c[2:])
        except ValueError:
            v = None
        if sub == "r":                                         # lab ramp rate; rehab Pr = PHASE RESISTANCE
            if v is not None:
                fw["emu_rate"] = max(0.1, v)
            clog(f"{c}  ⇒ ramp rate {fw['emu_rate']:g} A/s (kept on the PC)", "tx")
            return True
        if sub == "s":
            if v is not None:
                fw["emu_dir"] = -1 if v < 0 else 1
            clog(f"{c}  ⇒ emulated current direction {fw['emu_dir']:+d} (kept on the PC)", "tx")
            _emu_push_velocity()
            return True
        if sub == "i":
            if v is not None:
                fw["emu_cap"] = max(0.1, min(8.0, v))
            clog(f"{c}  ⇒ current cap {fw['emu_cap']:.2f} A", "tx")
            if not fw["emu"]:
                _dev_write(f"Pi{fw['emu_cap']:.2f}"); _dev_write(f"MLC{fw['emu_cap']:.2f}")
            return True
        return _dev_write(c)                                   # other P letters go through as typed
    if k in ("Z", "A"):
        clog(f"{c} ⇒ not available on the rehab firmware"
             + (" (rehab 'A' is the assist gain — not sent)" if k == "A" else ""), "err")
        return False
    if k == "M":
        if c[:3] in ("MVP", "MVI", "MVF"):
            fw["pid"] = [x for x in fw["pid"] if not x.startswith(c[:3])] + [c]
        return _dev_write(c)
    return _dev_write(c)                                       # D, L and anything else: native


def _emu_push_velocity():
    """(Re)send the LAB velocity target with the sign of the emulated current."""
    if not fw["emu"]:
        return
    sgn = fw["emu_dir"] * (1 if fw["emu_target"] >= 0 else -1)
    _dev_write(f"V{sgn * EMU_V:.1f}", quiet=True)


def emu_loop():
    """[g7-A] 50 Hz: ramp the rehab current limit toward the emulated target
    and publish it as iq_cmd; abort if the shaft turns out to be free."""
    last = time.time()
    while running:
        time.sleep(0.02)
        now = time.time(); dt = now - last; last = now
        if fw["kind"] != "rehab" or not fw["emu"]:
            continue
        with fw_lock:
            tgt = fw["emu_target"]; lvl = fw["emu_level"]; rate = fw["emu_rate"]
            step = rate * dt
            if abs(tgt - lvl) <= step:
                lvl = tgt
            else:
                lvl += step if tgt > lvl else -step
            fw["emu_level"] = lvl
            signed = fw["emu_dir"] * lvl
        T["iqc"] = signed
        with hist_lock:
            iqc_hist.append((now, signed))
        if fw["emu_sent"] is None or abs(abs(lvl) - fw["emu_sent"]) >= 0.004 or (abs(lvl) == abs(tgt) and fw["emu_sent"] != abs(lvl)):
            _dev_write(f"MLC{abs(lvl):.3f}", quiet=True)
            _dev_write(f"Pi{abs(lvl):.3f}", quiet=True)
            fw["emu_sent"] = abs(lvl)
        # runaway guard: an emulated current with a free shaft just spins at V
        jv = abs(T["vel"]) / N_CAPSTAN
        if jv > EMU_RUNAWAY and abs(lvl) > 0.05:
            if fw["emu_t_run"] is None:
                fw["emu_t_run"] = now
            elif now - fw["emu_t_run"] > EMU_RUNAWAY_S:
                rehab_send("O0")
                link["fault"] = ("RUNAWAY (emulated current mode): the joint moved at "
                                 f"{jv:.2f} rad/s — the shaft must be BLOCKED for a current "
                                 "command on the rehab firmware. Motor released.")
                clog("emulated current aborted: shaft free", "err")
        else:
            fw["emu_t_run"] = None


fw.setdefault("T_src", "model")   # [g7-C] rehab v2.9.2+ reports T_src=ntc when a bead is fitted


def rehab_parse_status(line):
    """[g7-A] `L state=… homed=1 … limits_deg=[a,b] … T_est=… iq_lim=… fw=v2.9`."""
    m = re.search(r"T_src=(\w+)", line)                         # [g7-C]
    if m:
        fw["T_src"] = m.group(1)
    m = re.search(r"homed=(\d)", line)
    if m:
        fw["homed"] = m.group(1) == "1"
    m = re.search(r"limits_deg=\[([-\d.]+),([-\d.]+)\]", line)
    if m:
        fw["lim"] = (float(m.group(1)), float(m.group(2)))
    m = re.search(r"iq_lim=([-\d.]+)", line)
    if m:
        fw["iq_lim"] = float(m.group(1))
    m = re.search(r"fw=(\S+)", line)
    if m:
        fw["fw_line"] = m.group(1)
        if m.group(1).startswith("v2.9"):
            _set_fw_kind("rehab", f"rehab {m.group(1)}")
        else:
            _set_fw_kind("rehab", f"rehab {m.group(1)} (expected v2.9 — some commands may differ)")


def _set_fw_kind(kind, label=None):
    if fw["kind"] != kind:
        fw["kind"] = kind
        clog(f"--- firmware detected: {kind.upper()} ---", "sys")
        if kind == "rehab":
            link["fault"] = ""
    if label:
        link["fw"] = label


def dev_reader():
    last_keep = 0.0
    while running:
        s = dev["ser"]
        if s is None or not s.is_open:
            time.sleep(0.2)
            continue
        if time.time() - last_keep > KEEPALIVE_S and fw["kind"] != "rehab":   # watchdog food (lab firmware only) [g7-A]
            _dev_write("K")
            last_keep = time.time()
        try:
            raw = s.readline()
        except Exception as e:
            link["fault"] = f"rx error: {e}"
            time.sleep(0.2)
            continue
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if not line:
            continue

        if line.startswith("D,"):
            p = line.split(",")
            T["nfields"] = len(p)
            if len(p) >= 13:
                # [g7-A] 13 fields = the ExArMo REHAB firmware v2.9:
                # D,t_ms,state,theta,theta_ref,vel,iq_meas,tau_m,tau_s,tau_d,tau_dob,Bv,iq_cmd,T_est
                if fw["kind"] != "rehab":
                    _set_fw_kind("rehab", link["fw"] or "rehab (13-field telemetry)")
                try:
                    rs = int(p[2]); th = float(p[3]); vj = float(p[5]); iq = float(p[6])
                    iqc = float(p[12]); tst = float(p[13]) if len(p) >= 14 else float("nan")
                except ValueError:
                    continue
                if not all(math.isfinite(v) for v in (th, vj, iq)):
                    clog("dropped a non-finite telemetry sample", "err")
                    continue
                now = time.time()
                fw["rstate"] = rs
                fw["hold_theta"] = th
                fw["T_est"] = tst if math.isfinite(tst) else None
                T["state"] = {0: 0, 7: (3 if fw["emu"] else 1), 6: 2, 1: 4, 2: 4, 3: 4, 4: 5, 5: 6}.get(rs, rs)
                T["ang"] = DIR_SIGN * th * N_CAPSTAN
                T["vel"] = DIR_SIGN * vj * N_CAPSTAN
                T["iq"] = iq
                if not fw["emu"]:
                    T["iqc"] = iqc
                T["idq"] = 0.0; T["vq"] = 0.0
                powered = rs not in (0, 5)
                _ntc = fw.get("T_src") == "ntc" and fw["T_est"] is not None   # [g7-C] v2.9.2 thermistor
                T["flags"] = 0x01 | 0x08 | 0x10 | (0x06 if powered else 0) | (0x20 if _ntc else 0)
                T["temp_ok"] = bool(_ntc)          # MEASURED (PA2 bead) on rehab v2.9.2+, else none
                T["temp"] = fw["T_est"] if _ntc else None
                T["t_rx"] = now
                with hist_lock:
                    iq_hist.append((now, T["iq"]))
                    if not fw["emu"]:
                        iqc_hist.append((now, T["iqc"]))
                    rpm_hist.append((now, T["vel"] * 60.0 / (2 * math.pi)))
                    ang_hist.append((now, T["ang"]))
                    if fw["T_est"] is not None and -50.0 < fw["T_est"] < 400.0:
                        temp_hist.append((now, fw["T_est"]))
                continue
            if fw["kind"] is None and len(p) >= 6:
                _set_fw_kind("lab", link["fw"])
            if len(p) >= 6:
                try:
                    # float("nan") and float("inf") both PARSE, and Arduino
                    # prints exactly those words. One such sample used to reach
                    # the plots, produce NaN canvas coordinates, raise TclError
                    # inside the periodic tick, and silently kill the whole
                    # live-update chain — every readout frozen, no error shown.
                    vals = [float(x) for x in p[3:7]] if len(p) >= 7 else [float(x) for x in p[3:6]] + [0.0]
                    if not all(math.isfinite(v) for v in vals):
                        clog("dropped a non-finite telemetry sample", "err")
                        continue
                    T["state"] = int(p[2])
                    T["ang"], T["vel"], T["iq"], T["iqc"] = vals
                    # v2 extras (absent on the old 7-field firmware)
                    _id = float(p[7]) if len(p) >= 8 else 0.0
                    _vq = float(p[8]) if len(p) >= 9 else 0.0
                    T["idq"] = _id if math.isfinite(_id) else 0.0
                    T["vq"] = _vq if math.isfinite(_vq) else 0.0
                    T["flags"] = int(float(p[9])) if len(p) >= 10 else -1
                    now = time.time()
                    T["t_rx"] = now
                    # [g3-A] field 11 = motor temperature, valid only while
                    # flags bit5 is set. The firmware publishes 0.0 with the
                    # bit CLEAR when the bead is open or shorted, and 0 degC is
                    # a number this program would otherwise plot, average and
                    # fit quite happily. Gate on the bit.
                    with hist_lock:
                        iq_hist.append((now, T["iq"]))
                        iqc_hist.append((now, T["iqc"]))
                        rpm_hist.append((now, T["vel"] * 60.0 / (2 * math.pi)))
                        ang_hist.append((now, T["ang"]))
                    T["temp_ok"] = bool(T["flags"] >= 0 and (T["flags"] & 0x20))
                    T["temp"] = None
                    if len(p) >= 11 and T["temp_ok"]:
                        try:
                            _tc = float(p[10])
                        except ValueError:
                            _tc = float("nan")
                        if math.isfinite(_tc):
                            T["temp"] = _tc
                            with hist_lock:
                                temp_hist.append((now, _tc))
                except ValueError:
                    pass
            continue

        # everything that is not telemetry is a message worth keeping
        link["rx"] = line
        clog(line, "err" if ("FAIL" in line or "WATCHDOG" in line) else "rx")
        if line.startswith("L state="):                         # [g7-A] rehab status dump
            rehab_parse_status(line)
        elif line.startswith("READY v2.9"):
            _set_fw_kind("rehab", "rehab v2.9")
        elif line.startswith("EV,THERM,"):
            link["fault"] = ("REHAB thermal derate: " + line +
                             " — if T_est is absurd, the IDLE integrator bug is active (docs/lab_procedures.md §0)")
        elif line.startswith("EV,RANGE_FAULT") or line.startswith("EV,HOMEFAIL") or line.startswith("EV,HOMETIMEOUT"):
            link["fault"] = "REHAB firmware fault: " + line
        elif line.startswith("EV,HOMED"):
            fw["homed"] = True; link["fault"] = ""
            _dev_write("L")
        elif line.startswith("EV,BOOT,"):
            link["fw"] = line.split(",", 2)[-1]
            link["fault"] = ""
            _set_fw_kind("lab", link["fw"])
        elif "FOCFAIL" in line:
            link["fault"] = "initFOC FAILED — motor cannot move (check magnet / pole pairs / power)"
        elif "SENSEFAIL" in line:
            link["fault"] = "CURRENT SENSE DEAD — commanding current, measuring none"
        elif "WATCHDOG" in line:
            link["fault"] = "WATCHDOG tripped — host went silent, motor released"
        elif "RUNAWAY" in line:
            link["fault"] = ("RUNAWAY trip — the shaft was free and accelerated away. "
                             "Load it against the cell before jogging, or lower Pv.")
        # [g3-A] the comma matters: "EV,TEMP," is the trip, "EV,TEMPFAIL" is a
        # broken sensor and "EV,TEMPDUMP" is just the 'T' report.
        elif line.startswith("EV,TEMP,"):
            link["fault"] = ("OVER-TEMPERATURE — the firmware stopped the motor. "
                             "Let it cool; it refuses torque until it does.")
        elif line.startswith("EV,TEMPFAIL"):
            if fw["kind"] == "rehab":
                fw["T_src"] = "model"                           # [g7-C]
            link["fault"] = ("THERMISTOR FAULT — open or shorted on GPIO3/PA2. "
                             "Send T on the Diagnostics console for the raw volts.")
        elif line.startswith("EV,TEMPOK"):
            if fw["kind"] == "rehab":
                fw["T_src"] = "ntc"                             # [g7-C]
            link["fault"] = ""
        elif line.startswith("EV,SELFTEST,"):
            link["fault"] = ""
        elif line.startswith("EV,ALIGN,"):
            if ",1," not in line:
                link["fault"] = "RE-ALIGN FAILED — is the shaft free to turn?"
            elif "pp_ok=0" in line:
                link["fault"] = "POLE-PAIR CHECK FAILED — torque per amp will be wrong"
            else:
                link["fault"] = "" 


def dev_connect(port):
    dev_disconnect()
    try:
        s = serial.Serial(port, BAUD_DEV, timeout=0.2)
        time.sleep(0.4)
        s.reset_input_buffer()
        with dev["lock"]:
            dev["ser"] = s
        link["fault"] = ""
        fw.update(kind=None, homed=False, emu=False, emu_level=0.0, emu_target=0.0, emu_sent=None, pid=[])   # [g7-A]
        clog(f"--- connected {port} @ {BAUD_DEV} ---", "sys")
        # [g2-E] never trust whatever state the board was left in
        _dev_write("O0")
        _dev_write(f"D{TELEM_MS}")
        _dev_write("L")
        dev_status.config(text=f"● {port}", fg=GREEN)
    except Exception as e:
        dev_status.config(text=f"● {e}", fg=RED)
        clog(f"connect failed: {e}", "err")


def dev_disconnect():
    with dev["lock"]:
        if dev["ser"] is not None:
            try:
                dev["ser"].write(b"O0\n"); dev["ser"].close()
            except Exception:
                pass
            dev["ser"] = None


def dev_alive():
    return T["t_rx"] > 0 and (time.time() - T["t_rx"]) < 1.0


# ── the load cell lives on the ARM BOARD now [g4-E] ───────────
# Everything that used to come off a second Arduino on its own port now
# comes off the one ESP32 that also carries the IMU. The shim that gives
# tabs 3-7 their familiar lc_* view of it is below arm_cal_load(), because
# it is built on the arm buffer and has to be defined after it.

def open_log(path, header, with_cal=True):
    """Open a run log, stamp the calibration, write the header.

    A grams column is only meaningful alongside the calibration that produced
    it. Recording it in the file means a run stays interpretable after the
    cell is re-calibrated — which, being a comment line, pandas reads with
    read_csv(path, comment='#')."""
    f = open(path, "w", newline="")
    if with_cal:
        f.write(lc_cal_line() + "\n")
    wr = csv.writer(f)
    wr.writerow(header)
    return f, wr


# ── arm sensor board [g4-A] ────────────────────────────────────
# A THIRD serial device, and deliberately not a second load cell.
#
# The ESP32 sketch (WIT-Motion IMU on UART1 + HX711) prints one human-readable
# line per 250 ms at 115200 baud:
#     Roll_deg: 1.23 | Pitch_deg: -4.56 | Yaw_deg: 0.00 | LoadCell_g: 512.34
# with LoadCell_raw instead of LoadCell_g until that board has been tared and
# calibrated, and the literal "nan" for any field it will not vouch for
# (stale IMU, saturated HX711, a tare/calibration job in progress).
#
# Two rules follow from that and are enforced below:
#   1. Grams on this link are calibrated ON THE BOARD. They are NOT passed
#      through lc_cal — that would apply a second, unrelated scale, and the
#      number would be wrong in a way no column in the log could reveal.
#   2. "nan" is not zero. Each field is carried as None when absent, so a
#      dropped IMU frame can never be averaged in as 0.0 degrees.
arm = dict(ser=None, lock=threading.Lock(), last="", msg="")
arm_lock = threading.Lock()
arm_buf = collections.deque(maxlen=20000)   # (t, roll, pitch, yaw, grams, raw)

# [g4-B] CALIBRATION LIVES HERE, NOT IN THE SKETCH.
#
# The board can tare and scale itself ('t' and 'c'), but that calibration is
# in its RAM: it dies on every reset, and making it survive means editing
# KNOWN_MASS_G and re-flashing. A bench that has to re-flash to weigh
# something will end up weighing things uncalibrated. So the GUI keeps its
# own calibration in labdata/arm_cal.json, computes grams from the RAW
# counts, and the board's own grams are used only when it sends them.
#
#   grams = (raw - tare) * scale        scale in grams per count
#
# The offset is subtracted BEFORE the multiply because that is where it
# physically is — preload, the beam's own weight, the amplifier's zero — and
# because a multi-point fit grams = a*raw + b is then the same line, with
# scale = a and tare = -b/a. A single multiplier cannot express a resting
# load at all, which is the mistake the load-cell code had to be fixed for
# ([g3-B] above).
#
# The IMU half is an offset and a gain per axis:
#
#   angle = wrap(reading - zero) * gain
#
# `zero` is the pose you declare to be zero. `gain` is only ever meaningfully
# different from 1 when the IMU's axis is not parallel to the joint's — and
# it is a first-order patch for that, not a fix: a real misalignment is a
# cosine, so the gain measured over one span does not hold over another.
# arm_cal_note() says so out loud whenever a gain is in force.
arm_zero = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}   # software angle reference
arm_gain = {"Roll": 1.0, "Pitch": 1.0, "Yaw": 1.0}
arm_cal = {"scale": 0.0,        # grams per count; 0 = no load-cell cal yet
           "tare": 0.0,         # RAW counts at zero load
           "source": "auto",    # auto | gui | board — which grams to trust
           "mode": "manual",    # manual | multi — how the scale was found
           "points": [],        # [(grams, raw)] for the multi-point fit
           "r2": None, "when": "", "span": "",
           "zero": arm_zero, "gain": arm_gain, "gain_when": ""}
ARM_CAL_FILE = os.path.join("labdata", "arm_cal.json")

_arm_fld = re.compile(
    r"(Roll_deg|Pitch_deg|Yaw_deg|LoadCell_g|LoadCell_raw)\s*:\s*"
    r"([-+]?(?:nan|NaN|NAN|inf|\d+(?:\.\d+)?(?:[eE][-+]?\d+)?))")


def _arm_num(v):
    """A field is a number only if it is finite. nan/inf become None."""
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def snap_arm():
    with arm_lock:
        return list(arm_buf)


def arm_wrap(a):
    """Fold an angle difference into (-180, +180].

    Yaw crosses the +/-180 seam, and a zero reference captured near it would
    otherwise make the joint angle jump a full turn in the middle of a log."""
    return None if a is None else (a + 180.0) % 360.0 - 180.0


def arm_reader():
    while running:
        s = arm["ser"]
        if s is None or not s.is_open:
            time.sleep(0.2)
            continue
        try:
            raw = s.readline()
        except Exception:
            time.sleep(0.2)
            continue
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if not line:
            continue
        d = dict(_arm_fld.findall(line))
        # Anything without all three angles is a MESSAGE from the sketch
        # ("Taring: keep unloaded and still...", "Calibration complete..."),
        # and it goes on screen rather than into the data. The load-cell
        # parser learned this the hard way — see [g3-C] above.
        if not ("Roll_deg" in d and "Pitch_deg" in d and "Yaw_deg" in d):
            arm["msg"] = line[:140]
            continue
        with arm_lock:
            arm_buf.append((time.time(),
                            _arm_num(d.get("Roll_deg")),
                            _arm_num(d.get("Pitch_deg")),
                            _arm_num(d.get("Yaw_deg")),
                            _arm_num(d.get("LoadCell_g")),
                            _arm_num(d.get("LoadCell_raw"))))
        arm["last"] = line


def arm_send(ch):
    """The sketch takes single-character commands: 't' tare, 'c' calibrate."""
    with arm["lock"]:
        s = arm["ser"]
        if s is None:
            return False
        try:
            s.write(ch.encode())
            return True
        except Exception:
            return False


def arm_connect(port):
    arm_disconnect()
    try:
        s = serial.Serial(port, BAUD_ARM, timeout=0.3)
        time.sleep(0.3)
        with arm["lock"]:
            arm["ser"] = s
        arm["msg"] = ""
        arm_status.config(text=f"● {port}", fg=GREEN)
    except Exception as e:
        arm_status.config(text=f"● {e}", fg=RED)


def arm_disconnect():
    with arm["lock"]:
        if arm["ser"] is not None:
            try:
                arm["ser"].close()
            except Exception:
                pass
            arm["ser"] = None


def arm_alive(seconds=2.0):
    b = snap_arm()
    return bool(b) and (time.time() - b[-1][0]) <= seconds


def arm_window(t0, t1):
    return [s for s in snap_arm() if t0 <= s[0] <= t1]


def arm_latest(seconds=1.0):
    """→ (roll, pitch, yaw, grams), each a trimmed mean over the window or
    None if that field never arrived valid inside it. `grams` is whatever the
    calibration in force says, so it is present on a raw-only stream too."""
    now = time.time()
    win = [s for s in snap_arm() if now - s[0] <= seconds]
    out = []
    for i in (1, 2, 3):
        vals = [s[i] for s in win if s[i] is not None]
        out.append(robust_mean(vals)[0] if vals else None)
    gs = [g for g in (arm_grams(s) for s in win) if g is not None]
    out.append(robust_mean(gs)[0] if gs else None)
    return tuple(out)


# ── arm calibration [g4-B] ─────────────────────────────────────
def arm_cal_load():
    try:
        with open(ARM_CAL_FILE) as f:
            d = json.load(f)
        for k in ("scale", "tare", "r2"):
            if d.get(k) is not None:
                arm_cal[k] = float(d[k])
        # A file written before the two ideas were separated stored the
        # source under "mode". Move it rather than silently reading it as a
        # calibration method.
        m = d.get("mode", "manual")
        arm_cal["source"] = d.get("source", m if m in ("auto", "gui", "board") else "auto")
        arm_cal["mode"] = m if m in ("manual", "multi") else d.get("cal_mode", "manual")
        arm_cal["points"] = [(float(a), float(b)) for a, b in d.get("points", [])]
        for k in ("when", "span", "gain_when"):
            arm_cal[k] = d.get(k, "")
        for k in ("Roll", "Pitch", "Yaw"):
            arm_zero[k] = float(d.get("zero", {}).get(k, 0.0))
            arm_gain[k] = float(d.get("gain", {}).get(k, 1.0)) or 1.0
        return True
    except Exception:
        return False


def arm_cal_save():
    try:
        os.makedirs("labdata", exist_ok=True)
        with open(ARM_CAL_FILE, "w") as f:
            json.dump({"scale": arm_cal["scale"], "tare": arm_cal["tare"],
                       "source": arm_cal["source"], "mode": arm_cal["mode"],
                       "points": arm_cal["points"],
                       "r2": arm_cal["r2"], "when": arm_cal["when"],
                       "span": arm_cal["span"], "gain_when": arm_cal["gain_when"],
                       "zero": arm_zero, "gain": arm_gain}, f, indent=2)
        return True
    except Exception:
        return False


def arm_cal_ready():
    return arm_cal["scale"] != 0.0


def arm_counts_per_gram():
    return None if arm_cal["scale"] == 0 else 1.0 / arm_cal["scale"]


def arm_cal_line():
    """One comment line stamped at the top of every arm log."""
    cpg = arm_counts_per_gram()
    r2 = "" if arm_cal["r2"] is None else f"{arm_cal['r2']:.5f}"
    return ("# loadcell (arm board): source=" + arm_cal["source"] +
            " mode=" + arm_cal["mode"] +
            f" scale_g_per_count={arm_cal['scale']:.8g} tare_raw={arm_cal['tare']:.8g}"
            f" counts_per_gram={'--' if cpg is None else f'{cpg:.6g}'}"
            f" r2={r2} calibrated={arm_cal['when'] or 'never'}"
            f" | imu zero={ {k: round(v, 3) for k, v in arm_zero.items()} }"
            f" gain={ {k: round(v, 5) for k, v in arm_gain.items()} }")


def arm_cal_note():
    """The one honest sentence about the state of the calibration."""
    if arm_cal["source"] == "board":
        return "grams come from the board's own calibration (lost on its reset)"
    if not arm_cal_ready():
        return "load cell NOT calibrated — raw counts only"
    cpg = arm_counts_per_gram()
    s = f"{cpg:.1f} counts/g, zero at {arm_cal['tare']:.0f}"
    if arm_cal["r2"] is not None:
        s += f", R²={arm_cal['r2']:.4f}"
    if arm_cal["when"]:
        s += f", set {arm_cal['when']}"
    off = [k for k in arm_gain if abs(arm_gain[k] - 1.0) > 1e-9]
    if off:
        s += ("  ·  IMU gain " +
              ", ".join(f"{k} ×{arm_gain[k]:.4f}" for k in off) +
              " — a first-order patch for axis misalignment, valid near the "
              "span it was measured over")
    return s


def arm_cell_fault():
    """→ a sentence naming a load-cell fault, or "" when the cell looks sane.

    [g4-F] A floating DT line reads LOW for ever, so is_ready() is true on
    every pass and the sketch takes a "conversion" as fast as it can loop:
    hundreds a second, alternating zero and full scale. Both halves of that
    are recognisable, and neither is a force. Saying so beats drawing it as
    data and letting someone calibrate against it — which is exactly what a
    zero captured from this stream would do, silently."""
    now = time.time()
    win = [(t, v) for (t, v) in snap_arm_raw() if now - t <= 3.0]
    if len(win) < 8:
        return ""
    rate = len(win) / 3.0
    extreme = sum(1 for _t, v in win if abs(v) >= 8.0e6)
    if rate > 120:
        msg = (f"{rate:.0f} samples/s is faster than any HX711 can convert "
               f"(80 SPS at most) — the DT line is stuck low: loose wire, "
               f"wrong pin, or the module has no power. The sketch is reading "
               f"garbage as fast as it can loop.")
        # And that flood has a second victim. Each line is ~95 characters, so
        # 171 of them a second needs about 162 kbit/s on a 115200 link: the
        # writes block, the sketch stops pumping the IMU's UART, and its frames
        # are lost. One cause, two dead readings — worth saying, because the
        # angles stopping looks like a separate IMU fault and is not.
        fresh_ang = [x for x in snap_arm()
                     if now - x[0] <= 3.0 and x[1] is not None]
        if not fresh_ang:
            msg += (f"  The angles have stopped with it: {rate:.0f} lines/s is "
                    f"more than the 115200 link can carry, so the prints block "
                    f"and the IMU's frames are dropped. Both come back when the "
                    f"cell is wired.")
        return msg
    if extreme >= max(2, len(win) // 10):
        return ("readings are pinned at the converter's limits (±8,388,607) — "
                "the bridge is open or miswired: check E+/E− and A+/A− at the "
                "cell, DT on GPIO4, SCK on GPIO5, and 3V3/GND at the module.")
    return ""


def arm_grams(s):
    """One sample → grams, by whichever calibration is in force.

    None means "this sample has no trustworthy weight", and every caller must
    keep it as None. Filling it with 0.0 would put an unweighed sample into a
    log column that reads as a real measurement."""
    g_board, raw = s[4], s[5]
    mode = arm_cal["source"]
    if mode == "board":
        return g_board
    gui = (None if (raw is None or arm_cal["scale"] == 0.0)
           else (raw - arm_cal["tare"]) * arm_cal["scale"])
    if mode == "gui":
        return gui
    return g_board if g_board is not None else gui      # auto


def arm_angle_cal(axis, v):
    """Raw axis reading → calibrated joint angle (still unsigned by the tab)."""
    if v is None:
        return None
    return arm_wrap(v - arm_zero[axis]) * arm_gain[axis]


def snap_arm_raw():
    return [(s[0], s[5]) for s in snap_arm() if s[5] is not None]


def arm_raw_settled(seconds=2.0):
    """→ (mean, sd, drift, n) of the RAW counts over the window.

    `drift` is the second half minus the first. A calibration point captured
    while the cell is still ringing down from the mass being placed is simply
    a wrong point, and it is wrong in the way that hurts most: it lands
    somewhere plausible and bends the fitted line. Averaging harder cannot
    help, because the error is a trend and not noise — the only defence is to
    notice the trend, which is what this returns."""
    now = time.time()
    win = [(t, v) for (t, v) in snap_arm_raw() if now - t <= seconds]
    if len(win) < 4:
        return (None, None, None, len(win))
    first = win[:len(win) // 2]
    second = win[len(win) // 2:]
    m, sd = robust_mean([v for _, v in win])
    a, _ = robust_mean([v for _, v in first])
    b, _ = robust_mean([v for _, v in second])
    return (m, sd, (b - a) if (a is not None and b is not None) else None, len(win))


arm_cal_load()


# ── one board [g4-E] ──────────────────────────────────────────
# Tabs 3-7 were written against a second Arduino on its own serial port, with
# its own load cell and its own calibration file. There is one board now: the
# ESP32 that carries the IMU and the cell together. Rather than rewrite five
# tabs and their experiment engines, the old lc_* names are kept and re-pointed
# at the arm buffer — same signatures, same units, one sensor underneath.
#
# The old labdata/loadcell_cal.json is NOT read. It calibrated a DIFFERENT cell
# on different hardware, and carrying a scale across a change of instrument is
# how a gram quietly comes to mean two things. Calibrate once, on tab 9.
lc = arm                    # so `lc["ser"] is None` still asks the right link
lc_cal = arm_cal            # one calibration, one owner
lc_cal_save = arm_cal_save
lc_cal_load = arm_cal_load
lc_cal_line = arm_cal_line


def snap_lc():
    """[(t, grams, tempC)] — the shape tabs 3-7 expect. The temperature slot is
    always None: the arm board has no thermistor, and inventing one would put a
    number in a column that nothing measured."""
    return [(s[0], arm_grams(s), None) for s in snap_arm()]


def lc_window(t0, t1):
    return [r for r in snap_lc() if t0 <= r[0] <= t1 and r[1] is not None]


def lc_latest(seconds=0.4):
    now = time.time()
    vals = [g for (t, g, _) in snap_lc() if g is not None and now - t <= seconds]
    return robust_mean(vals)[0] if vals else None


def lc_raw_latest(seconds=1.0):
    """[g3-B] Trimmed mean of the RAW stream. Calibration must never be
    computed from the calibrated value — that folds the old scale into the new
    one, so a second calibration silently squares the error."""
    now = time.time()
    vals = [v for (t, v) in snap_arm_raw() if now - t <= seconds]
    return robust_mean(vals)[0] if vals else None


def lc_raw_settled(seconds=2.0):
    """→ (mean, drift, n), as the Diagnostics tab has always called it."""
    m, _sd, drift, n = arm_raw_settled(seconds)
    return (m, drift, n)


def lc_drift_note(drift):
    """A drift worth mentioning, expressed in grams at the current scale."""
    if drift is None or not arm_cal["scale"]:
        return ""
    g = abs(drift * arm_cal["scale"])
    if g < 0.5:
        return ""
    return (f"  ⚠ the reading is still moving ({g:.1f} g across the sample "
            f"window) — let it settle and take it again.")


# ── temperature source [g3-A] ──────────────────────────────────
# Two possible sources, and which one is live changes what a temperature
# MEANS. The driver board's thermistor sits on the motor and shares the
# firmware's clock with Iq; the load-cell Arduino's sensor is wherever it was
# taped and runs on its own timebase, so its samples cannot be aligned with a
# current step to better than the two devices' drift. Prefer the driver, fall
# back to the Arduino, and always be able to say which — every place that shows
# or logs a temperature also shows the source.
def temp_src():
    """'driver' | None.

    [g4-E] There used to be a second source — a bead taped somewhere, on the
    load-cell Arduino's own clock. That board is gone, and the arm board that
    replaced it has no thermistor, so temperature now comes from the driver
    board or not at all. A branch that can never be taken is worse than no
    branch: it reads like a fallback that will save you."""
    if T["temp_ok"] and T["temp"] is not None and dev_alive():
        return "driver"
    if fw["kind"] == "rehab" and fw["T_est"] is not None and dev_alive() and -50.0 < fw["T_est"] < 400.0:
        return "model"                                          # [g7-A] firmware estimate, not a sensor
    return None


def temp_window(t0, t1, src=None):
    """[(t, degC)] from whichever source is live, oldest first."""
    src = src or temp_src()
    if src in ("driver", "model"):
        return [(t, c) for (t, c) in snap_hist(temp_hist) if t0 <= t <= t1]
    return []


def temp_now(seconds=2.0, src=None):
    now = time.time()
    vals = [c for (_, c) in temp_window(now - seconds, now, src)]
    return robust_mean(vals)[0] if vals else None


def temp_label():
    s = temp_src()
    return {"driver": "driver board (GPIO3)",
            "model": "firmware MODEL T_est (rehab v2.9) — an estimate, NOT a measurement"}.get(
        s, "no temperature sensor (fit the bead on the driver board)")


def robust_mean(vals, trim=0.2):
    """trimmed mean + std: sort, drop `trim` fraction each side, average."""
    if not vals:
        return None, None
    v = sorted(vals)
    k = int(len(v) * trim)
    core = v[k:len(v) - k] if len(v) - 2 * k >= 1 else v
    m = sum(core) / len(core)
    sd = (sum((x - m) ** 2 for x in core) / len(core)) ** 0.5
    return m, sd


def _mean_rpm(pts):
    """Mean speed over a span, from the ANGLE endpoints."""
    if len(pts) < 4:
        return None
    (t0, a0), (t1, a1) = pts[0], pts[-1]
    dt = t1 - t0
    return None if dt < 1e-3 else (a1 - a0) / dt * 60.0 / (2 * math.pi)


def kv_speed(window=1.0, nsub=4):
    """(mean rpm, spread rpm) over `window`, measured from shaft ANGLE.

    Two separate improvements over averaging the velocity field:

    1. ACCURACY. `vel` is a differentiated, low-pass-filtered estimate, so
       averaging it inherits both the encoder's per-sample quantisation
       (0.38 rad/s at 1 kHz) and the filter's lag. Shaft angle is absolute and
       accumulates full rotations, so (a1-a0)/dt over a whole second is a mean
       speed with essentially no noise — the endpoints are all that matter.

    2. AN HONEST STEADY TEST. The old indicator compared the sample standard
       deviation against 1% of target. Sample scatter does not shrink when the
       speed is genuinely steady, so that test could stay red forever, or go
       green on a drifting average. What matters for a back-EMF reading is
       whether the MEAN is holding still, so this splits the window into
       sub-windows and reports the spread of their means. Real drift shows up;
       per-sample noise does not."""
    now = time.time()
    pts = [(t, a) for (t, a) in snap_hist(ang_hist) if now - t <= window]
    if len(pts) < 8:
        return (None, None)
    mean = _mean_rpm(pts)
    if mean is None:
        return (None, None)
    sub = []
    for k in range(nsub):
        lo = now - window + k * window / nsub
        hi = lo + window / nsub
        seg = [p for p in pts if lo <= p[0] <= hi]
        v = _mean_rpm(seg)
        if v is not None:
            sub.append(v)
    spread = (max(sub) - min(sub)) if len(sub) >= 2 else None
    return (mean, spread)


def sense_alive():
    """measured-current path considered alive if it shows real signal while
    the firmware is commanding current"""
    now = time.time()
    meas = [abs(i) for (t, i) in snap_hist(iq_hist) if now - t <= 2.0]
    cmd = [abs(i) for (t, i) in snap_hist(iqc_hist) if now - t <= 2.0]
    if not meas or not cmd:
        return True                      # no data yet: don't cry wolf
    if max(cmd) < 0.05:
        return True                      # nothing commanded: can't judge
    return max(meas) > 0.25 * max(cmd)


def iqc_avg(seconds=1.0):
    now = time.time()
    return robust_mean([i for (t, i) in snap_hist(iqc_hist) if now - t <= seconds])


def iq_avg(seconds=1.0):
    now = time.time()
    return robust_mean([i for (t, i) in snap_hist(iq_hist) if now - t <= seconds])


# ── logging ────────────────────────────────────────────────────
def serial_no():
    return re.sub(r"[^A-Za-z0-9_\-]", "-", e_serial.get().strip())


def log_path(experiment):
    sn = serial_no()
    if not sn:
        return None
    os.makedirs("labdata", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"labdata/{sn}_{experiment}_{ts}.csv"


# ── first-order exponential fit [g3-A] ─────────────────────────
# T(t) = A + B·exp(−(t−t0)/tau)   — A is the asymptote, B is negative while
# heating and positive while cooling. With tau FIXED the remaining two
# parameters are linear, so the whole fit is a one-dimensional search over tau
# with an exact least-squares solve inside it: no gradients, no initial-guess
# sensitivity, no iteration that can fail to converge in front of the operator.
def _lsq_exp(ts, cs, tau):
    t0 = ts[0]
    e = [math.exp(-(t - t0) / tau) for t in ts]
    n = len(ts); s1 = sum(e); s2 = sum(x * x for x in e)
    sy = sum(cs); sye = sum(c * x for c, x in zip(cs, e))
    det = n * s2 - s1 * s1
    if abs(det) < 1e-9:
        return None
    A = (s2 * sy - s1 * sye) / det
    B = (n * sye - s1 * sy) / det
    return A, B, sum((c - (A + B * x)) ** 2 for c, x in zip(cs, e))


def _seed_tau(ts, cs):
    """A rough tau from dT/dt = (T_inf − T)/tau, which is a straight line in
    (T, dT/dt). Only used to centre the search below — it is unbiased but very
    noisy, because differentiating a 0.1 degC-resolution sensor is."""
    k = 2
    sm = [sum(cs[max(0, i - k):i + k + 1]) / len(cs[max(0, i - k):i + k + 1])
          for i in range(len(cs))]
    xs, ys = [], []
    for i in range(1, len(sm) - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt > 1e-6:
            xs.append(sm[i]); ys.append((sm[i + 1] - sm[i - 1]) / dt)
    f = linfit(xs, ys) if len(xs) >= 6 else None
    if not f or f[0] >= -1e-12:
        return None
    return -1.0 / f[0]


def expfit(ts, cs):
    """→ (tau_s, asymptote_C, B, r2) or None. r2 is against the TEMPERATURE
    data, not against the derivative, so it means what a reader expects."""
    if len(ts) < 8:
        return None
    span = ts[-1] - ts[0]
    if span <= 0:
        return None
    seed = _seed_tau(ts, cs) or span / 2.0
    seed = min(max(seed, span / 100.0), span * 20.0)
    lo, hi = seed / 10.0, seed * 10.0
    best = None
    for _ in range(4):                       # log-grid, then zoom in 4 times
        for i in range(41):
            tau = lo * (hi / lo) ** (i / 40.0)
            r = _lsq_exp(ts, cs, tau)
            if r and (best is None or r[2] < best[3]):
                best = (tau, r[0], r[1], r[2])
        if best is None:
            return None
        lo, hi = best[0] / 2.5, best[0] * 2.5
    tau, A, B, sse = best
    my = sum(cs) / len(cs)
    sst = sum((c - my) ** 2 for c in cs)
    return tau, A, B, (1.0 - sse / sst if sst > 1e-12 else 1.0)


def linfit(xs, ys):
    n = len(xs)
    mx = sum(xs) / n; my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-12:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    return a, b, r2


# ── UI helpers ─────────────────────────────────────────────────
root = tk.Tk()
root.title("ExArMo Lab — Motor Tester  (g7-C · lab firmware v2.7.1-R or rehab firmware v2.9.x)")
# Fit the screen we actually have instead of assuming a 1660x980 desktop.
_sw, _sh = root.winfo_screenwidth(), root.winfo_screenheight()
root.geometry(f"{min(1660, _sw - 80)}x{min(980, _sh - 120)}+20+20")
root.minsize(1000, 560)
root.configure(bg=BG)


def card(parent):
    o = tk.Frame(parent, bg=BORDER, padx=1, pady=1)
    i = tk.Frame(o, bg=SURFACE, padx=22, pady=18)
    i.pack(fill="both", expand=True)
    return o, i


def safe_tick(fn, period):
    """Run a periodic callback so that an exception cannot end the chain.

    Every live readout in this program depends on its `root.after` loop being
    re-armed. A single raise inside one tick used to unschedule it for good,
    which presents as "the numbers stopped updating" with nothing in the log
    and no traceback anywhere the operator can see. Now the error goes to the
    console and the loop carries on."""
    def wrapper():
        try:
            fn()
        except Exception as exc:
            clog(f"{getattr(fn, '__name__', 'tick')} error: {exc!r}", "err")
        finally:
            root.after(period, wrapper)
    return wrapper


def scroll_column(parent):
    """A vertically scrollable host for a tab's left-hand control column.

    The control columns have grown well past a laptop screen, and a button you
    cannot reach is a button that does not exist. Returns (holder, inner):
    pack the holder, put the card in the inner frame. The canvas viewport is
    kept exactly as wide as its content so nothing at the right edge gets
    clipped — only vertical scrolling is ever needed."""
    holder = tk.Frame(parent, bg=BG)
    cv = tk.Canvas(holder, bg=BG, highlightthickness=0, bd=0)
    sb = ttk.Scrollbar(holder, orient="vertical", command=cv.yview)
    cv.configure(yscrollcommand=sb.set)
    cv.pack(side="left", fill="both", expand=True)
    sb.pack(side="right", fill="y")
    inner = tk.Frame(cv, bg=BG)
    cv.create_window((0, 0), window=inner, anchor="nw")

    def _cfg(e):
        cv.configure(scrollregion=cv.bbox("all"), width=e.width)
    inner.bind("<Configure>", _cfg)

    def _wheel(e):
        # Windows sends delta in steps of 120; macOS sends small values (often
        # +/-1); X11 sends Button-4/5 instead of a delta at all. Normalise to
        # one notch so the wheel feels the same everywhere.
        delta = getattr(e, "delta", 0)
        if delta:
            cv.yview_scroll(-1 if delta > 0 else 1, "units")
        elif getattr(e, "num", 0) == 4:
            cv.yview_scroll(-1, "units")
        elif getattr(e, "num", 0) == 5:
            cv.yview_scroll(1, "units")

    def _bind(_e):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            cv.bind_all(seq, _wheel)

    def _unbind(_e):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            cv.unbind_all(seq)

    cv.bind("<Enter>", _bind)
    cv.bind("<Leave>", _unbind)
    return holder, inner


def section(parent, text):
    tk.Label(parent, text=text.upper(), bg=SURFACE, fg=FAINT,
             font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 8))


def btn(parent, text, bg, fg, cmd, width=None):
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                  activebackground=bg, activeforeground=fg, relief="flat",
                  bd=0, font=F_BTN, padx=18, pady=10, cursor="hand2",
                  highlightthickness=0)
    if width:
        b.config(width=width)
    return b


def entry_row(parent, label, default, width=8):
    f = tk.Frame(parent, bg=SURFACE)
    f.pack(fill="x", pady=3)
    tk.Label(f, text=label, bg=SURFACE, fg=TEXT, font=F_LBL).pack(side="left")
    e = tk.Entry(f, bg=FIELD, fg=BLUE, insertbackground=TEXT, relief="flat",
                 font=("Consolas", 14, "bold"), width=width, justify="center",
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=BLUE)
    e.insert(0, str(default))
    e.pack(side="right", ipady=3)
    return e


def mkstatus(parent):
    l = tk.Label(parent, text="●  Ready", bg=SURFACE, fg=MUTED, font=F_LBL,
                 wraplength=330, justify="left")
    l.pack(anchor="w", pady=(8, 0))

    def set_(t, c):
        l.config(text="●  " + t, fg=c)
    set_.label = l          # so a caller (or a test) can read back what it says
    return set_


def big_value(parent, caption, color=BLUE):
    v = tk.Label(parent, text="—", bg=SURFACE, fg=color, font=F_BIG)
    v.pack()
    tk.Label(parent, text=caption, bg=SURFACE, fg=FAINT, font=F_SM).pack()
    return v


STYLE = ttk.Style(); STYLE.theme_use("clam")
STYLE.configure("Treeview", background=SURFACE, fieldbackground=SURFACE,
                foreground=TEXT, rowheight=28, font=("Consolas", 12),
                borderwidth=0)
STYLE.configure("Treeview.Heading", background=FIELD, foreground=MUTED,
                font=("Segoe UI", 10, "bold"), borderwidth=0)


def clear_table(tbl):
    for it in tbl.get_children():
        tbl.delete(it)


def drop_last_row(tbl):
    kids = tbl.get_children()
    if kids:
        tbl.delete(kids[-1])


def confirm_reset(n, what="rows"):
    """Ask before discarding collected data.

    A reset that fires by accident costs a whole run — on this bench that can
    be twenty minutes of adding weights by hand — so it asks, but only when
    there is actually something to lose."""
    if n <= 0:
        return True
    return messagebox.askyesno("Reset data",
                               f"Discard {n} {what} and start recording again?")


def mktable(parent, cols, height=8):
    t = ttk.Treeview(parent, columns=[c[0] for c in cols], show="headings",
                     height=height)
    for cid, txt, w in cols:
        t.heading(cid, text=txt)
        t.column(cid, width=w, anchor="center")
    t.pack(pady=(8, 6))
    return t


def draw_ts(cv, series, color, ylabel, marks=(), y2=None, color2=PURPLE,
            unit1="", unit2="", label1="", label2="", signed=False):
    """Time-series plot, last 60 s. series / y2 = list[(t, v)].

    Both series are drawn as MAGNITUDE. Running the motor CCW makes Iq
    negative, and plotting it signed sent the current trace DOWNWARD at every
    step while the force went up — two curves that mirror each other when the
    physics says they should track. Sign is a direction convention and belongs
    in the readouts, not in a trend chart whose job is to show that torque and
    force rise together. Every fit downstream already uses abs().

    Each series is auto-scaled independently on its own axis, so the min/max
    labels below are what make the chart readable at all — without them two
    normalised curves share a frame and neither has a value.

    [g4-A] `signed=True` switches the magnitude rule off, for quantities whose
    sign is the measurement rather than a wiring convention — a joint angle
    either side of its zero, a force that can pull as well as push.
    """
    cv.delete("all")
    w = int(cv.winfo_width() or 600); h = int(cv.winfo_height() or 240)
    cv.create_rectangle(1, 1, w - 1, h - 1, outline=BORDER, fill="#fbfcfe")
    now = time.time(); t0 = now - 60.0
    top, bot = 30, h - 18                       # leave room for the header row

    def plot(data, col):
        pts = [(t, v if signed else abs(v)) for (t, v) in data
               if t >= t0 and v is not None and math.isfinite(v)]
        if len(pts) < 2:
            return None
        vs = [v for _, v in pts]
        lo, hi = min(vs), max(vs)
        if hi - lo < 1e-9:                      # flat line: keep it centred
            lo -= 0.5; hi += 0.5
        pad = 0.1 * (hi - lo)
        lo -= pad; hi += pad
        poly = []
        for t, v in pts:
            x = (t - t0) / 60.0 * (w - 20) + 10
            y = bot - (v - lo) / (hi - lo) * (bot - top)
            poly += [x, y]
        cv.create_line(*poly, fill=col, width=2)
        return min(vs), max(vs)

    for tm in marks:                            # step boundaries, behind traces
        if tm >= t0:
            x = (tm - t0) / 60.0 * (w - 20) + 10
            cv.create_line(x, top - 4, x, bot, fill=BORDER, dash=(3, 4))

    r1 = plot(series, color)
    r2 = plot(y2, color2) if y2 else None

    cv.create_text(12, 8, text=ylabel, fill=MUTED, font=F_SM, anchor="nw")
    # numeric range per series, in that series' own colour
    if r1:
        cv.create_text(12, h - 14,
                       text=f"{label1 or 'left'}  {r1[0]:.1f} – {r1[1]:.1f} {unit1}",
                       fill=color, font=F_SM, anchor="nw")
    if r2:
        cv.create_text(w - 12, h - 14,
                       text=f"{label2 or 'right'}  {r2[0]:.2f} – {r2[1]:.2f} {unit2}",
                       fill=color2, font=F_SM, anchor="ne")


def draw_scatter(cv, xs, ys, fit=None, xlabel="", ylabel="", color=BLUE):
    """Scatter of the raw points with the fitted line through them.

    Every tab reports a slope and an R², but a number cannot show you WHICH
    point is wrong. On these rigs the failures are all shape failures — a
    dead-zone point sitting on the floor, a step that saturated, one outlier
    dragging the slope — and they are obvious in a picture and invisible in a
    correlation coefficient. The axes are anchored at the origin so the
    intercept (friction, preload, offset) is always in view: that term is
    physics here, not a nuisance."""
    cv.delete("all")
    w = int(cv.winfo_width() or 320); h = int(cv.winfo_height() or 200)
    L, R, T, B = 50, 12, 16, 30
    cv.create_rectangle(1, 1, w - 1, h - 1, outline=BORDER, fill="#fbfcfe")
    pts = [(x, y) for x, y in zip(xs, ys)
           if x is not None and y is not None
           and math.isfinite(x) and math.isfinite(y)]
    if not pts:
        cv.create_text(w / 2, h / 2, text="no points yet", fill=FAINT, font=F_SM)
        return
    xmin = min(p[0] for p in pts); xmax = max(p[0] for p in pts)
    ymin = min(p[1] for p in pts); ymax = max(p[1] for p in pts)
    if fit:
        a, b, _ = fit
        for xx in (xmin, xmax):
            ymin = min(ymin, a * xx + b); ymax = max(ymax, a * xx + b)
    xmin = min(xmin, 0.0); ymin = min(ymin, 0.0)      # keep the intercept visible
    if xmax - xmin < 1e-12: xmax = xmin + 1.0
    if ymax - ymin < 1e-12: ymax = ymin + 1.0
    px = 0.08 * (xmax - xmin); py = 0.12 * (ymax - ymin)
    xmax += px; ymax += py

    def X(v): return L + (v - xmin) / (xmax - xmin) * (w - L - R)
    def Y(v): return h - B - (v - ymin) / (ymax - ymin) * (h - B - T)

    cv.create_line(L, h - B, w - R, h - B, fill=BORDER)
    cv.create_line(L, T, L, h - B, fill=BORDER)
    if fit:
        a, b, _ = fit
        cv.create_line(X(xmin), Y(a * xmin + b), X(xmax), Y(a * xmax + b),
                       fill=GREEN, width=2)
    for x, y in pts:
        cv.create_oval(X(x) - 3.5, Y(y) - 3.5, X(x) + 3.5, Y(y) + 3.5,
                       fill=color, outline="")
    cv.create_text(L - 5, Y(ymax), text=f"{ymax:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(L - 5, Y(ymin), text=f"{ymin:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(X(xmin), h - B + 5, text=f"{xmin:.3g}", fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text(w - R, h - B + 5, text=f"{xmax:.3g}", fill=MUTED, font=F_SM, anchor="ne")
    cv.create_text(L + 5, T - 4, text=ylabel, fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text((L + w - R) / 2, h - 9, text=xlabel, fill=FAINT, font=F_SM)


def draw_bars(cv, labels, values, ylabel="", warn_at=None):
    """Bar chart with the mean drawn across it — for comparing what should be
    three identical quantities. Phase-to-phase resistance is exactly that: any
    real difference between the bars is a winding fault, and a picture makes an
    outlier obvious at a glance."""
    cv.delete("all")
    w = int(cv.winfo_width() or 320); h = int(cv.winfo_height() or 200)
    L, R, T, B = 56, 12, 18, 30
    cv.create_rectangle(1, 1, w - 1, h - 1, outline=BORDER, fill="#fbfcfe")
    vals = [v for v in values if v is not None and math.isfinite(v)]
    if not vals:
        cv.create_text(w / 2, h / 2, text="no readings yet", fill=FAINT, font=F_SM)
        return
    vmax = max(vals) * 1.25
    vmin = 0.0
    mean = sum(vals) / len(vals)
    n = len(labels)
    slot = (w - L - R) / max(n, 1)

    def Y(v): return h - B - (v - vmin) / (vmax - vmin) * (h - B - T)

    cv.create_line(L, h - B, w - R, h - B, fill=BORDER)
    spread = (max(vals) - min(vals)) / mean * 100 if mean else 0
    bad = warn_at is not None and spread > warn_at
    for i, (lab, v) in enumerate(zip(labels, values)):
        if v is None or not math.isfinite(v):
            continue
        x0 = L + i * slot + slot * 0.22
        x1 = L + i * slot + slot * 0.78
        cv.create_rectangle(x0, Y(v), x1, h - B,
                            fill=(RED if bad else BLUE), outline="")
        cv.create_text((x0 + x1) / 2, Y(v) - 8, text=f"{v:.4g}",
                       fill=TEXT, font=F_SM)
        cv.create_text((x0 + x1) / 2, h - B + 12, text=lab, fill=MUTED, font=F_SM)
    cv.create_line(L, Y(mean), w - R, Y(mean), fill=AMBER, dash=(4, 3))
    cv.create_text(w - R, Y(mean) - 8, text=f"mean {mean:.4g}", fill=AMBER,
                   font=F_SM, anchor="ne")
    cv.create_text(L + 4, T - 6, text=ylabel, fill=MUTED, font=F_SM, anchor="nw")


# ── header ─────────────────────────────────────────────────────
head = tk.Frame(root, bg=SURFACE)
head.pack(fill="x")
hb = tk.Frame(head, bg=SURFACE); hb.pack(side="left", padx=22, pady=12)
tk.Label(hb, text="Ex", bg=SURFACE, fg=AMBER, font=F_TITLE).pack(side="left")
tk.Label(hb, text="ArMo Lab", bg=SURFACE, fg=TEXT, font=F_TITLE).pack(side="left")

sn_f = tk.Frame(head, bg=SURFACE); sn_f.pack(side="left", padx=30)
tk.Label(sn_f, text="Motor serial no.", bg=SURFACE, fg=MUTED, font=F_LBL).pack(side="left", padx=(0, 8))
e_serial = tk.Entry(sn_f, bg=FIELD, fg=TEXT, insertbackground=TEXT, relief="flat",
                    font=("Consolas", 15, "bold"), width=14, justify="center",
                    highlightthickness=1, highlightbackground=BORDER, highlightcolor=BLUE)
e_serial.pack(side="left", ipady=4)


def port_picker(parent, label, connect_fn, baud):
    f = tk.Frame(parent, bg=SURFACE); f.pack(side="left", padx=14)
    tk.Label(f, text=label, bg=SURFACE, fg=MUTED, font=F_SM).pack(side="left", padx=(0, 6))
    var = tk.StringVar()
    ports = [p.device for p in serial.tools.list_ports.comports()] or ["—"]
    var.set(ports[0])
    om = tk.OptionMenu(f, var, *ports)
    om.config(bg=FIELD, fg=TEXT, relief="flat", bd=0, highlightthickness=0, font=F_SM)
    om.pack(side="left")

    def refresh():
        m = om["menu"]; m.delete(0, "end")
        for p in [q.device for q in serial.tools.list_ports.comports()] or ["—"]:
            m.add_command(label=p, command=lambda v=p: var.set(v))
    tk.Button(f, text="↻", command=refresh, bg=FIELD, fg=MUTED, relief="flat",
              bd=0, font=F_SM, padx=6, cursor="hand2", highlightthickness=0).pack(side="left", padx=3)
    tk.Button(f, text="Connect", command=lambda: connect_fn(var.get()), bg=FIELD,
              fg=BLUE, relief="flat", bd=0, font=("Segoe UI", 11, "bold"),
              padx=10, pady=3, cursor="hand2", highlightthickness=0).pack(side="left")
    st = tk.Label(f, text="● off", bg=SURFACE, fg=FAINT, font=F_SM)
    st.pack(side="left", padx=(6, 0))
    return st


# [g4-E] TWO devices, two pickers. The load cell and the IMU are one board
# now, so the old "Loadcell/Temp" port is gone rather than left on screen
# offering to connect to something that is no longer part of the rig.
conn_f = tk.Frame(head, bg=SURFACE); conn_f.pack(side="right", padx=18)
conn_r1 = tk.Frame(conn_f, bg=SURFACE); conn_r1.pack(anchor="e")
dev_status = port_picker(conn_r1, "Driver", dev_connect, BAUD_DEV)
arm_status = port_picker(conn_r1, "Sensor board", arm_connect, BAUD_ARM)

# tab bar — [g4-A] a horizontally scrollable strip.
# Nine tabs are wider than a 1366-px laptop, and the ones that overflow are
# simply unreachable: they are still packed, just past the right edge with
# nothing to scroll. The strip below scrolls with the wheel, shows no
# scrollbar when everything already fits, and show_tab() brings the selected
# tab into view so a tab can never be selected and invisible at the same time.
tabs_wrap = tk.Frame(root, bg=SURFACE)
tabs_wrap.pack(fill="x")
tabs_cv = tk.Canvas(tabs_wrap, bg=SURFACE, highlightthickness=0, bd=0, height=42)
tabs_cv.pack(fill="x")
tabs_bar = tk.Frame(tabs_cv, bg=SURFACE)
tabs_cv.create_window((0, 0), window=tabs_bar, anchor="nw")


def _tabs_cfg(_e=None):
    tabs_cv.configure(scrollregion=tabs_cv.bbox("all"),
                      height=max(tabs_bar.winfo_reqheight(), 30))


def _tabs_wheel(e):
    if tabs_bar.winfo_reqwidth() <= tabs_cv.winfo_width():
        return                                   # nothing hidden, do nothing
    d = getattr(e, "delta", 0)
    if d:
        tabs_cv.xview_scroll(-3 if d > 0 else 3, "units")
    elif getattr(e, "num", 0) == 4:
        tabs_cv.xview_scroll(-3, "units")
    elif getattr(e, "num", 0) == 5:
        tabs_cv.xview_scroll(3, "units")


WHEEL_SEQS = ("<MouseWheel>", "<Shift-MouseWheel>", "<Button-4>", "<Button-5>")
tabs_bar.bind("<Configure>", _tabs_cfg)
for _s in WHEEL_SEQS:
    tabs_cv.bind(_s, _tabs_wheel)
    tabs_bar.bind(_s, _tabs_wheel)
tk.Frame(root, bg=BORDER, height=1).pack(fill="x")

# [g2-A] sticky fault banner — a firmware fault stays on screen until it is
# cleared by a good event, instead of being overwritten by the next keepalive.
banner = tk.Label(root, text="", bg="#fdecee", fg=RED, font=("Segoe UI", 12, "bold"),
                  anchor="w", padx=18, pady=6)

body = tk.Frame(root, bg=BG)
body.pack(fill="both", expand=True, padx=24, pady=18)

foot = tk.Frame(root, bg=SURFACE)
foot.pack(side="bottom", fill="x")
tk.Frame(root, bg=BORDER, height=1).pack(side="bottom", fill="x")
foot_state = tk.Label(foot, text="—", bg=SURFACE, fg=MUTED,
                      font=("Segoe UI", 12, "bold"))
foot_state.pack(side="left", padx=(22, 16), pady=6)
foot_line = tk.Label(foot, text="", bg=SURFACE, fg=FAINT, font=("Consolas", 12),
                     anchor="w")
foot_line.pack(side="left", fill="x", expand=True)


def foot_tick():
    live = dev_alive()
    foot_state.config(text=STATE_NAMES.get(T["state"], "?") + ("" if live else "  (no telemetry)"),
                      fg=GREEN if (live and T["state"]) else (MUTED if live else RED))
    _kind = {"rehab": "REHAB v2.9 · lab emulation", "lab": "lab firmware"}.get(fw["kind"], "firmware?")
    foot_line.config(text=f"[{_kind}]  rx: {link['rx'][-80:]}    tx: {link['tx'][-20:]}", fg=FAINT)
    if link["fault"]:
        banner.config(text="⚠  " + link["fault"] + "     (Diagnostics tab has the full console)")
        banner.pack(fill="x", before=body)
    else:
        banner.pack_forget()


TABS = []
cur_tab = [0]


def add_tab(name, frame):
    idx = len(TABS)
    b = tk.Label(tabs_bar, text=name, bg=SURFACE, fg=MUTED,
                 font=("Segoe UI", 13, "bold"), padx=16, pady=9, cursor="hand2")
    b.pack(side="left", padx=(14 if idx == 0 else 2, 2))
    b.bind("<Button-1>", lambda e, k=idx: show_tab(k))
    for s in WHEEL_SEQS:                         # [g4-A] wheel over a label too
        b.bind(s, _tabs_wheel)
    TABS.append((b, frame))


def _tabs_see(b):
    """Scroll the strip so tab `b` is fully visible."""
    tabs_cv.update_idletasks()
    total = max(tabs_bar.winfo_reqwidth(), 1)
    view = tabs_cv.winfo_width()
    if total <= view:
        tabs_cv.xview_moveto(0.0)
        return
    left = tabs_cv.canvasx(0)
    x0 = b.winfo_x(); x1 = x0 + b.winfo_width()
    if x0 < left:
        tabs_cv.xview_moveto(max(0.0, (x0 - 8) / total))
    elif x1 > left + view:
        tabs_cv.xview_moveto(min(1.0, (x1 - view + 8) / total))


def show_tab(k):
    cur_tab[0] = k
    for i, (b, f) in enumerate(TABS):
        if i == k:
            b.config(bg=FIELD, fg=BLUE)
            f.pack(fill="both", expand=True)
            _tabs_see(b)
        else:
            b.config(bg=SURFACE, fg=MUTED)
            f.pack_forget()


def stop_all():
    dev_send("O0")


# ═════════════ TAB 0 · DIAGNOSTICS ═════════════════════════════
# This tab exists because v1 gave you no way to answer the only question that
# matters when nothing happens: WHICH link is broken — FOC init, the command
# path, the direction, or the current sense?
dg_fr = tk.Frame(body, bg=BG)

dg_sc_o, dg_sc = scroll_column(dg_fr); dg_sc_o.pack(side="left", fill="y", padx=(0, 14))
dg_l_o, dg_l = card(dg_sc); dg_l_o.pack(fill="both", expand=True)
section(dg_l, "Firmware health")
dg_flag_lbls = []
for _bit, _name in FLAG_BITS:
    r = tk.Frame(dg_l, bg=SURFACE); r.pack(fill="x", pady=2)
    dot = tk.Label(r, text="●", bg=SURFACE, fg=FAINT, font=("Segoe UI", 14))
    dot.pack(side="left", padx=(0, 8))
    tk.Label(r, text=_name, bg=SURFACE, fg=TEXT, font=F_LBL).pack(side="left")
    dg_flag_lbls.append(dot)
dg_fw = tk.Label(dg_l, text="firmware: —", bg=SURFACE, fg=FAINT, font=F_SM)
dg_fw.pack(anchor="w", pady=(8, 0))

# [g7-A] rehab firmware v2.9: homing, status, the emulation's live state
section(dg_l, "Rehab firmware v2.9  (full device)")
dg_rehab = tk.Label(dg_l, text="not detected", bg=SURFACE, fg=FAINT, font=F_SM, justify="left")
dg_rehab.pack(anchor="w")
_rh = tk.Frame(dg_l, bg=SURFACE); _rh.pack(fill="x", pady=(4, 2))
btn(_rh, "⌂ HOME (rehab H)", FIELD, BLUE,
    lambda: (dev_send("H", raw=True) if fw["kind"] == "rehab" else dg_set("Only on the rehab firmware", AMBER)),
    width=16).pack(side="left", padx=(0, 6))
btn(_rh, "⚓ Hold here (G)", FIELD, BLUE,
    lambda: dev_send("H"), width=14).pack(side="left", padx=(0, 6))
btn(_rh, "L", FIELD, MUTED, lambda: dev_send("L"), width=3).pack(side="left")
tk.Label(dg_l, text="On the rehab firmware a current command is EMULATED (LAB velocity mode,\n"
                    "current limit ramped to the setpoint): the shaft must be BLOCKED —\n"
                    "E9 strut, load cell, or a hard stop. A free shaft trips the runaway guard.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))

section(dg_l, "Push direction  (Ps±1)")
dir_var = tk.IntVar(value=1)
dr = tk.Frame(dg_l, bg=SURFACE); dr.pack(fill="x", pady=(0, 6))


def set_dir():
    dev_send(f"Ps{dir_var.get()}")


for _txt, _v in (("+1  (CW)", 1), ("−1  (CCW)", -1)):
    tk.Radiobutton(dr, text=_txt, variable=dir_var, value=_v, command=set_dir,
                   bg=SURFACE, fg=TEXT, selectcolor=FIELD, font=F_LBL,
                   activebackground=SURFACE, highlightthickness=0).pack(side="left", padx=(0, 14))
tk.Label(dg_l, text="Jog, watch the grams. If the arm moves AWAY\nfrom the cell, flip this sign, then re-run the test.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 8))

section(dg_l, "Manual jog")
dg_jog_i = entry_row(dg_l, "Jog current (A)", 0.5)
dg_jog_t = entry_row(dg_l, "Jog time (s)", 2.0)
dg_lever = entry_row(dg_l, "Loadcell lever arm (m)", 0.10)


def engage_until_contact(z, thresh_g, i_start, i_max, step, dwell, alive=None,
                         report=None):
    """Raise the push current until the cell ACTUALLY reads force.

    A fixed engage current is a guess about breakaway torque, and breakaway is
    exactly the thing we do not know yet: static friction plus the gravity
    component has to be overcome before the first gram reaches the cell. Guess
    low and the arm never arrives (and you end up pushing it by hand); guess
    high and first contact is a slam. So don't guess — start low, step up, and
    stop the moment contact is real.

    The current at which contact happens is itself a measurement: it is the
    breakaway current, and it should agree with the friction intercept the Kt
    fit reports. Returns (True, current) or (False, None)."""
    i = i_start
    while i <= i_max + 1e-9:
        dev_send(f"C{i:.3f}")
        if report:
            report(i)
        t_end = time.time() + dwell
        while time.time() < t_end:
            if alive is not None and not alive():
                return (False, None)
            time.sleep(0.1)
            gg = lc_latest(0.5)
            if gg is not None and (gg - z) >= thresh_g:
                return (True, i)
        i += step
    return (False, None)


def _baseline():
    """Zero the load cell with NO current applied. Absolute grams are
    meaningless here — the arm's own weight is already sitting on the cell, so
    only the CHANGE caused by current tells you anything."""
    dev_send("O0")
    time.sleep(0.8)
    return lc_latest(0.6)


def dg_jog():
    try:
        i = float(dg_jog_i.get()); t = max(0.2, min(20.0, float(dg_jog_t.get())))
    except ValueError:
        dg_set("Check the numbers", AMBER); return

    def run():
        dg_set("Zeroing (no current)…", BLUE)
        base = _baseline()
        t_start = time.time()
        dev_send(f"C{i:.3f}")
        dg_set(f"Jogging at {i:.2f} A for {t:.1f} s…", BLUE)
        time.sleep(t)
        m, _ = iq_avg(min(t, 1.0))
        g = lc_latest(0.6)
        dev_send("C0"); stop_all()
        peak = max([abs(r) for (ts, r) in snap_hist(rpm_hist) if ts >= t_start] or [0.0])
        d = None if (g is None or base is None) else (g - base)
        txt = (f"Iq meas {0.0 if m is None else m:+.3f} A · peak {peak:.0f} rpm · "
               f"Δ loadcell {'--' if d is None else f'{d:+.1f} g'} "
               f"(from {'--' if base is None else f'{base:.1f}'} g)")
        if peak > 60 and (d is None or abs(d) < 5):
            root.after(0, lambda: dg_set(txt + "  — shaft was FREE (spun up, no force). "
                                         "Load the arm against the cell and jog again.", AMBER))
        elif d is not None and d < -3:
            root.after(0, lambda: dg_set(txt + "  — NEGATIVE: current is LIFTING the arm OFF "
                                         "the cell, not pressing it. Flip the direction above.", RED))
        else:
            root.after(0, lambda: dg_set(txt, GREEN))
    threading.Thread(target=run, daemon=True).start()


sweep_hist = []          # [(slope_g_per_A, kt_mNm_per_A)] across runs


def dg_sweep():
    """Direction + linearity sweep.

    Judged by a least-squares FIT, not by comparing endpoints. The earlier
    version tested `last_delta < 2.5 * first_delta`, which any constant
    mechanical preload destroys: an arm already leaning on the cell adds the
    same offset to every point, so a perfectly linear response reads as
    "not proportional". Slope is what carries the physics — tau = Kt*Iq + tau_f
    is exactly the model the Kt tabs fit — and slope is immune to that offset.
    The intercept is worth reading too: it IS the preload, in grams."""
    try:
        lever = float(dg_lever.get())
    except ValueError:
        dg_set("Check the lever arm", AMBER); return

    def run():
        if arm["ser"] is None:
            root.after(0, lambda: dg_set("Connect the load cell first", RED)); return
        dg_set("Sweep: zeroing (no current)…", BLUE)
        base = _baseline()
        if base is None:
            root.after(0, lambda: dg_set("No load-cell data", RED)); return
        steps = [0.2, 0.4, 0.6, 0.8, 1.0]
        ok, i_contact = engage_until_contact(base, 8.0, 0.2, 1.5, 0.1, 1.2)
        if not ok:
            dev_send("C0"); stop_all()
            root.after(0, lambda: dg_set(
                "No contact up to 1.5 A — arm not loaded against the cell, or wrong direction.", RED))
            return
        # start at contact: below breakaway the arm is not touching, and a
        # 0 g point drags the slope down while pretending to be data
        steps = [s for s in steps if s >= i_contact - 1e-9] or [i_contact]
        pts = []
        for i in steps:
            dev_send(f"C{i:.3f}")
            root.after(0, lambda i=i: dg_set(f"Sweep: {i:.2f} A…", BLUE))
            time.sleep(1.6)
            gg = lc_latest(0.6)
            m, _ = iq_avg(1.0)
            if gg is not None:
                pts.append((i, gg - base, abs(m) if m is not None else i))
        dev_send("C0"); stop_all()

        rows = " · ".join(f"{i:.1f}A→{d:+.0f}g" for i, d, _ in pts)
        if len(pts) < 3:
            root.after(0, lambda: dg_set("Sweep incomplete — load-cell dropouts. " + rows, AMBER))
            return

        fit = linfit([p[2] for p in pts], [p[1] for p in pts])   # grams vs |Iq meas|
        if fit is None:
            root.after(0, lambda: dg_set(rows + "  fit failed", AMBER)); return
        slope, intercept, r2 = fit
        stats = (f"slope {slope:+.1f} g/A · friction intercept {intercept:+.0f} g · "
                 f"R²={r2:.3f} · breakaway {i_contact:.2f} A")

        if slope < -2:
            msg = ("FALLING with current → the motor is UNLOADING the cell. "
                   "Flip the direction above (Ps) and sweep again.")
            col = RED
        elif abs(slope) < 2:
            msg = ("No torque response → the arm is not loaded against the cell, "
                   "or Iq is clamped. Check Pi and the fixture.")
            col = RED
        elif r2 < 0.90:
            msg = ("Response is NOT linear (R² low) → suspect the dq frame or the "
                   "current-loop tuning. Check id on the right.")
            col = AMBER
        else:
            kt = slope * 1e-3 * G_ACC * lever * 1000.0        # mNm/A, from SLOPE
            msg = f"LINEAR — good. Kt from slope ≈ {kt:.1f} mNm/A."
            col = GREEN
            # cross-check against the independent back-EMF number, if tab 1 has one
            try:
                kv = float(kv_res.get("kv", "")) if kv_res.get("kv") else None
            except ValueError:
                kv = None
            if kv:
                kt_kv = 9.55 / kv * 1000.0
                ratio = kt_kv / kt if kt > 1e-9 else 0
                if ratio > 1.3 or ratio < 0.77:
                    msg += (f" But back-EMF says {kt_kv:.0f} mNm/A — a factor of "
                            f"{ratio:.1f} apart. Linearity is fine, so this is a pure "
                            f"SCALE error: shunt value, amp gain, or the lever arm. "
                            f"Torque magnitudes cannot be trusted until it is resolved.")
                    col = AMBER
                else:
                    msg += f" Back-EMF agrees ({kt_kv:.0f} mNm/A)."
        # Slope across runs is the discriminator: a wrong scale factor (shunt,
        # gain, lever) is CONSTANT, so it cannot move between runs. A wrong
        # commutation angle moves every time the rotor rests somewhere new,
        # because torque falls by cos(error) and the error tracks position.
        if slope > 2:
            sweep_hist.append((slope, slope * 1e-3 * G_ACC * lever * 1000.0))
        if len(sweep_hist) >= 2:
            kts = [k for _, k in sweep_hist[-4:]]
            spread = max(kts) / min(kts) if min(kts) > 1e-9 else 1.0
            hist = " | ".join(f"{k:.0f}" for k in kts)
            msg += f"  ·  Kt across runs: {hist} mNm/A"
            if spread > 1.25:
                msg += (f" — varies {spread:.1f}x between runs. A scale error cannot "
                        f"do that; a wrong commutation angle can. Free the shaft and "
                        f"press RE-ALIGN, then sweep again.")
                col = AMBER
        root.after(0, lambda: dg_set(rows + "   " + stats + "   " + msg, col))
    threading.Thread(target=run, daemon=True).start()


jb = tk.Frame(dg_l, bg=SURFACE); jb.pack(fill="x", pady=(4, 2))
btn(jb, "↻ JOG", BLUE, "white", dg_jog, width=8).pack(side="left", padx=(0, 6))
btn(jb, "■ STOP", RED, "white", stop_all, width=8).pack(side="left")
btn(dg_l, "📈 DIRECTION + LINEARITY SWEEP  (0.2 → 1.0 A, 5 pts)", FIELD, BLUE,
    dg_sweep).pack(fill="x", pady=(6, 0))

section(dg_l, "Current-sense self-test")
tk.Label(dg_l, text="Block the shaft (arm resting on the cell),\nthen run. Firmware pushes 1.0 A for 1 s and\nreports commanded vs measured.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
btn(dg_l, "✓ RUN SELF-TEST (Z)", FIELD, GREEN, lambda: dev_send("Z")).pack(fill="x")
btn(dg_l, "ℹ Status dump (L)", FIELD, BLUE, lambda: dev_send("L")).pack(fill="x", pady=(6, 0))

section(dg_l, "Sensor alignment")
tk.Label(dg_l, text="initFOC only finds a correct zero-electric-angle\nif the shaft was FREE when it ran. Aligned against\na loaded arm it is silently wrong: Iq still tracks\nits setpoint and id still reads ~0, but real torque\nfalls by cos(error) — and changes with position.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))


def dg_realign():
    dev_send("O0")
    root.after(400, lambda: dev_send("A"))
    root.after(500, lambda: dev_send("L"))
    dg_set("Re-aligning — the shaft must be FREE to turn. Watch the console.", BLUE)


btn(dg_l, "⟳ RE-ALIGN  (free the shaft first)", FIELD, AMBER, dg_realign).pack(fill="x")

section(dg_l, "Load cell calibration")
tk.Label(dg_l, text="Everything here is measured AGAINST the load cell,\nso if its grams are wrong every Kt is wrong by the\nsame factor — and it will still look perfectly linear.\nCheck it against known masses before trusting a Kt.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
# [g3-B] Three modes, and YOU pick which one is in force:
#   manual  — you type scale and tare; the boxes are the truth
#   point1  — Zero, then one known mass; quick, and blind to linearity
#   multi   — several known masses, least-squares line through them
# The mode matters because only one thing may own `lc_cal` at a time. In
# manual the tick pushes the boxes into lc_cal; in the other two the computed
# result owns it and the boxes merely display it. Without that rule the
# 200 ms tick would overwrite a fresh calibration with whatever stale text
# happened to be sitting in the entry.
lc_mode = tk.StringVar(value=lc_cal["mode"] if lc_cal["mode"] in
                       ("manual", "point1", "multi") else "manual")
_mr = tk.Frame(dg_l, bg=SURFACE); _mr.pack(fill="x", pady=(0, 4))
for _txt, _v in (("Manual", "manual"), ("1-point", "point1"), ("Multi-point", "multi")):
    tk.Radiobutton(_mr, text=_txt, variable=lc_mode, value=_v,
                   command=lambda: dg_lc_mode_changed(),
                   bg=SURFACE, fg=TEXT, selectcolor=FIELD, font=F_SM,
                   activebackground=SURFACE,
                   highlightthickness=0).pack(side="left", padx=(0, 8))

dg_lc_scale = entry_row(dg_l, "Scale  (g per raw unit)", f"{lc_cal['scale']:.6g}")
dg_lc_tare = entry_row(dg_l, "Tare  (raw units)", f"{lc_cal['tare']:.6g}")
dg_lc_known = entry_row(dg_l, "Known mass (g)", 500.0)
dg_lc_live = tk.Label(dg_l, text="", bg=SURFACE, fg=MUTED, font=F_MONO)
dg_lc_live.pack(anchor="w", pady=(4, 2))
_lc_zero = {"v": None}


def dg_lc_sync_entries():
    for e, v in ((dg_lc_scale, lc_cal["scale"]), (dg_lc_tare, lc_cal["tare"])):
        e.delete(0, "end"); e.insert(0, f"{v:.6g}")


def dg_lc_commit(msg, colour=GREEN):
    lc_cal["mode"] = lc_mode.get()
    lc_cal["when"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    dg_lc_sync_entries()
    ok = lc_cal_save()
    dg_set(msg + ("" if ok else "  (could not write labdata/loadcell_cal.json)"),
           colour if ok else AMBER)


def dg_lc_mode_changed():
    m = lc_mode.get()
    lc_cal["mode"] = m
    if m == "manual":
        dg_set("Manual: the Scale and Tare boxes are now in force. "
               "Type values and they take effect immediately.", BLUE)
    elif m == "point1":
        dg_set("1-point: unload the cell → Tare now → put the known mass on → "
               "Calibrate.", BLUE)
    else:
        dg_set("Multi-point: add the empty cell as a 0 g point first, then each "
               "known mass, then Fit.", BLUE)
    lc_cal_save()


def dg_lc_tare_now():
    """Set the zero from the present RAW reading, whatever the mode."""
    v, drift, n = lc_raw_settled(2.0)
    if v is None:
        dg_set("No load-cell data (need ~2 s of samples)", RED); return
    lc_cal["tare"] = v
    _lc_zero["v"] = v
    note = lc_drift_note(drift)
    dg_lc_commit(f"Tared at raw {v:.1f} — the cell should now read ~0 g." + note,
                 AMBER if note else GREEN)


def dg_lc_apply_manual():
    try:
        s = float(dg_lc_scale.get()); t = float(dg_lc_tare.get())
    except ValueError:
        dg_set("Scale and Tare must both be numbers", AMBER); return
    if abs(s) < 1e-12:
        dg_set("A scale of 0 makes every reading 0 g", AMBER); return
    lc_mode.set("manual")
    lc_cal["scale"] = s; lc_cal["tare"] = t; lc_cal["r2"] = None
    dg_lc_commit(f"Applied manually: scale {s:.6g}, tare {t:.6g}.", GREEN)


def dg_lc_calibrate():
    """One known mass against the captured zero."""
    if _lc_zero["v"] is None:
        dg_set("Press 'Tare now' first, with nothing on the cell", AMBER); return
    v, drift, n = lc_raw_settled(2.0)
    if v is None:
        dg_set("No load-cell data (need ~2 s of samples)", RED); return
    try:
        known = float(dg_lc_known.get())
    except ValueError:
        dg_set("Check the known mass", AMBER); return
    if known <= 0:
        dg_set("The known mass must be greater than zero", AMBER); return
    delta = v - _lc_zero["v"]
    if abs(delta) < 1e-6:
        dg_set("The raw reading did not change — is the mass actually on the cell?",
               RED); return
    lc_mode.set("point1")
    lc_cal["tare"] = _lc_zero["v"]
    lc_cal["scale"] = known / delta
    lc_cal["r2"] = None
    note = lc_drift_note(drift)
    dg_lc_commit(f"1-point: {known:.1f} g moved the raw value by {delta:.1f} → "
                 f"scale {lc_cal['scale']:.6g}. Every Kt scales by the same factor."
                 + note, AMBER if note else GREEN)


def dg_lc_add_point():
    """Record (known mass, raw reading) as one point of the multi-point fit."""
    v, drift, n = lc_raw_settled(2.0)
    if v is None:
        dg_set("No load-cell data (need ~2 s of samples)", RED); return
    try:
        known = float(dg_lc_known.get())
    except ValueError:
        dg_set("Check the known mass", AMBER); return
    lc_cal["points"].append((known, v))
    dg_lc_points_redraw()
    lc_cal_save()
    note = lc_drift_note(drift)
    dg_set(f"Point {len(lc_cal['points'])}: {known:.1f} g at raw {v:.1f}. "
           f"Add more masses, then Fit." + note +
           (" Add a 0 g point (empty cell) too — it is what pins the tare."
            if not any(abs(m) < 1e-9 for m, _ in lc_cal["points"]) else ""),
           AMBER if note else BLUE)


def dg_lc_fit():
    """Least squares through the points: slope is the scale, intercept the tare.

    This is the same argument the Kt logbook makes about dividing one point:
    the offset is real — resting preload, beam imbalance, amplifier zero — and
    dividing mass by reading folds that offset into the slope, worst at the
    lightest masses. Fitting the line estimates the offset instead of pretending
    it is zero."""
    pts = lc_cal["points"]
    if len(pts) < 2:
        dg_set("Need at least 2 points — and they should span the range you "
               "actually measure in", AMBER); return
    raws = [r for _, r in pts]; masses = [m for m, _ in pts]
    if max(raws) - min(raws) < 1e-6:
        dg_set("Every point has the same raw reading — nothing to fit", RED); return
    f = linfit(raws, masses)                    # grams = a·raw + b
    if f is None:
        dg_set("The points do not support a fit", RED); return
    a, b, r2 = f
    if abs(a) < 1e-12:
        dg_set("Fitted slope is zero — check the points", RED); return
    lc_mode.set("multi")
    lc_cal["scale"] = a
    lc_cal["tare"] = -b / a
    lc_cal["r2"] = r2
    dg_lc_points_redraw()
    resid = max(abs(m - (a * r + b)) for m, r in pts)
    warn = ""
    if r2 < 0.999:
        warn = ("  ⚠ r² is poor for a load cell — suspect a mass that moved, a "
                "point taken before the reading settled, or the cell being "
                "loaded off-axis.")
    dg_lc_commit(f"Fit of {len(pts)} points: scale {a:.6g} g/raw, tare {-b/a:.1f} raw, "
                 f"r² = {r2:.5f}, worst residual {resid:.2f} g." + warn,
                 AMBER if warn else GREEN)


def dg_lc_clear_points():
    if not confirm_reset(len(lc_cal["points"]), "calibration points"):
        return
    lc_cal["points"] = []
    lc_cal["r2"] = None
    dg_lc_points_redraw()
    lc_cal_save()
    dg_set("Calibration points cleared. Scale and tare are unchanged.", BLUE)


def dg_lc_drop_point():
    if lc_cal["points"]:
        lc_cal["points"].pop()
        dg_lc_points_redraw()
        lc_cal_save()
        dg_set(f"Removed the last point — {len(lc_cal['points'])} left.", BLUE)


cb = tk.Frame(dg_l, bg=SURFACE); cb.pack(fill="x", pady=(4, 0))
btn(cb, "Tare now", FIELD, MUTED, dg_lc_tare_now, width=9).pack(side="left", padx=(0, 5))
btn(cb, "Apply typed", FIELD, BLUE, dg_lc_apply_manual, width=11).pack(side="left", padx=(0, 5))
btn(cb, "Calibrate", FIELD, GREEN, dg_lc_calibrate, width=10).pack(side="left")

cb2 = tk.Frame(dg_l, bg=SURFACE); cb2.pack(fill="x", pady=(5, 0))
btn(cb2, "+ Add point", FIELD, BLUE, dg_lc_add_point, width=11).pack(side="left", padx=(0, 5))
btn(cb2, "Fit", FIELD, GREEN, dg_lc_fit, width=6).pack(side="left", padx=(0, 5))
btn(cb2, "↶", FIELD, MUTED, dg_lc_drop_point, width=3).pack(side="left", padx=(0, 5))
btn(cb2, "Clear", FIELD, RED, dg_lc_clear_points, width=7).pack(side="left")

dg_lc_table = mktable(dg_l, [("m", "known g", 78), ("r", "raw", 86),
                             ("f", "fit g", 72), ("e", "err g", 68)], height=6)


def dg_lc_points_redraw():
    clear_table(dg_lc_table)
    a = lc_cal["scale"]; t = lc_cal["tare"]
    for m, r in lc_cal["points"]:
        fitg = (r - t) * a
        dg_lc_table.insert("", "end", values=(f"{m:.1f}", f"{r:.1f}",
                                              f"{fitg:.1f}", f"{fitg - m:+.2f}"))


dg_lc_points_redraw()
tk.Label(dg_l, text="Saved to labdata/loadcell_cal.json and stamped as a\n"
                    "'#' comment line at the top of every run CSV, so a log\n"
                    "stays readable after the cell is re-calibrated.\n"
                    "pandas: read_csv(path, comment='#')",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(6, 0))

section(dg_l, "Bus-current check  ·  is Iq real?")
tk.Label(dg_l, text="The one link never measured independently: whether\nthe current in the WINDINGS equals the Iq we believe.\nStall the shaft, read DC bus amps off the PSU at 0 A\nand at a test current, and enter both. Copper loss is\n1.5·Iq²·R, so a 4x current error shows up as 16x in\npower — impossible to miss.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
dg_bus_iq = entry_row(dg_l, "Test Iq commanded (A)", 2.0)
dg_bus_idle = entry_row(dg_l, "Bus current, IDLE (A)", 0.15)
dg_bus_load = entry_row(dg_l, "Bus current, pushing (A)", 0.0)
dg_bus_v = entry_row(dg_l, "Bus voltage (V)", 24.0)
dg_bus_r = entry_row(dg_l, "Phase resistance (ohm)", 0.54)


def dg_bus_check():
    try:
        iq = float(dg_bus_iq.get()); i0 = float(dg_bus_idle.get())
        i1 = float(dg_bus_load.get()); vb = float(dg_bus_v.get())
        r = float(dg_bus_r.get())
    except ValueError:
        dg_set("Check the numbers", AMBER); return
    p = (i1 - i0) * vb
    if p <= 0:
        dg_set("Pushing current is not above idle — is the shaft stalled and current flowing?", RED)
        return
    # copper loss = 3 * I_rms^2 * R_phase = 1.5 * I_peak^2 * R_phase
    i_act = math.sqrt(p / (1.5 * r))
    ratio = i_act / iq if iq else 0
    txt = (f"{p:.2f} W of copper loss → real current ≈ {i_act:.2f} A "
           f"vs {iq:.2f} A believed  ({ratio:.2f}x)")
    if 0.7 < ratio < 1.4:
        dg_set(txt + "  — the current path is honest. The discrepancy is elsewhere "
                     "(KV convention, or the torque geometry).", GREEN)
    elif ratio < 0.7:
        dg_set(txt + "  — LESS current is flowing than reported: the sense scale reads "
                     "HIGH, so the loop under-drives. That is your missing torque.", RED)
    else:
        dg_set(txt + "  — MORE current is flowing than reported: the sense scale reads LOW.", RED)


btn(dg_l, "∑ Compute real current", FIELD, BLUE, dg_bus_check).pack(fill="x", pady=(4, 0))
dg_set = mkstatus(dg_l)

dg_m_o, dg_m = card(dg_fr); dg_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(dg_m, "Serial console  ·  everything the firmware says")
dg_txt = tk.Text(dg_m, bg="#0f1419", fg="#c8d3e0", insertbackground="#c8d3e0",
                 font=("Consolas", 11), relief="flat", height=22, wrap="none")
dg_txt.pack(fill="both", expand=True)
dg_txt.tag_config("tx", foreground="#7dd3fc")
dg_txt.tag_config("rx", foreground="#c8d3e0")
dg_txt.tag_config("err", foreground="#fca5a5")
dg_txt.tag_config("sys", foreground="#fcd34d")
dg_n = [0]


def dg_console_tick():
    with console_lock:
        n = console_seq[0]
        if n != dg_n[0]:
            rows = list(console)
            dg_n[0] = n
        else:
            rows = None
    if rows is not None:
        dg_txt.config(state="normal")
        dg_txt.delete("1.0", "end")
        for ts, kind, text in rows:
            dg_txt.insert("end", f"{ts} {'>' if kind == 'tx' else ' '} {text}\n", kind)
        dg_txt.see("end")
        dg_txt.config(state="disabled")


cmd_row = tk.Frame(dg_m, bg=SURFACE); cmd_row.pack(fill="x", pady=(8, 0))
tk.Label(cmd_row, text="send:", bg=SURFACE, fg=MUTED, font=F_LBL).pack(side="left", padx=(0, 8))
dg_cmd = tk.Entry(cmd_row, bg=FIELD, fg=TEXT, insertbackground=TEXT, relief="flat",
                  font=("Consolas", 13), highlightthickness=1,
                  highlightbackground=BORDER, highlightcolor=BLUE)
dg_cmd.pack(side="left", fill="x", expand=True, ipady=4)


def dg_send_cmd(_e=None):
    c = dg_cmd.get().strip()
    if c:
        dev_send(c, raw=True)              # [g7-A] the console is RAW: typed text goes to the board as is
        dg_cmd.delete(0, "end")


dg_cmd.bind("<Return>", dg_send_cmd)
tk.Button(cmd_row, text="Send", command=dg_send_cmd, bg=FIELD, fg=BLUE,
          relief="flat", bd=0, font=("Segoe UI", 11, "bold"), padx=12, pady=4,
          cursor="hand2", highlightthickness=0).pack(side="left", padx=(8, 0))
tk.Label(dg_m, text="RAW (no translation).  lab:  L  Z  C0.5  O0  Pi4  Ps-1  Pv20  Pm2  Pt0     rehab v2.9:  L  H  O0  V5  MLC3  Pg0.6  Pz0.15",
         bg=SURFACE, fg=FAINT, font=F_SM).pack(anchor="w", pady=(4, 0))

dg_r_o, dg_r = card(dg_fr); dg_r_o.pack(side="left", fill="y")
section(dg_r, "Live · commanded vs measured")
dg_iqc = big_value(dg_r, "Iq COMMANDED (A)", AMBER)
dg_iqm = big_value(dg_r, "Iq MEASURED (A)", BLUE)
dg_verdict = tk.Label(dg_r, text="", bg=SURFACE, fg=MUTED, font=F_LBL,
                      wraplength=280, justify="left")
dg_verdict.pack(pady=(10, 0))
dg_extra = tk.Label(dg_r, text="", bg=SURFACE, fg=FAINT, font=("Consolas", 12),
                    justify="left")
dg_extra.pack(anchor="w", pady=(10, 0))
add_tab("0 · Diagnostics", dg_fr)


def dg_tick():
    # [g4-E] This used to copy the two entry boxes into the calibration on
    # every tick while the mode was "manual". With one shared calibration that
    # is a footgun: calibrate on tab 9, and 200 ms later this tick overwrites
    # it with whatever text these boxes happen to hold — which, on a fresh
    # launch, is scale 0 and tare 0, i.e. no grams at all. Typing a scale now
    # takes effect when "Apply typed" is pressed, which is also the only
    # moment at which anyone means it.
    _rawv = lc_raw_latest(1.0)
    _g = lc_latest(1.0)
    if _rawv is None:
        dg_lc_live.config(text="raw   --        →     --  g", fg=FAINT)
    else:
        _r2 = "" if lc_cal["r2"] is None else f"   r²={lc_cal['r2']:.4f}"
        dg_lc_live.config(
            text=f"raw {_rawv:9.1f}   →  {(_g if _g is not None else 0.0):8.1f} g"
                 f"   [{lc_mode.get()}]{_r2}", fg=MUTED)
    fl = T["flags"]
    idle = (T["state"] == 0)
    for i, (bit, _n) in enumerate(FLAG_BITS):
        if fl < 0:
            dg_flag_lbls[i].config(fg=FAINT)          # old 7-field firmware
        elif bit in (0x02, 0x04) and idle:
            # "motor enabled" and "driver powered" are SUPPOSED to be off in
            # IDLE — that is the true-freewheel design, not a fault. Grey, not
            # red, so a limp shaft never looks like a broken board.
            dg_flag_lbls[i].config(fg=FAINT)
        elif bit == 0x20 and not (fl & bit):
            # [g3-A] AMBER, not red: no thermistor fitted is a perfectly valid
            # way to run a KV or Kt test. It is only a fault for the thermal
            # tabs, and they say so themselves rather than colouring the whole
            # board broken.
            dg_flag_lbls[i].config(fg=AMBER)
        else:
            dg_flag_lbls[i].config(fg=GREEN if (fl & bit) else RED)
    dg_fw.config(text=f"firmware: {link['fw'] or '—'}   ({T['nfields']} telemetry fields)")
    if fw["kind"] == "rehab":                                   # [g7-A]
        _te = fw["T_est"]
        _tt = "—" if _te is None else (f"{_te:.1f} °C" + ("  (thermistor)" if fw.get("T_src") == "ntc" else "  (model)") + ("  ✗ IMPOSSIBLE (IDLE integrator bug)" if _te > 250 or _te < 0 else ""))
        _lim = "—" if fw["lim"][0] is None else f"{fw['lim'][0]:.1f}–{fw['lim'][1]:.1f}°"
        _emu = (f"EMU current {fw['emu_dir']*fw['emu_level']:+.3f} → {fw['emu_dir']*fw['emu_target']:+.3f} A @ {fw['emu_rate']:g} A/s"
                if fw["emu"] else f"cap {fw['emu_cap']:.2f} A · ramp {fw['emu_rate']:g} A/s · dir {fw['emu_dir']:+d}")
        dg_rehab.config(
            text=f"state {fw['rstate']}  homed {'yes' if fw['homed'] else 'NO'}  limits {_lim}  joint {math.degrees(fw['hold_theta']):+.1f}°\n"
                 f"T_est {_tt}   board iq_lim {fw['iq_lim'] if fw['iq_lim'] is not None else '—'}\n{_emu}",
            fg=(RED if (_te is not None and (_te > 250 or _te < 0)) else MUTED))
    else:
        dg_rehab.config(text="not detected (lab firmware, or press L)", fg=FAINT)
    dg_iqc.config(text=f"{T['iqc']:+.3f}")
    dg_iqm.config(text=f"{T['iq']:+.3f}")
    # the actual diagnosis, spelled out
    if not dev_alive():
        dg_verdict.config(text="No telemetry. Connect the driver port, or send D20.", fg=RED)
    elif fl >= 0 and not (fl & 0x01):
        dg_verdict.config(text="initFOC FAILED. The motor physically cannot move: check the AS5047P magnet, pole pairs and bus power.", fg=RED)
    elif fl >= 0 and not (fl & 0x10):
        dg_verdict.config(text="Vbus reading is out of range — phase voltage may be capped at 0. Set it with Pu24.", fg=RED)
    elif idle:
        dg_verdict.config(text="IDLE — bridge is open, shaft is free. Current reads exactly 0 by design; "
                               "sensing cannot be judged until a mode is entered. Jog to test.", fg=MUTED)
    elif abs(T["iqc"]) < 0.05:
        dg_verdict.config(text="Firmware is commanding ~0 A. Nothing is wrong with sensing yet — send a current (jog) first.", fg=MUTED)
    elif abs(T["iq"]) < 0.25 * abs(T["iqc"]):
        dg_verdict.config(text="Current is COMMANDED but not MEASURED. Shunt/ADC path or torque mode: try Pm1, check Pq, check M0_IB/M0_IC wiring.", fg=RED)
    elif abs(T["idq"]) > 0.5 * abs(T["iq"]) and abs(T["iq"]) > 0.2:
        # id should sit near zero: all current on the q axis is what makes
        # torque proportional to Iq. A large id means the dq frame is rotated
        # away from the rotor, so Kt per amp is neither constant nor correct.
        dg_verdict.config(text=f"|id| = {abs(T['idq']):.2f} A is large next to |iq| = {abs(T['iq']):.2f} A. "
                               "The commutation angle is off, so torque will NOT scale with current. "
                               "Suspect the current-sense phase mapping (skip_align) or the sensor zero angle.", fg=RED)
    else:
        dg_verdict.config(text="Command and measurement agree, id is small. If there is still no force, the push DIRECTION is wrong — flip Ps.", fg=GREEN)
    g = lc_latest(1.0)
    _tc = temp_now(2.0)
    dg_extra.config(text=(f"state    {STATE_NAMES.get(T['state'], '?')}\n"
                          f"angle    {math.degrees(T['ang']):+8.1f} deg\n"
                          f"velocity {T['vel']:+8.2f} rad/s\n"
                          f"id meas  {T['idq']:+8.3f} A\n"
                          f"voltage q{T['vq']:+8.3f} V\n"
                          f"loadcell {'--' if g is None else f'{g:8.1f} g'}\n"
                          f"temp     {'--' if _tc is None else f'{_tc:8.1f} C'}   ({temp_label()})"))


# ═════════════ TAB 1 · PHASE RESISTANCE ═══════════════════════
# Manual entry from a 4-wire Kelvin measurement (Fluke 289, low-ohm + REL).
# Nothing here talks to the driver: the point is a reference measurement that
# does NOT go through the motor controller, so it can be trusted when the
# controller itself is under suspicion.
r_fr = tk.Frame(body, bg=BG)
rs = dict(rows=[], res={})
PAIRS = ("AB", "AC", "BC")

r_sc_o, r_sc = scroll_column(r_fr); r_sc_o.pack(side="left", fill="y", padx=(0, 14))
r_l_o, r_l = card(r_sc); r_l_o.pack(fill="both", expand=True)
section(r_l, "Winding termination")
term_var = tk.StringVar(value="star")
tf_ = tk.Frame(r_l, bg=SURFACE); tf_.pack(fill="x", pady=(0, 4))
for _t, _v in (("Star (Y)", "star"), ("Delta (Δ)", "delta")):
    tk.Radiobutton(tf_, text=_t, variable=term_var, value=_v, bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_LBL, activebackground=SURFACE,
                   highlightthickness=0, command=lambda: r_compute()).pack(side="left", padx=(0, 16))
tk.Label(r_l, text="Star:  R_LL = 2·R_phase   →   R_phase = R_LL / 2\n"
                   "Delta: R_LL = R_ph ∥ 2R_ph = ⅔·R_ph   →   R_phase = 1.5·R_LL",
         bg=SURFACE, fg=FAINT, font=("Consolas", 11), justify="left").pack(anchor="w", pady=(0, 8))

section(r_l, "Measured phase-to-phase (Ω) · 4-wire Kelvin")
tk.Label(r_l, text="Fluke 289, low-ohm range, REL to null the leads.\nThree readings per pair — reseat the clips between\nreadings so contact resistance shows up as scatter.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))

grid = tk.Frame(r_l, bg=SURFACE); grid.pack(fill="x")
tk.Label(grid, text="", bg=SURFACE, width=5).grid(row=0, column=0)
for c in range(3):
    tk.Label(grid, text=f"#{c+1}", bg=SURFACE, fg=MUTED, font=F_SM).grid(row=0, column=c + 1, padx=3)
r_entries = {}
for rix, pair in enumerate(PAIRS):
    tk.Label(grid, text=pair, bg=SURFACE, fg=TEXT, font=F_LBL, width=5,
             anchor="w").grid(row=rix + 1, column=0, pady=2)
    for c in range(3):
        e = tk.Entry(grid, bg=FIELD, fg=BLUE, insertbackground=TEXT, relief="flat",
                     font=("Consolas", 13, "bold"), width=9, justify="center",
                     highlightthickness=1, highlightbackground=BORDER, highlightcolor=BLUE)
        e.grid(row=rix + 1, column=c + 1, padx=3, pady=2, ipady=2)
        e.bind("<KeyRelease>", lambda _e: r_compute())
        r_entries[(pair, c)] = e

r_temp = entry_row(r_l, "Winding temperature (°C)", 25.0)
r_kt = entry_row(r_l, "Kt for Km (mN·m/A, optional)", 86.8)

rb = tk.Frame(r_l, bg=SURFACE); rb.pack(fill="x", pady=(8, 2))
btn(rb, "∑ Compute", FIELD, BLUE, lambda: r_compute(), width=10).pack(side="left", padx=(0, 6))
btn(rb, "⤓ Save CSV", FIELD, GREEN, lambda: r_save(), width=10).pack(side="left")


def r_clear():
    n = sum(1 for e in r_entries.values() if e.get().strip())
    if not confirm_reset(n, "readings"):
        return
    for e in r_entries.values():
        e.delete(0, "end")
    r_compute()
    r_set("Data reset — enter readings again", BLUE)


btn(r_l, "↺ Reset data", FIELD, RED, r_clear).pack(fill="x", pady=(6, 0))
r_set = mkstatus(r_l)

r_r_sc_o, r_r_sc = scroll_column(r_fr); r_r_sc_o.pack(side="left", fill="y")
r_r_o, r_r = card(r_r_sc); r_r_o.pack(fill="both", expand=True)
section(r_r, "Per pair")
r_table = mktable(r_r, [("p", "pair", 70), ("m", "mean Ω", 100),
                        ("s", "std Ω", 90), ("d", "vs mean", 90)], height=4)
section(r_r, "Result")
rr = tk.Frame(r_r, bg=SURFACE); rr.pack()
rc1 = tk.Frame(rr, bg=SURFACE); rc1.pack(side="left", padx=12)
r_lbl_ph = tk.Label(rc1, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 28, "bold"))
r_lbl_ph.pack(); tk.Label(rc1, text="R_phase [Ω]", bg=SURFACE, fg=FAINT, font=F_SM).pack()
rc2 = tk.Frame(rr, bg=SURFACE); rc2.pack(side="left", padx=12)
r_lbl_km = tk.Label(rc2, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 28, "bold"))
r_lbl_km.pack(); tk.Label(rc2, text="Km [mN·m/√W]", bg=SURFACE, fg=FAINT, font=F_SM).pack()
r_lbl_note = tk.Label(r_r, text="enter at least one pair", bg=SURFACE, fg=FAINT,
                      font=F_LBL, wraplength=330, justify="left")
r_lbl_note.pack(pady=(8, 0))
section(r_r, "Balance across pairs")
r_cv = tk.Canvas(r_r, width=340, height=190, bg=SURFACE, highlightthickness=0)
r_cv.pack(fill="x", pady=(0, 4))


def r_pair_stats():
    out = {}
    for pair in PAIRS:
        vals = []
        for c in range(3):
            t = r_entries[(pair, c)].get().strip()
            if t:
                try:
                    vals.append(float(t))
                except ValueError:
                    pass
        if vals:
            m = sum(vals) / len(vals)
            sd = (sum((x - m) ** 2 for x in vals) / len(vals)) ** 0.5
            out[pair] = (m, sd, len(vals))
    return out


def r_compute():
    st = r_pair_stats()
    for it in r_table.get_children():
        r_table.delete(it)
    means = [st[p][0] if p in st else None for p in PAIRS]
    have = [m for m in means if m is not None]
    draw_bars(r_cv, list(PAIRS), means, ylabel="phase-to-phase Ω", warn_at=2.0)
    if not have:
        r_lbl_ph.config(text="—"); r_lbl_km.config(text="—")
        r_lbl_note.config(text="enter at least one pair", fg=FAINT)
        return
    grand = sum(have) / len(have)
    for p in PAIRS:
        if p in st:
            m, sd, n = st[p]
            r_table.insert("", "end", values=(p, f"{m:.4f}", f"{sd:.4f}",
                                              f"{(m-grand)/grand*100:+.2f}%"))
    r_ll = grand
    star = term_var.get() == "star"
    r_phase = r_ll / 2.0 if star else 1.5 * r_ll
    rs["res"] = dict(r_ll=r_ll, r_phase=r_phase, term=term_var.get())
    r_lbl_ph.config(text=f"{r_phase:.4f}")
    try:
        kt = float(r_kt.get()) * 1e-3
        km = kt / math.sqrt(r_phase) if r_phase > 0 else None
        r_lbl_km.config(text=f"{km*1000:.1f}" if km else "—")
        rs["res"]["km"] = km
    except (ValueError, ZeroDivisionError):
        r_lbl_km.config(text="—")
    spread = (max(have) - min(have)) / grand * 100 if len(have) > 1 and grand else 0.0
    rs["res"]["imbalance_pct"] = spread
    note = (f"R_LL mean {r_ll:.4f} Ω · {'star' if star else 'delta'} → "
            f"R_phase {r_phase:.4f} Ω · imbalance {spread:.2f}%")
    if len(have) < 3:
        r_lbl_note.config(text=note + "  · enter all three pairs to judge balance", fg=MUTED)
    elif spread > 2.0:
        # This is the check that would have caught the dead motor immediately.
        r_lbl_note.config(text=note + "  · IMBALANCE > 2% — the three pairs should be "
                                      "identical. Suspect a shorted or open winding, or a "
                                      "bad joint. Re-seat the clips and re-measure before "
                                      "trusting any Kt from this motor.", fg=RED)
    else:
        r_lbl_note.config(text=note + "  · pairs balanced", fg=GREEN)


def r_save():
    st = r_pair_stats()
    if not st:
        r_set("Nothing to save", AMBER); return
    path = log_path("phase-resistance")
    if not path:
        r_set("Enter the motor serial number first", RED); return
    r_compute()
    res = rs.get("res", {})
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pair", "r1_ohm", "r2_ohm", "r3_ohm", "mean_ohm", "std_ohm"])
        for p in PAIRS:
            vals = [r_entries[(p, c)].get().strip() for c in range(3)]
            if p in st:
                m, sd, _ = st[p]
                w.writerow([p] + vals + [f"{m:.5f}", f"{sd:.5f}"])
        w.writerow([])
        w.writerow(["termination", res.get("term", "")])
        w.writerow(["r_line_to_line_ohm", f"{res.get('r_ll', 0):.5f}"])
        w.writerow(["r_phase_ohm", f"{res.get('r_phase', 0):.5f}"])
        w.writerow(["imbalance_pct", f"{res.get('imbalance_pct', 0):.3f}"])
        w.writerow(["winding_temp_C", r_temp.get().strip()])
        w.writerow(["kt_mNm_per_A", r_kt.get().strip()])
        if res.get("km"):
            w.writerow(["km_mNm_per_sqrtW", f"{res['km']*1000:.4f}"])
        w.writerow(["method", "4-wire Kelvin, Fluke 289 low-ohm + REL, 3 readings per pair"])
    r_set(f"Saved → {path}", GREEN)


add_tab("1 · Phase R", r_fr)
r_compute()

# ═════════════ TAB 1 · KV back-EMF ═════════════════════════════
kv_fr = tk.Frame(body, bg=BG)
kv = dict(steps=[], idx=0, spinning=False, points=[])

# How the back-EMF voltage was measured decides the Kt conversion, and getting
# it wrong is a silent factor-of-2 class error. For a sinusoidal PMSM with
# lambda = flux linkage per phase (peak), p = pole pairs:
#     phase peak  E = lambda*p*w        torque tau = 1.5*p*lambda*iq
#     phase rms   E = lambda*p*w/sqrt2
#     line-line   E = sqrt3 * phase
# so with k = V per (rad/s mech) = 9.5493/KV, Kt = C*k with C as below.
# The familiar "8.27/KV" shorthand is the line-to-line PEAK case; a multimeter
# on AC volts across two leads gives line-to-line RMS, which is 11.70/KV.
BEMF_KINDS = {
    "ll_rms": ("line-to-line, RMS  (multimeter AC volts)", 1.5 * math.sqrt(2.0 / 3.0)),
    "ll_pk":  ("line-to-line, peak (scope)", 1.5 / math.sqrt(3.0)),
    "ph_rms": ("phase-to-neutral, RMS", 1.5 * math.sqrt(2.0)),
    "ph_pk":  ("phase-to-neutral, peak", 1.5),
}

kv_sc_o, kv_sc = scroll_column(kv_fr); kv_sc_o.pack(side="left", fill="y", padx=(0, 14))
kv_l_o, kv_l = card(kv_sc); kv_l_o.pack(fill="both", expand=True)
section(kv_l, "Test setup · DUT terminals OPEN")
kv_start = entry_row(kv_l, "Start speed (DUT rpm)", 300)
kv_end = entry_row(kv_l, "End speed (DUT rpm)", 1500)
kv_n = entry_row(kv_l, "Number of steps", 6)
kv_ratio = entry_row(kv_l, "Coupling ratio (DUT/drive)", 1.0)
kv_win = entry_row(kv_l, "Speed averaging window (s)", 3.0)
section(kv_l, "What the meter measures")
kv_kind = tk.StringVar(value="ll_rms")
for _k, (_lab, _c) in BEMF_KINDS.items():
    tk.Radiobutton(kv_l, text=f"{_lab}   →  Kt = {_c*9.5493:.2f}/KV",
                   variable=kv_kind, value=_k, bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
                   highlightthickness=0, anchor="w",
                   command=lambda: kv_fit()).pack(anchor="w")
tk.Label(kv_l, text="KV itself is just 1/slope and does not depend on this;\n"
                    "Kt does, by up to 2.4x across these four conventions.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(2, 6))


def kv_build():
    try:
        lo = float(kv_start.get()); hi = float(kv_end.get()); n = max(2, int(kv_n.get()))
    except ValueError:
        kv_set("Check the numbers", AMBER); return
    kv["steps"] = [lo + (hi - lo) * i / (n - 1) for i in range(n)]
    kv["idx"] = 0; kv_step_ui()
    kv_set(f"{n} steps ready", GREEN)


btn(kv_l, "⚙  Build steps", FIELD, BLUE, kv_build).pack(pady=(6, 12), fill="x")
section(kv_l, "Step control")
sc = tk.Frame(kv_l, bg=SURFACE); sc.pack()


def kv_ratio_v():
    try:
        return max(0.01, float(kv_ratio.get()))
    except ValueError:
        return 1.0


def kv_send():
    dut = kv["steps"][kv["idx"]]
    dev_send(f"V{dut / kv_ratio_v() * 2 * math.pi / 60:.2f}")
    kv_set(f"Step {kv['idx']+1}/{len(kv['steps'])}: {dut:.0f} rpm", BLUE)


def kv_move(d):
    if not kv["steps"]:
        return
    kv["idx"] = max(0, min(len(kv["steps"]) - 1, kv["idx"] + d))
    kv_step_ui()
    if kv["spinning"]:
        kv_send()


def kv_spin():
    if not kv["steps"]:
        kv_build()
    kv["spinning"] = True; kv_send()


def kv_stop():
    kv["spinning"] = False; dev_send("V0"); stop_all()
    kv_set("Stopped — shaft released", RED)


btn(sc, "◀", FIELD, TEXT, lambda: kv_move(-1), width=3).pack(side="left", padx=3)
kv_lbl_step = tk.Label(sc, text="—", bg=SURFACE, fg=TEXT, font=("Segoe UI", 16, "bold"), width=7)
kv_lbl_step.pack(side="left")
btn(sc, "▶", FIELD, TEXT, lambda: kv_move(+1), width=3).pack(side="left", padx=3)
kv_lbl_target = tk.Label(kv_l, text="—", bg=SURFACE, fg=AMBER, font=("Consolas", 28, "bold"))
kv_lbl_target.pack(pady=(6, 0))
tk.Label(kv_l, text="target rpm (DUT)", bg=SURFACE, fg=FAINT, font=F_SM).pack()


def kv_step_ui():
    kv_lbl_step.config(text=f"{kv['idx']+1} / {len(kv['steps'])}" if kv["steps"] else "—")
    kv_lbl_target.config(text=f"{kv['steps'][kv['idx']]:.0f}" if kv["steps"] else "—")


bb = tk.Frame(kv_l, bg=SURFACE); bb.pack(pady=(10, 2))
btn(bb, "▶ SPIN", GREEN, "white", kv_spin, width=8).pack(side="left", padx=(0, 6))
btn(bb, "■ STOP", RED, "white", kv_stop, width=8).pack(side="left")

section(kv_l, "Velocity-loop tuning")
tk.Label(kv_l, text="If the speed hunts or wanders, it is this loop —\nSimpleFOC's defaults are generic, and a coupled DUT\nadds cogging as a periodic load torque. Lower P and\nraise the filter to trade response for steadiness;\nfor KV only steadiness matters.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
kv_vp = entry_row(kv_l, "Velocity P", 0.20)
kv_vi = entry_row(kv_l, "Velocity I", 5.0)
kv_vf = entry_row(kv_l, "Velocity LPF Tf (s)", 0.020)


def kv_apply_pid():
    """Sent through the SimpleFOC Commander motor passthrough:
    MVP/MVI set the velocity PID gains, MVF its input filter."""
    try:
        p = float(kv_vp.get()); i = float(kv_vi.get()); f = float(kv_vf.get())
    except ValueError:
        kv_set("Check the numbers", AMBER); return
    dev_send(f"MVP{p:.4f}")
    dev_send(f"MVI{i:.4f}")
    dev_send(f"MVF{f:.4f}")
    kv_set(f"Sent P={p} I={i} Tf={f}s — re-run the step and watch the drift.", GREEN)


btn(kv_l, "⚙ Apply velocity gains", FIELD, BLUE, kv_apply_pid).pack(fill="x", pady=(4, 0))
kv_set = mkstatus(kv_l)

kv_m_o, kv_m = card(kv_fr); kv_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(kv_m, "Live speed")
kv_rpm = big_value(kv_m, "DUT rpm (measured)")
kv_steady = tk.Label(kv_m, text="", bg=SURFACE, fg=FAINT, font=("Segoe UI", 14, "bold"))
kv_steady.pack(pady=(4, 8))
kv_chart = tk.Canvas(kv_m, width=560, height=240, bg=SURFACE, highlightthickness=0)
kv_chart.pack(pady=4, fill="x")

kv_rsc_o, kv_rsc = scroll_column(kv_fr); kv_rsc_o.pack(side="left", fill="y")
kv_r_o, kv_r = card(kv_rsc); kv_r_o.pack(fill="both", expand=True)
section(kv_r, "Log a point · back-EMF from multimeter")
pf = tk.Frame(kv_r, bg=SURFACE); pf.pack(fill="x", pady=(0, 6))
tk.Label(pf, text="Phase pair measured:", bg=SURFACE, fg=TEXT, font=F_LBL).pack(side="left")
kv_pair_var = tk.StringVar(value="AB")
for _p in ("AB", "AC", "BC"):
    tk.Radiobutton(pf, text=_p, variable=kv_pair_var, value=_p, bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_LBL, activebackground=SURFACE,
                   highlightthickness=0).pack(side="left", padx=(8, 0))

vf = tk.Frame(kv_r, bg=SURFACE); vf.pack(fill="x")
kv_volt = tk.Entry(vf, bg=FIELD, fg=AMBER, insertbackground=TEXT, relief="flat",
                   font=("Consolas", 18, "bold"), width=8, justify="center",
                   highlightthickness=1, highlightbackground=BORDER, highlightcolor=BLUE)
kv_volt.pack(side="left", ipady=5)
tk.Label(vf, text="V", bg=SURFACE, fg=MUTED, font=F_LBL).pack(side="left", padx=(6, 10))


def kv_steady_rpm(window=None):
    """Speed MAGNITUDE and its drift, from shaft angle. See kv_speed().

    Magnitude because spinning CCW gives negative rpm, and V = a*n + b would
    then fit a NEGATIVE slope that kv_fit() rejects as "bad slope" — a good
    CCW run made unusable by a sign convention. KV is defined on back-EMF
    magnitude."""
    if window is None:
        try:
            window = max(0.5, float(kv_win.get()))
        except (ValueError, tk.TclError):
            window = 3.0
    m, spread = kv_speed(window)
    return (None, None) if m is None else (abs(m), spread)


def kv_log():
    try:
        v = float(kv_volt.get())
    except ValueError:
        kv_set("Type the voltage first", AMBER); return
    m, spread = kv_steady_rpm()
    if m is None:
        kv_set("No telemetry yet", AMBER); return
    dut = m * kv_ratio_v()
    dr = (spread or 0.0) * kv_ratio_v()
    tgt = kv["steps"][kv["idx"]] if kv["steps"] else dut
    pair = kv_pair_var.get()
    kv["points"].append((tgt, dut, v, dr, pair))
    kv_table.insert("", "end", values=(pair, f"{tgt:.0f}", f"{dut:.0f}",
                                       f"{v:.3f}", f"±{dr:.1f}"))
    kv_volt.delete(0, "end"); kv_fit()
    # the meter reading and the speed must describe the SAME interval; a point
    # logged while the speed was still moving pairs a voltage with a speed the
    # motor was not actually holding
    if dr > max(2.0, 0.01 * abs(dut)):
        kv_set(f"Logged, but the speed drifted {dr:.1f} rpm over the window — "
               f"this point pairs a meter reading with a speed that was still moving.", AMBER)
    else:
        kv_set(f"Logged at {dut:.0f} rpm (drift {dr:.1f} rpm)", GREEN)


def kv_undo():
    if kv["points"]:
        kv["points"].pop(); kv_table.delete(kv_table.get_children()[-1]); kv_fit()


btn(vf, "+ Log point", BLUE, "white", kv_log).pack(side="left")
kv_volt.bind("<Return>", lambda e: kv_log())
kv_table = mktable(kv_r, [("ph", "phase", 60), ("t", "target rpm", 85),
                          ("m", "meas rpm", 95), ("v", "back-EMF V", 90),
                          ("d", "drift", 65)])
kb = tk.Frame(kv_r, bg=SURFACE); kb.pack(pady=(0, 10))


def kv_export():
    if not kv["points"]:
        kv_set("Nothing to save", AMBER); return
    path = log_path("kv-backemf")
    if not path:
        kv_set("Enter the motor serial number first", RED); return
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase_pair", "target_rpm", "measured_rpm", "backemf_V",
                    "speed_drift_rpm"])
        for r in kv["points"]:
            w.writerow([r[4], f"{r[0]:.1f}", f"{r[1]:.1f}", f"{r[2]:.4f}", f"{r[3]:.2f}"])
        w.writerow([])
        w.writerow(["kv_rpm_per_V", kv_res.get("kv", "")])
        w.writerow(["bemf_measurement", BEMF_KINDS[kv_kind.get()][0]])
        w.writerow(["kt_mNm_per_A", kv_res.get("kt", "")])
        w.writerow(["kt_formula", f"{BEMF_KINDS[kv_kind.get()][1]*9.5493:.4f}/KV"])
    kv_set(f"Saved → {path}", GREEN)


def kv_reset():
    if not confirm_reset(len(kv["points"]), "logged points"):
        return
    kv["points"].clear()
    clear_table(kv_table)
    kv_fit()
    kv_set("Data reset — table and fit cleared", BLUE)


btn(kb, "⌫ Undo", FIELD, MUTED, kv_undo).pack(side="left", padx=(0, 6))
btn(kb, "↺ Reset", FIELD, RED, kv_reset).pack(side="left", padx=(0, 6))
btn(kb, "⤓ Save CSV", FIELD, GREEN, kv_export).pack(side="left")
section(kv_r, "Result · V = a·n + b → KV = 1/a")
rf = tk.Frame(kv_r, bg=SURFACE); rf.pack()
c1 = tk.Frame(rf, bg=SURFACE); c1.pack(side="left", padx=14)
kv_lbl_kv = tk.Label(c1, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
kv_lbl_kv.pack(); tk.Label(c1, text="KV [rpm/V]", bg=SURFACE, fg=FAINT, font=F_SM).pack()
c2 = tk.Frame(rf, bg=SURFACE); c2.pack(side="left", padx=14)
kv_lbl_kt = tk.Label(c2, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
kv_lbl_kt.pack(); tk.Label(c2, text="Kt [mNm/A]", bg=SURFACE, fg=FAINT, font=F_SM).pack()
kv_lbl_r2 = tk.Label(kv_r, text="log ≥ 2 points", bg=SURFACE, fg=FAINT, font=F_LBL)
kv_lbl_r2.pack(pady=(8, 0))
section(kv_r, "V = a·n + b")
kv_scatter = tk.Canvas(kv_r, width=340, height=200, bg=SURFACE, highlightthickness=0)
kv_scatter.pack(fill="x", pady=(0, 4))
kv_res = {}


def kv_fit():
    xs = [p[1] for p in kv["points"]]; ys = [p[2] for p in kv["points"]]
    if len(kv["points"]) < 2:
        kv_lbl_kv.config(text="—"); kv_lbl_kt.config(text="—")
        kv_lbl_r2.config(text="log ≥ 2 points")
        draw_scatter(kv_scatter, xs, ys, None, "measured rpm", "back-EMF V")
        return
    fit = linfit(xs, ys)
    draw_scatter(kv_scatter, xs, ys, fit, "measured rpm", "back-EMF V")
    if fit is None or fit[0] <= 1e-9:
        kv_lbl_r2.config(text="bad slope — check points"); return
    a, b, r2 = fit
    kvv = 1.0 / a
    C = BEMF_KINDS[kv_kind.get()][1]
    kt = C * 9.5493 / kvv                      # N·m/A, dq (peak-phase) current
    kv_res["kv"] = f"{kvv:.2f}"
    kv_res["kt"] = f"{kt*1000:.2f}"
    kv_res["kind"] = kv_kind.get()
    kv_lbl_kv.config(text=f"{kvv:.1f}")
    kv_lbl_kt.config(text=f"{kt*1000:.1f}")
    txt = f"R² = {r2:.4f} · offset {b*1000:+.0f} mV · {len(kv['points'])} pts"
    # per-pair breakdown: the three should agree, exactly as for resistance
    pairs = {}
    for p in kv["points"]:
        pairs.setdefault(p[4], []).append(p)
    if len(pairs) > 1:
        per = {}
        for name, pts in pairs.items():
            if len(pts) >= 2:
                f2 = linfit([q[1] for q in pts], [q[2] for q in pts])
                if f2 and f2[0] > 1e-9:
                    per[name] = 1.0 / f2[0]
        if len(per) > 1:
            spread = (max(per.values()) - min(per.values())) / (sum(per.values()) / len(per)) * 100
            txt += "  ·  " + " ".join(f"{k}:{v:.1f}" for k, v in sorted(per.items()))
            txt += f" ({spread:.1f}% spread)"
            if spread > 3.0:
                txt += " ⚠ pairs disagree — check the winding"
    # a large intercept means the fit is not passing through the origin, which
    # a back-EMF line physically must
    if abs(b) > 0.25:
        txt += f"  ⚠ offset {b*1000:+.0f} mV is large for a back-EMF line"
    kv_lbl_r2.config(text=txt)


add_tab("2 · KV · Back-EMF", kv_fr)

# ═════════════ TAB 2 · KT add-weight ═══════════════════════════
w_fr = tk.Frame(body, bg=BG)
wt = dict(rows=[], holding=False, ang0=0.0, ang0_set=False)

w_sc_o, w_sc = scroll_column(w_fr); w_sc_o.pack(side="left", fill="y", padx=(0, 14))
w_l_o, w_l = card(w_sc); w_l_o.pack(fill="both", expand=True)
section(w_l, "Setup · rotate arm HORIZONTAL first (shaft is free in IDLE)")
w_len = entry_row(w_l, "Arm length L to weights (m)", 0.10)
w_arm = entry_row(w_l, "Arm effective weight at tip (g)", 25.0)
tk.Label(w_l, text="= the mass which, placed AT RADIUS L, makes the same\n"
                   "gravity torque as the real arm:  m_eff = Σ(mᵢ·rᵢ) / L.\n"
                   "A uniform bar contributes HALF its mass (CoM at L/2);\n"
                   "a hook or pan at the tip counts in full.\n"
                   "It shifts the friction intercept, NOT Kt — the slope is\n"
                   "immune to any constant torque. Measure it if you want the\n"
                   "intercept to mean friction+cogging; otherwise leave it.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 4))
_arm_zero = {"v": None}


def w_arm_lc_zero():
    """Step 1: nothing on the cell."""
    v = lc_latest(1.0)
    if v is None:
        w_set("No load-cell data — connect it first", RED); return
    _arm_zero["v"] = v
    w_set(f"Zero {v:.1f} g. Now rest the arm HORIZONTALLY on the cell, "
          f"contacting at exactly radius L, motor in IDLE.", BLUE)


def w_arm_lc_capture():
    """Step 2: arm resting on the cell at radius L.

    Supporting the arm at radius L with the pivot at the shaft makes the
    scale force times L equal the arm's gravity torque — which is the
    definition of the effective tip mass. So the cell reads m_eff directly,
    with no need to find the centre of mass or weigh anything separately."""
    if _arm_zero["v"] is None:
        w_set("Press Zero first, with nothing on the cell", AMBER); return
    v = lc_latest(1.0)
    if v is None:
        w_set("No load-cell data", RED); return
    m_eff = v - _arm_zero["v"]
    if m_eff <= 0:
        w_set(f"Reading went the wrong way ({m_eff:+.1f} g) — is the arm "
              f"resting ON the cell?", RED); return
    w_arm.delete(0, "end"); w_arm.insert(0, f"{m_eff:.1f}")
    w_set(f"Arm effective weight = {m_eff:.1f} g (measured at radius L)", GREEN)


_ab = tk.Frame(w_l, bg=SURFACE); _ab.pack(fill="x", pady=(0, 6))
tk.Label(_ab, text="from load cell:", bg=SURFACE, fg=MUTED, font=F_SM).pack(side="left", padx=(0, 6))
btn(_ab, "Zero", FIELD, MUTED, w_arm_lc_zero, width=5).pack(side="left", padx=(0, 4))
btn(_ab, "Capture", FIELD, GREEN, w_arm_lc_capture, width=8).pack(side="left")
w_iqmx = entry_row(w_l, "Max Iq protection (A)", 4.0)


def w_hold():
    # Capture the HORIZONTAL reference BEFORE energising: in IDLE the shaft is
    # free and the arm is sitting where you placed it, so this angle is the
    # true horizontal. Every later sag is measured against it.
    #
    # But only ONCE per data set. Pressing HOLD again part-way through a run —
    # after the arm has already sagged — would re-zero the reference to the
    # sagged position. Every later tilt would then be under-reported, the
    # cos() correction would under-correct, and the SLOPE would inflate. The
    # rows already collected were measured against the original reference, so
    # changing it mid-set silently mixes two different geometries.
    if wt["rows"] and wt.get("ang0_set"):
        w_set(f"HOLDING — keeping the original horizontal reference "
              f"({math.degrees(wt['ang0']):+.1f}°). Level the arm and press "
              f"'Re-zero horizontal' if you have re-positioned it.", AMBER)
    else:
        wt["ang0"] = T["ang"]
        wt["ang0_set"] = True
    try:
        dev_send(f"Pi{float(w_iqmx.get()):.2f}")
    except ValueError:
        pass
    dev_send("H")
    wt["holding"] = True
    if not (wt["rows"] and wt.get("ang0_set")):
        w_set(f"HOLDING (horizontal reference {math.degrees(wt['ang0']):+.1f}°) — "
              f"add a weight, enter grams, Record", GREEN)


def w_stop():
    wt["holding"] = False
    stop_all()
    w_set("Released — shaft free", RED)


def w_rezero():
    wt["ang0"] = T["ang"]
    wt["ang0_set"] = True
    w_set(f"Horizontal reference set to {math.degrees(wt['ang0']):+.1f}°. "
          f"Rows already recorded used the previous reference — reset the data "
          f"if you have changed the geometry.", BLUE)


hb2 = tk.Frame(w_l, bg=SURFACE); hb2.pack(pady=(8, 2))
btn(hb2, "⚓ HOLD", GREEN, "white", w_hold, width=9).pack(side="left", padx=(0, 6))
btn(hb2, "■ RELEASE", RED, "white", w_stop, width=10).pack(side="left")
btn(w_l, "⌖ Re-zero horizontal (arm levelled, motor released)", FIELD, MUTED,
    w_rezero).pack(fill="x", pady=(4, 0))
section(w_l, "Record a step")
w_add = entry_row(w_l, "Total added weight on arm (g)", 0.0)


def w_record():
    if not wt["holding"]:
        w_set("HOLD first", AMBER); return
    try:
        add_g = float(w_add.get()); L = float(w_len.get()); arm_g = float(w_arm.get())
    except ValueError:
        w_set("Check the numbers", AMBER); return
    if T["state"] != 2:
        w_set(f"Firmware is not HOLDING (state = {STATE_NAMES.get(T['state'],'?')}). "
              "Press HOLD again and watch the Diagnostics console.", RED)
        return
    m, sd = iq_avg(1.0)
    if m is None:
        w_set("No telemetry — connect first", AMBER); return
    # A PD hold has finite stiffness, so the arm sags further with every added
    # weight: error = Iq / Kp. Gravity torque is m·g·L·cos(tilt), and ignoring
    # the cosine bends the SLOPE — it grows with load, so it is not absorbed by
    # the intercept the way a constant would be. Measure the tilt from the
    # encoder rather than trusting a gain value.
    tilt = T["ang"] - wt.get("ang0", 0.0)
    tau_flat = (add_g + arm_g) * 1e-3 * G_ACC * L
    tau = tau_flat * math.cos(tilt)
    tilt_deg = math.degrees(tilt)
    wt["rows"].append((add_g, add_g + arm_g, tau, abs(m), sd or 0.0, tilt_deg))
    w_table.insert("", "end", values=(f"{add_g:.0f}", f"{tau*1000:.1f}",
                                      f"{abs(m):.3f}", f"±{(sd or 0)*1000:.0f}m",
                                      f"{tilt_deg:+.1f}°"))
    w_fit()
    if abs(tilt_deg) > 5:
        w_set(f"Recorded {tau*1000:.1f} mNm at {abs(m):.3f} A — but the arm has sagged "
              f"{tilt_deg:+.1f}°. Corrected by cos(), but the correction is only as good "
              f"as the horizontal reference — raise the hold stiffness (Pp), or "
              f"RELEASE, re-level, HOLD and re-zero between steps.", AMBER)
    else:
        w_set(f"Recorded: {tau*1000:.1f} mNm at {abs(m):.3f} A (tilt {tilt_deg:+.1f}°)", GREEN)


btn(w_l, "➕ Record value", BLUE, "white", w_record).pack(pady=(4, 8), fill="x")


def w_end():
    if not wt["rows"]:
        w_set("No rows to save", AMBER); return
    path = log_path("kt-weight")
    if not path:
        w_set("Enter the motor serial number first", RED); return
    _f, wcsv = open_log(path, ["added_g", "total_g", "tau_Nm_cos_corrected",
                               "iq_A", "iq_std_A", "arm_tilt_deg"])
    with _f as f:
        for r in wt["rows"]:
            wcsv.writerow([f"{r[0]:.1f}", f"{r[1]:.1f}", f"{r[2]:.5f}",
                           f"{r[3]:.4f}", f"{r[4]:.4f}", f"{r[5]:.2f}"])
        wcsv.writerow([]); wcsv.writerow(["arm_effective_g", w_arm.get().strip()])
        wcsv.writerow(["lever_L_m", w_len.get().strip()])
        wcsv.writerow([]); wcsv.writerow(["kt_mNm_per_A", w_res.get("kt", "")])
    w_stop()
    w_set(f"Saved {len(wt['rows'])} rows → {path}", GREEN)


def w_undo():
    if not wt["rows"]:
        w_set("Nothing to undo", AMBER); return
    r = wt["rows"].pop()
    drop_last_row(w_table)
    w_fit()
    w_set(f"Removed the last step ({r[0]:.0f} g at {r[3]:.3f} A)", BLUE)


def w_reset():
    if not confirm_reset(len(wt["rows"]), "recorded steps"):
        return
    wt["rows"].clear()
    clear_table(w_table)
    w_fit()
    w_set("Data reset — add weights and record again", BLUE)


_wb = tk.Frame(w_l, bg=SURFACE); _wb.pack(fill="x", pady=(6, 0))
btn(_wb, "⌫ Undo last", FIELD, MUTED, w_undo).pack(side="left", padx=(0, 6))
btn(_wb, "↺ Reset data", FIELD, RED, w_reset).pack(side="left")

btn(w_l, "⏹ END — save log & release", FIELD, GREEN, w_end).pack(fill="x", pady=(6, 0))
w_set = mkstatus(w_l)

w_m_o, w_m = card(w_fr); w_m_o.pack(side="left", fill="y", padx=(0, 14))
section(w_m, "Live · holding current")
w_iq = big_value(w_m, "Iq (A, 1 s robust average)", BLUE)
w_state = tk.Label(w_m, text="—", bg=SURFACE, fg=MUTED, font=F_MONO)
w_state.pack(pady=(8, 0))
w_ang = tk.Label(w_m, text="", bg=SURFACE, fg=FAINT, font=F_MONO)
w_ang.pack(pady=(4, 0))

w_rsc_o, w_rsc = scroll_column(w_fr); w_rsc_o.pack(side="left", fill="y")
w_r_o, w_r = card(w_rsc); w_r_o.pack(fill="both", expand=True)
section(w_r, "Steps")
w_table = mktable(w_r, [("g", "added g", 70), ("t", "τ mNm", 80),
                        ("i", "Iq A", 80), ("s", "std", 60),
                        ("a", "tilt", 60)], height=10)
section(w_r, "Result · τ = Kt·Iq + τf")
w_lbl_kt = tk.Label(w_r, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
w_lbl_kt.pack()
tk.Label(w_r, text="Kt [mNm/A]  (motor)", bg=SURFACE, fg=FAINT, font=F_SM).pack()
w_lbl_r2 = tk.Label(w_r, text="record ≥ 2 steps", bg=SURFACE, fg=FAINT, font=F_LBL)
w_lbl_r2.pack(pady=(6, 0))
section(w_r, "τ = Kt·Iq + τf")
w_scatter = tk.Canvas(w_r, width=340, height=200, bg=SURFACE, highlightthickness=0)
w_scatter.pack(fill="x", pady=(0, 4))
w_res = {}


def w_fit():
    xs = [r[3] for r in wt["rows"]]; ys = [r[2] * 1000 for r in wt["rows"]]
    if len(wt["rows"]) < 2:
        # a stale Kt left on screen after an undo or reset is worse than none
        w_lbl_kt.config(text="—")
        w_lbl_r2.config(text="record ≥ 2 steps")
        w_res.pop("kt", None)
        draw_scatter(w_scatter, xs, ys, None, "Iq (A)", "τ (mN·m)")
        return
    fit = linfit([r[3] for r in wt["rows"]], [r[2] for r in wt["rows"]])
    if fit is None:
        return
    draw_scatter(w_scatter, xs, ys, (fit[0] * 1000, fit[1] * 1000, fit[2]),
                 "Iq (A)", "τ (mN·m)")
    a, b, r2 = fit
    w_res["kt"] = f"{a*1000:.2f}"
    w_lbl_kt.config(text=f"{a*1000:.1f}")
    txt = f"R² = {r2:.4f} · friction {b*1000:+.1f} mNm · {len(wt['rows'])} pts"
    if b < -0.005:
        # tau_gravity = Kt*Iq + tau_friction. Friction OPPOSES the sag, so it
        # helps hold the arm up and the intercept must be POSITIVE. A negative
        # one means the motor is fighting more torque than the entered masses
        # account for — almost always an under-counted arm weight.
        miss = -b / (1e-3 * G_ACC * max(float(w_len.get() or 0.1), 1e-6))
        txt += f"  ⚠ negative friction is impossible — arm weight looks ~{miss:.0f} g light"
        w_lbl_r2.config(text=txt, fg=RED)
    else:
        w_lbl_r2.config(text=txt, fg=FAINT)


add_tab("3 · KT · Weights", w_fr)


# ═════════════ step-test engine (tabs 3 & 5) ═══════════════════
def build_step_tab(exp_name, title_note, gear_default, with_eff):
    fr = tk.Frame(body, bg=BG)
    st = dict(rows=[], run=False, marks=[], breakaway=None)

    sc_o, sc = scroll_column(fr); sc_o.pack(side="left", fill="y", padx=(0, 14))
    l_o, l = card(sc); l_o.pack(fill="both", expand=True)
    section(l, f"Setup · {title_note}")
    e_i0 = entry_row(l, "Iq start (A)", 0.5)
    e_i1 = entry_row(l, "Iq end (A)", 3.0)
    e_ns = entry_row(l, "Number of steps", 6)
    e_ts = entry_row(l, "Step time (s)", 2.0)
    e_st = entry_row(l, "Settle discard (s)", 0.7)
    e_en = entry_row(l, "Engage current (A)", 0.30)
    e_rp = entry_row(l, "Current ramp (A/s)", 2.0)
    # [g2-B] contact detection replaces the old blind fixed-time engage
    e_cg = entry_row(l, "Contact threshold (g)", 15.0)
    e_em = entry_row(l, "Engage max (A)", 1.5)
    e_lv = entry_row(l, "Loadcell lever arm (m)", 0.10)
    e_gr = entry_row(l, "Gear ratio (out/motor)", gear_default)
    e_mk = entry_row(l, "Motor Kt for efficiency (mNm/A)", 83.7) if with_eff else None
    set_ = mkstatus(l)

    def runner():
        try:
            i0 = float(e_i0.get()); i1 = float(e_i1.get()); n = max(2, int(e_ns.get()))
            ts = max(0.8, float(e_ts.get())); settle = min(ts - 0.3, max(0.1, float(e_st.get())))
            lever = float(e_lv.get())
            i_eng = max(0.02, float(e_en.get())); ramp = max(0.1, float(e_rp.get()))
            thresh_g = max(1.0, float(e_cg.get()))
            i_engmax = max(i_eng, float(e_em.get()))
        except ValueError:
            set_("Check the numbers", AMBER); st["run"] = False; return
        if not arm_alive():
            set_("No sensor board — connect it in the header (it carries the "
                 "load cell and the IMU)", RED); st["run"] = False; return
        if not dev_alive():
            set_("No telemetry from the driver — connect it first", RED); st["run"] = False; return
        if T["flags"] > 0 and not (T["flags"] & 0x01):
            set_("Firmware reports initFOC FAILED — the motor cannot move. See Diagnostics.", RED)
            st["run"] = False; return

        st["rows"].clear(); st["marks"].clear()
        for it in table.get_children():
            table.delete(it)

        set_("Taring — arm should be RESTING on the loadcell", BLUE)
        dev_send(f"Pr{ramp:.2f}")
        t0 = time.time(); time.sleep(0.8)
        z, _ = robust_mean([g for (_, g, _) in lc_window(t0, time.time())])
        z = z or 0.0

        # ENGAGE: make first contact ONCE, gently — then never leave contact.
        # The current is RAMPED until the cell responds, so a stiction breakaway
        # higher than the entered engage current no longer means "no contact".
        st["marks"].append(time.time())
        ok, i_contact = engage_until_contact(
            z, thresh_g, i_eng, i_engmax, 0.1, 1.2,
            alive=lambda: st["run"],
            report=lambda i: root.after(0, lambda i=i: set_(
                f"Engaging — {i:.2f} A, waiting for contact…", BLUE)))
        if not ok:
            dev_send("C0"); stop_all(); st["run"] = False
            m, _ = iq_avg(1.0)
            hint = ("the current sense reads ~0 as well — check the Diagnostics tab"
                    if (m is None or abs(m) < 0.1) else
                    "current IS flowing, so the arm is very likely pushing the WRONG WAY "
                    "— flip the direction (Ps) on the Diagnostics tab")
            root.after(0, lambda: set_(
                f"NO CONTACT up to {i_engmax:.2f} A: {hint}.", RED))
            return
        st["breakaway"] = i_contact
        root.after(0, lambda: set_(f"Contact at {i_contact:.2f} A (breakaway) — running steps", GREEN))

        # never sweep below the current that was needed to make contact: those
        # points sit in the friction dead-zone and only bias the fit
        i0 = max(i0, i_contact)
        currents = [i0 + (i1 - i0) * i / (n - 1) for i in range(n)]
        prev = i_contact
        for k, iq in enumerate(currents):
            if not st["run"]:
                break
            dev_send(f"C{iq:.3f}")
            t_step = time.time(); st["marks"].append(t_step)
            settle_eff = max(settle, abs(iq - prev) / ramp + 0.4)
            settle_eff = min(settle_eff, ts - 0.2)
            prev = iq
            time.sleep(settle_eff)                   # discard: force not settled
            t_use0 = time.time()
            time.sleep(max(0.05, ts - settle_eff))
            samples = lc_window(t_use0, time.time())
            gm, gs = robust_mean([g for (_, g, _) in samples])
            im, _ = iq_avg(ts - settle)
            used_cmd = False
            if im is None or (abs(im) < 0.25 * iq and iq >= 0.1):
                # measured current path dead — fit with the commanded value
                icm, _ = iqc_avg(ts - settle)
                im = icm if icm is not None else iq
                used_cmd = True
                root.after(0, lambda: set_(
                    "Current sense reads ~0 — using COMMANDED Iq for the fit. "
                    "Run the self-test (Z) on the Diagnostics tab.", AMBER))
            if gm is None:
                root.after(0, lambda k=k: set_(f"Step {k+1}: no loadcell data", RED))
                continue
            grams = gm - z
            tau = grams * 1e-3 * G_ACC * lever
            st["rows"].append((iq, abs(im or iq), grams, gs or 0, tau, used_cmd))
            root.after(0, lambda r=st["rows"][-1]:
                       table.insert("", "end", values=(f"{r[0]:.2f}", f"{r[1]:.3f}",
                                    f"{r[2]:.1f}", f"±{r[3]:.1f}", f"{r[4]*1000:.1f}")))
            root.after(0, fit)
            if not used_cmd:
                root.after(0, lambda k=k, n=n: set_(f"Step {k+1}/{n} done", BLUE))
        dev_send("C0"); stop_all()
        st["run"] = False
        root.after(0, save)

    def start():
        if st["run"]:
            return
        st["run"] = True
        threading.Thread(target=runner, daemon=True).start()

    def stop():
        st["run"] = False
        dev_send("C0"); stop_all()
        set_("Stopped", RED)

    def fit():
        xs = [r[1] for r in st["rows"]]; ys = [r[4] * 1000 for r in st["rows"]]
        if len(st["rows"]) < 2:
            lbl_kt.config(text="—")
            lbl_r2.config(text="run ≥ 2 steps")
            draw_scatter(scatter, xs, ys, None, "Iq meas (A)", "τ (mN·m)")
            return
        f = linfit([r[1] for r in st["rows"]], [r[4] for r in st["rows"]])
        if f is None:
            return
        draw_scatter(scatter, xs, ys, (f[0] * 1000, f[1] * 1000, f[2]),
                     "Iq meas (A)", "τ (mN·m)")
        a, b, r2 = f
        lbl_kt.config(text=f"{a*1000:.1f}")
        extra = f"R² = {r2:.4f} · friction {b*1000:+.1f} mNm"
        if st.get("breakaway"):
            extra += f" · breakaway {st['breakaway']:.2f} A"
        if any(r[5] for r in st["rows"]):
            extra += " · ⚠ COMMANDED Iq used"
        if with_eff and e_mk is not None:
            try:
                ktm = float(e_mk.get()) * 1e-3
                N = float(e_gr.get())
                eff = a / (ktm * N) * 100.0
                extra += f" · efficiency {eff:.1f}%"
            except ValueError:
                pass
        lbl_r2.config(text=extra)

    def save():
        if not st["rows"]:
            return
        path = log_path(exp_name)
        if not path:
            set_("Enter the motor serial number first", RED); return
        _f, w = open_log(path, ["iq_cmd_A", "iq_meas_A", "grams", "grams_std",
                                "tau_Nm", "iq_from_command"])
        with _f as f:
            for r in st["rows"]:
                w.writerow([f"{r[0]:.3f}", f"{r[1]:.4f}", f"{r[2]:.2f}",
                            f"{r[3]:.2f}", f"{r[4]:.5f}", int(r[5])])
            w.writerow([]); w.writerow(["kt_out_mNm_per_A", lbl_kt.cget("text")])
        set_(f"Done — saved → {path}", GREEN)

    def undo():
        if st["run"]:
            set_("Stop the sweep first", AMBER); return
        if not st["rows"]:
            set_("Nothing to undo", AMBER); return
        r = st["rows"].pop()
        drop_last_row(table)
        fit()
        set_(f"Removed the last step ({r[0]:.2f} A, {r[2]:.1f} g)", BLUE)

    def reset():
        if st["run"]:
            set_("Stop the sweep first", AMBER); return
        if not confirm_reset(len(st["rows"]), "steps"):
            return
        st["rows"].clear(); st["marks"].clear(); st["breakaway"] = None
        clear_table(table)
        fit()
        set_("Data reset — press RUN to record again", BLUE)

    bb = tk.Frame(l, bg=SURFACE); bb.pack(pady=(8, 2))
    btn(bb, "▶ RUN", GREEN, "white", start, width=8).pack(side="left", padx=(0, 6))
    btn(bb, "■ STOP", RED, "white", stop, width=8).pack(side="left")
    bb2 = tk.Frame(l, bg=SURFACE); bb2.pack(fill="x", pady=(4, 2))
    btn(bb2, "⌫ Undo last", FIELD, MUTED, undo).pack(side="left", padx=(0, 6))
    btn(bb2, "↺ Reset data", FIELD, RED, reset).pack(side="left")

    m_o, m = card(fr); m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
    section(m, "Loadcell force (blue) · measured Iq (purple, own scale) · dashed = steps")
    cv = tk.Canvas(m, width=620, height=300, bg=SURFACE, highlightthickness=0)
    cv.pack(fill="both", expand=True, pady=4)
    live = tk.Label(m, text="", bg=SURFACE, fg=MUTED, font=F_MONO)
    live.pack()

    def tick():
        alive = sense_alive()
        y2 = snap_hist(iq_hist) if alive else snap_hist(iqc_hist)
        lab = ("force vs current — both as MAGNITUDE, so they rise together"
               if alive else
               "force vs COMMANDED current (magnitude) — sense reads 0!")
        _lc = snap_lc()
        draw_ts(cv, [(t, g) for (t, g, _) in _lc], BLUE, lab,
                marks=st["marks"], y2=y2,
                unit1="g", unit2="A",
                label1="|force|", label2=("|Iq| meas" if alive else "|Iq| CMD"))
        s = _lc[-1] if _lc else None
        base = f"loadcell {s[1]:.1f} g   " if s else "no loadcell data   "
        live.config(text=base + f"Iq meas {T['iq']:+.2f} / cmd {T['iqc']:+.2f} A   "
                                f"[{STATE_NAMES.get(T['state'], '?')}]",
                    fg=MUTED if alive else RED)

    safe_tick(tick, 150)()

    rsc_o, rsc = scroll_column(fr); rsc_o.pack(side="left", fill="y")
    r_o, r = card(rsc); r_o.pack(fill="both", expand=True)
    section(r, "Steps")
    table = mktable(r, [("c", "Iq cmd", 70), ("i", "Iq meas", 80), ("g", "grams", 80),
                        ("s", "std", 65), ("t", "τ mNm", 85)], height=10)
    section(r, "Result · τ = Kt·Iq")
    lbl_kt = tk.Label(r, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
    lbl_kt.pack()
    tk.Label(r, text=("Kt_out [mNm/A] (after capstan)" if with_eff else "Kt [mNm/A] (motor)"),
             bg=SURFACE, fg=FAINT, font=F_SM).pack()
    lbl_r2 = tk.Label(r, text="run ≥ 2 steps", bg=SURFACE, fg=FAINT, font=F_LBL)
    lbl_r2.pack(pady=(6, 0))
    section(r, "τ = Kt·Iq")
    scatter = tk.Canvas(r, width=340, height=200, bg=SURFACE, highlightthickness=0)
    scatter.pack(fill="x", pady=(0, 4))
    draw_scatter(scatter, [], [], None, "Iq meas (A)", "τ (mN·m)")
    return fr


add_tab("4 · KT · Loadcell", build_step_tab("kt-loadcell", "motor direct → loadcell", 1.0, False))

# ═════════════ TAB 4 · Heat over force ═════════════════════════
h_fr = tk.Frame(body, bg=BG)
ht = dict(run=False, rows=[], t0=0.0, file=None, wr=None, marks=[], overt=False)

h_sc_o, h_sc = scroll_column(h_fr); h_sc_o.pack(side="left", fill="y", padx=(0, 14))
h_l_o, h_l = card(h_sc); h_l_o.pack(fill="both", expand=True)
section(h_l, "Setup · continuous-current thermal test")
h_iq = entry_row(h_l, "Test Iq (A)", 2.0)
h_lim = entry_row(h_l, "High-temp protection (°C)", 80.0)
h_lv = entry_row(h_l, "Loadcell lever arm (m)", 0.10)
h_set = mkstatus(h_l)


def h_runner():
    try:
        iq = float(h_iq.get()); tlim = float(h_lim.get()); lever = float(h_lv.get())
    except ValueError:
        h_set("Check the numbers", AMBER); ht["run"] = False; return
    if arm["ser"] is None:
        h_set("Connect the sensor board first", RED); ht["run"] = False; return
    if not dev_alive():
        h_set("No telemetry from the driver — connect it first", RED); ht["run"] = False; return
    path = log_path("heat-force")
    if not path:
        h_set("Enter the motor serial number first", RED); ht["run"] = False; return
    ht["rows"].clear(); ht["marks"] = [time.time()]
    ht["file"], ht["wr"] = open_log(
        path, ["t_s", "iq_meas_A", "grams", "tau_Nm", "temp_C", "note"])
    dev_send(f"C{iq:.3f}")
    ht["t0"] = time.time()
    h_set(f"Running at {iq:.2f} A — logging 1 Hz → {os.path.basename(path)}", BLUE)
    while ht["run"]:
        time.sleep(1.0)
        now = time.time()
        win = lc_window(now - 1.0, now)
        gm, _ = robust_mean([g for (_, g, _) in win])
        # [g3-A] temperature now prefers the driver board's own thermistor and
        # falls back to the load-cell Arduino's, instead of only ever reading
        # the latter.
        _tv = [c for (_, c) in temp_window(now - 1.0, now)]
        tm, _ = robust_mean(_tv) if _tv else (None, None)
        im, _ = iq_avg(1.0)
        tau = (gm or 0.0) * 1e-3 * G_ACC * lever
        note = ""
        if tm is not None and tm >= tlim:
            note = "OVERTEMP-STOP"
        # a watchdog trip or fault mid-run must not be logged as valid data
        if T["state"] != 3:
            note = note or "FIRMWARE-LEFT-CURRENT-MODE"
        row = (now - ht["t0"], abs(im or iq), gm or 0.0, tau, tm)
        ht["rows"].append(row)
        ht["wr"].writerow([f"{row[0]:.1f}", f"{row[1]:.3f}", f"{row[2]:.1f}",
                           f"{row[3]:.5f}", "" if tm is None else f"{tm:.1f}", note])
        ht["file"].flush()
        root.after(0, lambda r=row: h_live_row(r))
        if note == "OVERTEMP-STOP":
            ht["overt"] = True
            root.after(0, lambda tm=tm: h_set(
                f"OVERTEMP {tm:.1f}°C ≥ {tlim:.0f}°C — stopped, log saved", RED))
            break
        if note == "FIRMWARE-LEFT-CURRENT-MODE":
            ht["overt"] = True
            root.after(0, lambda: h_set(
                "Firmware left CURRENT mode mid-test (watchdog or fault) — "
                "stopped. See the Diagnostics console.", RED))
            break
    dev_send("C0"); stop_all()
    ht["run"] = False
    try:
        ht["file"].close()
    except Exception:
        pass
    if not ht["overt"]:
        root.after(0, lambda: h_set(f"Saved → {path}", GREEN))
    ht["overt"] = False


def h_start():
    if ht["run"]:
        return
    ht["run"] = True
    threading.Thread(target=h_runner, daemon=True).start()


def h_stop():
    ht["run"] = False


def h_reset():
    if ht["run"]:
        h_set("Stop the run first", AMBER); return
    if not confirm_reset(len(ht["rows"]), "logged samples"):
        return
    ht["rows"].clear(); ht["marks"] = []
    clear_table(h_table)
    h_set("Data reset — the saved CSV from the last run is untouched", BLUE)


hb3 = tk.Frame(h_l, bg=SURFACE); hb3.pack(pady=(8, 2))
btn(hb3, "▶ RUN", GREEN, "white", h_start, width=8).pack(side="left", padx=(0, 6))
btn(hb3, "■ STOP", RED, "white", h_stop, width=8).pack(side="left")
_hb = tk.Frame(h_l, bg=SURFACE); _hb.pack(fill="x", pady=(4, 2))
btn(_hb, "↺ Reset data", FIELD, RED, h_reset).pack(side="left")
tk.Label(h_l, text="Goal: find the max Iq the motor holds\ncontinuously without overheating — that\nbecomes ExArMo's working current limit.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(10, 0))

h_m_o, h_m = card(h_fr); h_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(h_m, "Temperature (purple) · force (blue)")
h_cv = tk.Canvas(h_m, width=640, height=320, bg=SURFACE, highlightthickness=0)
h_cv.pack(fill="both", expand=True, pady=4)
h_live = tk.Label(h_m, text="", bg=SURFACE, fg=MUTED, font=F_MONO)
h_live.pack()


def h_tick():
    _lc = snap_lc()
    _now = time.time()
    draw_ts(h_cv, [(t, g) for (t, g, _) in _lc], BLUE, "force and temperature",
            marks=ht["marks"], y2=temp_window(_now - 60.0, _now),
            unit1="g", unit2="°C", label1="|force|", label2="temp")
    s = _lc[-1] if _lc else None
    tc = temp_now(2.0)
    if s:
        h_live.config(text=f"{s[1]:.1f} g   {('%.1f' % tc) if tc is not None else '--'} °C "
                           f"({temp_label()})   Iq {T['iq']:+.2f} A")


h_rsc_o, h_rsc = scroll_column(h_fr); h_rsc_o.pack(side="left", fill="y")
h_r_o, h_r = card(h_rsc); h_r_o.pack(fill="both", expand=True)
section(h_r, "Log (1 Hz)")
h_table = mktable(h_r, [("t", "t s", 60), ("i", "Iq A", 70), ("g", "grams", 75),
                        ("tau", "τ mNm", 80), ("T", "°C", 60)], height=14)


def h_live_row(r):
    h_table.insert("", "end", values=(f"{r[0]:.0f}", f"{r[1]:.2f}", f"{r[2]:.0f}",
                                      f"{r[3]*1000:.1f}", "--" if r[4] is None else f"{r[4]:.1f}"))
    kids = h_table.get_children()
    if kids:
        h_table.see(kids[-1])


add_tab("5 · Heat / Force", h_fr)

# ═════════════ TAB 5 · Capstan torque ══════════════════════════
add_tab("6 · Capstan · Torque", build_step_tab("capstan-torque",
        "motor → capstan → loadcell (output side)", 9.0, True))

# ═════════════ TAB 6 · Thermal rise  [g3-A] ════════════════════
# WHAT THIS MEASURES, AND WHY IT IS NOT THE HEAT/FORCE TAB.
# Tab 5 answers "does it survive N amps" by watching a number climb toward a
# limit. This tab answers the question that actually sets ExArMo's continuous
# current rating: how the motor's temperature RESPONDS to a known dissipation.
# Hold one current, log the curve, and fit a first-order thermal model
#
#       T(t) = T_amb + dT_inf * (1 - exp(-t/tau))
#
# from which R_th = dT_inf / P_cu and the continuous current for any allowed
# winding rise falls straight out. Copper loss for three phases driven by FOC
# is P_cu = 1.5 * Iq^2 * R_phase — it is 3/2 and not 3 because Iq is the peak
# of the phase current, not its RMS, and forgetting that overstates the loss
# by a factor of two and halves the R_th you report.
#
# THE FIT DOES NOT NEED THE RUN TO REACH STEADY STATE. Waiting for a real
# plateau on a 5008 takes 30-45 minutes and everybody stops early; reading
# dT_inf off a curve that was still climbing then understates it badly.
# Differentiating the model gives
#
#       dT/dt = (T_inf - T)/tau  =  (-1/tau)*T + T_inf/tau
#
# which is a straight line in (T, dT/dt) — so a least-squares fit of the
# SLOPE against the TEMPERATURE recovers both tau and T_inf from the transient
# alone. r^2 is reported because that line is also the honest warning when the
# data is too noisy or too short to support the extrapolation.
th_fr = tk.Frame(body, bg=BG)
th = dict(run=False, rows=[], t0=0.0, file=None, wr=None, marks=[],
          phase="", stopped="", src="", fit=None)

th_sc_o, th_sc = scroll_column(th_fr); th_sc_o.pack(side="left", fill="y", padx=(0, 14))
th_l_o, th_l = card(th_sc); th_l_o.pack(fill="both", expand=True)
section(th_l, "Setup · thermal step response")
th_iq   = entry_row(th_l, "Test Iq (A)", 2.0)
th_heat = entry_row(th_l, "Heat for (min)", 20.0)
th_cool = entry_row(th_l, "Then cool for (min)", 10.0)
th_per  = entry_row(th_l, "Log every (s)", 2.0)
th_lim  = entry_row(th_l, "Abort at (°C)", 80.0)
th_res  = entry_row(th_l, "Phase resistance R (Ω)", 0.54)
th_rise = entry_row(th_l, "Allowed rise for rating (K)", 60.0)
th_set = mkstatus(th_l)


def th_pcu(iq, r):
    """Copper loss [W] for a FOC drive commanding peak q-axis current iq."""
    return 1.5 * iq * iq * r


def th_runner():
    try:
        iq = float(th_iq.get()); heat_s = float(th_heat.get()) * 60.0
        cool_s = float(th_cool.get()) * 60.0; per = float(th_per.get())
        tlim = float(th_lim.get())
    except ValueError:
        th_set("Check the numbers", AMBER); th["run"] = False; return
    if per < 0.5:
        th_set("Log period below 0.5 s serves no purpose here", AMBER)
        th["run"] = False; return
    if not dev_alive():
        th_set("No telemetry from the driver — connect it first", RED)
        th["run"] = False; return
    src = temp_src()
    if src is None:
        th_set("No temperature source. Fit the thermistor on GPIO3/PA2, or "
               "connect the loadcell/temp Arduino.", RED)
        th["run"] = False; return
    path = log_path("thermal-rise")
    if not path:
        th_set("Enter the motor serial number first", RED); th["run"] = False; return

    t_amb = temp_now(3.0, src)
    if t_amb is None:
        th_set("Temperature source went quiet before the run started", RED)
        th["run"] = False; return

    th["src"] = src
    th["rows"].clear(); th["marks"] = [time.time()]; th["fit"] = None
    th["stopped"] = ""
    th["file"], th["wr"] = open_log(
        path, ["t_s", "phase", "iq_cmd_A", "iq_meas_A", "temp_C",
               "rise_K", "p_cu_W", "temp_source", "note"])
    p_cu = th_pcu(iq, float(th_res.get() or 0.0))

    th["t0"] = time.time()
    th["phase"] = "heat"
    dev_send(f"C{iq:.3f}")
    th_set(f"Heating at {iq:.2f} A ({p_cu:.1f} W) — ambient {t_amb:.1f} °C, "
           f"temperature from the {temp_label()}", BLUE)

    def log_sample(phase, iq_cmd):
        now = time.time()
        tm = temp_now(max(1.0, per * 0.5), th["src"])
        im, _ = iq_avg(min(2.0, per))
        note = ""
        if tm is None:
            note = "NO-TEMP-SAMPLE"
        if phase == "heat" and T["state"] != 3:
            # exactly the trap tab 5 already learned: a watchdog trip or an
            # over-temp stop mid-run leaves the motor cold-ish and the log
            # looking like a valid, gentle curve.
            note = note or "FIRMWARE-LEFT-CURRENT-MODE"
        row = (now - th["t0"], phase, iq_cmd, abs(im or 0.0), tm,
               None if tm is None else tm - t_amb)
        th["rows"].append(row)
        th["wr"].writerow([f"{row[0]:.1f}", phase, f"{iq_cmd:.3f}",
                           f"{row[3]:.3f}",
                           "" if tm is None else f"{tm:.2f}",
                           "" if tm is None else f"{row[5]:.2f}",
                           f"{p_cu:.2f}", th["src"], note])
        th["file"].flush()
        root.after(0, lambda r=row: th_live_row(r))
        return tm, note

    # ---- heating ----
    next_t = time.time()
    while th["run"] and (time.time() - th["t0"]) < heat_s:
        next_t += per
        time.sleep(max(0.0, next_t - time.time()))
        if not th["run"]:
            break
        tm, note = log_sample("heat", iq)
        if tm is not None and tm >= tlim:
            th["stopped"] = f"ABORT at {tm:.1f} °C ≥ {tlim:.0f} °C"
            break
        if note == "FIRMWARE-LEFT-CURRENT-MODE":
            th["stopped"] = ("the firmware left CURRENT mode mid-run "
                             "(watchdog, over-temp or fault) — see Diagnostics")
            break

    # ---- cooling ----
    # The cool-down is not padding. It is the SAME time constant measured with
    # the heat source switched off, so it is an independent check on tau that
    # does not depend on knowing R_phase or the current at all.
    dev_send("C0"); stop_all()
    if th["run"] and cool_s > 0 and not th["stopped"]:
        th["phase"] = "cool"
        th["marks"].append(time.time())
        root.after(0, lambda: th_set(
            f"Cooling — logging for {cool_s/60.0:.0f} more min "
            f"(this gives a second, independent tau)", BLUE))
        t_cool0 = time.time()
        next_t = time.time()
        while th["run"] and (time.time() - t_cool0) < cool_s:
            next_t += per
            time.sleep(max(0.0, next_t - time.time()))
            if not th["run"]:
                break
            log_sample("cool", 0.0)

    th["run"] = False
    th["phase"] = ""
    try:
        th["file"].close()
    except Exception:
        pass
    msg = th["stopped"]
    root.after(0, lambda: th_set(
        (f"{msg} — log saved → {os.path.basename(path)}" if msg
         else f"Done — {len(th['rows'])} samples → {os.path.basename(path)}. Press FIT."),
        RED if msg else GREEN))
    root.after(0, th_fit)


def th_start():
    if th["run"]:
        return
    th["run"] = True
    threading.Thread(target=th_runner, daemon=True).start()


def th_stop():
    th["run"] = False
    dev_send("C0"); stop_all()


def th_reset():
    if th["run"]:
        th_set("Stop the run first", AMBER); return
    if not confirm_reset(len(th["rows"]), "logged samples"):
        return
    th["rows"].clear(); th["marks"] = []; th["fit"] = None
    clear_table(th_table)
    for _l in th_out.values():
        _l.config(text="—")
    th_set("Data reset — the saved CSV from the last run is untouched", BLUE)


thb = tk.Frame(th_l, bg=SURFACE); thb.pack(pady=(8, 2))
btn(thb, "▶ RUN", GREEN, "white", th_start, width=8).pack(side="left", padx=(0, 6))
btn(thb, "■ STOP", RED, "white", th_stop, width=8).pack(side="left")
_thb = tk.Frame(th_l, bg=SURFACE); _thb.pack(fill="x", pady=(4, 2))
btn(_thb, "∿ Fit thermal model", FIELD, BLUE, lambda: th_fit()).pack(side="left")
btn(_thb, "↺ Reset", FIELD, RED, th_reset).pack(side="left", padx=(6, 0))
tk.Label(th_l, text="Motor at rest, still air, bead on the stator end-winding\n"
                    "or the can. The number this produces is only as good as\n"
                    "where that bead is — record it in the logbook.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(10, 0))

# ── middle: live chart ──
th_m_o, th_m = card(th_fr); th_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(th_m, "Temperature (purple) · Iq (blue)")
th_cv = tk.Canvas(th_m, width=640, height=300, bg=SURFACE, highlightthickness=0)
th_cv.pack(fill="both", expand=True, pady=4)
th_live = tk.Label(th_m, text="", bg=SURFACE, fg=MUTED, font=F_MONO)
th_live.pack()

th_res_f = tk.Frame(th_m, bg=SURFACE); th_res_f.pack(fill="x", pady=(10, 0))
th_out = {}
for _key, _cap in (("tau", "τ  (min)"), ("dinf", "ΔT∞  (K)"),
                   ("rth", "R_th  (K/W)"), ("icont", "I cont  (A)")):
    _c = tk.Frame(th_res_f, bg=SURFACE); _c.pack(side="left", expand=True)
    th_out[_key] = big_value(_c, _cap)
th_fit_note = tk.Label(th_m, text="", bg=SURFACE, fg=FAINT, font=F_SM,
                       justify="left", wraplength=620)
th_fit_note.pack(anchor="w", pady=(6, 0))


def th_fit():
    """Fit tau and T_inf from the HEATING transient (see the note above), and
    tau again from the COOL-DOWN as an independent check."""
    rows = [r for r in th["rows"] if r[1] == "heat" and r[4] is not None]
    if len(rows) < 8:
        th_fit_note.config(text="Need at least 8 heating samples with a "
                                "temperature before anything can be fitted.",
                           fg=AMBER)
        return
    ts = [r[0] for r in rows]
    cs = [r[4] for r in rows]
    f = expfit(ts, cs)
    if not f or f[2] >= 0.0:
        # B >= 0 while heating means the curve is not rising toward a limit.
        th_fit_note.config(
            text="The temperature is not rising toward a limit in this data — "
                 "either the run is far too short, or nothing is heating up. "
                 "No τ can be extracted.", fg=AMBER)
        return
    tau, t_inf, _B, r2 = f
    t_amb = cs[0] - (rows[0][5] or 0.0)     # the rise is logged, so this
    d_inf = t_inf - t_amb                   # recovers the ambient exactly
    try:
        iq = float(th_iq.get()); rph = float(th_res.get())
        allow = float(th_rise.get())
    except ValueError:
        iq = rph = allow = 0.0
    p_cu = th_pcu(iq, rph)
    rth = d_inf / p_cu if p_cu > 1e-9 else None
    icont = (math.sqrt(allow / (1.5 * rph * rth))
             if (rth and rth > 1e-9 and rph > 1e-9 and allow > 0) else None)

    # cool-down: same tau, measured with the heat source off, so it depends on
    # neither R_phase nor the current. If the two disagree badly, the bead is
    # reading its own mounting rather than the motor.
    crows = [r for r in th["rows"] if r[1] == "cool" and r[4] is not None]
    cf = expfit([r[0] for r in crows], [r[4] for r in crows]) if len(crows) >= 8 else None
    tau_c = cf[0] if cf else None

    th["fit"] = dict(tau=tau, t_inf=t_inf, d_inf=d_inf, r2=r2, rth=rth,
                     icont=icont, p_cu=p_cu, t_amb=t_amb, tau_cool=tau_c)

    th_out["tau"].config(text=f"{tau/60.0:.1f}")
    th_out["dinf"].config(text=f"{d_inf:.1f}")
    th_out["rth"].config(text="—" if rth is None else f"{rth:.2f}")
    th_out["icont"].config(text="—" if icont is None else f"{icont:.2f}")

    reached = (max(cs) - t_amb) / d_inf if d_inf > 1e-6 else 0.0
    warn = ""
    if r2 < 0.95:
        warn = ("  ⚠ r² is low for a first-order fit: the curve is noisy or is "
                "not a single exponential. Check the bead is actually attached "
                "to the motor and not swinging in the air.")
    elif reached < 0.4:
        warn = (f"  ⚠ the run only reached {reached*100:.0f}% of the fitted "
                f"ΔT∞ — a long extrapolation. Treat ΔT∞ and R_th as indicative "
                f"and re-run for at least 2τ ({2*tau/60.0:.0f} min).")
    elif tau_c is not None and (tau_c > 1.6 * tau or tau_c < 0.6 * tau):
        warn = (f"  ⚠ the cool-down gives τ = {tau_c/60.0:.1f} min against "
                f"{tau/60.0:.1f} min heating. One body does not have two time "
                f"constants — the sensor is probably reading its own mount.")
    cool_txt = "" if tau_c is None else f"  ·  τ from cool-down = {tau_c/60.0:.1f} min"
    th_fit_note.config(
        text=(f"T(t) = {t_amb:.1f} + {d_inf:.1f}·(1 − e^(−t/{tau/60.0:.1f} min)) °C   "
              f"·  r² = {r2:.4f}  ·  P_cu = {p_cu:.1f} W at {iq:.2f} A "
              f"(1.5·Iq²·R){cool_txt}  ·  source: {th['src'] or temp_label()}" + warn),
        fg=AMBER if warn else MUTED)


# ── right: sample table ──
th_rsc_o, th_rsc = scroll_column(th_fr); th_rsc_o.pack(side="left", fill="y")
th_r_o, th_r = card(th_rsc); th_r_o.pack(fill="both", expand=True)
section(th_r, "Log")
th_table = mktable(th_r, [("t", "t min", 62), ("ph", "phase", 58),
                          ("i", "Iq A", 62), ("T", "°C", 62),
                          ("d", "rise K", 66)], height=16)


def th_live_row(r):
    th_table.insert("", "end", values=(
        f"{r[0]/60.0:.1f}", r[1], f"{r[3]:.2f}",
        "--" if r[4] is None else f"{r[4]:.1f}",
        "--" if r[5] is None else f"{r[5]:+.1f}"))
    kids = th_table.get_children()
    if kids:
        th_table.see(kids[-1])


def th_tick():
    now = time.time()
    draw_ts(th_cv, [(t, i) for (t, i) in snap_hist(iq_hist) if t >= now - 60],
            BLUE, "Iq and motor temperature", marks=th["marks"],
            y2=temp_window(now - 60.0, now), unit1="A", unit2="°C",
            label1="|Iq|", label2="temp")
    tc = temp_now(2.0)
    el = (now - th["t0"]) / 60.0 if th["run"] else 0.0
    th_live.config(
        text=(f"{('%.1f' % tc) if tc is not None else '--'} °C   "
              f"Iq {T['iq']:+.2f} A   "
              f"{(th['phase'] + ' ' + ('%.1f' % el) + ' min') if th['run'] else temp_label()}"))


add_tab("7 · Thermal · Rise", th_fr)


# ═════════════ TAB 8 · ARM · ANGLE & FORCE [g4-A] ══════════════
# The rehab-arm bench test: one ESP32 reporting joint ANGLE (IMU) and the
# force the arm makes against the load cell, on ONE clock, so the two can
# honestly be plotted against each other.
#
# Three things happen in this tab and they are deliberately separate:
#   LIVE      — what the rig is doing right now, for setting it up.
#   LOG       — every sample the board sends, streamed to CSV, for the record.
#   POINTS    — an averaged capture at a pose you chose, for the fit.
# A continuous log is the honest record of a motion; a captured point is a
# statement that the rig was HELD somewhere and settled. Fitting the first
# would weight the fit by how long you happened to dwell at each angle, which
# is not a property of the arm, so the fit below only ever uses the points.
arm_fr = tk.Frame(body, bg=BG)
ar = dict(rows=[], f=None, wr=None, path="", n=0, t0=0.0, last_t=0.0,
          fit=None, mode="cos")


def _n(v, fmt="{:+.1f}"):
    return "--" if v is None else fmt.format(v)


# ── gravity model ──────────────────────────────────────────────
def _solve3(A, y):
    """3x3 least-squares normal equations, Gauss-Jordan with partial pivot."""
    M = [list(A[i]) + [y[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-12:
            return None
        M[i], M[p] = M[p], M[i]
        for r in range(3):
            if r == i:
                continue
            f = M[r][i] / M[i][i]
            for c in range(i, 4):
                M[r][c] -= f * M[i][c]
    return [M[i][3] / M[i][i] for i in range(3)]


def cosfit(degs, ys):
    """Fit  y(θ) = a·cos θ + b·sin θ + c.
    → (a, b, c, amplitude, phase_deg, r2) or None.

    The gravity torque of a limb about a pivot is M·g·L·cos(θ − φ), which is
    this three-parameter model rewritten — and rewritten that way it is LINEAR
    in its parameters, so the fit is one exact solve with no starting guess to
    get wrong and no iteration that can fail in front of the operator.
    `amplitude` is the peak gravity torque, `phase` is where the arm hangs
    horizontal, and `c` collects everything that does not depend on angle:
    preload, a load cell that was not re-zeroed, constant friction."""
    if len(degs) < 3:
        return None
    bas = [(math.cos(math.radians(d)), math.sin(math.radians(d)), 1.0)
           for d in degs]
    A = [[sum(b[i] * b[j] for b in bas) for j in range(3)] for i in range(3)]
    rhs = [sum(b[i] * v for b, v in zip(bas, ys)) for i in range(3)]
    sol = _solve3(A, rhs)
    if sol is None:
        return None
    a, b, c = sol
    pred = [a * u + b * v + c for (u, v, _) in bas]
    my = sum(ys) / len(ys)
    sse = sum((v - p) ** 2 for v, p in zip(ys, pred))
    sst = sum((v - my) ** 2 for v in ys)
    return (a, b, c, math.hypot(a, b), math.degrees(math.atan2(b, a)),
            1.0 - sse / sst if sst > 1e-12 else 1.0)


def ar_scatter(cv, xs, ys, curve=None, xlabel="", ylabel=""):
    """Points plus a fitted CURVE (draw_scatter only draws straight lines).

    Unlike the Kt scatters the origin is NOT forced into view here: a joint
    angle of zero is wherever you pressed Zero, so anchoring the axes there
    would squeeze the data into a corner for no physical reason."""
    cv.delete("all")
    w = int(cv.winfo_width() or 340); h = int(cv.winfo_height() or 210)
    L, R, T, B = 52, 12, 16, 30
    cv.create_rectangle(1, 1, w - 1, h - 1, outline=BORDER, fill="#fbfcfe")
    pts = [(x, y) for x, y in zip(xs, ys)
           if x is not None and y is not None
           and math.isfinite(x) and math.isfinite(y)]
    if not pts:
        cv.create_text(w / 2, h / 2, text="no captured points yet",
                       fill=FAINT, font=F_SM)
        return
    xmin = min(p[0] for p in pts); xmax = max(p[0] for p in pts)
    ymin = min(p[1] for p in pts); ymax = max(p[1] for p in pts)
    cs = []
    if curve:
        cs = [(xmin + (xmax - xmin) * i / 60.0,
               curve(xmin + (xmax - xmin) * i / 60.0)) for i in range(61)]
        cs = [(x, y) for x, y in cs if y is not None and math.isfinite(y)]
        for _x, y in cs:
            ymin = min(ymin, y); ymax = max(ymax, y)
    if xmax - xmin < 1e-9: xmin -= 1.0; xmax += 1.0
    if ymax - ymin < 1e-9: ymin -= 1.0; ymax += 1.0
    px = 0.06 * (xmax - xmin); py = 0.12 * (ymax - ymin)
    xmin -= px; xmax += px; ymin -= py; ymax += py

    def X(v): return L + (v - xmin) / (xmax - xmin) * (w - L - R)
    def Y(v): return h - B - (v - ymin) / (ymax - ymin) * (h - B - T)

    cv.create_line(L, h - B, w - R, h - B, fill=BORDER)
    cv.create_line(L, T, L, h - B, fill=BORDER)
    if ymin < 0 < ymax:
        cv.create_line(L, Y(0), w - R, Y(0), fill=BORDER, dash=(3, 4))
    if xmin < 0 < xmax:
        cv.create_line(X(0), T, X(0), h - B, fill=BORDER, dash=(3, 4))
    if len(cs) > 1:
        poly = []
        for x, y in cs:
            poly += [X(x), Y(y)]
        cv.create_line(*poly, fill=GREEN, width=2)
    for x, y in pts:
        cv.create_oval(X(x) - 3.5, Y(y) - 3.5, X(x) + 3.5, Y(y) + 3.5,
                       fill=BLUE, outline="")
    cv.create_text(L - 5, Y(ymax), text=f"{ymax:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(L - 5, Y(ymin), text=f"{ymin:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(X(xmin), h - B + 5, text=f"{xmin:.3g}", fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text(w - R, h - B + 5, text=f"{xmax:.3g}", fill=MUTED, font=F_SM, anchor="ne")
    cv.create_text(L + 5, T - 4, text=ylabel, fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text((L + w - R) / 2, h - 9, text=xlabel, fill=FAINT, font=F_SM)


# ── left column · controls ─────────────────────────────────────
ar_sc_o, ar_sc = scroll_column(arm_fr); ar_sc_o.pack(side="left", fill="y", padx=(0, 14))
ar_l_o, ar_l = card(ar_sc); ar_l_o.pack(fill="both", expand=True)

section(ar_l, "Arm sensor board (ESP32 · IMU+HX711)")
tk.Label(ar_l, text="Pick its port under 'Arm IMU/Load' above.\n"
                    "115200 baud; the rate is whatever the board\n"
                    "sends and the live line below states it.\n"
                    "Grams and angles come from tab 9's\n"
                    "calibration, which is kept here and survives\n"
                    "a reset of the board. Nothing in this tab\n"
                    "touches the load cell of tabs 3–6.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
ar_calnote = tk.Label(ar_l, text="", bg=SURFACE, fg=MUTED, font=F_SM,
                      wraplength=300, justify="left", anchor="w")
ar_calnote.pack(fill="x", pady=(0, 6))


def ar_cmd(ch):
    if not arm_send(ch):
        ar_set("No arm board connected — choose its port in the header.", RED)
        return
    if ch == "t":
        ar_set("Tare sent to the BOARD. It holds that zero only until its next "
               "reset — tab 9 keeps one that lasts.", BLUE)
    else:
        ar_set("Calibrate sent to the BOARD. It uses the KNOWN_MASS_G compiled "
               "into the sketch, and forgets the result on reset — tab 9 "
               "calibrates from the raw counts instead.", BLUE)


_arb = tk.Frame(ar_l, bg=SURFACE); _arb.pack(fill="x", pady=(0, 2))
btn(_arb, "Tare  (t)", FIELD, BLUE, lambda: ar_cmd("t"), width=9).pack(side="left", padx=(0, 6))
btn(_arb, "Calibrate  (c)", FIELD, MUTED, lambda: ar_cmd("c"), width=12).pack(side="left")
ar_msg = tk.Label(ar_l, text="board says:  —", bg=SURFACE, fg=FAINT, font=F_SM,
                  wraplength=300, justify="left", anchor="w")
ar_msg.pack(fill="x", pady=(4, 0))

section(ar_l, "Geometry")
ar_axis = tk.StringVar(value="Pitch")
_ax = tk.Frame(ar_l, bg=SURFACE); _ax.pack(fill="x", pady=3)
tk.Label(_ax, text="Joint angle axis", bg=SURFACE, fg=TEXT, font=F_LBL).pack(side="left")
_axm = tk.OptionMenu(_ax, ar_axis, "Roll", "Pitch", "Yaw")
_axm.config(bg=FIELD, fg=BLUE, relief="flat", bd=0, highlightthickness=0,
            font=("Segoe UI", 12, "bold"))
_axm.pack(side="right")
ar_inv = tk.IntVar(value=0)
tk.Checkbutton(ar_l, text="invert angle sign", variable=ar_inv, bg=SURFACE,
               fg=TEXT, selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
               highlightthickness=0).pack(anchor="w")
ar_lever = entry_row(ar_l, "Load-cell lever arm (m)", 0.10)
tk.Label(ar_l, text="distance from the JOINT AXIS to where\n"
                    "the arm touches the cell — τ = force × this.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 4))


def ar_axis_key():
    return ar_axis.get()


def ar_axis_idx():
    return {"Roll": 1, "Pitch": 2, "Yaw": 3}[ar_axis.get()]


def ar_angle(a):
    """Raw axis reading → joint angle: through the calibration (zero, gain,
    folded into ±180) and then signed the way the operator says the joint
    opens. The sign is a convention of this tab; the calibration is not."""
    if a is None:
        return None
    return (-1.0 if ar_inv.get() else 1.0) * arm_angle_cal(ar_axis_key(), a)


def ar_lever_m():
    try:
        L = float(ar_lever.get())
    except ValueError:
        return None
    return L if L > 0 else None


def ar_torque(g):
    """grams at the cell → N·m about the joint."""
    L = ar_lever_m()
    return None if (g is None or L is None) else g * 1e-3 * G_ACC * L


def ar_zero_now():
    r, p, y, _g = arm_latest(1.0)
    if r is None and p is None and y is None:
        ar_set("No angle data — connect the arm board first.", RED)
        return
    for k, v in (("Roll", r), ("Pitch", p), ("Yaw", y)):
        if v is not None:
            arm_zero[k] = v
    arm_cal_save()                      # a zero that dies with the window is
                                        # a zero you will forget you changed
    note = ("  Points already captured were measured against the OLD zero — "
            "reset them if the rig moved." if ar["rows"] else "")
    ar_set(f"Angle zero set here (roll {_n(r)}°, pitch {_n(p)}°, "
           f"yaw {_n(y)}°).{note}", GREEN)


def ar_zero_clear():
    for k in arm_zero:
        arm_zero[k] = 0.0
    arm_cal_save()
    ar_set("Angle zero cleared — angles are now the IMU's own reference.", BLUE)


btn(ar_l, "⌖ Zero angle here", FIELD, MUTED, ar_zero_now).pack(fill="x", pady=(4, 0))
btn(ar_l, "clear angle zero", FIELD, FAINT, ar_zero_clear).pack(fill="x", pady=(2, 0))

section(ar_l, "Continuous log (every sample)")


def ar_log_start():
    if ar["f"] is not None:
        ar_set("Already recording.", AMBER)
        return
    path = log_path("arm-angle-force")
    if not path:
        ar_set("Type the motor serial no. in the header first — it names the file.",
               AMBER)
        return
    try:
        f = open(path, "w", newline="")
    except Exception as e:
        ar_set(f"Cannot open the log: {e}", RED)
        return
    # The provenance goes in the file, not in a notebook. Which axis was the
    # joint, where its zero was, which way positive pointed and how long the
    # lever was are all needed to read the numbers back, and none of them can
    # be recovered from the columns afterwards.
    f.write("# source: arm sensor board (ESP32 IMU + HX711); the load cell of "
            "tabs 3-6 (lc_cal) is NOT involved\n")
    f.write(arm_cal_line() + "\n")
    f.write(f"# axis={ar_axis_key()} sign={'-' if ar_inv.get() else '+'} "
            f"zero_deg={arm_zero[ar_axis_key()]:.3f} "
            f"lever_m={ar_lever_m() if ar_lever_m() is not None else float('nan')}\n")
    wr = csv.writer(f)
    wr.writerow(["t_s", "roll_deg", "pitch_deg", "yaw_deg", "joint_deg",
                 "grams", "force_N", "torque_Nm"])
    ar.update(f=f, wr=wr, path=path, n=0, t0=time.time(), last_t=time.time())
    ar_set(f"Recording → {path}", GREEN)


def ar_log_stop():
    if ar["f"] is None:
        return
    try:
        ar["f"].close()
    except Exception:
        pass
    p, n = ar["path"], ar["n"]
    ar.update(f=None, wr=None)
    ar_set(f"Saved {n} samples → {p}", GREEN)


_lb = tk.Frame(ar_l, bg=SURFACE); _lb.pack(fill="x", pady=(2, 0))
btn(_lb, "● REC", GREEN, "white", ar_log_start, width=7).pack(side="left", padx=(0, 6))
btn(_lb, "■ Stop & save", FIELD, RED, ar_log_stop).pack(side="left")
ar_logl = tk.Label(ar_l, text="not recording", bg=SURFACE, fg=FAINT, font=F_SM,
                   wraplength=300, justify="left", anchor="w")
ar_logl.pack(fill="x", pady=(4, 0))

section(ar_l, "Capture a point (hold the pose)")
ar_avg = entry_row(ar_l, "Average window (s)", 1.0)
ar_note = entry_row(ar_l, "Label for this point", "", width=10)


def ar_capture():
    try:
        win = float(ar_avg.get())
    except ValueError:
        win = 1.0
    win = min(max(win, 0.3), 20.0)
    now = time.time()
    seg = [s for s in snap_arm() if now - s[0] <= win]
    if len(seg) < 2:
        ar_set("Nothing in the averaging window — connect the board, or widen "
               "the window (the stream is only 4 Hz).", RED)
        return
    i = ar_axis_idx()
    angs = [ar_angle(s[i]) for s in seg if s[i] is not None]
    gs = [g for g in (arm_grams(s) for s in seg) if g is not None]
    if not angs:
        ar_set(f"{ar_axis_key()} is nan — the IMU is not streaming angles.", RED)
        return
    if not gs:
        ar_set("No calibrated grams: the board is sending raw counts and this "
               "GUI has no load-cell calibration yet. Do tab 9 first — zero "
               "with nothing on the cell, then one known mass.", AMBER)
        return
    # Unwrap against the first sample before averaging: a pose sitting on the
    # ±180 seam would otherwise average to roughly zero, which is the one
    # answer that is wrong at both ends.
    a0 = angs[0]
    angs = [a0 + arm_wrap(a - a0) for a in angs]
    a, a_sd = robust_mean(angs)
    g, g_sd = robust_mean(gs)
    tq = ar_torque(g)
    if tq is None:
        ar_set("Enter a positive lever arm before capturing.", AMBER)
        return
    lab = ar_note.get().strip()
    ar["rows"].append((a, g, tq, a_sd, g_sd, len(seg), lab))
    # 0.1 deg in the table: a WIT IMU does not resolve hundredths, and a
    # column that prints them only invites someone to believe them. The
    # exported CSV keeps the full precision of the average.
    ar_table.insert("", "end", values=(f"{a:+.1f}", f"{g:.1f}",
                                       f"{tq*1000:.1f}", f"{a_sd:.1f}",
                                       f"{g_sd:.1f}", lab or "—"))
    kids = ar_table.get_children()
    if kids:
        ar_table.see(kids[-1])
    ar_fit()
    warn = ""
    if a_sd > 0.5:
        warn = f"  ⚠ the angle moved {a_sd:.1f}° across the window — hold it stiller."
    if g_sd > 0.02 * max(abs(g), 1.0) + 1.0:
        warn += f"  ⚠ the force is still settling (±{g_sd:.1f} g)."
    ar_set(f"Point {len(ar['rows'])}: {a:+.2f}° · {g:.1f} g · "
           f"{tq*1000:.1f} mN·m{warn}", GREEN if not warn else AMBER)


btn(ar_l, "＋ Capture point", BLUE, "white", ar_capture).pack(fill="x", pady=(6, 0))


def ar_undo():
    if not ar["rows"]:
        return
    ar["rows"].pop()
    drop_last_row(ar_table)
    ar_fit()
    ar_set(f"Removed the last point ({len(ar['rows'])} left).", BLUE)


def ar_reset():
    if not confirm_reset(len(ar["rows"]), "captured points"):
        return
    ar["rows"].clear()
    clear_table(ar_table)
    ar_fit()
    ar_set("Points cleared.", BLUE)


def ar_export():
    if not ar["rows"]:
        ar_set("No points to export.", AMBER)
        return
    path = log_path("arm-points")
    if not path:
        ar_set("Type the motor serial no. in the header first.", AMBER)
        return
    try:
        with open(path, "w", newline="") as f:
            f.write(f"# axis={ar_axis_key()} sign={'-' if ar_inv.get() else '+'} "
                    f"zero_deg={arm_zero[ar_axis_key()]:.3f} "
                    f"lever_m={ar_lever_m()}\n")
            if ar["fit"]:
                f.write("# " + ar["fit"] + "\n")
            wr = csv.writer(f)
            wr.writerow(["joint_deg", "grams", "torque_Nm", "angle_sd_deg",
                         "grams_sd", "n_samples", "label"])
            for r in ar["rows"]:
                wr.writerow([f"{r[0]:.3f}", f"{r[1]:.2f}", f"{r[2]:.6f}",
                             f"{r[3]:.3f}", f"{r[4]:.2f}", r[5], r[6]])
    except Exception as e:
        ar_set(f"Export failed: {e}", RED)
        return
    ar_set(f"Exported {len(ar['rows'])} points → {path}", GREEN)


_ub = tk.Frame(ar_l, bg=SURFACE); _ub.pack(fill="x", pady=(6, 0))
btn(_ub, "⌫ Undo", FIELD, MUTED, ar_undo).pack(side="left", padx=(0, 6))
btn(_ub, "↺ Reset", FIELD, RED, ar_reset).pack(side="left")
btn(ar_l, "⇩ Export points CSV", FIELD, GREEN, ar_export).pack(fill="x", pady=(6, 0))

section(ar_l, "Model fitted to the points")
ar_mode = tk.StringVar(value="cos")
for _t, _v in (("gravity   τ = M·cos(θ−φ) + c", "cos"),
               ("straight line   τ = k·θ + c", "lin")):
    tk.Radiobutton(ar_l, text=_t, variable=ar_mode, value=_v,
                   command=lambda: ar_fit(), bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
                   highlightthickness=0).pack(anchor="w")
ar_set = mkstatus(ar_l)

# ── middle column · live ───────────────────────────────────────
ar_m_o, ar_m = card(arm_fr); ar_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(ar_m, "Live")
# Two readouts side by side, in a font one size down from big_value()'s:
# this card has to share a 1280-px window with two others, and a number that
# pushes the results column off the right edge is worse than a smaller number.
F_BIG2 = ("Consolas", 32, "bold")
_bv = tk.Frame(ar_m, bg=SURFACE); _bv.pack(fill="x")
_bv1 = tk.Frame(_bv, bg=SURFACE); _bv1.pack(side="left", expand=True)
ar_big_a = tk.Label(_bv1, text="—", bg=SURFACE, fg=BLUE, font=F_BIG2)
ar_big_a.pack()
tk.Label(_bv1, text="joint angle (deg)", bg=SURFACE, fg=FAINT, font=F_SM).pack()
_bv2 = tk.Frame(_bv, bg=SURFACE); _bv2.pack(side="left", expand=True)
ar_big_g = tk.Label(_bv2, text="—", bg=SURFACE, fg=GREEN, font=F_BIG2)
ar_big_g.pack()
ar_cap_g = tk.Label(_bv2, text="weight at the cell (g)", bg=SURFACE, fg=FAINT,
                    font=F_SM)
ar_cap_g.pack()
F_MONO2 = ("Consolas", 13)
ar_rpy = tk.Label(ar_m, text="roll — · pitch — · yaw —", bg=SURFACE, fg=MUTED,
                  font=F_MONO2)
ar_rpy.pack(pady=(10, 0))
ar_ft = tk.Label(ar_m, text="", bg=SURFACE, fg=PURPLE, font=F_MONO2)
ar_ft.pack(pady=(4, 0))
ar_link = tk.Label(ar_m, text="● no arm board", bg=SURFACE, fg=FAINT, font=F_SM)
ar_link.pack(pady=(6, 0))
ar_cv = tk.Canvas(ar_m, width=380, height=240, bg=SURFACE, highlightthickness=0)
ar_cv.pack(fill="both", expand=True, pady=(12, 0))

# ── right column · points & fit ────────────────────────────────
ar_rsc_o, ar_rsc = scroll_column(arm_fr); ar_rsc_o.pack(side="left", fill="y")
ar_r_o, ar_r = card(ar_rsc); ar_r_o.pack(fill="both", expand=True)
section(ar_r, "Captured points")
ar_table = mktable(ar_r, [("a", "angle °", 62), ("g", "grams", 58),
                          ("t", "τ mNm", 64), ("as", "±°", 40),
                          ("gs", "±g", 40), ("l", "label", 56)], height=10)
section(ar_r, "Result")
ar_lbl_big = tk.Label(ar_r, text="—", bg=SURFACE, fg=GREEN,
                      font=("Consolas", 28, "bold"))
ar_lbl_big.pack()
ar_lbl_cap = tk.Label(ar_r, text="peak gravity torque [mN·m]", bg=SURFACE,
                      fg=FAINT, font=F_SM)
ar_lbl_cap.pack()
ar_lbl_sub = tk.Label(ar_r, text="capture ≥ 3 points", bg=SURFACE, fg=FAINT,
                      font=F_SM, wraplength=300, justify="left")
ar_lbl_sub.pack(pady=(6, 0))
section(ar_r, "τ vs joint angle")
ar_scv = tk.Canvas(ar_r, width=300, height=215, bg=SURFACE, highlightthickness=0)
ar_scv.pack(fill="x", pady=(0, 10))


def ar_fit():
    """Re-fit and redraw. Called after every change to the point set, so the
    number on screen can never belong to a set that no longer exists."""
    xs = [r[0] for r in ar["rows"]]
    ys = [r[2] * 1000.0 for r in ar["rows"]]     # mN·m
    ar["fit"] = None
    need = 3 if ar_mode.get() == "cos" else 2
    if len(xs) < need:
        ar_lbl_big.config(text="—")
        ar_lbl_sub.config(text=f"capture ≥ {need} points", fg=FAINT)
        ar_scatter(ar_scv, xs, ys, None, "joint angle (°)", "τ (mN·m)")
        return
    span = max(xs) - min(xs)
    if ar_mode.get() == "lin":
        f = linfit(xs, ys)
        if f is None:
            ar_lbl_sub.config(text="all the points are at the same angle", fg=AMBER)
            return
        k, c, r2 = f
        ar_lbl_cap.config(text="stiffness [mN·m / °]")
        ar_lbl_big.config(text=f"{k:.2f}")
        ar["fit"] = (f"fit: tau_mNm = {k:.4f}*theta_deg + {c:.4f}  "
                     f"r2={r2:.4f}  n={len(xs)}")
        ar_lbl_sub.config(text=f"R² = {r2:.4f} · offset {c:+.1f} mN·m · "
                               f"{len(xs)} pts over {span:.0f}°", fg=FAINT)
        ar_scatter(ar_scv, xs, ys, lambda x: k * x + c,
                   "joint angle (°)", "τ (mN·m)")
        return
    f = cosfit(xs, ys)
    if f is None:
        ar_lbl_sub.config(text="the points do not determine a gravity curve", fg=AMBER)
        return
    a, b, c, amp, ph, r2 = f
    ar_lbl_cap.config(text="peak gravity torque [mN·m]")
    ar_lbl_big.config(text=f"{amp:.1f}")
    L = ar_lever_m() or float("nan")
    # M·g·L_com = amplitude, so the amplitude alone does not give a mass —
    # only the moment M·L_com about the joint, which is what actually has to
    # be compensated. Reporting that, and not an invented mass, is the honest
    # form: the arm's centre of mass is not measured here.
    moment = amp * 1e-3 / G_ACC * 1000.0          # g·m  (mass × its radius)
    ar["fit"] = (f"fit: tau_mNm = {a:.4f}*cos(theta) + {b:.4f}*sin(theta) + {c:.4f} "
                 f"| amplitude={amp:.3f} mNm phase={ph:+.2f} deg r2={r2:.4f} "
                 f"n={len(xs)} lever_m={L}")
    msg = (f"R² = {r2:.4f} · horizontal at θ = {ph:+.1f}° · "
           f"offset {c:+.1f} mN·m · moment m·r = {moment:.1f} g·m · "
           f"{len(xs)} pts over {span:.0f}°")
    if span < 20.0:
        msg += ("\n⚠ only " + f"{span:.0f}° of angle — a cosine cannot be "
                "identified from a short arc; the amplitude and phase here "
                "are extrapolation, not measurement.")
        ar_lbl_sub.config(text=msg, fg=AMBER)
    else:
        ar_lbl_sub.config(text=msg, fg=FAINT)
    ar_scatter(ar_scv, xs, ys,
               lambda x: a * math.cos(math.radians(x)) + b * math.sin(math.radians(x)) + c,
               "joint angle (°)", "τ (mN·m)")


def ar_tick():
    now = time.time()
    b = snap_arm()
    live = bool(b) and (now - b[-1][0]) <= 2.0
    r, p, y, g = arm_latest(0.6)
    ang = ar_angle((None, r, p, y)[ar_axis_idx()])
    ar_big_a.config(text=_n(ang, "{:+.1f}") if ang is not None else "—")
    # [g4-C] An uncalibrated cell is READING — it just has no grams yet. The
    # earlier version put a bare "—" here, which looks exactly like a dead
    # sensor and sent the operator hunting for a wiring fault that was not
    # there. Show the counts, say what they are.
    raw_now = b[-1][5] if b else None
    if g is not None:
        ar_big_g.config(text=f"{g:.1f}", fg=GREEN)
        ar_cap_g.config(text="weight at the cell (g)", fg=FAINT)
    elif raw_now is not None:
        ar_big_g.config(text=f"{raw_now:.0f}", fg=AMBER)
        ar_cap_g.config(text="RAW counts — no calibration yet (tab 9)", fg=AMBER)
    else:
        ar_big_g.config(text="—", fg=GREEN)
        ar_cap_g.config(text="weight at the cell (g)", fg=FAINT)
    ar_rpy.config(text=f"roll {_n(r)}°  pitch {_n(p)}°  yaw {_n(y)}°")
    tq = ar_torque(g)
    force = None if g is None else g * 1e-3 * G_ACC
    ar_ft.config(text=(f"force {force:+.2f} N   τ {tq*1000:+.1f} mN·m"
                       if tq is not None else
                       "force —   τ —   (needs calibrated grams + lever arm)"))
    # Uncalibrated is a state worth naming: the sketch prints LoadCell_raw
    # until it has been tared AND calibrated, and raw counts in a grams column
    # would be a silent unit error in every row of the log.
    rawonly = (raw_now is not None and g is None)
    n60 = len([s for s in b if now - s[0] <= 5.0])
    if not b:
        ar_link.config(text="● no arm board — choose its port in the header", fg=FAINT)
    elif not live:
        ar_link.config(text=f"● stream stopped {now - b[-1][0]:.0f} s ago", fg=RED)
    elif arm_cell_fault():
        ar_link.config(text="● load cell fault — see tab 9", fg=RED)
    elif rawonly:
        ar_link.config(text=f"● {n60/5.0:.1f} samples/s — UNCALIBRATED "
                            f"(raw {b[-1][5]:.0f} counts): calibrate on tab 9",
                       fg=AMBER)
    else:
        src = "board" if (arm_cal["source"] != "gui" and b[-1][4] is not None) else "GUI cal"
        raw_txt = "" if raw_now is None else f" · raw {raw_now:.0f}"
        ar_link.config(text=f"● {n60/5.0:.1f} samples/s · grams from {src}{raw_txt}",
                       fg=GREEN)
    ar_calnote.config(text=arm_cal_note(),
                      fg=AMBER if not (arm_cal_ready() or arm_cal["source"] == "board") else MUTED)
    if arm["msg"]:
        ar_msg.config(text="board says:  " + arm["msg"], fg=MUTED)

    i = ar_axis_idx()
    seg = [s for s in b if s[0] >= now - 60.0]
    draw_ts(ar_cv, [(s[0], ar_angle(s[i])) for s in seg],
            BLUE, "joint angle and force, last 60 s",
            y2=[(s[0], arm_grams(s)) for s in seg],
            unit1="°", unit2="g", label1="angle", label2="weight", signed=True)

    if ar["f"] is not None:
        fresh = [s for s in b if s[0] > ar["last_t"]]
        for s in fresh:
            a = ar_angle(s[i])
            gg = arm_grams(s)
            fN = None if gg is None else gg * 1e-3 * G_ACC
            tN = ar_torque(gg)
            try:
                ar["wr"].writerow([
                    f"{s[0] - ar['t0']:.3f}",
                    "" if s[1] is None else f"{s[1]:.3f}",
                    "" if s[2] is None else f"{s[2]:.3f}",
                    "" if s[3] is None else f"{s[3]:.3f}",
                    "" if a is None else f"{a:.3f}",
                    "" if gg is None else f"{gg:.3f}",
                    "" if fN is None else f"{fN:.5f}",
                    "" if tN is None else f"{tN:.6f}"])
            except Exception:
                ar_log_stop()
                ar_set("The log file stopped accepting rows — recording ended.", RED)
                return
            ar["n"] += 1
        if fresh:
            ar["last_t"] = fresh[-1][0]
            try:
                ar["f"].flush()      # a crash must not cost the whole run
            except Exception:
                pass
        ar_logl.config(text=f"● REC {ar['n']} samples · {now - ar['t0']:.0f} s\n{ar['path']}",
                       fg=GREEN)
    else:
        ar_logl.config(text="not recording", fg=FAINT)


add_tab("8 · Arm · Angle & Force", arm_fr)


# ═════════════ TAB 9 · ARM · CALIBRATE [g4-B] ══════════════════
# Calibrating the arm board from the GUI, against its RAW counts.
#
# The sketch can tare and scale itself, but only into RAM: the numbers die on
# reset and only survive by editing KNOWN_MASS_G and re-flashing. This tab
# does the same measurement on the host, keeps it in labdata/arm_cal.json,
# and therefore still holds tomorrow morning. Everything is a POINT —
# (grams, raw counts) — and the calibration is always the straight line
# fitted through them:
#
#     raw = a·grams + b      →      scale = 1/a ,  tare = b
#
# The zero capture is not a special case, it is the point (0 g, raw). That is
# why two captures are enough and a third one is a check rather than a
# ceremony, and why the same code handles a proper multi-point calibration.
cal_fr = tk.Frame(body, bg=BG)
ac = dict(last_msg="")

cal_sc_o, cal_sc = scroll_column(cal_fr); cal_sc_o.pack(side="left", fill="y", padx=(0, 14))
cal_l_o, cal_l = card(cal_sc); cal_l_o.pack(fill="both", expand=True)

section(cal_l, "Load cell · how grams are made")
ac_mode = tk.StringVar(value=arm_cal["source"])


def ac_mode_changed():
    arm_cal["source"] = ac_mode.get()
    arm_cal_save()
    ac_redraw()
    ac_set({"auto": "Auto: the board's own grams when it sends them, this "
                    "calibration when it sends raw counts.",
            "gui": "Using THIS calibration, even if the board sends grams.",
            "board": "Using the BOARD's grams. It forgets them on reset, and "
                     "raw-only streams will show no weight at all."}[ac_mode.get()],
           BLUE)


for _t, _v in (("auto — board grams if sent, else this calibration", "auto"),
               ("this calibration only  (from raw counts)", "gui"),
               ("the board's own grams only", "board")):
    tk.Radiobutton(cal_l, text=_t, variable=ac_mode, value=_v,
                   command=ac_mode_changed, bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
                   highlightthickness=0, wraplength=262, justify="left",
                   anchor="w").pack(anchor="w", fill="x")

section(cal_l, "Load cell · calibrate")
ac_settle = entry_row(cal_l, "Settle window (s)", 2.0)
tk.Label(cal_l, text="each capture is a trimmed mean over this\n"
                     "window, checked for drift across it.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))


def ac_win():
    try:
        w = float(ac_settle.get())
    except ValueError:
        w = 2.0
    return min(max(w, 0.5), 30.0)


def ac_drift_note(drift, sd):
    """A drift worth mentioning — measured against this cell's own noise, and
    quoted in grams once there is a scale to quote it in."""
    if drift is None:
        return ""
    bar = max(2.0 * (sd or 0.0), 1e-9)
    if abs(drift) <= bar:
        return ""
    if arm_cal["scale"]:
        return (f"  ⚠ the reading is still moving ({abs(drift * arm_cal['scale']):.1f} g "
                f"across the window) — let it settle and take it again.")
    return (f"  ⚠ the reading is still moving ({abs(drift):.0f} counts across "
            f"the window) — let it settle and take it again.")


def ac_zero():
    fault = arm_cell_fault()
    if fault:
        ac_set("Not zeroing: " + fault, RED)
        return
    m, sd, drift, n = arm_raw_settled(ac_win())
    if m is None:
        ac_set("No raw counts in that window. The board must be connected and "
               "printing LoadCell_raw — if it prints LoadCell_g only, it is "
               "calibrating itself and there is nothing here to fit.", RED)
        return
    # The zero IS a point: (0 g, raw). Keeping it in the same list as the
    # others is what lets one fit serve the two-point and the multi-point case.
    pts = [(g, r) for (g, r) in arm_cal["points"] if abs(g) > 1e-9]
    arm_cal["points"] = sorted([(0.0, m)] + pts)
    arm_cal["tare"] = m          # true even before there is a scale: it is
                                 # what the drift note and the live raw read
                                 # themselves against
    ac_refit(f"Zero captured: {m:.0f} counts, ±{(sd or 0):.0f} over {n} samples."
             + ac_drift_note(drift, sd))


def ac_capture_mass():
    try:
        g = float(ac_mass.get())
    except ValueError:
        ac_set("Enter the mass in grams first.", AMBER)
        return
    if g <= 0:
        ac_set("The calibration mass has to be a positive number of grams.", AMBER)
        return
    if not any(abs(p[0]) < 1e-9 for p in arm_cal["points"]):
        ac_set("Capture the zero first, with nothing on the cell.", AMBER)
        return
    fault = arm_cell_fault()
    if fault:
        ac_set("Not capturing: " + fault, RED)
        return
    m, sd, drift, n = arm_raw_settled(ac_win())
    if m is None:
        ac_set("No raw counts in that window — is the board still streaming?", RED)
        return
    # Re-taking a mass replaces it instead of adding a second version of the
    # same point, which would quietly weight that mass twice in the fit.
    pts = [(gg, rr) for (gg, rr) in arm_cal["points"] if abs(gg - g) > 1e-9]
    arm_cal["points"] = sorted(pts + [(g, m)])
    ac_refit(f"{g:g} g captured: {m:.0f} counts, ±{(sd or 0):.0f} over {n} samples."
             + ac_drift_note(drift, sd))


_z = tk.Frame(cal_l, bg=SURFACE); _z.pack(fill="x", pady=(0, 4))
btn(_z, "① Zero — cell empty", BLUE, "white", ac_zero).pack(fill="x")
ac_mass = entry_row(cal_l, "Known mass (g)", 500.0)
btn(cal_l, "② Capture — mass on the cell", GREEN, "white",
    ac_capture_mass).pack(fill="x", pady=(2, 0))
tk.Label(cal_l, text="Repeat ② with other masses for a multi-point\n"
                     "fit. Two points give a line; three or more\n"
                     "give a line AND an R² that can argue back.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(4, 2))


def ac_drop():
    if not arm_cal["points"]:
        return
    arm_cal["points"] = arm_cal["points"][:-1]
    ac_refit("Dropped the last point.", BLUE)


def ac_clear():
    if not confirm_reset(len(arm_cal["points"]), "calibration points"):
        return
    arm_cal.update(points=[], scale=0.0, tare=0.0, r2=None, when="", span="")
    ac_refit("Load-cell calibration cleared. Grams are unavailable until you "
             "capture a zero and a mass again.", BLUE)


_d = tk.Frame(cal_l, bg=SURFACE); _d.pack(fill="x", pady=(4, 0))
btn(_d, "⌫ Drop last", FIELD, MUTED, ac_drop).pack(side="left", padx=(0, 6))
btn(_d, "↺ Clear all", FIELD, RED, ac_clear).pack(side="left")

section(cal_l, "IMU · zero and span")
tk.Label(cal_l, text="Zero is the pose you call 0°. Span is only\n"
                     "worth doing if the IMU's axis is not parallel\n"
                     "to the joint's — and then it is a patch, not\n"
                     "a fix: a misalignment is a cosine, so a gain\n"
                     "measured over one span will not hold over\n"
                     "another.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))


def ac_imu_zero():
    r, p, y, _g = arm_latest(ac_win())
    if r is None and p is None and y is None:
        ac_set("No angles in that window — connect the board first.", RED)
        return
    for k, v in (("Roll", r), ("Pitch", p), ("Yaw", y)):
        if v is not None:
            arm_zero[k] = v
    arm_cal_save()
    ac_redraw()
    ac_set(f"Zero set here — roll {_n(r)}°, pitch {_n(p)}°, yaw {_n(y)}°. "
           f"All three axes were zeroed at this pose.", GREEN)


def ac_imu_span():
    axis = ar_axis.get()
    try:
        known = abs(float(ac_angle.get()))
    except ValueError:
        ac_set("Enter the true angle of this pose, in degrees.", AMBER)
        return
    if known < 5.0:
        ac_set("Use a span of at least 5° — a gain measured across a couple of "
               "degrees is mostly measuring the IMU's noise.", AMBER)
        return
    r, p, y, _g = arm_latest(ac_win())
    v = {"Roll": r, "Pitch": p, "Yaw": y}[axis]
    if v is None:
        ac_set(f"No {axis} reading in that window.", RED)
        return
    meas = abs(arm_wrap(v - arm_zero[axis]))       # BEFORE any existing gain
    if meas < 1.0:
        ac_set("The joint is still at the zero pose — move it to the known "
               "angle first, then capture.", AMBER)
        return
    g = known / meas
    if not (0.5 <= g <= 2.0):
        ac_set(f"That would need a gain of ×{g:.3f} ({meas:.1f}° measured for "
               f"{known:.1f}° true). A factor that large is a wrong axis or a "
               f"wrong zero, not a calibration — check both before forcing it.",
               RED)
        return
    # The gain is a magnitude only. Direction is the tab-8 'invert' switch, and
    # folding a sign in here as well would let the two cancel each other and
    # leave nobody able to say which one is wrong.
    arm_gain[axis] = g
    arm_cal["gain_when"] = datetime.now().strftime("%Y-%m-%d %H:%M")
    arm_cal_save()
    ac_redraw()
    ac_set(f"{axis} gain = ×{g:.4f}  ({meas:.2f}° measured for {known:.2f}° "
           f"true). Valid near this span only.", GREEN)


def ac_gain_reset():
    for k in arm_gain:
        arm_gain[k] = 1.0
    arm_cal["gain_when"] = ""
    arm_cal_save()
    ac_redraw()
    ac_set("All IMU gains back to ×1 — angles are the IMU's own, offset by the "
           "zero.", BLUE)


_ia = tk.Frame(cal_l, bg=SURFACE); _ia.pack(fill="x", pady=3)
tk.Label(_ia, text="Span axis (shared with tab 8)", bg=SURFACE, fg=TEXT,
         font=F_SM).pack(side="left")
_iam = tk.OptionMenu(_ia, ar_axis, "Roll", "Pitch", "Yaw")
_iam.config(bg=FIELD, fg=BLUE, relief="flat", bd=0, highlightthickness=0,
            font=("Segoe UI", 12, "bold"))
_iam.pack(side="right")
btn(cal_l, "⌖ Zero all axes at this pose", BLUE, "white", ac_imu_zero).pack(fill="x", pady=(2, 4))
ac_angle = entry_row(cal_l, "True angle of this pose (°)", 90.0)
btn(cal_l, "Capture span → gain", FIELD, GREEN, ac_imu_span).pack(fill="x", pady=(2, 0))
btn(cal_l, "reset gains to ×1", FIELD, MUTED, ac_gain_reset).pack(fill="x", pady=(2, 0))

section(cal_l, "File")
ac_file = tk.Label(cal_l, text="", bg=SURFACE, fg=FAINT, font=F_SM,
                   wraplength=262, justify="left", anchor="w")
ac_file.pack(fill="x")


def ac_reload():
    if arm_cal_load():
        ac_mode.set(arm_cal["source"])
        ac_redraw()
        ac_set("Calibration reloaded from the file.", BLUE)
    else:
        ac_set("No saved calibration to reload yet.", AMBER)


def ac_save_now():
    if arm_cal_save():
        ac_set("Saved to " + ARM_CAL_FILE, GREEN)
    else:
        ac_set("Could not write " + ARM_CAL_FILE + " — is the labdata folder "
               "writable?", RED)


_calf = tk.Frame(cal_l, bg=SURFACE); _calf.pack(fill="x", pady=(4, 0))
btn(_calf, "Save now", FIELD, GREEN, ac_save_now).pack(side="left", padx=(0, 6))
btn(_calf, "Reload", FIELD, MUTED, ac_reload).pack(side="left")
ac_set = mkstatus(cal_l)

# ── middle column · what the cell is doing right now ───────────
cal_m_o, cal_m = card(cal_fr); cal_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(cal_m, "Live")
_cv1 = tk.Frame(cal_m, bg=SURFACE); _cv1.pack(fill="x")
_cva = tk.Frame(_cv1, bg=SURFACE); _cva.pack(side="left", expand=True)
ac_big_raw = tk.Label(_cva, text="—", bg=SURFACE, fg=PURPLE, font=("Consolas", 30, "bold"))
ac_big_raw.pack()
tk.Label(_cva, text="raw counts", bg=SURFACE, fg=FAINT, font=F_SM).pack()
_cvb = tk.Frame(_cv1, bg=SURFACE); _cvb.pack(side="left", expand=True)
ac_big_g = tk.Label(_cvb, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
ac_big_g.pack()
tk.Label(_cvb, text="grams by this calibration", bg=SURFACE, fg=FAINT, font=F_SM).pack()
ac_noise = tk.Label(cal_m, text="", bg=SURFACE, fg=MUTED, font=("Consolas", 12),
                    wraplength=360, justify="center")
ac_noise.pack(pady=(10, 0))
ac_ang = tk.Label(cal_m, text="", bg=SURFACE, fg=BLUE, font=("Consolas", 12),
                  wraplength=360, justify="center")
ac_ang.pack(pady=(4, 0))
ac_state = tk.Label(cal_m, text="", bg=SURFACE, fg=FAINT, font=F_SM,
                    wraplength=360, justify="left")
ac_state.pack(pady=(6, 0))
ac_cv = tk.Canvas(cal_m, width=380, height=230, bg=SURFACE, highlightthickness=0)
ac_cv.pack(fill="both", expand=True, pady=(10, 0))

# ── right column · the points and the line through them ────────
cal_r_sc_o, cal_r_sc = scroll_column(cal_fr); cal_r_sc_o.pack(side="left", fill="y")
cal_r_o, cal_r = card(cal_r_sc); cal_r_o.pack(fill="both", expand=True)
section(cal_r, "Calibration points")
ac_table = mktable(cal_r, [("g", "mass g", 60), ("r", "raw", 80),
                           ("d", "Δ counts", 74), ("c", "counts/g", 76)], height=8)
section(cal_r, "Result")
ac_lbl_big = tk.Label(cal_r, text="—", bg=SURFACE, fg=GREEN,
                      font=("Consolas", 26, "bold"))
ac_lbl_big.pack()
tk.Label(cal_r, text="counts per gram", bg=SURFACE, fg=FAINT, font=F_SM).pack()
ac_lbl_sub = tk.Label(cal_r, text="", bg=SURFACE, fg=FAINT, font=F_SM,
                      wraplength=282, justify="left")
ac_lbl_sub.pack(pady=(6, 0))
section(cal_r, "raw − zero  vs  mass")
ac_scv = tk.Canvas(cal_r, width=282, height=200, bg=SURFACE, highlightthickness=0)
ac_scv.pack(fill="x", pady=(0, 10))


def ac_refit(msg="", colour=GREEN):
    """Re-fit the line through every point and store it. Called after any
    change to the point set, so what is saved and what is on screen cannot
    disagree."""
    pts = arm_cal["points"]
    if len(pts) >= 2:
        f = linfit([p[0] for p in pts], [p[1] for p in pts])
        if f is None or abs(f[0]) < 1e-12:
            arm_cal.update(scale=0.0, r2=None)
            ac_redraw()
            ac_set("The counts did not change with the mass — check that the "
                   "cell is loaded, wired to A+/A−, and not saturated.", RED)
            return
        a, bb, r2 = f                     # raw = a*grams + b
        arm_cal.update(scale=1.0 / a, tare=bb,
                       r2=(r2 if len(pts) > 2 else None),
                       when=datetime.now().strftime("%Y-%m-%d %H:%M"),
                       span=f"{min(p[0] for p in pts):.0f}–{max(p[0] for p in pts):.0f} g")
    elif len(pts) <= 1:
        # One point is a zero, not a calibration: keep the offset, refuse to
        # invent a slope for it.
        arm_cal.update(scale=0.0, r2=None, span="")
        if pts and abs(pts[0][0]) < 1e-9:
            arm_cal["tare"] = pts[0][1]
    arm_cal_save()
    ac_redraw()
    if msg:
        ac_set(msg, colour)


def ac_redraw():
    clear_table(ac_table)
    pts = arm_cal["points"]
    for g, r in pts:
        d = r - arm_cal["tare"]
        cpg = "—" if abs(g) < 1e-9 else f"{d / g:.2f}"
        ac_table.insert("", "end", values=(f"{g:g}", f"{r:.0f}", f"{d:+.0f}", cpg))
    cpg = arm_counts_per_gram()
    if cpg is None:
        ac_lbl_big.config(text="—")
        ac_lbl_sub.config(
            text=("Capture the zero, then one known mass." if len(pts) < 2
                  else "No usable line through these points."), fg=FAINT)
    else:
        ac_lbl_big.config(text=f"{cpg:.2f}")
        bits = [f"zero {arm_cal['tare']:.0f} counts",
                f"{len(pts)} pts over {arm_cal['span'] or '—'}"]
        if arm_cal["r2"] is not None:
            bits.append(f"R² = {arm_cal['r2']:.5f}")
        else:
            bits.append("2 points — R² would be 1.0 by construction, so it is "
                        "not shown; add a third mass if you want a check")
        if cpg < 0:
            bits.append("⚠ negative: the cell reads DOWN as the mass goes up — "
                        "fine if it is mounted in tension, otherwise swap A+/A−")
        if arm_cal["when"]:
            bits.append("set " + arm_cal["when"])
        ac_lbl_sub.config(text=" · ".join(bits), fg=FAINT)
    xs = [p[0] for p in pts]
    ys = [p[1] - arm_cal["tare"] for p in pts]
    # The fit is plotted against (raw - zero), so the line through it passes
    # through the origin by construction and its slope IS counts per gram.
    curve = None if cpg is None else (lambda g: g * cpg)
    ar_scatter(ac_scv, xs, ys, curve, "mass (g)", "raw − zero (counts)")
    gz = ", ".join(f"{k} {arm_zero[k]:+.2f}°"
                   + (f" ×{arm_gain[k]:.4f}" if abs(arm_gain[k] - 1) > 1e-9 else "")
                   for k in ("Roll", "Pitch", "Yaw"))
    ac_file.config(text=f"{ARM_CAL_FILE}\nIMU zero/gain: {gz}")
    if "ar_calnote" in globals():
        ar_calnote.config(text=arm_cal_note())


_ac_seen = [None]


def ac_tick():
    sig = (len(arm_cal["points"]), arm_cal["scale"], arm_cal["tare"],
           arm_cal["when"], arm_cal["source"], tuple(arm_zero.values()),
           tuple(arm_gain.values()))
    if sig != _ac_seen[0]:
        # the Diagnostics tab edits the same calibration; redraw when it does
        _ac_seen[0] = sig
        ac_redraw()
    b = snap_arm()
    now = time.time()
    live = bool(b) and (now - b[-1][0]) <= 2.0
    m, sd, drift, n = arm_raw_settled(max(2.0, ac_win()))
    last_raw = b[-1][5] if b else None
    ac_big_raw.config(text="—" if last_raw is None else f"{last_raw:.0f}")
    g = arm_grams(b[-1]) if b else None
    ac_big_g.config(text="—" if g is None else f"{g:.2f}")
    if m is None:
        ac_noise.config(text="no raw counts in the window")
    else:
        # Noise in counts is only half an answer. What the operator needs to
        # know is what it costs in grams — that is the resolution of every
        # measurement this rig will make.
        ng = ("" if not arm_cal["scale"]
              else f"  =  ±{abs((sd or 0) * arm_cal['scale']):.2f} g")
        dr = ("" if drift is None else f"   drift {drift:+.0f}")
        ac_noise.config(text=f"noise ±{(sd or 0):.0f} counts{ng}{dr}  ({n} samples)")
    r, p, y, _gg = arm_latest(1.0)
    joint = ar_angle({"Roll": r, "Pitch": p, "Yaw": y}[ar_axis.get()])
    ac_ang.config(text=f"roll {_n(r)}°  pitch {_n(p)}°  yaw {_n(y)}°\n"
                       f"{ar_axis.get()} → {_n(joint)}° as the joint")
    if not b:
        ac_state.config(text="No arm board. Choose its port in the header, then "
                             "come back — nothing here can be calibrated blind.",
                        fg=AMBER)
    elif not live:
        ac_state.config(text=f"Stream stopped {now - b[-1][0]:.0f} s ago.", fg=RED)
    elif last_raw is None:
        ac_state.config(text="The board is sending grams (LoadCell_g) and no raw "
                             "counts, so there is nothing here to fit. That is "
                             "its own calibration — fine to use, but it is gone "
                             "at its next reset.", fg=MUTED)
    else:
        fault = arm_cell_fault()
        if fault:
            ac_state.config(text="⚠ " + fault, fg=RED)
        else:
            ac_state.config(text=arm_cal_note(),
                            fg=MUTED if arm_cal_ready() else AMBER)
    draw_ts(ac_cv, [(s[0], s[5]) for s in b if s[0] >= now - 60.0],
            PURPLE, "raw counts, last 60 s", unit1="counts", label1="raw",
            signed=True)


add_tab("9 · Arm · Calibrate", cal_fr)
ac_redraw()


# ═════════════ TAB 10 · E9 · GROUND TRUTH [g5-A] ═══════════════
# The E9 rig from the protocol: a load cell in a pinned strut holds the arm,
# the motor sweeps current, and the force the strut reads becomes the TRUTH
# that the firmware's torque estimate is judged against.
#
# This replaces the capstan push/pull tab, and it is a different experiment,
# not a re-skin. Three things had to change, and each is a rule in the
# protocol that the old tab broke:
#
#   1. THE MOMENT ARM IS NOT F x lever. The strut is pinned at both ends, so
#      it is a two-force member: the force acts along the line through both
#      pins, and the moment arm is the perpendicular distance from the joint
#      to THAT line. Measuring a height with a ruler and calling it the lever
#      gives a number that is wrong by cot(psi) and wrong in a way nothing in
#      the data would reveal. With A on the arm and B on the base:
#          A  = (L cos th, L sin th)          u = (A - B)/|A - B|
#          r_perp = A_x u_y - A_y u_x         tau_truth = F * r_perp
#      psi is the angle at A between the arm and the strut, and the identity
#      |r_perp| = L sin psi is computed every sample as a cross-check.
#
#   2. THE ARM'S WEIGHT MUST NOT BE TARED OUT. Zeroing the cell while it
#      carries the arm deletes gravity from the ground truth, and the residual
#      against joint angle — the most valuable plot here, because it measures
#      the error in tau_g(theta) directly — becomes flat by construction. So
#      this tab records ABSOLUTE force and never subtracts a baseline. The
#      30 s baseline at Iq = 0 is kept as data: its mean is the measured
#      gravity torque, and its spread is sigma_0, which is what a dead-zone
#      has to be justified against.
#
#   3. A SLOW RAMP, NOT STEPS. One ramp 0 -> Imax -> 0 over ~60 s gives
#      hundreds of points and the return leg for free; the hysteresis between
#      the legs IS the transmission's friction torque. Steps give a handful of
#      settled points and no loop.
#
# The estimator is computed HERE, from parameters you type in, because the lab
# firmware's telemetry does not carry one:
#       tau_est_static = K_t,j * Iq - m g l cos(theta - theta_0)
#       K_t,j = N * eta * K_t
# The direct and DOB columns exist in the log and are left empty on purpose:
# they need J, b and a filter cutoff, and the protocol's whole point is that
# all three must be recomputed OFFLINE from the same raw Iq, theta and omega —
# which every row carries — so that the comparison is bit-for-bit fair.
e9_fr = tk.Frame(body, bg=BG)
e9 = dict(run=False, rows=[], lock=threading.Lock(), t0=0.0, phase="idle",
          path="", run_id=0, slopes=[], p=None, pause=False, why="",
          sigma0=None, taug_meas=None, taug_err=None, taug_model=None,
          taug_both=False, taug_latched=None,
          last_arm=0.0)

# row layout — the protocol's CSV column order, built once per sample
(E9_TPC, E9_TMCU, E9_RUN, E9_ANG, E9_DIR, E9_STAGE, E9_THI, E9_THE, E9_XB,
 E9_YB, E9_PSI, E9_RP, E9_IQ, E9_OM, E9_F, E9_TT, E9_TE) = range(17)

E9_COLS = ["t_pc", "t_mcu", "run_id", "angle_id", "direction", "stage",
           "theta_incl_deg", "theta_enc_deg", "x_B_mm", "y_B_mm", "psi_deg",
           "r_perp_m", "I_q_A", "omega_rad_s", "force_N", "tau_truth_Nm",
           "tau_est_static_Nm", "tau_est_direct_Nm", "tau_est_dob_Nm"]

E9_STAGE_COLOR = {"baseline": MUTED, "ramp_up": BLUE, "ramp_dn": PURPLE,
                  "fiducial": AMBER}
SIN_PSI_MIN = 0.7          # protocol §2: below this the sample is discarded


def e9_rows():
    with e9["lock"]:
        return list(e9["rows"])


def e9_geometry(theta_deg, L_mm, xB_mm, yB_mm):
    """→ (r_perp in m, psi in deg) for one joint angle, from the pin
    coordinates. Both are recomputed for EVERY sample, because theta moves
    (a little, even in an isometric run — the frame flexes) and r_perp
    follows it."""
    if theta_deg is None or not L_mm:
        return (None, None)
    th = math.radians(theta_deg)
    L = L_mm / 1000.0
    ax, ay = L * math.cos(th), L * math.sin(th)          # O → A
    dx, dy = xB_mm / 1000.0 - ax, yB_mm / 1000.0 - ay    # A → B
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return (None, None)
    cross = ax * dy - ay * dx
    dot = ax * dx + ay * dy
    r_perp = -cross / n                        # = A × û, û = (A−B)/|A−B|
    psi = math.degrees(math.atan2(abs(cross), dot))
    return (r_perp, psi)


def e9_tau_g(theta_deg, m_kg, l_m, th0_deg):
    """The gravity model the firmware uses: m g l cos(theta − theta_0)."""
    if theta_deg is None:
        return None
    return m_kg * G_ACC * l_m * math.cos(math.radians(theta_deg - th0_deg))


def linfit_se(xs, ys):
    """→ (a, b, r2, se_a, se_b, rmse_about_identity, n) or None.

    The protocol asks for a ± SE and b ± SE, not just the point estimates:
    a slope of 1.03 means nothing until you know whether its standard error
    is 0.005 or 0.05. The last term is the RMSE of (y − x), which is the
    agreement number, NOT the residual about the fitted line — a fit can be
    tight around a line that is in the wrong place."""
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-12:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    sse = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    sst = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - sse / sst if sst > 1e-12 else 1.0
    s = math.sqrt(sse / (n - 2)) if n > 2 else 0.0
    se_a = s / math.sqrt(sxx)
    se_b = s * math.sqrt(1.0 / n + mx * mx / sxx)
    rmse = math.sqrt(sum((y - x) ** 2 for x, y in zip(xs, ys)) / n)
    return (a, b, r2, se_a, se_b, rmse, n)


# ── left column · the station ─────────────────────────────────────────────
e9_sc_o, e9_sc = scroll_column(e9_fr); e9_sc_o.pack(side="left", fill="y", padx=(0, 14))
e9_left_o, e9_left = card(e9_sc); e9_left_o.pack(fill="both", expand=True)

section(e9_left, "Station geometry  (measure, don't assume)")
tk.Label(e9_left, text="All distances from the JOINT AXIS O, in mm.\n"
                    "x_B and y_B are re-measured at every station\n"
                    "to ±0.5 mm — they are what enters the formula,\n"
                    "not where you meant to put the pin.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
e9_L = entry_row(e9_left, "L · joint → pin A (mm)", 150.0)
e9_xB = entry_row(e9_left, "x_B (mm, + forward)", 65.0)
e9_yB = entry_row(e9_left, "y_B (mm, + up)", -188.0)
e9_ang_id = entry_row(e9_left, "Station label (angle_id)", "-30")
e9_fsign = tk.IntVar(value=0)
tk.Checkbutton(e9_left, text="invert force sign (+ = strut in compression)",
               variable=e9_fsign, bg=SURFACE, fg=TEXT, selectcolor=FIELD,
               font=F_SM, activebackground=SURFACE, highlightthickness=0,
               wraplength=262, justify="left", anchor="w").pack(anchor="w", fill="x")
e9_geo = tk.Label(e9_left, text="", bg=SURFACE, fg=MUTED, font=("Consolas", 12),
                  wraplength=262, justify="left", anchor="w")
e9_geo.pack(fill="x", pady=(6, 0))

section(e9_left, "Estimator under test")
tk.Label(e9_left, text="K_t,j = N · η · K_t. Take K_t from E7/E3 — fitting\n"
                    "it from this data and then checking this data\n"
                    "with it is a circle, not a test.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 4))
e9_kt = entry_row(e9_left, "K_t motor (mN·m/A)", 90.0)
e9_N = entry_row(e9_left, "N · gear ratio", 9.0)
e9_eta = entry_row(e9_left, "η · efficiency", 1.0)
e9_m = entry_row(e9_left, "arm mass m (kg)", 0.50)
e9_lcom = entry_row(e9_left, "CoM radius l (m)", 0.13)
e9_th0 = entry_row(e9_left, "θ₀ · angle of max gravity (°)", 0.0)
e9_dz = entry_row(e9_left, "dead-zone under test (N·m)", 0.10)

section(e9_left, "Run · one station pass")
e9_imax = entry_row(e9_left, "I_max (A)", 3.0)
e9_ilim = entry_row(e9_left, "hard current limit Pi (A)", 5.0)
e9_base = entry_row(e9_left, "baseline at Iq=0 (s)", 30.0)
e9_ramp_s = entry_row(e9_left, "ramp 0→max→0 (s)", 60.0)
e9_reps = entry_row(e9_left, "repeats (nudge ±1° between)", 3)
e9_dir = tk.StringVar(value="both")
for _t, _v in (("both directions (flex + extend)", "both"),
               ("positive current only", "+"), ("negative current only", "-")):
    tk.Radiobutton(e9_left, text=_t, variable=e9_dir, value=_v, bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
                   highlightthickness=0, anchor="w").pack(anchor="w", fill="x")
e9_fid = tk.IntVar(value=1)
tk.Checkbutton(e9_left, text="fiducial current pulse at the ends of the file",
               variable=e9_fid, bg=SURFACE, fg=TEXT, selectcolor=FIELD,
               font=F_SM, activebackground=SURFACE, highlightthickness=0,
               wraplength=262, justify="left", anchor="w").pack(anchor="w", fill="x")
e9_flim = entry_row(e9_left, "abort above force (N)", 45.0)


def e9_val(entry, default=None):
    """An entry that may legitimately be blank. Tk thread only."""
    t = entry.get().strip()
    if not t:
        return default
    try:
        return float(t)
    except ValueError:
        return default


def e9_read_params():
    """Tk thread only — the worker never touches a widget."""
    return dict(
        L=e9_val(e9_L, 150.0), xB=e9_val(e9_xB, 0.0), yB=e9_val(e9_yB, 0.0),
        angle_id=e9_ang_id.get().strip() or "?",
        fsign=(-1.0 if e9_fsign.get() else 1.0),
        kt=e9_val(e9_kt, 0.0), N=e9_val(e9_N, 1.0) or 1.0, eta=e9_val(e9_eta, 1.0) or 1.0,
        m=e9_val(e9_m, 0.0), lcom=e9_val(e9_lcom, 0.0), th0=e9_val(e9_th0, 0.0),
        dz=abs(e9_val(e9_dz, 0.1) or 0.1),
        imax=abs(e9_val(e9_imax, 1.0) or 1.0), ilim=abs(e9_val(e9_ilim, 5.0) or 5.0),
        base_s=max(1.0, e9_val(e9_base, 30.0) or 30.0),
        ramp_s=max(4.0, e9_val(e9_ramp_s, 60.0) or 60.0),
        reps=max(1, int(e9_val(e9_reps, 3) or 3)),
        direction=e9_dir.get(), fid=bool(e9_fid.get()),
        flim=abs(e9_val(e9_flim, 45.0) or 45.0),
        axis=ar_axis.get(), axis_idx=ar_axis_idx(),
        sign=(-1.0 if ar_inv.get() else 1.0))


def e9_p(key, default=None):
    return (e9.get("p") or {}).get(key, default)


def e9_ktj():
    """K_t,j in N·m/A at the joint."""
    return e9_p("N", 1.0) * e9_p("eta", 1.0) * e9_p("kt", 0.0) / 1000.0


def e9_theta(s):
    """Joint angle from the inclinometer on the arm support, in degrees."""
    a = arm_angle_cal(e9_p("axis", "Pitch"), s[e9_p("axis_idx", 2)])
    return None if a is None else e9_p("sign", 1.0) * a


def e9_status(msg, colour=BLUE):
    root.after(0, lambda: e9_set(msg, colour))


def e9_sample(stage, s, direction):
    """One arm-board sample → one row, with the geometry and the estimator
    evaluated at that sample's own instant."""
    theta = e9_theta(s)
    r_perp, psi = e9_geometry(theta, e9_p("L"), e9_p("xB"), e9_p("yB"))
    grams = arm_grams(s)
    # ABSOLUTE force. No baseline subtraction anywhere in this tab.
    force = None if grams is None else e9_p("fsign", 1.0) * grams * 1e-3 * G_ACC
    tau_truth = None if (force is None or r_perp is None) else force * r_perp
    iq = hist_at(iq_hist, s[0])
    if iq is None:
        iq = T["iq"]
    om = hist_at(rpm_hist, s[0])
    om = (om * 2 * math.pi / 60.0) if om is not None else T["vel"]
    m_ang = hist_at(ang_hist, s[0])
    th_enc = (math.degrees(T["ang"] if m_ang is None else m_ang)
              / max(1e-9, e9_p("N", 1.0)))
    tg = e9_tau_g(theta, e9_p("m", 0.0), e9_p("lcom", 0.0), e9_p("th0", 0.0))
    tau_est = None if (tg is None) else e9_ktj() * iq - tg
    return (s[0] - e9["t0"], s[0], e9["run_id"], e9_p("angle_id", "?"),
            direction, stage, theta, th_enc, e9_p("xB"), e9_p("yB"), psi,
            r_perp, iq, om, force, tau_truth, tau_est)


def e9_collect(stage, duration, direction, stop_when=None):
    """Sample for `duration` s, one row per NEW arm-board sample. Returns ""
    or the reason it stopped."""
    e9["phase"] = stage
    t_end = time.time() + duration
    flim = e9_p("flim")
    while time.time() < t_end:
        if not e9["run"]:
            return "stopped"
        for s in snap_arm():
            if s[0] <= e9["last_arm"]:
                continue
            e9["last_arm"] = s[0]
            row = e9_sample(stage, s, direction)
            with e9["lock"]:
                e9["rows"].append(row)
            if row[E9_F] is not None and abs(row[E9_F]) > flim:
                return (f"force {row[E9_F]:+.1f} N past the {flim:.0f} N abort "
                        f"limit — check the cell's range and the current limit")
        if stop_when is not None and stop_when():
            return ""
        time.sleep(0.02)
    return ""


def e9_leg(sign):
    """Baseline, then one slow ramp 0 → Imax → 0, in one current direction."""
    P = e9["p"]
    imax = sign * P["imax"]
    rate = max(0.1, P["imax"] / max(1.0, P["ramp_s"] / 2.0))   # firmware min 0.1
    dev_send(f"Pr{rate:.3f}")
    dev_send("O0")
    dtag = "+" if sign > 0 else "-"

    # BASELINE — kept as data, not thrown away: its mean is the measured
    # gravity torque at this station and its spread is sigma_0.
    e9_status(f"Baseline {P['base_s']:.0f} s at Iq = 0 — nothing may touch the "
              f"arm but the strut.", BLUE)
    why = e9_collect("baseline", P["base_s"], dtag)
    if why:
        return why

    e9_status(f"Ramp {dtag}{P['imax']:.2f} A over {P['ramp_s']:.0f} s…", BLUE)
    dev_send(f"C{imax:.3f}")
    why = e9_collect("ramp_up", P["ramp_s"] / 2.0 + 8.0, dtag,
                     stop_when=lambda: abs(T["iqc"]) >= 0.98 * abs(imax) - 1e-6)
    if why:
        return why
    dev_send("C0")
    return e9_collect("ramp_dn", P["ramp_s"] / 2.0 + 8.0, dtag,
                      stop_when=lambda: abs(T["iqc"]) < 0.02)


def e9_fiducial(tag):
    """A short current pulse at each end of the file. Both streams see it —
    the cell as a force step, the MCU as a current step — so the two can be
    aligned afterwards without trusting either clock."""
    if not e9_p("fid"):
        return ""
    dev_send("Pr20")
    dev_send(f"C{min(0.5, e9_p('imax', 0.5)):.2f}")
    why = e9_collect("fiducial", 0.6, tag)
    dev_send("C0")
    if why:
        return why
    return e9_collect("fiducial", 0.8, tag)


def e9_runner():
    P = e9["p"]
    if P["kt"] <= 0 or P["L"] <= 0:
        e9_status("K_t and L must both be positive.", AMBER)
        e9["run"] = False; return
    if P["imax"] > P["ilim"]:
        e9_status(f"I_max {P['imax']:.2f} A is above the hard limit "
                  f"{P['ilim']:.2f} A.", AMBER)
        e9["run"] = False; return
    if not dev_alive():
        e9_status("No telemetry from the driver — connect it first.", RED)
        e9["run"] = False; return
    if not arm_alive():
        e9_status("No sensor board — connect it in the header.", RED)
        e9["run"] = False; return
    fault = arm_cell_fault()
    if fault:
        e9_status("Load cell fault: " + fault, RED)
        e9["run"] = False; return
    if not arm_cal_ready() and arm_cal["source"] != "board":
        e9_status("The cell is not calibrated — do tab 9 first. A run in raw "
                  "counts has no ground truth in it.", RED)
        e9["run"] = False; return

    # ψ must be inside the window BEFORE the run, or every sample of it will
    # be thrown away by the |sin ψ| ≥ 0.7 filter at analysis time.
    r, p, y, _g = arm_latest(1.0)
    th = e9_theta((0, r, p, y, None, None))
    r_perp, psi = e9_geometry(th, P["L"], P["xB"], P["yB"])
    if psi is None:
        e9_status("No angle from the inclinometer — is the sensor board "
                  "streaming?", RED)
        e9["run"] = False; return
    if abs(math.sin(math.radians(psi))) < SIN_PSI_MIN:
        e9_status(f"ψ = {psi:.1f}° — |sin ψ| < {SIN_PSI_MIN}, so every sample "
                  f"would be discarded. Move pin B and re-measure x_B, y_B.",
                  RED)
        e9["run"] = False; return
    if not (60.0 <= psi <= 120.0):
        e9_status(f"ψ = {psi:.1f}° is outside the 60–120° acceptable window — "
                  f"running anyway, but the force needed climbs steeply here.",
                  AMBER)

    dev_send(f"Pi{P['ilim']:.2f}")
    dev_send("O0")
    e9["t0"] = time.time()
    e9["last_arm"] = time.time()
    e9["slopes"] = []
    dirs = {"both": [+1, -1], "+": [+1], "-": [-1]}[P["direction"]]

    why = e9_fiducial("+")
    for rep in range(P["reps"]):
        if why or not e9["run"]:
            break
        e9["run_id"] = rep + 1
        for sign in dirs:
            why = e9_leg(sign)
            if why:
                break
        if why:
            break
        e9_mark_repeat()
        if rep < P["reps"] - 1:
            # The rotor has not moved all run, so its cogging torque is a
            # fixed offset in this repeat — and the estimator has no cogging
            # term, so it lands in b. Nudging the arm ~1° (≈2 cogging periods
            # at the motor) and averaging is what removes it; the spread
            # between repeats then MEASURES it.
            e9["pause"] = True
            e9_status(f"Repeat {rep+1}/{P['reps']} done. Nudge the arm ~1° "
                      f"(move B a few mm), re-measure x_B and y_B, type them "
                      f"in, then press Continue.", AMBER)
            while e9["pause"] and e9["run"]:
                time.sleep(0.1)
            if not e9["run"]:
                why = "stopped"
                break
    if not why:
        why = e9_fiducial("+")

    dev_send("C0")
    stop_all()
    e9["run"] = False
    e9["phase"] = "idle"
    e9["why"] = why
    root.after(0, e9_analyze)
    if why:
        e9_status("Run stopped — " + why, RED if why != "stopped" else AMBER)
    else:
        root.after(0, e9_save)


def e9_mark_repeat():
    """Fit this repeat on its own, so the SD across repeats can be reported —
    the protocol's repeatability criterion, and the measured size of cogging."""
    rows = [r for r in e9_rows() if r[E9_RUN] == e9["run_id"]]
    xs, ys = e9_pairs(rows)
    f = linfit_se(xs, ys)
    if f:
        e9["slopes"].append(f[0])


def e9_pairs(rows):
    """(tau_truth, tau_est) for the samples that survive the psi filter."""
    xs, ys = [], []
    for r in rows:
        if r[E9_TT] is None or r[E9_TE] is None or r[E9_PSI] is None:
            continue
        if abs(math.sin(math.radians(r[E9_PSI]))) < SIN_PSI_MIN:
            continue
        xs.append(r[E9_TT]); ys.append(r[E9_TE])
    return xs, ys


def e9_start():
    if e9["run"]:
        return
    e9["p"] = e9_read_params()
    with e9["lock"]:
        e9["rows"].clear()
    e9["run"] = True
    e9["pause"] = False
    threading.Thread(target=e9_runner, daemon=True).start()


def e9_continue():
    if not e9["pause"]:
        return
    e9["p"] = e9_read_params()          # pick up the re-measured geometry
    e9["pause"] = False
    e9_set("Continuing with the new geometry.", GREEN)


def e9_stop():
    e9["run"] = False
    e9["pause"] = False
    dev_send("C0")
    stop_all()
    e9_set("Stopped — current commanded to zero.", RED)


def e9_clear():
    if e9["run"]:
        e9_set("Stop the run first.", AMBER)
        return
    if not confirm_reset(len(e9_rows()), "recorded samples"):
        return
    with e9["lock"]:
        e9["rows"].clear()
    e9["slopes"] = []
    e9_analyze()
    e9_set("Cleared.", BLUE)


_e9b = tk.Frame(e9_left, bg=SURFACE); _e9b.pack(fill="x", pady=(8, 0))
btn(_e9b, "▶ RUN", GREEN, "white", e9_start, width=7).pack(side="left", padx=(0, 6))
btn(_e9b, "■ STOP", RED, "white", e9_stop, width=7).pack(side="left")
_e9b2 = tk.Frame(e9_left, bg=SURFACE); _e9b2.pack(fill="x", pady=(6, 0))
btn(_e9b2, "↻ Continue", FIELD, BLUE, e9_continue).pack(side="left", padx=(0, 6))
btn(_e9b2, "↺ Clear", FIELD, MUTED, e9_clear).pack(side="left")
e9_set = mkstatus(e9_left)

# ── middle column · live ──────────────────────────────────────────────────
e9_mid_o, e9_mid = card(e9_fr); e9_mid_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(e9_mid, "Live")
_eb = tk.Frame(e9_mid, bg=SURFACE); _eb.pack(fill="x")
_eb1 = tk.Frame(_eb, bg=SURFACE); _eb1.pack(side="left", expand=True)
e9_big_t = tk.Label(_eb1, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 30, "bold"))
e9_big_t.pack()
tk.Label(_eb1, text="τ_truth = F·r⊥  (N·m)", bg=SURFACE, fg=FAINT, font=F_SM).pack()
_eb2 = tk.Frame(_eb, bg=SURFACE); _eb2.pack(side="left", expand=True)
e9_big_e = tk.Label(_eb2, text="—", bg=SURFACE, fg=BLUE, font=("Consolas", 30, "bold"))
e9_big_e.pack()
tk.Label(_eb2, text="τ_est  (N·m)", bg=SURFACE, fg=FAINT, font=F_SM).pack()
e9_live = tk.Label(e9_mid, text="", bg=SURFACE, fg=MUTED, font=("Consolas", 12),
                   wraplength=360, justify="center")
e9_live.pack(pady=(10, 0))
e9_phase = tk.Label(e9_mid, text="idle", bg=SURFACE, fg=FAINT, font=F_SM,
                    wraplength=360, justify="center")
e9_phase.pack(pady=(4, 0))
e9_cv = tk.Canvas(e9_mid, width=380, height=240, bg=SURFACE, highlightthickness=0)
e9_cv.pack(fill="both", expand=True, pady=(10, 0))

# ── right column · the protocol's plots and its acceptance table ──────────
e9_rsc_o, e9_rsc = scroll_column(e9_fr); e9_rsc_o.pack(side="left", fill="y")
e9_right_o, e9_right = card(e9_rsc); e9_right_o.pack(fill="both", expand=True)
e9_plot_mode = tk.StringVar(value="fit")
_pm = tk.Frame(e9_right, bg=SURFACE); _pm.pack(fill="x")
for _t, _v in (("τ_est vs τ_truth", "fit"), ("residual vs θ", "resid"),
               ("Bland–Altman", "ba")):
    tk.Radiobutton(_pm, text=_t, variable=e9_plot_mode, value=_v,
                   command=lambda: e9_draw_plot(), bg=SURFACE, fg=TEXT,
                   selectcolor=FIELD, font=F_SM, activebackground=SURFACE,
                   highlightthickness=0).pack(anchor="w")
e9_pcv = tk.Canvas(e9_right, width=300, height=225, bg=SURFACE, highlightthickness=0)
e9_pcv.pack(fill="x", pady=(4, 8))
section(e9_right, "Result")
e9_lbl_big = tk.Label(e9_right, text="—", bg=SURFACE, fg=GREEN, font=("Consolas", 26, "bold"))
e9_lbl_big.pack()
tk.Label(e9_right, text="slope of τ_est vs τ_truth", bg=SURFACE, fg=FAINT,
         font=F_SM).pack()
e9_lbl_sub = tk.Label(e9_right, text="run a station to see a result", bg=SURFACE,
                      fg=FAINT, font=F_SM, wraplength=282, justify="left")
e9_lbl_sub.pack(pady=(6, 0))
section(e9_right, "Drive linearity  [g6-D]")
e9_lbl_lin = tk.Label(e9_right, text="—", bg=SURFACE, fg=FAINT, font=F_SM, wraplength=282, justify="left")
e9_lbl_lin.pack(anchor="w")
section(e9_right, "Acceptance  (registered in advance)")
e9_acc = mktable(e9_right, [("k", "metric", 118), ("v", "measured", 92),
                        ("p", "verdict", 66)], height=10)


def e9_draw_xy(cv, pts, xlabel, ylabel, ideal=False, fit=None, band=None,
               hlines=()):
    """Scatter with optional identity line, fitted line, acceptance band and
    horizontal reference lines. `pts` = [(x, y, colour)]."""
    cv.delete("all")
    w = int(cv.winfo_width() or 300); h = int(cv.winfo_height() or 225)
    L, R, T_, B = 52, 12, 16, 30
    cv.create_rectangle(1, 1, w - 1, h - 1, outline=BORDER, fill="#fbfcfe")
    good = [(x, y, c) for (x, y, c) in pts
            if x is not None and y is not None
            and math.isfinite(x) and math.isfinite(y)]
    if len(good) < 2:
        cv.create_text(w / 2, h / 2, text="no data yet", fill=FAINT, font=F_SM)
        return
    xs = [p[0] for p in good]; ys = [p[1] for p in good]
    xmin, xmax = min(xs), max(xs); ymin, ymax = min(ys), max(ys)
    if band:
        ymin = min(ymin, -band); ymax = max(ymax, band)
    for v, _col in hlines:
        ymin = min(ymin, v); ymax = max(ymax, v)
    if ideal:
        lo = min(xmin, ymin); hi = max(xmax, ymax)
        xmin = ymin = lo; xmax = ymax = hi
    if xmax - xmin < 1e-9: xmin -= 1; xmax += 1
    if ymax - ymin < 1e-9: ymin -= 1; ymax += 1
    px = 0.06 * (xmax - xmin); py = 0.10 * (ymax - ymin)
    xmin -= px; xmax += px; ymin -= py; ymax += py

    def X(v): return L + (v - xmin) / (xmax - xmin) * (w - L - R)
    def Y(v): return h - B - (v - ymin) / (ymax - ymin) * (h - B - T_)

    cv.create_line(L, h - B, w - R, h - B, fill=BORDER)
    cv.create_line(L, T_, L, h - B, fill=BORDER)
    if ymin < 0 < ymax:
        cv.create_line(L, Y(0), w - R, Y(0), fill=BORDER, dash=(3, 4))
    if band:
        cv.create_rectangle(L, Y(band), w - R, Y(-band), outline="",
                            fill="#e3f1eb")
        cv.create_line(L, Y(band), w - R, Y(band), fill=GREEN, dash=(4, 3))
        cv.create_line(L, Y(-band), w - R, Y(-band), fill=GREEN, dash=(4, 3))
    for v, col in hlines:
        cv.create_line(L, Y(v), w - R, Y(v), fill=col, dash=(4, 3))
    if ideal:
        cv.create_line(X(xmin), Y(xmin), X(xmax), Y(xmax), fill=MUTED,
                       dash=(5, 4))
        cv.create_text(w - R - 4, Y(xmax) + 10, text="slope 1.00", fill=MUTED,
                       font=F_SM, anchor="ne")
    for x, y, c in good:
        cv.create_oval(X(x) - 2.2, Y(y) - 2.2, X(x) + 2.2, Y(y) + 2.2,
                       fill=c, outline="")
    if fit:
        a, b = fit
        cv.create_line(X(xmin), Y(a * xmin + b), X(xmax), Y(a * xmax + b),
                       fill=BLUE, width=2)
    cv.create_text(L - 5, Y(ymax), text=f"{ymax:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(L - 5, Y(ymin), text=f"{ymin:.3g}", fill=MUTED, font=F_SM, anchor="e")
    cv.create_text(X(xmin), h - B + 5, text=f"{xmin:.3g}", fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text(w - R, h - B + 5, text=f"{xmax:.3g}", fill=MUTED, font=F_SM, anchor="ne")
    cv.create_text(L + 5, T_ - 4, text=ylabel, fill=MUTED, font=F_SM, anchor="nw")
    cv.create_text((L + w - R) / 2, h - 9, text=xlabel, fill=FAINT, font=F_SM)


def e9_draw_plot():
    rows = e9_rows()
    keep = [r for r in rows if r[E9_TT] is not None and r[E9_TE] is not None
            and r[E9_PSI] is not None
            and abs(math.sin(math.radians(r[E9_PSI]))) >= SIN_PSI_MIN]
    mode = e9_plot_mode.get()
    if mode == "fit":
        pts = [(r[E9_TT], r[E9_TE], E9_STAGE_COLOR.get(r[E9_STAGE], MUTED))
               for r in keep]
        fit = e9.get("fit")
        e9_draw_xy(e9_pcv, pts, "τ_truth (N·m)", "τ_est (N·m)", ideal=True,
                   fit=(fit[0], fit[1]) if fit else None)
    elif mode == "resid":
        pts = [(r[E9_THI], r[E9_TE] - r[E9_TT],
                E9_STAGE_COLOR.get(r[E9_STAGE], MUTED))
               for r in keep if r[E9_THI] is not None]
        e9_draw_xy(e9_pcv, pts, "joint angle θ (°)", "τ_est − τ_truth (N·m)",
                   band=0.05)
    else:
        diffs = [(0.5 * (r[E9_TE] + r[E9_TT]), r[E9_TE] - r[E9_TT],
                  E9_STAGE_COLOR.get(r[E9_STAGE], MUTED)) for r in keep]
        if len(diffs) > 2:
            d = [p[1] for p in diffs]
            bias = sum(d) / len(d)
            sd = math.sqrt(sum((x - bias) ** 2 for x in d) / (len(d) - 1))
            lines = ((bias, RED), (bias + 1.96 * sd, MUTED),
                     (bias - 1.96 * sd, MUTED))
        else:
            lines = ()
        e9_draw_xy(e9_pcv, diffs, "mean of the two (N·m)", "difference (N·m)",
                   hlines=lines)


def e9_analyze():
    rows = e9_rows()
    xs, ys = e9_pairs(rows)
    dropped = sum(1 for r in rows if r[E9_PSI] is not None
                  and abs(math.sin(math.radians(r[E9_PSI]))) < SIN_PSI_MIN)

    # sigma_0 and the measured gravity torque come from the baseline samples,
    # which is the whole reason they are recorded as data rather than used as
    # a tare and thrown away.
    base = [r[E9_TT] for r in rows if r[E9_STAGE] == "baseline"
            and r[E9_TT] is not None]
    e9["taug_err"] = e9["taug_model"] = None
    if len(base) > 4:
        m0, sd0 = robust_mean(base)
        e9["sigma0"] = sd0
        e9["taug_meas"] = m0
        # tau_g - tau_g_model, at zero current. "Nothing else is acting" is
        # almost true: Coulomb friction in the transmission stays LATCHED at
        # whatever the last motion left behind, so a single-direction baseline
        # carries gravity plus up to one friction torque. That friction
        # reverses with the direction of the preceding ramp, which is why the
        # protocol runs both directions — averaging the two baselines cancels
        # it, and what is left is gravity alone.
        by_dir = {}
        for r in rows:
            if r[E9_STAGE] == "baseline" and r[E9_TT] is not None:
                by_dir.setdefault(r[E9_DIR], []).append(r[E9_TT])
        if len(by_dir) >= 2:
            means = [robust_mean(v)[0] for v in by_dir.values()]
            m_grav = sum(means) / len(means)
            e9["taug_both"] = True
            # Averaging the two baselines removes the part of the latched
            # friction that is equal and opposite. Half their difference is
            # what was latched — and it is the honest error bar on the
            # gravity number, not something to quietly drop.
            e9["taug_latched"] = (max(means) - min(means)) / 2.0
        else:
            m_grav = m0
            e9["taug_both"] = False
            e9["taug_latched"] = None
        th_base = [r[E9_THI] for r in rows if r[E9_STAGE] == "baseline"
                   and r[E9_THI] is not None]
        if th_base:
            thm = sum(th_base) / len(th_base)
            model = e9_tau_g(thm, e9_p("m", 0.0), e9_p("lcom", 0.0),
                             e9_p("th0", 0.0))
            if model is not None:
                e9["taug_model"] = model
                # the cell holds the arm up, so it reads gravity with the
                # opposite sign to the model's magnitude — compare magnitudes
                e9["taug_err"] = abs(m_grav) - abs(model)
    fit = linfit_se(xs, ys) if len(xs) >= 3 else None
    e9["fit"] = fit
    e9_draw_plot()
    clear_table(e9_acc)
    if not fit:
        e9_lbl_big.config(text="—")
        e9_lbl_sub.config(text=("run a station to see a result" if not rows
                                else f"{len(rows)} samples, not enough inside "
                                     f"the ψ window to fit"), fg=FAINT)
        return
    a, b, r2, se_a, se_b, rmse, n = fit

    # residual against angle, binned by station: a trend here is the error in
    # tau_g(theta), measured rather than argued about.
    bins = {}
    for r in rows:
        if r[E9_TT] is None or r[E9_TE] is None or r[E9_THI] is None:
            continue
        if abs(math.sin(math.radians(r[E9_PSI] or 0.0))) < SIN_PSI_MIN:
            continue
        bins.setdefault(round(r[E9_THI] / 5.0) * 5, []).append(r[E9_TE] - r[E9_TT])
    resid = (max(abs(sum(v) / len(v)) for v in bins.values())
             if len(bins) >= 2 else None)   # one station cannot show a trend

    # hysteresis: the same current on the way up and on the way down, and the
    # gap between them is twice the friction torque.
    # Hysteresis by FITTING each leg, not by binning it. Binning compares
    # the up and down legs at "the same" current, but a bin is 0.3 A wide and
    # the line through it climbs at K_t,j — so a half-bin mismatch in where
    # the samples fell invents tens of mN.m of "friction". Fit tau against Iq
    # on each leg instead: Coulomb friction shifts the two lines apart without
    # tilting them, so the gap between their intercepts is 2*tau_friction, and
    # it does not care where the samples landed. Legs are fitted per
    # DIRECTION: +Ktj*I and -Ktj*I in one fit would be nonsense.
    legs = {}
    for r in rows:
        if r[E9_TT] is None or r[E9_STAGE] not in ("ramp_up", "ramp_dn"):
            continue
        legs.setdefault((r[E9_DIR], r[E9_STAGE]), []).append((r[E9_IQ], r[E9_TT]))
    fric = None
    gaps = []
    for d in {k[0] for k in legs}:
        u = legs.get((d, "ramp_up"), [])
        w = legs.get((d, "ramp_dn"), [])
        if len(u) < 4 or len(w) < 4:
            continue
        fu = linfit([p[0] for p in u], [p[1] for p in u])
        fw = linfit([p[0] for p in w], [p[1] for p in w])
        if fu and fw:
            gaps.append(abs(fu[1] - fw[1]) / 2.0)
    if gaps:
        fric = sum(gaps) / len(gaps)

    # [g6-D] Drive linearity. The strut torque the arm actually receives,
    # tau_truth, fitted against the REPORTED current on each ramp leg, per
    # load direction, over two current bands. A motor+driver cannot make the
    # torque per amp depend on which way the ARM is loaded, nor fall off above
    # 2 A while the rotor stands still — both runs on 13–14 Oct did exactly
    # that, and the cause was a hard contact in the rope path. This readout
    # says so at the end of the run instead of after an offline re-analysis.
    lin = {}
    for d in ("+", "-"):
        # slopes per LEG (up / down), then averaged: latched friction shifts a
        # leg's line without tilting it, so a pooled fit of both legs would
        # flatten the slope by the hysteresis — the leg slopes do not.
        def leg_slope(lo_a, hi_a):
            ks = []
            for st in ("ramp_up", "ramp_dn"):
                pts = [(abs(r[E9_IQ]), r[E9_TT] if d == "+" else -r[E9_TT])
                       for r in rows if r[E9_TT] is not None and r[E9_DIR] == d
                       and r[E9_STAGE] == st and r[E9_IQ] is not None
                       and lo_a <= abs(r[E9_IQ]) <= hi_a]
                if len(pts) >= 10:
                    f = linfit([p[0] for p in pts], [p[1] for p in pts])
                    if f:
                        ks.append(f[0])
            return (sum(ks) / len(ks)) if ks else None
        k_all = leg_slope(0.8, 3.3)
        if k_all is None:
            continue
        lin[d] = dict(k=k_all, k_lo=leg_slope(1.0, 2.0), k_hi=leg_slope(2.0, 3.3),
                      fmax=max(abs(r[E9_F]) for r in rows if r[E9_F] is not None and r[E9_DIR] == d))
    e9["lin"] = lin
    lin_bits, lin_warn = [], []
    k_dir_ok = k_sat_ok = None
    for d, v in lin.items():
        if v["k"] is None:
            continue
        seg = f"dir {d}: K_t,j {v['k']:.3f} N·m/A"
        if v["k_lo"] and v["k_hi"]:
            seg += f" (1–2 A {v['k_lo']:.2f} · 2–3 A {v['k_hi']:.2f})"
            ratio = v["k_hi"] / v["k_lo"] if v["k_lo"] else 1.0
            k_sat_ok = (k_sat_ok is not False) and ratio >= 0.85
            if ratio < 0.85:
                lin_warn.append(f"dir {d}: torque per amp drops {100*(1-ratio):.0f} % above 2 A while the rotor "
                                f"stands still — hard contact in the rope path (termination / flange / rod)?")
        seg += f" · F_max {v['fmax']:.1f} N"
        lin_bits.append(seg)
    if len(lin) == 2 and all(v["k"] for v in lin.values()):
        kp, km = lin["+"]["k"], lin["-"]["k"]
        diff = abs(kp - km) / max(abs((kp + km) / 2.0), 1e-9) * 100.0   # [g6-D1] abs: a negative mean gave a negative %
        k_dir_ok = diff <= 5.0
        if kp < 0 and km < 0:                                             # [g6-D1]
            lin_warn.append("K_t,j is NEGATIVE in both directions — the force sign is inverted "
                            "(flip 'force sign' and re-run)")
        kmean = (kp + km) / 2.0
        lin_bits.append(f"+ vs − differ by {diff:.1f} % → mean K_t,j {kmean:.3f} N·m/A "
                        f"(entered {e9_ktj():.3f}; N·η = {kmean/max(e9_p('kt', 105.3)/1000.0, 1e-6):.2f})")
        if diff > 5.0:
            lin_warn.append("the two load directions disagree — the drive, not the motor: "
                            "check the strut/cell with dead weights, then the rope terminations")
    e9_lbl_lin.config(text=(" · ".join(lin_bits) + ("\n⚠ " + "\n⚠ ".join(lin_warn) if lin_warn else "")) if lin_bits else "—",
                      fg=(RED if lin_warn else FAINT))

    sd_slope = None
    if len(e9["slopes"]) > 1:
        ms = sum(e9["slopes"]) / len(e9["slopes"])
        sd_slope = math.sqrt(sum((s - ms) ** 2 for s in e9["slopes"])
                             / (len(e9["slopes"]) - 1))

    e9_lbl_big.config(text=f"{a:.3f}")
    bits = [f"± {se_a:.3f} (SE)", f"b = {b:+.3f} ± {se_b:.3f} N·m",
            f"R² = {r2:.4f}", f"RMSE = {rmse:.3f} N·m", f"{n} samples"]
    if dropped:
        bits.append(f"{dropped} dropped by the |sin ψ| ≥ {SIN_PSI_MIN} filter")
    if fric is not None:
        bits.append(f"friction {fric*1000:.0f} mN·m (half the hysteresis)")
    if e9["taug_meas"] is not None:
        bits.append(f"τ_g measured {e9['taug_meas']:+.3f} N·m at Iq=0")
        if e9["taug_err"] is not None:
            pct = (100.0 * e9["taug_err"] / abs(e9["taug_model"])
                   if e9["taug_model"] else float("nan"))
            # The friction caveat is UNCONDITIONAL whenever there is friction
            # to have. Two directions bound only the part that reverses; a
            # friction offset common to both baselines is invisible to that
            # difference, so equal baselines are consistency, not proof. The
            # band that is actually defensible is the friction measured from
            # the hysteresis.
            if fric:
                note = (f" · carries up to ±{fric*1000:.0f} mN·m of latched "
                        f"friction (the hysteresis half-width)")
                if e9.get("taug_both") and e9.get("taug_latched") is not None:
                    note += (f"; the two directions' baselines differ by "
                             f"{2*e9['taug_latched']*1000:.0f} mN·m, which "
                             f"bounds only the part that reverses")
            else:
                note = (" · no friction measured yet — run a full ramp to put "
                        "an error bar on this")
            bits.append(f"gravity model off by {e9['taug_err']:+.3f} N·m "
                        f"({pct:+.1f}%) — mass, CoM or θ₀{note}")
    e9_lbl_sub.config(text=" · ".join(bits), fg=FAINT)

    dz = e9_p("dz", 0.1)
    s0 = e9["sigma0"]
    checks = [
        ("slope", f"{a:.3f} ± {se_a:.3f}", abs(a - 1.0) <= 0.05),
        ("|intercept|", f"{abs(b):.3f} N·m", abs(b) <= 0.05),
        ("R²", f"{r2:.4f}", r2 >= 0.99),
        ("RMSE", f"{rmse:.3f} N·m", rmse <= 0.10),
        ("residual vs θ",
         (f"{len(bins)} station" + ("s" if len(bins) != 1 else "") +
          " — need ≥2") if resid is None else f"{resid:.3f} N·m",
         None if resid is None else resid <= 0.05),
        ("dead-zone ≥ 3σ₀", "—" if s0 is None else f"3σ₀ = {3*s0:.3f} N·m",
         None if s0 is None else dz >= 3 * s0),
        ("SD of slope", "—" if sd_slope is None else f"{sd_slope:.3f}",
         None if sd_slope is None else sd_slope <= 0.02),
        ("K_t,j + vs − (≤5 %)", "—" if k_dir_ok is None else
         f"{abs(lin['+']['k'] - lin['-']['k']) / ((lin['+']['k'] + lin['-']['k']) / 2) * 100:.1f} %", k_dir_ok),
        ("linear to 3 A (≥0.85)", "—" if k_sat_ok is None else
         " / ".join(f"{v['k_hi']/v['k_lo']:.2f}" for v in lin.values() if v.get('k_lo') and v.get('k_hi')), k_sat_ok),
    ]
    for name, val, ok in checks:
        e9_acc.insert("", "end",
                      values=(name, val, "—" if ok is None else
                              ("PASS" if ok else "FAIL")))


def e9_save():
    rows = e9_rows()
    if not rows:
        e9_set("Nothing recorded.", AMBER)
        return
    path = log_path("E9-groundtruth")
    if not path:
        e9_set("Type the motor serial no. in the header — it names the file.",
               AMBER)
        return
    P = e9["p"] or {}
    fit = e9.get("fit")
    try:
        with open(path, "w", newline="") as f:
            f.write("# E9 stage 1 — isometric current ramp against a load cell "
                    "in a pinned strut\n")
            f.write(arm_cal_line() + "\n")
            f.write(f"# geometry: L_mm={P.get('L')} x_B_mm={P.get('xB')} "
                    f"y_B_mm={P.get('yB')} angle_id={P.get('angle_id')} "
                    f"axis={P.get('axis')} force_sign="
                    f"{'-' if P.get('fsign', 1) < 0 else '+'}\n")
            f.write(f"# estimator: Kt_mNm_A={P.get('kt')} N={P.get('N')} "
                    f"eta={P.get('eta')} Ktj_Nm_A={e9_ktj():.6f} "
                    f"m_kg={P.get('m')} l_m={P.get('lcom')} "
                    f"theta0_deg={P.get('th0')} deadzone_Nm={P.get('dz')}\n")
            f.write(f"# run: Imax_A={P.get('imax')} ramp_s={P.get('ramp_s')} "
                    f"baseline_s={P.get('base_s')} repeats={P.get('reps')} "
                    f"dir={P.get('direction')}\n")
            f.write("# NOTE: force is ABSOLUTE (the arm's weight is not tared "
                    "out); tau_est_direct and tau_est_dob are intentionally "
                    "empty — recompute them offline from I_q, theta, omega\n")
            if fit:
                f.write(f"# fit: slope={fit[0]:.5f} se={fit[3]:.5f} "
                        f"intercept={fit[1]:.5f} se={fit[4]:.5f} "
                        f"r2={fit[2]:.5f} rmse={fit[5]:.5f} n={fit[6]}\n")
            if e9["sigma0"] is not None:
                f.write(f"# sigma0_Nm={e9['sigma0']:.5f} "
                        f"tau_g_measured_Nm={e9['taug_meas']:.5f}"
                        + ("" if e9["taug_err"] is None else
                           f" tau_g_model_Nm={e9['taug_model']:.5f} "
                           f"tau_g_error_Nm={e9['taug_err']:.5f}") + "\n")
            wr = csv.writer(f)
            wr.writerow(E9_COLS)
            for r in rows:
                wr.writerow([
                    f"{r[E9_TPC]:.3f}", f"{r[E9_TMCU]:.3f}", r[E9_RUN],
                    r[E9_ANG], r[E9_DIR], r[E9_STAGE],
                    "" if r[E9_THI] is None else f"{r[E9_THI]:.3f}",
                    "" if r[E9_THE] is None else f"{r[E9_THE]:.3f}",
                    f"{r[E9_XB]:.2f}", f"{r[E9_YB]:.2f}",
                    "" if r[E9_PSI] is None else f"{r[E9_PSI]:.2f}",
                    "" if r[E9_RP] is None else f"{r[E9_RP]:.5f}",
                    f"{r[E9_IQ]:.4f}", f"{r[E9_OM]:.4f}",
                    "" if r[E9_F] is None else f"{r[E9_F]:.4f}",
                    "" if r[E9_TT] is None else f"{r[E9_TT]:.5f}",
                    "" if r[E9_TE] is None else f"{r[E9_TE]:.5f}",
                    "", ""])
        e9["path"] = path
    except Exception as e:
        e9_set(f"Could not save: {e}", RED)
        return
    e9_set(f"Saved {len(rows)} samples → {path}", GREEN)


def e9_tick():
    if not e9["run"]:
        e9["p"] = e9_read_params()
    b = snap_arm()
    s = b[-1] if b else None
    theta = e9_theta(s) if s else None
    r_perp, psi = e9_geometry(theta, e9_p("L"), e9_p("xB"), e9_p("yB"))
    grams = arm_grams(s) if s else None
    force = None if grams is None else e9_p("fsign", 1.0) * grams * 1e-3 * G_ACC
    tau_t = None if (force is None or r_perp is None) else force * r_perp
    tg = e9_tau_g(theta, e9_p("m", 0.0), e9_p("lcom", 0.0), e9_p("th0", 0.0))
    tau_e = None if tg is None else e9_ktj() * T["iq"] - tg
    e9_big_t.config(text="—" if tau_t is None else f"{tau_t:+.3f}")
    e9_big_e.config(text="—" if tau_e is None else f"{tau_e:+.3f}")
    e9_live.config(text=(f"θ {'--' if theta is None else f'{theta:+.2f}'}°   "
                         f"F {'--' if force is None else f'{force:+.2f}'} N   "
                         f"Iq {T['iq']:+.2f} A\n"
                         f"K_t,j = {e9_ktj():.4f} N·m/A   "
                         f"τ_g model {'--' if tg is None else f'{tg:+.3f}'} N·m"))
    if psi is None:
        e9_geo.config(text="ψ —   r⊥ —   (no angle)", fg=FAINT)
    else:
        sinp = abs(math.sin(math.radians(psi)))
        col = (GREEN if 75 <= psi <= 105 else
               AMBER if 60 <= psi <= 120 else RED)
        note = ("" if sinp >= SIN_PSI_MIN else
                "  — samples here are DISCARDED")
        e9_geo.config(text=f"ψ = {psi:.1f}°   r⊥ = {r_perp*1000:+.1f} mm   "
                           f"(L·sin ψ = {e9_p('L', 0.0)*sinp:.1f} mm){note}",
                      fg=col)
    e9_phase.config(text=("● " + e9["phase"] + f"   rep {e9['run_id']}   "
                          f"{len(e9_rows())} samples")
                    if e9["run"] else
                    ("PAUSED — re-measure, then Continue" if e9["pause"] else
                     f"idle · {len(e9_rows())} samples"
                     + (f" · {os.path.basename(e9['path'])}" if e9["path"] else "")),
                    fg=AMBER if e9["pause"] else (GREEN if e9["run"] else FAINT))
    now = time.time()
    rows = [r for r in e9_rows() if r[E9_TPC] >= (now - e9["t0"]) - 60.0] if e9["t0"] else []
    draw_ts(e9_cv,
            [(e9["t0"] + r[E9_TPC], r[E9_TT]) for r in rows if r[E9_TT] is not None],
            GREEN, "τ_truth and current, last 60 s",
            y2=[(e9["t0"] + r[E9_TPC], r[E9_IQ]) for r in rows],
            unit1="N·m", unit2="A", label1="τ_truth", label2="Iq", signed=True)


add_tab("10 · E9 · Ground truth", e9_fr)
e9_analyze()



# ═════════════ TAB 11 · E10 · KINEMATIC RATIO N [g6-A] ═════════════
# The one number every torque figure in this project is divided by, and the
# one that was never measured. N is a ratio of ANGLES — nothing else — so it
# is measured from the two angle sensors already on the bench: the AS5047P on
# the motor (telemetry `angle`, unwrapped across turns by SimpleFOC) and the
# IMU on the arm (tab 8's axis/sign/zero), on one PC clock.
#
# The arm must be FREE (E9 strut off). The motor is stepped in VEL mode:
#   V±v for `step` s  →  V0 for `dwell` s (the velocity loop holds the arm)
# and each dwell contributes ONE point: the mean of both angles over the last
# `avg` seconds. Fitting only the dwell means is what keeps the IMU's lag out
# of the result — a moving sample pairs a motor angle with a joint angle that
# is 50–100 ms stale, and at 3 °/s that is a 0.3° bias with the sign of the
# direction of travel, i.e. it would masquerade as backlash.
#
# Outputs: N per direction ± SE, pooled N, the forward/reverse offset at equal
# motor angle (backlash + rope creep, in joint degrees), and the file
# labdata/<serial>_E10-ratio_<stamp>.csv with EVERY sample plus one
# `dwell_mean` row per dwell — the format tools/analysis/.../e10_ratio.py reads.
#
# "Log by hand" is Method A with a data record: firmware IDLE, both angles
# logged while the rotor is turned by hand; the same fit applies.
e10_fr = tk.Frame(body, bg=BG)
e10 = dict(run=False, manual=False, rows=[], dwells=[], lock=threading.Lock(),
           t0=0.0, last_arm=0.0, f=None, wr=None, path="", hot=0.0, phase="",
           cycle=0, direction=0, ndwell=0, result="", prev_m=None,
           hand_rest=False, hand_last_m=None)
E10_COLS = ["t_s", "cycle", "direction", "phase", "vel_cmd_rad_s",
            "motor_deg", "joint_deg", "iq_meas_A", "state"]

e10_sc_o, e10_sc = scroll_column(e10_fr); e10_sc_o.pack(side="left", fill="y", padx=(0, 14))
e10_l_o, e10_l = card(e10_sc); e10_l_o.pack(fill="both", expand=True)
section(e10_l, "Setup · arm FREE (E9 strut off) · IMU zeroed in tab 9")
e10_vel = entry_row(e10_l, "Motor speed (rad/s, 0.5–1.5)", 1.0)
e10_step = entry_row(e10_l, "Move per step (s)", 0.6)
e10_dwell = entry_row(e10_l, "Dwell per step (s)", 2.0)
e10_avg = entry_row(e10_l, "Average last (s) of dwell", 1.2)
e10_jmin = entry_row(e10_l, "Joint min (°)", -65.0)
e10_jmax = entry_row(e10_l, "Joint max (°)", 65.0)
e10_cyc = entry_row(e10_l, "Cycles (fwd + rev)", 3)
e10_ilim = entry_row(e10_l, "Current limit Pi (A)", 2.0)
e10_vdir = tk.BooleanVar(value=True)
tk.Checkbutton(e10_l, text="+V raises the joint angle", variable=e10_vdir, bg=SURFACE,
               fg=TEXT, selectcolor=FIELD, activebackground=SURFACE, font=F_LBL).pack(anchor="w", pady=(2, 6))
tk.Label(e10_l, text="Joint angle uses tab 8's axis, sign and zero.\nStops itself if the joint leaves the window by > 5°,\n"
                     "if Iq sits at the limit > 1 s, or if a stream goes silent.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 8))
section(e10_l, "Velocity loop for the sweep  [g6-B]")
tk.Label(e10_l, text="The lab firmware leaves SimpleFOC's defaults (P 0.5, I 10,\nTf 5 ms) — tuned for a bare motor at speed. At ≤ 1 rad/s the\n"
                     "encoder velocity is a pulse train, so that P chatters the\ncurrent and rings the capstan + arm. The sweep sends these\n"
                     "(MVP / MVI / MVF / MVR) before the first step.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 6))
e10_vp = entry_row(e10_l, "Velocity P (A per rad/s)", 0.25)
e10_vi = entry_row(e10_l, "Velocity I", 3.0)
e10_vf = entry_row(e10_l, "Velocity LPF Tf (s)", 0.04)
e10_vr = entry_row(e10_l, "Output ramp MVR (A/s, 0 = off)", 5.0)
e10_hold = tk.BooleanVar(value=False)
tk.Checkbutton(e10_l, text="Dwell in HOLD (H) instead of V0", variable=e10_hold, bg=SURFACE,
               fg=TEXT, selectcolor=FIELD, activebackground=SURFACE, font=F_LBL).pack(anchor="w", pady=(2, 0))
tk.Label(e10_l, text="OFF by default [g6-C]: HOLD (Pp 8 A/rad) was tuned on the\nE9 strut. On the FREE arm any error > 1.6° joint saturates\n"
                     "it at ±Pi → bang-bang limit cycle (20:10 run). Leave off\nunless Pp/Pk have been re-tuned for the free arm.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(0, 8))
btn(e10_l, "⚙ Send velocity gains now", FIELD, BLUE,
    lambda: (e10_send_velocity_gains(), e10_set("Velocity gains sent (also re-sent automatically at each sweep).", GREEN)),
    ).pack(fill="x", pady=(2, 8))
section(e10_l, "Run")
e10_set = mkstatus(e10_l)


def e10_val(e, default):
    try:
        return float(e.get())
    except ValueError:
        return default


def e10_joint(s):
    """Arm-board sample → joint angle in degrees (tab 8 convention)."""
    return ar_angle(s[ar_axis_idx()])


def e10_status(msg, colour=BLUE):
    root.after(0, lambda: e10_set(msg, colour))


def e10_open_log():
    path = log_path("E10-ratio")
    if not path:
        return False
    f = open(path, "w", newline="")
    f.write("# E10 kinematic ratio: motor angle (AS5047P telemetry, unwrapped, deg) vs joint angle "
            "(arm-board IMU, tab-8 axis/sign/zero), one PC clock\n")
    f.write(f"# axis={ar_axis_key()} sign={'-' if ar_inv.get() else '+'} zero_deg={arm_zero[ar_axis_key()]:.3f} "
            f"gain={arm_gain[ar_axis_key()]:.5f} | vel={e10_val(e10_vel, 0.4)} step_s={e10_val(e10_step, 1.5)} "
            f"dwell_s={e10_val(e10_dwell, 2.0)} avg_s={e10_val(e10_avg, 1.2)} jmin={e10_val(e10_jmin, -65)} "
            f"jmax={e10_val(e10_jmax, 65)} cycles={int(e10_val(e10_cyc, 3))} ilim={e10_val(e10_ilim, 2.0)} "
            f"mode={'manual' if e10['manual'] else 'stepped'} "
            f"velPID P={e10_val(e10_vp, 0.05)} I={e10_val(e10_vi, 1.0)} Tf={e10_val(e10_vf, 0.08)} ramp={e10_val(e10_vr, 5.0)} "
            f"dwell_mode={'hold' if e10_hold.get() else 'v0'}\n")
    wr = csv.writer(f)
    wr.writerow(E10_COLS)
    e10["f"], e10["wr"], e10["path"] = f, wr, path
    return True


def e10_close_log(reason):
    f = e10["f"]
    if f:
        try:
            f.write(f"# stop: {reason}\n"); f.close()
        except Exception:
            pass
    e10["f"] = e10["wr"] = None


def e10_collect(phase, duration, vcmd):
    """Log one row per NEW arm-board sample for `duration` s. Returns "" or the stop reason."""
    e10["phase"] = phase
    t_end = time.time() + duration
    jmin, jmax = e10_val(e10_jmin, -65), e10_val(e10_jmax, 65)
    ilim = e10_val(e10_ilim, 2.0)
    while time.time() < t_end:
        if not e10["run"]:
            return "stopped"
        if not e10["manual"]:
            if not dev_alive():
                return "telemetry silent — driver link lost"
            if not arm_alive(2.0):
                return "arm board silent > 2 s"
        for s in snap_arm():
            if s[0] <= e10["last_arm"]:
                continue
            e10["last_arm"] = s[0]
            j = e10_joint(s)
            m_rad = hist_at(ang_hist, s[0])
            if m_rad is None:
                m_rad = T["ang"]
            if j is None or m_rad is None:
                continue
            m_deg = math.degrees(m_rad)
            dm = 0.0 if e10["prev_m"] is None else m_deg - e10["prev_m"]
            e10["prev_m"] = m_deg
            if not e10["manual"] and abs(dm) > 30.0:                       # [g6-C] > 5 rad/s during a 1 rad/s step
                return f"runaway — motor jumped {dm:+.0f}° in one sample (stick-slip burst / unstable loop) — released"
            iq = hist_at(iq_hist, s[0]); iq = T["iq"] if iq is None else iq
            row = (s[0] - e10["t0"], e10["cycle"], e10["direction"], phase, vcmd, m_deg, j, iq, T["state"])
            with e10["lock"]:
                e10["rows"].append(row)
            if e10["wr"]:
                e10["wr"].writerow([f"{row[0]:.3f}", row[1], f"{row[2]:+d}", row[3], f"{row[4]:.3f}",
                                    f"{row[5]:.3f}", f"{row[6]:.2f}", f"{row[7]:.3f}", row[8]])
            if not e10["manual"] and not (jmin - 5.0 <= j <= jmax + 5.0):
                return f"joint {j:+.1f}° is outside the window by > 5° — hard stop reached?"
        if not e10["manual"]:
            if abs(T["iq"]) >= 0.95 * ilim:
                e10["hot"] += 0.05
                if e10["hot"] > 1.0:
                    return "Iq sat at the limit for > 1 s — hard stop or jam"
            else:
                e10["hot"] = 0.0
        time.sleep(0.05)
    return ""


def e10_dwell_point(phase="dwell"):
    """Mean of both angles over the last `avg` s of rows in `phase` → one dwell_mean row."""
    avg = e10_val(e10_avg, 1.2)
    with e10["lock"]:
        rows = [r for r in e10["rows"] if r[3] == phase]
    if not rows:
        return
    t_now = rows[-1][0]
    pts = [r for r in rows if r[0] >= t_now - avg]
    if len(pts) < 3:
        return
    m = robust_mean([r[5] for r in pts])[0]; j = robust_mean([r[6] for r in pts])[0]
    e10["ndwell"] += 1
    d = (t_now, e10["cycle"], e10["direction"], m, j, len(pts))
    with e10["lock"]:
        e10["dwells"].append(d)
    if e10["wr"]:
        e10["wr"].writerow([f"{t_now:.3f}", e10["cycle"], f"{e10['direction']:+d}", "dwell_mean", "0.000",
                            f"{m:.3f}", f"{j:.2f}", "", e10["ndwell"]])


def e10_latest_joint():
    r, p, y, _g = arm_latest(0.5)
    return ar_angle((None, r, p, y)[ar_axis_idx()])


def e10_send_velocity_gains():
    """[g6-B] SimpleFOC Commander motor passthrough: MVP/MVI velocity PID, MVF its
    input LPF, MVR its output ramp (A/s). Volatile — resent on every sweep."""
    vp, vi, vf, vr = (e10_val(e10_vp, 0.05), e10_val(e10_vi, 1.0),
                      e10_val(e10_vf, 0.08), e10_val(e10_vr, 5.0))
    dev_send(f"MVP{vp:.4f}"); time.sleep(0.05)
    dev_send(f"MVI{vi:.4f}"); time.sleep(0.05)
    dev_send(f"MVF{vf:.4f}"); time.sleep(0.05)
    dev_send(f"MVR{vr:.2f}" if vr > 0 else "MVR0"); time.sleep(0.05)
    return vp, vi, vf, vr


def e10_runner():
    vel = abs(e10_val(e10_vel, 1.0)); step = e10_val(e10_step, 1.5); dwell = e10_val(e10_dwell, 2.0)
    jmin, jmax = e10_val(e10_jmin, -65), e10_val(e10_jmax, 65); cycles = max(1, int(e10_val(e10_cyc, 3)))
    ilim = e10_val(e10_ilim, 2.0)
    if vel > 1.5:
        e10_status("Keep the motor speed ≤ 1.5 rad/s (joint ≤ ~10°/s).", AMBER); e10["run"] = False; return
    if vel < 0.3:
        e10_status("Below 0.3 rad/s the encoder velocity is a pulse train and the loop chatters — use 0.5–1.5.", AMBER); e10["run"] = False; return
    if jmax - jmin < 20:
        e10_status("The joint window must span at least 20°.", AMBER); e10["run"] = False; return
    if not dev_alive():
        e10_status("No telemetry from the driver — connect it first.", RED); e10["run"] = False; return
    if not arm_alive():
        e10_status("No sensor board — connect it in the header.", RED); e10["run"] = False; return
    j0 = e10_latest_joint()
    if j0 is None:
        e10_status(f"{ar_axis_key()} is nan — the IMU is not streaming.", RED); e10["run"] = False; return
    if not (jmin <= j0 <= jmax):
        e10_status(f"Joint is at {j0:+.1f}°, outside [{jmin:.0f}, {jmax:.0f}]. Move the arm into the window by hand first.", AMBER)
        e10["run"] = False; return
    if not e10_open_log():
        e10_status("Enter a motor serial in the header first.", AMBER); e10["run"] = False; return
    e10["t0"] = time.time(); e10["last_arm"] = time.time(); e10["hot"] = 0.0; e10["ndwell"] = 0
    dev_send("O0"); time.sleep(0.3); dev_send(f"Pi{ilim:.2f}"); time.sleep(0.1)
    e10_send_velocity_gains(); time.sleep(0.1)                 # [g6-B]
    dwell_cmd = "H" if e10_hold.get() else "V0"
    dev_send("V0"); time.sleep(0.4)
    e10["last_arm"] = time.time(); e10["prev_m"] = None     # [g6-C] don't log the setup second as "move"
    reason = ""
    try:
        for cyc in range(1, cycles + 1):
            for direction in (+1, -1):
                e10["cycle"], e10["direction"] = cyc, direction
                target = jmax if direction > 0 else jmin
                v = vel * direction * (1.0 if e10_vdir.get() else -1.0)
                n = 0
                while e10["run"]:
                    j = e10_latest_joint()
                    if j is not None and ((direction > 0 and j >= target) or (direction < 0 and j <= target)):
                        break
                    e10_status(f"cycle {cyc}/{cycles} · {'forward' if direction > 0 else 'reverse'} · step {n + 1} · joint {j if j is None else round(j, 1)}°", BLUE)
                    dev_send(f"V{v:.3f}")
                    reason = e10_collect("move", step, v)
                    if reason: break
                    dev_send(dwell_cmd)                           # [g6-B] H: tuned position hold, no velocity-integrator limit cycle
                    reason = e10_collect("dwell", dwell, 0.0)
                    if reason: break
                    e10_dwell_point(); n += 1
                    if n > 400:
                        reason = "too many steps — check the '+V raises joint' box or the window"; break
                if reason: break
            if reason: break
    finally:
        dev_send("V0"); dev_send("O0")
        e10_close_log(reason or "done")
        e10["run"] = False
        e10_analyze()
        if reason and reason != "stopped":
            e10_status("STOPPED: " + reason + f"  ·  saved {e10['path']}", RED)
        else:
            e10_status(f"Done — {e10['ndwell']} dwell points · saved {e10['path']}", GREEN)


def e10_hand_still(avg):
    """[g6-C] True when both angles have been still for the last `avg` s."""
    with e10["lock"]:
        rows = list(e10["rows"][-80:])
    if len(rows) < 3:
        return False, None
    t_now = rows[-1][0]
    pts = [r for r in rows if r[0] >= t_now - avg]
    if len(pts) < 3 or (t_now - pts[0][0]) < 0.8 * avg:
        return False, None
    ms = [r[5] for r in pts]; js = [r[6] for r in pts]
    still = (max(ms) - min(ms)) < 0.6 and (max(js) - min(js)) < 0.25
    return still, sum(ms) / len(ms)


def e10_hand_runner():
    """[g6-C] Method B without the controller: firmware IDLE, YOU move the ARM mark to
    mark (up sweep, then down sweep, repeat). Both angles are logged continuously; each
    time they are still for `avg` s one dwell_mean point is recorded, with direction =
    sign of the motion since the previous point. Same CSV / same fit as the motor sweep."""
    if not dev_alive():
        e10_status("No telemetry from the driver — connect it (it supplies the motor angle).", RED); e10["run"] = False; return
    if not arm_alive():
        e10_status("No sensor board — connect it in the header.", RED); e10["run"] = False; return
    if not e10_open_log():
        e10_status("Enter a motor serial in the header first.", AMBER); e10["run"] = False; return
    avg = e10_val(e10_avg, 1.2); jmin, jmax = e10_val(e10_jmin, -65), e10_val(e10_jmax, 65)
    dev_send("O0")
    e10["t0"] = time.time(); e10["last_arm"] = time.time(); e10["cycle"] = 1; e10["direction"] = 0
    e10["ndwell"] = 0; e10["hand_rest"] = False; e10["hand_last_m"] = None
    e10_status("HAND SWEEP (firmware IDLE). Move the ARM to the first mark and hold still…", BLUE)
    try:
        while e10["run"]:
            e10_collect("hand", 0.25, 0.0)
            still, m_now = e10_hand_still(avg)
            if not still:
                if e10["hand_rest"]:
                    e10_status(f"moving… ({e10['ndwell']} points) — hold still ≥ {avg:.1f} s at the next mark", BLUE)
                e10["hand_rest"] = False
                continue
            if e10["hand_rest"]:
                continue                                   # already recorded this stop
            # a new stop: direction from the motion since the previous point
            if e10["hand_last_m"] is None:
                j = e10_latest_joint()
                direction = +1 if (j is None or abs(j - jmin) <= abs(j - jmax)) else -1
            else:
                dm = m_now - e10["hand_last_m"]
                if abs(dm) < 2.0:
                    continue                               # did not really move — same mark
                direction = +1 if dm > 0 else -1
                if direction == +1 and e10["direction"] == -1:
                    e10["cycle"] += 1                      # reversed again → next cycle
            e10["direction"] = direction
            e10_dwell_point("hand")
            e10["hand_rest"] = True; e10["hand_last_m"] = m_now
            e10_status(f"point #{e10['ndwell']} recorded (cycle {e10['cycle']}, {'up' if direction > 0 else 'down'}) — move to the next mark", GREEN)
    finally:
        e10_close_log("hand sweep done"); e10["run"] = False; e10_analyze()
        e10_status(f"Hand sweep saved {e10['path']} — {e10['ndwell']} points", GREEN)


def e10_manual_runner():
    if not arm_alive():
        e10_status("No sensor board — connect it in the header.", RED); e10["run"] = False; return
    if not e10_open_log():
        e10_status("Enter a motor serial in the header first.", AMBER); e10["run"] = False; return
    dev_send("O0")
    e10["t0"] = time.time(); e10["last_arm"] = time.time(); e10["cycle"] = 0; e10["direction"] = 0
    e10_status("Logging by hand (firmware IDLE). Turn the ROTOR — not the arm — through whole turns, pause 2 s at each mark. Press Stop when done.", BLUE)
    try:
        while e10["run"]:
            e10_collect("manual", 0.5, 0.0)
    finally:
        e10_close_log("manual done"); e10["run"] = False; e10_analyze()
        e10_status(f"Manual log saved {e10['path']} — fit uses all samples (turn slowly, pause at marks).", GREEN)


def e10_start(mode="sweep"):
    """mode: "sweep" (motor-driven), "hand" (IDLE, auto stop-detect), "manual" (IDLE, rotor by hand)."""
    if e10["run"]:
        return
    with e10["lock"]:
        e10["rows"].clear(); e10["dwells"].clear()
    e10["result"] = ""; e10["manual"] = (mode != "sweep"); e10["run"] = True; e10["prev_m"] = None
    target = {"sweep": e10_runner, "hand": e10_hand_runner, "manual": e10_manual_runner}[mode]
    threading.Thread(target=target, daemon=True).start()


def e10_stop():
    e10["run"] = False
    dev_send("V0"); stop_all()
    e10_set("Stopped — motor released to IDLE.", RED)


def e10_clear():
    if e10["run"]:
        e10_set("Stop the run first.", AMBER); return
    if not confirm_reset(len(e10["rows"]), "recorded samples"):
        return
    with e10["lock"]:
        e10["rows"].clear(); e10["dwells"].clear()
    e10["result"] = ""; e10_result.config(text="—"); e10_set("Cleared.", MUTED)


btn(e10_l, "✋  Hand sweep — move the ARM, auto-record stops (IDLE)", GREEN, "white", lambda: e10_start("hand")).pack(fill="x", pady=(4, 4))
btn(e10_l, "▶  Motor-driven stepped sweep (VEL)", FIELD, BLUE, lambda: e10_start("sweep")).pack(fill="x", pady=(0, 4))
btn(e10_l, "✎  Log by hand — turn the ROTOR (IDLE, Method A)", FIELD, BLUE, lambda: e10_start("manual")).pack(fill="x", pady=(0, 4))
btn(e10_l, "■  STOP  (V0 → O0)", RED, "white", e10_stop).pack(fill="x", pady=(0, 4))
btn(e10_l, "Clear points", FIELD, MUTED, e10_clear).pack(fill="x", pady=(0, 4))

# middle: live values + motor-vs-joint scatter
e10_m_o, e10_m = card(e10_fr); e10_m_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section(e10_m, "Live · motor angle (AS5047P) vs joint angle (IMU)")
_e10_live = tk.Frame(e10_m, bg=SURFACE); _e10_live.pack(fill="x")
_c1 = tk.Frame(_e10_live, bg=SURFACE); _c1.pack(side="left", expand=True)
e10_v_motor = big_value(_c1, "motor (°)")
_c2 = tk.Frame(_e10_live, bg=SURFACE); _c2.pack(side="left", expand=True)
e10_v_joint = big_value(_c2, "joint (°)", GREEN)
_c3 = tk.Frame(_e10_live, bg=SURFACE); _c3.pack(side="left", expand=True)
e10_v_ratio = big_value(_c3, "Δmotor / Δjoint since start", PURPLE)
e10_phase = tk.Label(e10_m, text="", bg=SURFACE, fg=MUTED, font=F_LBL); e10_phase.pack(anchor="w", pady=(6, 4))
e10_cv = tk.Canvas(e10_m, width=620, height=300, bg=SURFACE, highlightthickness=0); e10_cv.pack(fill="both", expand=True)

# right: result
e10_r_o, e10_r = card(e10_fr); e10_r_o.pack(side="left", fill="y")
section(e10_r, "Result · θ_motor = N · θ_joint + c  (dwell points)")
e10_result = tk.Label(e10_r, text="—", bg=SURFACE, fg=TEXT, font=("Consolas", 11), justify="left", anchor="w")
e10_result.pack(anchor="w", fill="x")
btn(e10_r, "Re-fit", FIELD, BLUE, lambda: e10_analyze()).pack(fill="x", pady=(10, 4))
tk.Label(e10_r, text="Pass if:  R² ≥ 0.9999 · N_fwd vs N_rev ≤ 0.5 % ·\ndrift of c between cycles ≤ 0.2° joint.\n"
                     "η_capstan = 7.90 / N  (E7 slope ratio).\nResidual plot with a 360°/N ripple = small-drum\neccentricity; a slow curve = rope creep.",
         bg=SURFACE, fg=FAINT, font=F_SM, justify="left").pack(anchor="w", pady=(10, 0))


def e10_points():
    """(joint, motor, direction) points for fitting: dwell means, or every 5th raw row in manual mode."""
    with e10["lock"]:
        d = list(e10["dwells"]); rows = list(e10["rows"])
    if len(d) >= 6:
        return [(x[4], x[3], x[2]) for x in d]
    return [(r[6], r[5], r[2]) for r in rows[::5]]


def e10_analyze():
    pts = e10_points()
    if len(pts) < 3:
        e10["result"] = "—"; root.after(0, lambda: e10_result.config(text="need ≥ 3 points")); return
    out = []
    fa = linfit_se([p[0] for p in pts], [p[1] for p in pts])
    fits = {}
    for dirn, name in ((+1, "forward"), (-1, "reverse")):
        g = [p for p in pts if p[2] == dirn]
        if len(g) >= 3:
            fits[dirn] = linfit_se([p[0] for p in g], [p[1] for p in g])
            a, b, r2, se_a, se_b, _rm, n = fits[dirn]
            out.append(f"{name:8s} N = {a:7.4f} ± {se_a:.4f}   c = {b:+8.2f}°   R² {r2:.6f}   n = {n}")
    if fa:
        a, b, r2, se_a, se_b, _rm, n = fa
        out.append(f"pooled   N = {a:7.4f} ± {se_a:.4f}   R² {r2:.6f}   n = {n}")
        out.append(f"η = 7.90 / N = {7.90 / a * 100:.1f} %   (E7 N·η = 7.90)")
        if +1 in fits and -1 in fits:
            dN = abs(fits[1][0] - fits[-1][0]) / a * 100
            off = (fits[-1][1] - fits[1][1]) / a
            out.append(f"N_fwd vs N_rev: {dN:.2f} %   {'ok' if dN <= 0.5 else 'CHECK (slip / lag?)'}")
            out.append(f"rev − fwd offset at same motor angle: {off:+.2f}° joint\n  (backlash + rope creep)")
        # per-cycle intercept drift
        cyc = sorted(set(x[1] for x in e10["dwells"]))
        cs = []
        for c in cyc:
            g = [(x[4], x[3]) for x in e10["dwells"] if x[1] == c]
            if len(g) >= 4:
                f = linfit_se([p[0] for p in g], [p[1] for p in g])
                if f: cs.append((c, f[0], f[1]))
        if len(cs) >= 2:
            drift = (max(x[2] for x in cs) - min(x[2] for x in cs)) / a
            out.append("cycles: " + "  ".join(f"c{c}: N {n_:.4f}" for c, n_, _b in cs) + f"\n  intercept drift {drift:.2f}° joint {'ok' if drift <= 0.2 else 'CHECK (rope slip?)'}")
    e10["result"] = "\n".join(out)
    root.after(0, lambda: e10_result.config(text=e10["result"]))


def e10_tick():
    r, p, y, _g = arm_latest(0.5)
    j = ar_angle((None, r, p, y)[ar_axis_idx()])
    m = math.degrees(T["ang"])
    e10_v_motor.config(text=f"{m:+.1f}"); e10_v_joint.config(text="—" if j is None else f"{j:+.1f}")
    with e10["lock"]:
        rows = list(e10["rows"])
    if rows and j is not None and abs(j - rows[0][6]) > 2.0:
        e10_v_ratio.config(text=f"{(m - rows[0][5]) / (j - rows[0][6]):.3f}")
    else:
        e10_v_ratio.config(text="—")
    if e10["run"]:
        e10_phase.config(text=f"● {e10['phase'].upper()}  cycle {e10['cycle']}  dir {e10['direction']:+d}  dwell points {e10['ndwell']}  state {STATE_NAMES.get(T['state'], '?')}", fg=GREEN)
    else:
        e10_phase.config(text=f"{len(rows)} samples · {len(e10['dwells'])} dwell points", fg=MUTED)
    pts = e10_points()
    fit = None
    if len(pts) >= 3:
        f = linfit([p[0] for p in pts], [p[1] for p in pts])
        if f: fit = f
    draw_scatter(e10_cv, [p[0] for p in pts], [p[1] for p in pts], fit=fit,
                 xlabel="joint angle (°)", ylabel="motor angle (°)", color=BLUE)


add_tab("11 · E10 · Ratio N", e10_fr)


# ═════════════ global live loops ═══════════════════════════════
def global_tick():
    m, spread = kv_steady_rpm()
    dut = (m or 0.0) * kv_ratio_v()
    kv_rpm.config(text=f"{dut:.0f}")
    if m is not None and kv["spinning"] and kv["steps"]:
        # 0.5% drift of the MEAN is a realistic bar once the measurement comes
        # from angle rather than from averaged velocity samples.
        tol = max(2.0, 0.005 * abs(kv["steps"][kv["idx"]]))
        dr = (spread or 99) * kv_ratio_v()
        ok = dr < tol
        kv_steady.config(
            text=(f"● STEADY — read the meter   (drift {dr:.1f} rpm)" if ok
                  else f"● settling…   drift {dr:.1f} rpm > {tol:.1f}"),
            fg=GREEN if ok else AMBER)
    else:
        kv_steady.config(text="")
    kvdata = [(t, r * kv_ratio_v()) for (t, r) in snap_hist(rpm_hist)]
    draw_ts(kv_chart, kvdata, BLUE, "DUT speed", unit1="rpm", label1="|speed|")
    m2, _ = iq_avg(1.0)
    w_iq.config(text=f"{abs(m2):.3f}" if m2 is not None else "—")
    w_state.config(text=STATE_NAMES.get(T["state"], "?"),
                   fg=GREEN if T["state"] == 2 else MUTED)
    w_ang.config(text=f"shaft {math.degrees(T['ang']):+.1f}°   cmd {T['iqc']:+.2f} A")


show_tab(0)
for _fn, _ms in ((foot_tick, 200), (dg_tick, 200), (dg_console_tick, 300),
                 (global_tick, 120), (h_tick, 200), (th_tick, 400),
                 (ar_tick, 250), (ac_tick, 400),
                 (e9_tick, 300), (e10_tick, 300)):   # [g4-A] [g4-B] [g5-A] [g6-A]
    safe_tick(_fn, _ms)()
threading.Thread(target=dev_reader, daemon=True).start()
threading.Thread(target=arm_reader, daemon=True).start()   # [g4-A]
threading.Thread(target=emu_loop, daemon=True).start()     # [g7-A] rehab current emulation


def on_close():
    global running
    running = False
    ht["run"] = False
    th["run"] = False          # [g3-A] a thermal run holds current — end it
    stop_all()
    e9["run"] = False          # [g5-A] an E9 run commands current — end it
    e9["pause"] = False
    e10["run"] = False         # [g6-A] an E10 sweep holds VEL — end it
    ar_log_stop()              # [g4-A] flush and close an open arm log
    dev_disconnect(); arm_disconnect()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()