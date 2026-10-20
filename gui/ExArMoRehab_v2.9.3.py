#!/usr/bin/env python3
"""
ExArMo Rehab v2.9 — patient / therapist front-end for ExArMo firmware v2.9
=========================================================================
Four therapy modes (Passive / Assistive / Resistive / Games), a live ExArMo
arm view, and CSV session logging. ALL motor control runs in the firmware on
the MKS XDrive Mini (FOC 20 kHz, estimators + admittance 1 kHz, homing,
virtual walls, thermal derate); this program is only a serial front-end, so
safety never depends on the PC.

    GUI action                → serial command to firmware
    ------------------------------------------------------------------
    Find limits               → H
    Assistive START           → E<n>, Pg/Pn/Px/Pt/Pw/Pi/Pz, A<gain>, O1
    Resistive START           → E<n>, Pg/Pw/Pi/Pz, A1, Pn=Px=level, O1
    Passive                   → Pv/Pa/Pc, G<angle> … (firmware ramps θ_ref)
    Games                     → O0 (motor free) or weightless: A1, light B_v, O1
    STOP (any mode)           → O0

Telemetry parsed (921600 baud):
    D,t_ms,state,theta,theta_ref,vel,iq_meas,tau_m,tau_s,tau_d,tau_dob,Bv,iq_cmd,T_est
    (firmware ≤ v2.8 sends 12 fields — T_est is then recorded as blank)
Firmware states: 0 IDLE 1 HOME↑ 2 HOME↓ 3 PARK 4 RUN 5 FAULT 6 PASSIVE 7 LAB

v2.9 changes (18 Oct 2026, from ExArMo_Final.py of 18 Aug):
  - CSV logs carry the 13th telemetry field T_est (estimated winding °C);
    the v2.8 header would have been one column short with firmware v2.9.
  - Recording at 500 Hz (D2) + a .meta.txt (firmware 'L' line, every setting).
  - Iq-limit sliders default 6 A (firmware v2.9 safety default; was 8 A and
    re-sent on every START). Thermal derate to 3 A is done by the firmware.
  - Gravity g, DOB ω_c, Iq limit and the new dead-zone are ONE shared value
    across Assistive / Resistive (sliders linked) and are saved to
    rehab_settings.json, so g tuned with the arm support survives a restart.
    Saved settings are pushed to the board on Connect.
  - Dead-zone slider (Pz, default 0.15 N·m from the E9 validation).
  - [18 Oct, firmware v2.9.2] reads the motor thermistor: the sidebar shows the
    MEASURED temperature (green < 40, amber < 55, red = stopped) when the board
    reports T_src=ntc, else the estimate as "~xx °C"; EV,TEMP / TEMPWARN /
    TEMPCLEAR / TEMPFAIL / TEMPOK shown in plain language.
  - Sidebar shows estimated winding temperature (green < 60, amber < 70,
    red = derating) and warns on EV,THERM / EV,VBUS.
  - Firmware check on Connect: warns if the board does not report fw=v2.9.
  - State 7 (LAB) named.

Run:  pip install pyserial   then   python ExArMoRehab_v2.9.py
      (tkinter ships with python.org Python on Windows and macOS)
"""

import tkinter as tk
import threading
import time
import math
import random
import csv
import os
import collections
import json
from datetime import datetime

import serial
import serial.tools.list_ports

BAUD = 921600
TELEM_MS_LIVE = 20      # 50 Hz for the GUI
TELEM_MS_REC  = 2       # 500 Hz while recording (fw v2.9; E08 logging)
APP_VERSION = "v2.9.3"
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rehab_settings.json")

# ════════════════ DESIGN SYSTEM ════════════════
BG       = "#0e1320"
SURFACE  = "#171f31"
SURFACE2 = "#212c45"
BORDER   = "#2a3651"
TEXT     = "#e9edf5"
MUTED    = "#a7b1c7"
FAINT    = "#75809a"

GREEN  = "#2dd4a7"
AMBER  = "#ffb454"
PURPLE = "#c084fc"
CYAN   = "#38c8f0"
RED    = "#f06a7e"
ORANGE = "#ff8a5c"
DIM_GREEN  = "#1b3a33"
DIM_AMBER  = "#3d3220"
DIM_PURPLE = "#342647"
DIM_CYAN   = "#1b3644"

F_TITLE = ("Segoe UI", 30, "bold")
F_H1    = ("Segoe UI", 20, "bold")
F_LABEL = ("Segoe UI", 15)
F_SMALL = ("Segoe UI", 13)
F_BTN   = ("Segoe UI", 17, "bold")
F_MONO  = ("Consolas", 18)

SCREEN_W, SCREEN_H = 1060, 520
PLAYER_SIZE = 56
OBSTACLE_W = 42
TELEM_MS_GAME = 10   # 100 Hz telemetry while gaming: smooth player input

STATE_NAMES = {0: "IDLE", 1: "HOMING \u2191", 2: "HOMING \u2193",
               3: "PARKING", 4: "RUN", 5: "FAULT", 6: "PASSIVE", 7: "LAB"}
STATE_COLORS = {0: MUTED, 1: CYAN, 2: CYAN, 3: CYAN, 4: GREEN, 5: RED, 6: GREEN, 7: CYAN}

# ════════════════ PERSISTENT SETTINGS ════════════════
# Saved next to this file. Holds every slider that maps to a firmware
# parameter, so the gravity coefficient tuned with the arm support (and the
# park angle, Iq limit, dead-zone …) survives a restart.
def _load_settings():
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            d = json.load(f)
            return d if isinstance(d, dict) else {}
    except Exception:
        return {}
SETTINGS = _load_settings()
_save_pending = [None]
def save_settings_later():
    """debounced write (1 s after the last change)"""
    def _w():
        _save_pending[0] = None
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump(SETTINGS, f, indent=1, sort_keys=True)
        except Exception:
            pass
    if _save_pending[0]:
        root.after_cancel(_save_pending[0])
    _save_pending[0] = root.after(1000, _w)

# keys shared between the Assistive and Resistive pages (one physical value)
SHARED_KEYS = ("Pg", "Pw", "Pi", "Pz")
# pushed to the board right after Connect
PUSH_ON_CONNECT = ("Pg", "Pw", "Pi", "Pz", "Pk", "Pa", "Pc", "Ph")   # Ph: homing current (fw v2.9.3 GUI slider)

# ════════════════ SHARED STATE ════════════════
running = True
mode = "menu"

# latest telemetry from the firmware
T = dict(t_ms=0, state=0, theta=0.0, theta_ref=0.0, vel=0.0, iq_meas=0.0,
         tau_m=0.0, tau_s=0.0, tau_d=0.0, tau_dob=0.0, Bv=0.0, iq_cmd=0.0,
         T_est=float("nan"))
fw_status = ""            # last 'L state=…' line from the firmware
t_src = "model"           # "ntc" when firmware v2.9.2+ reads the motor thermistor
fw_ok = None              # True = fw v2.9, False = older, None = unknown
rec_name = None
telem_fresh = False
last_line = ""

# joint working range, learned by watching homing (θ=0 at min stop + margin)
lim_min = 0.0
lim_max = 0.0
homed = False
est_sel = 2

# +1 normal, -1 inverted. Affects DISPLAY & GAMES only (up/down labels,
# gauge, game mapping). The real fix for a reversed rig is DIR_SIGN=-1 in
# the firmware — see firmware_direction_and_park_patch.md.
ui_sign = 1

# serial
ser = None
ser_lock = threading.Lock()

# recording
recording = False
csv_f = None
csv_w = None
n_rec = 0

# passive sequencer
passive_running = False
passive_min_pct = 5.0        # % of homed range
passive_max_pct = 95.0
passive_speed = 0.5          # [rad/s] joint, sent as P v
passive_acc = 1.2            # [rad/s²] profile acceleration, sent as Pa (restored from settings below)
passive_reps = 10
passive_rep_count = 0
passive_dwell = 0.6

# game state
player_y = SCREEN_H // 2
score = 0
game_over = False
game_kind = "flappy"
GAME_INSTR = {
    "flappy": "Steer through the gaps \u2014 don't hit the walls",
    "catch":  "Catch yellow stars, dodge red bombs",
    "follow": "Keep your dot on the moving target line",
}
obstacles = []
catch_items = []
catch_flash = 0
follow_t = 0.0
follow_target_y = SCREEN_H // 2
follow_on_target = 0
follow_total = 0
spawn_timer = 0.0
spawn_next = 1.5

# ════════════════ SERIAL LAYER ════════════════
def send(cmd, echo=True):
    """Send one newline-terminated command to the firmware."""
    global last_line
    with ser_lock:
        if ser is None or not ser.is_open:
            if echo:
                last_line = "not connected"
            return False
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            last_line = f"tx error: {e}"
            return False
    if echo:
        last_line = "> " + cmd
    return True

def reader_thread():
    """Parse telemetry and event lines from the firmware."""
    global telem_fresh, last_line, homed, lim_min, lim_max, n_rec, t_src
    prev_state = 0
    home_lo, home_hi = None, None
    while running:
        s = ser
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
        try:
            line = raw.decode(errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue
        if line.startswith("EV,"):
            p = line.split(",")
            ev = p[1] if len(p) > 1 else ""
            if ev == "HOMED" and len(p) >= 4:
                try:
                    lim_min = float(p[2]); lim_max = float(p[3])
                    homed = True
                    home_lo, home_hi = None, None
                    root.after(0, on_homed)
                except ValueError:
                    pass
            elif ev == "ZERO":             # [fw v2.9z] manual zero: EV,ZERO,<raw>,<still homed 0/1>
                still = len(p) >= 4 and p[3].strip() == "1"
                if still:                  # after homing: limits follow in EV,HOMED
                    root.after(0, lambda: set_status(home_status,
                               "Zero moved here \u2014 limits updated, still homed", GREEN))
                else:
                    homed = False
                    root.after(0, lambda: set_status(home_status,
                               "Zero set here \u2014 now press Find limits", AMBER))
                    root.after(0, lambda: set_all_status("Zero set \u2014 press Find limits", AMBER))
            elif ev == "SOFTSTOP" and len(p) >= 3 and p[2].strip() == "0":   # [fw t6] released early
                root.after(0, lambda: set_all_status("Stopped \u2014 motor released", RED))
            elif ev == "PARKED":
                root.after(0, lambda: set_all_status("Stopped at 0\u00b0 \u2014 motor released", GREEN))
                root.after(0, lambda: set_status(home_status,
                           "Parked at rest position \u2014 motor released", GREEN))
            elif ev == "HOMEFAIL":
                root.after(0, lambda: set_status(home_status,
                           "Homing failed \u2014 range too small (raise Ph?)", RED))
            elif ev == "HOMEBOOST":        # [fw v2.9.3] lift stalled short, retrying at 1.4x Ph
                root.after(0, lambda: set_status(home_status,
                           "Lift stalled short \u2014 retrying with more force\u2026", AMBER))
            elif ev == "HOMESHORT":        # [fw v2.9.3] homed, but range below PR (sent after EV,HOMED)
                try:
                    rdeg = math.degrees(float(p[2]))
                except (ValueError, IndexError):
                    rdeg = float("nan")
                root.after(50, lambda r=rdeg: set_status(home_status,
                           f"Homed, but range only {r:.0f}\u00b0 \u2014 raise homing current "
                           f"(Advanced) or check rope tension", AMBER))
            elif ev == "HOMETIMEOUT":
                root.after(0, lambda: set_status(home_status,
                           "Homing timeout \u2014 check mechanism / raise Ph", RED))
            elif ev == "RANGE_FAULT":
                root.after(0, lambda: set_all_status(
                           "FAULT \u2014 angle escaped range, send Home or Stop", RED))
            elif ev == "THERM":
                try:
                    tt, lim = float(p[2]), float(p[3])
                    msg = (f"Motor warm ({tt:.0f} \u00b0C) \u2014 force limited to {lim:.1f} A, let it rest"
                           if tt >= 70 else f"Motor cooled ({tt:.0f} \u00b0C) \u2014 full force available")
                    root.after(0, lambda m=msg, c=(AMBER if tt >= 70 else GREEN): set_all_status(m, c))
                except (ValueError, IndexError):
                    pass
            elif ev in ("TEMP", "TEMPWARN", "TEMPCLEAR", "TEMPFAIL", "TEMPOK"):   # [v2.9.2 firmware thermistor]
                try:
                    val = float(p[2]) if len(p) > 2 else float("nan")
                except ValueError:
                    val = float("nan")
                if ev == "TEMPOK":
                    t_src = "ntc"
                elif ev == "TEMPFAIL":
                    t_src = "model"
                msg, col = {
                    "TEMP":      (f"Motor too hot ({val:.0f} \u00b0C) \u2014 stopped, arm free. Rest until it cools", RED),
                    "TEMPWARN":  (f"Motor warm ({val:.0f} \u00b0C) \u2014 force is being reduced", AMBER),
                    "TEMPCLEAR": (f"Motor cooled ({val:.0f} \u00b0C) \u2014 ready again", GREEN),
                    "TEMPFAIL":  ("Temperature sensor not read \u2014 using the estimate (check the bead on GPIO3)", AMBER),
                    "TEMPOK":    (f"Temperature sensor OK ({val:.0f} \u00b0C)", GREEN),
                }[ev]
                root.after(0, lambda m=msg, c=col: set_all_status(m, c))
            elif ev == "VBUS":
                root.after(0, lambda: set_status(home_status,
                           "Supply voltage read failed \u2014 check power, firmware assumed 24 V", AMBER))
            last_line = line
            continue
        if line.startswith("D,"):
            p = line.split(",")
            if len(p) < 13:
                continue
            try:
                T["t_ms"]     = int(p[1]);   T["state"]   = int(p[2])
                T["theta"]    = float(p[3]); T["theta_ref"] = float(p[4])
                T["vel"]      = float(p[5]); T["iq_meas"] = float(p[6])
                T["tau_m"]    = float(p[7]); T["tau_s"]   = float(p[8])
                T["tau_d"]    = float(p[9]); T["tau_dob"] = float(p[10])
                T["Bv"]       = float(p[11]); T["iq_cmd"] = float(p[12])
                T["T_est"]    = float(p[13]) if len(p) > 13 else float("nan")
            except ValueError:
                continue
            T["_t"] = time.time()      # sample timestamp for player prediction
            telem_fresh = True
            st = T["state"]
            # fallback range learning (the EV,HOMED event is authoritative):
            # phase 1 (state 2) runs in the pre-zero frame, so only accumulate
            # after the zero is set — states 1 (lift) and 3 (park)
            if st == 2 and prev_state != 2:
                home_lo, home_hi = None, None
            if st in (1, 3):
                th = T["theta"]
                home_lo = th if home_lo is None else min(home_lo, th)
                home_hi = th if home_hi is None else max(home_hi, th)
            if prev_state in (1, 3) and st == 0 and home_hi is not None and not homed:
                lim_min = 0.0
                lim_max = max(0.05, home_hi)
                homed = True
                root.after(0, on_homed)
                home_lo, home_hi = None, None
            if st == 5 and prev_state != 5:
                root.after(0, lambda: set_all_status("FAULT \u2014 angle escaped range, send Home or Stop", RED))
            prev_state = st
            if recording and csv_w is not None:
                try:
                    row = p[1:14] if len(p) >= 14 else p[1:13] + [""]
                    csv_w.writerow([f"{time.time():.4f}"] + row)
                    n_rec += 1
                except Exception:
                    pass
        else:
            last_line = line     # firmware event / commander echo
            if line.startswith("L state="):
                global fw_status, fw_ok
                fw_status = line
                fw_ok = "fw=v2.9" in line
                t_src = "ntc" if "T_src=ntc" in line else "model"
                last_line = "board: fw v2.9 \u2713" if fw_ok else "board: OLD firmware \u2014 flash v2.9"
                root.after(0, show_fw_check)

def connect(port):
    global ser
    disconnect()
    try:
        s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.3)
        with ser_lock:
            ser = s
        send(f"D{TELEM_MS_LIVE}", echo=False)
        conn_status.config(text=f"\u25cf  {port}", fg=GREEN)
        root.after(400, push_saved_settings)       # board may have just booted
        root.after(700, lambda: send("L", echo=False))
        return True
    except Exception as e:
        conn_status.config(text=f"\u25cf  {e}", fg=RED)
        return False

def disconnect():
    global ser
    with ser_lock:
        if ser is not None:
            try:
                ser.write(b"O0\n")
                ser.close()
            except Exception:
                pass
            ser = None

def push_saved_settings():
    """send the saved firmware parameters (g tuned with the arm support,
    park angle, Iq limit, dead-zone …) so a fresh board matches the GUI"""
    for k in PUSH_ON_CONNECT:
        if k in SETTINGS:
            send(f"{k}{SETTINGS[k]:g}", echo=False)

def show_fw_check():
    if fw_ok is False:
        set_status(home_status, "Board firmware is older than v2.9 \u2014 flash v2.9 before therapy/recording", AMBER)
        conn_status.config(text=conn_status.cget("text").split("  fw")[0] + "  fw OLD", fg=AMBER)
    elif fw_ok:
        if "fw" not in conn_status.cget("text"):
            conn_status.config(text=conn_status.cget("text") + "  fw v2.9 \u2713")

# ════════════════ HELPERS ════════════════
def theta_norm():
    """joint angle mapped to -1..+1 across the homed range (for games)"""
    if not homed or lim_max - lim_min < 1e-3:
        return 0.0
    x = 2.0 * (T["theta"] - lim_min) / (lim_max - lim_min) - 1.0
    return ui_sign * max(-1.0, min(1.0, x))

def pct_to_theta(pct):
    return lim_min + pct / 100.0 * (lim_max - lim_min)

def stop_all_modes():
    global passive_running
    passive_running = False
    send("O0")

def soft_stop():
    """[fw t6] STOP buttons: lower the arm smoothly to 0° and then release ('S').
    Pressing STOP again while it is lowering releases it immediately."""
    global passive_running
    passive_running = False
    send("S")

def update_telem_rate():
    """100 Hz while gaming (smooth input), 50 Hz otherwise; recording (250 Hz)
    always wins."""
    if recording:
        return
    send(f"D{TELEM_MS_GAME if mode == 'game' else TELEM_MS_LIVE}", echo=False)

def set_all_status(text, color):
    for lbl in (passive_status, assist_status, resist_status):
        set_status(lbl, text, color)

def on_homed():
    rng = math.degrees(lim_max - lim_min)
    txt = f"Homed \u2014 range 0\u00b0 \u2026 {math.degrees(lim_max):.0f}\u00b0 ({rng:.0f}\u00b0)"
    set_status(home_status, txt, GREEN)
    set_all_status("Homed \u2014 ready", GREEN)

# ════════════════ RECORDING ════════════════
FIELDS = ["t_host", "t_ms", "state", "theta", "theta_ref", "vel", "iq_meas",
          "tau_m", "tau_s", "tau_d", "tau_dob", "Bv", "iq_cmd", "T_est"]

def write_meta(csv_name, n):
    try:
        with open(csv_name.replace(".csv", ".meta.txt"), "w", encoding="utf-8") as f:
            f.write(f"log: {csv_name}\nsaved: {datetime.now():%Y-%m-%d %H:%M:%S}\n")
            f.write(f"app: ExArMo Rehab {APP_VERSION}\nfirmware status: {fw_status}\n")
            f.write(f"mode: {mode}\nestimator: {['static','direct','DOB'][est_sel]}\n")
            f.write(f"assist gain A: {assist_gain:.2f}\nresistance level: {resist_level:.2f}\n")
            f.write(f"homed range [rad]: {lim_min:.4f} .. {lim_max:.4f}\n")
            f.write(f"telemetry: D{TELEM_MS_REC} ({1000 // TELEM_MS_REC} Hz)\nsamples: {n}\n")
            f.write("saved firmware settings:\n")
            for k in sorted(SETTINGS):
                f.write(f"  {k}{SETTINGS[k]:g}\n")
    except Exception:
        pass

def toggle_record():
    global recording, csv_f, csv_w, n_rec
    if not recording:
        os.makedirs("logs", exist_ok=True)
        name = datetime.now().strftime("logs/exo_%Y%m%d_%H%M%S.csv")
        csv_f = open(name, "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(FIELDS)
        n_rec = 0
        recording = True
        global rec_name
        rec_name = name
        send("L", echo=False)                      # fresh firmware status for the meta file
        send(f"D{TELEM_MS_REC}", echo=False)     # 500 Hz for the paper
        rec_btn.config(text="\u25a0 STOP", fg="#2b070d", bg=RED)
        rec_info.config(text=os.path.basename(name), fg=RED)
    else:
        recording = False
        send(f"D{TELEM_MS_GAME if mode == 'game' else TELEM_MS_LIVE}", echo=False)
        time.sleep(0.05)
        w, f = csv_w, csv_f
        csv_w = None
        csv_f = None
        try:
            f.close()
        except Exception:
            pass
        if rec_name:
            write_meta(rec_name, n_rec)
        rec_btn.config(text="\u25cf REC", fg=RED, bg=SURFACE)
        rec_info.config(text=f"{n_rec} samples \u2192 ./logs", fg=GREEN)

# ════════════════ PASSIVE SEQUENCER (host side) ════════════════
def passive_goto(theta_target, timeout):
    """Send G<angle> and wait for arrival (firmware ramps θ_ref at P v)."""
    send(f"G{theta_target:.4f}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        if not passive_running or mode != "passive":
            return False
        if T["state"] == 5:      # fault
            return False
        if abs(T["theta"] - theta_target) < 0.04 and abs(T["vel"]) < 0.08:
            return True
        time.sleep(0.03)
    return True   # timeout: continue anyway (wall / obstruction handled by firmware)

def passive_wait(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        if not passive_running or mode != "passive":
            return False
        time.sleep(0.02)
    return True

def passive_thread():
    global passive_rep_count, passive_running
    while running:
        if mode == "passive" and passive_running:
            lo = pct_to_theta(passive_min_pct)
            hi = pct_to_theta(passive_max_pct)
            span = max(0.05, hi - lo)
            tmo = (span / max(0.05, passive_speed)
                   + 2.0 * passive_speed / max(0.1, passive_acc) + 3.0)
            send(f"Pv{passive_speed:.3f}", echo=False)
            ok = passive_goto(lo, tmo)
            while ok and passive_running and mode == "passive" \
                    and passive_rep_count < passive_reps:
                ok = passive_goto(hi, tmo) and passive_wait(passive_dwell)
                if not ok:
                    break
                ok = passive_goto(lo, tmo) and passive_wait(passive_dwell)
                if not ok:
                    break
                passive_rep_count += 1
                root.after(0, update_passive_ui)
            if passive_running and passive_rep_count >= passive_reps:
                passive_running = False
                root.after(0, lambda: set_status(passive_status,
                           "Done \u2014 lowering to rest", GREEN))
                send("S")
        time.sleep(0.1)

def start_passive():
    global passive_running, passive_rep_count
    if not homed:
        set_status(passive_status, "Home first (sidebar \u2192 Find limits)", AMBER)
        return
    passive_rep_count = 0
    passive_running = True
    update_passive_ui()
    set_status(passive_status, "Running\u2026", CYAN)

def stop_passive():
    soft_stop()
    set_status(passive_status, "Lowering to 0\u00b0\u2026 (STOP again = release now)", AMBER)

def update_passive_ui():
    passive_rep_label.config(text=f"{passive_rep_count} / {passive_reps}")

# ════════════════ ASSISTIVE / RESISTIVE (firmware does the work) ════════
assist_gain = 1.0
resist_level = 0.8            # [N·m·s/rad] viscous resistance the user feels
assist_store = {}             # this page's firmware params, re-sent on START
resist_store = {}

def start_assist():
    if not homed:
        set_status(assist_status, "Home first (sidebar \u2192 Find limits)", AMBER)
        return
    send(f"E{est_sel}", echo=False)
    # re-send the whole assist parameter set (resistive mode overrides
    # Pn/Px on the firmware, so switching back must restore them)
    for k, v in assist_store.items():
        send(f"{k}{SETTINGS.get(k, v):g}", echo=False)
    send(f"A{assist_gain:.2f}", echo=False)
    send("O1")
    set_status(assist_status, "Active \u2014 supporting your arm", GREEN)

def stop_assist():
    soft_stop()
    set_status(assist_status, "Lowering to 0\u00b0\u2026 (STOP again = release now)", AMBER)

def start_resist():
    if not homed:
        set_status(resist_status, "Home first (sidebar \u2192 Find limits)", AMBER)
        return
    send(f"E{est_sel}", echo=False)
    for k, v in resist_store.items():
        send(f"{k}{SETTINGS.get(k, v):g}", echo=False)
    # Resistance = VISCOUS damping, not negative assist. With A=1 and
    # B_min = B_max = R, the admittance yields θ̇_ref = τ̂/R: the opposing
    # torque you feel is R × your speed, in BOTH directions, and it never
    # escalates while you hold still. (Negative A made θ_ref run AWAY from
    # the arm, so the PD force grew without bound — the "spring that kicks".)
    send("A1", echo=False)
    send(f"Pn{resist_level:.2f}", echo=False)
    send(f"Px{resist_level:.2f}", echo=False)
    send("O1")
    set_status(resist_status, "Active \u2014 move against the drag", PURPLE)

def stop_resist():
    soft_stop()
    set_status(resist_status, "Lowering to 0\u00b0\u2026 (STOP again = release now)", AMBER)

# ════════════════ MINI-GAMES — ultra-smooth engine ══════════════════════
# 60 FPS on the Tk main thread (no cross-thread jitter), all motion scaled
# by real frame time, and the player dot is velocity-predicted between
# 100 Hz telemetry samples then low-pass filtered — it glides, not steps.
def reset_game():
    global player_y, score, game_over
    global obstacles, catch_items, catch_flash, spawn_timer, spawn_next
    global follow_t, follow_target_y, follow_on_target, follow_total
    player_y = SCREEN_H // 2
    score = 0.0
    game_over = False
    obstacles.clear()
    catch_items.clear()
    catch_flash = 0.0
    spawn_timer = 0.0
    spawn_next = 1.5           # [s] first spawn
    follow_t = 0.0
    follow_target_y = SCREEN_H // 2
    follow_on_target = 0.0
    follow_total = 0.0

def set_game(kind):
    global game_kind
    game_kind = kind
    reset_game()
    game_instr.config(text=GAME_INSTR.get(kind, ""))
    for k, btn in game_buttons.items():
        if k == kind:
            btn.config(bg=CYAN, fg="#062430")
        else:
            btn.config(bg=SURFACE2, fg=MUTED)

PLAYER_TAU = 0.06   # [s] low-pass time constant of the dot (smooth but responsive)

def theta_norm_pred():
    """joint angle → -1..+1, dead-reckoned forward with the measured velocity.
    Prediction horizon = sample age + the smoothing filter's own lag, so the
    dot is smooth AND has near-zero latency."""
    if not homed or lim_max - lim_min < 1e-3:
        return 0.0
    age = min(0.08, max(0.0, time.time() - T.get("_t", 0.0)))
    th = T["theta"] + T["vel"] * (age + 0.8 * PLAYER_TAU)
    x = 2.0 * (th - lim_min) / (lim_max - lim_min) - 1.0
    return ui_sign * max(-1.0, min(1.0, x))


def update_player(dt):
    global player_y
    target = SCREEN_H / 2 - theta_norm_pred() * (SCREEN_H / 2 - 40)
    alpha = 1.0 - math.exp(-dt / PLAYER_TAU)
    player_y += alpha * (target - player_y)

# world speeds in px/s (identical feel to the original 30 fps values)
FLAPPY_SPEED = 82.0   # px/s, same crossing time on the wider arena
CATCH_SPEED  = 122.0

def update_flappy(dt):
    global score, game_over, spawn_timer, spawn_next
    spawn_timer += dt
    if spawn_timer >= spawn_next:
        gap_y = random.randint(110, SCREEN_H - 110)
        obstacles.append([SCREEN_W, gap_y])
        spawn_next = random.uniform(3.0, 5.0)
        spawn_timer = 0.0
    for obs in obstacles:
        obs[0] -= FLAPPY_SPEED * dt
    obstacles[:] = [o for o in obstacles if o[0] > -OBSTACLE_W]
    px = 80
    gap_size = 145
    for obs in obstacles:
        if px + PLAYER_SIZE - 5 > obs[0] and px + 5 < obs[0] + OBSTACLE_W:
            if player_y - PLAYER_SIZE // 2 < obs[1] - gap_size // 2 or \
               player_y + PLAYER_SIZE // 2 > obs[1] + gap_size // 2:
                game_over = True
    if player_y < 20 or player_y > SCREEN_H - 20:
        game_over = True
    score += 30.0 * dt          # same rate as 1/frame at the old 30 fps

def update_catch(dt):
    global score, spawn_timer, spawn_next, catch_flash
    spawn_timer += dt
    if spawn_timer >= spawn_next:
        kind = "bomb" if random.random() < 0.25 else "star"
        y = random.randint(40, SCREEN_H - 40)
        catch_items.append([SCREEN_W, y, kind])
        spawn_next = random.uniform(0.8, 1.7)
        spawn_timer = 0.0
    px = 80
    remaining = []
    for it in catch_items:
        it[0] -= CATCH_SPEED * dt
        x, y, kind = it
        caught = (px + PLAYER_SIZE > x and px < x + 34 and
                  abs(player_y - y) < PLAYER_SIZE // 2 + 17)
        if caught:
            if kind == "star":
                score += 50
            else:
                score = max(0, score - 30)
                catch_flash = 0.2          # [s]
        elif x > -30:
            remaining.append(it)
    catch_items[:] = remaining
    if catch_flash > 0:
        catch_flash = max(0.0, catch_flash - dt)

def update_follow(dt):
    global score, follow_t, follow_target_y, follow_on_target, follow_total
    follow_t += dt
    amp = SCREEN_H // 2 - 60
    follow_target_y = SCREEN_H / 2 - amp * math.sin(follow_t * 0.9)   # same 0.9 rad/s
    follow_total += dt
    if abs(player_y - follow_target_y) < 48:
        follow_on_target += dt
        score += 30.0 * dt

_last_frame = [time.perf_counter()]

def game_tick():
    """60 FPS game engine on the Tk main thread."""
    if not running:
        return
    now = time.perf_counter()
    dt = min(0.05, now - _last_frame[0])   # clamp: a UI hiccup never teleports the world
    _last_frame[0] = now
    if mode == "game":
        update_player(dt)
        if not game_over:
            if game_kind == "flappy":
                update_flappy(dt)
            elif game_kind == "catch":
                update_catch(dt)
            elif game_kind == "follow":
                update_follow(dt)
        draw_game()
    root.after(15, game_tick)              # ~60 fps

def draw_player(on_target=False):
    px = 80
    fill = GREEN if on_target else CYAN
    canvas.create_oval(px, player_y - PLAYER_SIZE // 2,
                       px + PLAYER_SIZE, player_y + PLAYER_SIZE // 2,
                       fill=fill, outline="#ffffff", width=2)
    canvas.create_oval(px + 12, player_y - 11, px + 23, player_y, fill="white", outline="")
    canvas.create_oval(px + 32, player_y - 11, px + 43, player_y, fill="white", outline="")

def draw_game():
    canvas.delete("all")
    canvas.create_rectangle(0, 0, SCREEN_W, SCREEN_H, fill="#0a0f1a", outline="")
    canvas.create_line(0, SCREEN_H // 2, SCREEN_W, SCREEN_H // 2,
                       fill="#26304a", width=1, dash=(4, 8))
    top_limit = 40
    bot_limit = SCREEN_H - 40
    canvas.create_line(0, top_limit, SCREEN_W, top_limit, fill="#25403a", width=1, dash=(4, 8))
    canvas.create_line(0, bot_limit, SCREEN_W, bot_limit, fill="#25403a", width=1, dash=(4, 8))
    canvas.create_text(16, top_limit + 16, text="upper stop", fill="#3f6a5e", font=F_SMALL, anchor="w")
    canvas.create_text(16, bot_limit - 16, text="lower stop", fill="#3f6a5e", font=F_SMALL, anchor="w")

    on_target = False
    if game_kind == "flappy":
        gap_size = 145
        for obs in obstacles:
            gap_y = obs[1]
            canvas.create_rectangle(obs[0], 0, obs[0] + OBSTACLE_W, gap_y - gap_size // 2,
                                    fill="#e5566d", outline="#ff7d90", width=1)
            canvas.create_rectangle(obs[0], gap_y + gap_size // 2, obs[0] + OBSTACLE_W, SCREEN_H,
                                    fill="#e5566d", outline="#ff7d90", width=1)
    elif game_kind == "catch":
        if catch_flash > 0:
            canvas.create_rectangle(0, 0, SCREEN_W, SCREEN_H, fill="#4a1228", outline="")
        for x, y, kind in catch_items:
            if kind == "star":
                canvas.create_oval(x, y - 17, x + 34, y + 17, fill="#ffd75e", outline="#fff0b0", width=3)
            else:
                canvas.create_oval(x, y - 17, x + 34, y + 17, fill="#e5566d", outline="#ff7d90", width=3)
                canvas.create_line(x + 17, y - 17, x + 25, y - 28, fill=AMBER, width=3)
    elif game_kind == "follow":
        ty = follow_target_y
        canvas.create_rectangle(0, ty - 48, SCREEN_W, ty + 48, fill="#14322a", outline="")
        canvas.create_line(0, ty, SCREEN_W, ty, fill=GREEN, width=2)
        on_target = abs(player_y - ty) < 48

    draw_player(on_target=on_target)

    if game_kind == "flappy":
        canvas.create_text(SCREEN_W - 20, 22, text=f"SCORE  {int(score) // 10}",
                           fill=TEXT, font=("Segoe UI", 20, "bold"), anchor="ne")
    elif game_kind == "catch":
        canvas.create_text(SCREEN_W - 20, 22, text=f"SCORE  {int(score)}",
                           fill=TEXT, font=("Segoe UI", 20, "bold"), anchor="ne")
    elif game_kind == "follow":
        pct = 100 * follow_on_target / max(1, follow_total)
        canvas.create_text(SCREEN_W - 20, 22, text=f"ON TARGET  {pct:.0f}%",
                           fill=TEXT, font=("Segoe UI", 20, "bold"), anchor="ne")

    canvas.create_text(20, 20, text=f"ARM  {math.degrees(T['theta']):+.1f}\u00b0",
                       fill=CYAN, font=F_MONO, anchor="nw")

    if game_over:
        canvas.create_rectangle(SCREEN_W//2 - 250, SCREEN_H//2 - 100,
                                SCREEN_W//2 + 250, SCREEN_H//2 + 120,
                                fill=SURFACE, outline=RED, width=2)
        canvas.create_text(SCREEN_W // 2, SCREEN_H // 2 - 52,
                           text="GAME OVER", fill=RED, font=("Segoe UI", 48, "bold"))
        shown = int(score) // 10 if game_kind == "flappy" else int(score)
        canvas.create_text(SCREEN_W // 2, SCREEN_H // 2 + 14,
                           text=f"Score  {shown}", fill=TEXT, font=("Segoe UI", 26))
        canvas.create_text(SCREEN_W // 2, SCREEN_H // 2 + 66,
                           text="Press R to restart", fill=MUTED, font=F_LABEL)

# ════════════════ UI HELPERS ════════════════
root = tk.Tk()
root.title(f"ExArMo Rehab {APP_VERSION} \u2014 Elbow Rehabilitation System")
root.geometry("1800x960")
root.minsize(1500, 820)
try:
    root.state("zoomed")          # start maximized on Windows
except tk.TclError:
    pass
root.configure(bg=BG)

def _lighten(hexcol, amt=20):
    hexcol = hexcol.lstrip("#")
    r, g, b = (int(hexcol[i:i+2], 16) for i in (0, 2, 4))
    return "#%02x%02x%02x" % (min(255, r+amt), min(255, g+amt), min(255, b+amt))

def flat_button(parent, text, bg, fg, command, width=None):
    b = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                  activebackground=_lighten(bg), activeforeground=fg,
                  relief="flat", bd=0, font=F_BTN, padx=28, pady=14,
                  cursor="hand2", highlightthickness=0)
    if width:
        b.config(width=width)
    b.bind("<Enter>", lambda e: b.config(bg=_lighten(b.cget("bg"))))
    b.bind("<Leave>", lambda e: b.config(bg=bg))
    return b

def card(parent):
    outer = tk.Frame(parent, bg=BORDER, padx=1, pady=1)
    inner = tk.Frame(outer, bg=SURFACE, padx=26, pady=22)
    inner.pack(fill="both", expand=True)
    return outer, inner

def section_label(parent, text):
    tk.Label(parent, text=text.upper(), bg=SURFACE, fg=FAINT,
             font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(0, 6))

def set_status(label, text, color):
    label.config(text="\u25cf  " + text, fg=color)

_linked = collections.defaultdict(list)   # fw_cmd -> [(var, value_label, unit)]
_linking = [False]

def slider_row(parent, label, unit, from_, to, res, default, callback,
               accent, fw_cmd=None, store=None, persist=True):
    """A slider. If fw_cmd is set (e.g. 'Pn'), the value is ALSO sent to the
    firmware (debounced 300 ms) as fw_cmd + value — firmware is the brain.
    If store is given, the page remembers {fw_cmd: value} so a mode's START
    button can re-send its whole parameter set (modes share firmware keys).
    [v2.9] Values with a fw_cmd are saved to rehab_settings.json and restored
    at start-up; sliders for the SHARED_KEYS move together on every page."""
    save_key = fw_cmd if (fw_cmd and persist) else None
    if save_key and save_key in SETTINGS:
        default = max(from_, min(to, float(SETTINGS[save_key])))
    if fw_cmd and store is not None:
        store[fw_cmd] = default
    holder = tk.Frame(parent, bg=SURFACE)
    holder.pack(fill="x", pady=(4, 3))
    top = tk.Frame(holder, bg=SURFACE)
    top.pack(fill="x")
    tk.Label(top, text=label, bg=SURFACE, fg=TEXT, font=F_LABEL).pack(side="left")
    val = tk.Label(top, text=f"{default:g} {unit}", bg=SURFACE, fg=accent,
                   font=("Consolas", 15, "bold"))
    val.pack(side="right")
    var = tk.DoubleVar(value=default)
    deb = [None]
    def _fire():
        deb[0] = None
        if fw_cmd:
            send(f"{fw_cmd}{var.get():g}", echo=False)
    def _cb(v):
        val.config(text=f"{float(v):g} {unit}")
        if _linking[0]:
            return
        if callback:
            callback(v)
        if save_key:
            SETTINGS[save_key] = float(v)
            save_settings_later()
        if fw_cmd in SHARED_KEYS:
            _linking[0] = True
            try:
                for ovar, olab, ounit in _linked[fw_cmd]:
                    if ovar is not var:
                        ovar.set(float(v)); olab.config(text=f"{float(v):g} {ounit}")
            finally:
                _linking[0] = False
        if fw_cmd:
            if store is not None:
                store[fw_cmd] = float(v)
            if deb[0]:
                root.after_cancel(deb[0])
            deb[0] = root.after(300, _fire)
    s = tk.Scale(holder, from_=from_, to=to, resolution=res, orient="horizontal",
                 variable=var, command=_cb, showvalue=0,
                 bg=SURFACE, fg=TEXT, troughcolor=SURFACE2,
                 highlightthickness=0, bd=0, sliderrelief="flat",
                 activebackground=accent, sliderlength=30, width=18)
    s.pack(fill="x")
    if fw_cmd:
        _linked[fw_cmd].append((var, val, unit))
    return var

def readout(parent, caption):
    v = tk.Label(parent, text="\u2014", bg=SURFACE, fg=CYAN, font=F_MONO)
    v.pack()
    tk.Label(parent, text=caption, bg=SURFACE, fg=FAINT, font=F_SMALL).pack(pady=(0, 6))
    return v

# ---------- layout: sidebar + content ----------
sidebar = tk.Frame(root, bg=SURFACE, width=300)
sidebar.pack(side="left", fill="y")
sidebar.pack_propagate(False)

content = tk.Frame(root, bg=BG)
content.pack(side="left", fill="both", expand=True)

_brand = tk.Frame(sidebar, bg=SURFACE)
_brand.pack(pady=(24, 0))
tk.Label(_brand, text="Ex", bg=SURFACE, fg=AMBER,
         font=("Segoe UI", 28, "bold")).pack(side="left")
tk.Label(_brand, text="ArMo", bg=SURFACE, fg=TEXT,
         font=("Segoe UI", 28, "bold")).pack(side="left")
tk.Label(sidebar, text=f"ELBOW REHABILITATION \u00b7 {APP_VERSION}", bg=SURFACE, fg=FAINT,
         font=("Segoe UI", 10, "bold")).pack(pady=(0, 12))

# --- connection ---
cframe = tk.Frame(sidebar, bg=SURFACE)
cframe.pack(fill="x", padx=14, pady=(0, 6))
port_var = tk.StringVar()
ports = [p.device for p in serial.tools.list_ports.comports()] or ["\u2014"]
port_var.set(ports[0])
port_menu = tk.OptionMenu(cframe, port_var, *ports)
port_menu.config(bg=SURFACE2, fg=TEXT, relief="flat", bd=0, highlightthickness=0,
                 font=F_SMALL, activebackground=BORDER, activeforeground=TEXT)
port_menu["menu"].config(bg=SURFACE2, fg=TEXT, bd=0)
port_menu.pack(fill="x")
def refresh_ports():
    m = port_menu["menu"]
    m.delete(0, "end")
    for p in [q.device for q in serial.tools.list_ports.comports()] or ["\u2014"]:
        m.add_command(label=p, command=lambda v=p: port_var.set(v))
crow = tk.Frame(cframe, bg=SURFACE)
crow.pack(fill="x", pady=(4, 0))
tk.Button(crow, text="\u21bb", command=refresh_ports, bg=SURFACE2, fg=MUTED,
          relief="flat", bd=0, font=F_SMALL, cursor="hand2",
          highlightthickness=0, padx=6).pack(side="left")
tk.Button(crow, text="Connect", command=lambda: connect(port_var.get()),
          bg=SURFACE2, fg=CYAN, relief="flat", bd=0, font=("Segoe UI", 13, "bold"),
          cursor="hand2", highlightthickness=0, padx=10).pack(side="left", padx=4)
conn_status = tk.Label(cframe, text="\u25cf  disconnected", bg=SURFACE, fg=MUTED,
                       font=F_SMALL, anchor="w")
conn_status.pack(fill="x", pady=(3, 0))

# --- homing ---
# [fw v2.9z] manual zero: arm on the bottom stop by hand → "0 here" → Find limits lifts only
home_row = tk.Frame(sidebar, bg=SURFACE)
home_row.pack(fill="x", padx=14, pady=(4, 2))
zero_btn = tk.Button(home_row, text="0 here",
                     command=lambda: (stop_all_modes(), send("Z")),
                     bg=SURFACE2, fg=AMBER, relief="flat", bd=0,
                     font=("Segoe UI", 16, "bold"), pady=12, padx=10,
                     cursor="hand2", highlightthickness=0)
zero_btn.pack(side="left", padx=(0, 6))
home_btn = tk.Button(home_row, text="\u2302 Find limits",
                     command=lambda: (stop_all_modes(), send("H")),
                     bg=SURFACE2, fg=CYAN, relief="flat", bd=0,
                     font=("Segoe UI", 16, "bold"), pady=12,
                     cursor="hand2", highlightthickness=0)
home_btn.pack(side="left", fill="x", expand=True)
home_status = tk.Label(sidebar, text="\u25cf  Not homed", bg=SURFACE, fg=MUTED,
                       font=F_SMALL, anchor="w", wraplength=245, justify="left")
home_status.pack(fill="x", padx=16, pady=(0, 4))

# --- Advanced fold: engineering controls, hidden for everyday use ---
adv_visible = [False]
adv_toggle = tk.Label(sidebar, text="\u2699  Advanced  \u25be", bg=SURFACE,
                      fg=FAINT, font=F_SMALL, cursor="hand2", anchor="w", padx=16)
adv_toggle.pack(fill="x", pady=(2, 0))
adv_frame = tk.Frame(sidebar, bg=SURFACE)
def toggle_adv(e=None):
    adv_visible[0] = not adv_visible[0]
    if adv_visible[0]:
        adv_frame.pack(fill="x", after=adv_toggle)
        adv_toggle.config(text="\u2699  Advanced  \u25b4", fg=MUTED)
    else:
        adv_frame.pack_forget()
        adv_toggle.config(text="\u2699  Advanced  \u25be", fg=FAINT)
adv_toggle.bind("<Button-1>", toggle_adv)

# --- direction invert (display & games; real fix = DIR_SIGN in firmware) ---
invert_var = tk.BooleanVar(value=False)
def apply_invert():
    global ui_sign
    ui_sign = -1 if invert_var.get() else 1
tk.Checkbutton(adv_frame, text="Invert up/down",
               variable=invert_var, command=apply_invert,
               bg=SURFACE, fg=MUTED, font=F_SMALL, selectcolor=SURFACE2,
               activebackground=SURFACE, activeforeground=TEXT,
               highlightthickness=0, anchor="w").pack(fill="x", padx=12)

# --- [fw v2.9z] full automatic homing (push down to the bottom stop, then up) ---
tk.Button(adv_frame, text="Full auto homing (H1)",
          command=lambda: (stop_all_modes(), send("H1")),
          bg=SURFACE2, fg=MUTED, relief="flat", bd=0, font=F_SMALL,
          cursor="hand2", highlightthickness=0, pady=4).pack(fill="x", padx=14, pady=(4, 0))

# --- park angle after homing (needs 'Pk' firmware patch) ---
park_frame = tk.Frame(adv_frame, bg=SURFACE)
park_frame.pack(fill="x", padx=14, pady=(4, 0))
park_top = tk.Frame(park_frame, bg=SURFACE)
park_top.pack(fill="x")
tk.Label(park_top, text="Park after homing", bg=SURFACE, fg=MUTED,
         font=F_SMALL).pack(side="left")
park_val = tk.Label(park_top, text="10\u00b0", bg=SURFACE, fg=CYAN,
                    font=("Consolas", 14, "bold"))
park_val.pack(side="right")
_park_deb = [None]
def _park_send(deg):
    _park_deb[0] = None
    SETTINGS["Pk"] = round(math.radians(deg), 4); save_settings_later()
    send(f"Pk{math.radians(deg):.4f}", echo=False)
def _park_cb(v):
    park_val.config(text=f"{float(v):.0f}\u00b0")
    if _park_deb[0]:
        root.after_cancel(_park_deb[0])
    _park_deb[0] = root.after(300, lambda: _park_send(float(v)))
park_var = tk.DoubleVar(value=round(math.degrees(SETTINGS.get("Pk", math.radians(10)))))
park_val.config(text=f"{park_var.get():.0f}\u00b0")
tk.Scale(park_frame, from_=0, to=90, resolution=1, orient="horizontal",
         variable=park_var, command=_park_cb, showvalue=0,
         bg=SURFACE, fg=TEXT, troughcolor=SURFACE2, highlightthickness=0,
         bd=0, sliderrelief="flat", activebackground=CYAN,
         sliderlength=26, width=14).pack(fill="x")

# --- homing current Ph (fw v2.9.3): the firmware's own messages say "raise Ph"
#     when the arm stalls before a stop; this is the only place a therapist can.
#     1.2 A was enough for the old rope; the stiffer rope may need 1.5–2 A.
#     Keep below the 3 A continuous rating — homing takes ~10 s at full push.
home_i_frame = tk.Frame(adv_frame, bg=SURFACE)
home_i_frame.pack(fill="x", padx=14, pady=(6, 0))
home_i_top = tk.Frame(home_i_frame, bg=SURFACE)
home_i_top.pack(fill="x")
tk.Label(home_i_top, text="Homing force (Ph)", bg=SURFACE, fg=MUTED,
         font=F_SMALL).pack(side="left")
home_i_val = tk.Label(home_i_top, text="1.2 A", bg=SURFACE, fg=CYAN,
                      font=("Consolas", 14, "bold"))
home_i_val.pack(side="right")
_home_i_deb = [None]
def _home_i_send(a):
    _home_i_deb[0] = None
    SETTINGS["Ph"] = round(a, 2); save_settings_later()
    send(f"Ph{a:.2f}", echo=False)
def _home_i_cb(v):
    home_i_val.config(text=f"{float(v):.1f} A")
    if _home_i_deb[0]:
        root.after_cancel(_home_i_deb[0])
    _home_i_deb[0] = root.after(300, lambda: _home_i_send(float(v)))
home_i_var = tk.DoubleVar(value=float(SETTINGS.get("Ph", 1.2)))
home_i_val.config(text=f"{home_i_var.get():.1f} A")
tk.Scale(home_i_frame, from_=0.5, to=2.5, resolution=0.1, orient="horizontal",
         variable=home_i_var, command=_home_i_cb, showvalue=0,
         bg=SURFACE, fg=TEXT, troughcolor=SURFACE2, highlightthickness=0,
         bd=0, sliderrelief="flat", activebackground=CYAN,
         sliderlength=26, width=14).pack(fill="x")
tk.Frame(sidebar, bg=BORDER, height=1).pack(fill="x", padx=16, pady=(6, 8))

nav_buttons = {}
nav_active = ["menu"]

def make_nav(key, text, color, command):
    b = tk.Label(sidebar, text=text, bg=SURFACE, fg=MUTED, font=("Segoe UI", 17),
                 anchor="w", padx=28, pady=14, cursor="hand2")
    b.pack(fill="x")
    b.bind("<Button-1>", lambda e: command())
    b.bind("<Enter>", lambda e: b.config(bg=SURFACE2) if nav_active[0] != key else None)
    b.bind("<Leave>", lambda e: b.config(bg=SURFACE) if nav_active[0] != key else None)
    nav_buttons[key] = (b, color)

def set_nav(key):
    nav_active[0] = key
    for k, (b, color) in nav_buttons.items():
        b.config(bg=SURFACE2 if k == key else SURFACE,
                 fg=color if k == key else MUTED)

# sidebar footer: state + angle + record
footer = tk.Frame(sidebar, bg=SURFACE)
footer.pack(side="bottom", fill="x")
tk.Frame(footer, bg=BORDER, height=1).pack(fill="x", padx=16)
fw_state = tk.Label(footer, text="\u2014", bg=SURFACE, fg=MUTED,
                    font=("Segoe UI", 17, "bold"))
fw_state.pack(pady=(10, 0))
_encrow = tk.Frame(footer, bg=SURFACE)
_encrow.pack(pady=(2, 4))
side_enc = tk.Label(_encrow, text="+0.0\u00b0", bg=SURFACE, fg=CYAN, font=F_MONO)
side_enc.pack(side="left")
side_temp = tk.Label(_encrow, text="", bg=SURFACE, fg=FAINT, font=F_SMALL)
side_temp.pack(side="left", padx=(10, 0))
rrow2 = tk.Frame(footer, bg=SURFACE)
rrow2.pack(pady=(0, 4))
rec_btn = tk.Button(rrow2, text="\u25cf REC", command=toggle_record,
                    bg=SURFACE, fg=RED, relief="flat", bd=0,
                    font=("Segoe UI", 15, "bold"), padx=12, pady=4,
                    cursor="hand2", highlightthickness=0)
rec_btn.pack()
rec_info = tk.Label(footer, text="telemetry \u2192 CSV", bg=SURFACE, fg=FAINT, font=F_SMALL)
rec_info.pack(pady=(0, 4))
console = tk.Label(footer, text="", bg=SURFACE, fg=FAINT, font=("Consolas", 11),
                   wraplength=255, justify="left", anchor="w")
console.pack(fill="x", padx=10, pady=(0, 2))
cmd_entry = tk.Entry(adv_frame, bg=SURFACE2, fg=TEXT, insertbackground=TEXT,
                     relief="flat", font=("Consolas", 14), highlightthickness=0)
cmd_entry.pack(fill="x", padx=14, pady=(4, 8))
cmd_entry.insert(0, "command\u2026")
def _cmd_focus(e):
    if cmd_entry.get() == "command\u2026":
        cmd_entry.delete(0, "end")
def _cmd_send(e):
    c = cmd_entry.get().strip()
    if c:
        send(c)
        cmd_entry.delete(0, "end")
cmd_entry.bind("<FocusIn>", _cmd_focus)
cmd_entry.bind("<Return>", _cmd_send)

def emergency_stop():
    # [fw t6] soft stop: lower to 0° then release; press again = release now
    soft_stop()
    try:
        weightless.set(False)
        motor_badge.config(text="MOTOR: FREE", fg=GREEN)
    except Exception:
        pass
    try:
        set_all_status("Lowering to 0\u00b0\u2026 (STOP again = release now)", AMBER)
    except Exception:
        pass
estop_btn = tk.Button(sidebar, text="\u25a0  STOP", command=emergency_stop,
                      bg=RED, fg="#2b070d", relief="flat", bd=0,
                      font=("Segoe UI", 20, "bold"), pady=14,
                      activebackground="#ff8a99", activeforeground="#2b070d",
                      cursor="hand2", highlightthickness=0)
estop_btn.pack(side="bottom", fill="x", padx=14, pady=(6, 10))

def update_sidebar():
    st = T["state"]
    fw_state.config(text=STATE_NAMES.get(st, "?"), fg=STATE_COLORS.get(st, MUTED))
    side_enc.config(text=f"{math.degrees(T['theta']):+.1f}\u00b0")
    te = T.get("T_est", float("nan"))
    if te == te:     # not NaN
        if t_src == "ntc":   # measured: rehab limits 40 warn / 55 stop (firmware v2.9.2)
            col = GREEN if te < 40 else AMBER if te < 55 else RED
            side_temp.config(text=f"{te:.0f} \u00b0C", fg=col)
        else:                # software estimate: derate band 70-80
            col = GREEN if te < 60 else AMBER if te < 70 else RED
            side_temp.config(text=f"~{te:.0f} \u00b0C", fg=col)
    else:
        side_temp.config(text="", fg=FAINT)
    if recording:
        rec_info.config(text=f"\u25cf {n_rec} samples", fg=RED)
    console.config(text=last_line[-70:])
    root.after(150, update_sidebar)

# ---------- frames ----------
menu_frame    = tk.Frame(content, bg=BG)
passive_frame = tk.Frame(content, bg=BG)
assist_frame  = tk.Frame(content, bg=BG)
resist_frame  = tk.Frame(content, bg=BG)
game_frame    = tk.Frame(content, bg=BG)

def hide_all():
    for f in (menu_frame, passive_frame, assist_frame, resist_frame, game_frame):
        f.pack_forget()

def show_menu():
    global mode
    mode = "menu"
    stop_all_modes()
    update_telem_rate()
    hide_all(); set_nav("menu")
    menu_frame.pack(fill="both", expand=True, padx=36, pady=28)
    _reset_menu_cards()

def show_passive():
    global mode
    mode = "passive"
    stop_all_modes()
    update_telem_rate()
    hide_all(); set_nav("passive")
    passive_frame.pack(fill="both", expand=True, padx=36, pady=28)

def show_assistive():
    global mode
    mode = "assistive"
    stop_all_modes()
    update_telem_rate()
    hide_all(); set_nav("assistive")
    assist_frame.pack(fill="both", expand=True, padx=36, pady=28)
    set_status(assist_status, "Ready \u2014 press START, then move your arm", MUTED)

def show_resistive():
    global mode
    mode = "resistive"
    stop_all_modes()
    update_telem_rate()
    hide_all(); set_nav("resistive")
    resist_frame.pack(fill="both", expand=True, padx=36, pady=28)
    set_status(resist_status, "Ready \u2014 press START, then push against it", MUTED)

def show_game():
    global mode
    mode = "game"
    stop_all_modes()
    weightless.set(False)      # games ALWAYS start with the motor free
    apply_game_motor()
    update_telem_rate()
    reset_game()
    hide_all(); set_nav("game")
    game_frame.pack(fill="both", expand=True, padx=36, pady=28)

make_nav("menu",      "\u2302   Home",       MUTED,  show_menu)
make_nav("passive",   "\u27f3   Passive",    GREEN,  show_passive)
make_nav("assistive", "\u2708   Assistive",  AMBER,  show_assistive)
make_nav("resistive", "\u2b71   Resistive",  PURPLE, show_resistive)
make_nav("game",      "\u25b6   Games",      CYAN,   show_game)

# ---------- MENU ----------
_mtitle = tk.Frame(menu_frame, bg=BG)
_mtitle.pack(anchor="w")
tk.Label(_mtitle, text="Ex", bg=BG, fg=AMBER, font=F_TITLE).pack(side="left")
tk.Label(_mtitle, text="ArMo Rehab", bg=BG, fg=TEXT, font=F_TITLE).pack(side="left")
tk.Label(menu_frame, text="Elbow rehabilitation \u2014 follow the steps, then choose a therapy mode",
         bg=BG, fg=MUTED, font=F_LABEL).pack(anchor="w", pady=(0, 12))

_chk = tk.Frame(menu_frame, bg=BG)
_chk.pack(anchor="w", pady=(0, 18))
def _pill(text):
    l = tk.Label(_chk, text=text, bg=SURFACE, fg=MUTED,
                 font=("Segoe UI", 14, "bold"), padx=18, pady=9)
    l.pack(side="left", padx=(0, 10))
    return l
chk_conn  = _pill("\u25cb  1 \u00b7 Connect device")
chk_home  = _pill("\u25cb  2 \u00b7 Find movement range")
chk_ready = _pill("\u25cb  3 \u00b7 Choose a therapy mode")
def update_checklist():
    ok_conn = ser is not None and getattr(ser, "is_open", False)
    chk_conn.config(text=("\u2713" if ok_conn else "\u25cb") + "  1 \u00b7 Connect device",
                    fg=GREEN if ok_conn else MUTED)
    chk_home.config(text=("\u2713" if homed else "\u25cb") + "  2 \u00b7 Find movement range",
                    fg=GREEN if homed else MUTED)
    ready = ok_conn and homed
    chk_ready.config(text=("\u2192  3 \u00b7 Choose a therapy mode" if ready
                           else "\u25cb  3 \u00b7 Choose a therapy mode"),
                     fg=CYAN if ready else MUTED)
    root.after(400, update_checklist)

_menu_cards = []   # (widgets, dim) for hover-reset — fixes cards stuck highlighted

def _reset_menu_cards():
    for widgets, _dim in _menu_cards:
        for w in widgets:
            w.config(bg=SURFACE)

def menu_card(parent, title, desc, accent, dim, command):
    outer = tk.Frame(parent, bg=BORDER, padx=1, pady=1)
    inner = tk.Frame(outer, bg=SURFACE, padx=30, pady=26, cursor="hand2")
    inner.pack(fill="both", expand=True)
    bar = tk.Frame(inner, bg=accent, height=7, width=76)
    bar.pack(anchor="w", pady=(0, 16))
    t = tk.Label(inner, text=title, bg=SURFACE, fg=TEXT,
                 font=("Segoe UI", 24, "bold"), anchor="w", cursor="hand2")
    t.pack(anchor="w")
    d = tk.Label(inner, text=desc, bg=SURFACE, fg=MUTED, font=("Segoe UI", 15),
                 anchor="w", justify="left", wraplength=430, cursor="hand2")
    d.pack(anchor="w", pady=(8, 0))
    go = tk.Label(inner, text="Tap to start  \u203a", bg=SURFACE, fg=accent,
                  font=("Segoe UI", 15, "bold"), anchor="w", cursor="hand2")
    go.pack(anchor="w", side="bottom", pady=(14, 0))
    widgets = (inner, t, d, go)
    _menu_cards.append((widgets, dim))
    def on_enter(e):
        for w in widgets:
            w.config(bg=dim)
    def on_leave(e):
        for w in widgets:
            w.config(bg=SURFACE)
    for w in widgets:
        w.bind("<Button-1>", lambda e: command())
        w.bind("<Enter>", on_enter)
        w.bind("<Leave>", on_leave)
    return outer

mgrid = tk.Frame(menu_frame, bg=BG)
mgrid.pack(fill="both", expand=True)
mgrid.columnconfigure((0, 1), weight=1, uniform="col")
mgrid.rowconfigure((0, 1), weight=1, uniform="row")

menu_card(mgrid, "Passive Rehab",
          "The machine gently moves your arm for you, back and forth between two angles. No effort needed \u2014 best for early recovery.",
          GREEN, DIM_GREEN, show_passive).grid(row=0, column=0, sticky="nsew", padx=(0, 9), pady=(0, 9))
menu_card(mgrid, "Assistive Support",
          "The machine carries your arm's weight and helps every movement you start \u2014 light while you move, steady when you rest.",
          AMBER, DIM_AMBER, show_assistive).grid(row=0, column=1, sticky="nsew", padx=(9, 0), pady=(0, 9))
menu_card(mgrid, "Resistive Training",
          "Move against a smooth, adjustable drag \u2014 like exercising underwater. The faster you move, the harder it works you.",
          PURPLE, DIM_PURPLE, show_resistive).grid(row=1, column=0, sticky="nsew", padx=(0, 9), pady=(9, 0))
menu_card(mgrid, "Game Rehab",
          "Three mini-games you play by raising and lowering your arm. Motor stays off \u2014 optional weightless help if lifting is hard.",
          CYAN, DIM_CYAN, show_game).grid(row=1, column=1, sticky="nsew", padx=(9, 0), pady=(9, 0))

# ════════════════ ExArMo — live arm view ════════════════
# A live animated side-view of the exoskeleton arm. Forearm follows the
# smoothed/predicted joint angle (same pipeline as the game dot), a dashed
# ghost shows the firmware's theta_ref target, the arc shows the homed range,
# and the elbow glows with the estimated human effort. Display only — the
# real dynamics run in the firmware.
ARM_VIEW_ZERO = 0.0       # θ = 0 → forearm HORIZONTAL (resting on the table);
                          # flexion sweeps upward from the table surface

_arm_sims = []            # (canvas, state_dict, visible_modes)

def make_arm_sim(parent, w=430, h=470, modes=()):
    o, inner = card(parent)
    section_label(inner, "ExArMo \u2014 live arm view")
    c = tk.Canvas(inner, width=w, height=h, bg="#0a0f1a", highlightthickness=0)
    c.pack()
    cap = tk.Label(inner, text="\u25cf actual arm    \u25cb target", bg=SURFACE,
                   fg=FAINT, font=F_SMALL)
    cap.pack(pady=(8, 0))
    st = {"w": w, "h": h, "af": None}
    _arm_sims.append((c, st, set(modes)))
    return o

def _arm_angle_pred():
    """joint angle [rad], velocity-predicted + used with per-canvas smoothing"""
    age = min(0.08, max(0.0, time.time() - T.get("_t", 0.0)))
    return T["theta"] + T["vel"] * (age + 0.05)

def _draw_arm(c, st, dt):
    w, h = st["w"], st["h"]
    # geometry: seated at a table — elbow at table height, flexion sweeps upward
    ex, ey = int(w * 0.42), int(h * 0.64)          # elbow, at the table edge
    U  = int(h * 0.26)                             # shoulder height above table
    Lf = int(min(w, h) * 0.40)                     # forearm length
    sy = ey - U
    rng = (lim_max - lim_min) if homed else math.radians(120)

    # smooth the measured angle (same feel as the game dot)
    a_target = _arm_angle_pred() if homed else 0.35 + 0.0*dt
    if st["af"] is None:
        st["af"] = a_target
    st["af"] += (1.0 - math.exp(-dt / 0.06)) * (a_target - st["af"])
    a_scr  = math.radians(ARM_VIEW_ZERO) + st["af"]           # forearm screen angle
    a_ref  = math.radians(ARM_VIEW_ZERO) + T["theta_ref"]

    c.delete("all")
    # the table the arm support rests on at θ = 0
    ty = ey + 14
    c.create_rectangle(ex - 24, ty, w - 8, ty + 10, fill="#1c2740", outline="")
    c.create_line(ex - 24, ty, w - 8, ty, fill="#3b4a6d", width=3)
    c.create_line(ex + 30, ty + 10, ex + 30, h - 10, fill="#26304a", width=4)
    c.create_line(w - 40, ty + 10, w - 40, h - 10, fill="#26304a", width=4)
    c.create_text(w - 14, ty + 26, text="table", fill=FAINT, font=F_SMALL, anchor="ne")
    # range arc around the elbow
    r = Lf + 14
    c.create_arc(ex - r, ey - r, ex + r, ey + r, style="arc",
                 start=ARM_VIEW_ZERO, extent=math.degrees(rng),
                 outline="#2a3651", width=3)
    for lim_a, txt in ((math.radians(ARM_VIEW_ZERO), "lower stop"),
                       (math.radians(ARM_VIEW_ZERO) + rng, "upper stop")):
        lx, ly = ex + r * math.cos(lim_a), ey - r * math.sin(lim_a)
        c.create_line(ex + (r - 12) * math.cos(lim_a), ey - (r - 12) * math.sin(lim_a),
                      lx, ly, fill="#3f6a5e", width=3)
    if not homed:
        c.create_text(w // 2, 24, text="not homed \u2014 demo range",
                      fill=AMBER, font=F_SMALL)

    # seated figure to the LEFT of the mechanism (clear of the flexion arc)
    bx = ex - int(w * 0.34)
    c.create_oval(bx - 20, sy - 66, bx + 20, sy - 26, outline=MUTED, width=3)   # head
    c.create_line(bx, sy - 26, bx, ty, fill=MUTED, width=6, capstyle="round")   # torso
    # upper arm: shoulder on the torso, diagonally down to the elbow
    c.create_line(bx, sy - 14, ex, ey, fill="#5a6a8f", width=12, capstyle="round")
    c.create_oval(bx - 9, sy - 23, bx + 9, sy - 5, fill="#5a6a8f", outline="")

    # effort glow at the elbow: green = flexion effort, orange = extension
    tau = active_tau_h() * ui_sign
    if T["state"] == 4 and abs(tau) > 0.04:
        col = GREEN if tau > 0 else ORANGE
        gr = 16 + min(22, abs(tau) * 40)
        c.create_oval(ex - gr, ey - gr, ex + gr, ey + gr, outline=col, width=3)

    # ghost forearm at theta_ref (only while the firmware is driving/tracking)
    if homed and T["state"] in (4, 6):
        gx, gy = ex + Lf * math.cos(a_ref), ey - Lf * math.sin(a_ref)
        c.create_line(ex, ey, gx, gy, fill="#4a5878", width=5, dash=(7, 7),
                      capstyle="round")
        c.create_oval(gx - 10, gy - 10, gx + 10, gy + 10, outline="#4a5878", width=3)

    # forearm + hand at the measured angle
    hx, hy = ex + Lf * math.cos(a_scr), ey - Lf * math.sin(a_scr)
    c.create_line(ex, ey, hx, hy, fill=CYAN, width=13, capstyle="round")
    c.create_oval(ex - 11, ey - 11, ex + 11, ey + 11, fill="#38c8f0", outline="#a5e6f7", width=2)
    c.create_oval(hx - 13, hy - 13, hx + 13, hy + 13, fill="#a5e6f7", outline="#ffffff", width=2)

    # big live angle readout
    c.create_text(w - 16, h - 18, text=f"{math.degrees(st['af']):+.0f}\u00b0",
                  fill=CYAN, font=("Consolas", 26, "bold"), anchor="se")

_arm_last = [time.perf_counter()]

def arm_tick():
    if not running:
        return
    now = time.perf_counter()
    dt = min(0.05, now - _arm_last[0])
    _arm_last[0] = now
    for c, st, modes in _arm_sims:
        if mode in modes:
            _draw_arm(c, st, dt)
    root.after(33, arm_tick)              # ~30 fps is plenty for the avatar

# ---------- PASSIVE ----------
tk.Label(passive_frame, text="Passive Rehab", bg=BG, fg=GREEN, font=F_TITLE).pack(anchor="w")
tk.Label(passive_frame,
         text="The machine gently moves your arm between the two angles set below \u2014 relax and let it work",
         bg=BG, fg=MUTED, font=F_LABEL).pack(anchor="w", pady=(0, 14))

prow = tk.Frame(passive_frame, bg=BG)
prow.pack(fill="both", expand=True)

pcard_o, pcard = card(prow)
pcard_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
section_label(pcard, "Motion settings  (% of homed range)")
def upd_pmin(v):
    global passive_min_pct; passive_min_pct = float(v)
def upd_pmax(v):
    global passive_max_pct; passive_max_pct = float(v)
def upd_pspeed(v):
    global passive_speed; passive_speed = float(v)
def upd_pacc(v):
    global passive_acc; passive_acc = float(v)
def upd_preps(v):
    global passive_reps; passive_reps = int(float(v)); update_passive_ui()
def upd_pdwell(v):
    global passive_dwell; passive_dwell = float(v)
slider_row(pcard, "Lower angle", "%", 0, 50, 1, 5, upd_pmin, GREEN)
slider_row(pcard, "Upper angle", "%", 50, 100, 1, 95, upd_pmax, GREEN)
slider_row(pcard, "Speed", "rad/s", 0.1, 3.0, 0.05, 0.5, upd_pspeed, GREEN)
slider_row(pcard, "Smoothness \u2014 acceleration (lower = softer)", "rad/s\u00b2",
           0.2, 8.0, 0.1, SETTINGS.get("Pa", 1.2), upd_pacc, GREEN, fw_cmd="Pa")
slider_row(pcard, "Friction helper \u2014 raise until motion stops skipping", "N\u00b7m",
           0.0, 0.4, 0.01, 0.12, None, GREEN, fw_cmd="Pc")
slider_row(pcard, "Rounds", "", 1, 50, 1, 10, upd_preps, GREEN)
slider_row(pcard, "Dwell at ends", "s", 0.0, 3.0, 0.1, 0.6, upd_pdwell, GREEN)

pbtns = tk.Frame(pcard, bg=SURFACE)
pbtns.pack(pady=(14, 8))
flat_button(pbtns, "\u25b6  START", GREEN, "#06251d", start_passive, width=11).pack(side="left", padx=(0, 8))
flat_button(pbtns, "\u25a0  STOP", RED, "#2b070d", stop_passive, width=11).pack(side="left")
passive_status = tk.Label(pcard, text="\u25cf  Ready", bg=SURFACE, fg=MUTED, font=F_LABEL)
passive_status.pack(anchor="w")

make_arm_sim(prow, w=400, h=460, modes=("passive",)).pack(side="left", fill="y", padx=(0, 14))

pside_o, pside = card(prow)
pside_o.pack(side="left", fill="y")
section_label(pside, "Progress")
passive_rep_label = tk.Label(pside, text="0 / 10", bg=SURFACE, fg=TEXT,
                             font=("Segoe UI", 34, "bold"))
passive_rep_label.pack(pady=(2, 0))
tk.Label(pside, text="rounds complete", bg=SURFACE, fg=FAINT, font=F_SMALL).pack(pady=(0, 10))
p_theta = readout(pside, "joint angle")
p_ref   = readout(pside, "\u03b8_ref (firmware)")


# ---------- shared gauge / torque panel ----------
def draw_gauge(c, torque, maxt, up_col, down_col):
    c.delete("all")
    W, H = 170, 260
    cx, cy = W // 2, H // 2
    c.create_rectangle(1, 1, W - 1, H - 1, outline=BORDER)
    for frac_line in (-0.5, 0.5):
        y = cy - frac_line * (cy - 16)
        c.create_line(14, y, W - 14, y, fill="#2a3651", dash=(2, 4))
    c.create_line(10, cy, W - 10, cy, fill="#3b4a6d", dash=(3, 3))
    c.create_text(W - 8, 12, text="UP", fill=up_col, anchor="e", font=("Segoe UI", 12, "bold"))
    c.create_text(W - 8, H - 12, text="DOWN", fill=down_col, anchor="e", font=("Segoe UI", 12, "bold"))
    m = max(0.01, maxt)
    frac = max(-1.0, min(1.0, torque / m))
    bar = frac * (cy - 16)
    if torque >= 0:
        c.create_rectangle(cx - 26, cy - bar, cx + 26, cy, fill=up_col, outline="")
    else:
        c.create_rectangle(cx - 26, cy, cx + 26, cy - bar, fill=down_col, outline="")

def torque_panel(parent, accent):
    o, inner = card(parent)
    section_label(inner, "Your effort \u03c4\u0302_h \u2014 live")
    gauge = tk.Canvas(inner, width=170, height=260, bg="#0a0f1a", highlightthickness=0)
    gauge.pack(pady=(0, 8))
    state = tk.Label(inner, text="Idle", bg=SURFACE, fg=accent, font=("Segoe UI", 17, "bold"))
    state.pack(pady=(0, 8))
    pos = readout(inner, "\u03b8 (rad)")
    vel = readout(inner, "\u03b8\u0307 (rad/s)")
    trq = readout(inner, "motor \u03c4_m (N\u00b7m)")
    bv  = readout(inner, "B_v damping")
    return o, gauge, state, pos, vel, trq, bv

def active_tau_h():
    return (T["tau_s"], T["tau_d"], T["tau_dob"])[est_sel]

def estimator_bar(parent, accent):
    section_label(parent, "Active estimator (feeds the admittance)")
    bar = tk.Frame(parent, bg=SURFACE)
    bar.pack(anchor="w", pady=(0, 6))
    btns = {}
    def sel(i):
        global est_sel
        est_sel = i
        send(f"E{i}", echo=False)
        for k, b in btns.items():
            if k == i:
                b.config(bg=accent, fg="#111")
            else:
                b.config(bg=SURFACE2, fg=MUTED)
    for i, name in enumerate(("Static", "Direct", "DOB")):
        b = tk.Button(bar, text=name, command=lambda k=i: sel(k),
                      bg=SURFACE2, fg=MUTED, activebackground=accent,
                      activeforeground="#111", relief="flat", bd=0,
                      font=("Segoe UI", 15, "bold"), padx=14, pady=6,
                      cursor="hand2", highlightthickness=0)
        b.pack(side="left", padx=3)
        btns[i] = b
    btns[est_sel].config(bg=accent, fg="#111")
    return btns

# ---------- ASSISTIVE ----------
tk.Label(assist_frame, text="Assistive Support", bg=BG, fg=AMBER, font=F_TITLE).pack(anchor="w")
tk.Label(assist_frame, text="The machine carries your arm's weight and amplifies every movement you start",
         bg=BG, fg=MUTED, font=F_LABEL).pack(anchor="w", pady=(0, 14))

arow = tk.Frame(assist_frame, bg=BG)
arow.pack(fill="both", expand=True)

acard_o, acard = card(arow)
acard_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
assist_est_btns = estimator_bar(acard, AMBER)
section_label(acard, "Control settings  (sent to firmware)")
def upd_again(v):
    global assist_gain
    assist_gain = float(v)
    send(f"A{assist_gain:.2f}", echo=False)
slider_row(acard, "Assist gain A", "", 0.0, 2.0, 0.05, 1.0, upd_again, AMBER)
slider_row(acard, "Gravity support g \u2014 raise until the arm floats", "N\u00b7m",
           0.0, 2.0, 0.01, 0.6, None, AMBER, fw_cmd="Pg", store=assist_store)
slider_row(acard, "B_min \u2014 damping when you move (light)", "",
           0.01, 0.5, 0.01, 0.05, None, AMBER, fw_cmd="Pn", store=assist_store)
slider_row(acard, "B_max \u2014 damping when passive (firm)", "",
           0.1, 2.0, 0.05, 0.8, None, AMBER, fw_cmd="Px", store=assist_store)
slider_row(acard, "\u03c4\u2080 \u2014 effort scale of the transition", "N\u00b7m",
           0.05, 1.0, 0.01, 0.4, None, AMBER, fw_cmd="Pt", store=assist_store)
slider_row(acard, "DOB bandwidth \u03c9_c", "rad/s",
           5, 100, 1, 30, None, AMBER, fw_cmd="Pw", store=assist_store)
slider_row(acard, "Dead-zone \u2014 ignore effort below (0 while tuning g)", "N\u00b7m",
           0.0, 0.5, 0.01, 0.15, None, AMBER, fw_cmd="Pz", store=assist_store)
slider_row(acard, "Iq limit (safety)", "A",
           1.0, 8.0, 0.5, 6.0, None, AMBER, fw_cmd="Pi", store=assist_store)

abtns = tk.Frame(acard, bg=SURFACE)
abtns.pack(pady=(12, 8))
flat_button(abtns, "\u25b6  START", AMBER, "#2e2005", start_assist, width=11).pack(side="left", padx=(0, 8))
flat_button(abtns, "\u25a0  STOP", RED, "#2b070d", stop_assist, width=11).pack(side="left")
assist_status = tk.Label(acard, text="\u25cf  Ready", bg=SURFACE, fg=MUTED, font=F_LABEL)
assist_status.pack(anchor="w")

make_arm_sim(arow, w=380, h=460, modes=("assistive",)).pack(side="left", fill="y", padx=(0, 14))

(apanel_o, assist_gauge, assist_state_label, assist_pos_label,
 assist_vel_label, assist_tau_label, assist_bv_label) = torque_panel(arow, AMBER)
apanel_o.pack(side="left", fill="y")

def update_assist_display():
    if mode == "assistive":
        vel = T["vel"] * ui_sign          # display sense (UP = positive)
        assist_pos_label.config(text=f"{T['theta']:+.3f}")
        assist_vel_label.config(text=f"{vel:+.3f}")
        assist_tau_label.config(text=f"{T['tau_m'] * ui_sign:+.3f}")
        assist_bv_label.config(text=f"{T['Bv']:.2f}")
        if T["state"] != 4:
            assist_state_label.config(text="Idle", fg=MUTED)
        elif abs(vel) <= 0.05:
            assist_state_label.config(text="Supporting", fg=AMBER)
        elif vel > 0:
            assist_state_label.config(text="Assisting UP", fg=GREEN)
        else:
            assist_state_label.config(text="Assisting DOWN", fg=ORANGE)
        # gauge shows YOUR estimated effort — pull up, bar goes up.
        # (raw motor torque is dominated by gravity holding + PD braking,
        # which looks backwards even though it is physically correct)
        draw_gauge(assist_gauge, active_tau_h() * ui_sign, 1.0, GREEN, ORANGE)
    root.after(60, update_assist_display)

# ---------- RESISTIVE ----------
tk.Label(resist_frame, text="Resistive Training", bg=BG, fg=PURPLE, font=F_TITLE).pack(anchor="w")
tk.Label(resist_frame, text="Viscous drag \u2014 the faster you move, the harder it resists, both directions, no kick",
         bg=BG, fg=MUTED, font=F_LABEL).pack(anchor="w", pady=(0, 14))

rrow = tk.Frame(resist_frame, bg=BG)
rrow.pack(fill="both", expand=True)

rcard_o, rcard = card(rrow)
rcard_o.pack(side="left", fill="both", expand=True, padx=(0, 14))
resist_est_btns = estimator_bar(rcard, PURPLE)
section_label(rcard, "Control settings  (sent to firmware)")
def upd_rlevel(v):
    global resist_level
    resist_level = float(v)
    # live update while running: resistance = constant virtual damping
    send(f"Pn{resist_level:.2f}", echo=False)
    send(f"Px{resist_level:.2f}", echo=False)
slider_row(rcard, "Resistance level \u2014 torque per speed", "N\u00b7m\u00b7s/rad",
           0.1, 3.0, 0.05, 0.8, upd_rlevel, PURPLE)
slider_row(rcard, "Gravity support g \u2014 raise until the arm floats", "N\u00b7m",
           0.0, 2.0, 0.01, 0.6, None, PURPLE, fw_cmd="Pg", store=resist_store)
slider_row(rcard, "DOB bandwidth \u03c9_c", "rad/s",
           5, 100, 1, 30, None, PURPLE, fw_cmd="Pw", store=resist_store)
slider_row(rcard, "Dead-zone \u2014 ignore effort below", "N\u00b7m",
           0.0, 0.5, 0.01, 0.15, None, PURPLE, fw_cmd="Pz", store=resist_store)
slider_row(rcard, "Iq limit (safety)", "A",
           1.0, 8.0, 0.5, 6.0, None, PURPLE, fw_cmd="Pi", store=resist_store)
tk.Label(rcard, text="How it works: A = 1 with constant virtual damping B = level.\n"
                     "Moving the arm needs \u03c4 \u2248 level \u00d7 speed; holding still needs nothing.",
         bg=SURFACE, fg=FAINT, font=F_SMALL, justify="left").pack(anchor="w", pady=(6, 0))

rbtns2 = tk.Frame(rcard, bg=SURFACE)
rbtns2.pack(pady=(12, 8))
flat_button(rbtns2, "\u25b6  START", PURPLE, "#241238", start_resist, width=11).pack(side="left", padx=(0, 8))
flat_button(rbtns2, "\u25a0  STOP", RED, "#2b070d", stop_resist, width=11).pack(side="left")
resist_status = tk.Label(rcard, text="\u25cf  Ready", bg=SURFACE, fg=MUTED, font=F_LABEL)
resist_status.pack(anchor="w")

make_arm_sim(rrow, w=380, h=460, modes=("resistive",)).pack(side="left", fill="y", padx=(0, 14))

(rpanel_o, resist_gauge, resist_state_label, resist_pos_label,
 resist_vel_label, resist_tau_label, resist_bv_label) = torque_panel(rrow, PURPLE)
rpanel_o.pack(side="left", fill="y")

def update_resist_display():
    if mode == "resistive":
        vel = T["vel"] * ui_sign          # display sense (UP = positive)
        resist_pos_label.config(text=f"{T['theta']:+.3f}")
        resist_vel_label.config(text=f"{vel:+.3f}")
        resist_tau_label.config(text=f"{T['tau_m'] * ui_sign:+.3f}")
        resist_bv_label.config(text=f"{T['Bv']:.2f}")
        if T["state"] != 4:
            resist_state_label.config(text="Idle", fg=MUTED)
        elif abs(vel) <= 0.05:
            resist_state_label.config(text="Holding", fg=PURPLE)
        elif vel > 0:
            resist_state_label.config(text="Resisting UP", fg=ORANGE)
        else:
            resist_state_label.config(text="Resisting DOWN", fg=GREEN)
        draw_gauge(resist_gauge, active_tau_h() * ui_sign, 1.0, GREEN, ORANGE)
    root.after(60, update_resist_display)

def update_passive_display():
    if mode == "passive":
        p_theta.config(text=f"{math.degrees(T['theta']):+.1f}\u00b0")
        p_ref.config(text=f"{math.degrees(T['theta_ref']):+.1f}\u00b0")
    root.after(100, update_passive_display)

# ---------- GAME ----------
tk.Label(game_frame, text="Game Rehab", bg=BG, fg=CYAN, font=F_TITLE).pack(anchor="w")
ghead = tk.Frame(game_frame, bg=BG)
ghead.pack(fill="x", pady=(10, 0))

weightless = tk.BooleanVar(value=False)
def apply_game_motor():
    if mode != "game":
        return
    if weightless.get() and homed:
        # transparency: gravity comp + follow the user with light damping.
        # (A=0 would freeze θ_ref and the PD would HOLD the arm like a
        # spring; A=1 with low B_v lets the arm float and move freely.)
        send("Pn0.05", echo=False)
        send("Px0.15", echo=False)
        send("Pt0.4", echo=False)
        send("A1", echo=False)
        send("O1")           # weightless arm: easier gameplay for weak users
        motor_badge.config(text="MOTOR: WEIGHTLESS", fg=AMBER)
    else:
        send("O0")           # limp: raw arm, zero motor force
        motor_badge.config(text="MOTOR: FREE", fg=GREEN)
motor_badge = tk.Label(ghead, text="MOTOR: FREE", bg=BG, fg=GREEN,
                       font=("Segoe UI", 15, "bold"))
motor_badge.pack(side="right", padx=(0, 14))
wl = tk.Checkbutton(ghead, text="Weightless help (optional)", variable=weightless,
                    command=apply_game_motor, bg=BG, fg=MUTED, font=F_LABEL,
                    selectcolor=SURFACE2, activebackground=BG, activeforeground=TEXT,
                    highlightthickness=0)
wl.pack(side="right", padx=(0, 10))

game_buttons = {}
gbar = tk.Frame(ghead, bg=BG)
gbar.pack(side="left")
for key, label in (("flappy", "Flappy"), ("catch", "Catch"), ("follow", "Follow")):
    b = tk.Button(gbar, text=label, command=lambda k=key: set_game(k),
                  bg=SURFACE2, fg=MUTED, activebackground=CYAN, activeforeground="#062430",
                  relief="flat", bd=0, font=("Segoe UI", 15, "bold"),
                  padx=20, pady=10, cursor="hand2", highlightthickness=0)
    b.pack(side="left", padx=3)
    game_buttons[key] = b
rb = tk.Button(gbar, text="\u21bb Restart", command=reset_game,
               bg=SURFACE, fg=MUTED, activebackground=SURFACE2, activeforeground=TEXT,
               relief="flat", bd=0, font=("Segoe UI", 15),
               padx=18, pady=10, cursor="hand2", highlightthickness=0)
rb.pack(side="left", padx=(10, 0))

game_instr = tk.Label(game_frame, text=GAME_INSTR["flappy"], bg=BG, fg=MUTED, font=F_LABEL)
game_instr.pack(anchor="w", pady=(2, 10))

gcanvas_o = tk.Frame(game_frame, bg=BORDER, padx=1, pady=1)
gcanvas_o.pack(pady=(6, 0))
canvas = tk.Canvas(gcanvas_o, width=SCREEN_W, height=SCREEN_H,
                   bg="#0a0f1a", highlightthickness=0)
canvas.pack()

root.bind("<r>", lambda e: reset_game() if mode == "game" else None)
root.bind("<R>", lambda e: reset_game() if mode == "game" else None)

# ---------- start everything ----------
passive_acc = float(SETTINGS.get("Pa", passive_acc))
set_game("flappy")
set_nav("menu")
menu_frame.pack(fill="both", expand=True, padx=36, pady=28)

update_sidebar()
update_assist_display()
update_resist_display()
update_passive_display()
game_tick()
arm_tick()
update_checklist()

threading.Thread(target=reader_thread, daemon=True).start()
threading.Thread(target=passive_thread, daemon=True).start()

def on_close():
    global running, passive_running
    running = False
    passive_running = False
    if recording:
        toggle_record()
    send("O0", echo=False)
    disconnect()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()