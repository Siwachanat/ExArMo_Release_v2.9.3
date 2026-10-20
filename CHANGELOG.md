# ExArMo Release — CHANGELOG

## ★ CURRENT: Firmware v2.9z — 20 Oct 2026 (on the v2.9 base) · ExArMo Rehab / GUI / Lab with "0 here"

**This folder's `firmware/…/src/main.cpp` is v2.9z, built on the original v2.9 — the version that runs best on
the new (larger) capstan rope.** The v2.9.3 homing changes described further down (separate homing gravity,
escalating push, position-ramp lift, friction compensation) are **NOT** in this firmware; v2.9 homing runs as-is.
Everything below was added to v2.9, nothing else changed:

* **Manual zero `Z`** ("0 here"): in IDLE, the arm's current position becomes 0°.
  Before homing → `H` skips the push-down and only lifts to find the top. After homing → the limits shift with
  the new zero and the device stays homed (rope creep), unless the range would drop below 17°.
  `H1` = full automatic homing (down + up), replaces the manual zero. `L` shows `zero=manual|auto|none`.
* **Thermistor** (v2.9.2 code): 100k NTC on PA2, `T_est` = measured temperature while valid, warn + current derate
  from 40 °C (`PW`), coast-stop at 55 °C (`PS`, `PS0` = off), `T` dump, `X<°C>` beta calibration,
  `PN/PO/PB/PT` bead parameters, `EV,TEMPOK/TEMPFAIL/TEMPWARN/TEMP/TEMPCLEAR`.
  * Read on **ADC3** by direct registers, not `_readRegularADCVoltage()` (which used ADC1 = the motor current
    sense and caused a 5 Hz torque kick = shaking).
  * By default read **only while the motor is off** (`PA1`); while running the thermal model continues from the
    last measured value with the same 40/55 °C limits. `PA2` = always read, `PA0` = never (model only).
  * Software model: integrates 0 A while the driver is off (no more runaway to 26 000 °C in IDLE).
* **Driver wake-up**: for 25 ms after the gate driver switches on (start of Home / Passive / Run from IDLE) zero
  command, current PIDs held reset, Passive/Run start from the arm's position — removes the kick at Start
  (DRV8301 current-sense amps waking up).
* **Soft stop `S`**: from Run / Passive the arm is lowered smoothly to 0° (park controller, speed `Py`) and then
  released. `S` again while lowering = release immediately. `O0` unchanged (instant release).
  Events `EV,SOFTSTOP,1/0`, then `EV,PARKED`.

**GUIs (gui/):**
* ExArMo Rehab: "0 here" next to Find limits, "Full auto homing (H1)" under Advanced, **all STOP buttons = soft
  stop** (press again = release now), passive session end lowers to rest, status messages for zero / soft stop.
  Kept from v2.9.3: homing-current slider (Ph) under Advanced.
* ExArMo GUI: "0 here" · "1. Home" · "Full auto" row.  ExArMo Lab: "0 here (Z)" and "Full auto home (H1)".
  (Their "Idle / Stop" stays instant — type `S` in the raw command box for a soft stop.)

**Tuning notes for the new rope** (no reflash, raw command box): rope friction measured ≈ 0.66 N·m (old rope
≈ 0.1). `Pc0.5` matches the friction model to it (fixes self-oscillation in weightless/admittance); set `Pg` to
what is attached (Lab → Gravity → Run gravity stations → Apply fitted g, φ); `Pa0.8` for a gentler sweep start.

**Not compiled in the cloud** (no toolchain access) — built and flashed by the user on 20 Oct.

---

## Firmware v2.9.3 — 18–19 Oct 2026 (homing fix after the rope change) — superseded, NOT in the current main.cpp · ExArMo Rehab v2.9.3

**Symptom:** after fitting the larger capstan rope, every H launched the arm upward and then stopped ~15° short of the
upper stop (18 Oct meta `limits_deg=[2.9,127.0]`; the mechanism is ~140°). The mechanism checked out by hand.

**Cause:** the lift phase used the *session* gravity feed-forward `Pg` (0.60 N·m = patient arm + support, pushed by
ExArMo Rehab on connect), but homing runs with an **empty cuff** (support alone 0.05–0.07 N·m, E9). At the lower stop
the FF alone stepped to 0.77 A lifting nothing → launch (the v2.2 governor only trimmed the push and could not brake).
Past 90°, `cos θ < 0` made the FF a down-pull (−0.47 A at 127°), leaving ~0.6 A of the 1.2 A push for friction —
enough for the old rope, not the stiffer one, so the stall detector fired early.

* **`PG` homing/park gravity** (default 0.06 N·m) used by the lift and the park ramp instead of `Pg`. Set it to what is
  attached while homing (support only). `Pg` is unchanged.
* **Governor can brake:** above `HOME_VCAP` (0.25 rad/s) the homing current ramps through zero to −½·Ph (at ~1.75×VCAP).
  Inactive below VCAP, so it cannot chatter at a stall (the v2.1 problem). Also stops gravity slamming the cuff into
  the lower stop in phase 1.
* **Stricter stall test:** the full push must actually be applied (ramp done *and* not trimmed by the governor);
  600 ms with < 0.02 rad progress (was 400 ms / 0.03 rad) — stick-slip of the stiffer rope no longer looks like a stop.
* **Short-range retry:** a lift that stalls below `PR` (2.3 rad ≈ 132°) is re-ramped once at 1.4×Ph (`EV,HOMEBOOST`).
  If still short, it homes anyway and sends `EV,HOMESHORT,range` after `EV,HOMED`.
* `L` adds `Ph= Ghome= Rmin_deg=` and reports `fw=v2.9.3` (all GUIs accept it — they test for "fw=v2.9").
* **ExArMo Rehab v2.9.3:** "Homing force (Ph)" slider 0.5–2.5 A in the Advanced fold (saved, pushed on connect —
  the firmware's "raise Ph" message finally has a control); plain-language status for HOMEBOOST / HOMESHORT.
* `tools/sim_homing.py`: 1 kHz model of both versions against a plant with stops, stiction and angle-dependent rope
  friction. Old version reproduces the field symptom (launch ~110°/s, stall at 129–132°); v2.9.3 lifts at ~19°/s to
  140°, recovers via HOMEBOOST up to ~1.1 N·m friction at the top, and reports HOMEFAIL instead of a silent short home.
  Simulated plant constants are assumptions — the model confirms the direction of the fix, not the numbers.
  **Not yet compiled with PlatformIO or run on the board.**

**Bring-up:** flash → console shows `READY v2.9.3 … homing Ghome 0.06` → **empty the cuff** → Find limits → expect a
slow descent, no launch on the way up, `EV,HOMED` ≈ 0.05 … 2.39 rad (≈ 3 … 137°) → `L` shows `fw=v2.9.3`. If
HOMEBOOST / HOMESHORT appear, raise Homing force to 1.5–1.8 A (stay ≤ 3 A continuous). The rope changed, so **re-measure
N (E10 method A)** before trusting `N_CAPSTAN = 9.05` and `K_t,j = 0.775`.

## Firmware v2.9.2 — 18 Oct 2026 (motor thermistor from LabTest v2.7.1-R) · ExArMo Rehab + exarmolab g7-C

* **Thermistor on the rehab board**, ported unchanged in method from LabTest v2.7.1-R: 100k NTC bead on the
  end-winding, `3V3 ─[NTC]─ GPIO3/PA2 ─[10k 1 %]─ GND`, read at 5 Hz outside the control interrupt with the
  shared-ADC helper, open/short rejected as wiring faults (`EV,TEMPFAIL` once), `T` raw dump, `X<°C>` beta calibration.
* **Rehab limits** (logbook-thermistor-integration §6 — a limb touches the device; the lab firmware stops at 85 °C):
  ≥ **40 °C** (`PW`) `EV,TEMPWARN` and the Iq limit derates 6 → 3 A linearly; ≥ **55 °C** (`PS`, 0 = off)
  `EV,TEMP` and the board **coasts** to IDLE (driver high-Z, arm free — never a brake); `O1`/`G`/`H`/`V` are refused
  until it is below 50 °C (`EV,TEMPCLEAR`). `O0` is always accepted.
* Telemetry field 13 carries the **measured** temperature while the sensor is valid (the model is re-anchored to it);
  with no bead or a wiring fault the v2.9.1 software model takes over automatically. `L` reports `T_src=ntc|model`.
* New parameters (letters differ from the lab, where h/e/y/o are taken here): `PN` R25 Ω · `PO` divider Ω · `PB` beta ·
  `PT` offset °C · `PW` warn °C · `PS` stop °C.
* Tested on the host against simulated sensor voltages (21 checks: conversion matches the lab table 0.300/0.524/0.829/
  1.454 V at 25/40/55/80 °C, warn, linear derate, coast-stop latch, command refusal, 5 °C re-arm, single TEMPFAIL,
  model fallback from the last measured value, IDLE garbage current ignored). **Not yet compiled with PlatformIO
  or run on the board.**
* ExArMo Rehab: sidebar shows the measured temperature (green < 40, amber < 55, red = stopped; "~" marks the
  estimate) and the new events in plain language. exarmolab g7-C: the rehab board's bead is treated as a real
  sensor (flags bit5, source "thermistor").

**Bring-up:** flash v2.9.2 → console `T` (expect ≈ 0.30 V at 25 °C with 100k/10k; 0.05 V = open wire) → pinch the
bead, the voltage must rise → status `L` shows `T_src=ntc` → optional `PS45` and warm the bead to check the
coast-stop, then `PS55`. Calibrate beta once in a 60–80 °C bath with `X<°C>`.

## Firmware v2.9.1 — 18 Oct 2026 (thermal-model fix) · exarmolab g7-B

* **Fix:** the software winding-temperature estimate `T_est` ran away while the motor was **idle** (≈ 26 000 °C in
  the 18 Oct session, `exo_20261018_175130`), which held the Iq limit at the 3 A thermal derate for the whole
  recording. Cause: in IDLE/FAULT the gate driver is disabled (EN_GATE low), which also switches off the DRV8301
  current-sense amplifiers, so the measured q-current is meaningless — and the model integrated it as copper loss.
  Now: 0 A is used while the driver is off (no current can flow), the current is clamped to 1.5 × the Iq limit
  when powered, and a non-finite or absurd `T_est` is reset to ambient. Status line reports `fw=v2.9.1`; all GUIs
  accept it (they test for "v2.9…"). **Re-flash before the next recording** and run the "IDLE phantom-current
  check" (ExArMoLab_v2.9 → Thermal): T_est must now fall toward ~28 °C in IDLE.
* `gui/exarmolab.py` → **g7-B**: the 18 Oct g7-A (rehab-firmware support) merged with the Mac g6-D1 line
  (tab 11 E10 ratio N + hand sweep, E9 drive-linearity readout). Nothing of either version dropped.

## Motor tester `gui/exarmolab.py` g7-A — 18 Oct 2026  (rehab firmware v2.9 support)

The lab-rig program (`1_ExArMoLab/gui/exarmolab.py`, g5-A/g6) used to refuse the rehab firmware outright
("WRONG FIRMWARE: flash the labtest main.cpp"). It now detects the firmware (13-field telemetry or `fw=v2.9` in
`L` = rehab; 11 fields / `EV,BOOT` = lab) and translates its commands on the rehab firmware, so the E9 ground-truth
tab, the KT/Heat/Capstan/Thermal tabs and the arm-board tabs run on the finished device without reflashing:

| lab command | on rehab v2.9 |
|---|---|
| `C<A>` current | emulated: `V±8` (LAB velocity mode) with the current limit `MLC`+`Pi` ramped to \|A\| at the `Pr` rate; with the shaft blocked the velocity PID saturates at the limit → constant-current stall. Runaway guard (joint > 0.6 rad/s for 0.3 s → `O0` + fault). The ramp level is published as `iq_cmd` (the firmware reports 0 in LAB mode). |
| `C0` / `O0` | limit ramped to 0 (state stays CURRENT) / `O0` and the limit cap restored |
| `H` hold | `G<θ now>` passive hold (needs the rehab homing — Diagnostics has a HOME button) |
| `Pr`, `Ps`, `Pi` | kept on the PC as ramp rate, direction, cap (rehab `Pr` is phase resistance — never forwarded) |
| `Z`, `A`, `K` | blocked (rehab `A` is the assist gain; no watchdog) |
| `V`, `D`, `L`, `MV…` | native; MVP/MVI/MVF are re-sent after `V` because `doLabVel()` re-tunes the PID on entry |

Telemetry: joint angle/velocity × 9.05 × DIR_SIGN → motor-shaft units; states mapped (LAB→CURRENT while emulating,
PASSIVE→HOLDING, RUN/HOMING/FAULT named); flags synthesised with the thermistor bit CLEAR. Temperature source
"model" = the firmware's `T_est`, labelled as an estimate everywhere; values outside −50…400 °C are refused (the
IDLE integrator bug of 18 Oct). The Diagnostics console is now RAW (typed text goes to the board unchanged).
Nothing else in the program changed. Tested headless against a simulated v2.9 board: ramp up/down tracks the
limit, direction, O0 restore, H→G, PID re-send, runaway abort, model temperature gating.

## ExArMo Lab v2.9 — 18 Oct 2026  (`gui/ExArMoLab_v2.9.py`, `tools/analyze_exo_session.py`, `docs/lab_procedures.md`)

New lab program for the full device, driven by the analysis of the first v2.9 session
(`gui/logs/exo_20261018_175130.csv`, DOB, assist 1.12, user in the arm, 50 s). That log showed the control
chain working (tracking RMS 0.78°, K_t,j in the log 0.761 vs 0.775 set, DOB noise 0.024 N·m vs static 0.125 /
direct 0.157) but three things that must be measured or fixed before a device result goes into the thesis:

| Finding (18 Oct log) | What ExArMo Lab adds |
|---|---|
| `T_est` = 26 130 °C before RUN, Iq limit derated to 3 A all session; in RUN `T_est` decays with exactly 1/(R_th·C_th) → the integrator runs away in IDLE (driver high-Z, phantom current) | **Thermal tab**: IDLE phantom-current check (verdict), stall test at a set current vs the model's own prediction. Firmware fix proposed in `docs/lab_procedures.md` §0 (integrate 0 A in IDLE/FAULT, clamp). |
| g = 0.60, φ = 0, J, B, τ_f are typed values; with a user in the loop the log cannot separate gravity-model error from user torque; no hands-off segment, so no σ₀ | **Gravity tab** (E-13a): PASSIVE holds at N stations up and down → g, φ fitted from measured current, hysteresis, at-rest σ₀ of all three estimators; "Apply" sends Pg/Pf. **Plant ID tab** (E-13b): passive sweeps at several speeds/accels → J, B, τ_f, g, φ ± SE; no-human bias/SD of the estimators (E8 condition d); "Apply" sends Pj/Pb/Pc/Pg/Pf. |
| "Assist 1.12" measured as τ_m − G = −0.28·ω − 0.07·sgn(ω), net motor power −0.21 W: the device is a damper (θ_ref lags θ by 1.0° per rad/s; the DOB reads τ_h from the position loop's reaction) | **Estimator tab**: live noise/agreement/lag and the interaction-torque fit with net power; mode protocol steps A with markers and tabulates B_int, C, P per mode. Report as transparency, not assistance, unless P > 0. |
| `D2` delivers ≈ 280 Hz (dt 2–7 ms, firmware `t_ms`) — MCU print cost, not PC loss | Rate label corrected; derivative fits smooth over 50 ms; quote the measured rate. |

`tools/analyze_exo_session.py` reproduces every number above offline from any 13-column log (both GUIs).
Logs: `gui/logs/lab_*.csv` + `.meta.txt` (same format as the control panel, experiment results appended) and
`gui/logs/lab_<experiment>_<time>.json`. Launchers `gui/run_lab.bat` / `run_lab.command` share `gui/.venv`.
Tested offscreen against a simulated v2.9 board: all four experiments end to end, fitted values recover the
simulated g/φ/τ_f/hysteresis, stall test restores `Pi6`/`MLC6` and returns to IDLE.

## ExArMo Rehab v2.9 — 18 Oct 2026  (`gui/ExArMoRehab_v2.9.py`, from `ExArMo_Final.py` of 18 Aug)

The patient/therapist app (Passive / Assistive / Resistive / Games, live ExArMo arm view) was never in the
project folder; it is now part of the release and matched to firmware v2.9. Every command it sends was checked
against firmware v2.9 (H, O0/O1, E, A, G, D, Pg Pn Px Pt Pw Pi Pz Pk Pv Pa Pc) — no letter collisions.

| Fix / change | Why |
|---|---|
| CSV header gains `T_est` (14 columns); rows padded for older firmware | v2.9 sends 13 telemetry fields — the old 13-column header would have mis-aligned every row |
| Recording at 500 Hz (D2) + `.meta.txt` (firmware `L` line, mode, estimator, every saved setting) | same logging standard as the control panel; each log carries its constants |
| Iq-limit sliders default **6 A** (max 8) | v2.4 defaulted to 8 A and re-sent it on every START, overriding the firmware's 6 A safety default |
| **Dead-zone** slider (`Pz`, 0.15 N·m) on Assistive and Resistive | new in firmware v2.9 (E9 noise floor + hysteresis) |
| Gravity g, DOB ω_c, Iq limit, dead-zone are **one shared value** across Assistive/Resistive (sliders move together) and saved to `rehab_settings.json`; park angle, passive acceleration and friction helper are saved too | g tuned with the arm support used to be lost on restart and could differ between pages |
| Saved settings pushed to the board on Connect | a freshly booted board matches what the screen shows |
| Firmware check on Connect (`fw v2.9 ✓` / "fw OLD — flash v2.9") | the app works with v2.8 but the constants are wrong there |
| Sidebar shows estimated motor temperature (green < 60 °C, amber < 70, red = derating); `EV,THERM` and `EV,VBUS` shown as plain-language status | thermal derate is new in v2.9 |
| State 7 (LAB) named; status text "PARKING" instead of "CENTERING" | matches firmware |

Tested on 18 Oct against a simulated v2.9 board (pseudo-serial, 13-field telemetry, homing/thermal events) and a
simulated v2.8 board: homing, all four modes, weightless games, recording (14 columns, 2 ms spacing), meta file,
settings save/restore/push, linked sliders, old-firmware warning; screenshots checked at 1920×1080.

## v2.9 firmware + GUI v2.9 — 16 Oct 2026  (for the §4.8 / E08 re-run with the full arm support)

Every constant that the lab measured between 26 Aug and 15 Oct 2026 is now in the
release firmware, and the bugs found on the bench rig are ported. The 13 Jul 2026
session (`logs/exo_20260713_153238.csv`) was taken **without the finished arm
support** and with the old constants — it is superseded and should not be compared
with the new runs.

### Firmware `firmware/mks_xdrive_mini_firmware/src/main.cpp` v2.8 → v2.9

| What | v2.8 | v2.9 | Source |
|---|---|---|---|
| Joint torque gain τ_m = K_t,j·I_q | N·K_t = 9.0 × 0.0955 = **0.860** N·m/A | **P_Ktj = 0.775** N·m/A (`PK`) | E9 ground truth 15 Oct (6 repeats, ±0.001). v2.8 was 11 % high: no capstan efficiency (83 ± 4 %) and back-EMF K_t |
| Kinematic ratio N | 9.0 | **9.05** (kinematics only) | E10 hand sweep 13 Oct, 9.05 ± 0.05 |
| phase_resistance | 0.54 Ω, set *after* initFOC | **0.47 Ω** (`Pr`), set *before* initFOC | E01 four-wire Kelvin: 0.472 / 0.496 / 0.485 Ω |
| phase_inductance | unset → `tuneCurrentController()` ran `characteriseMotor()` at boot | **0.25 mH** explicit (`Pl`) | lab fix v2-E |
| KV_rating | 114.1 | **123** | E02, three motors 122.5–123.5 rpm/V |
| Dead-zone on τ̂_h | none | **P_dz = 0.15 N·m** (`Pz`), admittance loop only | E9: σ₀ 0.018 N·m, latched hysteresis 0.02–0.13 N·m |
| Iq limit default | 8 A | **6 A** + software thermal derate to P_icont = **3 A** (`Pu`) between 70 and 80 °C estimated | E05: 3 A continuous, 6 A → 80 °C in 25 s |
| Thermal model | — | first-order, C_th 14 J/K, R_th 7.8 K/W, P_cu = 1.5·I²·R; `T_est` = telemetry field 13; `EV,THERM,T,lim` | E05 heating curves |
| Vbus guard | VSENS read used blindly | out-of-range read → 24 V + `EV,VBUS` | lab fix v2-F |
| Current-PID windup | q/d PIDs integrate while driver is high-Z in IDLE | PIDs reset on every IDLE → powered transition | lab fix v2-G |
| LAB velocity mode (`V`) | SimpleFOC default velocity PID (vibrated in E10) | P 0.25, I 3, Tf 40 ms on entry; Tf restored on exit | lab GUI g6-C |
| Telemetry | 12 fields; D4 = 250 Hz max useful | 13 fields (`T_est`); **D2 = 500 Hz** recommended for logging | paper §4.9: 100 Hz logging was the largest B_v scatter term |
| `L` status | state, homed, angle, limits | + Ktj, N, R, L, dz, T_est, iq_lim, **fw=v2.9** | |

Unchanged: DIR_SIGN, homing, passive mode, estimator equations, admittance law,
inner PD gains, gravity model `G = g·cos(θ+φ)` (g = 0.60 default — **re-tune with the
finished arm support**, see procedure).

New `P` letters: `K` K_t,j · `z` dead-zone · `u` continuous Iq · `r` R_ph · `l` L_ph.

### GUI `gui/ExArMoGui_v2.9.py`  (replaces v2.4, archived in `archive/gui_history/`)

* **Impedance tab removed.** Firmware v2.8/v2.9 has no impedance mode (`O2`), and the
  tab's letters collided with firmware parameters: `Pm0.15` switched the torque
  controller to *voltage* mode, `Pa0/Pa1` overwrote the passive acceleration, `Pe1.0`
  set the passive integral gain. "Send all parameters" in v2.4 therefore put the board
  into a wrong state. All 24 letters sent by v2.9 are checked against firmware v2.9.
* **Lab constants tab**: K_t,j 0.775, R 0.47 Ω, L 0.25 mH, continuous Iq 3 A.
* **Dead-zone** and **Coulomb friction** sliders (Admittance / Model tabs).
* **Passive tab**: goto min/max and a min ↔ max sweep sequencer with dwell (firmware `G`).
* **Advanced**: torque-controller mode (`Pm2/1/0`), measured-Iq sign override (`Pq`).
* Telemetry parser accepts 12 (≤ v2.8) or 13 (v2.9) fields; `T_est` readout with
  colour (green < 60 °C, amber < 70 °C, red = derating).
* **Record** writes `<log>.meta.txt`: firmware `L` line, every parameter value,
  estimator, assist, note, and time-stamped markers — so each log carries the
  constants it was taken with.
* **E08 protocol timer**: rest 30 s (park) → rest 30 s (hold mid) → rest 30 s (hold
  near max) → voluntary flexion–extension 120 s → rest 30 s; each stage change is a
  marker. "Next stage" / "Abort" buttons.
* Default logging rate 500 Hz (D2); the rate and `L` are sent on connect.
* "Analyze last recording" finds `analyze_log.py` or the LogAnalyzer in `../tools`.

### Flashing

```
cd firmware/mks_xdrive_mini_firmware
pio run -t upload          # DFU, as before
```
Then connect with the GUI, press **Status** and check the console shows
`... Ktj=0.7750 N=9.05 R=0.470 L=0.250mH dz=0.150 ... fw=v2.9` and `EV,TUNE,1` was
printed at boot (deterministic current-loop tuning; no multi-second characterisation).

### Procedure for the §4.8 re-run (E08) — one afternoon

1. **Note the motor serial installed in the device** in the GUI note field
   (K_t,j was measured on R4001; the other two are within ±2 % in K_t).
2. Home. Confirm `EV,STOP_LOW,iq_sign=±1` and `EV,HOMED`.
3. **Gravity tune with the finished arm support** (this is what the 13 Jul session
   lacked): Admittance, Assist = 0, **dead-zone 0**, estimator DOB. Move the arm by hand
   to ~20°, 45°, 70°, 90° and let go. Adjust *g* (and φ if the drift changes sign across
   the range) until the arm stays where it is left everywhere. Record the final g, φ.
   Do not compute g from the drawing — E9 showed the drawn value 2.5× too high.
4. Set dead-zone back to **0.15 N·m**, Coulomb friction 0.15 N·m.
5. Let the motor cool (T_est < 50 °C). Rate 500 Hz. Press **Record**, then
   **Start protocol**; follow the stage label (rests are for the noise floors, the
   120 s motion stage for the estimator comparison and the B_v(τ̂) figure). Stop
   recording after "complete".
6. Repeat step 5 with Assist = 1.0 (assistive) and once with Assist = −0.5
   (resistance) if time allows. Three recordings ≈ 15 min.
7. Analyse with `tools/ExArMoLogAnalyzer_v2.4.py` (rest/motion split now matches the
   markers in the `.meta.txt`). Expected: DOB rest noise ≈ 15–20 mN·m (E9 floor 18),
   static/direct 5–6× higher; the absolute numbers differ from July by the ratio
   0.775/0.860 of the torque gain.

### Still open (after 25 Oct)
* Thermistor on the rehab board (the software model assumes 28 °C ambient).
* Direction-dependent K_t,j (flexion ≈ 10 % weaker than extension, E7/E9).
* Cogging/friction compensation table, drum bearing clamp, cable pre-tension procedure.

## v2.8 — 22 Aug 2026
Lab velocity mode `V`. Deployed state archived in `releases/v2.8_20260822/`
(firmware v2.8 + GUI v2.4).
