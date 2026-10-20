# ExArMo Lab v2.9 — experiment procedures on the full device (firmware v2.9)

`gui/ExArMoLab_v2.9.py` (`run_lab.bat` / `run_lab.command`) is the lab companion of the control
panel. It uses only firmware-v2.9 commands (`H O0/O1 E A G V D L P… M…`) and automates the
experiments that the 18 Oct analysis of `logs/exo_20261018_175130.csv` showed are still missing
before a device-level result can go into the thesis. Each experiment writes
`gui/logs/lab_<experiment>_<time>.json` (results + firmware status + note) and, where it recorded,
a 13-column `.csv` in the same format as the control panel.

Order for one afternoon: **Thermal IDLE check → Home → Gravity → Plant ID → apply → Mode protocol.**

## 0. Before anything: Thermal tab → "IDLE phantom-current check"

> **Fixed in rehab firmware v2.9.1 (18 Oct 2026, 21:40)** — the IDLE run-away below is corrected exactly as
> suggested (0 A while the driver is off, current clamp, range reset). Flash v2.9.1 and this check should pass;
> keep running it once per session as a guard.

Board in IDLE (`O0`), motor cold. Run 20 s. The firmware's thermal integrator runs on the
*measured* current at all times, including IDLE, when the gate driver is high-Z and the current
sense has no valid reading. On 18 Oct `T_est` was already 26 130 °C **before** RUN was pressed,
which means the integrator ran away while idle; in RUN it then decayed with exactly the model's
time constant (fitted 0.0094 s⁻¹ = 1/(R_th·C_th) = 1/(7.8 × 14)), so the cooling term is right and
only the IDLE accumulation is wrong.

Verdict "✗ T_est is NOT cooling in IDLE" → fix the firmware before any thermal claim. Suggested
change in `controlUpdate()` (main.cpp, the `[v2.9] thermal estimate` block):

```cpp
  // [v2.9] thermal estimate from the MEASURED current (1 kHz Euler step)
  {
    // driver high-Z in IDLE/FAULT: the low-side current sense is not valid there,
    // and nothing is heating → integrate 0 A instead of a phantom reading
    bool live = (state != ST_IDLE && state != ST_FAULT);
    float ith = live ? fmaxf(fabsf(iq_meas), fabsf(s_iq_cmd)) : 0.0f;
    ith = fminf(ith, 2.0f * P_iq_lim);                       // a sensor glitch cannot cook the model
    float pcu = 1.5f * ith * ith * P_R;
    T_est += ((pcu - (T_est - TH_AMB) / TH_RTH) / TH_C) * DT;
    T_est  = clampf(T_est, TH_AMB - 10.0f, 250.0f);          // physical range only
    ...
```

Also consider initialising `T_est = TH_AMB` again in `doHome()` after a long idle, and printing
`EV,THERM` only on a 0→1 transition (already the case).

Re-run the IDLE check after flashing: slope must be ≤ 0 and ≈ −(T−28)/109 K/s.

## 1. Home, then Gravity tab (E-13a) — nobody touches the arm

Stations default `15, 35, 55, 75, 95, 115°` (inside the homed limits), settle 3 s, dwell 10 s,
ascending then descending. The firmware holds each station in PASSIVE (`G`, PID + integral), so the
measured holding torque at rest is the gravity torque plus the latched drive hysteresis:

    τ_m(θ) = K_t,j·I_q = g·cos(θ+φ)  ± hysteresis

The tab fits g and φ **from measured current**, independent of the model values on the board, and
reports per station: τ_m, the three estimators (they should read ≈ 0; what they read is the
gravity-model residual), σ of τ̂_DOB over the dwell (= at-rest noise floor σ₀ in admittance
operation — E9 gave 0.018 N·m in current mode) and the up/down hysteresis half-width (E9: 0.02–0.13 N·m).
"Apply fitted g, φ" sends `Pg` / `Pf`. Expect the fitted g to differ from the typed 0.60; the 18 Oct
run could not tell gravity-model error from user torque because a user was in the loop throughout.

Acceptance (write into the logbook before running): R² ≥ 0.99 over ≥ 6 stations; residual of the
*applied* model ≤ 0.05 N·m at every station; σ₀(DOB) ≤ 0.03 N·m; hysteresis ≤ 0.15 N·m
(otherwise the dead-zone 0.15 is too small — see E9 §4).

## 2. Plant ID tab (E-13b) — nobody touches the arm

Motor-driven sweeps between min/max (default 15–110°) for every speed × accel combination
(defaults 0.3/0.6/1.0 rad/s × 1.2/3.0 rad/s², 3 reps). Least squares on the whole motion:

    τ_m = J·α + B·ω + g·cos(θ+φ) + τ_f·sgn(ω)         (τ_h = 0)

with α from the smoothed derivative of the telemetry velocity (50 ms window). Outputs J, B, τ_f, g, φ
with standard errors and R². J is the full reflected inertia at the joint (rotor × 9.05² included),
which is what the DOB's `P_J` must be. The warning "low acceleration content" means the accel list
is too gentle for J — raise it to 3–5 rad/s² (the passive profile allows 5). "Apply" sends
`Pj Pb Pc Pg Pf`.

The same run gives **E8 condition (d)** for free: bias and SD of static / direct / DOB while the
motor moves the arm and no human is present. Prediction to write down first: static biased by
−J·α (small at these accelerations), direct noisiest, DOB in between with |bias| < 0.05 N·m.

## 3. Estimator tab (E8 / transparency) — user in the arm

Live window numbers: estimator noise SD (18 Oct: static 0.125, direct 0.157, DOB 0.024 N·m),
mutual RMSE, DOB lag vs static (−8 ms), and in RUN the **interaction-torque fit**

    τ_m − G(θ) = B_int·ω + C·sgn(ω),   net motor power P = mean[(τ_m − G)·ω]

18 Oct, "assist A = 1.12": B_int = −0.28 N·m·s/rad, C = −0.07 N·m, P = −0.21 W, i.e. the device
is a damper and the user does the work. This is the architecture (the DOB measures τ_h from the
position loop's reaction, so θ_ref always lags θ), not a tuning error. Report it as
*transparency / residual interaction torque*; do not call it assistance unless P > 0.

Mode protocol (`A:seconds` list, default `0:30, 1.0:30, -0.5:30, 1.12:30`): press RUN with the
user in the arm, then "Run mode protocol". Markers go into the recording; the result table gives
B_int, C, P, tracking RMS and DOB noise per A. The figure for the thesis is τ_m − G vs ω with one
fitted line per mode on the same axes — the slopes are the behavioural result. If true assistance
is wanted, the firmware needs a feed-forward term (e.g. `tau_cmd += (A−1)·τ̂_h`) and the protocol
is re-run to show P turning positive.

## 4. Thermal stall test — arm blocked

Only after the IDLE check passes. Block the arm with the E9 strut (preferred) or let it rest on
the hard stop in the push direction, motor cold (T_est < 40 °C). Set the current (start with 2 A,
then 3 A = the continuous rating) and the duration (120–300 s). The GUI sets `Pi` and `MLC` to the
test current and starts LAB velocity mode `V±5` (motor rad/s ≈ 0.55 rad/s at the joint); when the
arm is blocked the velocity PID saturates at the current limit, i.e. a constant-current stall.
It stops with `O0` and restores the 6 A limit.

Result: fitted τ and T_ss of `T_est` against the model's own prediction (τ = R_th·C_th = 109 s;
T_ss = 28 + 1.5·I²·0.47·7.8 → 77 °C at 3 A). There is no thermistor on the rehab board, so this is
a self-consistency check of the implementation; the physical validation is E05 on the lab rig
(3 A plateau 77 °C, 6 A → 80 °C in 25 s). Warn if the measured current misses the set point by
> 0.3 A (arm not blocked, or derate already active).

## 5. Logging rate

The board prints 13 floats per frame; at `D2` it delivers ≈ 280 Hz (dt 2–7 ms), not 500 Hz — the
`t_ms` stamps are the firmware's own, so this is print cost on the MCU, not PC-side loss. Quote the
measured rate in the thesis. The derivative-based fits in the Plant-ID tab smooth over 50 ms for
that reason.

## Offline re-analysis

`tools/analyze_exo_session.py <csv> --g <fitted g> --phi <fitted φ>` reproduces the 18 Oct metrics
(tracking, estimator noise and agreement, interaction fit, B_v law, thermal sanity) on any 13-column
log from either GUI and writes a 6-panel PNG.

## 6. Motor tester `exarmolab.py` on the rehab firmware (g7-A)

Start `run_motortester`, connect the device port (921600) and, for E9/KT/Heat, the arm sensor board as before.
The footer shows `[REHAB v2.9 · lab emulation]` once the board answers. Then:

1. **Diagnostics → HOME (rehab H)** once per power-up if you will use HOLD (KT · Add weight) — the rehab `G` hold
   needs the joint limits. Current-mode tabs (E9, KT · Loadcell, Heat, Capstan, Thermal) do not need homing.
2. **Block the shaft before any current command.** The emulation stalls LAB velocity mode against the current
   limit; if the shaft is free the arm moves at ≈ 0.9 rad/s and the runaway guard releases the motor within 0.3 s.
   In LAB mode the firmware applies **no joint limits** — the strut or the load cell is the only stop.
3. **Direction:** `Ps±1` on Diagnostics flips the sign of the emulated current exactly as it did on the lab firmware
   (jog, watch the grams). The E9 tab's `both` directions work unchanged.
4. **Current accuracy:** the measured current equals the commanded limit to the accuracy of the foc_current loop
   (18 Oct log: I_q,meas vs cmd RMSE 0.11 A at ≈280 Hz logging — the same number as in admittance). Ramps up to
   ≈ 15 A/s follow the limit; `Pr20` fiducial pulses are slightly slower than on the lab firmware (irrelevant).
5. **Temperature:** there is no thermistor on the rehab board. The Thermal and Heat tabs run on the firmware's
   `T_est` and say "firmware MODEL" in every label and log — use them to check the implementation against E05, not
   as a thermal measurement. If `T_est` is impossible the source is refused and Diagnostics shows why (§0).
6. **What the KT tabs measure here:** the torque reaches the cell through the capstan, so the slope is K_t,j
   (N·m per reported ampere at the joint, E9: 0.775), not the bare-motor K_t of the lab rig.
