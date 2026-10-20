# Firmware v2.9 — constants and their sources


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

