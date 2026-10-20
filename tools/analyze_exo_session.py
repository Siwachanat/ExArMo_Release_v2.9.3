#!/usr/bin/env python3
"""
analyze_exo_session.py — quick-look analysis of an ExArMo rehab-firmware session log
(columns: t_ms,state,theta,theta_ref,vel,iq_meas,tau_m,tau_s,tau_d,tau_dob,Bv,iq_cmd,T_est)

Usage:
    python analyze_exo_session.py exo_YYYYMMDD_HHMMSS.csv [--g 0.60 --phi 0.0 --out figures/]

Prints a metrics table (tracking, estimator noise, interaction torque vs velocity,
variable-damping law, thermal-estimator sanity) and writes a 6-panel PNG.
First used on gui/logs/exo_20261018_175130.csv (18 Oct 2026); the same fits run live inside
gui/ExArMoLab_v2.9.py. See docs/lab_procedures.md for what each number means.
"""
import argparse, sys
import numpy as np
import pandas as pd


def fit(A, y):
    c, res, _, _ = np.linalg.lstsq(A, y, rcond=None)
    rms = np.sqrt(res[0] / len(y)) if len(res) else np.nan
    return c, rms


def xcorr_lag(a, b, maxlag=60):
    a = a - a.mean(); b = b - b.mean()
    best = (-1, 0)
    for l in range(-maxlag, maxlag + 1):
        x = a[max(0, l):len(a) + min(0, l)]
        y = b[max(0, -l):len(b) + min(0, -l)]
        r = np.corrcoef(x, y)[0, 1]
        if r > best[0]:
            best = (r, l)
    return best[1], best[0]


def analyze(path, g=0.60, phi=0.0, out=None, show_window=(10, 20)):
    df = pd.read_csv(path)
    t = (df.t_ms - df.t_ms.iloc[0]) / 1000.0
    dt = t.diff().fillna(t.diff().median())
    dur = t.iloc[-1]
    rate = len(df) / dur
    tau_g = g * np.cos(df.theta - phi)
    ex = df.tau_m - tau_g                       # motor torque beyond gravity comp
    err_deg = np.degrees(df.theta - df.theta_ref)
    sg = np.sign(df.vel)

    R = {}
    R['duration_s'] = dur
    R['log_rate_Hz'] = rate
    R['dt_ms_min_max'] = (dt.min() * 1000, dt.max() * 1000)
    R['theta_range_deg'] = (np.degrees(df.theta.min()), np.degrees(df.theta.max()))
    R['speed_deg_s_p50_p95_max'] = tuple(np.degrees(df.vel.abs().quantile([.5, .95, 1.0])))
    R['track_rms_deg'] = np.sqrt((err_deg ** 2).mean())
    R['track_max_deg'] = err_deg.abs().max()
    R['track_mean_deg'] = err_deg.mean()
    R['track_deg_per_rad_s'] = np.polyfit(df.vel, err_deg, 1)[0]

    # K_t,j as used by the firmware
    (k, b), _ = fit(np.c_[df.iq_meas, np.ones(len(df))], df.tau_m)
    R['Ktj_firmware_Nm_per_A'] = -k
    R['tau_m_iq_corr'] = np.corrcoef(df.iq_meas, df.tau_m)[0, 1]

    # estimator noise (sample-to-sample diff SD) and mutual agreement
    for c in ['tau_s', 'tau_d', 'tau_dob']:
        R[f'noise_{c}'] = df[c].diff().std()
    for a, b_ in [('tau_s', 'tau_d'), ('tau_s', 'tau_dob'), ('tau_d', 'tau_dob')]:
        d = df[a] - df[b_]
        R[f'rmse_{a}_vs_{b_}'] = np.sqrt((d ** 2).mean())
    R['lag_dob_vs_static_ms'] = xcorr_lag(df.tau_s.values, df.tau_dob.values)[0] * dt.median() * 1000

    # model-term identification (checks that GUI params reached the board)
    c, rms = fit(np.c_[np.cos(df.theta), np.sin(df.theta), df.tau_m, sg, np.ones(len(df))], df.tau_s)
    R['tau_s_fit(cos,sin,tau_m,sgn,c)'] = (tuple(np.round(c, 3)), rms)
    acc = (df.vel.diff() / dt)
    m = ~acc.isna()
    c, rms = fit(np.c_[df.tau_s[m], df.vel[m], acc[m], np.ones(m.sum())], df.tau_d[m])
    R['tau_d_fit(tau_s,vel,acc,c)'] = (tuple(np.round(c, 4)), rms)

    # interaction torque: motor torque beyond gravity vs velocity
    c, rms = fit(np.c_[df.vel, sg, np.ones(len(df))], ex)
    R['tau_m-tau_g = B*w + C*sgn + c'] = (tuple(np.round(c, 3)), rms)
    R['net_motor_power_W'] = (ex * df.vel).mean()
    R['corr_tau_dob_vel'] = np.corrcoef(df.tau_dob, df.vel)[0, 1]

    # variable damping law
    c, rms = fit(np.c_[df.tau_dob.abs(), np.ones(len(df))], df.Bv)
    R['Bv = a*|tau_dob| + b'] = (tuple(np.round(c, 3)), rms, (df.Bv.min(), df.Bv.max()))

    # dead-zone / hands-off
    R['frac_|tau_dob|<0.1'] = (df.tau_dob.abs() < 0.1).mean()
    still = (df.vel.abs() < 0.03).values
    best = cur = 0
    for v in still:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    R['longest_still_s'] = best * dt.median()

    # thermal estimator sanity: dT/dt = a*i^2 + b*T + c  (expect b<0, equilibrium ≈ T_amb)
    dT = df.T_est.diff() / dt
    m = ~dT.isna()
    c, _ = fit(np.c_[df.iq_meas[m] ** 2, df.T_est[m], np.ones(m.sum())], dT[m])
    R['T_est dT/dt = a*i2 + b*T + c'] = tuple(np.round(c, 5))
    R['T_est_start_end'] = (df.T_est.iloc[0], df.T_est.iloc[-1])
    R['T_est_equilibrium'] = -c[2] / c[1] if c[1] != 0 else np.nan
    R['T_est_time_const_s'] = -1 / c[1] if c[1] < 0 else np.nan

    R['iq_abs_max_A'] = df.iq_meas.abs().max()
    R['iq_cmd_min_max'] = (df.iq_cmd.min(), df.iq_cmd.max())
    R['iq_meas_vs_cmd_rmse_A'] = np.sqrt(((df.iq_meas - df.iq_cmd) ** 2).mean())

    for k_, v in R.items():
        print(f'{k_:40s} {v}')

    if out:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        w0, w1 = show_window
        fig, ax = plt.subplots(3, 2, figsize=(14, 10)); ax = ax.ravel()
        a = ax[0]; a.plot(t, np.degrees(df.theta), lw=.8, label='θ'); a.plot(t, np.degrees(df.theta_ref), lw=.8, label='θ_ref')
        a.set_xlim(w0, w1); a.set_ylabel('deg'); a.legend(); a.set_title(f'Tracking: RMS {R["track_rms_deg"]:.2f}°, max {R["track_max_deg"]:.2f}°')
        a = ax[1]; a.plot(t, df.tau_s, lw=.5, alpha=.6, label='static'); a.plot(t, df.tau_d, lw=.5, alpha=.6, label='direct'); a.plot(t, df.tau_dob, lw=1.2, color='k', label='DOB')
        a.set_xlim(w0, w1); a.set_ylabel('τ_h est [N·m]'); a.legend(); a.set_title('Three estimators')
        c = R['tau_m-tau_g = B*w + C*sgn + c'][0]; v = np.linspace(df.vel.min(), df.vel.max(), 50)
        a = ax[2]; a.scatter(df.vel, ex, s=2, alpha=.3); a.plot(v, c[0] * v + c[1] * np.sign(v) + c[2], 'r', label=f'{c[0]:.3f}·ω {c[1]:+.3f}·sgn(ω)')
        a.set_xlabel('ω [rad/s]'); a.set_ylabel('τ_m − τ_g(θ) [N·m]'); a.legend(); a.set_title(f'Interaction torque (net motor power {R["net_motor_power_W"]:+.2f} W)')
        c = R['Bv = a*|tau_dob| + b'][0]; x = np.linspace(0, df.tau_dob.abs().max(), 50)
        a = ax[3]; a.scatter(df.tau_dob.abs(), df.Bv, s=2, alpha=.3); a.plot(x, np.clip(c[0] * x + c[1], df.Bv.min(), df.Bv.max()), 'r', label=f'{c[1]:.3f} {c[0]:+.3f}·|τ|')
        a.set_xlabel('|τ_DOB| [N·m]'); a.set_ylabel('B_v'); a.legend(); a.set_title('Variable damping law')
        a = ax[4]; a.plot(t, df.T_est); a.set_xlabel('s'); a.set_ylabel('T_est'); a.set_title(f'Thermal estimator (τ≈{R["T_est_time_const_s"]:.0f} s → {R["T_est_equilibrium"]:.0f})')
        a = ax[5]; a.scatter(df.vel, err_deg, s=2, alpha=.3); a.plot(v, R['track_deg_per_rad_s'] * v, 'r', label=f'{R["track_deg_per_rad_s"]:.2f}° per rad/s')
        a.set_xlabel('ω [rad/s]'); a.set_ylabel('θ − θ_ref [deg]'); a.legend(); a.set_title('Tracking error vs velocity')
        plt.tight_layout(); plt.savefig(out, dpi=110); print('figure ->', out)
    return R


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('csv'); p.add_argument('--g', type=float, default=0.60); p.add_argument('--phi', type=float, default=0.0)
    p.add_argument('--out', default=None)
    a = p.parse_args()
    analyze(a.csv, a.g, a.phi, a.out or a.csv.replace('.csv', '_analysis.png'))
