"""Homing state-machine simulation: v2.9.2 (as shipped) vs v2.9.3 (patched).
Joint-frame plant with hard stops, Coulomb + stiction friction, gravity of
whatever is attached. Mirrors controlUpdate() at 1 kHz."""
import math, sys

KTJ = 0.775
IQ_LIM = 6.0
DT = 1e-3

def run(version, Ph=1.2, g_true=0.06, tau_f=0.35, Pg=0.60, Ghome=0.06,
        stop_hi=math.radians(140), J=0.03, B=0.05, start=0.9, verbose=False):
    # --- constants per version ---
    if version == "2.9.2":
        PROGRESS_EPS, STALL_TICKS, BRAKE, boost_ok = 0.03, 400, 0.0, False
    else:
        PROGRESS_EPS, STALL_TICKS, BRAKE, boost_ok = 0.02, 600, 0.5, True
    VCAP, RAMP, MARGIN, MIN_RANGE, RMIN, BOOST = 0.25, 800, 0.05, 0.30, 2.3, 1.4
    K_STOP, D_STOP = 400.0, 8.0            # hard-stop spring/damper [Nm/rad], [Nm s/rad]
    stiction = 1.3 * tau_f

    th, vel = start, 0.0                   # joint angle in the TRUE frame (lower stop = 0)
    offset = None                          # firmware joint zero (unknown until phase 1)
    state = "NEG"; tick = 0; state_tick = 0
    progress_ref = th; progress_tick = 0; boost = 1.0
    log = {"vmax_lift": 0.0, "vmax_lower": 0.0, "events": [], "range": None, "t": None,
           "peak_i": 0.0, "stop_hit_v": 0.0, "lift_imin": 9}
    while tick < 60000:
        tick += 1
        theta_fw = th if offset is None else th - offset   # what the firmware sees
        lifting = state == "POS"; d = 1.0 if lifting else -1.0
        ip = min(Ph * boost, IQ_LIM)
        frac = min(max((tick - state_tick) / (RAMP + 1.0), 0.0), 1.0)
        iq_push = d * ip * frac
        over = d * vel - VCAP
        if over > 0:
            iq_push -= d * (ip / (0.5 * VCAP)) * over
        if d > 0: iq_push = min(max(iq_push, -BRAKE * ip), ip)
        else:     iq_push = min(max(iq_push, -ip), BRAKE * ip)
        if lifting:
            gcoef = Pg if version == "2.9.2" else Ghome
            iq_grav = gcoef * math.cos(theta_fw) / KTJ
        else:
            iq_grav = 0.0
        iq = min(max(iq_push + iq_grav, -IQ_LIM), IQ_LIM)
        log["peak_i"] = max(log["peak_i"], abs(iq))
        if version == "2.9.2":
            pushing = frac > 0.95
        else:
            pushing = frac > 0.95 and d * iq_push > 0.9 * ip
        if d * (theta_fw - progress_ref) > PROGRESS_EPS:
            progress_ref = theta_fw; progress_tick = tick
        if not pushing: progress_tick = tick
        stalled = tick - progress_tick > STALL_TICKS

        # ---- plant ----
        tau_m = KTJ * iq
        tau_g = -g_true * math.cos(th)         # gravity pulls the support DOWN (toward θ=0)
        tau_stop = 0.0
        if th < 0:        tau_stop = -K_STOP * th - D_STOP * vel
        if th > stop_hi:  tau_stop = -K_STOP * (th - stop_hi) - D_STOP * vel
        tau_net = tau_m + tau_g + tau_stop - B * vel
        if abs(vel) < 1e-3:
            if abs(tau_net) <= stiction: tau_net = 0.0; vel = 0.0
            else: tau_net -= math.copysign(tau_f, tau_net)
        else:
            tau_net -= math.copysign(tau_f, vel)
        acc = tau_net / J
        vel_new = vel + acc * DT
        if vel != 0 and (vel_new * vel < 0) and abs(tau_net) < tau_f: vel_new = 0.0
        vel = vel_new
        th += vel * DT
        if lifting: log["vmax_lift"] = max(log["vmax_lift"], vel)
        else:       log["vmax_lower"] = max(log["vmax_lower"], -vel)
        if lifting and th > stop_hi - 0.01: log["stop_hit_v"] = max(log["stop_hit_v"], vel)

        # ---- transitions ----
        if stalled:
            if state == "NEG":
                offset = th
                log["events"].append((tick, "STOP_LOW", round(math.degrees(th), 1)))
                state = "POS"; state_tick = tick; progress_ref = 0.0; progress_tick = tick
            else:
                rng = theta_fw
                if rng < MIN_RANGE:
                    log["events"].append((tick, "HOMEFAIL", round(math.degrees(rng), 1))); break
                if boost_ok and rng < RMIN and boost < BOOST:
                    boost = BOOST; state_tick = tick; progress_ref = theta_fw; progress_tick = tick
                    log["events"].append((tick, "HOMEBOOST", round(math.degrees(rng), 1))); continue
                log["events"].append((tick, "HOMESHORT" if rng < RMIN else "HOMED", round(math.degrees(rng), 1)))
                log["range"] = math.degrees(rng); log["t"] = tick / 1000; break
        if tick - state_tick > 25000:
            log["events"].append((tick, "HOMETIMEOUT", state)); break
    return log

def show(title, L):
    print(f"\n{title}")
    print(f"  events: {L['events']}")
    print(f"  found range {L['range'] and round(L['range'],1)}°  in {L['t']} s  | peak |Iq| {L['peak_i']:.2f} A")
    print(f"  max speed lowering {math.degrees(L['vmax_lower']):.0f}°/s, lifting {math.degrees(L['vmax_lift']):.0f}°/s, "
          f"speed when reaching upper stop {math.degrees(L['stop_hit_v']):.0f}°/s")

if __name__ == "__main__":
    for tf in (0.15, 0.35, 0.5):
        print(f"\n=== Empty cuff (support only, g=0.06), rope friction τ_f={tf} N·m ===")
        show("v2.9.2 as shipped (Pg=0.60 FF, Ph=1.2)", run("2.9.2", tau_f=tf))
        show("v2.9.3 patched   (Ghome=0.06, Ph=1.2)", run("2.9.3", tau_f=tf))
    print("\n=== Patient in cuff during homing (g=0.60) — NOT recommended, but should not break ===")
    show("v2.9.3 patched (Ghome=0.06, Ph=1.2)", run("2.9.3", tau_f=0.35, g_true=0.60))
    show("v2.9.3 patched (Ghome=0.06, Ph=2.0)", run("2.9.3", tau_f=0.35, g_true=0.60, Ph=2.0))
