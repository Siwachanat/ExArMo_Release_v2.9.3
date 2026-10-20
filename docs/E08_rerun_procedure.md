# E08 — estimator comparison on the full device (firmware/GUI v2.9)


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

