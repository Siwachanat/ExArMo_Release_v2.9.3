# ExArMo — Release v2.9.3

Firmware and Python applications for the ExArMo single-degree-of-freedom elbow exoskeleton using an MKS XDrive Mini / STM32F405 motor board.

**Version distinction:** the release directory and Rehab application are named v2.9.3, but the included `main.cpp` identifies itself as **v2.9z**. It uses the v2.9 homing implementation with manual zero, thermistor support, driver wake-up handling, and soft stop. The superseded v2.9.3 homing changes described in the changelog are **not included** in this source.

## Contents

| Path | Purpose |
| --- | --- |
| `firmware/mks_xdrive_mini_firmware/` | PlatformIO firmware project; `src/main.cpp` is the main source |
| `gui/ExArMoRehab_v2.9.3.py` | Tkinter rehabilitation interface: Passive, Assistive, Resistive, and Games |
| `gui/ExArMoGui_v2.9.py` | PyQt5 control panel and live plots |
| `gui/ExArMoLab_v2.9.py` | PyQt5 identification and laboratory experiments |
| `gui/exarmolab.py` | Tkinter motor tester / lab-rig interface |
| `tools/` | Recording analysis and homing simulation |
| `docs/` | Laboratory procedures, E08 procedure, and firmware constants |

See [CHANGELOG.md](CHANGELOG.md) for the supplied development history. Historical measurements and test claims there are records from the original release, not new verification performed for this repository.

## Start on Windows or macOS

Install Python with Tkinter support. Dependencies are listed in [gui/requirements.txt](gui/requirements.txt).

- Windows: launch `gui/run_rehab.bat`, `gui/run_gui.bat`, or `gui/run_lab.bat`.
- macOS: launch the corresponding `.command` file, or run it with Bash. A Python installation with Tkinter is needed for Rehab.
- Manual installation in your own Python environment:

```sh
python -m pip install -r gui/requirements.txt
python gui/ExArMoRehab_v2.9.3.py
```

Use only one application on a serial port at a time. Select the board's port and connect. Firmware uses **921600 baud**; `platformio.ini` currently specifies 230400 for its serial monitor, so override the monitor baud when using that tool:

```sh
pio device monitor -b 921600
```

The board's boot message starts with `READY v2.9z`; command `L` reports `fw=v2.9z`.

## Build and flash firmware

Install PlatformIO, then run:

```sh
cd firmware/mks_xdrive_mini_firmware
pio run
```

To upload, put the board in DFU mode (hold BOOT, press RESET, release BOOT), then:

```sh
pio run -t upload
```

Windows may require a WinUSB driver for the STM32 bootloader. Reconnect USB after flashing and select the new serial port.

The configuration pins Simple FOC to 2.4.0 and Arduino-FOC-drivers to commit `61751ea`. SimpleCANio and the PlatformIO platform are unpinned; first-time builds need network access and may resolve different dependency versions. Build caches and prebuilt binaries are omitted from this repository; build the included source before flashing. This repository preparation did not compile or flash firmware.

## Raspberry Pi setup

The Python source enumerates serial ports using PySerial and does not require a fixed Windows COM port. Raspberry Pi operation has **not been tested as part of this repository preparation**. The following is a proposed desktop setup based on the supplied deployment notes.

Use a Raspberry Pi with a desktop OS, its appropriate power supply, and the motor's separate power supply. On Raspberry Pi OS, distribution packages are a practical way to install Qt and scientific dependencies:

```sh
sudo apt update
sudo apt install -y python3-tk python3-pyqt5 python3-pyqtgraph python3-serial \
  python3-numpy python3-pandas python3-matplotlib python3-scipy dfu-util
sudo usermod -aG dialout "$USER"
sudo reboot
```

Distribution package versions may differ from the pip requirements; verify the applications on the target OS. Do not copy a Windows `.venv` to the Pi.

```sh
cd ~/ExArMo_Release_v2.9.3/gui
python3 ExArMoRehab_v2.9.3.py
# Alternatives, one at a time:
python3 ExArMoGui_v2.9.py
python3 ExArMoLab_v2.9.py
```

Select the actual device in the port menu; `/dev/ttyACM0` is a common example, not a guaranteed name. Start Python directly rather than using Windows `.bat` launchers. Fonts may be substituted. If live plotting is slow, try `D10` (requested 100 Hz). `D2` requests 500 Hz, but serial output and host processing can reduce the measured rate.

Optional DFU flashing of a binary built from this source:

```sh
sudo dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

Optional desktop autostart: create `~/.config/autostart/exarmo.desktop` with the following, replacing USER and the path with your own:

```ini
[Desktop Entry]
Type=Application
Name=ExArMo Rehab
Exec=python3 /home/USER/ExArMo_Release_v2.9.3/gui/ExArMoRehab_v2.9.3.py
```

## Homing and the larger rope

The reported symptoms were an upward launch / overshoot and stopping short of the mechanical upper position after changing the rope. The supplied history attributes these to load-model mismatch and increased friction, but that diagnosis has not been independently established here.

Current v2.9z supports:

- `Z` / **0 here**: set the current position as zero while idle. Position and support the arm at the intended bottom reference first.
- `H` / **Find limits**: with a manual zero, skip the downward search and find the upper limit.
- `H1` / **Full auto homing**: perform the full down-and-up search.
- `Ph`: homing-current setting; the Rehab Advanced panel exposes a slider.

The separate homing gravity parameter and short-range retry described under historical v2.9.3 are absent. Do not assume those fixes are active. A friction stall can be mistaken for a limit; confirm the measured range against the mechanism before operation. Rope diameter, winding geometry, tension, and compliance should be checked before treating a higher current setting as a fix. Revalidate the effective capstan ratio and joint torque calibration after a rope change.

## Weightless game instability

Weightless uses admittance (`O1`) with assist 1. The Rehab GUI sends `Pn0.05` and `Px0.15` when enabled. The firmware estimates interaction torque using motor current and a model; gravity/friction errors can therefore cause unintended motion, while low damping and rope compliance can contribute to oscillation.

The Claude discussion suggested friction around 0.66 N·m for the new rope, but that number is an unverified tuning observation here. The firmware defaults to `Pc=0.15` and clamps `Pc` to 0–0.5 N·m, so `Pc0.5` cannot exactly represent 0.66 N·m.

The suggested commands `Pc0.5`, `Px0.4`, and `Pz0.25` are **experimental tuning candidates, not validated settings or a guaranteed fix**. Gravity must match the attached load; `Pg0.06` for an empty cuff and `Pg0.6` for a loaded arm are examples from the discussion, not universal calibration values. Use the Lab gravity measurements and document the actual configuration. Validate on a supported bench setup before attaching a person.

Rehab sends saved parameters including `Pc` and `Pg` on connection, and re-sends `Px0.15` whenever Weightless is enabled. Raw-command adjustments can therefore be overwritten. The existing friction slider tops out at 0.4 N·m although firmware accepts 0.5 N·m. No control or tuning code was changed for this repository.

## STOP behavior — read before operation

In the included firmware, `S` from a homed Run or Passive state enters the park controller and lowers toward **`lim_min`, the learned lower limit**. The interface describes this as 0°, but it need not be exactly physical zero.

- Rehab STOP buttons send the soft-stop command; passive-session completion also lowers to rest.
- `Py` sets park reference speed, default **0.4 rad/s** (about 23°/s). This is a commanded reference speed, not a measured guarantee.
- Release occurs after settling within 0.03 rad and below 0.05 rad/s, **or after an eight-second timeout**, even if the target was not reached.
- A second `S` during soft stop releases immediately. `O0` also releases immediately.
- In other states, including homing/lab/idle, `S` falls back to immediate stop.
- Switching pages, closing the application, and some setup actions still use immediate release. Control panel / Lab Idle buttons also remain immediate release; their raw command boxes accept `S`.

Soft stop is not a mechanical brake or a guarantee against falling. Loss of motor power, faults, thermal stops, immediate release, or timeout can leave the arm unsupported. Provide suitable mechanical support and validate behavior on the actual mechanism.

Thermal defaults in this source are warning/derating at 40 °C and coast-stop at 55 °C. By default, the NTC is read while the motor is off and the model continues during operation. Do not interpret every temperature sample as a live measurement. See the current changelog entry for details.

## Serial command summary

| Command | Meaning |
| --- | --- |
| `L` | Status and firmware version |
| `Z` | Manual zero while idle |
| `H`, `H1` | Home / full automatic home |
| `O1`, `O0` | Run / immediate idle and release |
| `S` | Soft stop where supported; second press releases |
| `G<rad>` | Passive position target |
| `A<value>` | Assist setting |
| `E0`, `E1`, `E2` | Estimator selection |
| `D<ms>` | Requested telemetry interval |
| `P<letter><value>` | Parameter setting; see `doParam` in `main.cpp` |

Telemetry:

```text
D,t_ms,state,theta,theta_ref,vel,iq_meas,tau_m,tau_s,tau_d,tau_dob,Bv,iq_cmd,T_est
```

Angles are radians and torques are N·m. Consult the source for accepted parameter ranges.

## Logs and analysis

Applications create local recordings under `gui/logs/`. Use `tools/run_analyzer.bat` / `.command`, or:

```sh
python tools/analyze_exo_session.py path/to/recording.csv --g 0.60 --phi 0
```

Supply the gravity parameters appropriate to the recording. Additional procedures:

- [Lab procedures](docs/lab_procedures.md)
- [E08 rerun procedure](docs/E08_rerun_procedure.md)
- [Firmware constants](docs/firmware_v2.9_constants.md)

These supporting documents include historical material; current source takes precedence where versions differ.

## Repository scope and contributions

This copy excludes virtual environments, PlatformIO caches, generated logs, editor settings, and local Rehab settings. Firmware and application source files are unchanged from the provided release. Recreate local settings through the GUI and review them before connecting hardware.

For changes, state the firmware/GUI versions, hardware and rope configuration, reproduction steps, and validation performed. Keep build artifacts and personal/session data out of commits. No license has been added; the owner should choose one before representing this project as openly licensed.
