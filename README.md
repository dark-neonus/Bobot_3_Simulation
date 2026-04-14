# Bobot-3 Dynamics & Control Simulator

Digital-twin simulation scaffold for a two-wheel self-balancing robot with:

- PyBullet rigid-body dynamics
- Decoupled 2000 Hz physics and 500 Hz control loop timing
- PID balance controller (ESP32-style sample-and-hold update)
- Motor/electrical model with current and voltage limits
- Battery voltage sag model
- Backlash dead-zone model during drive-direction reversals
- Adjustable battery tray position (CoM shift) through URDF prismatic joint
- CSV telemetry logging and post-run step-response analysis

## Project Structure

```text
.
├── Context/
│   ├── Bobot_3_Main_Context.md
│   ├── Bobot3_MainContext.md
│   ├── General_Prompt.md
│   └── General_Prompt_Origin.md
├── data/
│   └── .gitkeep
├── src/
│   ├── analyze.py
│   ├── control_panel.py
│   ├── controller.py
│   ├── logger.py
│   ├── main.py
│   ├── physics.py
│   └── robot.py
├── urdf/
│   └── bobot3.urdf
├── requirements.txt
└── README.md
```

## Environment Setup (venv)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## Run Simulation

GUI mode (recommended for tuning sliders live):

```bash
python src/main.py --duration 20 --realtime
```

By default, GUI mode opens a separate Tk control panel with readable fonts and unclamped labels.

The panel also includes:

- buttons to reset the robot with positive or negative tilt, or upright
- automatic reset clearance correction to avoid launch-up from ground penetration
- short post-reset torque hold so the robot does not jump on re-engagement
- one-click `Apply Stable PID Defaults` action
- live derived motor readout (RPM/current/torque/voltage/ratio)
- persistent parameter autosave (`data/presets/__last_session__.json`)
- preset save/load controls for named parameter sets (`data/presets/*.json`)

If you want the old built-in PyBullet sliders instead:

```bash
python src/main.py --duration 20 --realtime --legacy-gui-controls
```

Headless mode:

```bash
python src/main.py --headless --duration 20 --log-file data/sim_log.csv
```

### Main Runtime Controls (GUI)

- `pid_kp`, `pid_ki`, `pid_kd`: PID regulator coefficients (P, I, D)
- `control_polarity`: torque sign selector (-1 or +1) to match wheel/joint orientation
- `target_pitch_deg`: setpoint for body pitch
- `max_torque_nm`: output clamp for PID command
- `backlash_deg`: dead-zone traversal requirement after reversal
- `backlash_impact_gain`: re-engagement kick gain when backlash closes
- `drive_mode`: 0 direct, 1 gear reducer, 2 belt, 3 gear+belt
- `reducer_ratio`, `belt_ratio`: transmission ratio tuning
- `battery_z_m`: move battery above/below wheel axis
- `battery_nominal_v`, `battery_internal_r`, `battery_max_current_a`: electrical pack constraints
- `motor_kt_nm_per_a`, `motor_ke_v_per_rad_s`, `motor_phase_r_ohm`: motor constants used to derive current, torque, and speed behavior
- `disturbance_torque_nm`: injected disturbance for step-response testing

### Reset Controls (Tk Panel)

- `reset_tilt_deg`: magnitude used by reset buttons
- `Reset Tilt Left`: resets pose with positive pitch tilt
- `Reset Tilt Right`: resets pose with negative pitch tilt
- `Reset Upright`: resets pose with zero pitch
- reset now applies automatic ground-clearance correction and a brief zero-torque hold to prevent launch-up

### Live Derived Readout (Tk Panel)

- drive mode and drive ratio
- battery voltage under sag
- left/right motor RPM
- left/right motor current
- left/right raw motor torque and torque after backlash

### Preset Controls (Tk Panel)

- Enter preset name and click `Save Current` to store all current parameters.
- Select any preset in list and click `Load Selected` to restore all parameters.
- Latest slider state is autosaved and reused automatically on next GUI launch.

## Control Timing

The simulator enforces:

- Physics frequency: 2000 Hz
- Control frequency: 500 Hz

By default, control updates every 4 physics steps using sample-and-hold behavior.

## Logged Telemetry

Every simulation cycle writes to CSV (example columns):

- `timestamp_s`
- `pitch_true_deg`, `pitch_est_deg`
- `gyro_pitch_rate_deg_s`, accelerometer channels
- `left/right_encoder_tick`, wheel angle and speed
- `left/right_motor_rpm`, motor current
- `battery_voltage_v`
- `motor_kt_nm_per_a`, `motor_ke_v_per_rad_s`, `motor_phase_r_ohm`
- `commanded_torque_nm`, torque after backlash
- `reflected_inertia_kg_m2`
- `drive_mode`, `drive_ratio`
- base pose channels

## Step-Response Analysis

Generate a plot and recovery-time estimate from telemetry:

```bash
python src/analyze.py --input data/sim_log.csv --output data/step_response.png
```

The analyzer uses a threshold and settling-band method:

- Disturbance detect threshold: default 5 deg
- Settled band: default +/-0.5 deg
- Settling window: default 0.25 s

## Notes

- If simulation becomes unstable, reduce PID gains and backlash dead-zone.
- Large fixed time steps can destabilize contact dynamics. Keep high physics frequency.
- Reflected inertia is computed as:

$$
J_{total} = J_{rotor} + \frac{J_{wheel} + J_{gear}}{N^2}
$$

- Motor and battery model uses:

$$
	au = K_t I, \quad V_{actual} = V_{nominal} - I R_{internal}
$$