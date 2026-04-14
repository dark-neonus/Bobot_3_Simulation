# Project Specification: Bobot-3 Dynamics & Control Simulator

## 1. Project Overview & Motivation
The Bobot-3 is a 3kg balancing robot platform. We are encountering a classic engineering trade-off: **Stability vs. Mechanical Complexity**. We need a simulation environment that is not just a visual representation, but a high-fidelity physics environment that can expose failures caused by **gearbox backlash, reflected inertia, and control loop latency**. 

This simulator will serve as our "digital twin." By simulating the physical interaction between the ESP32 control logic and the mechanical drivetrain, we can iterate on gear ratios and motor selection *before* spending our 40 USD per-wheel budget on hardware that might cause the robot to oscillate and fall.

## 2. Technical Stack & Environment
* **Core Physics:** `PyBullet` (Industry standard for robotics research).
* **Numerical Engine:** `NumPy` and `SciPy` (for differential equations).
* **Data Logging:** `Pandas` (recording state snapshots).
* **Control/Visualization:** `PyBullet` built-in GUI (using `addUserDebugParameter` for real-time slider manipulation).
* **Virtual Environment:** A Python `venv` is strictly required to isolate these dependencies.

## 3. High-Frequency Simulation Logic (Control Loop Timing)
This is the most critical constraint. You must emulate the **asynchronous nature of a real microcontroller.** * **Simulation Frequency:** The physics engine (PyBullet) should run at a high fixed time step (e.g., $1/2000$s) to ensure solver stability.
* **Control Frequency:** Your Python PID class must simulate an ESP32 loop running at exactly $500$Hz. This means for every $4$ physics steps ($2000 / 500 = 4$), the PID controller should calculate new torque, and the motors should update their output. 
* **Implementation:** Use a counter or a modulo operator inside the main loop:
    ```python
    if step_count % (physics_freq / control_freq) == 0:
        # Run PID Logic
    ```
    This decoupling is mandatory to correctly simulate the "sample-and-hold" behavior of an ESP32-S3.

## 4. Modeling Physical Components
* **Robot Body:** A URDF-defined model consisting of:
    * **Main Chassis:** The center of mass (CoM) must be dynamically adjustable via a GUI slider.
    * **Battery Tray:** Modeled as a separate link. We need to be able to move it from "Above Wheels" to "Below Wheel Axis" to observe the impact on the pendulum stability.
    * **Wheel/Motor System:** A link that can switch between Direct Drive, Belt, or Planetary Reducer configurations.
* **Backlash Implementation (Non-Linearity):**
    * Since physics engines don't support backlash natively, implement a "dead-zone" in the torque transmission logic. 
    * When the motor reverses direction, there must be a period of `N` degrees (configurable in the GUI) where the motor's motion does not translate to wheel motion. This must be a "soft-link" constraint or a threshold-based torque gate.

## 5. Electrical & Motor Dynamics
The motor model cannot be "ideal." It must include:
* **Torque derivation:** Calculate $\tau = K_t \cdot I$. 
* **Current Draw:** Calculate $I = \tau / K_t$. Factor in battery voltage sag: $V_{actual} = V_{nominal} - (I \cdot R_{internal})$.
* **Voltage Limit:** If the calculated required voltage exceeds 21V, cap it.
* **Reflected Inertia:** The system must compute $J_{total} = J_{rotor} + (\frac{J_{wheel} + J_{gear}}{N^2})$. This is the primary parameter we need to monitor to avoid "sluggish" balancing.

## 6. Required Project Structure
Create Context, src, urdf, and other directories in curreent Bobot_3_Simulation directory, dont create additiona lsubdirectory like bobot_sim.
```text
.
├── Context/
│   └── Bobot3_MainContext.md  # Keep the original requirement spec here
├── src/
│   ├── main.py                # Main simulation loop
│   ├── physics.py             # Backlash & Inertia calculations
│   ├── controller.py          # ESP32 PID emulation (500Hz loop)
│   ├── robot.py               # PyBullet URDF management
│   └── logger.py              # Data recording (CSV format)
├── urdf/                      # Robot geometry
├── data/                      # Output logs for debugging
└── requirements.txt           # List of dependencies
```

## 7. Step-by-Step Implementation Plan
1.  **Phase 1 (Environment):** Establish the simulation scaffold. Create the `PyBullet` window and a fixed-step loop.
2.  **Phase 2 (Control Loop Timing):** Implement the logic to force the PID controller to run at exactly $500$Hz, independent of the variable physics simulation frequency.
3.  **Phase 3 (Physical Modeling):** Load the URDF. Implement the "reconfigurable" link system so you can move the battery tray and see the CoM change in real-time.
4.  **Phase 4 (Nonlinearities):** Write the backlash function. Test it by intentionally setting the backlash threshold high and observing the robot oscillate.
5.  **Phase 5 (Telemetry):** Ensure every simulation cycle records `[timestamp, battery_voltage, motor_rpm, actual_tilt, torque_output]` to a CSV.
6.  **Phase 6 (Analysis):** Build an analysis script that takes the CSV and plots the "Step Response"—the time it takes for the robot to recover from a 5-degree tilt.

## 8. Important Technical Notes for the Agent
* **Simulation Stability:** If the simulation "explodes" (robots flying into the air), it is usually because the `fixedTimeStep` is too large or the backlash constraint forces are too high. Start with conservative stiffness settings for your backlash "springs."
* **Inertia Calculation:** Please ensure that the `robot_model.py` uses the standard formula for a cylinder to estimate the mass distribution of the wheels. If you neglect this, the simulation will be physically inaccurate.
* **Motivation:** We are building this because we do not trust "black box" AI calculations. We need the data to prove *our own* design choices. Every parameter must be inspectable and tunable.