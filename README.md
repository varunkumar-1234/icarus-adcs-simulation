# ADCS Team 3-Day Sprint - CubeSat Simulation

This repository contains the closed-loop MATLAB/Simulink simulation for a ~500 km LEO CubeSat, featuring 6-DOF dynamics, sensor noise, a custom B-dot detumbling control law, and automatic handoff to pointing control.

## How to Run

1. **Open the Project:**
   Double-click `CubeSat Simulation Project.prj` or open it from the MATLAB Start Page.

2. **Initialize Workspace Parameters:**
   In the MATLAB Command Window, run the initialization scripts:
   ```matlab
   setup_day1_orbit
   sensor_noise_config
   ```

3. **Open the Simulink Model:**
   Open `asbCubeSat.slx`.

4. **Run Automated Scenarios & Plots:**
   To automatically run the high-tumble detumble scenarios and generate the required demo plots, run:
   ```matlab
   run_sprint_scenarios
   ```

## Included Custom Logic
- `bdot_detumble.m`: The custom control law ($M = k (\omega \times B)$).
- `adcs_mode_manager.m`: State machine that monitors angular velocity and hands off to pointing control when the tumble rate is $< 0.05$ rad/s.
