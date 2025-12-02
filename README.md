# 0. Python Environment Setup

Before running any scripts, it is recommended to create a Python virtual environment and install the required packages.

Once you have a `requirements.txt` file (to be provided), run:

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

This ensures all dependencies are installed in an isolated environment.

# SCARA Plotter 3DOF FX5U PLC Project

This project provides code and resources for controlling a SCARA robot plotter using a Mitsubishi FX5U PLC. Below are descriptions and usage instructions for the three main files:

---

## 1. `trajectory_sender.py`

**Purpose:**
- Sends a sequence of motor commands (trajectory) to the PLC for drawing images or patterns.
- Reads motor position and velocity commands from a CSV file and transmits them to the PLC in real time.

**How it works:**
- Connects to the PLC using the `rk_mcprotocol` library.
- Loads a CSV file containing the trajectory (motor positions, velocities, and pen state).
- Sends each row as a command to the PLC, triggering the robot to move and draw.
- Controls pen up/down via the PLC.

**Usage:**
1. Edit the `csv_path` variable in the script to point to your desired motor command CSV file.
2. Run the script:
   ```bash
   python3 trajectory_sender.py
   ```
3. The script will connect to the PLC and execute the drawing trajectory.

---

## 2. `scara_grid_test.py`

**Purpose:**
- Provides a user interface for testing and validating the SCARA robot's precision and control.
- Allows users to run a grid test, test a single point, or manually control the motors.

**How to Use:**
1. Run the script:
   ```bash
   python3 scara_grid_test.py
   ```
2. You will be prompted to choose a test mode:
   - **1. Full 3x3 Grid Test**: Runs the external `trajectory_sender_grid_3.py` script to execute a predefined grid test.
   - **2. Single Point Test**: Enter X and Y coordinates to test the robot's movement to a specific point.
   - **3. Manual Motor Control**: Directly input motor positions and velocities for manual testing.

**Note:**
- For choice 1, the script will launch `trajectory_sender_grid_3.py` in a new process.
- Ensure the PLC is connected and powered on before running tests.

---

## 3. `PLC_Ladder_code/kine_3.gx3`

**Purpose:**
- Mitsubishi GX Works3 ladder logic program for the FX5U PLC.
- Contains the control logic required for the SCARA robot.

**How to Use:**
1. Open the file `kine_3.gx3` using the [GX Works3](https://www.mitsubishielectric.com/fa/products/software/eng/gxworks3/) software (Windows only).
2. Download the ladder logic to your FX5U PLC.
3. Ensure the PLC is in RUN mode and properly connected to your robot and PC.

---

## Requirements
- Python 3.x
- `rk_mcprotocol` Python library (for PLC communication)
- Mitsubishi FX5U PLC
- GX Works3 software (for ladder logic)

---

## Project Structure
- `trajectory_sender.py` — Sends drawing trajectories to PLC
- `scara_grid_test.py` — User interface for grid and manual tests
- `PLC_Ladder_code/kine_3.gx3` — Ladder logic for PLC (open with GX Works3)
- `Motor_command/` — Example CSV files for motor commands

---

For more details, refer to comments in each script or contact the project maintainer.
