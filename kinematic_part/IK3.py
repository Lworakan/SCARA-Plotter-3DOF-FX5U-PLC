import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import math
import csv
import os

# Try importing SVG library
try:
    from svgpathtools import svg2paths
except ImportError:
    print("Warning: svgpathtools not found. SVG mode will not work.")

# ==========================================
# 1. SETTINGS & CONSTANTS
# ==========================================
# Robot Arm Lengths
L1 = 155
L2 = 160

# Simulation Time Step
DT = 0.02  # 20ms update rate

# Motor Settings
STEPS_PER_REV = 200
MICROSTEPS = 32
GEAR_RATIO_J1 = 2
GEAR_RATIO_J2 = 2.5

# Calculate Resolution
STEPS_PER_RAD_J1 = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_J1) / (2 * math.pi)
STEPS_PER_RAD_J2 = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_J2) / (2 * math.pi)

# Drawing Settings
# Drawing Settings
TRAVEL_SPEED = 50 
DRAW_SPEED = 30    
PEN_DOWN_DELAY = 2 
OFFSET_X = -180       
OFFSET_Y = 90     

# --- SEPARATE SCALING ---
# Keep Y at 1.1 since it is correct.
# Lower X slightly. Try calculating the exact value, or start guessing around 1.09
SCALE_X = 1  # <--- ADJUST THIS based on the formula above
SCALE_Y = 1.1   # <--- Keep this as is

# --- SELECTION MENU ---
# Options: "waypoints", "svg", "circle"
DRAW_MODE = "svg" 

# Shape Parameters
CIRCLE_RADIUS = 40

# ==========================================
# 2. ROBOT KINEMATICS CLASS
# ==========================================
class ScaraRobot:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2

    def inverse_kinematics(self, x, y):
        denom = 2 * self.l1 * self.l2
        if denom == 0: return 0, 0
        
        numerator = x**2 + y**2 - self.l1**2 - self.l2**2
        c2 = numerator / denom
        
        if c2 > 1.0: c2 = 1.0
        if c2 < -1.0: c2 = -1.0

        s2 = math.sqrt(1 - c2**2)
        theta2 = math.atan2(s2, c2)

        den_k = x**2 + y**2
        if den_k == 0: return 0, 0
        term = self.l1 + self.l2 * c2
        s1 = (-x * self.l2 * s2 + y * term) / den_k
        c1 = (y * self.l2 * s2 + x * term) / den_k
        theta1 = math.atan2(s1, c1)

        return theta1, theta2

    def forward_kinematics(self, t1, t2):
        xe = self.l1 * math.cos(t1)
        ye = self.l1 * math.sin(t1)
        xf = xe + self.l2 * math.cos(t1 + t2)
        yf = ye + self.l2 * math.sin(t1 + t2)
        return xe, ye, xf, yf

# ==========================================
# 3. TRAJECTORY GENERATORS
# ==========================================
def get_quintic_scalar(t, T):
    if t >= T: return 1.0
    if t <= 0: return 0.0
    tau = t / T
    return 10*(tau**3) - 15*(tau**4) + 6*(tau**5)

def interpolate_travel(start_p, end_p, speed, dt, pen_state):
    dist = math.dist(start_p, end_p)
    if dist < 0.001: return []

    duration = dist / speed 
    if duration < dt: duration = dt
    num_samples = int(duration / dt)
    
    # Force minimum steps for physical smoothing
    if num_samples < 5: num_samples = 5
    duration = num_samples * dt # Recalculate true duration

    trajectory = []
    for i in range(num_samples):
        curr_time = i * dt
        s = get_quintic_scalar(curr_time, duration)
        nx = start_p[0] + (end_p[0] - start_p[0]) * s
        ny = start_p[1] + (end_p[1] - start_p[1]) * s
        trajectory.append((nx, ny, pen_state))
    return trajectory

def interpolate_linear(start_p, end_p, speed, dt, pen_state):
    dist = math.dist(start_p, end_p)
    if dist < 0.001: return [] 
    duration = dist / speed
    if duration < dt: 
        return [(end_p[0], end_p[1], pen_state)]
        
    num_samples = int(duration / dt)
    trajectory = []
    for i in range(1, num_samples + 1): 
        t = i / num_samples
        nx = start_p[0] + (end_p[0] - start_p[0]) * t
        ny = start_p[1] + (end_p[1] - start_p[1]) * t
        trajectory.append((nx, ny, pen_state))
    return trajectory

# ==========================================
# 4. HELPER: SVG TO WAYPOINTS
# ==========================================
def parse_svg_to_waypoints(filename):
    """
    Reads an SVG and returns a clean list of dicts: {'x':, 'y':, 'pen':}
    Handles scaling, centering, and enforcing jumps between paths.
    """
    raw_waypoints = []
    try:
        paths, attributes = svg2paths(filename)
        print(f"Loaded {len(paths)} SVG paths.")
        
        if not paths: return []
        
        # Manual Offset & Scale
        final_offset_x = OFFSET_X
        final_offset_y = OFFSET_Y
        
        # Log the separate scales
        print(f"SVG Parsed. Scale X: {SCALE_X:.3f}, Scale Y: {SCALE_Y:.3f}")
        print(f"Offset: ({final_offset_x:.1f}, {final_offset_y:.1f})")

        # 2. Convert Paths to Waypoints
        for path in paths:
            # Track previous end point to detect internal gaps
            prev_end_x = None
            prev_end_y = None
            
            for segment in path:
                p_start = segment.point(0)
                
                # APPLY SEPARATE SCALES HERE
                sx = (p_start.real * SCALE_X) + final_offset_x
                sy = (p_start.imag * SCALE_Y) + final_offset_y
                
                # Check for discontinuity
                is_gap = True
                if prev_end_x is not None:
                    dist = math.hypot(sx - prev_end_x, sy - prev_end_y)
                    if dist < 0.1:
                        is_gap = False
                
                if is_gap:
                    # Force Jump (Pen Up)
                    raw_waypoints.append({'x': sx, 'y': sy, 'pen': 0})
                
                # Discretize Segment
                # We use the larger scale for resolution calculation to ensure smoothness
                max_scale = max(SCALE_X, SCALE_Y)
                seg_len = segment.length() * max_scale
                if seg_len == 0: continue
                
                num_points = max(int(seg_len / 0.5), 2)
                
                for i in range(1, num_points + 1):
                    t = i / num_points
                    p = segment.point(t)
                    
                    # APPLY SEPARATE SCALES HERE
                    wx = (p.real * SCALE_X) + final_offset_x
                    wy = (p.imag * SCALE_Y) + final_offset_y
                    
                    # Draw (Pen Down)
                    raw_waypoints.append({'x': wx, 'y': wy, 'pen': 1})
                
                p_end = segment.point(1)
                prev_end_x = (p_end.real * SCALE_X) + final_offset_x
                prev_end_y = (p_end.imag * SCALE_Y) + final_offset_y
                
    except Exception as e:
        print(f"SVG Parsing Error: {e}")
        return []
        
    return raw_waypoints

# ==========================================
# 5. MAIN LOGIC
# ==========================================
robot = ScaraRobot(L1, L2)
full_trajectory = []

HOME_X = 0
HOME_Y = L1 + L2
current_x, current_y = HOME_X, HOME_Y 
full_trajectory.append((current_x, current_y, 0))

# --- UNIFIED WAYPOINT LIST ---
target_list = []

print(f"Mode: {DRAW_MODE}")

if DRAW_MODE == "waypoints":
    filename = "waypoints.txt"
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 3 and parts[0] != "None":
                    target_list.append({
                        'x': float(parts[0]) + OFFSET_X,
                        'y': float(parts[1]) + OFFSET_Y, 
                        'pen': int(parts[2])
                    })

elif DRAW_MODE == "svg":
    filename = "grid_clean.svg" # Ensure this matches your flipped file
    target_list = parse_svg_to_waypoints(filename)

elif DRAW_MODE == "circle":
    cx, cy, r = OFFSET_X, OFFSET_Y, CIRCLE_RADIUS
    target_list.append({'x': cx+r, 'y': cy, 'pen': 0})
    circumference = 2 * math.pi * r
    steps = int(circumference / 1.0)
    for i in range(1, steps + 1):
        angle = (i / steps) * 2 * math.pi
        nx, ny = cx + r * math.cos(angle), cy + r * math.sin(angle)
        target_list.append({'x': nx, 'y': ny, 'pen': 1})

# 2. PROCESS TARGET LIST (TRAJECTORY GENERATION)
print(f"Processing {len(target_list)} targets...")

for i, target in enumerate(target_list):
    target_x = target['x']
    target_y = target['y']
    target_pen = target['pen']
    
    start_pos = (current_x, current_y)
    target_pos = (target_x, target_y)
    
    if math.dist(start_pos, target_pos) < 0.001 and target_pen == 1:
        continue

    if target_pen == 0:
        # --- TRAVEL (PEN UP) ---
        # Add explicit Dwell before moving (Wait for pen to lift fully)
        # This prevents "drag" lines while the pen is rising
        wait_steps = int(0.2 / DT) # 0.2s pause to ensure lift
        for _ in range(wait_steps):
            full_trajectory.append((start_pos[0], start_pos[1], 0))

        moves = interpolate_travel(start_pos, target_pos, TRAVEL_SPEED, DT, 0)
        
        if not moves:
            moves.append((target_x, target_y, 0))
            
    else:
        # --- DRAW (PEN DOWN) ---
        last_state = full_trajectory[-1][2] if full_trajectory else 0
        if last_state == 0:
            # We just landed. Wait for pen to lower fully.
            wait_steps = int(PEN_DOWN_DELAY / DT)
            for _ in range(wait_steps):
                full_trajectory.append((start_pos[0], start_pos[1], 1))
        
        moves = interpolate_linear(start_pos, target_pos, DRAW_SPEED, DT, 1)

    full_trajectory.extend(moves)
    current_x, current_y = target_x, target_y

print(f"Trajectory Generation Complete. Total Points: {len(full_trajectory)}")

# ==========================================
# 6. MOTOR COMMAND GENERATION (OUTPUT)
# ==========================================
commands = []
t1_home, t2_home = robot.inverse_kinematics(HOME_X, HOME_Y)
OFFSET_STEPS_1 = int(t1_home * STEPS_PER_RAD_J1)
OFFSET_STEPS_2 = int(t2_home * STEPS_PER_RAD_J2)

prev_steps_1 = 0
prev_steps_2 = 0

for i, (x, y, pen) in enumerate(full_trajectory):
    t1, t2 = robot.inverse_kinematics(x, y)
    
    if t1 is None:
        curr_steps_1, curr_steps_2 = prev_steps_1, prev_steps_2
    else:
        raw_steps_1 = int(t1 * STEPS_PER_RAD_J1)
        raw_steps_2 = int(t2 * STEPS_PER_RAD_J2)
        
        curr_steps_1 = (raw_steps_1 - OFFSET_STEPS_1) * -1
        curr_steps_2 = (raw_steps_2 - OFFSET_STEPS_2) * -1
    
    delta_1 = curr_steps_1 - prev_steps_1
    delta_2 = curr_steps_2 - prev_steps_2
    
    dir_1 = 1 if delta_1 >= 0 else 0
    dir_2 = 1 if delta_2 >= 0 else 0
    
    hz_1 = abs(delta_1) / DT
    hz_2 = abs(delta_2) / DT
    
    commands.append([
        round(i * DT, 3),
        curr_steps_1, dir_1, abs(delta_1), round(hz_1, 1),
        curr_steps_2, dir_2, abs(delta_2), round(hz_2, 1),
        pen
    ])
    
    prev_steps_1, prev_steps_2 = curr_steps_1, curr_steps_2

# --- SAVE TO CSV ---
csv_filename = "motor_command_grid.csv"
try:
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ["Time", "J1_Pos", "J1_Dir", "J1_Delta", "J1_Hz", "J2_Pos", "J2_Dir", "J2_Delta", "J2_Hz", "Pen"]
        writer.writerow(header)
        writer.writerows(commands)
    print(f"SUCCESS: Motor commands saved to '{csv_filename}'")
except Exception as e:
    print(f"Error saving CSV: {e}")

# ==========================================
# 7. VISUALIZATION
# ==========================================
if len(full_trajectory) > 300000:
    print("Trajectory too long for full animation. Skipping animation.")
else:
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-200, 200)
    ax.set_ylim(0, 300)
    ax.set_zlim(-10, 10)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f'SCARA Simulation\nMode: {DRAW_MODE}')

    line_arm, = ax.plot([], [], [], 'o-', lw=3, color='blue')
    line_path, = ax.plot([], [], [], '-', lw=1, color='red')

    drawn_x, drawn_y, drawn_z = [], [], []

    def update(frame):
        if frame >= len(full_trajectory): return line_arm, line_path
        
        x, y, pen = full_trajectory[frame]
        
        t1, t2 = robot.inverse_kinematics(x, y)
        if t1 is None: return line_arm, line_path

        xe, ye, xf, yf = robot.forward_kinematics(t1, t2)
        
        line_arm.set_data([0, xe, xf], [0, ye, yf])
        line_arm.set_3d_properties([0, 0, 0])
        
        if pen == 1:
            drawn_x.append(xf)
            drawn_y.append(yf)
            drawn_z.append(0)
        else:
            drawn_x.append(np.nan)
            drawn_y.append(np.nan)
            drawn_z.append(np.nan)

        line_path.set_data(drawn_x, drawn_y)
        line_path.set_3d_properties(drawn_z)

        return line_arm, line_path

    ani = animation.FuncAnimation(fig, update, frames=len(full_trajectory), interval=0.00001, blit=False)
    plt.show()