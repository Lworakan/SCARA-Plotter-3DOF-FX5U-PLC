import csv
import math
import numpy as np

# Robot parameters
L1 = 160.0
L2 = 149.5
STEPS_PER_REV = 6400
STEP_TO_ANGLE = 360.0 / STEPS_PER_REV
DT = 0.02  # 20ms

def inverse_kinematics(x, y):
    # Law of cosines for theta2
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_theta2) > 1:
        return 0, 0
    theta2 = math.atan2(math.sqrt(1 - cos_theta2**2), cos_theta2)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

def generate_trajectory():
    # Grid points (3x3)
    center_x, center_y = 200, 0
    spacing = 60
    points = []
    for i in range(3):
        for j in range(3):
            x = center_x + (j - 1) * spacing
            y = center_y + (i - 1) * spacing
            points.append((x, y))

    def forward_kinematics(steps1, steps2):
        theta1 = math.radians(steps1 * STEP_TO_ANGLE)
        theta2 = math.radians(steps2 * STEP_TO_ANGLE)
        x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        return x, y

    # Generate path: Home -> P1 -> P2 ... -> Home
    # We'll just do point-to-point with interpolation
    
    current_x, current_y = forward_kinematics(0, 0) # Start at home (0,0 steps)
    
    csv_data = []
    time_counter = 0.0
    
    # Helper to add moves
    def add_move(target_x, target_y, duration=2.0):
        nonlocal current_x, current_y, time_counter
        steps = int(duration / DT)
        start_x, start_y = current_x, current_y
        
        for s in range(steps):
            t = s / steps
            # Linear interpolation
            x = start_x + (target_x - start_x) * t
            y = start_y + (target_y - start_y) * t
            
            theta1, theta2 = inverse_kinematics(x, y)
            j1_steps = int(theta1 / STEP_TO_ANGLE)
            j2_steps = int(theta2 / STEP_TO_ANGLE)
            
            # Calculate velocity (simple finite difference)
            if len(csv_data) > 0:
                prev_j1 = int(csv_data[-1]['J1_Pos'])
                prev_j2 = int(csv_data[-1]['J2_Pos'])
                j1_hz = abs(j1_steps - prev_j1) / DT
                j2_hz = abs(j2_steps - prev_j2) / DT
            else:
                j1_hz, j2_hz = 0, 0
                
            csv_data.append({
                'Time': round(time_counter, 3),
                'J1_Pos': j1_steps,
                'J1_Hz': round(j1_hz, 1),
                'J2_Pos': j2_steps,
                'J2_Hz': round(j2_hz, 1)
            })
            time_counter += DT
            
        current_x, current_y = target_x, target_y

    # Generate moves
    for p in points:
        add_move(p[0], p[1], duration=1.0)
        # Add a small pause at each point
        for _ in range(10): # 0.2s pause
            csv_data.append({
                'Time': round(time_counter, 3),
                'J1_Pos': csv_data[-1]['J1_Pos'],
                'J1_Hz': 0,
                'J2_Pos': csv_data[-1]['J2_Pos'],
                'J2_Hz': 0
            })
            time_counter += DT

    # Write to CSV
    with open('motor_commands_grid.csv', 'w', newline='') as f:
        fieldnames = ['Time', 'J1_Pos', 'J1_Hz', 'J2_Pos', 'J2_Hz']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(csv_data)
        
    print(f"Generated motor_commands_grid.csv with {len(csv_data)} rows.")

if __name__ == "__main__":
    generate_trajectory()
