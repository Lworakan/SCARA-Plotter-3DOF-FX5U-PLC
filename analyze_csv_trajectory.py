import csv
import math

# Robot parameters from scara_grid_test.py
L1 = 160.0
L2 = 149.5
STEPS_PER_REV = 200 * 32  # 6400
STEP_TO_ANGLE = 360.0 / STEPS_PER_REV

def forward_kinematics(steps1, steps2):
    theta1 = math.radians(steps1 * STEP_TO_ANGLE)
    theta2 = math.radians(steps2 * STEP_TO_ANGLE)
    
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y

csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_day3.csv'

points = []
try:
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            # Read every 50th point to get a shape overview without too much data
            if i % 50 == 0:
                j1 = int(row['J1_Pos'])
                j2 = int(row['J2_Pos'])
                x, y = forward_kinematics(j1, j2)
                points.append((x, y))
                
    print(f"Analyzed {len(points)} points.")
    if not points:
        print("No points found.")
        exit()

    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)

    print(f"X Range: {min_x:.2f} to {max_x:.2f} (Width: {max_x - min_x:.2f})")
    print(f"Y Range: {min_y:.2f} to {max_y:.2f} (Height: {max_y - min_y:.2f})")
    
    print("\nFirst 10 points:")
    for i, (x, y) in enumerate(points[:10]):
        print(f"{i}: ({x:.2f}, {y:.2f})")

except Exception as e:
    print(f"Error: {e}")
