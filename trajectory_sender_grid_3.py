import rk_mcprotocol as mc
import csv
import time

HOST = '192.168.3.250'
PORT = 5007

# Connect to PLC
try:
    s = mc.open_socket(HOST, PORT)
    if s:
        print(f"Successfully connected to {HOST}:{PORT}")
    else:
        print(f"Failed to connect to {HOST}:{PORT}")
        exit(1)
except Exception as e:
    print(f"Connection error: {e}")
    exit(1)

# Read trajectory from CSV
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\imotor_commands_frab_pen.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\imotor_commands_nut_head_20cm_2_wait_eye_pen_flip.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_nuttt_333_fn.csv'
csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_command_grid_3_scale.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_frab10.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_day4.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\imotor_commands_grid_3.csv'
# csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_kid.csv'

print("Waiting for M3 to be enabled on PLC...")

# Wait for M3 to be enabled on PLC
while True:
    try:
        m3_status = mc.read_bit(s, headdevice='M3', length=1)
        if m3_status and m3_status[0] == 1:
            print("M3 is enabled! Starting trajectory execution...")
            break
        time.sleep(0.1)  # Check every 100ms
    except Exception as e:
        print(f"Error reading M3: {e}")
        time.sleep(0.5)

start_time = time.time()

with open(csv_path, 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    
    for i, row in enumerate(reader):
        # Extract timestamp from CSV
        target_time = float(row['Time'])
        
        # Wait until the scheduled time is reached
        # while True:
        #     elapsed_time = time.time() - start_time
        #     if elapsed_time >= target_time:
        #         break
        time.sleep(0.02)  # Removed sleep to prevent timing drift
        
        # Extract values from CSV row
        # Assuming mapping:
        # D100 = J1_Steps (position motor 1)
        # D102 = J1_Hz (velocity motor 1)
        # D104 = J2_Steps (position motor 2)
        # D106 = J2_Hz (velocity motor 2)
        
        # Using int(float(...)) to handle strings like "250.0"
        # Correct Mapping based on scara_grid_test.py:
        # D100 = J1 (Motor 1)
        # D104 = J2 (Motor 2)
        d100_value = int(row['J1_Pos'])
        d102_value = int(float(row['J1_Hz']))
        d104_value = int(row['J2_Pos'])
        d106_value = int(float(row['J2_Hz']))
        on_off = int(float(row['Pen'])) 
        
        try:
            # Write data first
            mc.write_sign_Dword(s, headdevice='D100', 
                                data_list=[d100_value, d102_value, d104_value, d106_value], 
                                signed_type=True)

            mc.write_bit(s, headdevice='M4', data_list=[on_off])
            
        except Exception as e:
            print(f"\nError at row {i}: {e}")
            import traceback
            traceback.print_exc()
            print("Stopping execution")
            break
        
        
        print(f"serving row {i+1} ,:, {on_off}", end='\r')
        if i % 10 == 0:
            actual_time = time.time() - start_time
            print(f"Row {i} at {actual_time:.3f}s (target: {target_time:.3f}s): D100={d100_value}, D102={d102_value}, D104={d104_value}, D106={d106_value}")


mc.write_bit(s, headdevice='M4', data_list=[0])
print("Trajectory execution complete!")
print("Note: M3 is controlled on PLC side - not disabled by this script")

# Close connection
# mc.close_socket(s)
