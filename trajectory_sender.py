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
csv_path = r'C:\Users\worak\Documents\GitHub\Plotter-3DOF-PLC\motor_commands_go2.csv'

print("Starting trajectory execution...")

start_time = time.time()
m3_enabled = False  # Track M3 state

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
        time.sleep(0.02)  # Sleep 1ms to avoid busy waiting
        
        # Extract values from CSV row
        # Assuming mapping:
        # D100 = J1_Steps (position motor 1)
        # D102 = J1_Hz (velocity motor 1)
        # D104 = J2_Steps (position motor 2)
        # D106 = J2_Hz (velocity motor 2)
        
        d100_value = int(row['J2_Pos'])
        d102_value = int(float(row['J2_Hz']))
        d104_value = int(row['J1_Pos'])
        d106_value = int(float(row['J1_Hz']))
        
        try:
            # Check if there's movement and M3 is currently 0
            if d100_value != 0 or d104_value != 0:
                m3_status = mc.read_bit(s, headdevice='M3', length=1)
                if m3_status and m3_status[0] == 0:
                    mc.write_bit(s, headdevice='M3', data_list=[1])
                    if not m3_enabled:
                        m3_enabled = True
                        print(f"M3 trigger started at row {i}")
            
            mc.write_sign_Dword(s, headdevice='D100', 
                                data_list=[d100_value, d102_value, d104_value, d106_value], 
                                signed_type=True)
            
        except Exception as e:
            print(f"\nError at row {i}: {e}")
            import traceback
            traceback.print_exc()
            print("Stopping execution")
            break
        
        # Print progress every 100 rows
        if i % 100 == 0:
            actual_time = time.time() - start_time
            print(f"Row {i} at {actual_time:.3f}s (target: {target_time:.3f}s): D100={d100_value}, D102={d102_value}, D104={d104_value}, D106={d106_value}")

print("Trajectory execution complete!")

mc.write_bit(s, headdevice='M3', data_list=[0])
print("M3 trigger disabled")

# Close connection
# mc.close_socket(s)
