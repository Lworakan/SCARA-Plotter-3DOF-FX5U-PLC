from pymodbus.client import ModbusTcpClient
import time

client = ModbusTcpClient(host="192.168.3.250", port=502)

if client.connect():
    print("✓ Connected!\n")
    
    trajectory = [
        [0,    5000,  300],
        [1000, 1000, 200],
        [2000, 8000,  250],
    ]
    
    # Send all points
    for i, point in enumerate(trajectory):
        base = 200 + (i * 10)
        client.write_register(address=base, value=point[0])
        client.write_register(address=base+1, value=point[1])
        client.write_register(address=base+2, value=point[2])
        print(f"Point {i}: D{base}={point}")
    
    # Set count
    client.write_register(address=10, value=len(trajectory))
    print(f"\n✓ Sent {len(trajectory)} points\n")
    
    # Verify
    for i in range(len(trajectory)):
        base = 200 + (i * 10)
        r = client.read_holding_registers(address=base, count=3)
        print(f"Verify Point {i}: D{base}={list(r.registers)}")
    
    # Trigger
    print("\nTriggering M6...")
    client.write_coil(address=6, value=True)
    time.sleep(0.1)
    client.write_coil(address=6, value=False)
    
    client.close()
    print("\n✓ Done!")