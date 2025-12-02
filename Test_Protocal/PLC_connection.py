import os
import time
from pymodbus.client import ModbusTcpClient  # แก้ import

idslave_ip = 0x01

# สร้าง client
client = ModbusTcpClient(
    host="192.168.3.250",
    port=502,
    timeout=3,
    retries=3
)

# Connect
if client.connect():
    print("✓ Connected to PLC!")

    client.close()
    print("✓ Disconnected")
else:
    print("✗ Connection failed")