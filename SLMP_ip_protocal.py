from turtle import position
import rk_mcprotocol as mc
import time

HOST = '192.168.3.250'
PORT = 5007

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

d100_value = int(input("Enter value for D100: "))
d102_value = int(input("Enter value for D102: "))
d104_value = int(input("Enter value for D104: "))
d106_value = int(input("Enter value for D106: "))



# while True:
st = time.time()

# print(mc.read_bit(s,headdevice = 'm0' , length = 3584 ))   
# print(mc.read_sign_word(s,headdevice = 'd0' , length = 960, signed_type=False))

position_m1 = mc.read_sign_Dword(s, headdevice='D100', length=1, signed_type=True)
velocity_m1 = mc.read_sign_Dword(s, headdevice='D102', length=1, signed_type=True)
position_m2 = mc.read_sign_Dword(s, headdevice='D104', length=1, signed_type=True)
velocity_m2 = mc.read_sign_Dword(s, headdevice='D106', length=1, signed_type=True)


# print(mc.write_sign_Dword(s, headdevice='D100', data_list =[500], signed_type=True))

# mc.write_sign_Dword(s, headdevice='D100', data_list=[d100_value], signed_type=True)
# mc.write_sign_Dword(s, headdevice='D102', data_list=[d102_value], signed_type=True)
# mc.write_sign_Dword(s, headdevice='D104', data_list=[d104_value], signed_type=True)
# mc.write_sign_Dword(s, headdevice='D106', data_list=[d106_value], signed_type=True)
# print(f"Written values - D100: {d100_value}, D102: {d102_value}, D104: {d104_value}, D106: {d106_value}")

# # print(mc.write_bit(s,headdevice = 'm0' , data_list = [1]*3584 )) 
# # print(mc.write_sign_word(s,headdevice = 'd0' , data_list = [-999]*960 ,signed_type =True))
# # print(mc.write_sign_Dword(s,headdevice = 'r0' , data_list = [9999999]*480 ,signed_type =True))
# mc.write_bit(s,headdevice='M2', data_list=[1])  # Set TRUE


mc.write_sign_Dword(s, headdevice='D100', 
                    data_list=[d100_value, d102_value, d104_value, d106_value], 
                    signed_type=True)
print(f"All values written in one batch!")

# Then trigger
mc.write_bit(s, headdevice='M3', data_list=[1])
et = time.time()
elapsed = et -st
time.sleep(1)  

print (f' elapsed time = {elapsed}')
print(f'Position: {position_m1[0]}, Velocity: {velocity_m1[0]}, Position2: {position_m2[0]}, Velocity2: {velocity_m2[0]}')
