import socket
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for data on UDP port {UDP_PORT}...")

UP_THRESHOLD = 1.5
DOWN_THRESHOLD = -1.5
COOLDOWN_TIME = 1 

last_command_time = 0

def process_linear_acc(az, current_time):
    global last_command_time


    if current_time - last_command_time < COOLDOWN_TIME:
        return 0.0

    if az > UP_THRESHOLD:
        last_command_time = current_time
        print("Moving up")
        return 1.0 
    elif az < DOWN_THRESHOLD:
        last_command_time = current_time
        print("Moving down")
        return -1.0 
    else:
        return 0.0  


while True:
    data, addr = sock.recvfrom(1024)
    decoded = data.decode('utf-8').strip()
    values = list(map(float, decoded.split(',')))
    accel = values[:3]
    grav = values[3:6]
    lin = values[6:9] # first value for x, second for y, third for z
    orient = values[9:12] # first value for yaw, second for pitch, third for roll

    #a_z = ax * gx + ay * gy + az * gz
    a_z = accel[0] * grav[0] + accel[1] * grav[1] + accel[2] * grav[2]



    #print(f"Accel {accel}")
    process_linear_acc(az=lin[2], current_time=time.time())
    #print(f"Gyro {gyro}")
    #print(f"Mag {mag}")
    #print(f"grav {grav[1]}")
    #print(f"Accel {accel}, Gyro {gyro}, Mag {mag}")



