import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for data on UDP port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    decoded = data.decode('utf-8').strip()
    values = list(map(float, decoded.split(',')))
    accel = values[:3]
    gyro = values[3:6]
    mag = values[6:9]
    lin = values[9:12]
    #print(f"Accel {accel}")
    #print(f"Gyro {gyro}")
    #print(f"Mag {mag}")
    print(f"Lin {lin}")
    #print(f"Accel {accel}, Gyro {gyro}, Mag {mag}")
