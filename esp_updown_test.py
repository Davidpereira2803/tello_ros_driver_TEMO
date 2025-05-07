import socket
import csv
import time
from collections import deque

UDP_IP = "192.168.4.2"
UDP_PORT = 8888

UP_THRESHOLD = 0.3
DOWN_THRESHOLD = -0.3
WINDOW_SIZE = 5
COOLDOWN_TIME = 1.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for packets on port {UDP_PORT}")

az_window = deque(maxlen=WINDOW_SIZE)
last_detection_time = 0
last_direction = None

with open('data_esp.csv', mode='a+', newline='') as file:
    writer = csv.writer(file)

    file.seek(0)
    if not file.read(1):
        writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

    while True:
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode()
        print("Received:", decoded_data)

        try:
            fields = decoded_data.strip().split(',')
            values = {}
            for field in fields:
                key, value = field.split(':')
                values[key.lower()] = float(value)

            timestamp = int(time.time())
            ax = values.get('ax', 0.0)
            ay = values.get('ay', 0.0)
            az = values.get('az', 0.0)
            gx = values.get('gx', 0.0)
            gy = values.get('gy', 0.0)
            gz = values.get('gz', 0.0)

            row = [timestamp, ax, ay, az, gx, gy, gz]
            writer.writerow(row)
            file.flush()

            az_window.append(az)
            current_time = time.time()

            if len(az_window) == WINDOW_SIZE:
                avg_az = sum(az_window) / len(az_window)
                deviation = avg_az - 1

                if current_time - last_detection_time > COOLDOWN_TIME:
                    if deviation > UP_THRESHOLD:
                        print("Detected UPWARD movement!")
                        last_detection_time = current_time
                        last_direction = "up"
                    elif deviation < DOWN_THRESHOLD:
                        print("Detected DOWNWARD movement!")
                        last_detection_time = current_time
                        last_direction = "down"

        except Exception as e:
            print(f"⚠️ Failed to parse data: {decoded_data} - Error: {e}")
