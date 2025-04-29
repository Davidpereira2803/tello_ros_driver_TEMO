import socket
import csv
import time

UDP_IP = "192.168.4.2"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for packets on port {UDP_PORT}")

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
            row = [
                timestamp,
                values.get('ax', 0.0),
                values.get('ay', 0.0),
                values.get('az', 0.0),
                values.get('gx', 0.0),
                values.get('gy', 0.0),
                values.get('gz', 0.0),
            ]
            writer.writerow(row)
            file.flush()

        except Exception as e:
            print(f"Failed to parse data: {decoded_data} - Error: {e}")
