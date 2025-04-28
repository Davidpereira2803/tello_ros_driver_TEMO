import socket
import csv
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for packets: {UDP_PORT}")

# Open CSV file for appending
with open('data_phone.csv', mode='a+', newline='') as file:
    file.seek(0)
    writer = csv.writer(file)
    
    # Write header if file is empty
    if not file.read(1):
        writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'])

    while True:
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode().strip()
        print("Received:", decoded_data)
        
        try:
            values = [float(val) for val in decoded_data.split(',')]
            
            if len(values) == 9:
                arrival_timestamp = int(time.time())
                row = [arrival_timestamp] + values
                writer.writerow(row)
                file.flush()
            else:
                print(f"Warning: received unexpected number of fields ({len(values)})")

        except Exception as e:
            print(f"Failed to parse data: {decoded_data} - Error: {e}")
