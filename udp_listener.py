import socket

UDP_IP = "192.168.4.2"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Listening for packets")

while True:
    data, addr = sock.recvfrom(1024)
    print("Received:", data.decode())