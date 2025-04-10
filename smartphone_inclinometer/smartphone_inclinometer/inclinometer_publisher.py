#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
import socket
import threading
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5555

TAKEOFF_THRESHOLD = 0.9
LAND_THRESHOLD = -0.9
TAKEOFFLAND_HOLD_TIME = 1

class SmartphonePublisher(Node):
    def __init__(self):
        super().__init__('smartphone_publisher')

        self.inclinometer_pub = self.create_publisher(Float32MultiArray, '/smartphone/inclinometer', 10)

        self.sock = None
        self.get_logger().info("Starting Smartphone UDP listener...")

        while self.sock is None and rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.bind((UDP_IP, UDP_PORT))

                self.get_logger().info("Smartphone UDP listener started. Waiting for data...")
            except Exception as e:
                self.get_logger().error(f"Error starting Smartphone UDP listener: {e}")
                self.sock = None
                time.sleep(3)

        if self.sock:
            self.receive_data()

        
    def receive_data(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                decoded_data = data.decode().strip()
                self.get_logger().info(f"Received: {decoded_data}")

                parts = decoded_data.split(",")
                if len(parts) != 12:
                    self.get_logger().error("Invalid data format. Expected 12 values.")
                    continue

                values = list(map(float, parts))
                accel = values[:3]
                gyro = values[3:6]
                mag = values[6:9]
                bar = values[9:12]
                lin = values[12:15]
                orient = values[15:18]

                ax = accel[0]
                ay = accel[1]
                az = accel[2]
                gx = gyro[0]
                gy = gyro[1]
                gz = gyro[2]
                mx = mag[0]
                my = mag[1]
                mz = mag[2]
                lx = lin[0]
                ly = lin[1]
                lz = lin[2]

                msg = Float32MultiArray()
                msg.data = [ax, ay, az, gx, gy, gz, mx, my, mz, lx, ly, lz]
                self.inclinometer_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
            
        
