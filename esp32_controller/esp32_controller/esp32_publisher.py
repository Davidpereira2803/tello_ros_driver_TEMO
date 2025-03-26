#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import math

ESP32_IP = "192.168.4.2"
ESP32_PORT = 8888

class ESP32Publisher(Node):
    def __init__(self):
        super().__init__('esp32_publisher')
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/esp32/inclinometer', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ESP32_IP, ESP32_PORT))

        self.get_logger().info("ESP32 UDP listener started. Waiting for data...")

        self.receive_data()

    def receive_data(self):
        """Continuously receive UDP packets from ESP32 and publish to ROS2."""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                decoded_data = data.decode().strip()
                self.get_logger().info(f"Received: {decoded_data}")

                parts = decoded_data.split(",")
                ax = float(parts[0].split(":")[1])
                ay = float(parts[1].split(":")[1])
                az = float(parts[2].split(":")[1])
                gx = float(parts[3].split(":")[1])
                gy = float(parts[4].split(":")[1])
                gz = float(parts[5].split(":")[1])


                roll = self.compute_roll(ax, ay, az)
                pitch = self.compute_pitch(ax, ay, az)

                msg = Float32MultiArray()
                msg.data = [roll, pitch]

                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error receiving ESP32 data: {e}")

    def compute_roll(self, ax, ay, az):
        """Compute roll angle from accelerometer data."""
        return math.degrees(math.atan2(-ay, -az))
    
    def compute_pitch(self, ax, ay, az):
        """
        Compute pitch angle from accelerometer data.
        ax, ay, az: Accelerometer readings (in g)
        Returns pitch angle in degrees.
        """
        pitch_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        pitch_deg = math.degrees(pitch_rad)
        return pitch_deg
    