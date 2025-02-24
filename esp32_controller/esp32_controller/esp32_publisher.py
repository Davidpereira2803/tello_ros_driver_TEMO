#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket

# UDP Settings
ESP32_IP = "0.0.0.0"
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

                parts = decoded_data.split(", ")
                roll = float(parts[0].split(": ")[1])
                pitch = float(parts[1].split(": ")[1])

                msg = Float32MultiArray()
                msg.data = [roll, pitch]

                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error receiving ESP32 data: {e}")
