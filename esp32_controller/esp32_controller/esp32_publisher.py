#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import math
import time

ESP32_IP = "192.168.4.2"
ESP32_PORT = 8888

TAKEOFF_THRESHOLD = 80
LAND_THRESHOLD = -80
TAKEOFFLAND_HOLD_TIME = 1

UP_ACCEL_THRESHOLD = 0.1
DOWN_ACCEL_THRESHOLD = -0.1
MOVEMENT_COOLDOWN = 1

MOVEMENT_HOLD_TIME = 0.3

class ESP32Publisher(Node):
    def __init__(self):
        super().__init__('esp32_publisher')
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/esp32/inclinometer', 10)
        
        self.sock = None
        self.get_logger().info("Starting ESP32 UDP listener...")

        while self.sock is None and rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.bind((ESP32_IP, ESP32_PORT))

                self.get_logger().info("ESP32 UDP listener started. Waiting for data...")
            except Exception as e:
                self.get_logger().error(f"Error starting ESP32 UDP listener: {e}")
                self.sock = None
                time.sleep(3)

        if self.sock:
            self.takeoff_triggered = False
            self.land_triggered = False
            self.takeoff_start_time = None
            self.land_start_time = None
            self.last_up_time = 0
            self.last_down_time = 0
            self.up_detected_time = 0
            self.down_detected_time = 0
            self.upward_movement = False
            self.downward_movement = False
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

                self.take_off(pitch)    
                self.land(pitch)

                self.detect_up_down(az)

                msg = Float32MultiArray()
                msg.data = [roll, pitch, self.takeoff_triggered, self.land_triggered, self.upward_movement, self.downward_movement]

                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error receiving ESP32 data: {e}")

    def compute_pitch(self, ax, ay, az):
        """
        Compute pitch angle from accelerometer data.
        ax, ay, az: Accelerometer readings (in g)
        Returns pitch angle in degrees.
        """
        return -math.degrees(math.atan2(-ay, -az))
    
    def compute_roll(self, ax, ay, az):
        """Compute roll angle from accelerometer data."""
        roll_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        roll_deg = math.degrees(roll_rad)
        return roll_deg
    
    def take_off(self, pitch):
        """Publish takeoff message."""
        if not self.takeoff_triggered:
            if pitch > TAKEOFF_THRESHOLD:
                if self.takeoff_start_time is None:
                    self.takeoff_start_time = time.time()
                elif time.time() - self.takeoff_start_time > TAKEOFFLAND_HOLD_TIME:
                    self.takeoff_triggered = True
                    self.land_triggered = False
            else:
                self.takeoff_start_time = None
            
    def land(self, pitch):
        """Publish land message."""
        if not self.land_triggered and self.takeoff_triggered:
            if pitch < LAND_THRESHOLD:
                if self.land_start_time is None:
                    self.land_start_time = time.time()
                elif time.time() - self.land_start_time > TAKEOFFLAND_HOLD_TIME:
                    self.land_triggered = True
                    self.takeoff_triggered = False
            else:
                self.land_start_time = None

    def detect_up_down(self, az):
        """Detect upward and downward movement."""
        current_time = time.time()

        delta_az = az - (-1)

        if delta_az > UP_ACCEL_THRESHOLD and (current_time - self.last_up_time) > MOVEMENT_COOLDOWN:
            self.last_up_time = current_time
            self.up_detected_time = current_time

        elif delta_az < DOWN_ACCEL_THRESHOLD and (current_time - self.last_down_time) > MOVEMENT_COOLDOWN:
            self.last_down_time = current_time
            self.down_detected_time = current_time

        self.upward_movement = (current_time - self.up_detected_time) < MOVEMENT_HOLD_TIME
        self.downward_movement = (current_time - self.down_detected_time) < MOVEMENT_HOLD_TIME