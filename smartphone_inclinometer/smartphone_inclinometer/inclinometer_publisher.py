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

UP_THRESHOLD = 1.5
DOWN_THRESHOLD = -1.5
COOLDOWN_TIME = 1 

class SmartphonePublisher(Node):
    def __init__(self):
        super().__init__('smartphone_publisher')

        self.inclinometer_pub = self.create_publisher(Float32MultiArray, '/smartphone/inclinometer', 10)

        self.create_subscription(Empty, '/calibrate', self.trigger_calibration, 10)

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
            self.last_command_time = 0
            self.takeoff_triggered = False
            self.land_triggered = False
            self.takeoff_start_time = None
            self.land_start_time = None
            self.receive_data_active = True
            self.calibration_values = {'roll': 0.0, 'pitch': 0.0, "yaw": 0.0}

            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()

        
    def receive_data(self):
        """Receive data from the smartphone and publish it."""
        while rclpy.ok():
            if not self.receive_data_active:
                time.sleep(0.1)
                continue
            try:
                data, addr = self.sock.recvfrom(1024)
                decoded_data = data.decode().strip()
                self.get_logger().info(f"Received: {decoded_data}")

                parts = decoded_data.split(",")
                values = list(map(float, parts))
                accel = values[:3]
                grav = values[3:6]
                lin = values[6:9]
                orient = values[9:12]

                roll = (orient[2] - self.calibration_values['roll']) * 0.01
                pitch = (orient[1] - self.calibration_values['pitch']) * 0.01
                yaw = self.angle_difference(orient[0], self.calibration_values['yaw'])

                self.take_off(pitch)
                self.land(pitch)
                up_down = self.process_linear_acc(lin[1], time.time())

                msg = Float32MultiArray()
                msg.data = [roll, pitch, yaw, up_down, self.takeoff_triggered, self.land_triggered]
                self.inclinometer_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")

    def process_linear_acc(self, az, current_time):
        """Process the linear acceleration data and determine the command."""
        if current_time - self.last_command_time < COOLDOWN_TIME:
            return 0.0

        if az > UP_THRESHOLD:
            self.last_command_time = current_time
            return 1.0
        elif az < DOWN_THRESHOLD:
            self.last_command_time = current_time
            return -1.0 
        else:
            return 0.0

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
        
    def trigger_calibration(self, msg):
        """Trigger calibration process."""
        self.get_logger().info("Triggering calibration...")
        self.receive_data_active = False
        time.sleep(0.5)

        self.calibration()
        self.receive_data_active = True

    def _wait_for_data(self):
        """Wait for data from Smartphone."""
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                decoded_data = data.decode().strip()
                parts = decoded_data.split(",")
                values = list(map(float, parts))
                accel = values[:3]
                grav = values[3:6]
                lin = values[6:9]
                orient = values[9:12]
                return {'roll': orient[2], 'pitch': orient[1], 'yaw': orient[0]}
            except:
                continue

    def calibration(self):
        self.get_logger().info("Calibrating Inclinometer...")

        self.get_logger().info(" Place the inclinometer in neutral PITCH position.")
        time.sleep(5)
        data = self._wait_for_data()
        self.calibration_values['pitch'] = data['pitch']
        self.get_logger().info(f"Pitch calibrated to {data['pitch']:.2f}")

        self.get_logger().info("Place the inclinometer in neutral ROLL position.")
        time.sleep(5)
        data = self._wait_for_data()
        self.calibration_values['roll'] = data['roll']
        self.get_logger().info(f"Roll calibrated to {data['roll']:.2f}")

        self.get_logger().info("Place the inclinometer in neutral YAW position.")
        time.sleep(5)
        data = self._wait_for_data()
        self.calibration_values['yaw'] = data['yaw']
        self.get_logger().info(f"Yaw calibrated to {data['yaw']:.2f}")

        self.get_logger().info("Calibration complete.")


    def angle_difference(self, current, reference):
        """Compute shortest angular difference (in degrees) between two angles (0-360)."""
        diff = (current - reference + 180) % 360 - 180
        return diff
