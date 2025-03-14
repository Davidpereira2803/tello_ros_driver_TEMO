import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt, QTimer
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from cv_bridge import CvBridge

class TelloGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "tello_gui")
        QWidget.__init__(self)

        self.bridge = CvBridge()

        self.setWindowTitle("Tello Interface")
        self.setGeometry(600, 500, 1000, 800)

        main_layout = QHBoxLayout()

        video_layout = QVBoxLayout()
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        width, height = 500, 400
        black_image = np.zeros((height, width, 3), dtype=np.uint8)
        q_img = QImage(black_image.data, width, height, width * 3, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        self.video_label.setPixmap(pixmap)

        video_layout.addWidget(self.video_label)

        right_layout = QVBoxLayout()
        self.battery_label = QLabel("Battery: %")
        self.battery_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.mpu_label = QLabel("MPU Values")
        self.mpu_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.emotion_label = QLabel("Detected Emotion")
        self.emotion_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        right_layout.addStretch(1)  
        right_layout.addWidget(self.battery_label)
        right_layout.addWidget(self.mpu_label)
        right_layout.addWidget(self.emotion_label)
        right_layout.addStretch(1)
 
        main_layout.addLayout(video_layout)
        main_layout.addLayout(right_layout)
        main_layout.setStretchFactor(video_layout, 3)
        main_layout.setStretchFactor(right_layout, 1)
        
        self.setLayout(main_layout)

        self.create_subscription(BatteryState, '/battery_state', self.update_battery, 10)
        self.create_subscription(Image, '/camera/image_raw', self.update_video_feed, 10)
        self.create_subscription(Float32MultiArray, '/esp32/inclinometer', self.update_mpu, 10)
        self.emotion_sub = self.create_subscription(String, 'detected_emotion', self.update_emotions, 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)

    def update_battery(self, msg):
        """ Update battery status display """
        percentage = msg.percentage
        self.battery_label.setText(f"Battery: {percentage:.1f}%")
    
    def update_mpu(self, msg):
        """ Update mpu values display """
        roll = msg.data[0]
        pitch = msg.data[1]
        yaw = msg.data[2]
        takeoff = msg.data[3]
        land = msg.data[4]
        self.mpu_label.setText(f"Received roll: {roll}, pitch: {pitch}, yaw: {yaw}, take off: {takeoff}, land: {land}")

    def update_video_feed(self, msg):
        """ Update video feed display """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)

        pixmap = QPixmap.fromImage(q_img)
        self.video_label.setPixmap(pixmap)

    def update_emotions(self, msg):
        """ Update detected emotions"""
        current_emotion = msg.data
        self.emotion_label.setText(f"Current Emotion: {current_emotion}")

    def ros_spin(self):
        """ Process ROS2 events """
        rclpy.spin_once(self, timeout_sec=0.1)
