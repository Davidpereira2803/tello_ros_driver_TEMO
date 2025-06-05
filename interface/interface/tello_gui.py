import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt, QTimer
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from tello_msgs.msg import ModeStatus, GameStatus

import numpy as np
from cv_bridge import CvBridge

class TelloGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "tello_gui")
        QWidget.__init__(self)

        self.bridge = CvBridge()

        self.game_mode = "GAMEOFF"
        self.video_feed = 0

        self.setWindowTitle("Tello Interface")
        self.setGeometry(400, 400, 1200, 800)



        self.game_status_label = QLabel("Game Status")
        self.game_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.game_status_label.setStyleSheet("background-color: gray; color: white; border-radius: 5px; padding: 5px; font-size: 16px;")

        left_layout = QVBoxLayout()
        left_layout.addStretch(1)
        left_layout.addWidget(self.game_status_label)
        left_layout.addStretch(1)


        main_layout = QHBoxLayout()

        video_layout = QVBoxLayout()
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        width, height = 600, 500
        black_image = np.zeros((height, width, 3), dtype=np.uint8)
        q_img = QImage(black_image.data, width, height, width * 3, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        self.video_label.setPixmap(pixmap)

        video_layout.addWidget(self.video_label)

        right_layout = QVBoxLayout()
        right_layout.setSpacing(20)
        right_layout.setContentsMargins(10, 20, 10, 20)

        self.current_mode_label = QLabel("Current Mode: Default")
        self.current_mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.emotion_enabled_label = QLabel("Emotion Detection: Disabled")
        self.emotion_enabled_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.game_mode_label = QLabel("Game Mode: GameOff")
        self.game_mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.battery_label = QLabel("Battery: %")
        self.battery_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.tello_speed_label = QLabel("Tello Speed: Default")
        self.tello_speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.mpu_label = QLabel("MPU Values")
        self.mpu_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.flying_status_label = QLabel("Drone in Idle status")
        self.flying_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.flying_status_label.setStyleSheet("background-color: gray; color: white; border-radius: 5px; padding: 5px; font-size: 16px;")

        self.takeoff_timer = QTimer(self)
        self.takeoff_timer.setSingleShot(True)
        self.takeoff_timer.timeout.connect(self.reset_flying_status)

        self.land_timer = QTimer(self)
        self.land_timer.setSingleShot(True)
        self.land_timer.timeout.connect(self.reset_flying_status)

        self.emotion_label = QLabel("Detected Emotion")
        self.emotion_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.emotion_label.setStyleSheet("background-color: gray; color: white; border-radius: 5px; padding: 5px; font-size: 16px;")

        right_layout.addStretch(1)  
        right_layout.addWidget(self.current_mode_label)
        right_layout.addWidget(self.emotion_enabled_label)
        right_layout.addWidget(self.game_mode_label)
        right_layout.addWidget(self.battery_label)
        right_layout.addWidget(self.tello_speed_label)
        right_layout.addWidget(self.mpu_label)
        right_layout.addWidget(self.flying_status_label)
        right_layout.addWidget(self.emotion_label)
        right_layout.addStretch(1)
 
        main_layout.addLayout(left_layout)
        main_layout.addLayout(video_layout)
        main_layout.addLayout(right_layout)
        main_layout.setStretchFactor(left_layout, 1)
        main_layout.setStretchFactor(video_layout, 3)
        main_layout.setStretchFactor(right_layout, 1)
        
        self.setLayout(main_layout)

        self.battery_sub =self.create_subscription(BatteryState, '/battery_state', self.update_battery, 10)

        self.video_sub = self.create_subscription(Image, '/camera/image_raw', self.update_video_feed, 10)

        self.inclinometer_sub = self.create_subscription(Float32MultiArray, '/esp32/inclinometer', self.update_mpu, 10)
        self.emotion_sub = self.create_subscription(String, '/detected_emotion', self.update_emotions, 10)
        self.emotion_enabled_sub = self.create_subscription(ModeStatus, '/control_mode_status', self.update_mode, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.update_tello_speed, 10)
        self.game_status_sub = self.create_subscription(GameStatus, '/game/status', self.update_game_status, 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(16)

    def update_tello_speed(self, msg):
        """ Update tello speed display """
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z

        self.tello_speed_label.setText(
            f"Speed:\n"
            f"Linear X: {linear_x:.2f}\n"
            f"Linear Y: {linear_y:.2f}\n"
            f"Linear Z: {linear_z:.2f}\n"
            f"Angular X: {angular_x:.2f}\n"
            f"Angular Y: {angular_y:.2f}\n"
            f"Angular Z: {angular_z:.2f}"
        )

    def update_mode(self, msg):
        """ Update current mode display """
        mode_map = {
            0: "Default",
            1: "MPU",
            2: "PS4",
            3: "PHONEIMU"
        }
        emotion_map = {
            0: "Disabled",
            1: "Enabled"
        }
        game_mode_map = {
            0: "GAMEOFF",
            1: "GAMEON"
        }

        mode_str = mode_map.get(msg.mode, "Unknown")
        emotion_enabled = emotion_map.get(msg.emotion_enabled, "Unknown") 
        new_game_mode = game_mode_map.get(msg.game_mode, "Unknown")

        if new_game_mode != self.game_mode:
            self.game_mode = new_game_mode
            self.destroy_subscription(self.video_sub)

            if self.game_mode == "GAMEOFF":
                self.video_sub = self.create_subscription(Image, '/camera/image_raw', self.update_video_feed, 10)
            elif self.game_mode == "GAMEON":
                self.video_sub = self.create_subscription(Image, '/camera/game_image', self.update_video_feed, 10)

        self.game_mode_label.setText(f"Game Mode: {self.game_mode}")      
        self.current_mode_label.setText(f"Current Mode: {mode_str}")
        self.emotion_enabled_label.setText(f"Emotion Detection: {emotion_enabled}")

    def update_battery(self, msg):
        """ Update battery status display """
        percentage = msg.percentage
        self.battery_label.setText(f"Battery: {percentage:.1f}%")
    
    def update_mpu(self, msg):
        """ Update mpu values display """
        roll = msg.data[0]
        pitch = msg.data[1]
        takeoff = msg.data[2]
        land = msg.data[3]
        updown = msg.data[4]
        yaw = msg.data[5]

        self.mpu_label.setText(
            f"Received MPU Values:\n"
            f"Roll: {roll:.2f}\n"
            f"Pitch: {pitch:.2f}\n"
            f"Yaw: {yaw:.2f}\n"
            f"Up Down: {updown:.2f}\n"
        )

        if takeoff != 0:
            self.flying_status_label.setText("Status: TakeOff!")
            self.flying_status_label.setStyleSheet("background-color: green; color: white; border-radius: 5px; padding: 10px; font-size: 18px;")
            self.takeoff_timer.start(2500)

        if land != 0:
            self.flying_status_label.setText("Status: Landing!")
            self.flying_status_label.setStyleSheet("background-color: red; color: white; border-radius: 5px; padding: 5px; font-size: 18px;")
            self.land_timer.start(2500)


    def reset_flying_status(self):
        """ Reset flying status label color after 2 second timeout """
        self.flying_status_label.setText("Status: Idle")
        self.flying_status_label.setStyleSheet("background-color: black; color: white; border-radius: 5px; padding: 5px; font-size: 16px;")

    def update_video_feed(self, msg):
        """ Update video feed display """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)

        pixmap = QPixmap.fromImage(q_img)
        self.video_label.setPixmap(pixmap)

    def update_emotions(self, msg):
        """ Update detected emotions with dynamic styling """
        current_emotion = msg.data
        self.emotion_label.setText(f"Current Emotion: {current_emotion}")

        emotion_colors = {
            "Angry": "#ff4c4c",      # Red
            "Disgust": "#66cc66",    # Green
            "Fear": "#8c52ff",       # Purple
            "Happy": "#ffd700",      # Gold
            "Neutral": "#666666",    # Gray
            "Sad": "#3399ff",        # Blue
            "Surprise": "#ff9900"    # Orange
        }

        bg_color = emotion_colors.get(current_emotion, "#000000")
        self.emotion_label.setStyleSheet(f"""
            background-color: {bg_color};
            color: white;
            border-radius: 5px;
            padding: 10px;
            font-size: 16px;
            font-weight: bold;
        """)

    def update_game_status(self, msg):
        """ Update game status display """
        remaining_time = "Game Over" if msg.formatted_time == "00:00" else msg.formatted_time

        game_info = (
            f"Score: {msg.score}\n"
            f"Ammo: {msg.magazine}/10\n"
            f"Targets: {msg.alive_targets} alive, {msg.hit_targets} hit"
            f"\nRemaining Time: {remaining_time}\n"
        )
        self.game_status_label.setText(game_info)

    def ros_spin(self):
        """ Process ROS2 events """
        rclpy.spin_once(self, timeout_sec=0.1)
