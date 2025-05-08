import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
import time
import pygame
from tello_msgs.msg import PS4Buttons, Game, GameStatus, ModeStatus

from playsound import playsound

class TelloGame(Node):
    def __init__(self):
        super().__init__('tello_game_node')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.ps4_btn_sub = self.create_subscription(PS4Buttons, '/ps4_btn', self.ps4_button_callback, 10)
        self.trigger_state_sub = self.create_subscription(Game, '/trigger_state', self.update_trigger_state, 10)
        self.control_mode_status_sub = self.create_subscription(ModeStatus, '/control_mode_status', self.update_mode, 10)

        self.alive_targets = set()
        self.score = 0
        self.magazine = 10
        self.shoot_pressed = False 
        self.reload_pressed = False
        self.game_mode = "GAMEOFF"

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        self.camera_pub = self.create_publisher(Image, '/camera/game_image', 10)
        self.game_status_pub = self.create_publisher(GameStatus, '/game/status', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        h, w = frame.shape[:2]
        center_screen = (w // 2, h // 2)

        hit_positions = []

        if ids is not None:
            ids = ids.flatten()
            for corner, marker_id in zip(corners, ids):
                self.alive_targets.add(marker_id)

                if marker_id not in self.alive_targets:
                    continue

                aruco.drawDetectedMarkers(frame, [corner])
                c = corner[0]
                center_x, center_y = c[:, 0].mean(), c[:, 1].mean()
                enemy_center = (int(center_x), int(center_y))
                cv2.circle(frame, enemy_center, 20, (0, 204, 255), -1)

        if self.shoot_pressed and self.magazine > 0:
            playsound('/home/david/Projects/TEMO_ros_ws/src/tello_ros_driver_TEMO/sound/gun.wav')
            if ids is not None:
                for corner, marker_id in zip(corners, ids):
                    if marker_id not in self.alive_targets:
                        continue

                    c = corner[0]
                    center_x, center_y = c[:, 0].mean(), c[:, 1].mean()
                    enemy_center = (int(center_x), int(center_y))
                    dist = np.linalg.norm(np.array(center_screen) - np.array(enemy_center))

                    if dist < 30:
                        self.alive_targets.remove(marker_id)
                        hit_positions.append(enemy_center)
                        self.score += 1
                        self.get_logger().info(f"Target hit!") 

            self.magazine -= 1
            self.shoot_pressed = False

        elif self.magazine == 0 and self.shoot_pressed:
            self.get_logger().info(f"Reload!")

        if self.reload_pressed and self.magazine < 10:
            playsound('/home/david/Projects/TEMO_ros_ws/src/tello_ros_driver_TEMO/sound/reload.wav')
            self.magazine = 10
            self.get_logger().info(f"Reloading!")

        
        cv2.drawMarker(frame, center_screen, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=25, thickness=2)

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "tello_game"
        self.camera_pub.publish(ros_image)


        game_status_msg = GameStatus()
        game_status_msg.score = self.score
        game_status_msg.magazine = self.magazine
        game_status_msg.total_targets = len(self.alive_targets) + self.score
        game_status_msg.alive_targets = len(self.alive_targets)
        game_status_msg.hit_targets = self.score

        self.game_status_pub.publish(game_status_msg)


    def ps4_button_callback(self, msg):
        if self.game_mode == "GAMEOFF":
            return
        self.shoot_pressed = msg.buttons[7] == 1 
        self.reload_pressed = msg.buttons[6] == 1
    
    def update_trigger_state(self, msg):
        if self.game_mode == "GAMEOFF":
            return
        self.shoot_pressed = msg.state == 1
        self.reload_pressed = msg.state == 2

    def update_mode(self, msg):
        """
        Update the mode based on the received message.
        """
        game_mode_map = {
            0: "GAMEOFF",
            1: "GAMEON"
        }
        self.game_mode = game_mode_map.get(msg.game_mode, "Unknown")
