from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_msgs.msg import FlipControl, ModeStatus

import sys

from .publisher_setup import PublisherSetup
from .subscriber_setup import SubscriberSetup
from .input_handler import InputHandler
from .emotion_handler import EmotionHandler



class Controller(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Movement dictionary
        self.key_pressed = {
            "th": 0.0,
            "right": 0.0,
            "forward": 0.0,
            "cw": 0.0,
        }

        self.speed = 0.5
        self.shutdown = False
        self.shift_key_pressed = False

        # Mode flags
        self.ps4controller = False
        self.handmotion = False
        self.smartphone_inclinometer = False
        self.current_mode = "Default"

        # Emotion states
        self.emotionactive = False
        self.notperformed = True
        self.latest_emotion = None

        # Emotion reaction flags
        self.count = 0
        self.happyperforming = False
        self.happyperf = False
        self.happyclean = False

        self.sadperforming = False
        self.sadperf = False
        self.sadclean = False

        self.angryperforming = False
        self.angryperf = False
        self.angryclean = False
        self.angryperf2 = False

        self.surprisedperforming = False
        self.surprisedperf = False
        self.surprisedclean = False
        self.surprisedperf2 = False

        self.fearperforming = False
        self.fearperf = False

        self.disgustperforming = False
        self.disgustperf = False
        self.disgustclean = False

        self.fliptriggered = False

        # Timer setup interval for emotion reactions
        self.calltime = 1.5

        # === Setup all modular subsystems ===
        self.publishers = PublisherSetup(self)
        self.subscribers = SubscriberSetup(self)
        self.input_handler = InputHandler(self)
        self.emotion_handler = EmotionHandler(self)

    def begin(self):
        self.print_controls()
        self.init_timers()

    def init_timers(self):
        self.cmd_vel_timer = self.create_timer(0.05, self.cmd_vel_callback)

    def print_controls(self):
        print_controls()

    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.key_pressed["forward"]
        msg.linear.y = self.key_pressed["right"]
        msg.linear.z = self.key_pressed["th"]
        msg.angular.z = self.key_pressed["cw"]
        self.publishers.cmd_vel_pub.publish(msg)

    def set_control_mode(self, mode: str, emotion_enabled: bool):
        if self.current_mode != mode:
            self.current_mode = mode
            msg_str = String()
            msg_str.data = mode
            self.publishers.control_mode_pub.publish(msg_str)

        msg_status = ModeStatus()
        mode_map = {
            "Default": ModeStatus.DEFAULT,
            "MPU": ModeStatus.MPU,
            "PS4": ModeStatus.PS4
        }

        msg_status.mode = mode_map.get(mode, ModeStatus.DEFAULT)
        msg_status.emotion_enabled = (
            ModeStatus.ENABLED if emotion_enabled else ModeStatus.DISABLED
        )
        self.publishers.control_mode_status_pub.publish(msg_status)

    # These callbacks are wired in the SubscriberSetup class
    def face_position_callback(self, msg):
        if self.emotionactive:
            if msg.x_offset > 130:
                self.key_pressed["cw"] = -1.0
            elif msg.x_offset < -130:
                self.key_pressed["cw"] = 1.0
            else:
                self.key_pressed["cw"] = 0.0

            if msg.y_offset < -130:
                self.key_pressed["th"] = -1.0
            elif msg.y_offset > 130:
                self.key_pressed["th"] = 1.0
            else:
                self.key_pressed["th"] = 0.0

            if msg.face_area < 5000:
                self.key_pressed["forward"] = 1.0
            elif msg.face_area > 25000:
                self.key_pressed["forward"] = -1.0
            else:
                self.key_pressed["forward"] = 0.0

    def emotion_callback(self, msg):
        self.latest_emotion = msg.data

    def ps4_cmd_vel_callback(self, msg):
        if self.ps4controller:
            self.key_pressed["th"] = msg.linear.z * self.speed
            self.key_pressed["right"] = msg.linear.y * self.speed
            self.key_pressed["forward"] = msg.linear.x * self.speed
            self.key_pressed["cw"] = msg.angular.z * self.speed

    def ps4_btn_callback(self, msg):
        if self.current_mode == "PS4":
            if msg.buttons[5] == 1:
                self.speed = min(1.0, self.speed + 0.1)
            if msg.buttons[6] == 1:
                self.speed = max(0.1, self.speed - 0.1)
            if msg.buttons[0] == 1:
                self.publishers.takeoff_pub.publish(Empty())
            if msg.buttons[1] == 1:
                self.publishers.land_pub.publish(Empty())
            if msg.buttons[9] == 1:
                self.ps4controller = False
                self.set_control_mode("Default", self.emotionactive)

            if not self.fliptriggered:
                if msg.buttons[13] == 1:
                    self._trigger_flip(flip_backward=True)
                elif msg.buttons[14] == 1:
                    self._trigger_flip(flip_forward=True)
                elif msg.buttons[15] == 1:
                    self._trigger_flip(flip_right=True)
                elif msg.buttons[16] == 1:
                    self._trigger_flip(flip_left=True)

    def _trigger_flip(self, **kwargs):
        msg = FlipControl()
        for k, v in kwargs.items():
            setattr(msg, k, v)
        self.publishers.flip_control_pub.publish(msg)
        self.fliptriggered = True

    def inclinometer_callback(self, msg):
        roll = msg.data[0]
        pitch = msg.data[1]
        takeoff = msg.data[2]
        land = msg.data[3]
        up_down = msg.data[4]
        yaw = msg.data[5]

        left_right = roll
        forward_backward = pitch

        clockwise = 0.0
        z_movement = 0.0

        if yaw < -5 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2 and 0.9 < up_down < 1.1:
            clockwise = -1.0
        elif yaw > 5 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2 and 0.9 < up_down < 1.1:
            clockwise = 1.0

        if up_down > 1.1 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2:
            z_movement = 1.0
        elif up_down < 0.9 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2:
            z_movement = -1.0

        if abs(left_right) < 0.2:
            left_right = 0.0
        if abs(forward_backward) < 0.2 or abs(forward_backward) > 0.85:
            forward_backward = 0.0

        if self.handmotion:
            self.get_logger().info(f"Received roll: {roll}, pitch: {pitch}, yaw: {yaw}, z: {up_down}")
            self.key_pressed["right"] = -left_right
            self.key_pressed["forward"] = -forward_backward
            self.key_pressed["cw"] = clockwise
            self.key_pressed["th"] = z_movement

            if takeoff:
                self.get_logger().info("Drone is about to take off!")
                self.publishers.takeoff_pub.publish(Empty())
            if land:
                self.get_logger().info("Drone is about to land!")
                self.publishers.land_pub.publish(Empty())
