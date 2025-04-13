from rclpy.node import Node
from pynput import keyboard

from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty, String, Float32MultiArray
from geometry_msgs.msg import Twist
from tello_msgs.msg import PS4Buttons, ModeStatus, FacePosition

import sys

from .publisher_setup import PublisherSetup
from .subscriber_setup import SubscriberSetup
from .input_handler import InputHandler
from .emotion_handler import EmotionHandler



class Controller(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.print_controls()
        self.key_pressed = {
            "th": 0.0,
            "right": 0.0,
            "forward": 0.0,
            "cw": 0.0,
        }
        self.speed = 0.5  # from 0 - 1
        self.shift_key_pressed = False
        self.shutdown = False

        # activate ps4 controller
        self.ps4controller = False

        # activate hand motion control
        self.handmotion = False

        # activate smartphone inclinometer
        self.smartphone_inclinometer = False

        # Current mode
        self.current_mode = "Default"

        # Flip triggered
        self.fliptriggered = False

        self.latest_emotion = None
        self.notperformed = True
        self.emotionactive = False
        
        # Happy
        self.happyperforming = False
        self.happyperf = False
        self.happyclean = False

        # Sad
        self.sadperforming = False
        self.sadperf = False
        self.sadclean = False

        # Angry
        self.angryperforming = False
        self.angryperf = False
        self.angryclean = False
        self.angryperf2 = False
        self.count = 0

        # Surprised
        self.surprisedperforming = False
        self.surprisedperf = False
        self.surprisedclean = False
        self.surprisedperf2 = False

        # Fear
        self.fearperforming = False
        self.fearperf = False

        # Disgust
        self.disgustperforming = False
        self.disgustperf = False
        self.disgustclean = False


        self.calltime = 1.5

        self.publishers = PublisherSetup(self)
        self.subscribers = SubscriberSetup(self)
        self.input_handler = InputHandler(self)
        self.emotion_handler = EmotionHandler(self)

    def print_controls(self):
        print("---------------------")
        print("- Movement Controls -")
        print("---------------------")
        print("w: pitch forward")
        print("s: pitch backward")
        print("a: roll left")
        print("d: roll right")
        print("up arrow: more altitude")
        print("down arrow: less altitude")
        print("left arrow: yaw ccw")
        print("right arrow: yaw cw")
        print("--------------------")
        print("- General Controls -")
        print("--------------------")
        print("t: takeoff")
        print("l: land")
        print("x: more speed")
        print("z: less speed")
        print("-----------------")
        print("- Flip Controls -")
        print("-----------------")
        print("shift + up arrow: flip forward")
        print("shift + down arrow: flip backward")
        print("shift + left arrow: flip left")
        print("shift + right arrow: flip right")
        print("shift + u: flip forward left")
        print("shift + i: flip forward right")
        print("shift + j: flip backwards left")
        print("shift + k: flip backwards right")
        print("--------------------------------------------")

        sys.stdout.flush()

    def begin(self):
        self.print_controls()
        self.init_timers()

    def init_timers(self):
        self.cmd_vel_timer = self.create_timer(0.05, self.cmd_vel_callback)

    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.key_pressed["forward"]
        msg.linear.y = self.key_pressed["right"]
        msg.linear.z = self.key_pressed["th"]

        msg.angular.z = self.key_pressed["cw"]
        self.cmd_vel_pub.publish(msg)