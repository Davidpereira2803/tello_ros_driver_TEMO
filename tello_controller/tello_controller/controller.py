from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_msgs.msg import FlipControl, ModeStatus

import sys

class Controller(Node):
    # - Topics
    tello_vel_cmd_stamped_topic_name = "/cmd_vel"
    tello_takeoff_topic_name = "/takeoff"
    tello_land_topic_name = "/land"
    tello_flip_control_topic_name = "/flip"

    def __init__(self, node_name):
        super().__init__(node_name)
        self.print_controls()

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

        # Flip state
        self.fliptriggered = False

        # Keyboard listener
        self._keyboard_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )

        # Timer for emotion callback (to be moved)
        self.calltime = 1.5
        self.timer = self.create_timer(self.calltime, self.emotion_reactions)

    def begin(self):
        self.print_controls()
        self.init_pub()
        self.init_timers()
        self._keyboard_listener.start()

    def init_timers(self):
        self.cmd_vel_timer = self.create_timer(0.05, self.cmd_vel_callback)

    def init_pub(self):
        self.cmd_vel_pub = self.create_publisher(Twist, self.tello_vel_cmd_stamped_topic_name, 1)
        self._takeoff_pub = self.create_publisher(Empty, self.tello_takeoff_topic_name, 1)
        self._land_pub = self.create_publisher(Empty, self.tello_land_topic_name, 1)
        self._flip_control_pub = self.create_publisher(FlipControl, self.tello_flip_control_topic_name, 1)
        self.control_mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.control_mode_pub_msg = self.create_publisher(ModeStatus, '/control_mode_status', 10)
        self.calibrate_pub = self.create_publisher(Empty, '/calibrate', 10)

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
        print("shift + arrow keys or u/i/j/k for flips")
        print("----------------------------------------")
        sys.stdout.flush()
