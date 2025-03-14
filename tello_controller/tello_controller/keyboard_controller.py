from rclpy.node import Node
from pynput import keyboard

# ROS messages
from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty, String, Float32MultiArray
from geometry_msgs.msg import Twist

import time



class Controller(Node):
    # - Topics
    tello_vel_cmd_stamped_topic_name = "/cmd_vel"
    tello_takeoff_topic_name = "/takeoff"
    tello_land_topic_name = "/land"
    tello_flip_control_topic_name = "/flip"

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
        self._keyboard_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.shift_key_pressed = False
        self.shutdown = False

        # Detected emotion subscriber
        self.emotion_sub = self.create_subscription(String, 'detected_emotion', self.emotion_callback, 10)

        # ESP-32 subscriber 
        self.subscription = self.create_subscription( Float32MultiArray, '/esp32/inclinometer', self.inclinometer_callback, 10)

        # activate hand motion control
        self.handmotion = False

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
        self.timer = self.create_timer(self.calltime, self.emotion_reactions)  


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

    def begin(self):
        self.init_pub()
        self.init_timers()
        self._keyboard_listener.start()

    def init_timers(self):
        self.cmd_vel_timer = self.create_timer(0.05, self.cmd_vel_callback)

    def init_pub(self):
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.tello_vel_cmd_stamped_topic_name, 1
        )

        self._takeoff_pub = self.create_publisher(
            Empty, self.tello_takeoff_topic_name, 1
        )

        self._land_pub = self.create_publisher(
            Empty, self.tello_land_topic_name, 1
        )

        self._flip_control_pub = self.create_publisher(
            FlipControl, self.tello_flip_control_topic_name, 1
        )


    def emotion_callback(self, msg):
        """Callback for emotion updates."""
        self.latest_emotion = msg.data  


    def inclinometer_callback(self, msg):
        """Process inclinometer data and map it to drone movement."""
        roll = msg.data[0]
        pitch = msg.data[1]
        yaw = msg.data[2]
        #updown = msg.data[3]
        takeoff = msg.data[3]
        land = msg.data[4]

        left_right = max(-1.0, min(1.0, roll * 0.05))
        forward_backward = max(-1.0, min(1.0, pitch * 0.05))
        clockwise = max(-1.0, min(1.0, yaw * 0.05))
        #up_down = max(-1.0, min(1.0, updown))

        if abs(left_right) < 0.1:
            left_right = 0.0
        if abs(forward_backward) < 0.1:
            forward_backward = 0.0
        if abs(clockwise) < 0.1:
            clockwise = 0.0

        if self.handmotion:
            self.get_logger().info(f"Received roll: {roll}, pitch: {pitch}, yaw: {yaw}")
            self.key_pressed["right"] = left_right
            self.key_pressed["forward"] = forward_backward
            self.key_pressed["cw"] = clockwise
            #self.key_pressed["th"] = up_down

        if takeoff:
            self.get_logger().info(f"Drone is about to take off!")
            self._takeoff_pub.publish(Empty())
            self.key_pressed["right"] = left_right
            self.key_pressed["forward"] = forward_backward
            self.key_pressed["cw"] = clockwise
        
        if land:
            self.get_logger().info(f"Drone is about to land!")
            self._land_pub.publish(Empty())
            self.key_pressed["right"] = left_right
            self.key_pressed["forward"] = forward_backward
            self.key_pressed["cw"] = clockwise

    
    def emotion_reactions(self):
        """Function to perfrom the emotion related reactions"""
        if self.latest_emotion:
            self.get_logger().info(f"Latest Emotion: {self.latest_emotion}")

            if self.emotionactive:

                # Happy
                if self.happyperforming or (self.notperformed and (self.latest_emotion == "Happy")):
                    self.get_logger().info("Happy Front FLip!")
                    msg = FlipControl()
                    msg.flip_forward = False
                    msg.flip_backward = True
                    msg.flip_left = False
                    msg.flip_right = False
                    self._flip_control_pub.publish(msg)
                    self.notperformed = False
                    self.happyclean = True
                    self.happyperforming = False

                    return

                if self.happyclean:
                    msg = FlipControl()
                    msg.flip_forward = False
                    msg.flip_backward = False
                    msg.flip_left = False
                    msg.flip_right = False
                    msg.flip_forward_left = False
                    msg.flip_forward_right = False
                    msg.flip_back_left = False
                    msg.flip_back_right = False
                    self._flip_control_pub.publish(msg)
                    self.get_logger().info("Happy Clean!")
                    self.happyclean = False
                    self.happyperf = True
                    return

                if self.happyperf:
                    msg = FlipControl()
                    msg.flip_forward = True
                    msg.flip_backward = False
                    msg.flip_left = False
                    msg.flip_right = False
                    self._flip_control_pub.publish(msg)
                    self.happyperf = False
                    self.get_logger().info("Happy Back Flip!")

                # Angry
                if  self.angryperforming or (self.notperformed and (self.latest_emotion == "Angry")):

                    self.key_pressed["th"] = -self.speed
                    

                    self.get_logger().info("Angry Movement 1!")
                    self.notperformed = False
                    self.angryclean = True
                    self.angryperforming = False

                    return

                if self.angryclean:
                    
                    self.key_pressed["th"] = 0.0

                    self.get_logger().info("Angry Clean!")
                    self.angryclean = False
                    self.angryperf = True
                    return

                if self.angryperf and self.count < 4:
                    self.count += 1
                    self.key_pressed["cw"] = self.speed

                    self.angryperf2 = True
                    self.get_logger().info("Angry Movement 2!")
                    return
                elif self.angryperf:
                    self.key_pressed["cw"] = 0.0
                    self.count = 0
                    self.angryperf = False

                if self.angryperf2:

                    self.key_pressed["forward"] = self.speed

                    self.angryperf2 = False
                    self.get_logger().info("Angry Movement 3!")
                    return 

                self.key_pressed["forward"] = 0.0

                # Sad
                if self.sadperforming or (self.notperformed and (self.latest_emotion == "Sad")):

                    self.key_pressed["forward"] = -self.speed
                    

                    self.get_logger().info("Sad Movement 1!")
                    self.notperformed = False
                    self.sadclean = True
                    self.sadperforming = False

                    return

                if self.sadclean:
                    
                    self.key_pressed["forward"] = 0.0

                    self.get_logger().info("Sad Clean!")
                    self.sadclean = False
                    self.sadperf = True
                    return

                if self.sadperf and self.count < 4:
                    self.count += 1
                    self.key_pressed["cw"] = self.speed

                    self.get_logger().info("Sad Movement 2!")
                    return
                elif self.sadperf:
                    self.key_pressed["cw"] = 0.0
                    self.count = 0
                    self.sadperf = False
                
                # Surprised
                if self.surprisedperforming or (self.notperformed and (self.latest_emotion == "Surprise")):

                    self.key_pressed["th"] = self.speed

                    self.get_logger().info("Surprised Movement 1!")
                    self.notperformed = False
                    self.surprisedclean = True
                    self.surprisedperforming = False

                    return

                if self.surprisedclean:
                    
                    self.key_pressed["th"] = 0.0

                    self.get_logger().info("Surprised Clean!")
                    self.surprisedclean = False
                    if not self.surprisedperf2:
                        self.surprisedperf = True
                    self.surprisedperf2 = False

                    return

                if self.surprisedperf and self.count < 3:
                    self.count += 1
                    self.key_pressed["cw"] = self.speed * 2

                    self.get_logger().info("Surprised Movement 2!")

                    return
                elif self.surprisedperf:
                    self.key_pressed["cw"] = 0.0
                    self.count = 0
                    self.surprisedperf = False
                    self.surprisedperf2 = True

                    return

                if self.surprisedperf2:

                    self.key_pressed["th"] = -self.speed

                    self.get_logger().info("Surprised Movement 3!")
                    self.surprisedclean = True

                    return

                # Fear
                if self.fearperforming or (self.notperformed and (self.latest_emotion == "Fear")):
                    
                    self._land_pub.publish(Empty())

                    self.get_logger().info("Fear Movement 1!")
                    self.notperformed = False
                    self.fearperforming = False

                    return
            

                # Disgust
                if self.disgustperforming or (self.notperformed and (self.latest_emotion == "Disgust")):

                    self.key_pressed["forward"] = -self.speed * 2

                    self.key_pressed["th"] = self.speed * 2


                    self.get_logger().info("Disgust Movement 1!")
                    self.notperformed = False
                    self.disgustclean = True
                    self.disgustperforming = False

                    return

                
                if self.disgustclean:

                    self.key_pressed["forward"] = 0.0
                    
                    self.key_pressed["th"] = 0.0

                    self.get_logger().info("Disgust Clean!")
                    self.disgustclean = False

                    return


        else:
            self.get_logger().info("No emotion detected yet.")

    def on_press(self, key):
        print(f"pressing the key {key}")
        try:
            # Perform Happy Movement
            if key.char == "1":
                self.happyperforming = True
                self.emotionactive = True

            # Perform Sad Movement
            if key.char == "2":
                self.sadperforming = True
                self.emotionactive = True

            # Perform Angry Movement
            if key.char == "3":
                self.angryperforming = True
                self.emotionactive = True
            
            # Perform Surprised Movement
            if key.char == "4":
                self.surprisedperforming = True
                self.emotionactive = True

            # Perform Fear Movement
            if key.char == "5":
                self.fearperforming = True
                self.emotionactive = True

            # Perform Disgust Movement
            if key.char == "6":
                self.disgustperforming = True
                self.emotionactive = True


            # Activate Emotion Reaction
            if key.char == "7":
                self.emotionactive = True
                self.notperformed = True
            # Deactivate Emotion Reaction
            if key.char == "8":
                self.emotionactive = False
                self.notperformed = False

            # Activate Hand Motion Control
            if key.char == "9":
                self.handmotion = True
            
            # Deactivate Hand Motion Control
            if key.char == "0":
                self.handmotion = False

            if key.char == "w":
                self.key_pressed["forward"] = self.speed
            if key.char == "s":
                self.key_pressed["forward"] = -self.speed
            if key.char == "d":
                self.key_pressed["right"] = -self.speed
            if key.char == "a":
                self.key_pressed["right"] = self.speed
            if key.char == "t":
                self._takeoff_pub.publish(Empty())
            if key.char == "l":
                self._land_pub.publish(Empty())
            if key.char == "z":
                self.speed -= 0.1
                if self.speed < 0.1:
                    self.speed = 0.1
            if key.char == "x":
                self.speed += 0.1
                if self.speed > 1:
                    self.speed = 1
        except AttributeError:
            pass

        try:
            if key == key.shift:
                self.shift_key_pressed = True
            if key == key.up and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = True
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                msg.flip_right = False
            if key == key.down and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = True
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
            if key == key.left and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = True
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
            if key == key.right and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = True
                self._flip_control_pub.publish(msg)

            if key == key.up and not self.shift_key_pressed:
                self.key_pressed["th"] = self.speed
            if key == key.down and not self.shift_key_pressed:
                self.key_pressed["th"] = -self.speed
            if key == key.left and not self.shift_key_pressed:
                self.key_pressed["cw"] = self.speed
            if key == key.right and not self.shift_key_pressed:
                self.key_pressed["cw"] = -self.speed

        except AttributeError:
            if key.char == "u" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = True
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "i" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = True
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "j" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = True
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "k" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = True
                self._flip_control_pub.publish(msg)

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.shutdown = True
            return False
        try:
            if key.char == "w":
                self.key_pressed["forward"] = 0.0
            if key.char == "s":
                self.key_pressed["forward"] = 0.0
            if key.char == "d":
                self.key_pressed["right"] = 0.0
            if key.char == "a":
                self.key_pressed["right"] = 0.0
        except AttributeError:
            pass

        try:
            if key == key.shift:
                self.shift_key_pressed = False

            if key == key.up and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.down and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.left and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.right and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)

            if key == key.up and not self.shift_key_pressed:
                self.key_pressed["th"] = 0.0
            if key == key.down and not self.shift_key_pressed:
                self.key_pressed["th"] = 0.0
            if key == key.left and not self.shift_key_pressed:
                self.key_pressed["cw"] = 0.0
            if key == key.right and not self.shift_key_pressed:
                self.key_pressed["cw"] = 0.0

        except AttributeError:
            if key.char == "u" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "i" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "j" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "k" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)

    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.key_pressed["forward"]
        msg.linear.y = self.key_pressed["right"]
        msg.linear.z = self.key_pressed["th"]

        msg.angular.z = self.key_pressed["cw"]
        self.cmd_vel_pub.publish(msg)