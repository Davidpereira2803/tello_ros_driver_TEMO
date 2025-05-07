from rclpy.node import Node
from pynput import keyboard
from rclpy.duration import Duration

# ROS messages
from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty, String, Float32MultiArray
from geometry_msgs.msg import Twist
from tello_msgs.msg import PS4Buttons, ModeStatus, FacePosition

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

        # Face position subscriber
        self.face_position_sub = self.create_subscription(FacePosition, '/face_position', self.face_position_callback, 1)

        # Detected emotion subscriber
        self.emotion_sub = self.create_subscription(String, '/detected_emotion', self.emotion_callback, 10)

        # ESP-32 subscriber 
        self.subscription = self.create_subscription( Float32MultiArray, '/esp32/inclinometer', self.inclinometer_callback, 10)

        # PS4 Controller subscriber
        self.ps4_cmd_vel_sub = self.create_subscription(Twist, '/ps4_cmd_vel', self.ps4_cmd_vel_callback, 10)

        # PS4 Buttons subscriber
        self.ps4_btn_sub = self.create_subscription(PS4Buttons, '/ps4_btn', self.ps4_btn_callback, 10)

        # Smartphone Inclinometer subscriber
        self.smartphone_inclinometer_sub = self.create_subscription(Float32MultiArray, '/smartphone/inclinometer', self.smartphone_inclinometer_callback, 10)

        # activate ps4 controller
        self.ps4controller = False

        # activate hand motion control
        self.handmotion = False

        # activate smartphone inclinometer
        self.smartphone_inclinometer = False

        # Current mode
        self.current_mode = "Default"
        self.game_mode = "GAMEOFF"

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
        self.timer = self.create_timer(self.calltime, self.emotion_reactions)  

        self.last_up_time = self.get_clock().now()

        self.last_updown_time = self.get_clock().now()
        self.updown_cooldown = Duration(seconds=1.0)

        self.z_movement_duration = Duration(seconds=0.5)
        self.z_movement_end_time = self.get_clock().now()
        self.z_movement_active = 0.0



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

        self.control_mode_pub = self.create_publisher(
            String, '/control_mode',
            10
        )

        self.control_mode_pub_msg = self.create_publisher(
            ModeStatus, '/control_mode_status',
            10
        )

        self.calibrate_pub = self.create_publisher(
            Empty, '/calibrate',
            10
        )

    def set_control_mode(self, mode: str, emotion_enabled: bool, game_mode: bool):
        if self.current_mode != mode:
            self.current_mode = mode
            msg_str = String()
            msg_str.data = mode
            self.control_mode_pub.publish(msg_str)

        msg_status = ModeStatus()
        mode_map = {
            "Default": ModeStatus.DEFAULT,
            "MPU": ModeStatus.MPU,
            "PS4": ModeStatus.PS4,
            "PHONEIMU": ModeStatus.PHONEIMU
        }

        msg_status.mode = mode_map.get(mode, ModeStatus.DEFAULT)
        msg_status.emotion_enabled = ModeStatus.ENABLED if emotion_enabled else ModeStatus.DISABLED
        msg_status.game_mode = ModeStatus.GAMEON if game_mode == "GAMEON" else ModeStatus.GAMEOFF

        self.control_mode_pub_msg.publish(msg_status)


    def face_position_callback(self, msg):
        """Callback for face position updates."""
        face_area = msg.face_area
        x_offset = msg.x_offset
        y_offset = msg.y_offset

        if self.emotionactive:
        
            if x_offset > 130:
                self.key_pressed["cw"] = -1.0
            elif x_offset < -130:
                self.key_pressed["cw"] = 1.0
            else:
                self.key_pressed["cw"] = 0.0

            if y_offset < -130:
                self.key_pressed["th"] = -1.0
            elif y_offset > 130:
                self.key_pressed["th"] = 1.0
            else:
                self.key_pressed["th"] = 0.0


            if face_area < 5000:
                self.key_pressed["forward"] = 1.0
            elif face_area > 25000:
                self.key_pressed["forward"] = -1.0
            else:
                self.key_pressed["forward"] = 0.0


    def emotion_callback(self, msg):
        """Callback for emotion updates."""
        self.latest_emotion = msg.data  

    def ps4_cmd_vel_callback(self, msg):
        """Callback for ps4 controller updates."""

        if self.ps4controller:
            #self.get_logger().info(f"Received ps4 command: {msg}") 
            self.key_pressed["th"] = msg.linear.z * self.speed
            self.key_pressed["right"] = msg.linear.y * self.speed
            self.key_pressed["forward"] = msg.linear.x * self.speed
            self.key_pressed["cw"] = msg.angular.z * self.speed

    def ps4_btn_callback(self, msg):
        """Callback for ps4 controller button updates."""

        if self.current_mode == "PS4":

            if msg.buttons[5] == 1:
                self.speed += 0.1
                if self.speed > 1:
                    self.speed = 1
            
            if msg.buttons[6] == 1:
                self.speed -= 0.1
                if self.speed < 0.1:
                    self.speed = 0.1

            if msg.buttons[0] == 1:
                self._takeoff_pub.publish(Empty())

            if msg.buttons[1] == 1:
                self._land_pub.publish(Empty())

            if msg.buttons[9] == 1:
                self.ps4controller = False
                self.set_control_mode("Default", self.emotionactive, self.game_mode)

            if msg.buttons[13] == 1 and self.fliptriggered == False:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = True
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                self.fliptriggered = True

                return
            
            if msg.buttons[14] == 1 and self.fliptriggered == False:
                msg = FlipControl()
                msg.flip_forward = True
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                self.fliptriggered = True

                return
            
            if msg.buttons[15] == 1 and self.fliptriggered == False:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = True
                self._flip_control_pub.publish(msg)
                self.fliptriggered = True

                return
            
            if msg.buttons[16] == 1 and self.fliptriggered == False:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = True
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                self.fliptriggered = True

                return

            if self.fliptriggered:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                self.fliptriggered = False

                return

    def smartphone_inclinometer_callback(self, msg):
        """Process smartphone inclinometer data and map it to drone movement."""
        roll = msg.data[0]
        pitch = msg.data[1]
        yaw = msg.data[2]
        up_down = msg.data[3]
        takeoff = msg.data[4]
        land = msg.data[5]

        if yaw < -10:
            clockwise = -1.0
        elif yaw > 10:
            clockwise = 1.0
        else:
            clockwise = 0.0

        if self.smartphone_inclinometer:
            self.get_logger().info(f"Received roll: {roll}, pitch: {pitch}, yaw: {yaw}, updown: {up_down}")
            self.key_pressed["right"] = roll
            self.key_pressed["forward"] = pitch
            self.key_pressed["cw"] = clockwise
            self.key_pressed["th"] = up_down

            if takeoff:
                self.get_logger().info(f"Drone is about to take off!")
                self._takeoff_pub.publish(Empty())
            
            if land:
                self.get_logger().info(f"Drone is about to land!")
                self._land_pub.publish(Empty())

    def inclinometer_callback(self, msg):
        """Process inclinometer data and map it to drone movement."""
        roll = msg.data[0]
        pitch = msg.data[1]
        takeoff = msg.data[2]
        land = msg.data[3]
        up_down = msg.data[4]
        yaw = msg.data[5]

        left_right = roll
        forward_backward = pitch
        now = self.get_clock().now()

        if yaw < -10 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2 and 0.9 < up_down < 1.1:
            clockwise = -1.0
        elif yaw > 10 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2 and 0.9 < up_down < 1.1:
            clockwise = 1.0
        else:
            clockwise = 0.0

        if self.get_clock().now() < self.z_movement_end_time:
            z_movement = self.z_movement_active
        else:
            z_movement = 0.0

        deviation = up_down - 1.0
        if (now - self.last_updown_time) > self.updown_cooldown:
            if deviation > 0.3 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2:
                self.z_movement_active = 1.0
                self.z_movement_end_time = now + self.z_movement_duration
                self.last_updown_time = now
                self.get_logger().info(f"Up movement detected")
            elif deviation < -0.3 and abs(left_right) < 0.2 and abs(forward_backward) < 0.2:
                self.z_movement_active = -1.0
                self.z_movement_end_time = now + self.z_movement_duration
                self.last_updown_time = now
                self.get_logger().info(f"Down movement detected")


        if abs(left_right) < 0.2:
            left_right = 0.0
        if abs(forward_backward) < 0.2 or abs(forward_backward) > 0.85:
            forward_backward = 0.0

        if self.handmotion:
            #self.get_logger().info(f"Received roll: {roll}, pitch: {pitch}, yaw: {clockwise}, updown: {z_movement}")
            self.key_pressed["right"] = -left_right
            self.key_pressed["forward"] = -forward_backward
            self.key_pressed["cw"] = clockwise
            self.key_pressed["th"] = z_movement

            if takeoff:
                #self.get_logger().info(f"Drone is about to take off!")
                self._takeoff_pub.publish(Empty())

            if land:
                #self.get_logger().info(f"Drone is about to land!")
                self._land_pub.publish(Empty())
    
    def emotion_reactions(self):
        """Function to perfrom the emotion related reactions"""
        if self.emotionactive:
            if self.latest_emotion:
                self.get_logger().info(f"Latest Emotion: {self.latest_emotion}")

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
                self.get_logger().info("No emotion yet.")

    def on_press(self, key):
        print(f"pressing the key {key}")
        try:
            # Perform Happy Movement
            if key.char == "1" and self.shift_key_pressed:
                self.happyperforming = True
                self.emotionactive = True

            # Perform Sad Movement
            if key.char == "2" and self.shift_key_pressed:
                self.sadperforming = True
                self.emotionactive = True

            # Perform Angry Movement
            if key.char == "3" and self.shift_key_pressed:
                self.angryperforming = True
                self.emotionactive = True
            
            # Perform Surprised Movement
            if key.char == "4" and self.shift_key_pressed:
                self.surprisedperforming = True
                self.emotionactive = True

            # Perform Fear Movement
            if key.char == "5" and self.shift_key_pressed:
                self.fearperforming = True
                self.emotionactive = True

            # Perform Disgust Movement
            if key.char == "6" and self.shift_key_pressed:
                self.disgustperforming = True
                self.emotionactive = True


            # Activate Emotion Reaction
            if key.char == "1":
                self.emotionactive = True
                self.notperformed = True
                self.set_control_mode(self.current_mode, self.emotionactive, self.game_mode)
            # Deactivate Emotion Reaction
            if key.char == "2":
                self.emotionactive = False
                self.notperformed = False
                self.set_control_mode(self.current_mode, self.emotionactive, self.game_mode)

            # Activate Hand Motion Control with MPU
            if key.char == "3":
                if self.ps4controller == False and self.smartphone_inclinometer == False:
                    self.handmotion = True
                    self.set_control_mode("MPU", self.emotionactive, self.game_mode)
            # Deactivate Hand Motion Control with MPU
            if key.char == "4":
                self.handmotion = False
                self.set_control_mode("Default", self.emotionactive, self.game_mode)

            if key.char == "5":
                if self.handmotion == False and self.smartphone_inclinometer == False:
                   self.ps4controller = True
                   self.set_control_mode("PS4", self.emotionactive, self.game_mode)
            # Deactivate PS4 Controller
            if key.char == "6":
                self.ps4controller = False
                self.set_control_mode("Default", self.emotionactive, self.game_mode)

            # Activate Smartphone Inclinometer
            if key.char == "7":
                if self.handmotion == False and self.ps4controller == False:
                   self.smartphone_inclinometer = True
                   self.set_control_mode("PHONEIMU", self.emotionactive, self.game_mode)
            # Deactivate Smartphone Inclinometer
            if key.char == "8":
                self.smartphone_inclinometer = False
                self.set_control_mode("Default", self.emotionactive, self.game_mode)


            # Calibrate Inclinometer
            if key.char == "c":
                if self.handmotion or self.smartphone_inclinometer:
                    self._land_pub.publish(Empty())
                    self.emotionactive = False
                    self.set_control_mode("Default", self.emotionactive, self.game_mode)
                    self.get_logger().info("Calibrating Inclinometer...")
                    self.calibrate_pub.publish(Empty())                

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