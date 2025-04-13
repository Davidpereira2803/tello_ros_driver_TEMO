from pynput import keyboard
from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty

class InputHandler:
    def __init__(self, controller):
        self.controller = controller
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        c = self.controller
        print(f"pressing the key {key}")
        try:
            if key.char == "1" and c.shift_key_pressed:
                c.happyperforming = True
                c.emotionactive = True
            if key.char == "2" and c.shift_key_pressed:
                c.sadperforming = True
                c.emotionactive = True
            if key.char == "3" and c.shift_key_pressed:
                c.angryperforming = True
                c.emotionactive = True
            if key.char == "4" and c.shift_key_pressed:
                c.surprisedperforming = True
                c.emotionactive = True
            if key.char == "5" and c.shift_key_pressed:
                c.fearperforming = True
                c.emotionactive = True
            if key.char == "6" and c.shift_key_pressed:
                c.disgustperforming = True
                c.emotionactive = True

            if key.char == "1":
                c.emotionactive = True
                c.notperformed = True
                c.set_control_mode(c.current_mode, c.emotionactive)
            if key.char == "2":
                c.emotionactive = False
                c.notperformed = False
                c.set_control_mode(c.current_mode, c.emotionactive)
            if key.char == "3":
                if not c.ps4controller and not c.smartphone_inclinometer:
                    c.handmotion = True
                    c.set_control_mode("MPU", c.emotionactive)
            if key.char == "4":
                c.handmotion = False
                c.set_control_mode("Default", c.emotionactive)
            if key.char == "5":
                if not c.handmotion and not c.smartphone_inclinometer:
                    c.ps4controller = True
                    c.set_control_mode("PS4", c.emotionactive)
            if key.char == "6":
                c.ps4controller = False
                c.set_control_mode("Default", c.emotionactive)
            if key.char == "7":
                if not c.handmotion and not c.ps4controller:
                    c.smartphone_inclinometer = True
                    c.set_control_mode("PHONE-IMU", c.emotionactive)
            if key.char == "8":
                c.smartphone_inclinometer = False
                c.set_control_mode("Default", c.emotionactive)

            if key.char == "c":
                c._land_pub.publish(Empty())
                c.emotionactive = False
                c.set_control_mode("Default", c.emotionactive)
                c.get_logger().info("Calibrating Inclinometer...")
                c.publishers.calibrate_pub.publish(Empty())

            if key.char == "w":
                c.key_pressed["forward"] = c.speed
            if key.char == "s":
                c.key_pressed["forward"] = -c.speed
            if key.char == "a":
                c.key_pressed["right"] = c.speed
            if key.char == "d":
                c.key_pressed["right"] = -c.speed
            if key.char == "t":
                c.publishers.takeoff_pub.publish(Empty())
            if key.char == "l":
                c.publishers.land_pub.publish(Empty())
            if key.char == "z":
                c.speed = max(0.1, c.speed - 0.1)
            if key.char == "x":
                c.speed = min(1.0, c.speed + 0.1)

        except AttributeError:
            pass

        try:
            if key == key.shift:
                c.shift_key_pressed = True

            if key == key.up and c.shift_key_pressed:
                self._flip(flip_forward=True)
            if key == key.down and c.shift_key_pressed:
                self._flip(flip_backward=True)
            if key == key.left and c.shift_key_pressed:
                self._flip(flip_left=True)
            if key == key.right and c.shift_key_pressed:
                self._flip(flip_right=True)

            if key == key.up and not c.shift_key_pressed:
                c.key_pressed["th"] = c.speed
            if key == key.down and not c.shift_key_pressed:
                c.key_pressed["th"] = -c.speed
            if key == key.left and not c.shift_key_pressed:
                c.key_pressed["cw"] = c.speed
            if key == key.right and not c.shift_key_pressed:
                c.key_pressed["cw"] = -c.speed

        except AttributeError:
            try:
                if key.char == "u" and c.shift_key_pressed:
                    self._flip(flip_forward_left=True)
                if key.char == "i" and c.shift_key_pressed:
                    self._flip(flip_forward_right=True)
                if key.char == "j" and c.shift_key_pressed:
                    self._flip(flip_back_left=True)
                if key.char == "k" and c.shift_key_pressed:
                    self._flip(flip_back_right=True)
            except AttributeError:
                pass

    def on_release(self, key):
        c = self.controller
        if key == keyboard.Key.esc:
            c.shutdown = True
            return False

        try:
            if key.char in ["w", "s"]:
                c.key_pressed["forward"] = 0.0
            if key.char in ["a", "d"]:
                c.key_pressed["right"] = 0.0
        except AttributeError:
            pass

        try:
            if key == key.shift:
                c.shift_key_pressed = False

            if key in [key.up, key.down, key.left, key.right]:
                if c.shift_key_pressed:
                    self._flip()  # cancel all flips
                else:
                    if key == key.up or key == key.down:
                        c.key_pressed["th"] = 0.0
                    if key == key.left or key == key.right:
                        c.key_pressed["cw"] = 0.0

        except AttributeError:
            try:
                if key.char in ["u", "i", "j", "k"] and c.shift_key_pressed:
                    self._flip()
            except AttributeError:
                pass

    def _flip(self, **kwargs):
        msg = FlipControl()
        for attr, val in kwargs.items():
            setattr(msg, attr, val)
        self.controller.publishers.flip_control_pub.publish(msg)
