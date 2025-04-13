from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from tello_msgs.msg import PS4Buttons, FacePosition

class SubscriberSetup:
    def __init__(self, node):
        self.node = node

        self.face_position_sub = node.create_subscription(
            FacePosition, '/face_position', node.face_position_callback, 1
        )

        self.emotion_sub = node.create_subscription(
            String, '/detected_emotion', node.emotion_callback, 10
        )

        self.inclinometer_sub = node.create_subscription(
            Float32MultiArray, '/esp32/inclinometer', node.inclinometer_callback, 10
        )

        self.ps4_cmd_vel_sub = node.create_subscription(
            Twist, '/ps4_cmd_vel', node.ps4_cmd_vel_callback, 10
        )

        self.ps4_btn_sub = node.create_subscription(
            PS4Buttons, '/ps4_btn', node.ps4_btn_callback, 10
        )
