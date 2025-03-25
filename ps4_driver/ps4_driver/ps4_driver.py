from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from tello_msgs.msg import PS4Buttons

class PS4Driver(Node):
    def __init__(self):
        super().__init__('ps4_driver')

        self.twist_publisher_ = self.create_publisher(Twist, '/ps4_cmd_vel', 10)
        self.btn_publisher_ = self.create_publisher(PS4Buttons, '/ps4_btn', 10)

        self.takeoff_publisher = self.create_publisher(Empty, '/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/land', 10)

        self.subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        self.prev_buttons = []

    def joystick_callback(self, msg: Joy):
        twist = Twist()

        dead_zone = 0.3 

        def apply_dead_zone(value):
            return value if abs(value) > dead_zone else 0.0

        twist.linear.x = apply_dead_zone(msg.axes[1]) * 1.0
        twist.linear.y = apply_dead_zone(msg.axes[0]) * 1.0
        twist.linear.z = apply_dead_zone(msg.axes[4]) * 1.0
        twist.angular.z = apply_dead_zone(msg.axes[3]) * 1.0

        self.twist_publisher_.publish(twist)


        buttons = list(msg.buttons) 

        buttons += [
            1 if msg.axes[7] == -1.0 else 0,
            1 if msg.axes[7] == 1.0 else 0,
            1 if msg.axes[6] == -1.0 else 0,
            1 if msg.axes[6] == 1.0 else 0 
        ]

        buttons_msg = PS4Buttons()
        buttons_msg.buttons = buttons
        self.btn_publisher_.publish(buttons_msg)

        