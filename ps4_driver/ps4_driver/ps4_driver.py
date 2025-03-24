from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class PS4Driver(Node):
    def __init__(self):
        super().__init__('ps4_driver')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.takeoff_publisher = self.create_publisher(Empty, '/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/land', 10)

        self.subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        self.prev_buttons = []

    def joystick_callback(self, msg: Joy):
        twist = Twist()

        dead_zone = 0.2 

        def apply_dead_zone(value):
            return value if abs(value) > dead_zone else 0.0

        twist.linear.x = apply_dead_zone(msg.axes[1]) * 0.5
        twist.linear.y = apply_dead_zone(msg.axes[0]) * 0.5
        twist.linear.z = apply_dead_zone(msg.axes[5]) * 0.5
        twist.angular.z = apply_dead_zone(msg.axes[2]) * 1.0

        self.publisher_.publish(twist)

        if not self.prev_buttons:
            self.prev_buttons = msg.buttons
            return

        if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.takeoff_publisher.publish(Empty())

        if msg.buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.land_publisher.publish(Empty())

        self.prev_buttons = msg.buttons
