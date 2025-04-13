from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_msgs.msg import FlipControl, ModeStatus

class PublisherSetup:
    def __init__(self, node):
        self.node = node
        self.tello_vel_cmd_stamped_topic_name = "/cmd_vel"
        self.tello_takeoff_topic_name = "/takeoff"
        self.tello_land_topic_name = "/land"
        self.tello_flip_control_topic_name = "/flip"

        self.cmd_vel_pub = self.node.create_publisher(
            Twist, self.tello_vel_cmd_stamped_topic_name, 1
        )

        self.takeoff_pub = self.node.create_publisher(
            Empty, self.tello_takeoff_topic_name, 1
        )

        self.land_pub = self.node.create_publisher(
            Empty, self.tello_land_topic_name, 1
        )

        self.flip_control_pub = self.node.create_publisher(
            FlipControl, self.tello_flip_control_topic_name, 1
        )

        self.control_mode_pub = self.node.create_publisher(
            String, '/control_mode', 10
        )

        self.control_mode_status_pub = self.node.create_publisher(
            ModeStatus, '/control_mode_status', 10
        )

        self.calibrate_pub = self.node.create_publisher(
            Empty, '/calibrate', 10
        )
