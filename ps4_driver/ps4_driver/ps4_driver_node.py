import rclpy
from ps4_driver.ps4_driver import PS4Driver


def main(args=None):
    rclpy.init(args=args)
    node = PS4Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()