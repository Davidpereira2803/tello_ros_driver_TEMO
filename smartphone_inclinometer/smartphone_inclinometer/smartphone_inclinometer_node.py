#!/usr/bin/env python3
from smartphone_inclinometer.inclinometer_publisher import SmartphonePublisher
import rclpy


def main(args=None):
    rclpy.init(args=args)
    inclinometer = SmartphonePublisher()
    try:
        rclpy.spin(inclinometer)
    except KeyboardInterrupt:
        inclinometer.get_logger().info("Shutting down Smartphone Inclinometer Publisher.")
    finally:
        inclinometer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()