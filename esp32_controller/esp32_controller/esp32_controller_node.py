#!/usr/bin/env python3
from esp32_controller.esp32_publisher import ESP32Publisher
import rclpy


def main(args=None):
    rclpy.init(args=args)
    esp32 = ESP32Publisher()
    try:
        rclpy.spin(esp32)
    except KeyboardInterrupt:
        esp32.get_logger().info("Shutting down ESP32 Publisher.")
    finally:
        esp32.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()