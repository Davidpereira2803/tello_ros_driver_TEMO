#!/usr/bin/env python3
from tello_game_interface.tello_game_interface import TelloGame
import rclpy
import cv2


def main(args=None):
    rclpy.init(args=args)
    game = TelloGame()
    try:
        rclpy.spin(game)
    except KeyboardInterrupt:
        pass
    game.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()