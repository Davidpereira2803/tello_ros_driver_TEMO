#!/usr/bin/env python3
from interface.tello_gui import TelloGUI
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
import rclpy
import sys


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = TelloGUI()
    gui.show()
    sys.exit(app.exec())
    rclpy.spin(gui)


if __name__ == '__main__':
    main()