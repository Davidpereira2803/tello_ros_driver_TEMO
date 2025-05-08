#!/bin/bash

# Run colcon build first and wait for it to finish
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; colcon build; echo 'Colcon build finished! Press any key to continue...'; read"

read -p "Press Enter to continue launching processes..."

# Launch Driver T1
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch tello_driver tello_driver.launch.py; exec bash"

# Launch Controller T2
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch tello_controller tello_controller.launch.py; exec bash"

# Launch ESP32 Controller T3
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch esp32_controller esp32.launch.py; exec bash"

# Launch Graphical Interface T4
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch interface interface.launch.py; exec bash"

# Launch PS4 Driver T5
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch ps4_driver ps4_driver.launch.py; exec bash"

# Launch Game Logic T6
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch tello_game_interface tello_game.launch.py; exec bash"
