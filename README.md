# Tello ROS2 Wrapper TEMO (Fork)
(This is not the original README.md -> [View Original README](docs/ORIGINAL_README.md))

## Introduction

This repository is a fork of the Tello driver developped by the SNT-ARG team, it adds an emotion recognition model to the driver, and drone reaction movements for 6 emotions. Furthermore, a graphical interface was developped to display the video feed and some sensory data, such as the battery status.

## What is new?

The project adds 2 new main functionalities:

1. Facial emotion detection and reaction movements to each emotion
2. Full drone control using an inclinometer module (MPU9250) paired with a microcontroller (ESP32)

Emotion Recognition Model used: [GitHub Link](https://github.com/SHAIK-AFSANA/facialemotionrecognizerinrealtime)

## How to start and launch the project
Create a ROS2 environment and clone this repository into the `src`folder. Always stay in the root folder of the ROS2 environment for the next steps.

#### Manually launch one by one

Install ROS2, then source the environment
```bash
source /opt/ros/jazzy/setup.bash
```
Create a ROS workspace
```bash
mkdir -p ~/TEMO_ROS/src
```

```bash
cd ~/TEMO_ROS/src
```

```bash
git clone https://github.com/Davidpereira2803/tello_ros_driver_TEMO.git

cd ..
```
Next build the app
```bash
colcon build
```

and source the build

```bash
source install/setup.bash
```

Tello Driver
```bash
ros2 launch tello_driver tello_driver.launch.py
```

Keyboard Controller
```bash
ros2 launch tello_controller tello_controller.launch.py
```

ESP32 + MPU9250 Controller
```bash
ros2 launch esp32_controller esp32.launch.py
```

Graphical Interface
```bash
ros2 launch interface interface.launch.py
```
---

