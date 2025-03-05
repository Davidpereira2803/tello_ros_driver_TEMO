# Tello ROS2 Wrapper TEMO (Fork)
(This is not the original README.md -> [View Original README](docs/ORIGINAL_README.md))

## Introduction

This repository is a fork, and adds an emotion recognition model to the driver, and adds drone reaction movements for 6 emotions.

Emotion Recognition Model used: [GitHub Link](https://github.com/SHAIK-AFSANA/facialemotionrecognizerinrealtime)

### How to start

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

