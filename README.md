# Tello ROS2 Wrapper TEMO (Fork)
(This is not the original README.md -> [View Original README](docs/ORIGINAL_README.md))

## Introduction

This repository is a forked version of the Tello driver developped by the SNT-ARG team, it adds an emotion recognition model to the driver, and drone reaction movements for 6 emotions. Furthermore, a graphical interface was developped to display the video feed and some sensory data, such as the battery status.

## What is new?

The project adds 2 new main functionalities:

1. Facial emotion detection and reaction movements to each emotion
2. Full drone control using an inclinometer module (MPU9250) paired with a microcontroller (ESP32)

Emotion Recognition Model used: [GitHub Link](https://github.com/SHAIK-AFSANA/facialemotionrecognizerinrealtime)

## How to start and launch the project

### Build & Launch

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

#### Build & Launch all together

Be in the root folder of the ROS2 environment, and use this command: 
```bash
./src/tello_ros_driver_TEMO/launch_all.sh
```
---

### How to use it

After launching everything there is a interface on which one can see the video feed and some data from the drone and the MPU-9250 sensor. After clicking on the interface, one can start using the keyboard by default to control the drone:

| Key  | Action                      | Description                             |
|------|-----------------------------|-----------------------------------------|
| `t`  | take off                    | drone lifts off into the air            |
| `l`  | land                        | drone lands safely                      |
| `w`  | forward                     | moves the drone forward                 |
| `s`  | backward                    | moves the drone backward                |
| `a`  | left                        | moves the drone left                    |
| `d`  | right                       | moves the drone right                   |
| `↑`  | move up                     | increases altitude                      |
| `↓`  | move down                   | decreases altitude                      |
| `←`  | yaw left                    | rotates the drone left                  |
| `→`  | yaw right                   | rotates the drone right                 |
| `z`  | increase speed              | increases the drone speed by 0.1        |
| `x`  | decrease speed              | reduces the speed by 0.1                |
| `c`  | calibrate | starts MPU calibration |
| `space` | shoot | shoots one bullet in GAMEON mode|
| `r`| reload | reloads the magazine in GAMEON mode|
| `Shift + ↑`   | flip forward       | drone flips forward                     |
| `Shift + ↓`   | flip backward      | drone flips backward                    |
| `Shift + ←`   | flip left          | drone flips left                        |
| `Shift + →`   | flip right         | drone flips right                       |
| `Shift + 1`  | happy movement              | drone performs happy movement           |
| `Shift + 2`  | sad movement                | drone performs sad movement             |
| `Shift + 3`  | angry movement              | drone performs angry movement           |
| `Shift + 4`  | surprised movement          | drone performs surprised movement       |
| `Shift + 5`  | fear movement               | drone performs fear movement            |
| `Shift + 6`  | disgust movement            | drone performs disgust movement         |
| `1`  | activate emotion reaction   | enables emotion reaction mode           |
| `2`  | deactivate emotion reaction | disables emotion reaction mode          |
| `3`  | activate MPU control        | enables MPU control                     |
| `4`  | deactivate MPU control      | disables MPU control                    |
| `5`  | activate PS4 control   | enables PS4 control           |
| `6`  | deactivate PS4 control | disables PS4 control         |
| `7`  | activate Smartphone IMU control   | enables Smartphone IMU control          |
| `8`  | deactivate Smartphone IMU control | disables Smartphone IMU control         |
| `9`  | activate Game mode (GAMEON)        | starts the game                   |
| `0`  | deactivate Game mode (GAMEOFF)      | ends the game                   |

**Note:**
- Arrow keys (`↑`, `↓`, `←`, `→`) control altitude and yaw rotation.  
- **Shift + Arrow Keys** trigger flips in the corresponding direction.  
- Speed adjustments (`z`, `x`) modify the drone's speed gradually.

---

### PS4 Controller Mappings


| Button/Joystick | Action                      |
|-----------------|-----------------------------|
| `L3`--Left Joystick   | Move up/down, left/right |
| `R3`--Right Joystick  | Move forward/backward, yaw left/yaw right|
| `L2` | Reload |
| `R2` | Shoot |
| `L1` | Decrease Speed |
| `R1` | Increase Speed |
| `✕`         | Take off                    |
| `○`   | Land                        |
| `△`  | Set GAMEON   |
| `□`   | Set GAMEOFF |
| `Options`   | Activate Default Mode |
| `↑` | Front Flip |
| `↓` | Back Flip |
| `←` | Left Flip |
| `→` | Right Flip |
