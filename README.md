# ROS2 Smart Lift

This project aims to develop a physical add-on kit for an elevator to allow the elevator to be controlled by a robot. The add-on kit will be programmed using ROS2 and is ready for future integration into Robotics Middleware Framework (RMF).

### Hardware:
- Raspberry Pi 4,
- Raspberry Pi Camera V2.1,
- PCA 9685 16 x 12-bit PWM,
- SG91R Micro Servo

### Software:
- Ubuntu 22.04,
- ROS2 Humble


### Interfaces:
Custom interfaces were developed to communicate between a robot and this Raspberry Pi add-on kit via DDS.
The custom interfaces include:

**LiftRequests** - To be published by the robot on the /lift_requests topic

**LiftStates** - To be subscribed by the robot on the /lift_states topic


### To start the process, there are a few nodes that have to be run:
Most of the code is written in the lift_controller_v2 package, which will be in the src folder in a ROS2 workspace.
```
ros2 run lift_controller_v2 lift_control_node
```
```
ros2 run lift_controller_v2 button_press_node
```
```
ros2 run lift_controller_v2 qr_decoder
```
```
ros2 run image_tools cam2image
```
