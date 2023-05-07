# Camera trigger Qt
Creating custom GUI with Qt and ROS2

## Pre-Requisites

- ROS2 Foxy installed.

- [ROS2 wrapper for Intel Depth Cameras Series 400](https://github.com/IntelRealSense/realsense-ros) compiled and working.

- Clone this repository in our workspace (ws/src)


## How to launch the program

1. Connect the series D400 camera.

2. Launch rs_launch.py from ROS2 Wrapper: ros2 launch realsense2_camera rs_launch.py

3. Launch gui.launch.py from this repository: ros2 launch camera_trigger_qt gui.launch.py 
