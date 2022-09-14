# metr4202_ximea_ros

# METR4202 Ximea Camera Setup
## Introduction
This tutorial is provided as guidelines for setting up the given Ximea cameras with ROS.

There will be four steps to this tutorial:
- Step 1: Install the ROS Ximxea Package
- Step 2: Camera Calibration
- Step 3: Setup the ArUco Tag Detection Library

**Note:**
- **This can be done on any Ubuntu 20.04 system with IO/USB access.**
- **You cannot run the Ximea Camera on Construct/ROSjects or WSL.**
- **You should run this natively, on the RPi4 or on a dual booted machine.**
- **You will need to see the GUI for the calibration part of the tutorial.**
- **Do not login with root, as this will affect your permissions.**

After following the RPi4 setup, clone this to your `src` folder in your ROS workspace.
```console
cd ~/catkin_ws/src
```
```console
git clone https://github.com/UQ-METR4202/metr4202_ximea_ros.git
```
Then return to your workspace
```console
cd ~/catkin_ws/
```

# Step 1: Install the ROS Ximxea Package

## Building the Packages
You may need to install the dependency ```vision_msgs```
```console
sudo apt install ros-noetic-vision-msgs
```
- Run `catkin_make` or `catkin build` in your workspace to build the packages.



## Testing the XIMEA Camera

- You need to run the following command after each boot to disable the USB memory limits
```console
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

Run the XIMEA ROS camera node
```console
rosrun ximea_ros ximea_demo
```
You can check the output using `rqt_image_view`
```console
rosrun rqt_image_view rqt_image_view
```

# Step 2: Camera Calibration
If it isn't installed you can install the camera calibration ROS package
```console
sudo apt install ros-noetic-camera-calibration
```
You can run the `ximea_demo` node, if it isn't already running.
```console
rosrun ximea_ros ximea_demo
```

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/ximea_cam/image_raw camera:=/ximea_cam
```

# Step 3: Setup the ArUco Tag Detection Library
```console
roslaunch ximea_ros ximea_aruco.launch
```
