# Computer Vision (Tag Tracking) AR Drone
This ROS package utilizes OpenCV and ARDrone_Autonomy on ROS to have a Parrot AR Drone follow an augmented reality tag.

[Check out a short demo video here](https://youtu.be/gC164K3fPVw)

## Prerequisites
* [ROS (Kinetic)](http://wiki.ros.org/kinetic/Installation) - Provides the basic I/O and infrastructure for the AR Drone
* [OpenCV](https://opencv.org/) - Open Source computer vision library to process images from the Drone 
* [ArUco](http://www.uco.es/investiga/grupos/ava/node/26) - simple-to-use fast augmented reality tage recognition library
* [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/) - ROS driver for the AR Drone 1.0 and 2.0
* [aruco_ros](https://github.com/pal-robotics/aruco_ros) - ROS wrapper for the ArUco library, publishes pose information to ROS topics

## ROS Workspace Setup
After installing ROS, OpenCV, ArUco and ardrone_autonomy, initialize a new catkin workspace. In the src folder clone the aruco_ros library and `catkin_make`. then clone this repo to the src folder and make again. 

## Startup and Controls
There is probably a more organized way to do this but I ran 3 different terminals to run the commands, the image_view window from the drone must be in focus for the key inputs to work correctly.

* bring up the AR Drone and connect to the wireless network
* In the first terminal run `roslaunch ardrone_autonomy ardrone.launch` to bring up the ardrone topics
* In the second terminal run `roslaunch aruco_ros single.launch markerId:=100 markerSize:=0.168` to bring up the tag recognition
* In a third terminal run `rosrun ardrone_tag_follow ardrone_tag_follow.py` to bring up the follow control
* press `t` to takeoff and `space` to land 

__NOTE: The drone will follow the last known position of the tag, if it's within the deadzone the drone will continue to hover.__

## Notes
* to view aruco debug info run `rosrun image_view image_view image:=/aruco_single/result`
* to switch camera on the ardrone run `rosservice call /ardrone/togglecam`
* the tags folder has some simple AR tags that can be printed to use with aruco libraries
