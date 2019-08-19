# kawasaki
ROS interface to Kawasaki robots.


# Network
Set a static IP in range 192.168.2.xxx


# Package
- Build the kawasaki_driver package
- Import the driver in python


# Notes
- Make sure if you are using ipython to use ipython2 version (otherwise it will
  not import).
Do not use brew version, install through: pip2 install ipython.


# Running
- To run a node start: rosrun kawasaki_driver driver_simple.py
- The driver publishes the joint state and current robot pose and subscribes
  to a pose topic which defines the next target.


# Robot
- Turn off Teach mode
- Turn on Motors
- Switch to Repeat mode on the controller  


# Requirements
- rospy
- geometry_msgs
- sensor_msgs
- std_msgs
