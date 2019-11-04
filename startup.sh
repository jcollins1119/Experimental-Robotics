#!/bin/bash
source /home/t2-19/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/t2-19/catkin_ws/devel/setup.bash

echo "fuck you"
service ssh start

#roscore 
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud &  
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 &


#rosrun joy joy_node

#rosrun joycontrol_joy

#roslaunch realsense2_camera rs_camera.launch filters:=pointcloud 
