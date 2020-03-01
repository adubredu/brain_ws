#!/bin/bash
source /opt/ros/melodic/setup.bash; 
roscore & 
sleep 3;
roslaunch ps3joy ps3.launch &  
source /home/stark_brain/brain_ws/devel/setup.bash;  
python /home/stark_brain/brain_ws/run_appendages.py & 
rosrun cartbot_motor_drivers pathfollower & 
sleep 1; 
roslaunch usb_cam usb_cam-test.launch & 
roslaunch openni2_launch openni2.launch & 
python3 /home/stark_brain/brain_ws/src/cartbot_motor_drivers/src/start_motors.py
