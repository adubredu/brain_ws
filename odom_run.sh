#!/bin/bash
./serials.sh
roscore &
sleep 3
source devel/setup.bash
python run_appendages.py &
roslaunch realsense2_camera rs_t265.launch &
sleep  3
roslaunch rplidar_launch rplidar.launch &
sleep 3
rosrun cartbot_motor_drivers pathfollower &
sleep 3
rosrun cartbot_motor_drivers local_planner &
sleep 3
roslaunch odometry_publisher odometry_publisher.launch &
sleep 3
roslaunch ps3joy ps3.launch &
sleep 3
python3 src/cartbot_motor_drivers/src/start_motors.py
