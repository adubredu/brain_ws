#!/bin/bash
roscore &
sleep 3
source devel/setup.bash
python run_appendages.py &
roslaunch realsense_ros rs_t265.launch &
sleep  3
rosrun cartbot_motor_drivers pathfollower &
sleep 3
rosrun cartbot_motor_drivers local_planner &
sleep 3
python3 src/cartbot_motor_drivers/src/start_motors.py