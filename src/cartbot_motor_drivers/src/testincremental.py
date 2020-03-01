#!/usr/bin/env python
"""
   start_motors.py

   Copyright (c) Alphonsus Adu-Bredu
"""

import rospy
import roslib
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist 
import odrive
from odrive.enums import *


my_drive = odrive.find_any()
print('odrive found')
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

my_drive.axis1.controller.move_incremental(-5,False)
