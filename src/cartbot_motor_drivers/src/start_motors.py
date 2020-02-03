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

#############################################################
#############################################################
class SendSpeedToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("send_speed_to_motors")
        
        self.my_drive = odrive.find_any()
        rospy.loginfo("odrive found")
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.cart_width = 0.64
        self.wheel_diameter = 0.16
        self.speed_gain = 60

        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.spin()
        
                
    #############################################################
    def send_speed_to_motors(self):
    #############################################################
    
        '''
        #stop
        if self.dr == 0 and self.dx == 0:
            self.my_drive.axis1.controller.vel_setpoint = 0
            self.my_drive.axis0.controller.vel_setpoint = 0

        #turn left
        if self.dr > 0 and self.dx == 0:
            self.my_drive.axis1.controller.vel_setpoint = -20
            self.my_drive.axis0.controller.vel_setpoint = -20

        #turn right
        elif self.dr < 0 and self.dx == 0:
            self.my_drive.axis1.controller.vel_setpoint = 20
            self.my_drive.axis0.controller.vel_setpoint = 20

        #move forward
        elif self.dx > 0 and self.dr == 0:
            self.my_drive.axis1.controller.vel_setpoint = -40
            self.my_drive.axis0.controller.vel_setpoint = 40

        #move backward
        elif self.dx < 0 and self.dr == 0:
            self.my_drive.axis1.controller.vel_setpoint = 40
            self.my_drive.axis0.controller.vel_setpoint = -40

        #move forward left
        elif self.dr < 0 and self.dx > 0:
            self.my_drive.axis1.controller.vel_setpoint = -60
            self.my_drive.axis0.controller.vel_setpoint = 45

        #move backward right
        elif self.dr > 0 and self.dx < 0:
            self.my_drive.axis1.controller.vel_setpoint = 45
            self.my_drive.axis0.controller.vel_setpoint = -60

        #move forward right
        elif self.dx > 0 and self.dr > 0:
            self.my_drive.axis1.controller.vel_setpoint = -45
            self.my_drive.axis0.controller.vel_setpoint = 60

        #move backwardright
        elif self.dx < 0 and self.dr < 0:
            self.my_drive.axis1.controller.vel_setpoint = 60
            self.my_drive.axis0.controller.vel_setpoint = -45

        '''
        left_motor_speed  = (self.dx - self.dr * (self.cart_width/2))*self.speed_gain
        right_motor_speed = (self.dx + self.dr * (self.cart_width/2))*self.speed_gain

        self.my_drive.axis1.controller.vel_setpoint = -left_motor_speed
        self.my_drive.axis0.controller.vel_setpoint = right_motor_speed

        # print(left_motor_speed," --- ", right_motor_speed)





    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
        self.send_speed_to_motors()
    


#############################################################
#############################################################
if __name__ == '__main__':
    try:
        send = SendSpeedToMotors()
    except rospy.ROSInterruptException:
        pass
