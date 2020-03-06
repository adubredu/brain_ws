#!/usr/bin/env python
"""
   start_motors.py

   Copyright (c) Alphonsus Adu-Bredu
"""

import rospy
import roslib
from std_msgs.msg import Float32
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist 
import odrive
import time
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
        self.speed_gain = 15
        self.fine_increment = 5

        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('fine_motion', String, self.finemotionCallback)
        rospy.spin()
        
                
    def send_speed_to_motors(self):
        left_motor_speed  = (self.dx - self.dr * (self.cart_width/2))*self.speed_gain
        right_motor_speed = (self.dx + self.dr * (self.cart_width/2))*self.speed_gain

        self.my_drive.axis1.controller.vel_setpoint = -left_motor_speed
        self.my_drive.axis0.controller.vel_setpoint = right_motor_speed

    def stop(self):
        time.sleep(0.5)
        self.my_drive.axis1.controller.vel_setpoint = 0
        self.my_drive.axis0.controller.vel_setpoint = 0

    def finemotionCallback(self, data):
        if data.data == 'forward':
            #self.my_drive.axis0.controller.move_incremental(self.fine_increment,True)
            #self.my_drive.axis1.controller.move_incremental(-self.fine_increment,True)

            self.my_drive.axis0.controller.vel_setpoint = self.fine_increment
            self.my_drive.axis1.controller.vel_setpoint = -self.fine_increment
            self.stop()
		
        elif data.data == 'backward':
            #self.my_drive.axis0.controller.move_incremental(-self.fine_increment,True)
            #self.my_drive.axis1.controller.move_incremental(self.fine_increment,True)
        
            self.my_drive.axis0.controller.vel_setpoint = -self.fine_increment
            self.my_drive.axis1.controller.vel_setpoint = self.fine_increment
            self.stop()
		
        elif data.data == 'left':
            #self.my_drive.axis0.controller.move_incremental(self.fine_increment,True)
            #self.my_drive.axis1.controller.move_incremental(self.fine_increment,True)

            self.my_drive.axis0.controller.vel_setpoint = self.fine_increment
            self.my_drive.axis1.controller.vel_setpoint = self.fine_increment
            self.stop()

        elif data.data == 'right':
            #self.my_drive.axis0.controller.move_incremental(-self.fine_increment,True)
            #self.my_drive.axis1.controller.move_incremental(-self.fine_increment,True)

            self.my_drive.axis0.controller.vel_setpoint = -self.fine_increment
            self.my_drive.axis1.controller.vel_setpoint = -self.fine_increment
            self.stop()



    def twistCallback(self,msg):
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
