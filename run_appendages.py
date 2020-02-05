#! /usr/bin/env python
import rospy
import serial
import time
from sensor_msgs.msg import Joy

port="/dev/ttyACM1"
gport="/dev/ttyACM0"
ser=serial.Serial(port,9600)
gser = serial.Serial(gport,9600);
gser.flushInput
ser.flushInput
status = 0


class joystick_interface:
    def __init__(self):
        rospy.init_node('joystick_interface_node')
        self.direction = 0
	self.gdirection = 0
	self.extdirection = 0
        self.open_picker = None
	self.open_gripper = None
        self.close_picker = None
	self.close_gripper = None
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.run_elevator()
        rospy.spin()


    def joy_callback(self,data):
	if data.axes[1]!=0.0:
        	if data.buttons[8] == 1:
			self.direction = data.axes[1]
			self.extdirection = 0
		else:
			self.extdirection = data.axes[1]
			self.direction = 0
	else:
		self.extdirection = 0
		self.direction = 0
        self.open_picker = data.buttons[0]
        self.close_picker = data.buttons[4]

	self.open_gripper = data.buttons[1]
	self.close_gripper = data.buttons[3]
	self.gdirection = data.axes[0]

    def raise_gripper(self):
	ser.write(b'i')
	print('raise gripper')

    def lower_gripper(self):
	ser.write(b'k')
	print('lower gripper')

    def raise_picker(self):
        ser.write(b'f')
	print('raise picker')

    def extend_gripper(self):
	gser.write(b'y')
	print('extending gripper')

    def retract_gripper(self):
	gser.write(b'h')
	print('retracting gripper')           

    def lower_picker(self):
        ser.write(b'r')
	print('lower picker')


    def stop_g_moving(self):
	ser.write(b'm')
	print('stop moving gripper')


    def stop_moving(self):
        ser.write(b's')
	print('stop moving picker')
   

    def control_gripper(self):
	if self.open_gripper == 1:
		gser.write(b'p')
		print('open gripper')
	elif self.close_gripper == 1:
		gser.write(b'l')
		print('close gripper')

    def control_pick_up(self):
        if self.open_picker == 1:
                ser.write(b'o')
		print('open picker')
        elif self.close_picker == 1:
                ser.write(b'c')
		print('close picker')

    def run_elevator(self):
        while not rospy.is_shutdown():
            if self.extdirection ==1:
		self.extend_gripper()
	    elif self.extdirection == -1:
		self.retract_gripper()

            if self.direction == 1:
                self.raise_picker()
            elif self.direction == -1:
                self.lower_picker()
            #elif self.direction == 0:
                #self.stop_moving()
	
	    if self.gdirection == 1:
		self.raise_gripper()
	    elif self.gdirection == -1:
		self.lower_gripper()
	    #elif self.gdirection == 0:
		#self.stop_g_moving()
	
	    self.control_gripper()
            self.control_pick_up()
                        
            time.sleep(0.5)

                    


if __name__=='__main__':
    try:
        joy = joystick_interface()
    
    except rospy.ROSInterruptException:
        pass
