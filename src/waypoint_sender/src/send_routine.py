#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from math import sqrt, pow

class transmit_routine:
	def __init__(self):
		rospy.init_node('transmit_routine')
		self.robotX = 0; self.robotY = 0;
		self.pub = rospy.Publisher("/way_point", PointStamped, queue_size=1)
		rospy.Subscriber("/state_estimation", Odometry, self.handleOdometry)
		self.tolerance = 1.0
		self.xarray = [4.41,3.736,2.389,0.918,0.14,-0.649,-1.38,-2.223,-2.599,-2.418,-1.339, -0.170,1.297,2.383,3.889,4.836]
		self.yarray = [-5.95,-5.233,-4.976,-4.506,-4.218,-3.978,-3.786,-3.626,-4.076,-4.897,-5.414,-5.72,-5.821,-5.935,-6.455,-6.092]
		self.x = [4.93,4.5,3.27,1.73,-0.39,-1.82,-1.86,-1.15,0.055,1.89,4.15]
		self.y = [-3.50,-4.71,-5.28,-5.08,-5.05,-4.48,-3.42,-2.64,-2.84,-3.08,-3.53]
		self.count = 0
		self.size = 11
		self.transmit_routine()
		rospy.spin()


	def handleOdometry(self,odom):
		self.robotX = odom.pose.pose.position.x 
		self.robotY = odom.pose.pose.position.y 


	def at_goal(self,goalX,goalY):
		distance = sqrt(pow((self.robotX - goalX),2) + pow((self.robotY - goalY),2));
		if (distance < self.tolerance):
			return True;

		return False;

	def transmit_routine(self):
		while not rospy.is_shutdown():
			wp = PointStamped()
			wp.header.frame_id="odom"
			x = self.x[self.count%self.size]
			y = self.y[self.count%self.size]
			wp.point.x = x
			wp.point.y = y

			self.pub.publish(wp)
			if self.at_goal(x,y):
				self.count+=1




if __name__=='__main__':
    try:
        send_goal = transmit_routine()

    except rospy.ROSInterruptException: pass
