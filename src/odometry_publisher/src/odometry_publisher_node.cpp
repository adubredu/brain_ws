#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher odom_pub;

double x_offset = 0.22;
double y_offset = 0.0;
double z_offset = 0.54; //+0.25 from ground


void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_in)
{
	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = odom_in->header.stamp;
	odom_trans.header.frame_id = "map";
	odom_trans.child_frame_id = "base_link";
	
	odom_trans.transform.translation.x = odom_in->pose.pose.position.x - x_offset;
	odom_trans.transform.translation.y = odom_in->pose.pose.position.y - y_offset;
	odom_trans.transform.translation.z =  0.25;
	
	odom_trans.transform.rotation = odom_in->pose.pose.orientation;
	
	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = odom_in->header.stamp;
	odom.header.frame_id = "map";
	
	odom.pose.pose.position.x = odom_in->pose.pose.position.x - x_offset;
	odom.pose.pose.position.y = odom_in->pose.pose.position.y - y_offset;
	odom.pose.pose.position.z = 0.25;
	odom.pose.pose.orientation = odom_in->pose.pose.orientation;

	odom.child_frame_id = "base_link";
	odom.twist = odom_in->twist;

	odom_pub.publish(odom);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle n;
	odom_pub = n.advertise<nav_msgs::Odometry>("state_estimation", 50);
	ros::Subscriber subOdometry = n.subscribe<nav_msgs::Odometry>
				("/camera/odom/sample",5, odometryCallback);
	ros::spin();									
	
	return 0;

}
