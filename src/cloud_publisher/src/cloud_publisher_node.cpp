#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudTrans(new pcl::PointCloud<pcl::PointXYZRGB>());
ros::Publisher cloud_pub;
float robotX = 0, robotY = 0, robotZ = 0;
float robotRoll = 0, robotPitch = 0, robotYaw = 0;
float x_offset = 0.22;
float y_offset = 0.0;
float z_offset = 0.54;
double odomtime = 0.0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).
							    getRPY(roll, pitch, yaw);
	robotRoll = roll;
	robotYaw = yaw;
	robotPitch = pitch;
	robotX = odom->pose.pose.position.x;
	robotY = odom->pose.pose.position.y;
	robotZ = odom->pose.pose.position.z;
	
	odomtime = odom->header.stamp.toSec();
}


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
	laserCloud->clear();
	pcl::fromROSMsg(*laserCloud2, *laserCloud);

	pcl::PointXYZRGB point;
	laserCloudTrans->clear();
	int size = laserCloud->size();
	
	float cosYaw = cos(robotYaw);
	float sinYaw = sin(robotYaw);
	
	for (int i=0; i<size; i++)
	{
		point = laserCloud->points[i];
		float px = point.x + x_offset;
		float py = point.y + y_offset;
		float pz = point.z + z_offset;
		
		float rx = px * cosYaw - py * sinYaw;
		float ry = py * sinYaw + px * cosYaw;
		
		point.x = rx + robotX;
		point.y = ry + robotY;
		point.z = pz;
		
		laserCloudTrans->push_back(point);
		
	}
	
	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*laserCloudTrans, cloud_out);
	cloud_out.header.stamp = ros::Time().fromSec(odomtime);
	cloud_out.header.frame_id = "/map";
	cloud_pub.publish(cloud_out);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cloud_publisher");
	ros::NodeHandle n;

	ros::Subscriber subOdometry = n.subscribe<nav_msgs::Odometry>
				("state_estimation", 5, odometryCallback);
	ros::Subscriber subCloud = n.subscribe<sensor_msgs::PointCloud2>
				("/camera/depth_registered/points", 5, cloudCallback);
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("stark_cloud_registered", 5);

	ros::spin();
	return 0;
}
