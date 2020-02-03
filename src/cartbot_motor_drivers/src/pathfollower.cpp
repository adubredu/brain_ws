#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 0.8;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 5.0;

float joySpeed = 0;
float joyYaw = 0;

float robotX = 0;
float robotY = 0;
float robotZ = 0.5;
float robotRoll = 0;
float robotPitch = 0;
float robotYaw = 0;

float robotXRec = 0;
float robotYRec = 0;
float robotZRec = 0.5;
float robotRollRec = 0;
float robotPitchRec = 0;
float robotYawRec = 0;

float robotYawRate = 0;
float robotSpeed = 0;

double odomTime = 0;
double joyTime = 0;
int pathPointID = 0;
int pathInit = false;
bool navFwd = true;
double switchTime = 0;
double real_speed = 1.0;

double goalX = 0, goalY = 0;
double tolerance = 0.5;
bool wp_backward = false;

nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  robotRoll = roll;
  robotPitch = pitch;
  robotYaw = yaw;
  robotX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  robotY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  robotZ = odomIn->pose.pose.position.z;
}


void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joySpeed = sqrt(pow(joy->axes[2],2) + pow(joy->axes[3],2));
    if (joy->axes[3] == 0) joySpeed = 0;
    if (joy->axes[3] < 0) joySpeed*=-1;
  joyYaw = joy->axes[2];
  autonomyMode = false;
}

void waypointHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

bool goalReached()
{
  double distance = sqrt(pow((robotX - goalX),2) + pow((robotY - goalY),2));
  if (distance < tolerance)
    return true;

  return false;
}


bool waypoint_forward()
{
  bool waypoint_forward = true;
  double pointX = goalX - robotX;
  double pointY = goalY - robotY;
  double x = pointX * cos(robotYaw) + pointY * sin(robotYaw);
  double y = -pointX * sin(robotYaw) + pointY * cos(robotYaw);

  double ang = atan2(x,y);
  if (ang > 0){
    waypoint_forward = true;
  }
  else if (ang < 0){
    waypoint_forward = false;
  }

  return waypoint_forward;
}


void autonomyMode_activate(const std_msgs::Bool::ConstPtr& data)
{
  if (data->data)
  {
    autonomyMode = true;
    autonomySpeed = 1.0;
  }
  else
  {
    autonomyMode = false;
    autonomySpeed = 0;
  }
}

void backwardHandler(const std_msgs::Bool::ConstPtr& data)
{
  wp_backward = data->data;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/local_waypoint", 5, waypointHandler);
  ros::Subscriber subAutonomy = nh.subscribe<std_msgs::Bool> ("/activate_autonomy", 1, autonomyMode_activate);
  ros::Subscriber subDirection = nh.subscribe<std_msgs::Bool> ("/wp_backward", 1, backwardHandler);
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
  geometry_msgs::Twist cmd_spd;

  ros::Rate rate(100);
  bool status = ros::ok();

  while(status)
  {
  	ros::spinOnce();
    if (autonomyMode)
    {
      float pathDir = atan2((goalY - robotY), (goalX - robotX));
      float dirDiff = robotYaw - pathDir;
      if (dirDiff > PI) dirDiff -= 2*PI;
      else if (dirDiff < -PI) dirDiff += 2*PI;

      double joySpeed2 = maxSpeed * autonomySpeed;

      robotYawRate = dirDiff;
      if (robotYawRate > maxYawRate) 
        robotYawRate = maxYawRate;
      else if (robotYawRate < -maxYawRate) 
        robotYawRate = -maxYawRate;

      robotYawRate *= autonomySpeed;

      if (robotSpeed < joySpeed2) robotSpeed += maxAccel / 100.0;
      else if (robotSpeed > joySpeed2) robotSpeed -= maxAccel / 100.0;

      if (not waypoint_forward())
        robotSpeed = 0;

      if (wp_backward)
      {
        robotSpeed = 0;
        robotYawRate = 1;
        ROS_INFO("Turning");
      }

      if (goalReached())
      {
          robotSpeed = 0;
          robotYawRate = 0;
      }

    }

    else
    {
      float joySpeed2 = maxSpeed * joySpeed;
      robotYawRate = joyYaw;

       if (joySpeed != 0)
       {
         if (robotSpeed < joySpeed2) robotSpeed += maxAccel / 100.0;
         else if (robotSpeed > joySpeed2) robotSpeed -= maxAccel / 100.0;
       }

       else if (joySpeed == 0)
         robotSpeed= 0;
      
        if (joyYaw == 0)
          robotYawRate = 0;
    }
  	
 	 cmd_spd.linear.x = robotSpeed;
 	 cmd_spd.angular.z = robotYawRate;
 	 pubSpeed.publish(cmd_spd);

 	 status = ros::ok();
 	 rate.sleep();
 	}

 	return 0;
 }



