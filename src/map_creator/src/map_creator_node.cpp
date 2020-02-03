#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <std_msgs/Bool.h>

using namespace std;

ros::Publisher pubPassthrough;
ros::Publisher pubPlannerCloud;
ros::Publisher to_0_3_pub;
ros::Publisher	to_1_0_pub;
ros::Publisher	to_2_5_pub;
ros::Publisher	to_3_0_pub;
ros::Publisher	full_pub;
const int laserCloudStackNum = 10000;
int laserCloudCount = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudStack[laserCloudStackNum];



void init_stack()
{
    for (int i = 0; i < laserCloudStackNum; i++) {
        laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  }
}

void passthrough_and_voxel()
{
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;

   
    // Publish the data
    sensor_msgs::PointCloud2 pcloudf;
    pcl::toROSMsg(*plannerCloud, pcloudf);
    pcloudf.header.frame_id = "/map";
    full_pub.publish(pcloudf);


}


void stack_pointcloud()
{
    laserCloudStack[laserCloudCount]->clear();
    *laserCloudStack[laserCloudCount] = *laserCloud;
    laserCloudCount = (laserCloudCount + 1)%laserCloudStackNum;

    plannerCloud->clear();
    for (int i=0; i<laserCloudStackNum; i++)
        *plannerCloud+=*laserCloudStack[i];

    sensor_msgs::PointCloud2 pcloudf;
    pcl::toROSMsg(*plannerCloud, pcloudf);
    pcloudf.header.frame_id = "/map";
    full_pub.publish(pcloudf);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);
    stack_pointcloud();
}

void saveMapCallback(const std_msgs::Bool::ConstPtr& data)
{
    if (data->data)
    {
        int size = plannerCloud->points.size();
        ofstream map ("testMap.ply");
        if (map.is_open()){
        map << "ply\n format ascii 1.0\n";
        map << "element vertex " << size <<endl;
        map << "property float x\n property float y\n property float z\n";
        map << "end_header\n";

        for (int i=0; i<size; i++)
        {
            map << plannerCloud->points[i].x << " " 
            << plannerCloud->points[i].y << " " << plannerCloud->points[i].z <<endl;
            cout << "saving"<<endl;
        }
        
    }
    else cout << "Couldn't open file\n";
    map.close();
        cout << "done saving\n";


    }

}




int main (int argc, char** argv)
{
	
	ros::init (argc, argv, "map_creator");
	ros::NodeHandle nh;
    init_stack();
	ros::Subscriber sub = nh.subscribe ("/stark_cloud_registered", 1, pointCloudCallback);
	full_pub = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);

	ros::spin ();
}

