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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummyCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);



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

    pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;
    cloud_filtered->clear();

/*
    //pass through filter
    pass_through_filter.setInputCloud (plannerCloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(-0.7,0.3);
    pass_through_filter.filter (*cloud_pass);

    // Publish the data
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg(*cloud_pass, pcloud);
    pcloud.header.frame_id = "/map";
    to_0_3_pub.publish(pcloud);


    // to 1.0
    pcl::PassThrough<pcl::PointXYZRGB> pass1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass1(new pcl::PointCloud<pcl::PointXYZRGB>);

    //pass through filter
    pass1.setInputCloud (plannerCloud);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(-0.7,1.0);
    pass1.filter (*cloud_pass1);

    // Publish the data
    sensor_msgs::PointCloud2 pcloud1;
    pcl::toROSMsg(*cloud_pass1, pcloud1);
    pcloud1.header.frame_id = "/map";
    // pubPassthrough.publish(pcloud);
    to_1_0_pub.publish(pcloud1);





    // to 2.5
    pcl::PassThrough<pcl::PointXYZRGB> pass2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass2(new pcl::PointCloud<pcl::PointXYZRGB>);

    //pass through filter
    pass2.setInputCloud (plannerCloud);
    pass2.setFilterFieldName("z");
    pass2.setFilterLimits(-0.7,2.5);
    pass2.filter (*cloud_pass2);

    // Publish the data
    sensor_msgs::PointCloud2 pcloud2;
    pcl::toROSMsg(*cloud_pass2, pcloud2);
    pcloud2.header.frame_id = "/map";
    // pubPassthrough.publish(pcloud);
    to_2_5_pub.publish(pcloud2);




    // to 3.0
    pcl::PassThrough<pcl::PointXYZRGB> pass3;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass3(new pcl::PointCloud<pcl::PointXYZRGB>);

    //pass through filter
    pass3.setInputCloud (plannerCloud);
    pass3.setFilterFieldName("z");
    pass3.setFilterLimits(-0.7,3.0);
    pass3.filter (*cloud_pass3);

    // Publish the data
    sensor_msgs::PointCloud2 pcloud3;
    pcl::toROSMsg(*cloud_pass3, pcloud3);
    pcloud3.header.frame_id = "/map";
    // pubPassthrough.publish(pcloud);
    to_3_0_pub.publish(pcloud3);

*/


    // full
   

    // Publish the data
    sensor_msgs::PointCloud2 pcloudf;
    pcl::toROSMsg(*plannerCloud, pcloudf);
    pcloudf.header.frame_id = "/map";
    // pubPassthrough.publish(pcloud);
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

    // sensor_msgs::PointCloud2 pcloud;
    // pcl::toROSMsg(*plannerCloud, pcloud);
    // pcloud.header.frame_id = "/map";
    // pubPlannerCloud.publish(pcloud);

    passthrough_and_voxel();
    // sensor_msgs::PointCloud2 pcloud;
    // pcl::toROSMsg(*plannerCloud, pcloud);
    // pcloud.header.frame_id = "/map";
    // pubPassthrough.publish(pcloud);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
    laserCloud->clear();
    dummyCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *dummyCloud);
    pcl::PointXYZRGB point;

    int size = dummyCloud->points.size();
    for (int i=0; i<size; i++)
    {
    	point.x = dummyCloud->points[i].z;
        point.y = dummyCloud->points[i].x;
        point.z = dummyCloud->points[i].y;
        laserCloud->push_back(point);
    } 
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
    // ros::Subscriber savesub = nh.subscribe ("/save_map", 1, saveMapCallback);
	to_0_3_pub = nh.advertise<sensor_msgs::PointCloud2> ("/0_1_0_3_map", 1);
	to_1_0_pub = nh.advertise<sensor_msgs::PointCloud2> ("/0_1_1_0_map", 1);
	to_2_5_pub = nh.advertise<sensor_msgs::PointCloud2> ("/0_1_2_5_map", 1);
	to_3_0_pub = nh.advertise<sensor_msgs::PointCloud2> ("/0_1_3_0_map", 1);
	full_pub = nh.advertise<sensor_msgs::PointCloud2> ("/full_map", 1);
    // pubPlannerCloud = nh.advertise<sensor_msgs::PointCloud2> ("/plannerCloud", 1);


	// Spin
	ros::spin ();
}

