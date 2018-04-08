// #include "markers_to_pcl_node.hpp"

// MarkersToPCLNode::MarkersToPCLNode(int argc, char** argv)
// {
// 	ros::init(argc, argv, "markers_to_pcl");
// 	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
// 	markers_sub = n.subscribe("/svo/points_array", 10, MarkersToPCLNode::markers_to_pcl);
// }

// void MarkersToPCLNode::markers_to_pcl(const visualization_msgs::MarkerArray marker_array)
// {
// 	std::cout << "Here1" << std::endl;
// 	MarkersToPCLNode::publish_pcl();
// }

// void MarkersToPCLNode::publish_pcl()
// {
// 	std::cout << "Here2" << std::endl;
// }


// int main(int argc, char** argv)
// {
// 	// setup up node, publisher, and subscriber
// 	MarkersToPCLNode markers_to_pcl = MarkersToPCLNode(argc, argv);
// }

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pcl_pub;
bool use_imu;

void markers_cb(const visualization_msgs::Marker& marker)
{
	std::vector<geometry_msgs::Point> points;
	sensor_msgs::PointCloud2 pcl_msg;
	PointCloud::Ptr pcl (new PointCloud);
	for (std::vector<int>::size_type i = 0; i != marker.points.size(); i++)
	{
		pcl::PointXYZ point;
		//std::cout << "Use IMU: " << use_imu << std::endl;
		//if (use_imu)
		//point = pcl::PointXYZ(marker.points[i].x, marker.points[i].y, marker.points[i].z);
		//td::cout << "X: " << marker.points[i].x << " Y: " << marker.points[i].y << " Z: " << marker.points[i].z << std::endl;
		//else
		point = pcl::PointXYZ(marker.points[i].z, -1*marker.points[i].x, -1*marker.points[i].y);
		
		pcl->points.push_back(point);
	}
	//pcl.sensor_origin_ = 
	//std::cout << pcl << std::endl
	pcl::toROSMsg(*pcl, pcl_msg);
	std_msgs::Header pcl_header;
	pcl_header.frame_id = "world";
	pcl_msg.header = pcl_header;
	pcl_msg.is_dense = false;
	pcl_pub.publish(pcl_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "markers_to_pcl");
	ros::NodeHandle nh;
	nh.getParam("use_imu", use_imu);

	ros::Subscriber sub = nh.subscribe("/svo/points", 1, markers_cb);

	pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);

	ros::spin();
}