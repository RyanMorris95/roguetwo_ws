#ifndef MARKERS_TO_PCL_NODE_H
#define MARKERS_TO_PCL_NODE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

class MarkersToPCLNode
{
public:
	MarkersToPCLNode(int argc, char**argv);
	~MarkersToPCLNode();
	void markers_to_pcl(const visualization_msgs::MarkerArray marker_array);
	void publish_pcl();

private:
	ros::NodeHandle n;
	ros::Publisher pcl_pub;
	ros::Subscriber markers_sub;
};

#endif