#include <ros/ros.h>
#include <std_msgs/Bool.h>

void restart_svo()


int main(int argc, char** argv)
{
	ros::init(argc, argv, "svo_monitor");
	ros::NodeHandle nh;

	ros::Subscriber start_autonomous_sub = nh.subscribe("/start_autonomous", 1, restart_svo);

	ros::spin();
}