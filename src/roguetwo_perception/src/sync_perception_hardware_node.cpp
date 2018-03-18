#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>

image_transport::Publisher camL_pub;
image_transport::Publisher camR_pub;

cv::VideoCapture camL_stream(1);
//cv::VideoCapture camR_stream(1);

void capture_camL()
{
	std::cout << "CAM L thread" << std::endl;
	cv::Mat camL_frame;
	camL_stream.read(camL_frame);

	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	header.frame_id = "camera_left_stereo";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", camL_frame).toImageMsg();
	camL_pub.publish(msg);
}

void capture_camR()
{
	std::cout << "CAM R thread" << std::endl;
	cv::Mat camR_frame;
	camL_stream.read(camR_frame);
	
	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	header.frame_id = "camera_right_stereo";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", camR_frame).toImageMsg();
	camR_pub.publish(msg);
}

void capture_imu()
{
	std::cout << "IMU thread" << std::endl;

}

void collect_perception_hardware(const ros::TimerEvent& event)
{
	boost::thread thread1(&capture_camL);
	boost::thread thread2(&capture_camR);
	boost::thread thread3(&capture_imu);

	thread1.join();
	thread2.join();
	thread3.join();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sync_perception_hardware");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	camL_pub = it.advertise("/sync/image_left", 1);
	camR_pub = it.advertise("/sync/image_right", 1);

	ros::Timer timer = nh.createTimer(ros::Duration(0.0333), 
		collect_perception_hardware);


	ros::spin();

	return 0;
}