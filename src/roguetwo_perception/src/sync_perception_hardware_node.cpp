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
#include <string> 
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>

image_transport::Publisher camL_pub;
image_transport::Publisher camR_pub;

cv::VideoCapture camL_stream(1);
cv::VideoCapture camR_stream(2);

void capture_camL()
{
	cv::Mat camL_frame;
	camL_stream.read(camL_frame);
	//ostr_l << left_count;
	//std::string left_file = "/home/ryan/stereo_images/left/left_" + boost::lexical_cast<std::string>(left_count) + ".png";
	//cv::imwrite(left_file, camL_frame);
	//left_count++;

	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	header.frame_id = "camera_left_stereo";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", camL_frame).toImageMsg();
	camL_pub.publish(msg);
}

void capture_camR()
{
	cv::Mat camR_frame;
	camR_stream.read(camR_frame);
	//ostr_r << right_count;
	//std::string right_file = "/home/ryan/stereo_images/right/right_" + boost::lexical_cast<std::string>(right_count) + ".png";
	//cv::imwrite(right_file, camR_frame);
	//right_count++;

	std_msgs::Header header = std_msgs::Header();
	header.stamp = ros::Time::now();
	header.frame_id = "camera_right_stereo";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", camR_frame).toImageMsg();
	camR_pub.publish(msg);
}

void capture()
{
	cv::Mat camR_frame;
	cv::Mat camR_frame_gray;
	camR_stream.read(camR_frame);
	cv::cvtColor(camR_frame, camR_frame_gray, cv::COLOR_BGR2GRAY);

	std_msgs::Header header1 = std_msgs::Header();
	header1.stamp = ros::Time::now();
	header1.frame_id = "camera_right_stereo";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header1, "mono8", camR_frame_gray).toImageMsg();
	//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header1, "bgr8", camR_frame).toImageMsg();
	camR_pub.publish(msg);

	cv::Mat camL_frame;
	cv::Mat camL_frame_gray;
	camL_stream.read(camL_frame);
	cv::cvtColor(camL_frame, camL_frame_gray, cv::COLOR_BGR2GRAY);

	std_msgs::Header header2 = std_msgs::Header();
	header2.stamp = header1.stamp;
	header2.frame_id = "camera_left_stereo";
	msg = cv_bridge::CvImage(header2, "mono8", camL_frame_gray).toImageMsg();
	//msg = cv_bridge::CvImage(header2, "bgr8", camL_frame).toImageMsg();

	camL_pub.publish(msg);
}

void capture_imu()
{
	std::cout << "IMU thread" << std::endl;

}

void collect_perception_hardware(const ros::TimerEvent& event)
{
	capture();
	// boost::thread thread1(&capture_camL);
	// boost::thread thread2(&capture_camR);
	// //boost::thread thread3(&capture_imu);

	// thread1.join();
	// thread2.join();
	//thread3.join();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sync_perception_hardware");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	camL_pub = it.advertise("/sync/camera_left/image_raw", 1);
	camR_pub = it.advertise("/sync/camera_right/image_raw", 1);

	ros::Timer timer = nh.createTimer(ros::Duration(0.03333),
		collect_perception_hardware);


	ros::spin();

	return 0;
}
