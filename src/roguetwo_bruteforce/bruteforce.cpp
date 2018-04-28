#include "ros/ros.h"
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include "ackermann_msgs/AckermannDrive.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/Range.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

bool initial = true;
bool direction = true;				
bool turn = true;
bool home = false;
bool box = false;
bool boxfront = false;
bool boxside = false;
bool tempt = false;
bool boxpass = false;
bool face = false;
bool goal;
bool finish = false;
bool offcenter = false;
//bool initialRotation = true;

double X, Y, deltaY, deltaX;
//double qX, qY, qZ, qW;
double Yaw, angle, comp;
double destX = 0.0;					
double destY = 0.0;
double temp;

float frontL = 3.3;
float frontR = 3.3;
float back, left, right;

int pid;
int i = 0;

std::string robot = "base_link";
std::string test;
/*
 void coords(const tf2_msgs::TFMessage::ConstPtr& msg){

 	test = msg->transforms[0].header.frame_id;

 	if (test.compare(robot) == 0){

 		X = msg->transforms[0].transform.translation.x;
		Y = msg->transforms[0].transform.translation.y;

 		//qX = msg->transforms[0].transform.rotation.x;
 		//qY = msg->transforms[0].transform.rotation.y;
 		//qZ = msg->transforms[0].transform.rotation.z;
 		//qW = msg->transforms[0].transform.rotation.w;

 		//angle = atan2(destY - Y, destX - X);

 		Yaw = tf::getYaw(msg->transforms[0].transform.rotation);
 		//printf("Yaw: %f\n", Yaw);

 		//angle = roundf(10 * angle) / 10;
 		Yaw = roundf(100 * Yaw) / 100;
 		//printf("Yaw: %f\n", Yaw);
 		//printf("angle: %f\n", angle);

		X = roundf(10 * X) / 10;
 		Y = roundf(10 * Y) / 10;

 		//face = Yaw == angle;

 		if (destX == X && destY == Y) home = true;

 		//printf("GOTO: %f, CURRENT: %f\n", angle, Yaw);
 	}
 }*/

void coords(const nav_msgs::Odometry::ConstPtr& msg) {
	X = msg->pose.pose.position.x;
	Y = msg->pose.pose.position.y;

	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, Yaw);
}

void sens4(const sensor_msgs::Range::ConstPtr& msg){

	//printf("R\n");
	frontR  = msg->range;
}

void sens0(const sensor_msgs::Range::ConstPtr& msg){

	frontL = msg->range;
	//printf("L: %f\n", frontL);
}

void sens1(const sensor_msgs::Range::ConstPtr& msg){

	right  = msg->range;
}

void sens2(const sensor_msgs::Range::ConstPtr& msg){

	back  = msg->range;
}

void sens3(const sensor_msgs::Range::ConstPtr& msg){

	left  = msg->range;
}

int main(int argc, char** argv){

	int in;
	int forward = 0;
	int side = 0;
	
	ros::init(argc, argv, "bruteforce");

	ros::NodeHandle nh;
	
	ros::Subscriber place = nh.subscribe("/odometry/filtered", 10, coords);
	ros::Subscriber sensor4 = nh.subscribe("lidar_front_right", 10, sens4);
	ros::Subscriber sensor0 = nh.subscribe("lidar_front_left", 10, sens0);
	ros::Subscriber sensor1 = nh.subscribe("lidar_right", 10, sens1);
	//ros::Subscriber sensor2 = nh.subscribe("sonar_back_distance", 10, sens2);
	ros::Subscriber sensor3 = nh.subscribe("lidar_left", 10, sens3);
	
	/*
	ros::Subscriber place = nh.subscribe("tf", 10, coords);
	ros::Subscriber sensor4 = nh.subscribe("sonar_frontR_distance", 10, sens4);
	ros::Subscriber sensor0 = nh.subscribe("sonar_frontL_distance", 10, sens0);
	ros::Subscriber sensor1 = nh.subscribe("sonar_right_distance", 10, sens1);
	ros::Subscriber sensor2 = nh.subscribe("sonar_back_distance", 10, sens2);
	ros::Subscriber sensor3 = nh.subscribe("sonar_left_distance", 10, sens3); 
*/
	ros::spinOnce();
	//ros::spin();

	ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 10);
	ackermann_msgs::AckermannDrive msg;

	sleep(1);			//asefkjlha srklgdlrgjklearg i hate my life

	ros::Rate rate(10);

	while(ros::ok()){

		ros::spinOnce();

		if (initial){				

			angle = atan2(destY - Y, destX - X);

			angle = roundf(100 * angle) / 100;
			
			msg.steering_angle = 0.0;
			

			msg.speed = 0.30;
			pub.publish(msg);
			//sleep(1);
			initial = false;
		}

		angle = atan2(destY - Y, destX - X);
		angle = roundf(100 * angle) / 100;
			
		if (angle >=0 && Yaw >= 0) direction = Yaw - angle > 0;

		else if (angle < 0 && Yaw < 0) direction = (fabs(Yaw) - fabs(angle)) < 0;

		else if (angle < 0  && Yaw >= 0) direction = (fabs(angle) < (M_PI - Yaw));

		else if (angle >= 0 && Yaw < 0) direction = (angle > (M_PI - fabs(Yaw)));

		ros::spinOnce();


		printf("angle: %f      Yaw: %f\n", angle, Yaw);


		msg.speed = 0.30;
		pub.publish(msg);

		ros::spinOnce(); 
	
		if ((Yaw <= 0 && angle <= 0) || (Yaw >= 0 && angle >=0)){

			face = fabs(Yaw) - fabs(angle) <= .04 || fabs(Yaw) - fabs(angle) >= 0.04;
		}

		face = Yaw == angle;

		if (face) { 

			//printf("face\n"); 
			msg.steering_angle = 0;
			pub.publish(msg);
			//msg.steering_angle = msg.steering_angle < 0 ? 0.07 : -0.07;

		}

		if (frontL >= 1.1 && frontR >= 1.1 && !face){

			msg.steering_angle = direction ? -0.30 : 0.30;
			msg.speed = 0.25;
			pub.publish(msg);
			ros::spinOnce();
		}

		if (frontL < 0.4 && frontR < 0.4) {

			msg.speed = -0.25;
			msg.steering_angle = 0.0;
			pub.publish(msg);
			sleep(1);
			ros::spinOnce();
		}

		if (frontL < 1.1 && frontR < 1.1) {

			boxfront = true;
			box = true;
			msg.steering_angle = direction ? -0.53 : 0.53;
			msg.speed = 0.25;
			pub.publish(msg);
			offcenter = false;
			ros::spinOnce();

			printf("\n\n\n\n\n\n	BOX		\n\n\n\n\n\n");
		}

		else if (frontL < 0.8 && frontR > 0.8){

			boxfront = true;
			direction = 1;
			printf("\n\n\n\n\n\nBOX			\n\n\n\n\n\n");
			msg.steering_angle = -0.37;
			pub.publish(msg);
			offcenter = true;
			ros::spinOnce();
		}

		else if (frontR < 0.8 && frontL > 0.8){

			boxfront = true;
			printf("\n\n\n\n\n\n		BOX\n\n\n\n\n\n");
			direction = 0;
			msg.steering_angle = 0.37;
			pub.publish(msg);
			offcenter = true;
			ros::spinOnce();
		}

		i = 0;

		
		while (boxfront && ros::ok()){

			printf("tset1\n");

			if (direction && left < 0.65 || !direction && right < 0.65) {		//  || i == 160000

				boxfront = false;
				boxside = true;
				msg.speed = 0;

				msg.steering_angle = direction ? 0.45 : -0.45;
	
				pub.publish(msg);	
				ros::spinOnce();	
				i = 0;		
				break;
			}
			++i;
			pub.publish(msg);
			ros::spinOnce();

		}

		i = 0;

		while (boxside && ros::ok()){

			printf("tset2\n");

			if (direction && left > 0.65 || !direction && right > 0.65) {

				boxpass = true;
				boxside = false;
				break;
			}

			//printf("side\n");

			//if (i == 5)
			//msg.steering_angle = direction ? 0.45 : -0.45;
			/*
			if (i % 3 == 0 && !offcenter){

				msg.steering_angle = msg.steering_angle < 0 ? msg.steering_angle + 0.07 : msg.steering_angle - 0.7;
			}

			else if (i % 4 == 0 && offcenter){

				msg.steering_angle = msg.steering_angle < 0 ? msg.steering_angle + 0.1 : msg.steering_angle - 0.1;
				offcenter = false;
			}
			*/
			msg.speed = 0.25;
	
			pub.publish(msg);
			ros::spinOnce();
			++i;

			//rate.sleep();
		}
		i = 0;

	
		while (fabs(X) <= 0.5  && fabs(Y) <= 0.5 && ros::ok()){

			printf("tset3\n");

			printf("X: %f    Y: %f", X, Y);

			msg.speed = 0;
			pub.publish(msg);
		}

		rate.sleep();

	}
	return 0;
}	
	

	
