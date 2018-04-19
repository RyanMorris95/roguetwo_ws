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
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h" 
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

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
bool start = false;
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

// void coords(const tf2_msgs::TFMessage::ConstPtr& msg){

// 	test = msg->transforms[0].header.frame_id;

// 	if (test.compare(robot) == 0){

// 		X = msg->transforms[0].transform.translation.x;
// 		Y = msg->transforms[0].transform.translation.y;

// 		//qX = msg->transforms[0].transform.rotation.x;
// 		//qY = msg->transforms[0].transform.rotation.y;
// 		//qZ = msg->transforms[0].transform.rotation.z;
// 		//qW = msg->transforms[0].transform.rotation.w;

// 		//angle = atan2(destY - Y, destX - X);

// 		Yaw = tf::getYaw(msg->transforms[0].transform.rotation);
// 		//printf("Yaw: %f\n", Yaw);

// 		//angle = roundf(10 * angle) / 10;
// 		Yaw = roundf(100 * Yaw) / 100;
// 		//printf("Yaw: %f\n", Yaw);
// 		//printf("angle: %f\n", angle);

// 		X = roundf(10 * X) / 10;
// 		Y = roundf(10 * Y) / 10;

// 		//face = Yaw == angle;

// 		if (destX == X && destY == Y) home = true;

// 		//printf("GOTO: %f, CURRENT: %f\n", angle, Yaw);
// 	}
// }

void coords(const nav_msgs::Odometry::ConstPtr& msg)
{
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

void start_autonomous(const std_msgs::Bool::ConstPtr& msg)
{
	start = true;
}

int main(int argc, char** argv){

	int in;
	int forward = 0;
	int side = 0;

	//double destX = 0.0;
	//double destY = 0.0;

	ros::init(argc, argv, "bruteforce");	

	ros::NodeHandle nh;

	//ros::Subscriber place = nh.subscribe("tf", 10, coords);
	ros::Subscriber place = nh.subscribe("odometry/filtered", 10, coords);
	ros::Subscriber sensor4 = nh.subscribe("sonar_frontR_distance", 10, sens4);
	ros::Subscriber sensor0 = nh.subscribe("sonar_frontL_distance", 10, sens0);
	ros::Subscriber sensor1 = nh.subscribe("sonar_right_distance", 10, sens1);
	ros::Subscriber sensor2 = nh.subscribe("sonar_back_distance", 10, sens2);
	ros::Subscriber sensor3 = nh.subscribe("sonar_left_distance", 10, sens3); 
	ros::Subscriber start_autonomous_sub = nh.subscribe("start_autonomous", 1, start_autonomous);
	ros::spinOnce();
	//ros::spin();

	ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 10);
	ackermann_msgs::AckermannDrive msg;

	sleep(1);			//asefkjlha srklgdlrgjklearg i hate my life

	//ros::Rate rate(10);

	if (start)
	{
		while(ros::ok()){

			ros::spinOnce();

			if (initial){				//fix this, needs to relate to destination, not 0

				angle = atan2(destY - Y, destX - X);		//for destX and destY

				//angle = Yaw;								//testing going forward

				//angle = M_PI;								//North
				//angle = 0;									//South
				
				angle = roundf(100 * angle) / 100;

				//goal = angle == 3.14 || angle == 0;	//1 = Face Horizontal, 0 = Face vertical
				
				//msg.steering_angle = 0.0;

				//if (goal) msg.steering_angle = X <= destX ? -0.30 : 0.30;

				//else if (!goal) msg.steering_angle = Y <= destY ? -0.30 : 0.30;
				

				msg.speed = 1;
				pub.publish(msg);
				//sleep(1);
				initial = false;
			}

			//if (finish) angle = atan2(destY - Y, destX - X);

			angle = atan2(destY - Y, destX - X);	
			angle = roundf(100 * angle) / 100;
				
			if (angle >=0 && Yaw >= 0) direction = Yaw - angle > 0;

			else if (angle < 0 && Yaw < 0) direction = (fabs(Yaw) - fabs(angle)) < 0;

			else if (angle < 0  && Yaw >= 0) direction = (fabs(angle) < (M_PI - Yaw));

			else if (angle >= 0 && Yaw < 0) direction = (angle > (M_PI - fabs(Yaw)));

			ros::spinOnce();


			//printf("%d\n", goal);

			msg.speed = 1;
			pub.publish(msg);

			ros::spinOnce(); 
		
			printf("Yaw: %f\n", Yaw);
			printf("angle: %f\n", angle);
			
			face = Yaw == angle;

			if (fabs(Yaw) == 3.14 && fabs(angle) == 3.14) face = true;
			//printf("face: %d\n", face);

			if (face) { 

				printf("face\n"); 
				msg.steering_angle = 0.0;
				pub.publish(msg);

			}
			/*
			if (!finish && ((goal && (X - 1.5 == destX || X + 1.5 == destX)) || (!goal && (Y - 1.5 == destY || Y + 1.5 == destY)))){

				angle = atan2(destY - Y, destX - X);
				angle = roundf(100 * angle) / 100;

				if (angle >=0 && Yaw >= 0) direction = Yaw - angle > 0;

				else if (angle < 0 && Yaw < 0) direction = (fabs(Yaw) - fabs(angle)) < 0;

				else if (angle < 0  && Yaw >= 0) direction = (fabs(angle) < (M_PI - Yaw));

				else if (angle >= 0 && Yaw < 0) direction = (angle > (M_PI - fabs(Yaw)));

				//printf("d: %d\n", direction);
				//printf("lawgic: %lf\n", fabs(Yaw) - fabs(angle));
				printf("Yaw: %lf\n", Yaw);
				printf("angle: %lf\n", angle);

				finish = true;
				
				msg.steering_angle = direction ? -0.25 : 0.25;
				pub.publish(msg);
				ros::spinOnce();
			}
			*/
			//ros::spinOnce();
			//face = Yaw == angle;

			if (frontL >= 1.1 && frontR >= 1.1 && !face){

				msg.steering_angle = direction ? -0.30 : 0.30;
				msg.speed = 1;
				pub.publish(msg);
				ros::spinOnce();
			}

			if (frontL < 0.4 && frontR < 0.4) {

				msg.speed = -1.0;
				msg.steering_angle = 0.0;
				pub.publish(msg);
				sleep(1);
				ros::spinOnce();
			}

			if (frontL < 1.2 && frontR < 1.2) {

				printf("BOTH\n");
				boxfront = true;
				box = true;
				msg.steering_angle = direction ? -0.55 : 0.55;
				ros::spinOnce();
			}

			else if (frontL < 0.9 && frontR > 0.9){

				boxfront = true;
				offcenter = true;
				direction = 1;
				msg.steering_angle = -0.37;
				ros::spinOnce();
			}

			else if (frontR < 0.9 && frontL > 0.9){

				boxfront = true;
				offcenter = true;
				direction = 0;
				msg.steering_angle = 0.37;
				ros::spinOnce();
			}

			i = 0;

			//ros::Rate rate(10);
			
			while (boxfront && ros::ok()){

				if (direction && left < 0.65 || !direction && right < 0.65 || i == 160000) {

					boxfront = false;
					boxside = true;
					msg.speed = 0;

					msg.steering_angle = direction ? 0.45 : -0.45;
		
					pub.publish(msg);	
					ros::spinOnce();			
					break;
				}

				//++i;
				//printf("%d\n", i);

				++i;

				//printf("%d\n", direction);
				pub.publish(msg);
				ros::spinOnce();

				//rate.sleep();
			}

			i = 0;

			while (boxside && ros::ok()){

				if (direction && left > 0.55 || !direction && right > 0.55) {

					boxpass = true;
					boxside = false;
					break;
				}

				ros::Duration(0.1).sleep();
				i++;

				if (i % 3 == 0 && !offcenter){

				msg.steering_angle = msg.steering_angle < 0 ? msg.steering_angle + 0.07 : msg.steering_angle - 0.7;
				}

				else if (i % 4 == 0 && offcenter){

				msg.steering_angle = msg.steering_angle < 0 ? msg.steering_angle + 0.1 : msg.steering_angle - 0.1;
				offcenter = false;
				}
				pub.publish(msg);

				//printf("side\n");

				//if (i == 5)
				//msg.steering_angle = direction ? 0.45 : -0.45;

				msg.speed = 1;
		
				pub.publish(msg);
				ros::spinOnce();
				//++i;

			}
			
		
			while (X == destX && Y == destY && ros::ok()){

				msg.speed = 0;
				pub.publish(msg);
			}


			//rate.sleep();

		}
	}
	return 0;
}	
	

	
