#include <stdio.h>
#include "ros/ros.h"
#include <stdlib.h>
#include "wiringPi.h"
//#include <wiringPi.h>
#include <stdbool.h>
#include <pthread.h>
#include "sensor_msgs/Range.h"
#include <unistd.h>
#include <sys/types.h>
 
// frontL
#define TRIG_0 5
#define ECHO_0 6

// right
#define TRIG_1 2
#define ECHO_1 3

// back
#define TRIG_2 28
#define ECHO_2 29

// left
#define TRIG_3 0
#define ECHO_3 1

// frontR
#define TRIG_4 8
#define ECHO_4 9

int i = 0;
bool leave = false;
long startTime;
long travelTime;

long valid0;
long valid1;
long valid2;
long valid3;
long valid4;

double distance;

pid_t pid;

double getCM(int TRIG, int ECHO, int sensor) {
        //Send trig pulse

		//pinMode(TRIG, OUTPUT);
        //pinMode(ECHO, INPUT);

        digitalWrite(TRIG, HIGH);
        delayMicroseconds(900);
        digitalWrite(TRIG, LOW);

		i = 0;
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW && ros::ok()){
 
        //Wait for echo end
			startTime = micros();	

			++i;
			//printf("%d\n", i);
	
			if (digitalRead(ECHO) == HIGH) break;

			if (digitalRead(ECHO) == LOW && i == 100000) {

				//printf("Timed out\n\n\n\n\n\n");
				i = 0;

				switch(sensor){

				case 0: return valid0;
					break;
				case 1: return valid1;
					break;
				case 2: return valid2;
					break;
				case 3: return valid3;
					break;
				case 4: return valid4;
					break;
				}
			}	

		}

		i = 0;

        while(digitalRead(ECHO) == HIGH && ros::ok()){

        	travelTime = micros() - startTime;

			++i;
			//printf("%d\n", i);
	
			if (digitalRead(ECHO) == LOW) break;

			if (digitalRead(ECHO) == HIGH && i == 100000) {

				//printf("Timed out\n\n\n\n\n\n");
				i = 0;

				switch(sensor){

				case 0: return valid0;
					break;
				case 1: return valid1;
					break;
				case 2: return valid2;
					break;
				case 3: return valid3;
					break;
				case 4: return valid4;
					break;
				}
			}	

		}
 
		double distance = travelTime / 58;

		if(distance != 0){

			switch(sensor){

				case 0: valid0 = distance;
					break;
				case 1: valid1 = distance;
					break;
				case 2: valid2 = distance;
					break;
				case 3: valid3 = distance;
					break;
				case 4: valid3 = distance;
					break;
			}
		}

        return distance;
}

int main(int argc, char** argv) {
        //setup();

	ros::init(argc, argv, "sensors");
	ros::NodeHandle nh;

	ros::Publisher pub0 = nh.advertise<sensor_msgs::Range>("sonar_frontR_distance", 10);
	ros::Publisher pub1 = nh.advertise<sensor_msgs::Range>("sonar_right_distance", 10);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::Range>("sonar_left_distance", 10);
	ros::Publisher pub3 = nh.advertise<sensor_msgs::Range>("sonar_back_distance", 10);
	ros::Publisher pub4 = nh.advertise<sensor_msgs::Range>("sonar_frontL_distance", 10);

	sensor_msgs::Range msg0;
	sensor_msgs::Range msg1;
	sensor_msgs::Range msg2;
	sensor_msgs::Range msg3;
	sensor_msgs::Range msg4;

	//printf("pub\n");

	wiringPiSetup();

	pinMode(TRIG_0, OUTPUT);
        pinMode(ECHO_0, INPUT);
        //TRIG pin must start LOW
        digitalWrite(TRIG_0, LOW);

	pinMode(TRIG_1, OUTPUT);
        pinMode(ECHO_1, INPUT);
        digitalWrite(TRIG_1, LOW);

	pinMode(TRIG_2, OUTPUT);
        pinMode(ECHO_2, INPUT);
        digitalWrite(TRIG_2, LOW);

	pinMode(TRIG_3, OUTPUT);
        pinMode(ECHO_3, INPUT);
        digitalWrite(TRIG_3, LOW);

	pinMode(TRIG_4, OUTPUT);
        pinMode(ECHO_4, INPUT);
        digitalWrite(TRIG_4, LOW);

	printf("pinmode\n");
	delay(30);

	double sonar_frontR_distance = 0; //4
	double sonar_frontL_distance = 0; //0
	double sonar_right_distance = 0; //1
	double sonar_back_distance = 0; //2
	double sonar_left_distance = 0; //3
 	
	while(1 && ros::ok())
	{
		sonar_frontL_distance = getCM(TRIG_0, ECHO_0, 4);
        	printf("Distance frontL: %fcm\n", sonar_frontL_distance);

		sonar_right_distance = getCM(TRIG_1, ECHO_1, 1);
        	printf("Distance right: %fcm\n", sonar_right_distance);
		msg1.range = sonar_right_distance;
		pub1.publish(msg1);

		sonar_back_distance = getCM(TRIG_2, ECHO_2, 3);
        	printf("Distance back: %fcm\n", sonar_back_distance);
		msg3.range = sonar_back_distance;
		pub3.publish(msg3);

		sonar_left_distance = getCM(TRIG_3, ECHO_3, 2);
        	printf("Distance left: %fcm\n", sonar_left_distance);
		msg2.range = sonar_left_distance;
		pub2.publish(msg2);

		sonar_frontR_distance = getCM(TRIG_4, ECHO_4, 0);
        	printf("Distance frontR: %fcm\n", sonar_frontR_distance);
		msg0.range = sonar_frontR_distance;
		pub0.publish(msg0);

	}
 
        return 0;
}

