#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <iostream>

using namespace std;

double thetaTemp = 0;
double kd =0.25;
double kv =.75;
double PI = 3.14159265;
double x,y,theta,thetaD,thetaP;
ros::Publisher pubAngle;

void poseMessageReceived(const turtlesim::Pose& msg){
	x = msg.x;
	y = msg.y;
	thetaP=msg.theta;
	}

void calculations(const std_msgs::Float32MultiArray msg){
		vector<float> data(4);
		data=msg.data;
		//ROS_INFO_STREAM("Object is located at: <" <<data[0] <<"," <<data[1] <<"> ");
		//Calculate x and y 
		double xDes,yDes;
		xDes = (11.09/data[2])*data[0];
		yDes = -(11.09/data[3])*data[1]+11.09;
		//Calculate target angle relative to current turle location
		theta = atan2((yDes-y),(xDes-x));
		//Calculate desired angular velocity based on previous velocity
		thetaD = theta-thetaP;
		if(thetaD>PI)
			thetaD-=2*PI;
		if(thetaD<=-PI)
			thetaD+=2*PI;
		thetaTemp = thetaD;
		//Calculate the destance used to determine speed of turtle
		double dist = sqrt((xDes-x)*(xDes-x)+(yDes-y)*(yDes-y));
		//ROS_INFO_STREAM("Target position: <" <<x <<"," <<y <<"> Direction to take: " <<theta);
		geometry_msgs::Twist msgP;
		msgP.linear.x = kd*dist;
		msgP.angular.z = kv*thetaD;
		pubAngle.publish(msgP);
}

int main(int argc, char **argv){
	ros::init(argc, argv,"calculate_Angle");
	ros::NodeHandle nh;
	ros::Subscriber subCenter = nh.subscribe("std_msgs/Float32MultiArray", 1000, calculations);
	ros::Subscriber subDirection = nh.subscribe("turtle1/pose",1000,&poseMessageReceived);
	pubAngle = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	ros::Rate rate(10);
		ros::spin();
}
