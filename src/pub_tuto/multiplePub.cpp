#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>

int main(int argc, char **argv){
	
	ros::init(argc, argv,"publish_int_arduino");
	ros::NodeHandle nh;
	
	ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("std_msgs/Int32",1000);
	ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("std_msgs/Int32",1000);
	srand(time(0));
	
	ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = 2*double(rand())/double(RAND_MAX);
		msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
		
		pub.publish(msg);
		ROS_INFO_STREAM("Sending random velocity command:"<<" linear="<<msg.linear.x <<" angular="<<msg.angular.z);
		rate.sleep();
		}

}
