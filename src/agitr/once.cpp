#include <ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"log_once");
	ros::NodeHandle nh;
	while(ros::ok()){
		ROS_DEBUG_STREAM_ONCE("THIS APPEARS ONLY ONCE. ");
		ROS_INFO_STREAM_ONCE("THIS APPEARS ONLY ONCE. ");
		ROS_WARN_STREAM_ONCE("THIS APPEARS ONLY ONCE. ");
		ROS_ERROR_STREAM_ONCE("THIS APPEARS ONLY ONCE. ");
		ROS_FATAL_STREAM_ONCE("THIS APPEARS ONLY ONCE. ");
	}
}
