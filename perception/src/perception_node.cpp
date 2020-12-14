#include <ros/ros.h>

#include <perception/perception.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "perception");
	cleanup::Perception od;

	return 0;
}