#include <ros/ros.h>

#include <perception/perception.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "Perception Module");

    Perception detect;
    ros::spin();

	return 0;
}