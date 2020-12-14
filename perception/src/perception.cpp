#include <perception/perception.h>
// #include <darknet_ros/YoloObjectDetector.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <perception/matrixf.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>

namespace cleanup {

Perception::Perception():it_(nh) {
	depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,  &Perception::depthCallback, this);
	boudingBoxesSubcriber_ = nh.subscribe("publishers/bounding_boxes/topic", 1, &Perception::boundingBoxesCallback, this);
	objectCountSubcriber_ = nh.subscribe("publishers/object_detector/topic", 1, &Perception::objectCountCallback, this);
	objectLocation = nh.advertise<perception::ObjectLocations>("objectLocationData",1);
}

// std::vector<int> Perception::detectObjects() {

// }

// geometry_msgs::PoseStamped Perception::getObjectPose() {

// }

std::vector<std::vector<double>> Perception::getpose(const double& u, const double& v) {
	std::vector<std::vector<double>> intrinsic {
		{463.889, 0.0, 320.0},
		{0.0, 463.889, 240.0},
		{0.0, 0.0, 1.0}
	};
	std::vector<std::vector<double>> intrinsic_i {
		{0.00215, 0.0, -0.6898},
		{0.0, 0.00215, -0.51736},
		{0.0, 0.0, 1.0}
	};
	
	std::vector<std::vector<double>> im_cood {
		{u}, {v}, {1}
	};
	std::vector<std::vector<double>> translation {
		{1}, {1}, {1}
	};
	std::vector<std::vector<double>> rotation {
		{1,0,0}, {0,1,0}, {0,0,1}
	};

	pose = mf.req_multiply(intrinsic_i, im_cood, translation, rotation);

	return pose;
}

void Perception::objectCountCallback(const std_msgs::Int8& msg) {
	objectCount = msg;
}

void Perception::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxes) {
	for(int i=0;i<sizeof(bboxes.bounding_boxes);i++) {
		bbox.xmin = bboxes.bounding_boxes[i].xmin;
        bbox.xmax = bboxes.bounding_boxes[i].xmax;
        bbox.ymin = bboxes.bounding_boxes[i].ymin;
        bbox.ymax = bboxes.bounding_boxes[i].ymax;

        if(bbox.xmin!=0 && bbox.xmax!=0) {
        	int x_center = (bbox.xmax+bbox.xmin)/2;
        	int y_center=(bbox.ymax+bbox.ymin)/2;
        	u = x_center;
        	v = y_center;

        	objL.id = bboxes.bounding_boxes[i].id;
        	objL.p = getpose(u, v);
        	objL.d = depth;

        	location_array.push_back(objL);
        }
	}
	// @TODO Santosh need to convert from object_loc to perception/ObjectLocation
	// objectLocation.publish(location_array);
}

void Perception::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
	cv_bridge::CvImagePtr cv_depth_ptr;

	if ("16UC1" == depth_msg->encoding) {
		try {
		    cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
		}
		catch (cv_bridge::Exception& e) {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    // return false;
		}
    }
	else if ("32FC1" == depth_msg->encoding) {
	    try {
	        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	    }
	    catch (cv_bridge::Exception& e) {
	        ROS_ERROR("cv_bridge exception: %s", e.what());
	        // return false;
	    }
	}
    depth = cv_depth_ptr->image.at<short int>(cv::Point(u,v));
}

void Perception::runVisionAlgo() {

}

} // namespace cleanup