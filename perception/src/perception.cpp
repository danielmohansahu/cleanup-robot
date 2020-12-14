#include <perception/perception.h>

namespace cleanup {

Perception::Perception():it_(nh) {
	depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,  &Perception::depthCallback, this);
	boudingBoxesSubcriber_ = nh.subscribe("publishers/bounding_boxes/topic", 1, &Perception::boundingBoxesCallback, this);
	objectLocation = nh.advertise<geometry_msgs::PoseStamped>("objectLocationData",1);
}

std::vector<int> Perception::detectObjects() {

}

// geometry_msgs::PoseStamped Perception::getObjectPose() {

// }

void cleanup::Perception::getpose(int& u, int& v) {
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

	pose = mf.req_multipy(intrinsic_i, im_cood, translation, rotation);
}

void Perception::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxes) {
	for(box:bboxes) {
		bbox.xmin = box.xmin;
        bbox.xmax = box.xmax;
        bbox.ymin = box.ymin;
        bbox.ymax = box.ymax;

        if(bbox.xmin!=0 && bbox.xmax!=0) {
        	x_center = (bbox.xmax+bbox.xmin)/2;
        	y_center=(bbox.ymax+bbox.ymin)/2;
        	getpose(x_center, y_center);
        }
        
	}
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
