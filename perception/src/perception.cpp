#include <perception/perception.h>
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
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>

namespace cleanup {

Perception::Perception() : it_(nh) {
	numClasses = od.getnumClasses();
	int count = floor(255/numClasses);
	frameWidth = 320;
	frameHeight = 320;
	classLabels = od.getclassLabels;

	for (int i=0; i < numClasses; i++) {
		bboxColors[i] = cv::Scalar(255 - count*i, 0 + count*i, 255 - count*i);
	}

	image_sub = it_.subscribe("/camera/rgb/image_raw", 1,  &Perception::imageCallback, this);
	dpeth_image_sub = it_.subscribe("/camera/depth/image_raw", 1,  &Perception::depthCallback, this);
	objectFound = nh.advertise<std_msgs::Int8>("found_object", 1);
    bboxes = nh.advertise<darknet_ros::bbox_array>("YOLO_bboxes", 1);

	cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
}

geometry_msgs::PoseStamped Perception::getObjectPose() {

}

Perception::~Perception() {
	cv::destroyWindow(OPENCV_WINDOW);
}

void Perception::getpose() {
	std::vector<std::vector<double>> intrinsic {
		{463.889, 0.0, 320.0},
		{0.0, 463.889, 240.0},
		{0.0, 0.0, 1.0}
	};
	std::vector<std::vector<double>> intrinsic_i;

	intrinsic_i = getInverse(intrinsic);
	
	std::vector<std::vector<double>> im_cood {
		{u}, {v}, {1}
	};
	std::vector<std::vector<double>> translation {
		{1}, {1}, {1}
	};
	std::vector<std::vector<double>> rotation {
		{1,0,0}, {0,1,0}, {0,0,1}
	};

	pose = req_multipy(intrinsic_i, im_cood, translation, rotation);
}

void Perception::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
	cv_bridge::CvImagePtr cv_depth_ptr;

	if ("16UC1" == depth_msg->encoding) {
		try {
		    cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
		}
		catch (cv_bridge::Exception& e) {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return false;
		}
    }
	else if ("32FC1" == depth_msg->encoding) {
	    try {
	        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	    }
	    catch (cv_bridge::Exception& e) {
	        ROS_ERROR("cv_bridge exception: %s", e.what());
	        return false;
	    }
	}
    depth = cv_ptr->image.at<short int>(cv::Point(u,v));
}

void Perception::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr camImage;
    try
    {
      camImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (cam_image) {
    	image_to_detect = camImage->image.clone();
    	runVisionAlgo(image_to_detect);
    }
 
    return;
}

void Perception::drawBBoxes(cv::Mat &input_frame, PredBox &classBboxes,\
        int &classObjCount, cv::Scalar &bboxColor, const std::string &classLabels) {
	darknet_ros::bbox bbox_result;

    for (int i = 0; i < classObjCount; i++) {
    	int xmin = (classBboxes[i].x - classBboxes[i].width/2)*frameWidth;
    	int ymin = (classBboxes[i].y - classBboxes[i].height/2)*frameHeight;
    	int xmax = (classBboxes[i].x + classBboxes[i].width/2)*frameWidth;
    	int ymax = (classBboxes[i].y + classBboxes[i].height/2)*frameHeight;

    	u = (xmin + xmax)/2;
    	v = (ymin + ymax)/2;

    	bbox_result.Class = classLabels;
    	bbox_result.xmin = xmin;
        bbox_result.ymin = ymin;
        bbox_result.xmax = xmax;
        bbox_result.ymax = ymax;
        bboxResults.bboxes.push_back(bbox_result);

        // draw bounding box of first object found
        cv::Point topLeftCorner = cv::Point(xmin, ymin);
        cv::Point botRightCorner = cv::Point(xmax, ymax);
	    cv::rectangle(input_frame, topLeftCorner, botRightCorner, bboxColor, 2);
        cv::putText(input_frame, classLabels, cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,\
        	1.0, bboxColor, 2.0);
    }
}

void Perception::runVisionAlgo(cv::Mat &frame) {
	cv::Mat inputFrame = frame.clone();
	std::vector< std::vector<PredBox> > classBboxes(numClasses);
    std::vector<int> classObjCount(numClasses, 0);

	// Run Yolo and get prediction boxes
    cv::Rect pred_boxes = od.predict(inputFrame);

    int num = od.getObjectCount();

	// Draw the boxes
	if (num > 0  && num <= 100) {
		ROS_INFO_STREAM("objects:"<< num);

		// split bounding boxes by class
		for (int i = 0; i < num; i++) {
			for (int j = 0; j < numClasses; j++) {
				// if (pred_boxes[i].Class == j) {
				classBboxes[j].push_back(pred_boxes[i]);
				classObjCount[j]++;
				// }
            }
        }

	    // Send message that an object has been detected
        std_msgs::Int8 outMsg;
        outMsg.data = 1;
        objectFound.publish(outMsg);

        for (int i = 0; i < numClasses; i++) {
        	if (classObjCount[i] > 0) drawBBoxes(input_frame, classBboxes[i],
				      classObjCount[i], bboxColors[i], classLabels[i]);
        }
        bboxes.publish(bboxResults);
        bboxResults.bboxes.clear();
    }
    else {
    	std_msgs::Int8 outMsg;
        outMsg.data = 0;
        objectFound.publish(outMsg);
    }

    cv::imshow(OPENCV_WINDOW, input_frame);
    cv::waitKey(3);
}

} // namespace cleanup