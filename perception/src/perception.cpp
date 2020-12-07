#include <perception/perception.h>
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
	objectFound = nh.advertise<std_msgs::Int8>("found_object", 1);
    bboxes = nh.advertise<darknet_ros::bbox_array>("YOLO_bboxes", 1);

	cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
}

geometry_msgs::PoseStamped Perception::getObjectPose() {

}

Perception::~Perception() {
	cv::destroyWindow(OPENCV_WINDOW);
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

void Perception::drawBBoxes(cv::Mat &input_frame, std::vector<cv::Rect> &classBboxes,\
        int &classObjCount, cv::Scalar &bboxColor, const std::string &classLabels) {
	darknet_ros::bbox bbox_result;

    for (int i = 0; i < classObjCount; i++) {
    	int xmin = (classBboxes[i].x - classBboxes[i].width/2)*frameWidth;
    	int ymin = (classBboxes[i].y - classBboxes[i].height/2)*frameHeight;
    	int xmax = (classBboxes[i].x + classBboxes[i].width/2)*frameWidth;
    	int ymax = (classBboxes[i].y + classBboxes[i].height/2)*frameHeight;

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
	std::vector< std::vector<cv::Rect> > classBboxes(numClasses);
    std::vector<int> classObjCount(numClasses, 0);

	// Run Yolo and get prediction boxes
    auto pred_boxes = od.predict(inputFrame);

    int num = od.getObjectCount();

	// Draw the boxes
	if (num > 0  && num <= 100) {
		ROS_INFO_STREAM("objects:"<< num);

		// split bounding boxes by class
		for (int i = 0; i < num; i++) {
			for (int j = 0; j < numClasses; j++) {
				//
				classBboxes[j].push_back(_boxes[i]);
				classObjCount[j]++;
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