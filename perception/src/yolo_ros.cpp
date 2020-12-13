#include <perception/yolo_ros.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace cleanup {

cleanup::Yolo::Yolo() {

	/* YOLO initialization */
	confidenceThreshold = 0.5;  // Confidence threshold
	nmsThreshold = 0.4;  // Non-maximum suppression threshold
	frameWidth = 320;  // Width of network's input image
	frameHeight = 320;  // Height of network's input image
	objectCount = 0;

	configPath = "../darknet_ros/yolo_network_config/cfg";  // Add path to the config file
	weightsPath = "../darknet_ros/yolo_network_config/weights";  //Add path to the weight file

	//  Store class labels in the session memory
	classLabelsFile = "../models/class.Labels";
	std::ifstream ifs(classLabelsFile.c_str());
	std::string line;
	while (ifs >> line) {
		classLabels.push_back(line);
	}
	// Load the network
	net = cv::dnn::readNetFromDarknet(configPath, weightsPath);
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

std::vector<cv::String> cleanup::Yolo::getOutputsNames(const cv::dnn::Net& net) {
	static std::vector<cv::String> labels;
	if (labels.empty()) {
		std::vector<int> out_layers = net.getUnconnectedOutLayers();
		std::vector<cv::String> layer_names = net.getLayerNames();
		labels.resize(out_layers.size());
		for (size_t i = 0; i < out_layers.size(); ++i) {
			labels[i] = layer_names[out_layers[i] - 1];
		}
	}
	// std::cout<<labels.size()<<std::endl;
	return labels;
}

std::vector<cv::Rect> cleanup::Yolo::predict(cv::Mat frame) {
	if (frame.empty()) {
		std::cout << "Error reading frame!!!" << std::endl;
	}
	cv::Mat blob;
	cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(frameWidth, frameHeight),\
							cv::Scalar(0,0,0), true, false);
	net.setInput(blob);
	std::vector<cv::Mat> preds;
	net.forward(preds, getOutputsNames(net));
	std::vector<int> class_ids;
	std::vector<float> confidences;
	std::vector<int> indices;
	postProcess(frame, preds, class_ids, confidences, indices);
	return prediction_boxes;
}

void cleanup::Yolo::postProcess(cv::Mat& frame, const std::vector<cv::Mat>& preds, \
	                std::vector<int> class_ids, std::vector<float> confidences,\
	                std::vector<int> indices) {
	for (size_t i = 0; i < preds.size(); ++i) {
		float* data = (float*)preds[i].data;
		for (int j = 0; j < preds[i].rows; ++j, data += preds[i].cols) {
			cv::Mat scores = preds[i].row(j).colRange(5, preds[i].cols);
			cv::Point class_id_point;
			double confidence;
			// Get the value and location of the maximum score
			cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);
			if (confidence > confidenceThreshold) {
				int centerX = (int)(data[0] * frame.cols);
				int centerY = (int)(data[1] * frame.rows);
				int width = (int)(data[2] * frame.cols);
				int height = (int)(data[3] * frame.rows);
				int left = centerX - width / 2;
				int top = centerY - height / 2;
				if(class_id_point.x == 0){
					class_ids.push_back(class_id_point.x);
					confidences.push_back((float)confidence);
					prediction_boxes.push_back(cv::Rect(left, top, width, height));
                    objectCount++;
				}
			}
		}
	}
	cv::dnn::NMSBoxes(prediction_boxes, confidences, confidenceThreshold, nmsThreshold,\
		             indices);
	// for (size_t i = 0; i < indices.size(); ++i) {
	// 	int idx = indices[i];
	// 	std::vector<cv::Rect> box = prediction_boxes[idx];
		// std::cout << "Box " << idx << ":" <<  box.x << " " << box.y << " " << box.x \
		             + box.width << " " << box.y + box.height << std::endl;
	
	// }
}

int cleanup::Yolo::getObjectCount() {
	return objectCount;
}

int cleanup::Yolo::getnumClasses() {
	return int (classLabels.size()/classLabels[0].size());
}

std::vector<std::string> cleanup::Yolo::getclassLabels() {
	return classLabels;
}

} //namespace cleanuo