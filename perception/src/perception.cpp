#include <perception/perception.h>

namespace cleanup {

Perception::Perception() {

	/* YOLO initialization */
	confidenceThreshold = 0.5;  // Confidence threshold
	nmsThreshold = 0.4;  // Non-maximum suppression threshold
	inputWidth = 320;  // Width of network's input image
	inputHeight = 320;  // Height of network's input image

	modelCofig = "";  // Add path to the config file
	modelWeight = "";  //Add path to the weight file

	//  Store class labels in the session memory
	classLabelsFile = "";
	std::ifstream ifs(classLabelsFile.c_str());
	std::string line;
	while (ifs >> line) {
		classLabels.push_back(line);
	}
	// Load the network
	net = cv::dnn::readNetFromDarknet(modelConfig, modelWeight);
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

	/* Laserscan initialization*/
	obstacle_dis = 0.5;
    obstacle_ahead = false;
    //  Publish velocity data into the node.
    velocity = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribe to the laser scan message.
    laser = nh.subscribe <sensor_msgs::LaserScan> \
            ("scan", 1, &walker::laserCallback, this);

}

std::vector<cv::Rect> Perception::predict(cv::Mat frame) {
	if (frame.empty()) {
		std::cout << "Error reading frame!!!" << std::endl;
	}
	cv::Mat blob;
	cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(inputWidth, inputHeight),\
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

std::vector<int> Perception::detectObjects() {

}

geometry_msgs::PoseStamped Perception::getObjectPose() {

}

void Perception::imageCallback() {

}

void Perception::runVisionAlgo() {

}

} // namespace cleanup