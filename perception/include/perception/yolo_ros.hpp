#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace cleanup {

class Yolo {
 public:

  /* @brief Constructor */
  Yolo();

  std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

  std::vector<cv::Rect> predict(cv::Mat frame);

  void postProcess(cv::Mat& frame, const std::vector<cv::Mat>& preds, \
          std::vector<int> class_ids, std::vector<float> confidences,\
          std::vector<int> indices);

  int getObjectCount();

private:
  double confidenceThreshold;
  double nmsThreshold;
  double frameWidth;
  double frameHeight;
  int objectCount;

  std::string configPath;
  std::string weightsPath;
  std::string classLabelsFile;
  std::vector<std::string> classLabels;
  std::vector<cv::Rect> prediction_boxes;
  cv::dnn::Net net;
};

} // namespace cleanup