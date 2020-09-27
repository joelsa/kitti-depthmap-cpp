#include "DepthRanging.h"
#include <opencv2/core.hpp>

bool DepthRanging::fitsInto(const cv::Mat &img) {
  cv::Rect rect(cv::Point(), img.size());
  cv::Point point(x, y);
  return rect.contains(point);
}