#include <opencv2/core.hpp>

class DepthRanging {
public:
  double range;
  int x, y;

  DepthRanging(double range, int x, int y) : range(range), x(x), y(y) {}
  ~DepthRanging() {}

  bool operator<(DepthRanging b) const { return this->range < b.range; }

  bool fitsInto(const cv::Mat &img);
};