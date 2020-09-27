#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

class Calibration {
  public:
    Eigen::Affine3f velo2cam;
    Eigen::Matrix3f cam2cam;

    void loadVelo2Cam(std::string file);
    void loadCam2Cam(std::string file, std::string camera);

  private:
    std::vector<std::string> split(const std::string &s, char delim);
    std::vector<float> getValuesByKey(const std::string file, const std::string inKey);
  
};