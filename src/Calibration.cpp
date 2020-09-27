#include "Calibration.h"

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "DepthRanging.h"

std::vector<std::string> Calibration::split(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<float> Calibration::getValuesByKey(const std::string file,
                                               const std::string inKey) {

  std::ifstream infile(file);
  std::stringstream ss;
  if (infile) {
    ss << infile.rdbuf();
    infile.close();
  }

  std::vector<float> values;
  for (std::string line; std::getline(ss, line);) {
    std::vector<std::string> entries = split(line, ':');
    std::string key = entries.at(0);
    std::string content = entries.at(1);
    if (key == inKey) {
      std::vector<std::string> words = split(content, ' ');
      for (std::string word : words) {
        if (word != "\n") {
          try {
            float value = std::stof(word);
            values.push_back(value);
          } catch (std::exception &e) {
          }
        }
      }
    }
  }
  return values;
}

void Calibration::loadVelo2Cam(const std::string file) {

  std::vector<float> Rot_values =
      getValuesByKey(file, std::string("R")); // Rotation vector
  Eigen::Matrix3f Rot_v2c =
      Eigen::Map<Eigen::Matrix<float, 3, 3>>(Rot_values.data());
  Rot_v2c.transposeInPlace();

  std::vector<float> Transl_values =
      getValuesByKey(file, std::string("T")); // Translation Vector
  Eigen::Vector3f Transl_v2c =
      Eigen::Map<Eigen::Matrix<float, 1, 3>>(Transl_values.data());

  Eigen::Matrix4f transform; // Your Transformation Matrix
  // Set to Identity to make bottom row of Matrix 0,0,0,1
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = Rot_v2c;
  transform.block<3, 1>(0, 3) = Transl_v2c;

  this->velo2cam = transform.matrix();
}

void Calibration::loadCam2Cam(const std::string file,
                              const std::string camera) {

  std::vector<float> Proj_values = getValuesByKey(
      file, std::string("P_rect_") + std::string(camera)); // Rotation vector

  Eigen::Matrix<float, 4, 3> large;
  large = Eigen::Map<Eigen::Matrix<float, 4, 3>>(Proj_values.data());
  Eigen::Matrix3f small = large.block<3, 3>(0, 0);
  small.transposeInPlace();
  this->cam2cam = small;
}