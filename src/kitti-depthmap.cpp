#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "DepthRanging.h"
#include "Calibration.h"

cv::Mat overimposeDepth(const cv::Mat& background, const cv::Mat& depthmap, const int colorMap) {
  cv::Mat mask = depthmap > 0;
  cv::applyColorMap(depthmap, depthmap, colorMap);
  depthmap &= mask;
  background &= ~mask;
  return background + depthmap;                   
}

std::vector<Eigen::Vector3f> readCloud(std::string file) {

  std::vector<Eigen::Vector3f> cloud;
  
	std::fstream input(file.c_str(), std::ios::in | std::ios::binary);
	if(!input.good()) {
		std::cerr << "Could not read file: " << file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		Eigen::Vector3f point;
    float intensity;
		input.read((char *) &point[0], 3*sizeof(float));
		input.read((char *) &intensity, sizeof(float));
		cloud.push_back(point);
	}
	input.close();

  return cloud;

}

std::vector<DepthRanging> transformCloudToCamera(std::vector<Eigen::Vector3f> cloud, Calibration calib, cv::Mat img) {

  std::vector<DepthRanging> rangings;

  for( Eigen::Vector3f lidar_pt : cloud ) {
  
    Eigen::Vector3f camera_pt = calib.velo2cam * lidar_pt;
    double range = camera_pt.norm(); 
    Eigen::Vector3f pixel_pt = calib.cam2cam * camera_pt;

    // Dismiss all points that lie behind the camera or outside the image
    if (pixel_pt(2) > 0) {
    	pixel_pt = pixel_pt / pixel_pt(2); 

    	DepthRanging rxy(range, (int)pixel_pt(0), (int)pixel_pt(1));
    	if (rxy.fitsInto(img)) {
    		rangings.push_back(rxy);
    	}
    }
  }

  return rangings;

}

const char * usage =
"This program allows a user to generate a point cloud from KITTI.\n"
"Usage: ./kitti-depthmap-cpp\n"
"       cloud_file              # the location of the .bin file\n"
"       image_file              # the location of the .png file to superimpose upon\n"
"       velo_calib_file         # the location of the velo2cam .txt file\n"
"       cam_calib_file          # the location of the cam2cam file\n"
"\n";

static void help() {
    printf("%s", usage);
}

int main (int argc, char** argv) {

  int radius = 1;
  bool gray = true;
  std::string camera = (gray ? "00" : "02");

  cv::CommandLineParser parser(argc, argv,
        "{help ||}"
        "{@cloud_file|<none>|point cloud location}"
        "{@image_file|<none>|png image location}"
        "{@velo_calib_file|<none>|velodyne calibration location}"
        "{@cam_calib_file|<none>|camera calibration location}");

  if (parser.has("help")) {
    help();
    return 0;
  }

  std::string cloud_file = parser.get<std::string>(0);
  std::string image_file =parser.get<std::string>(1);
  std::string velo_calib_file = parser.get<std::string>(2);
  std::string cam_calib_file = parser.get<std::string>(3);

  if (!parser.check()) {
    help();
    parser.printErrors();
    return -1;
  }

  Calibration calib;
  calib.loadVelo2Cam(velo_calib_file);
  calib.loadCam2Cam(cam_calib_file, camera);

  std::vector<Eigen::Vector3f> cloud = readCloud(cloud_file);

  cv::Mat input_img;
  input_img = cv::imread(image_file);

  cv::Mat normalized = cv::Mat(input_img.rows, input_img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat raw = cv::Mat(input_img.rows, input_img.cols, CV_32FC1, cv::Scalar(0.0));

  // ############################################################################################ //

  std::vector<DepthRanging> rangings = transformCloudToCamera(cloud, calib, input_img);

  std::vector<DepthRanging>::iterator minPointIt = 
    std::min_element(rangings.begin(), rangings.end());
  std::vector<DepthRanging>::iterator maxPointIt = 
    std::max_element(rangings.begin(), rangings.end());
  double min = minPointIt->range;
  double max = maxPointIt->range;
  int scale = 255;

  for( DepthRanging point : rangings ) {
    cv::Point position(point.x, point.y);
    double range = point.range;
    raw.at<float>(position) = range;

    // normalize
    int color = (max - range) / (max - min) * 255;
    color = std::max(std::min(color, 255), 0);
    cv::circle(normalized, position, radius, cv::Scalar(color, color, color), cv::FILLED, 8,0);
  }

  cv::Mat output_img = overimposeDepth(input_img, normalized, cv::COLORMAP_VIRIDIS);

  cv::imshow("Input image", output_img);
  cv::imwrite("overlay.png", output_img);
  cv::waitKey(0);
  
  return (0);

}