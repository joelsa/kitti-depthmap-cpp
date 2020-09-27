add_executable (kitti-depthmap kitti-depthmap.cpp)

add_library (DepthRanging DepthRanging.cpp)
add_library (Calibration Calibration.cpp)
target_link_libraries (Calibration Eigen3::Eigen ${OpenCV_LIBS})

set_target_properties(kitti-depthmap PROPERTIES
  CXX_STANDARD 11
  CXX_EXTENSIONS OFF
  CXX_STANDARD_REQUIRED YES)
  
target_link_libraries (kitti-depthmap Eigen3::Eigen DepthRanging Calibration ${OpenCV_LIBS})