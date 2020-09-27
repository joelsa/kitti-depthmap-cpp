cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(kitti-depthmap)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_BINARY_DIR}/lib)

##### Package Dependencies #####
find_package( Eigen3 3.0 REQUIRED NO_MODULE )
find_package( OpenCV 4.0 REQUIRED )

##### Subdirectories #####
add_subdirectory(src)