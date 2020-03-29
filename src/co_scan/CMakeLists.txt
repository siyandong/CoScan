cmake_minimum_required(VERSION 2.8.3)
project(co_scan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/share/OpenCV)
set(octomap_DIR ../octomap/octomap-devel/lib/cmake/octomap)
set(CGAL_DIR ../cgal/CGAL-4.9.1/build)
SET(eigen_INCLUDE_DIRS ./include/eigen)
#SET( EIGEN_INCLUDE_DIR /usr/include/eigen3 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap REQUIRED
  OpenCV 
  camera_calibration_parsers
  cv_bridge
  CGAL REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES co_scan
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
)

add_library(data_engine src/data_engine.cpp)
add_library(navigation src/navigation.cpp)

add_executable(co_scan src/co_scan.cpp
src/global.cpp
src/geodesic/RichModel.cpp
src/geodesic/Xin_Wang.cpp
src/geodesic/DistanceApproach.cpp
src/geodesic/ExactDGPMethod.cpp
src/geodesic/BaseModel.cpp
src/geodesic/Chen_Han.cpp
src/geodesic/ICH_WindowFiltering.cpp
src/geodesic/Point3D.cpp
src/tsp/tsp.cpp
src/tsp/threads.cpp
src/tsp/MyThread.cpp
src/tsp/twoOpt.cpp
)
target_link_libraries(co_scan 
${catkin_LIBRARIES} 
${OCTOMAP_LIBRARIES}
#${CGAL_LIBRARIES}
CGAL::CGAL
CGAL::CGAL_Core
data_engine
navigation
#opencv_core, opencv_videoio, 
#opencv_highgui
${OpenCV_LIBS}
)
add_executable(poly_diff src/polygon_diff.cpp src/global.cpp)
target_link_libraries(poly_diff 
CGAL::CGAL CGAL::CGAL_Core
#opencv_core, opencv_videoio, 
#opencv_highgui
${OpenCV_LIBS}
)

