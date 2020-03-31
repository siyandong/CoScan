cmake_minimum_required(VERSION 2.8.3)
project(virtual_scan)

#set(octomap_DIR /opt/ros/indigo/share/octomap)

#set(gazebo_DIR "/usr/share/gazebo/cmake")

#set(OpenCV_DIR "/usr/local/share/OpenCV")
set(OpenCV_DIR "/usr/share/OpenCV")

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  #gazebo
  #gazebo_ros
  #gazebo_msgs
  OpenCV 
  camera_calibration_parsers
  cv_bridge 
  image_transport 
  message_filters 
  nodelet 
  rosconsole 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES virtual_scan
#  CATKIN_DEPENDS rospy std_msgs roscpp
#  DEPENDS system_lib
)

SET( EIGEN_INCLUDE_DIR /usr/include/eigen3 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
  #${OCTOMAP_INCLUDE_DIRS}
  #${EIGEN_INCLUDE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(init_locations src/init_locations.cpp)
target_link_libraries(init_locations 
${catkin_LIBRARIES} 
${gazebo_msgs_LIBRARIES} 
${GAZEBO_LIBRARIES}
)

add_executable(vscan_server src/vscan_server.cpp)
target_link_libraries(vscan_server 
${catkin_LIBRARIES} 
${gazebo_msgs_LIBRARIES} 
${GAZEBO_LIBRARIES} 
#${OCTOMAP_LIBRARIES} 
#${OpenCV_LIBS}
)

