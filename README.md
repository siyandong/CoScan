# Multi-Robot Collaborative Dense Scene Reconstruction (SIGGRAPH 2019)
We provide the implementation of the optimal-mass-transport algorithm tailored for collaborative scanning along with virtual scan examples, on top of ROS.

# Requirements
This package depends on OpenCV, CGAL and OctoMap. Please install these libraries first.

# Download and Installation
```
catkin_ws$ git clone 
catkin_ws$ catkin_make_isolated
```

# Run
roslaunch virtual_scan simulator.launch

roslaunch virtual_scan data_sever.launch

rosrun co_scan co_scan
