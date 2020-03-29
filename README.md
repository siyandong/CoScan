# CoScan
Implementation on top of ROS for the virtual scan of the paper Multi-Robot Collaborative Dense Scene Reconstruction (SIGGRAPH 2019).

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
