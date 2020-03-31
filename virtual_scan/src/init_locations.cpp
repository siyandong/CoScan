#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <cstdlib>
#include <math.h>

#include <fstream>
#include <sstream>

using namespace std;

#define PI 3.1415926

const float camera_height = 1.1;

bool setStartPose(ros::ServiceClient client, int rbtID, float x, float y, float z = camera_height)
{
  float x0 = x, y0 = y, z0 = z;
  float theta0 = 0.0;
  float ox0 = 0.0, oy0 = 0.0, oz0 = 1.0;
  geometry_msgs::Pose start_pose0;
        start_pose0.position.x = x0;
        start_pose0.position.y = y0;
        start_pose0.position.z = z0;
        start_pose0.orientation.x = ox0*sin(theta0/2);
        start_pose0.orientation.y = oy0*sin(theta0/2);
        start_pose0.orientation.z = oz0*sin(theta0/2);
        start_pose0.orientation.w = cos(theta0/2);
  geometry_msgs::Twist start_twist0;
        start_twist0.linear.x = 0.0;
        start_twist0.linear.y = 0.0;
        start_twist0.linear.z = 0.0;
        start_twist0.angular.x = 0.0;
        start_twist0.angular.y = 0.0;
        start_twist0.angular.z = 0.0;
  gazebo_msgs::ModelState modelstate0;
  		char name[100];
  		sprintf(name, "robot_%d", rbtID);
        modelstate0.model_name = (std::string)name;
        modelstate0.reference_frame = (std::string) "world";
        modelstate0.pose = start_pose0;
        modelstate0.twist = start_twist0;
  gazebo_msgs::SetModelState setmodelstate0;
  setmodelstate0.request.model_state = modelstate0;
  //ROS_INFO("call server to move a model");
  if (client.call(setmodelstate0))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ModelState_client");

  ros::NodeHandle n;

  // client
  ros::ServiceClient sclient0 = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::ServiceClient gclient0 = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  
  bool poseSetSuccess;

/*
  // suncad2_1f 20rbt
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 0.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 1.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 10
	poseSetSuccess = setStartPose(sclient0, 10, 0.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 11
	poseSetSuccess = setStartPose(sclient0, 11, 0.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 12
	poseSetSuccess = setStartPose(sclient0, 12, 1.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 13
	poseSetSuccess = setStartPose(sclient0, 13, 1.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 14
	poseSetSuccess = setStartPose(sclient0, 14, 2.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 15
	poseSetSuccess = setStartPose(sclient0, 15, 2.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 16
	poseSetSuccess = setStartPose(sclient0, 16, 3.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 17
	poseSetSuccess = setStartPose(sclient0, 17, 3.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 18
	poseSetSuccess = setStartPose(sclient0, 18, 4.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 19
	poseSetSuccess = setStartPose(sclient0, 19, 4.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

//   // sdu
//   	// 0
//   	poseSetSuccess = setStartPose(sclient0, 0, -3.0, 0.0);
// 	if(!poseSetSuccess) printf("set pose failed\n");
// 	// 1
// 	poseSetSuccess = setStartPose(sclient0, 1, -3.0, -1.0);
// 	if(!poseSetSuccess) printf("set pose failed\n");
// 	// 2
// 	poseSetSuccess = setStartPose(sclient0, 2, -3.0, -2.0);
// 	if(!poseSetSuccess) printf("set pose failed\n");
// 	// 3
// 	poseSetSuccess = setStartPose(sclient0, 3, -4.0, -2.0);
// 	if(!poseSetSuccess) printf("set pose failed\n");
// //*/


/*
	// pku3
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 1.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -4.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/  

/*
    // bfa_home
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 2.0, 3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 2.0, 2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -1.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -4.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
    // pku1
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 5.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 2.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -1.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -4.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/


    // mp01
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 0.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
  // sun4e5b_1f
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.5, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 2.5, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 3.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 2.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 3.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
    // sun65f8_2f
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 0.5, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 1.5, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// sun0d1d_1f
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -6.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -6.0, -0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -6.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -6.0, -1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -6.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, -7.0, -0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -7.0, -0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -7.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, -7.0, -1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, -7.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// mp01_2
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 2.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 0.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -1.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -4.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -1.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -2.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, -3.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, -4.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// mp02_timefirst not finished
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -1.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -1.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -0.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 0.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 0.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, -1.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -1.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -0.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 0.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 0.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// mp02
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -1.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -1.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -0.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 0.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 0.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, -1.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -1.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -0.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 0.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 0.5, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
	// mp03
  	poseSetSuccess = setStartPose(sclient0, 0, 0.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 2.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 3.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 3.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 1.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 2.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 3.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// sun8cd9 ke ting
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -1.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -2.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -3.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -4.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -1.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -2.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, -3.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, -4.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// sun8cd9
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -6.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -6.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -6.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -6.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -6.0, 3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, -5.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -5.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -5.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, -5.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, -5.0, 3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// sun5eb2
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -0.5, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -0.5, -1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -0.5, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -0.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
  // big mp04
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, 7.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 0.0, 8.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 0.0, 9.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 1.0, 7.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 1.0, 8.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 1.0, 9.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 1.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 2.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 2.5, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 3.0, -2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// bfa
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 4.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 4.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 4.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 4.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 4.0, 2.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 4.0, 3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, 3.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
 //*/
/*
// bfagjj
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, -3.0, -7.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, -3.0, -6.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -3.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -3.0, -4.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -3.0, -3.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, -3.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// jy0
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 5.5, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 5.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 4.5, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 5.0, 1.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -1.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

/*
// jy2
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 2.5, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 2.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.5, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 0.5, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, -1.0, 0.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");

	//*/

/*  	
  // wallob
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 0.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 1.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 3.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 3.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 4.0, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 4.5, -9.5);
	if(!poseSetSuccess) printf("set pose failed\n");
*/
/*
// rect center
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 1.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, -1.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, -1.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, 1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 1.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, -2.0, -1.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, -1.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.0, 2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, -1.0, -2.0);
	if(!poseSetSuccess) printf("set pose failed\n");
//*/

  	/*
// rect
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 0.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 0.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 1.5, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 2.0, -4.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 0.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 0.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 1.5, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 2.0, -5.0);
	if(!poseSetSuccess) printf("set pose failed\n");
  	*/

  	/*
  	// 0
	poseSetSuccess = setStartPose(sclient0, 0, 2.0, 0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 1
	poseSetSuccess = setStartPose(sclient0, 1, 1.5, 0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 2
	poseSetSuccess = setStartPose(sclient0, 2, 1.0, 0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 3
	poseSetSuccess = setStartPose(sclient0, 3, 0.5, 0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 4
	poseSetSuccess = setStartPose(sclient0, 4, 0.0, 0);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 5
	poseSetSuccess = setStartPose(sclient0, 5, 2.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 6
	poseSetSuccess = setStartPose(sclient0, 6, 1.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 7
	poseSetSuccess = setStartPose(sclient0, 7, 1.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 8
	poseSetSuccess = setStartPose(sclient0, 8, 0.5, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
	// 9
	poseSetSuccess = setStartPose(sclient0, 9, 0.0, 0.5);
	if(!poseSetSuccess) printf("set pose failed\n");
*/

  return 0;
}
