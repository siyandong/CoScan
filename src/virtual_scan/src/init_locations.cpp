#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <cstdlib>
#include <math.h>

#include <fstream>
#include <sstream>

//#include <syswait.h>

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
  		//sprintf(name, "test%d", rbtID);
  		sprintf(name, "robot_%d", rbtID);
        modelstate0.model_name = (std::string)name;
        //modelstate0.model_name = (std::string) "test" + rbtID;
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
  //ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
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



	//// 5
	//poseSetSuccess = setStartPose(sclient0, 5, 3.0, -2.5);
	//if(!poseSetSuccess) printf("set pose failed\n");
	//// 6
	//poseSetSuccess = setStartPose(sclient0, 6, 2.5, -2.5);
	//if(!poseSetSuccess) printf("set pose failed\n");
	//// 7
	//poseSetSuccess = setStartPose(sclient0, 7, 2.0, -2.5);
	//if(!poseSetSuccess) printf("set pose failed\n");
	//// 8
	//poseSetSuccess = setStartPose(sclient0, 8, 1.5, -2.5);
	//if(!poseSetSuccess) printf("set pose failed\n");
	//// 9
	//poseSetSuccess = setStartPose(sclient0, 9, 1.0, -2.5);
	//if(!poseSetSuccess) printf("set pose failed\n");

/*

  // 0 pose
  //float x0 = 0.0, y0 = -1.0, z0 = camera_height;
  //float x0 = 0.68, y0 = -0.76, z0 = camera_height; //r1 3
  //float x0 = 1.0, y0 = -3.0, z0 = camera_height; //r2 3
  //float x0 = 3.0, y0 = -1.0, z0 = camera_height; //c1 3
  //float x0 = 1.0, y0 = -1.0, z0 = camera_height; //mp1
  float x0 = 3.0, y0 = -3.0, z0 = camera_height; //mp01
  //float x0 = -3.0, y0 = 0.0, z0 = camera_height; //office
  //float x0 = 1.5, y0 = 0.5, z0 = camera_height; //progressive in mp3
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
        modelstate0.model_name = (std::string) "test0";
        modelstate0.reference_frame = (std::string) "world";
        modelstate0.pose = start_pose0;
        modelstate0.twist = start_twist0;
  gazebo_msgs::SetModelState setmodelstate0;
  setmodelstate0.request.model_state = modelstate0;
  //ROS_INFO("call server to move a model");
  if (sclient0.call(setmodelstate0))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  // 1 pose
  //float x1 = 0.0, y1 = -0.5, z1 = camera_height;
  //float x1 = -0.02, y1 = 0.0, z1 = camera_height;//r1 3
  //float x1 = 1.0, y1 = -2.0, z1 = camera_height;//r2 3
  //float x1 = 3.0, y1 = 0.0, z1 = camera_height;//c1 3
  //float x1 = 0.0, y1 = -1.0, z1 = camera_height;//mp1
  float x1 = 2.5, y1 = -3.0, z1 = camera_height;//mp01
  //float x1 = -3.0, y1 = -1, z1 = camera_height;//office
  //float x1 = -0.5, y1 = 0.5, z1 = camera_height;//prog
  float theta1 = 0.0;
  float ox1 = 0.0, oy1 = 0.0, oz1 = 1.0;
  geometry_msgs::Pose start_pose1;
        start_pose1.position.x = x1;
        start_pose1.position.y = y1;
        start_pose1.position.z = z1;
        start_pose1.orientation.x = ox1*sin(theta1/2);
        start_pose1.orientation.y = oy1*sin(theta1/2);
        start_pose1.orientation.z = oz1*sin(theta1/2);
        start_pose1.orientation.w = cos(theta1/2);
  geometry_msgs::Twist start_twist1;
        start_twist1.linear.x = 0.0;
        start_twist1.linear.y = 0.0;
        start_twist1.linear.z = 0.0;
        start_twist1.angular.x = 0.0;
        start_twist1.angular.y = 0.0;
        start_twist1.angular.z = 0.0;
  gazebo_msgs::ModelState modelstate1;
        modelstate1.model_name = (std::string) "test1";
        modelstate1.reference_frame = (std::string) "world";
        modelstate1.pose = start_pose1;
        modelstate1.twist = start_twist1;
  gazebo_msgs::SetModelState setmodelstate1;
  setmodelstate1.request.model_state = modelstate1;
  //ROS_INFO("call server to move a model");
  if (sclient0.call(setmodelstate1))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  // 2 pose
  //float x2 = 0.0, y2 = 0.0, z2 = camera_height;
  //float x2 = -0.72, y2 = 0.76, z2 = camera_height;//r1
  //float x2 = 1.0, y2 = -1.0, z2 = camera_height;//r2
  //float x2 = 3.0, y2 = 1.0, z2 = camera_height;//c1
  //float x2 = 2.0, y2 = -1.0, z2 = camera_height;//r2 5
  float x2 = 2.0, y2 = -3.0, z2 = camera_height;//mp01
  //float x2 = -3.0, y2 = -2.0, z2 = camera_height;//office
  //float x2 = 0.5, y2 = 0.5, z2 = camera_height;//prog
  float theta2 = 0.0;
  float ox2 = 0.0, oy2 = 0.0, oz2 = 1.0;
  geometry_msgs::Pose start_pose2;
        start_pose2.position.x = x2;
        start_pose2.position.y = y2;
        start_pose2.position.z = z2;
        start_pose2.orientation.x = ox2*sin(theta2/2);
        start_pose2.orientation.y = oy2*sin(theta2/2);
        start_pose2.orientation.z = oz2*sin(theta2/2);
        start_pose2.orientation.w = cos(theta2/2);
  geometry_msgs::Twist start_twist2;
        start_twist2.linear.x = 0.0;
        start_twist2.linear.y = 0.0;
        start_twist2.linear.z = 0.0;
        start_twist2.angular.x = 0.0;
        start_twist2.angular.y = 0.0;
        start_twist2.angular.z = 0.0;
  gazebo_msgs::ModelState modelstate2;
        modelstate2.model_name = (std::string) "test2";
        modelstate2.reference_frame = (std::string) "world";
        modelstate2.pose = start_pose2;
        modelstate2.twist = start_twist2;
  gazebo_msgs::SetModelState setmodelstate2;
  setmodelstate2.request.model_state = modelstate2;
  //ROS_INFO("call server to move a model");
  if (sclient0.call(setmodelstate2))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  // 3 pose
  float x3 = 1.5, y3 = -3.0, z3 = camera_height; // mp01
  float theta3 = 0.0;
  float ox3 = 0.0, oy3 = 0.0, oz3 = 1.0;
  geometry_msgs::Pose start_pose3;
        start_pose3.position.x = x3;
        start_pose3.position.y = y3;
        start_pose3.position.z = z3;
        start_pose3.orientation.x = ox3*sin(theta3/2);
        start_pose3.orientation.y = oy3*sin(theta3/2);
        start_pose3.orientation.z = oz3*sin(theta3/2);
        start_pose3.orientation.w = cos(theta3/2);
  geometry_msgs::Twist start_twist3;
        start_twist3.linear.x = 0.0;
        start_twist3.linear.y = 0.0;
        start_twist3.linear.z = 0.0;
        start_twist3.angular.x = 0.0;
        start_twist3.angular.y = 0.0;
        start_twist3.angular.z = 0.0;
  gazebo_msgs::ModelState modelstate3;
        modelstate3.model_name = (std::string) "test3";
        modelstate3.reference_frame = (std::string) "world";
        modelstate3.pose = start_pose3;
        modelstate3.twist = start_twist3;
  gazebo_msgs::SetModelState setmodelstate3;
  setmodelstate3.request.model_state = modelstate3;
  //ROS_INFO("call server to move a model");
  if (sclient0.call(setmodelstate3))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  // 4 pose
  float x4 = 1.0, y4 = -3.0, z4 = camera_height; //mp01
  float theta4 = 0.0;
  float ox4 = 0.0, oy4 = 0.0, oz4 = 1.0;
  geometry_msgs::Pose start_pose4;
        start_pose4.position.x = x4;
        start_pose4.position.y = y4;
        start_pose4.position.z = z4;
        start_pose4.orientation.x = ox4*sin(theta4/2);
        start_pose4.orientation.y = oy4*sin(theta4/2);
        start_pose4.orientation.z = oz4*sin(theta4/2);
        start_pose4.orientation.w = cos(theta4/2);
  geometry_msgs::Twist start_twist4;
        start_twist4.linear.x = 0.0;
        start_twist4.linear.y = 0.0;
        start_twist4.linear.z = 0.0;
        start_twist4.angular.x = 0.0;
        start_twist4.angular.y = 0.0;
        start_twist4.angular.z = 0.0;
  gazebo_msgs::ModelState modelstate4;
        modelstate4.model_name = (std::string) "test4";
        modelstate4.reference_frame = (std::string) "world";
        modelstate4.pose = start_pose4;
        modelstate4.twist = start_twist4;
  gazebo_msgs::SetModelState setmodelstate4;
  setmodelstate4.request.model_state = modelstate4;
  //ROS_INFO("call server to move a model");
  if (sclient0.call(setmodelstate4))
  {
    //ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }















  // pass main loop
  return 0;

/*

  // main loop for offline data
  int frame_count = 0;
  int max_count = 0;
  ifstream ifs_off;
  ifs_off.open("/home/dsy/catkin_ws/src/virtual_scan/data/offline/pose_history.txt");
  while(1){
    if(frame_count==max_count)
      break;
    float pass[7];
    geometry_msgs::Pose posep;
    // 0
    ifs_off>>pass[0]>>pass[1]>>pass[2]>>pass[3]>>pass[4]>>pass[5]>>pass[6];
    posep.position.x = pass[0];
    posep.position.y = pass[1];
    posep.position.z = pass[2];
    posep.orientation.x = pass[3];
    posep.orientation.y = pass[4];
    posep.orientation.z = pass[5];
    posep.orientation.w = pass[6];
    modelstate0.pose = posep;
    setmodelstate0.request.model_state = modelstate0;
    if (sclient0.call(setmodelstate0)){}
    else
      ROS_ERROR("Failed to call service");
    // 1
    ifs_off>>pass[0]>>pass[1]>>pass[2]>>pass[3]>>pass[4]>>pass[5]>>pass[6];
    posep.position.x = pass[0];
    posep.position.y = pass[1];
    posep.position.z = pass[2];
    posep.orientation.x = pass[3];
    posep.orientation.y = pass[4];
    posep.orientation.z = pass[5];
    posep.orientation.w = pass[6];
    modelstate1.pose = posep;
    setmodelstate1.request.model_state = modelstate1;
    if (sclient0.call(setmodelstate1)){}
    else
      ROS_ERROR("Failed to call service");
    // 2
    ifs_off>>pass[0]>>pass[1]>>pass[2]>>pass[3]>>pass[4]>>pass[5]>>pass[6];
    posep.position.x = pass[0];
    posep.position.y = pass[1];
    posep.position.z = pass[2];
    posep.orientation.x = pass[3];
    posep.orientation.y = pass[4];
    posep.orientation.z = pass[5];
    posep.orientation.w = pass[6];
    modelstate2.pose = posep;
    setmodelstate2.request.model_state = modelstate2;
    if (sclient0.call(setmodelstate2)){}
    else
      ROS_ERROR("Failed to call service");

    sleep(1);

    frame_count++;
  }
  ifs_off.close();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  // main loop test
  char input_char;
  while(1){
    printf("input to move:\n");
    scanf("%c", &input_char);
    if(input_char == 'e'){
      break;
    }
    else{
        gazebo_msgs::GetModelState getmodelstate0;
        getmodelstate0.request.model_name = (std::string) "test0";
        //ROS_INFO("call server to get pose");
        if (gclient0.call(getmodelstate0))
        {
          //ROS_INFO("success");
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 1;
        }
        geometry_msgs::Pose pose0;
        pose0.position.x = x0;
        pose0.position.y = y0;
        pose0.position.z = z0;

        float cx = getmodelstate0.response.pose.position.x;
        float cy = getmodelstate0.response.pose.position.y;
        float ox = 0.2;
        float oy = 0;
        float dx = ox - cx;
        float dy = oy - cy;
        float dir_theta;
        
        if (dx > 0){
          if(dy == 0){
            dir_theta = 0;
          }
          else{
            dir_theta = atan(dy / dx);
          }
        }
        else if (dx == 0){
          if(dy > 0){
            dir_theta = PI/2;
          }
          else if (dy == 0){
            dir_theta = 0; // need to modify to origin direction
          }
          else{// dy < 0
            dir_theta = -PI/2;
          }
        }
        else{ // dx < 0
          if(dy>0){
            dir_theta = PI - fabs(atan(dy / dx));
          }
          else if(dy == 0){
            dir_theta = PI;
          }
          else{// dy < 0
            dir_theta = -(PI - fabs(atan(dy / dx)));
          }
        }

        pose0.orientation.x = ox0*sin(dir_theta/2);
        pose0.orientation.y = oy0*sin(dir_theta/2);
        pose0.orientation.z = oz0*sin(dir_theta/2);
        pose0.orientation.w = cos(dir_theta/2);
        modelstate0.pose = pose0;
        setmodelstate0.request.model_state = modelstate0;
        if (sclient0.call(setmodelstate0)){
          printf("rotated.\n");
        }
        else
          ROS_ERROR("Failed to call service");

        // gazebo_msgs::GetModelState getmodelstate0;
        // getmodelstate0.request.model_name = (std::string) "test0";
        // //ROS_INFO("call server to get pose");
        // if (gclient0.call(getmodelstate0))
        // {
        //   //ROS_INFO("success");
        // }
        // else
        // {
        //   ROS_ERROR("Failed to call service");
        //   return 1;
        // }
        // float tempx = getmodelstate0.response.pose.position.x + 0.0;
        // float tempy = getmodelstate0.response.pose.position.y + 0.1;
        // geometry_msgs::Pose pose0;
        // pose0.position.x = tempx;
        // pose0.position.y = tempy;
        // pose0.position.z = z0;
        // pose0.orientation.x = ox0*sin(theta0/2);
        // pose0.orientation.y = oy0*sin(theta0/2);
        // pose0.orientation.z = oz0*sin(theta0/2);
        // pose0.orientation.w = cos(theta0/2);
        // modelstate0.pose = pose0;
        // setmodelstate0.request.model_state = modelstate0;
        // if (sclient0.call(setmodelstate0)){}
        // else
        //   ROS_ERROR("Failed to call service");
    }
  }

  // // main loop for get&set pose for multi robots
  // char rbtnum;
  // while(1){
  //   printf("select robot:\n");
  //   scanf("%c", &rbtnum);
  //   if(rbtnum=='e'){
  //     break;
  //   }
  //   printf("set pose: x y z theta\n");
  //   // rbt 0
  //   if(rbtnum=='0'){
  //     scanf("%f %f %f %f", &x0, &y0, &z0, &theta0);
  //     geometry_msgs::Pose pose0;
  //       pose0.position.x = x0;
  //       pose0.position.y = y0;
  //       pose0.position.z = z0;
  //       pose0.orientation.x = ox0*sin(theta0/2);
  //       pose0.orientation.y = oy0*sin(theta0/2);
  //       pose0.orientation.z = oz0*sin(theta0/2);
  //       pose0.orientation.w = cos(theta0/2);
  //     modelstate0.pose = pose0;
  //     setmodelstate0.request.model_state = modelstate0;
  //     if (sclient0.call(setmodelstate0)){}
  //     else
  //       ROS_ERROR("Failed to call service");
  //   }
  //   // rbt 1
  //   else if(rbtnum=='1'){
  //     scanf("%f %f %f %f", &x1, &y1, &z1, &theta1);
  //     geometry_msgs::Pose pose1;
  //       pose1.position.x = x1;
  //       pose1.position.y = y1;
  //       pose1.position.z = z1;
  //       pose1.orientation.x = ox1*sin(theta1/2);
  //       pose1.orientation.y = oy1*sin(theta1/2);
  //       pose1.orientation.z = oz1*sin(theta1/2);
  //       pose1.orientation.w = cos(theta1/2);
  //     modelstate1.pose = pose1;
  //     setmodelstate1.request.model_state = modelstate1;
  //     if (sclient0.call(setmodelstate1)){}
  //     else
  //       ROS_ERROR("Failed to call service");
  //   }else if(rbtnum=='2'){
  //   }
  // }

  /*
  // main loop
  char rsign='0', sign;
  while(1){    
    scanf("%c", &sign);
    if(sign=='x')
      break;
    else if(rsign=='0'){
      if(sign=='0'){
        rsign = '0';
        continue;
      }
      else if(sign=='1'){
        rsign = '1';
        continue;
      }
      else if(sign=='p'){
        gazebo_msgs::GetModelState getmodelstate0;
        getmodelstate0.request.model_name = (std::string) "test0";
        //ROS_INFO("call server to get pose");
        if (gclient0.call(getmodelstate0))
        {
          //ROS_INFO("success");
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 1;
        }
        printf("theta0: %f\n", theta0);
        printf("q0: w x y z\n%f %f %f %f\n", 
          getmodelstate0.response.pose.orientation.w, 
          getmodelstate0.response.pose.orientation.x,
          getmodelstate0.response.pose.orientation.y,
          getmodelstate0.response.pose.orientation.z);
        printf("t0: x y z\n%f %f %f\n", 
          getmodelstate0.response.pose.position.x,
          getmodelstate0.response.pose.position.y,
          getmodelstate0.response.pose.position.z);
        continue;
      }
      else if(sign=='w')
        start_pose0.position.x += 0.005;
      else if(sign=='s')
        start_pose0.position.x -= 0.005;
      else if(sign=='a')
        start_pose0.position.y += 0.005;
      else if(sign=='d')
        start_pose0.position.y -= 0.005;
      else if(sign=='r')
        start_pose0.position.z += 0.005;
      else if(sign=='f')
        start_pose0.position.z -= 0.005;
      else if(sign=='q'){
        theta0+=0.01;
        if (theta0>PI)
        {
          theta0 = -(PI-(theta0-PI));
        }
      }
      else if(sign=='e'){
        theta0-=0.01;
        if (theta0<-PI)
        {
          theta0 = PI-(-PI-theta0);
        }
      }
      start_pose0.orientation.x = ox0*sin(theta0/2);
      start_pose0.orientation.y = oy0*sin(theta0/2);
      start_pose0.orientation.z = oz0*sin(theta0/2);
      start_pose0.orientation.w = cos(theta0/2);
      modelstate0.pose = start_pose0;
      gazebo_msgs::SetModelState setmodelstate0;
      setmodelstate0.request.model_state = modelstate0;
      //ROS_INFO("call server to move a model");
      if (sclient0.call(setmodelstate0))
      {
        //ROS_INFO("success");
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return 1;
      }
    }
    else if(rsign=='1'){
      if(sign=='0'){
        rsign = '0';
        continue;
      }
      else if(sign=='1'){
        rsign = '1';
        continue;
      }
    }
    
  }//*/

  //*/

  return 0;
}
