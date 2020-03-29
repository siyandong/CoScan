// ros msgs && opencv
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h> 
#include <stdlib.h> 
// socket
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <pthread.h>
#include <netinet/tcp.h>

//#include <octomap/octomap.h>

#include <fstream>
#include <sstream>

using namespace std;

#define SERVPORT 3333 /*服务器监听端口号 */
int sockfd,client_fd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
struct sockaddr_in my_addr; /* 本机地址信息 */
struct sockaddr_in remote_addr; /* 客户端地址信息 */
const int MAXRECV = 10240;
#define BACKLOG 10 /* 最大同时连接请求数 */

#define PI 3.1415926

int rbtnum = 3;
float camera_height = 1.1;

cv::Mat rgbImg;
cv::Mat shortImg(480, 640, CV_16UC1);
bool rgb_ready = false;
bool depth_ready = false;

// ros node handle
//ros::NodeHandle n;
// image transport
image_transport::ImageTransport* it;
// rgbd data from topics
vector<cv::Mat> crt_rgb_images(rbtnum);
vector<cv::Mat> crt_depth_images(rbtnum);
// subscribers
vector<image_transport::Subscriber> subsColor(rbtnum);
vector<image_transport::Subscriber> subsDepth(rbtnum);

ros::ServiceClient gclient;
ros::ServiceClient sclient;
float pose[7];
bool pose_ready = false;

const float camera_factor = 1000;
const float camera_cx = 320.500000;
const float camera_cy = 240.500000;
const float camera_fx = 554.382713;
const float camera_fy = 554.382713;

const int times = 1.0;
float rbt_v = 1.0/times - 0.01;

// 每次移动走路径的比例
const double move_distance_rate = 1;

//vector< vector< float > > objective_positions;

// data offline rcv writer
ofstream ofs_off;

// void int2str(int n, char* ch)
// {
//     stringstream ss;
//     string s;
//     ss<<n;
//     ss>>s;
//     strcpy(ch, const_cast<char*>(s.c_str()));
// } 

string int2str(int n)
{
    stringstream ss;
    string s;
    ss<<n;
    ss>>s;
    return s;
} 

int str2int(string s)
{
    stringstream ss(s);
    int number;
    ss>>number;
    return number;
}

// color frame callback. 2018-09-10. no mem leak. 2018-09-19.
void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string &rbtID) 
{    
    printf("rgb callback, robot index: %s\n", rbtID.c_str());

    int rbtIndex = str2int(rbtID);
    
    cv_bridge::CvImagePtr cvImgPtr;
    cv::Mat rgbPass; 

    //rgbPass.clone(); // test

    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbPass = cvImgPtr->image; 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // way 1
    crt_rgb_images[rbtIndex] = rgbPass; 

    // // way 2. 2019-09-18.
    // for (int i = 0; i < 480; ++i)
    // {
    //     for (int j = 0; j < 640; ++j)
    //     {
    //         crt_rgb_images[rbtIndex].ptr<cv::Vec3b>(i)[j][0] = rgbPass.ptr<cv::Vec3b>(i)[j][0];
    //         crt_rgb_images[rbtIndex].ptr<cv::Vec3b>(i)[j][1] = rgbPass.ptr<cv::Vec3b>(i)[j][1];
    //         crt_rgb_images[rbtIndex].ptr<cv::Vec3b>(i)[j][2] = rgbPass.ptr<cv::Vec3b>(i)[j][2];
    //     }
    // }
}

// depth frame callback. 2018-09-10. no mem leak. 2018-09-19.
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string &rbtID) 
{
    printf("depth callback, robot index: %s\n", rbtID.c_str());

    int rbtIndex = str2int(rbtID);

    cv_bridge::CvImagePtr cvImgPtr;
    cv::Mat depthImg;

    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depthImg = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat shortPass(480, 640, CV_16UC1); 

    // way 1
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            shortPass.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000); 
        }
    }
    crt_depth_images[rbtIndex] = shortPass; 
    
    // // way 2. 2019-09-18.
    // for (int i = 0; i < 480; ++i)
    // {
    //     for (int j = 0; j < 640; ++j)
    //     {
    //         crt_depth_images[rbtIndex].ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000); 
    //     }
    // }
}

// pose
void callForPose(int id){
    gazebo_msgs::GetModelState getmodelstate;
    //string s;
    //stringstream ss;
    //ss<<"test"<<id;
    //ss>>s;
    char s[10];
    sprintf(s, "robot_%d", id);
    getmodelstate.request.model_name = (std::string) s;
    //ROS_INFO("call server to get pose");
    if (gclient.call(getmodelstate))
    {
      //ROS_INFO("success");
    }
    else
    {
      ROS_ERROR("Failed to call service");
    }
    //printf("position x: %f\n", getmodelstate.response.pose.position.x);
    pose[0] = getmodelstate.response.pose.position.x;
    pose[1] = getmodelstate.response.pose.position.y;
    pose[2] = getmodelstate.response.pose.position.z;
    pose[3] = getmodelstate.response.pose.orientation.x;
    pose[4] = getmodelstate.response.pose.orientation.y;
    pose[5] = getmodelstate.response.pose.orientation.z;
    pose[6] = getmodelstate.response.pose.orientation.w;
    //printf("call and get pose of robot%d q:\n w: %f, x: %f, y: %f, z: %f\n", id, pose[6], pose[3], pose[4], pose[5]);
    pose_ready = true;
}

void setForPose(int id, float x, float y, float z, float qx, float qy, float qz, float qw){

    printf("set pose for robot %d...", id);

    gazebo_msgs::SetModelState setmodelstate;
    //string s;
    //stringstream ss;
    //ss<<"test"<<id;
    //ss>>s;
    char s[10];
    sprintf(s, "robot_%d", id);
    geometry_msgs::Pose posemsg;
        posemsg.position.x = x;
        posemsg.position.y = y;
        posemsg.position.z = z;
        posemsg.orientation.x = qx;
        posemsg.orientation.y = qy;
        posemsg.orientation.z = qz;
        posemsg.orientation.w = qw;
    gazebo_msgs::ModelState modelstate;
        modelstate.pose = posemsg;
        modelstate.model_name = (std::string) s;
    setmodelstate.request.model_state = modelstate;
    if (sclient.call(setmodelstate)){}
    else
        ROS_ERROR("Failed to call service");

    printf(" succeed.\n");
}

void infoCallback(const sensor_msgs::CameraInfoConstPtr &msg){
    
    //printf("cx: %lf, cy: %lf, fx: %lf, fy: %lf\n", msg->K[2], msg->K[5], msg->K[0], msg->K[4]);

    //printf("fx: %lf\n", msg->K[0]);
}

//接收数据
int recvData(const int client_fd, char buf[], int len)
{
    memset (buf, 0, len);

    int i=0;
    while(i<len){
        char buf_tem[MAXRECV];
        memset (buf_tem, 0, MAXRECV);
        int status = recv(client_fd, buf_tem, MAXRECV, 0);
        memcpy(buf+i, buf_tem, status);
        i = i+status;

        //printf("i:%d\n", i);
        //printf("len:%d\n", len);
        //printf("status:%d\n", status);

        if ( status == -1 )
        {
            printf("status == -1 errno == %s in Socket::recv\n",errno);
            return 0;
        }
        else if( status == 0 )
        {
            //stop(client_fd);
            //return 0;
            break;
        }
        else if(len<=MAXRECV+1)
        {
            break;
        }
        /*else
        {
        return status;
        }*/
    }
}

// send data. 向指定客户端发送数据
bool sendData(const int client_fd, const char *ch, const int len)
{
    int status = send(client_fd, ch, len, 0);
    if ( status == -1 )
    {
        return false;
    }
    else
    {
        return true;
    }
}

// send data. 2018-09-17
int sendTotalData(const int client_fd, const char *buf, const int len){
    
    int total = 0; // how many bytes we've sent
    int bytesleft = len; // how many we have left to send
    int n;

    while(total < len) {
        n = send(client_fd, buf+total, bytesleft, 0);
        if (n == -1) { break; }
        total += n;
        bytesleft -= n;
    }

    return n==-1 ? 0:1; // return -1 onm failure, 0 on success
}

// socket get rgbd
bool getRGBD(int client_fd){

//printf("-2\n");

    ros::spinOnce();

//printf("-1\n");

    ros::Rate rate(1); // 1hz
    //ros::Rate rate(10); // 10hz. 01-09. wrong!
    rate.sleep();

//printf("0\n");
    // rgb
    {
        int data_len = 480 * 640 * 3 * sizeof(uchar) * rbtnum;
        char* rgbData = (char *)malloc(data_len);
//if(rgbData == NULL) printf("malloc failed\n");
        while(!rgbData)
        {
            usleep(100000);
            rgbData = (char *)malloc(data_len);
        }
//printf("1\n");

        // for multi robot
        int ind = 0;
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            for (int i = 0; i < 480; ++i)
            {
                for (int j = 0; j < 640; ++j)
                {
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                    ind+=sizeof(uchar);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                    ind+=sizeof(uchar);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                    ind+=sizeof(uchar);
                }
            }
        }

//printf("2\n");

        //bool rtnCode = sendData(client_fd, rgbData, data_len);
        int rtnCode = sendTotalData(client_fd, rgbData, data_len);
        printf("rgb data %d byte send back done. rtnCode = %d\n", data_len, rtnCode);
        free(rgbData);

//printf("3\n");

    }   
    // depth
    {
        int data_len = 480 * 640 * sizeof(short) * rbtnum;
        char* depthData = (char *)malloc(data_len);
        while(!depthData)
        {
            usleep(100000);
            depthData = (char *)malloc(data_len);
        }
        
        // for multi robot
        int ind = 0;
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            for (int i = 0; i < 480; ++i)
            {
                for (int j = 0; j < 640; ++j)
                {
                    memcpy(&depthData[ind], &crt_depth_images[rid].ptr<short>(i)[j], sizeof(short));
                    ind+=sizeof(short);
                }
            }
        }

        //int rtnCode = sendData(client_fd, depthData, data_len);
        int rtnCode = sendTotalData(client_fd, depthData, data_len);
        printf("depth data %d byte send back done. rtnCode = %d\n", data_len, rtnCode);
        free(depthData);
        
//printf("4\n");
    }

    return true;
}

// socket get depth
bool getDepth(int client_fd){

    printf("this method are abandoned");
    getchar();
    
    /*

    ros::spinOnce();
    //char* depthData = (char *)malloc(480 * 640 * (sizeof(short)) * rbtnum);
    int data_len = 480 * 640 * sizeof(short) * rbtnum;
    char* depthData = (char *)malloc(data_len);
    // rbt0
    int ind = 0;
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg0.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    // rbt1
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg1.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    // rbt2
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg2.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    sendData(client_fd, depthData, data_len);
    printf("depth send back\n");
    free(depthData);

    */

    return true;
}

// socket get pose
bool getPose(int client_fd)
{
    printf("\ngetPose: ...\n");
    int data_len = 7 * sizeof(float) * rbtnum;
    char* poseData = (char *)malloc(data_len);
    while(!poseData)
    {
        usleep(100000);
        poseData = (char *)malloc(data_len);
    }
    // for multi robot
    int ind = 0;
    for (int rid = 0; rid < rbtnum; ++rid)
    {
        callForPose(rid);
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&poseData[ind], &pose[i], sizeof(float));
            ind+=sizeof(float);
        }
    }
    //int rtnCode = sendData(client_fd, poseData, data_len);
    int rtnCode = sendTotalData(client_fd, poseData, data_len);
    //printf("pose data %d byte send back done. rtnCode = %d\n", data_len, rtnCode);
    ind = 0;
    for (int rid = 0; rid < rbtnum; ++rid)
    {
        for (int i = 0; i < 7; ++i)
        {
            float temp;
            memcpy(&temp, &poseData[ind], sizeof(float));
            ind+=sizeof(float);
printf("%f ", temp);
        }
printf("\n");
    }
    free(poseData);
    printf("getPose: done.\n");
    return true;
}

void goToPose(float x0 , float y0, float qx0, float qy0, float qz0, float qw0, 
              float x1 , float y1, float qx1, float qy1, float qz1, float qw1, 
              float x2 , float y2, float qx2, float qy2, float qz2, float qw2){
    printf("\ngoToPose\n");
    //printf("set to x0=%f y0=%f x1=%f y1=%f x2=%f y2=%f\n", x0, y0, x1, y1, x2, y2);
    // rotation try
    // pose0
    callForPose(0);
    float originTheta0 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta0 = -originTheta0;
    float objectTheta0 = acos(qw0)*2;
    if(qz0<0)
        objectTheta0 = -objectTheta0;
    float dTheta0 = objectTheta0 - originTheta0;
    float dx0 = rbt_v * (x0 - pose[0]);
    float dy0 = rbt_v * (y0 - pose[1]);
    // pose1
    callForPose(1);
    float originTheta1 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta1 = -originTheta1;
    float objectTheta1 = acos(qw1)*2;
    if(qz1<0)
        objectTheta1 = -objectTheta1;
    float dTheta1 = objectTheta1 - originTheta1;
    float dx1 = rbt_v * (x1 - pose[0]);
    float dy1 = rbt_v * (y1 - pose[1]);
    // pose2
    callForPose(2);
    float originTheta2 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta2 = -originTheta2;
    float objectTheta2 = acos(qw2)*2;
    if(qz2<0)
        objectTheta2 = -objectTheta2;
    float dTheta2 = objectTheta2 - originTheta2;
    float dx2 = rbt_v * (x2 - pose[0]);
    float dy2 = rbt_v * (y2 - pose[1]);
    for (int i = 0; i < times; ++i)
    {
        float ox = 0.0, oy = 0.0, oz = 1.0;
        // pose 0
        callForPose(0);
        float crtTheta0 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta0 = -crtTheta0;
        float temp_theta0 = crtTheta0 + dTheta0/times;
        setForPose(0, pose[0], pose[1], camera_height, ox*sin(temp_theta0/2), oy*sin(temp_theta0/2), oz*sin(temp_theta0/2), cos(temp_theta0/2));
        // pose 1
        callForPose(1);
        float crtTheta1 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta1 = -crtTheta1;
        float temp_theta1 = crtTheta1 + dTheta1/times;
        setForPose(1, pose[0], pose[1], camera_height, ox*sin(temp_theta1/2), oy*sin(temp_theta1/2), oz*sin(temp_theta1/2), cos(temp_theta1/2));
        // pose 2
        callForPose(2);
        float crtTheta2 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta2 = -crtTheta2;
        float temp_theta2 = crtTheta2 + dTheta2/times;
        setForPose(2, pose[0], pose[1], camera_height, ox*sin(temp_theta2/2), oy*sin(temp_theta2/2), oz*sin(temp_theta2/2), cos(temp_theta2/2));
        // send to socket
        ros::spinOnce();
        ros::Rate rate(1);
        //ros::Rate rate(10); // 01-09.
        rate.sleep();
        //getDepth(client_fd);
        getPose(client_fd);   
        getRGBD(client_fd);
    }
    // move
    for (int i = 0; i < times; ++i){
        //0
        callForPose(0);
        float tx0 = pose[0] + dx0;
        float ty0 = pose[1] + dy0;
        //1
        callForPose(1);
        float tx1 = pose[0] + dx1;
        float ty1 = pose[1] + dy1;
        //2
        callForPose(2);
        float tx2 = pose[0] + dx2;
        float ty2 = pose[1] + dy2;
        setForPose(0, tx0, ty0, camera_height, qx0, qy0, qz0, qw0);
        setForPose(1, tx1, ty1, camera_height, qx1, qy1, qz1, qw1);
        setForPose(2, tx2, ty2, camera_height, qx2, qy2, qz2, qw2);
        // send to socket
        ros::spinOnce();
        ros::Rate rate(1);
        //ros::Rate rate(10); // 01-09.
        rate.sleep();
        //getDepth(client_fd);
        getPose(client_fd);
        getRGBD(client_fd);  
    }
    printf("goToPose: done.\n");
}

// socket move to views
bool move_to_views(int client_fd){

    printf("\nfunc move_to_views begin\n");

    // receive data 
    int data_len = rbtnum * 7 * sizeof(float);
    char* poseData = (char *)malloc(data_len);
    while(!poseData)
    {
        usleep(100000);
        poseData = (char *)malloc(data_len);
    }
    //printf("before recv\n");
    int rcv_len = recvData(client_fd, poseData, data_len); // recv
    //printf("after recv\n");
    float pass_pose[rbtnum][7];
    int ind = 0;

    // move: set poses to views
    for (int id = 0; id < rbtnum; ++id)
    {
        float set_pose[7];
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&set_pose[i], &poseData[ind], sizeof(float));
            memcpy(&pass_pose[id][i], &poseData[ind], sizeof(float));
            ind+=sizeof(float);
        }
        setForPose(id, 
            set_pose[0], set_pose[1], set_pose[2], 
            set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
        //printf("pose for robot%d: %f, %f, %f, %f, %f, %f, %f\n", id, 
        //    set_pose[0], set_pose[1], set_pose[2],
        //    set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    }

//printf("before free()\n");
    // free 
    free(poseData);
//printf("after free()\n");

    printf("func move_to_views end\n\n");

    // update rgbd and pose topic
    
    ros::spinOnce();
    ros::Rate rate(1);
    //ros::Rate rate(10); // 01-09.
    rate.sleep();

    return true;
}

// socket set pose
bool setPose(int client_fd){
    printf("\nsetPose\n");
    int data_len = rbtnum * 7 * sizeof(float);
    char* poseData = (char *)malloc(data_len);
    while(!poseData)
    {
        usleep(100000);
        poseData = (char *)malloc(data_len);
    }
//printf("before recv\n");
    int rcv_len = recvData(client_fd, poseData, data_len);
//printf("after recv\n");
    float pass_pose[rbtnum][7];
    int ind = 0;
    for (int id = 0; id < rbtnum; ++id)
    {
        float set_pose[7];
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&set_pose[i], &poseData[ind], sizeof(float));
            memcpy(&pass_pose[id][i], &poseData[ind], sizeof(float));
            ind+=sizeof(float);
        }
        // setForPose(id, 
        //     set_pose[0], set_pose[1], set_pose[2], 
        //     set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
        // printf("client pose for robot%d: %f, %f, %f, %f, %f, %f, %fn", id, 
        //     set_pose[0], set_pose[1], set_pose[2],
        //     set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    }
    goToPose(
        pass_pose[0][0], pass_pose[0][1], pass_pose[0][3], pass_pose[0][4], pass_pose[0][5], pass_pose[0][6],
        pass_pose[1][0], pass_pose[1][1], pass_pose[1][3], pass_pose[1][4], pass_pose[1][5], pass_pose[1][6],
        pass_pose[2][0], pass_pose[2][1], pass_pose[2][3], pass_pose[2][4], pass_pose[2][5], pass_pose[2][6]);
    // // write to offline
    // ofs_off<<pass_pose[0][0]<<" "<<pass_pose[0][1]<<" "<<1<<" "
    // << pass_pose[0][3]<<" "<<pass_pose[0][4]<<" "<<pass_pose[0][5]<<" "<<pass_pose[0][6]<<endl;
    // ofs_off<<pass_pose[1][0]<<" "<<pass_pose[1][1]<<" "<<1<<" "
    // << pass_pose[1][3]<<" "<<pass_pose[1][4]<<" "<<pass_pose[1][5]<<" "<<pass_pose[1][6]<<endl;
    // ofs_off<<pass_pose[2][0]<<" "<<pass_pose[2][1]<<" "<<1<<" "
    // << pass_pose[2][3]<<" "<<pass_pose[2][4]<<" "<<pass_pose[2][5]<<" "<<pass_pose[2][6]<<endl;
    free(poseData);
    printf("\nsetPose: done.\n");
    return true;
}

//vector< vector< vector<float> > > send_poses;
struct PositionInGazebo{
	float x;
	float y;
	float z;
	PositionInGazebo(float ix, float iy, float iz){
		x = ix;
		y = iy;
		z = iz;
	}
};
vector< vector< PositionInGazebo > > path_poses;
vector< PositionInGazebo > task_poses;

// set path
bool setPath(int client_fd){
	// clear and resize path_poses
	path_poses.clear();
	path_poses.resize(rbtnum);
	// recv path data
	for (int rid = 0; rid < rbtnum; ++rid)
	{	
		char len_data[sizeof(int)];
		recvData(client_fd, len_data, sizeof(int));//recv data lenth
		int data_len = 0;
		memcpy(&data_len, &len_data, sizeof(int));
		char path_data[MAXRECV];
		recvData(client_fd, path_data, data_len);// recv path data
		// memcpy to path_poses
		int path_size = ceil(data_len/3/sizeof(float));
		int ind = 0;
		for (int pid = 0; pid < path_size; ++pid)
		{
			float x, y, z;
			memcpy(&x, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			memcpy(&y, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			memcpy(&z, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			PositionInGazebo crt_p = PositionInGazebo(x, y, z);
			path_poses[rid].push_back(crt_p);
		}			
	}
	// compute path distance for each robot
	vector<float> path_dises;
	double min_dis = DBL_MAX;
	int min_rid = -1;
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		if(path_poses[rid].size() == 1){ // if dont have move path
			path_dises.push_back(0);
			continue;
		}
		float path_dis = 0;
		for (int pid = 0; pid < path_poses[rid].size()-1; ++pid)
		{
			path_dis+=
			sqrt(
			(path_poses[rid][pid].x-path_poses[rid][pid+1].x)*(path_poses[rid][pid].x-path_poses[rid][pid+1].x)
			+
			(path_poses[rid][pid].y-path_poses[rid][pid+1].y)*(path_poses[rid][pid].y-path_poses[rid][pid+1].y));
		}
		path_dises.push_back(path_dis);
	}
	// compute min distance path
	for (int rid = 0; rid < path_dises.size(); ++rid)
	{
		printf("rbt%d path distance = %f\n", rid, path_dises[rid]);
		if(min_dis>path_dises[rid]){
			min_dis = path_dises[rid];
			min_rid = rid;
		}
	}
	printf("min_dis = %f\n", min_dis);
    {//move_distance_rate
        min_rid = -1;
        min_dis *= move_distance_rate;
    }
	// move robots
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		// move for no path robot
		if(path_poses[rid].size() == 1){
			
			// callForPose(rid);
			// float dir_theta = PI/3;
   //      	// set pose
   //      	setForPose(rid, , , camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			// printf("rbt%d dont move\n", rid);
			// printf("rbt%d set to (%f, %f)\n", rid, pose[0], pose[1]);
			
			callForPose(rid);
			float obj_x=.1, obj_y=.1, crt_x = .1, crt_y = .1;
			obj_x = task_poses[rid].x;
	 		obj_y = task_poses[rid].y;
	 		crt_x = pose[0];
	 		crt_y = pose[1];
	 		float dir_theta;
	 		printf("fabs(obj_x - crt_x)=%f fabs(obj_y - crt_y)=%f\n", fabs(obj_x - crt_x), fabs(obj_y - crt_y));
	 		if(fabs(obj_x - crt_x) < 0.2 && fabs(obj_y - crt_y) < 0.2){ // task_pose == rbt pose
	 			float originTheta = acos(pose[6])*2;
		    	if(pose[5]<0)
		        	originTheta = -originTheta;
		        dir_theta = originTheta+PI/3;
		        if(dir_theta>PI)
		        	dir_theta = -PI + dir_theta - PI;
		        printf("turn PI/3\n");
	 		}else{// task_pose != rbt pose
				// compute direction
				float dx = obj_x - crt_x;
				float dy = obj_y - crt_y;
				// compute rotation
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
	 		}
	 		
	        setForPose(rid, crt_x, crt_y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d dont move\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, crt_x, crt_y);

			continue;
		}
		// move for min robot
		if(rid == min_rid){
			int node_num = path_poses[rid].size();
			// compute direction
			float dx = task_poses[rid].x - path_poses[rid][node_num-1].x;
			float dy = task_poses[rid].y - path_poses[rid][node_num-1].y;
			// float dx = path_poses[rid][node_num-1].x - path_poses[rid][node_num-2].x;
			// float dy = path_poses[rid][node_num-1].y - path_poses[rid][node_num-2].y;
			float dir_theta;
			// compute rotation
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
	        // set pose
        	setForPose(rid, path_poses[rid][node_num-1].x, path_poses[rid][node_num-1].y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d min\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, path_poses[rid][node_num-1].x, path_poses[rid][node_num-1].y);
		}
		// move for not min robot
		else{
			float obj_x=.1, obj_y=.1, crt_x = .1, crt_y = .1, lst_x=.1, lst_y=.1;
			float crt_dis = 0;
			for (int pid = 0; pid < path_poses[rid].size()-1; ++pid) // find obj position and lst position
			{
			 	float tmp_dis = 
				 	sqrt(
					(path_poses[rid][pid].x-path_poses[rid][pid+1].x)*(path_poses[rid][pid].x-path_poses[rid][pid+1].x)
					+
					(path_poses[rid][pid].y-path_poses[rid][pid+1].y)*(path_poses[rid][pid].y-path_poses[rid][pid+1].y));
			 	crt_dis += tmp_dis;
			 	//printf("	rbt%d crt_dis = %f\n", rid, crt_dis);
			 	if(crt_dis == min_dis){
			 		//printf("	rbt%d crt_dis == min_dis\n", rid);
			 		obj_x = task_poses[rid].x;
			 		obj_y = task_poses[rid].y;
			 		crt_x = path_poses[rid][pid+1].x;
			 		crt_y = path_poses[rid][pid+1].y;
			 		// obj_x = path_poses[rid][pid+1].x;
			 		// obj_y = path_poses[rid][pid+1].y;
			 		// lst_x = path_poses[rid][pid].x;
			 		// lst_y = path_poses[rid][pid].y;
			 		break;
			 	}
			 	if(crt_dis > min_dis){
			 		//printf("	rbt%d crt_dis > min_dis\n", rid);
			 		crt_dis -= tmp_dis;
			 		float res_dis = min_dis - crt_dis;
			 		{// temp use this
			 			obj_x = task_poses[rid].x;
				 		obj_y = task_poses[rid].y;
				 		crt_x = path_poses[rid][pid+1].x;
				 		crt_y = path_poses[rid][pid+1].y;
			 			// obj_x = path_poses[rid][pid+1].x;
				 		// obj_y = path_poses[rid][pid+1].y;
				 		// lst_x = path_poses[rid][pid].x;
				 		// lst_y = path_poses[rid][pid].y;
			 		}
		 			{/* dai shi xian */
			 			
		 			}

			 		break;
			 	}
			} 
			// compute direction
			float dx = obj_x - crt_x;
			float dy = obj_y - crt_y;
			float dir_theta;
			// compute rotation
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
	        // set pose
        	setForPose(rid, crt_x, crt_y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d not min\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, crt_x, crt_y);
		}
	}
	// get scans // send to socket
    ros::spinOnce();
    ros::Rate rate(1);
    //ros::Rate rate(10); // 01-09.
    rate.sleep();
    getPose(client_fd);
    getRGBD(client_fd);

	//getchar();

	printf("move correctly\n");
	return true;
}

// set up
bool scanSurroundings(int client_fd){

    printf("scanning surroundings...\n");

	//float cover_angle = PI/3;
	for (int i = 0; i < 6; ++i)
	{
		for (int rid = 0; rid < rbtnum; ++rid)
		{
			callForPose(rid);
			float originTheta = acos(pose[6])*2;
	    	if(pose[5]<0)
	        	originTheta = -originTheta;
	        float dir_theta = originTheta+PI/3;
	        if(dir_theta>PI)
	        	dir_theta = -PI + dir_theta - PI;
	        setForPose(rid, pose[0], pose[1], camera_height, 
	        	0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));

            printf("robot_%d turned 60 degree.\n", rid);
		}
		// get scans // send to socket
        printf("ros sleep and spin once to get pose and rgbd...\n");
  	  	ros::spinOnce();
        ros::Rate rate(1);
        //ros::Rate rate(10); // 01-09.
        rate.sleep();
        //printf("ros getting pose...\n");
        getPose(client_fd);
        //printf("ros getting rgbd...\n");
    	getRGBD(client_fd);
	}	

	return true;
}

// set task positions
bool setTaskPositions(int client_fd){
	task_poses.clear();

	// recv data
	int data_len = rbtnum*2*sizeof(float);
	char otp_data[MAXRECV];
	recvData(client_fd, otp_data, data_len);// recv otp data

	// memcpy to task_poses
	int ind = 0;
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		float x, y;
		memcpy(&x, &otp_data[ind], sizeof(float));
		ind+=sizeof(float);
		memcpy(&y, &otp_data[ind], sizeof(float));
		ind+=sizeof(float);

		PositionInGazebo crt_p = PositionInGazebo(x, y, camera_height);
		task_poses.push_back(crt_p);
	}			

	return true;
}

// change robot number
void change_robot_number_local(int number)
{
    // change robot number
    rbtnum = number;
    // re initialization
    {
        // frame data.
        crt_rgb_images.clear();
        crt_rgb_images.resize(rbtnum);
        crt_depth_images.clear();
        crt_depth_images.resize(rbtnum);
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            crt_rgb_images[rid] = cv::Mat(480, 640, CV_8UC3);
            crt_depth_images[rid] = cv::Mat(480, 640, CV_16UC1);
        }
        // ros topics. RGBD.
        subsColor.clear();
        subsColor.resize(rbtnum);
        subsDepth.clear();
        subsDepth.resize(rbtnum);
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            { // color
                char topicName[100];
                sprintf(topicName, "camera%d/rgb/image_raw", rid);
                //subsColor[rid] = it.subscribe(topicName, 1, boost::bind(rgbImageCallback, _1, int2str(rid)));
                subsColor[rid] = it->subscribe(topicName, 1, boost::bind(rgbImageCallback, _1, int2str(rid)));
            }
            {// depth
                char topicName[100];
                sprintf(topicName, "camera%d/depth/image_raw", rid);
                //subsDepth[rid] = it.subscribe(topicName, 1, boost::bind(depthImageCallback, _1, int2str(rid)));
                subsDepth[rid] = it->subscribe(topicName, 1, boost::bind(depthImageCallback, _1, int2str(rid)));
            }
        }
    }
    // end.
    return;
}

// change robot number
bool change_robot_number(int client_fd)
{
    // recv robot number
    int data_len = sizeof(int);
    char recv_data[MAXRECV];
    recvData(client_fd, recv_data, data_len); 
    int number = rbtnum;
    memcpy(&number, &recv_data[0], sizeof(int));
    // change number
    change_robot_number_local(number);
    printf("changed robot number: %d\n", rbtnum);
    return true;
}

// thread
void *thread(void *ptr)
{
    //unsigned int tid = (unsigned int)pthread_self(); //获取当前线程id
    int client = *(int *)ptr;
    bool stopped=false;
    while(!stopped)
    {
        char message [MAXRECV+1];
        printf("wait for a command...\n");
        if(recvData(client_fd, message, MAXRECV+1))
        {
            printf("message: %s\n", message);
            if(message!=NULL&&strlen(message)!=0)
            {
                printf("command message: %c\n", message[0]);
                switch(message[0])
                {
                    case '0':
                    {
                        printf("ask for depth\n");
                        getDepth(client_fd);
                        break;
                    }
                    case '1':
                    {      
                        printf("ask for pose\n");
                        getPose(client_fd);                      
                        break;
                    }
                    case '2':
                    {
                        printf("ask to set pose\n");
                        setPose(client_fd);
                        break;
                    }
                    case '3':
                    {
                        printf("ask for rgbd\n");
                        getRGBD(client_fd);
                        break;
                    }
                    case '4':
                    {
                    	printf("ask to set path\n");
                    	setPath(client_fd);
                    	break;
                    }
                    case '5':
                    {
                    	printf("set up surroundings\n");
                    	scanSurroundings(client_fd);
                    	break;
                    }
                    case '6':
                    {
                    	printf("ask to set task positions\n");
                    	setTaskPositions(client_fd);
                    	break;
                    }
                    case 'm':
                    {
                        printf("move_to_views\n");
                        move_to_views(client_fd);
                        break;
                    }
                    case 'e':
                    {
                        printf("command: stop the socket thread\n");
                        stopped = true;
                        break;
                    }
                    case 'n':
                    {
                        printf("check robot number\n");
                        change_robot_number(client_fd);
                        break;
                    }
                    default:
                    {
                        printf("invalid command\n");
                        break;
                    }
                }
            }
        }
        usleep(100000); // 100 ms
    }
    printf("thread stop done.\n");
    return 0;
}

// enter point
int main(int argc, char **argv){

    printf("this new workspace\n");
    //getchar();

    // set up ros env
    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    it = new image_transport::ImageTransport(n); // need release manually.

    // // write
    // ofs_off.open("/home/dsy/catkin_ws/src/virtual_scan/data/offline/pose_history.txt");
    // //ofs_off.close();

    // input robot number
    {
        ifstream ifs("/home/bamboo/catkin_ws/src/virtual_scan/config_nrbt.txt");
        ifs>>rbtnum;
        ifs.close();
        printf("robot number = %d\n", rbtnum);
    }

    // initialization
    {
        // frame data.
        crt_rgb_images.clear();
        crt_rgb_images.resize(rbtnum);
        crt_depth_images.clear();
        crt_depth_images.resize(rbtnum);
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            crt_rgb_images[rid] = cv::Mat(480, 640, CV_8UC3);
            crt_depth_images[rid] = cv::Mat(480, 640, CV_16UC1);
        }
        // ros topics. RGBD.
        subsColor.clear();
        subsColor.resize(rbtnum);
        subsDepth.clear();
        subsDepth.resize(rbtnum);
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            { // color
                char topicName[100];
                sprintf(topicName, "camera%d/rgb/image_raw", rid);
                //subsColor[rid] = it.subscribe(topicName, 1, boost::bind(rgbImageCallback, _1, int2str(rid)));
                subsColor[rid] = it->subscribe(topicName, 1, boost::bind(rgbImageCallback, _1, int2str(rid)));
            }
            {// depth
                char topicName[100];
                sprintf(topicName, "camera%d/depth/image_raw", rid);
                //subsDepth[rid] = it.subscribe(topicName, 1, boost::bind(depthImageCallback, _1, int2str(rid)));
                subsDepth[rid] = it->subscribe(topicName, 1, boost::bind(depthImageCallback, _1, int2str(rid)));
            }
        }
    }

    // kinect calibration
    //ros::Subscriber calib_sub = n.subscribe("camera/depth/camera_info", 1, infoCallback);

    // services: pose
    gclient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    sclient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // // test callback func
    // ros::spin();
    // return 0;

    // socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)//建立 socket
    {
        perror("socket");
        exit(1);
    }
    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(SERVPORT);
    my_addr.sin_addr.s_addr = INADDR_ANY; //表示监听任何地址
    bzero(&(my_addr.sin_zero),8);
    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) //将本机地址与建立的套接字号进行绑定
    {
        perror("bind");
        exit(1);
    }
    if (listen(sockfd, BACKLOG) == -1) //开始监听
    {
        perror("listen");
        exit(1);
    }

    // set keepalive
    {
        int keepAlive = 1; // enable keepalive
        int keepIdle = 60; // 如该连接在60秒内没有任何数据往来,则进行探测 
        int keepInterval = 5; // 探测时发包的时间间隔为5 秒
        int keepCount = 3; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发
        setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
        setsockopt(client_fd, SOL_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle));
        setsockopt(client_fd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
        setsockopt(client_fd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount)); 
    }

    // disable nagle
    {
        int flag = 1;
        if(setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0){
            printf("disable nagle failed\n");
        }
    }

    while (1)
    {
        sin_size = sizeof(my_addr);

        printf("%s\n", "waiting for a connection");

        client_fd = accept(sockfd, (struct sockaddr*)&remote_addr, (socklen_t *) &sin_size); //接收客户端的连接

        if (client_fd == -1)
        {
            perror("failed");
            continue;
        }

        printf("%s\n", "received a connection");
        
        // set keepalive
        {
            int keepAlive = 1; // enable keepalive
            int keepIdle = 60; // 如该连接在60秒内没有任何数据往来,则进行探测 
            int keepInterval = 5; // 探测时发包的时间间隔为5 秒
            int keepCount = 3; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发
            setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
            setsockopt(client_fd, SOL_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle));
            setsockopt(client_fd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
            setsockopt(client_fd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount)); 
        }

        // disable nagle
        {
            int flag = 1;
            if(setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0){
                printf("disable nagle failed\n");
            }
        }

        pthread_t id;
        int ret = pthread_create(&id, NULL, thread, &client_fd);
        if(ret!=0) 
        {
            printf("Create pthread error: %s\n", strerror(ret));
            continue;
        }

        printf("succeed connected.\n");

        usleep(100000);
    }

    return 0;
}