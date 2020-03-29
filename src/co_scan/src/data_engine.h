#pragma once
// std
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <vector>
// socket
#include <sys/wait.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <pthread.h>
#include <thread>  
#include <arpa/inet.h>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// octomap 
#include <octomap/octomap.h>
// eigen
#include "../include/eigen/Eigen/Dense"
#include "../include/eigen/Eigen/src/Geometry/Quaternion.h"
// se2
#include "../include/se2/se2.h"
// my headers
#include "global.h"

// 2D recon
struct CellMap{
	cv::Point2f coordinate;
	bool isScanned = false;
	bool isFree = false;
	bool isOccupied = false;
	//bool onFrontier = false;
};
const float map_cellsize = 0.05;
const int map_rows = 1200;
const int map_cols = 1200;
const float xmax_oc_g = 30;
const float xmin_oc_g = -30;
const float zmax_oc_g = 30;
const float zmin_oc_g = -30;
// grid project height 
const double project_max_height = 1.5; // meter
const double project_min_height = 0.15; // meter
const double free_voxel_porject_height = project_max_height; // meter, this variable does not make sense
class Recon2D
{
public:
	CellMap** m_cellmap;
	Recon2D()
	{
		m_cellmap = new CellMap*[map_rows];
		for (int rid = 0; rid < map_rows; ++rid)
			m_cellmap[rid] = new CellMap[map_cols];
		for (int rid = 0; rid < map_rows; ++rid)
		{
			for (int cid = 0; cid < map_cols; ++cid)
			{
				m_cellmap[rid][cid].isScanned = false;
				m_cellmap[rid][cid].isFree = false;
				m_cellmap[rid][cid].isOccupied = false;
			}
		}
		return;
	}
	~Recon2D() // todo:
	{
		// delete
	}
};
// 3D recon
class Recon3D
{
public:
    // 
    octomap::OcTree * m_tree;
    //
    Recon3D()
    {
        m_tree = new octomap::OcTree(map_cellsize);
        return;
    }
    Recon3D(float resolution)
    {
        m_tree = new octomap::OcTree(resolution);
        return;
    }
    //
    ~Recon3D()
    {
        delete m_tree;
        return;
    }
};

// process scanning data
class DataEngine
{
public:
	// camera internal params
	const double camera_factor = 1000;
	const double camera_cx = 320.5;
	const double camera_cy = 240.5;
	const double camera_fx = 554.38;
	const double camera_fy = 554.38;
	const int scan_max_range = 3000; // mm
	// socket
	#define PORT 3333
	const int MAXRECV = 10240;
	const int frame_rows = 480;
	const int frame_cols = 640;
	char *server_ip = "192.168.180.128";
	int sockClient;
	//bool m_sock_success = false;
	// robot
	int rbt_num = 3;
	std::vector<cv::Mat> m_rgb;
	std::vector<cv::Mat> m_depth;
	std::vector<std::vector<float>> m_pose; 	// 7 value: x, y, z, q()
	std::vector<std::vector<float>> m_set_pose; // 7 value: x, y, z, q()
	// scene recon
    Recon3D m_recon3D;
    Recon2D m_recon2D;
    // 2d frustum
    std::vector<std::vector<cv::Point>> m_frustum_contours;
    // 2d free
    std::vector<std::vector<cv::Point>> m_free_space_contours2d;
    // 2d pose
    std::vector<iro::SE2> m_pose2d;

	// constructor
	DataEngine(int r_num)
	{
		rbt_num = r_num;
		// frames
		m_rgb.clear();
		m_depth.clear();
		for (int i = 0; i < rbt_num; i++)
		{
			m_rgb.push_back(cv::Mat(frame_rows, frame_cols, CV_8UC3));
			m_depth.push_back(cv::Mat(frame_rows, frame_cols, CV_16UC1));
		}
		// poses
		m_pose.clear();
		m_pose.resize(rbt_num);
		for (int i = 0; i < rbt_num; ++i)
		{
			m_pose[i].resize(7);
		}
		//
		m_frustum_contours.clear();
		m_frustum_contours.resize(rbt_num);
		//
		m_free_space_contours2d.clear();
		m_free_space_contours2d.resize(rbt_num);
		//
		m_pose2d.clear();
		m_pose2d.resize(rbt_num);
		// finished.
		return;
	}

	// initialize, not finished.
	int initialize()
	{
		// init
		create_connection();

		//todo

		// finished.
		return 0;
	}

	// initialization: connect socket.
	bool create_connection();

	// get rgbd 
	bool getRGBDFromServer();
	// rcv rgbd from server
	bool rcvRGBDFromServer();
	// get pose
	bool getPoseFromServer();
	// rcv robot pose
	bool rcvPoseFromServer();
	// ask to set up surroundings, use to initialization
	void scanSurroundingsCmd();
	// move to views
	bool socket_move_to_views(std::vector<std::vector<double>> poses);

	// insert a frame 2 tree
	void insertAFrame2Tree(cv::Mat & depth, std::vector<float> pose);
	
	// fuse scans by multi-robot, update robot poses
	void fuseScans2MapAndTree();

	// compute ideal frustum
	std::vector<cv::Point> loadIdealFrustum(Eigen::MatrixXd r, Eigen::Vector3d t);

	// fill trivial holes in known region.
	void fillTrivialHolesKnownRegion();

	// set up scan envir
	void SetUpSurroundings();

	// find free contours that octomap cant handle, not finished
	void findExtraFreeSpace(int rid, cv::Mat depth, std::vector<float> pose);

	// coordinate system transfer
	std::pair<Eigen::MatrixXd, Eigen::Vector3d> coord_trans_7f_rt(std::vector<float> pose);

	// coordinate system transfer
	iro::SE2 coord_trans_7f_se2(std::vector<float> pose);

	// coordinate system transfer: se22gazebo
	std::vector<double> coord_trans_se22gazebo(iro::SE2 pose);

	// coordinate transformation: oc2cell return (row, col)
	Eigen::Vector2i coord_trans_oc2cell(Eigen::Vector3d p_oc);

	// coordinate transformation: oc2cell return (row, col)
	Eigen::Vector2i coord_trans_oc2cell(Eigen::Vector3d p_oc, double node_size, int scale);

	// project octree 2 map
	void projectOctree2Map();

	// vis
	cv::Mat visCellMap();

	// show
	cv::Mat showCellMap();

	// show
	cv::Mat showStatement();

	// test
	void test()
	{
/*
		getPoseFromServer();
		getRGBDFromServer();
		for (int rid = 0; rid < rbt_num; ++rid)
		{
			insertAFrame2Tree(m_depth[rid], m_pose[rid]);
			char output_dir[100];
			sprintf(output_dir, "/home/bamboo/catkin_ws/src/co_scan/temp/dsy_octree_%d.bt", rid);
			m_recon3D.m_tree->writeBinary(output_dir);
		}
//*/
		// set up initial region. ok.
		SetUpSurroundings();
		m_recon3D.m_tree->writeBinary("/home/bamboo/catkin_ws/src/co_scan/temp/dsy_octree.bt");
//*/
/*
		// test se2. ok.
		iro::SE2 view2d(1.2, -3.4, -3.15);
		std::cerr<<view2d.translation().x()<<std::endl;
		std::cerr<<view2d.translation().y()<<std::endl;
		std::cerr<<view2d.rotation().angle()<<std::endl;
//*/

		// vis
		visCellMap();
	}
};