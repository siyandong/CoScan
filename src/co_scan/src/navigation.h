#pragma once
// std
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <vector>
#include <list>
#include <set>

/*
// cgal
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/intersections.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef K::Point_2 												  Point_2;
typedef CGAL::Polygon_2<K>  									  Polygon_2;
struct FaceInfo2
{
	FaceInfo2(){}
	int nesting_level;
	bool in_domain()
	{
		return nesting_level % 2 == 1;
	}
};
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K>   Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>       Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CGAL::Polygon_with_holes_2<K>                             Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                                Pwh_list_2;
//*/

// headers
#include "global.h" 			// global variables
#include "data_engine.h"		// process data
#include "distance.h"			// distance cost metric
#include "omt_solver.h"			// solve omt problem
#include "tsp/tsp.h"			// tsp
#include "tsp/usage.h"			// tsp
#include "tsp/twoOpt.h"			// tsp
#include "tsp/MyThread.h"		// tsp
#include "path_optimization.h"	// solve path
#define CPS CLOCKS_PER_SEC
#define NUM_THREADS 1

const int frontier_exploration_sample_range_pixel = 10; //10 pixel = 0.5m
const int offset_size = (int)(0.15/map_cellsize); // offset to avoid collision
// random site num of CDT
const int random_site_num = 200; // CDT resolution. 50~200 is OK.

// next best view
struct NextBestView
{
	iro::SE2 pose;
	Eigen::Vector3i target;
	int type = -1; // 0-object 1-frontier
	int index = -1; // index in objects/frontiers
	NextBestView()
	{
		pose = iro::SE2(0, 0, 0);
		target = Eigen::Vector3i(0, 0, 0);
	}
	NextBestView(iro::SE2 p){
		pose = p;
		target = Eigen::Vector3i(0, 0, 0);
	}
	NextBestView(iro::SE2 p, Eigen::Vector3i t, int tp, int ind=-1){
		pose = p;
		target = t;
		type = tp;
		index = ind;
	}
	NextBestView(iro::SE2 p, int tp, int ind){
		pose = p;
		target = Eigen::Vector3i(0, 0, 0);
		type = tp;
		index = ind;
	}
	NextBestView(const NextBestView & other) // 2018-11-12.
	{
		pose = iro::SE2(other.pose);
		target = Eigen::Vector3i(other.target);
		type = other.type;
		index = other.index;
		//cerr << "constructor: input const NextBestView &" << endl;
	}
	NextBestView & operator = (const NextBestView & other) // 2018-11-12.
	{
		pose = iro::SE2(other.pose);
		target = Eigen::Vector3i(other.target);
		type = other.type;
		index = other.index;
		//cerr << "constructor: operator =" << endl;
		return *this;
	}
};

class ScanningTask
{
public:
	Eigen::Vector3d target_position; // se2 coordinate
	int target_type = -1; // 0-quality 1-exploration
	NextBestView view; // nbv

	ScanningTask()
	{
		// none paras
	}

	ScanningTask(NextBestView v)
	{
		view = v;
	}

	ScanningTask(int type, NextBestView v)
	{
		this->target_type = type;
		this->view = v;
	}

	ScanningTask(Eigen::Vector3d posi, int type, NextBestView v)
	{
		this->target_position = posi;
		this->target_type = type;
		this->view = v;
	}

	//vector<NextBestView> computeTaskView(Polygon_2 boundary, vector<Polygon_2> holes);

};

// frontiers
struct FrontierElement
{
	int id = -1;
	Point_2 position;
	FrontierElement(Point_2 p)
	{
		position = p;
	}
};

class Navigation
{
public:

	int m_max_task_num = 30;

	std::vector<cv::Point> m_boundary;
	std::vector<cv::Point> m_frontiers;
	std::vector<Point_2> m_frontiers_p2;
	std::vector<std::vector<cv::Point>> m_holes;
	int rbt_num;
	std::vector<Point_2> m_robot_sites; 

	// data engine
	DataEngine* m_p_de;
	// distance metric
	DistanceMetric m_metric;

	// todo: organize temp variables...
	std::vector<FrontierElement> m_frontierList;
	std::set<Point_2> task_maybe_invalid;
	std::vector<NextBestView> m_valid_object_nbvs; // valid nbvs for all objects in objectList
	std::vector<int> m_task_index_in_valid_object_nbvs; // task indexes

	std::vector<std::vector<iro::SE2>> m_robot_move_views; // 记录规划出的机器人移动路径，坐标系为OT坐标系，包括当前位置，所以至少有一个节点(机器人当前位置)
	std::vector<std::vector<iro::SE2>> m_robot_move_nodes; // 机器人移动路径的每个节点，至少有一个节点(机器人当前位置)
	std::vector<std::vector<bool>> m_robot_move_nodes_nbv_sign; // 机器人移动路径的每个节点，记录是否为NBV节点
	std::vector<std::vector<iro::SE2>> m_sync_move_paths; // 同步控制机器人移动的每个节点

	// constructor
	Navigation(DataEngine& de, int paramK=6)
	{
		m_p_de = &de;
		rbt_num = m_p_de->rbt_num;
		m_max_task_num = m_p_de->rbt_num * paramK;
		return;
	}

	// extract boundary, holes, and frontiers
	void processCurrentScene();

	// polygon simplification. reduce number of vertexes.
	bool simplifyPolygon(Polygon_2 & poly, const double colinearThresh, bool addNoise);

	// polygon difference. exact difference of two polygons. use CGAL exact kernel. sometimes CGAL doesn't work...
	Pwh_list_2 differenceCGALExactKernel(Polygon_with_holes_2 domain, Polygon_2 hole);

	// load and check boundary
	Polygon_2 load_and_check_boundary();

	// load and check holes
	std::vector<Polygon_2> load_and_check_holes(Polygon_2 boundary, std::vector<Polygon_2> & origin_holes);

	// correction of boundary and holes: double check boundary&holes together, remove overlaps, then reset boundary and holes
	void correct_boundary_holes(Polygon_2 & boundary, std::vector<Polygon_2> & holes);

	// generate robot move domain
	bool robotMoveDomainProcess(Polygon_2 & boundary, std::vector<Polygon_2> & holes, std::vector<Polygon_2> & origin_holes);

	// CDT
	bool generateMoveDomainCDT(Polygon_2 boundary, std::vector<Polygon_2> holes, std::vector<Polygon_2> origin_holes, std::vector<ScanningTask> tasks, CDT & cdt);

	// load and check frontiers, saved in the variable frontiers
	std::vector<Point_2> load_and_check_frontiers(Polygon_2 & boundary, std::vector<Polygon_2> & holes);

	// compute location map
	cv::Mat computeLoationMap();

	// remove invalid frontiers, save valid frontiers to frontier list.
	std::vector<int> preprocess_frontiers(Polygon_2 outer, std::vector<Polygon_2> iner);

	// check view ray validness(without occlusion) 2018-09-12.
	bool checkViewRayValidness(cv::Point source, cv::Point target);

	// frustum contour.
	vector<cv::Point> get_frustum_contuor(iro::SE2 pose);

	// exe func: return nbvs
	std::vector<NextBestView> extractQualityTaskViews(Polygon_2 boundary, std::vector<Polygon_2> holes);

	// NBV for frontiers. 2018-09-22.
	std::vector<NextBestView> generateViewsFrontiers(bool enableSampling=false, double sampleRate=0.5);

	// tasks func
	std::vector<NextBestView> compute_frontier_tasks(Polygon_2 boundary, std::vector<Polygon_2> holes);

	// extract tasks
	bool extractTasks(Polygon_2 boundary, std::vector<Polygon_2> holes, std::vector<ScanningTask> & tasks);

	// TSP, include the first node for current pose
	std::vector<int> TSP_path(std::vector<Point_2> points, std::vector<vector<double>> weights);

	// OMT with TSP
	void OMT_TSP();

/*
	// check trojectories
	void Navigation::checkTrojectories();
//*/

	// compute m_robot_move_nodes
	bool compute_robot_move_nodes(std::vector<double> & distances);

	// path_occlusion_check
	bool path_occlusion_check(cv::Mat background, cv::Point beg, cv::Point end);

	// check Point_2 equal
	bool points_equal(Point_2 p1, Point_2 p2);

	// check se2 equal
	bool se2_equal(iro::SE2 p1, iro::SE2 p2);

	// compute m_sync_move_paths 
	bool compute_sync_move_paths(std::vector<double> distances);

	// uniform sample nodes from nodes inside length
	std::vector<iro::SE2> uniformSampleWithNBVInfo(const int robot_id, const double step, const double length);

	// camera trajectory interpolation: delta distance/angle constraints.
	void trajectoryInterpolation(std::vector<iro::SE2> & path, double dAngle = PI / 12);

	// traj opt
	void Trajectory_Optimization();

	// ask robot to move and scan 
	void moveRobotsAndScan();

	// visualization. 2018-11-16.
	bool visualizeScan(std::vector<iro::SE2> current_views, int vid);

/*
	// test
	void test()
	{
		Point_2 source(667.085, -562.966);
		vector<Point_2> targets;
		targets.push_back(Point_2(20, -20));
		targets.push_back(Point_2(652.233, -636.267));
		DistanceMetric dm;
		vector<double> distances = dm.getGeodesicDistances(source, targets);
		for (int i = 0; i < distances.size(); ++i)
		{
			cerr<<"distances "<<i<<" "<<distances[i]<<endl;
		}

		return;
	}
//*/

	// finished.
	void scanFinished();
};
