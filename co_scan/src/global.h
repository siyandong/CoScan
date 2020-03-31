#pragma once
#include <string>
#include <string.h>

const std::string domainFilePath = "domain";
const std::string holeFilePath = "hole";
const std::string resultFilePath = "diff_result";
const std::string communicateFile = "communication";
const std::string poly_diff = "devel_isolated/co_scan/lib/co_scan/poly_diff";
const std::string cdt_obj_path = "cdt.obj";
const std::string cdt_off_path = "cdt.off";

#define PI 3.1415926
// cluster compact constraint
extern double compactParam; // voxels. todo: config file.
extern double g_distance_step; // meter.
// angle constraint
extern double g_angleDifference;
// dense interpolation
extern bool g_fpInterpolation;

#include "../include/se2/se2.h"
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

// trajectories
extern std::vector<std::vector<Point_2>> g_rbtTrajectories; // need initialize
extern std::vector<std::vector<iro::SE2>> g_camTrajectories; // need initialize
extern float g_camera_height; // meter.

extern int g_plan_iteration;


// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern std::vector<cv::Point> g_scene_boundary;