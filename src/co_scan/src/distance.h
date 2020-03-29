#pragma once
// std
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <vector>
#include <list>
#include <set>
// eigen
#include <Eigen/Dense>
// other headers
#include "global.h"
#include "geodesic/Xin_Wang.h" 

// todo: organize distance funcs.

class DistanceMetric
{
public:

	CRichModel* m_geodesic_domain;
	int m_geodesic_domain_version = 0;

	DistanceMetric()
	{
		m_geodesic_domain = new CRichModel(cdt_obj_path);
	}
	~DistanceMetric()
	{
		delete m_geodesic_domain;
	}

	// euclidean distance
	double get_euclidean_distance(Point_2 p1, Point_2 p2);
	// euclidean distance
	double get_euclidean_distance(cv::Point p1, cv::Point p2);
	// euclidean distance
	double get_euclidean_distance(Point_2 p1, Point_2 p2, vector<Point_2> & path);

	// getGeodesicDistances
	vector<double> GeodesicDistances(Eigen::Vector2d source, vector<Eigen::Vector2d> targets);
	// getGeodesicDistances
	vector<double> getGeodesicDistances(Point_2 source, vector<Point_2> targets);
	// get_geodesic_distance_fast initialization.
	void get_geodesic_distance_fast_initialization();
	// geodesic distance fast, need initialization.
	double get_geodesic_distance_fast(Point_2 p0, Point_2 p1);
	// geodesic distance
	double get_geodesic_distance(Point_2 p0, Point_2 p1, vector<Point_2> & path);
};

// euclidean distance
double DistanceMetric::get_euclidean_distance(cv::Point p1, cv::Point p2)
{
	Eigen::Vector2d v1(p1.x, p1.y);
	Eigen::Vector2d v2(p2.x, p2.y);
	return (v1 - v2).norm();
}

// euclidean distance
double DistanceMetric::get_euclidean_distance(Point_2 p1, Point_2 p2)
{
	Eigen::Vector2d v1(p1.x(), p1.y());
	Eigen::Vector2d v2(p2.x(), p2.y());
	return (v1 - v2).norm();
}

// euclidean distance
double DistanceMetric::get_euclidean_distance(Point_2 p1, Point_2 p2, vector<Point_2> & path)
{
	Eigen::Vector2d v1(p1.x(), p1.y());
	Eigen::Vector2d v2(p2.x(), p2.y());
	path.clear();
	path.push_back(p1);
	path.push_back(p2);
	return (v1 - v2).norm();
}

// geodesic distance
double DistanceMetric::get_geodesic_distance(Point_2 p0, Point_2 p1, vector<Point_2> & path)
{
	// load model
	CRichModel m_model(cdt_obj_path);
	m_model.LoadModel();
	// find source index in model
	Point_2 source(p0.x(), p0.y()); // source point := p0
	int source_index = -1;
	// find source point id
	for (int i = 0; i < m_model.GetNumOfVerts(); i++)
	{
		if (ceil(m_model.m_Verts[i].x) == ceil(source.x()) && ceil(m_model.m_Verts[i].y) == ceil(source.y())){
			source_index = i;
			break;
		}
	}
	if (source_index == -1){
		double min_dis = DBL_MAX;
		int min_index = -1;
		for (int i = 0; i < m_model.GetNumOfVerts(); i++)// find the closet point instead
		{
			double crt_dis = get_euclidean_distance(Point_2(m_model.m_Verts[i].x, m_model.m_Verts[i].y), source);
			if (crt_dis < min_dis){
				min_dis = crt_dis;
				min_index = i;
			}
		}
		source_index = min_index;
	}
	if (source_index == -1){
		cout << "source point: " << source.x() << " " << source.y() << endl;
		cout << "error in " << __FUNCTION__ << ", source index cant find, input to exit" << endl;
		getchar(); getchar(); getchar(); // dsy
		exit(-1);
	}
	// geodesic algorithm
	CXin_Wang alg(m_model, source_index);
	alg.Execute();
	auto distanceField = alg.GetDistanceField();
	// find target index in model
	Point_2 target(p1.x(), p1.y()); // target point := p1
	int target_index = -1;
	// find task point id
	for (int i = 0; i < m_model.GetNumOfVerts(); i++)
	{
		if (ceil(m_model.m_Verts[i].x) == ceil(target.x()) && ceil(m_model.m_Verts[i].y) == ceil(target.y())){
			target_index = i;
			break;
		}
	}
	if (target_index == -1){
		double min_dis = DBL_MAX;
		int min_index = -1;
		for (int i = 0; i < m_model.GetNumOfVerts(); i++)// find the closet point instead
		{
			double crt_dis = get_euclidean_distance(Point_2(m_model.m_Verts[i].x, m_model.m_Verts[i].y), target);
			if (crt_dis < min_dis){
				min_dis = crt_dis;
				min_index = i;
			}
		}
		target_index = min_index;
	}
	if (target_index == -1){
		cout << "task point: " << target.x() << " " << target.y() << endl;
		cout << "error in " << __FUNCTION__ << ", task index cant find, input to exit" << endl;
		getchar(); getchar(); getchar(); // dsy
		exit(-1);
	}
	// distance result
	double distance_result = distanceField[target_index];
	// distance path 
	path.clear();
	auto path_result = alg.BacktraceShortestPath(target_index);
	for (int i = 0; i < path_result.size(); i++)
		path.push_back(Point_2(path_result[i].Get3DPoint(m_model).x, path_result[i].Get3DPoint(m_model).y));
	if (ceil(path.front().x()) != ceil(p0.x()) || ceil(path.front().y()) != ceil(p0.y()))
		path.front() = p0;
	if (ceil(path.back().x()) != ceil(p1.x()) || ceil(path.back().y()) != ceil(p1.y()))
		path.back() = p1;
	return distance_result;
}

// GeodesicDistances
vector<double> DistanceMetric::GeodesicDistances(Eigen::Vector2d source, vector<Eigen::Vector2d> targets)
{
	Point_2 s(source.x(), source.y());
	vector<Point_2> t(targets.size());
	for (int idx = 0; idx < targets.size(); idx++)
		t[idx] = Point_2(targets[idx].x(), targets[idx].y());
	return getGeodesicDistances(s, t);
}

// getGeodesicDistances
vector<double> DistanceMetric::getGeodesicDistances(Point_2 source, vector<Point_2> targets)
{
	// timing 
	double t1 = clock();

	// set up
	vector<double> stDistances;
	for (int tid = 0; tid < targets.size(); tid++)
		stDistances.push_back(0);

	// load model
	CRichModel m_model(cdt_obj_path);
	m_model.LoadModel();

	// find source index in model
	int source_index = -1;
	// find source point id
	for (int i = 0; i < m_model.GetNumOfVerts(); i++)
	{
		if (ceil(m_model.m_Verts[i].x) == ceil(source.x()) && ceil(m_model.m_Verts[i].y) == ceil(source.y())){
			source_index = i;
			break;
		}
	}
	if (source_index == -1){
		double min_dis = DBL_MAX;
		int min_index = -1;
		for (int i = 0; i < m_model.GetNumOfVerts(); i++)// find the closet point instead
		{
			double crt_dis = get_euclidean_distance(Point_2(m_model.m_Verts[i].x, m_model.m_Verts[i].y), source);
			if (crt_dis < min_dis){
				min_dis = crt_dis;
				min_index = i;
			}
		}
		source_index = min_index;
	}
	if (source_index == -1)
	{
		cerr << "source point: " << source.x() << " " << source.y() << endl;
		cerr << "error in " << __FUNCTION__ << ", source index cant find, input to exit" << endl;
		getchar(); getchar(); getchar(); // dsy
		exit(-1);
	}

	// geodesic algorithm
	CXin_Wang alg(m_model, source_index);
	alg.Execute();
	auto distanceField = alg.GetDistanceField();

	// retrival distances
	for (int tid = 0; tid < targets.size(); tid++)
	{
		// find target index in model
		Point_2 target(targets[tid].x(), targets[tid].y()); // target point
		int target_index = -1;
		// find task point id
		for (int i = 0; i < m_model.GetNumOfVerts(); i++)
		{
			if (ceil(m_model.m_Verts[i].x) == ceil(target.x()) && ceil(m_model.m_Verts[i].y) == ceil(target.y())){
				target_index = i;
				break;
			}
		}
		if (target_index == -1){
			double min_dis = DBL_MAX;
			int min_index = -1;
			for (int i = 0; i < m_model.GetNumOfVerts(); i++)// find the closet point instead
			{
				double crt_dis = get_euclidean_distance(Point_2(m_model.m_Verts[i].x, m_model.m_Verts[i].y), target);
				if (crt_dis < min_dis){
					min_dis = crt_dis;
					min_index = i;
				}
			}
			target_index = min_index;
		}
		if (target_index == -1)
		{
			cerr << "task point: " << target.x() << " " << target.y() << endl;
			cerr << "error in " << __FUNCTION__ << ", target index can't find, input to exit" << endl;
			getchar(); getchar(); getchar(); // dsy
			exit(-1);
		}
		// distance result
		stDistances[tid] = distanceField[target_index];
	}

	// timing 
	double t2 = clock();
	//cerr << "geodesic timing " << t2 - t1 << " ms" << endl;

	return stDistances;
}

// get_geodesic_distance_fast initialization
void DistanceMetric::get_geodesic_distance_fast_initialization()
{
	// load model
	m_geodesic_domain = new CRichModel(cdt_obj_path);
	m_geodesic_domain->LoadModel();
	//m_geodesic_domain_version = exe_count; // todo.
	return;
}

// geodesic distance fast, need initialization.
double DistanceMetric::get_geodesic_distance_fast(Point_2 p0, Point_2 p1)
{
	// find source index in model
	Point_2 source(p0.x(), p0.y()); // source point := p0
	int source_index = -1;
	// find source point id
	for (int i = 0; i < m_geodesic_domain->GetNumOfVerts(); i++)
	{
		if (ceil(m_geodesic_domain->m_Verts[i].x) == ceil(source.x()) && ceil(m_geodesic_domain->m_Verts[i].y) == ceil(source.y())){
			source_index = i;
			break;
		}
	}
	if (source_index == -1){
		double min_dis = DBL_MAX;
		int min_index = -1;
		for (int i = 0; i < m_geodesic_domain->GetNumOfVerts(); i++)// find the closet point instead
		{
			double crt_dis = get_euclidean_distance(Point_2(m_geodesic_domain->m_Verts[i].x, m_geodesic_domain->m_Verts[i].y), source);
			if (crt_dis < min_dis){
				min_dis = crt_dis;
				min_index = i;
			}
		}
		source_index = min_index;
	}
	if (source_index == -1){
		cout << "source point: " << source.x() << " " << source.y() << endl;
		cout << "error in " << __FUNCTION__ << ", source index cant find, input to exit" << endl;
		getchar(); getchar(); getchar(); // dsy
		exit(-1);
	}
	// geodesic algorithm
	CXin_Wang alg(*m_geodesic_domain, source_index);
	alg.Execute();
	auto distanceField = alg.GetDistanceField();
	// find target index in model
	Point_2 target(p1.x(), p1.y()); // target point := p1
	int target_index = -1;
	// find task point id
	for (int i = 0; i < m_geodesic_domain->GetNumOfVerts(); i++)
	{
		if (ceil(m_geodesic_domain->m_Verts[i].x) == ceil(target.x()) && ceil(m_geodesic_domain->m_Verts[i].y) == ceil(target.y())){
			target_index = i;
			break;
		}
	}
	if (target_index == -1){
		double min_dis = DBL_MAX;
		int min_index = -1;
		for (int i = 0; i < m_geodesic_domain->GetNumOfVerts(); i++)// find the closet point instead
		{
			double crt_dis = get_euclidean_distance(Point_2(m_geodesic_domain->m_Verts[i].x, m_geodesic_domain->m_Verts[i].y), target);
			if (crt_dis < min_dis){
				min_dis = crt_dis;
				min_index = i;
			}
		}
		target_index = min_index;
	}
	if (target_index == -1){
		cout << "task point: " << target.x() << " " << target.y() << endl;
		cout << "error in " << __FUNCTION__ << ", task index cant find, input to exit" << endl;
		getchar(); getchar(); getchar(); // dsy
		exit(-1);
	}
	// distance result
	double distance_result = distanceField[target_index];
	return distance_result;
}
