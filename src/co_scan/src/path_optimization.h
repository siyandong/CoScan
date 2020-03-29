#pragma once
// std
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <vector>
#include <list>
#include <set>
#include <unistd.h> 				// sleep
// other headers
#include "global.h"					// global variables
#include "data_engine.h"			// scene reconstruction
#include "../include/LBFGS/LBFGS.h"	// solve path optimizaion

using namespace std;
using namespace LBFGSpp;

cv::Mat dist;
vector<int> fixed_node_indexes;
vector<int> fixed_variabe_indexes;
double distanceField[map_rows][map_cols];

// distance field
void initDistanceField(DataEngine* p_de);

// get field value
double fieldIntensity(Eigen::Vector2d xy);

// get distance from obstacles
double getValueDistanceField(int r, int c);

// get field gradient
Eigen::Vector2d fieldGradient(Eigen::Vector2d xy);

// rho
double rho(Eigen::Vector2d xy);

// grho
Eigen::Vector2d grho(Eigen::Vector2d xy);

// objective func
double energyFunc(const Eigen::VectorXd& x, Eigen::VectorXd& grad);

// check if path cross obstacles.
bool crossObstacles(DataEngine* p_de, int cr, int cc, int lr, int lc);

// get distance from obstacles
float getValueDist(int r, int c);

// optimization
bool Optimize_Path(DataEngine* p_de, vector<iro::SE2> & path);

// distance field
void initDistanceField(DataEngine* p_de)
{
	// distance transform
	cv::Mat map_mat = p_de->visCellMap();
	cv::Mat binary_mat(map_rows, map_cols, CV_8UC1);
	for (int i = 0; i < binary_mat.rows; i++)
	{
		for (int j = 0; j < binary_mat.cols; j++)
		{
			binary_mat.ptr<uchar>(i)[j] = 0;
			if (map_mat.ptr<cv::Vec3b>(i)[j][1] == 255)
				binary_mat.ptr<uchar>(i)[j] = 255;
		}
	}
	cv::threshold(binary_mat, binary_mat, 0, 255, cv::THRESH_BINARY);
	cv::distanceTransform(binary_mat, dist, CV_DIST_L2, 3);
	cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX); // normalize distance field.
	// dist.type() = CV_32F
	//cv::imshow("normalized distance field", dist);
	//cv::waitKey(0);
	return;
}

// get distance from obstacles
double getValueDistanceField(int r, int c)
{
	// information field value

	//distanceField[r][c] = -(double)getValueDist(r, c);

	distanceField[r][c] = 1.0/(double)getValueDist(r, c);

	cerr<<"distance value "<<distanceField[r][c]<<endl;
	return distanceField[r][c];
}

// get distance from obstacles
float getValueDist(int r, int c)
{
	float d = dist.ptr<float>(r)[c];
	if (d<=0) d = 0.0001;
	return d;
}

// get field value
double fieldIntensity(Eigen::Vector2d xy)
{
	cv::Point p((int)round(xy.x()), (int)round(xy.y()));
	if (p.x<0 || p.x>map_cols || p.y<0 || p.y>map_rows)
		return 0;
	//return distanceField[p.y][p.x];
	return getValueDistanceField(p.y, p.x);
}

// get field gradient
Eigen::Vector2d fieldGradient(Eigen::Vector2d xy)
{
	cv::Point p((int)round(xy.x()), (int)round(xy.y()));
	if (p.x<0 || p.x>map_cols || p.y<0 || p.y>map_rows)
		return Eigen::Vector2d(0, 0);
	Eigen::Vector2d grd;
	grd.x() = getValueDistanceField(p.y, p.x + 1) - getValueDistanceField(p.y, p.x);
	grd.y() = getValueDistanceField(p.y + 1, p.x) - getValueDistanceField(p.y, p.x);
	return grd;
}

// rho
double rho(Eigen::Vector2d xy)
{
	return pow((255 + fieldIntensity(xy)), 1);
}

// grho
Eigen::Vector2d grho(Eigen::Vector2d xy)
{
	return fieldGradient(xy);
}

// objective func
double energyFunc(const Eigen::VectorXd& x, Eigen::VectorXd& grad)
{
	// f = sigma rho(mid)^2 * ( x_i - x_i+1 )^2
	// df = 
	const int n = x.size();
	// f
	double f(0);
	for (int i = 0; i < n - 4; i += 2)
	{
		Eigen::Vector2d mid((x[i] + x[i + 2]) / 2, (x[i + 1] + x[i + 3]) / 2);
		f +=
			(
			pow((x[i] - x[i + 2]), 2)
			+
			pow((x[i + 1] - x[i + 3]), 2)
			)
			*
			//pow(rho(mid), 2);
			pow(rho(mid), 3); // 2018-09-23
	}
	// grad
	// fix begin and end points
	{
		grad[0] = 0;
		grad[1] = 0;
		grad[n - 2] = 0;
		grad[n - 1] = 0;
	}
	// gradients of other points
	for (int i = 2; i < n - 2; i += 2)
	{
		Eigen::Vector2d xy_i(x[i], x[i + 1]);
		// next point
		Eigen::Vector2d xy_n(x[i + 2], x[i + 3]);
		Eigen::Vector2d mid_n = (xy_i + xy_n) / 2;
		Eigen::Vector2d g_mid_n = grho(mid_n);
		// last point
		Eigen::Vector2d xy_l(x[i - 2], x[i - 1]);
		Eigen::Vector2d mid_l = (xy_i + xy_l) / 2;
		Eigen::Vector2d g_mid_l = grho(mid_l);
		grad[i] =
			3 * rho(mid_n) * g_mid_n.x() * (pow(x[i] - x[i + 2], 2) + pow(x[i + 1] - x[i + 3], 2)) + pow(rho(mid_n), 2) * 2 * (x[i] - x[i + 2])
			+
			3 * rho(mid_l) * g_mid_l.x() * (pow(x[i] - x[i - 2], 2) + pow(x[i + 1] - x[i - 1], 2)) + pow(rho(mid_l), 2) * 2 * (x[i] - x[i - 2]);

		grad[i + 1] =
			3 * rho(mid_n) * g_mid_n.y() * (pow(x[i] - x[i + 2], 2) + pow(x[i + 1] - x[i + 3], 2)) + pow(rho(mid_n), 2) * 2 * (x[i + 1] - x[i + 3])
			+
			3 * rho(mid_l) * g_mid_l.y() * (pow(x[i] - x[i - 2], 2) + pow(x[i + 1] - x[i - 1], 2)) + pow(rho(mid_l), 2) * 2 * (x[i + 1] - x[i - 1]);
	}
	// fix begin and end points
	{
		grad[0] = 0;
		grad[1] = 0;
		grad[n - 2] = 0;
		grad[n - 1] = 0;
	}
	return f;
}

// check if path cross obstacles.
bool crossObstacles(DataEngine* p_de, int cr, int cc, int lr, int lc)
{
	// set up
	cv::Point beg(cc, cr);
	cv::Point end(lc, lr);
	// dda check
	bool occlusion = false;
	int dx = end.x - beg.x;
	int dy = end.y - beg.y;
	if (dx == 0)
	{
		if (beg.y < end.y)
		{
			for (int y = beg.y; y <= end.y; y++)
			{
				if (!(p_de->m_recon2D.m_cellmap[y][beg.x].isScanned && p_de->m_recon2D.m_cellmap[y][beg.x].isFree))
				{
					occlusion = true;
					break;
				}
			}
		}
		else
		{
			for (int y = beg.y; y >= end.y; y--)
			{
				if (!(p_de->m_recon2D.m_cellmap[y][beg.x].isScanned && p_de->m_recon2D.m_cellmap[y][beg.x].isFree))
				{
					occlusion = true;
					break;
				}
			}
		}
	}
	else if (dy == 0)
	{
		if (beg.x < end.x)
		{
			for (int x = beg.x; x <= end.x; x++)
			{
				if (!(p_de->m_recon2D.m_cellmap[beg.y][x].isScanned && p_de->m_recon2D.m_cellmap[beg.y][x].isFree))
				{
					occlusion = true;
					break;
				}
			}
		}
		else
		{
			for (int x = beg.x; x >= end.x; x--)
			{
				if (!(p_de->m_recon2D.m_cellmap[beg.y][x].isScanned && p_de->m_recon2D.m_cellmap[beg.y][x].isFree))
				{
					occlusion = true;
					break;
				}
			}
		}
	}
	else if (dx == 0 && dy == 0){}
	else 
	{
		int MaxStep = abs(dx) > abs(dy) ? abs(dx) : abs(dy); // steps
		double fXUnitLen = 1.0;  // X delta
		double fYUnitLen = 1.0;  // Y delta
		fYUnitLen = (double)(dy) / (double)(MaxStep);
		fXUnitLen = (double)(dx) / (double)(MaxStep);
		double x = (double)(beg.x);
		double y = (double)(beg.y);
		// dda begin
		for (long i = 1; i < MaxStep; i++) // '<' or '<=', not include end point
		{
			x = x + fXUnitLen;
			y = y + fYUnitLen;
			int crt_r = (int)round(y);
			int crt_c = (int)round(x);
			if (!(p_de->m_recon2D.m_cellmap[crt_r][crt_c].isScanned && p_de->m_recon2D.m_cellmap[crt_r][crt_c].isFree))
			{
				occlusion = true;
				break;
			}
		}
	}
	// done.
	return occlusion;
}

// optimization
bool Optimize_Path(DataEngine* p_de, vector<iro::SE2> & path)
{
	// set up
	initDistanceField(p_de);
	const int n = path.size() * 2;
	LBFGSParam<double> param;
	param.epsilon = 1e-6;
	//param.max_iterations = 1000; // defualt
	param.max_iterations = 10;
	LBFGSSolver<double> solver(param);
	Eigen::VectorXd x = Eigen::VectorXd::Zero(n); // variable
	fixed_node_indexes.clear(); // fixed point
	fixed_variabe_indexes.clear(); // fixed point
	for (int nid = 0; nid < path.size(); nid++)
	{
		x[nid * 2] = path[nid].translation().x();
		x[nid * 2 + 1] = -path[nid].translation().y();
		if (fabs(path[nid].rotation().angle() - 0.0) > 0.01) // key view
		{
			fixed_node_indexes.push_back(nid);
			fixed_variabe_indexes.push_back(nid * 2);
			fixed_variabe_indexes.push_back(nid * 2 + 1);
		}
	}
	double fx;
/*
	// vis show origin
	{
		cv::Mat vis = p_de->visCellMap();
		for (int i = 0; i <= n - 2; i += 2)
		{
			cv::circle(vis, cv::Point(x[i], x[i + 1]), 3, CV_RGB(0, 0, 0));
			if (i != 0)
				cv::line(vis, cv::Point(x[i], x[i + 1]), cv::Point(x[i - 2], x[i - 1]), CV_RGB(0, 0, 0));
		}
		cv::imshow("origin", vis);
	}
//*/

	int niter = solver.minimize(energyFunc, x, fx); // lbfgs
/*
	// vis show result
	{
		cv::Mat vis = p_de->visCellMap();
		for (int i = 0; i <= n - 2; i += 2)
		{
			cv::circle(vis, cv::Point(x[i], x[i + 1]), 3, CV_RGB(0, 0, 0));
			if (i != 0)
				cv::line(vis, cv::Point(x[i], x[i + 1]), cv::Point(x[i - 2], x[i - 1]), CV_RGB(0, 0, 0));
		}
		cv::imshow("result", vis);
	}
	cv::waitKey(0);
//*/

	// save optimized path
	int lr = 0;
	int lc = 0;
	for (int nid = 0; nid < path.size(); nid++)
	{
		int cr = (int)round(x[nid * 2 + 1]);
		int cc = (int)round(x[nid * 2]);
		// avoid out of boundary. 2018-09-24.
		if (cr < 0 || cr >= map_rows || cc < 0 || cc >= map_cols) return false;
		//if (getValueDist((int)round(x[nid * 2 + 1]), (int)round(x[nid * 2]))) return false;
		// avoid crossing obstacles when sample points are few. 2019-06-12.
		if (nid > 0) if (crossObstacles(p_de, cr, cc, lr, lc)) return false;
		lr = cr;
		lc = cc;
	}
	for (int nid = 0; nid < path.size(); nid++)
	{
		path[nid].translation().x() = x[nid * 2];
		path[nid].translation().y() = -x[nid * 2 + 1];
	}
	return true;
}