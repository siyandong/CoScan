#pragma once
// std
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <vector>
#include <list>
#include <set>
#include <unistd.h> // sleep
// eigen
#include <Eigen/Dense>
// other headers
#include "global.h"
#include "data_engine.h" // todo: remove.
#include "geodesic/Xin_Wang.h" 

// cluster
struct Cluster
{
	int label = -1;
	Eigen::Vector2d centroid;
	vector<int> samples;
	Cluster()
	{
		label = -1;
		centroid = Eigen::Vector2d(0, 0);
		if (!samples.empty())
			samples.clear();
	}
	Cluster(int l, Eigen::Vector2d p, vector<int> s)
	{
		label = l;
		centroid = Eigen::Vector2d(p.x(), p.y());
		samples.clear();
		for (int i = 0; i < s.size(); i++)
			samples.push_back(s[i]);
	}
	Cluster(const Cluster & other)
	{
		label = other.label;
		centroid = Eigen::Vector2d(other.centroid.x(), other.centroid.y());
		samples.clear();
		for (int i = 0; i < other.samples.size(); i++)
			samples.push_back(other.samples[i]);
	}
	Cluster & operator = (const Cluster & other)
	{
		label = other.label;
		centroid = Eigen::Vector2d(other.centroid.x(), other.centroid.y());
		samples.clear();
		for (int i = 0; i < other.samples.size(); i++)
			samples.push_back(other.samples[i]);
		return *this;
	}
};

// omt solver
class DiscreteSolver
{
	// input: domain, cost metric, sources, targets.
	// output: assignment from sources to targets.

	DataEngine* m_p_de;														// domain.
	CDT m_domain;															// domain.
	DistanceMetric* m_metric;												// metric.
	//vector<double>(*CostFunc)(Eigen::Vector2d, vector<Eigen::Vector2d>);	// metric. todo: remove.
	std::vector<Eigen::Vector2d> m_sources;									// robot coordinates.
	std::vector<Eigen::Vector2d> m_targets;									// task coordinates. 
	double m_delta;															// param delta to constrain the compactness and capacity.
	std::vector<std::vector<int>> m_result_s_t;								// results.
public:
	// constructor.
	DiscreteSolver(DataEngine& de, CDT d, 

		DistanceMetric& dm,

		//vector<double>(*cf)(Eigen::Vector2d, vector<Eigen::Vector2d>), 

		vector<Eigen::Vector2d> s, vector<Eigen::Vector2d> t, double delta)
	{
		// domain
		m_p_de = &de;
		// domain
		this->m_domain = CDT(d);
		// metric
		m_metric = &dm;
		// metric
		//CostFunc = cf;
		// sources
		this->m_sources.clear();
		for (int i = 0; i < s.size(); i++)
			this->m_sources.push_back(s[i]);
		// targets
		this->m_targets.clear();
		for (int i = 0; i < t.size(); i++)
			this->m_targets.push_back(t[i]);
		// delta
		this->m_delta = delta;
		// results
		this->m_result_s_t.clear();
		this->m_result_s_t.resize(this->m_sources.size());
		// end.
		return;
	}
	// destructor
	~DiscreteSolver()
	{
		// todo: CDT need manually release?
		vector<Eigen::Vector2d>().swap(m_sources);
		vector<Eigen::Vector2d>().swap(m_targets);
		for (int i = 0; i < m_result_s_t.size(); i++)
			vector<int>().swap(m_result_s_t[i]);
		vector<vector<int>>().swap(m_result_s_t);
		return;
	}

	// execute the solveer
	bool SolveOMT(int mode = 0);
	// results
	std::vector<std::vector<int>> getSolution(){ return m_result_s_t; }
	// visualize domain
	void CerrDomainCDT();
	// visualize xxx
	void CerrClusters(std::vector<Cluster> clusters);

//private:

	// cluster
	std::vector<Cluster> ClusterTargets(int mode = 0);
	// match
	std::vector<int> Match(std::vector<Cluster> clusters);
};

using namespace std;

// hungarian algorithm O(n^3)
vector<int> hungarian_algorithm(Eigen::MatrixXd cost)
{
	/* cost matrix format
	
	target_1    target_2    ...    target_m
	source_1
	source_2
	.
	.
	.
	source_m

	*/

	// detail info
	bool log = false;

	// results
	vector<int> assignment(cost.rows());

	// Step 1: Subtract row minima
Step1:
	{
		for (int r = 0; r < cost.rows(); r++)
		{
			double minima = DBL_MAX;
			for (int c = 0; c < cost.cols(); c++)
				if (cost(r, c) < minima)
					minima = cost(r, c);
			for (int c = 0; c < cost.cols(); c++)
				cost(r, c) -= minima;
		}
		if (log)
			cerr << "Subtract row minima" << endl << cost << endl << endl;
	}

	// Step 2: Subtract column minima
Step2:
	{
		for (int c = 0; c < cost.cols(); c++)
		{
			double minima = DBL_MAX;
			for (int r = 0; r < cost.rows(); r++)
				if (cost(r, c) < minima)
					minima = cost(r, c);
			for (int r = 0; r < cost.rows(); r++)
				cost(r, c) -= minima;
		}
		if (log)
			cerr << "Subtract column minima" << endl << cost << endl << endl;
	}

	// Step 3: Cover all zeros with a minimum number of lines
	vector<bool> tick_rows(cost.rows());
	vector<bool> tick_cols(cost.cols());
	vector<vector<bool>> circle;
	vector<vector<bool>> xxxxxx;
Step3:
	circle.clear();
	xxxxxx.clear();
	{
		circle.resize(cost.rows());
		xxxxxx.resize(cost.rows());
		for (int r = 0; r < cost.rows(); r++)
		{
			circle[r].resize(cost.cols());
			xxxxxx[r].resize(cost.cols());
			for (int c = 0; c < cost.cols(); c++)
			{
				circle[r][c] = false;
				xxxxxx[r][c] = false;
			}
		}
	}
Check3:
	tick_rows.clear();
	tick_rows.resize(cost.rows());
	tick_cols.clear();
	tick_cols.resize(cost.cols());
	{
		// count num of '0'
		vector<int> num_rows_0(cost.rows());
		vector<int> num_cols_0(cost.cols());
		{
			for (int r = 0; r < num_rows_0.size(); r++)
				num_rows_0[r] = 0;
			for (int c = 0; c < num_cols_0.size(); c++)
				num_cols_0[c] = 0;
			for (int r = 0; r < cost.rows(); r++)
			{
				for (int c = 0; c < cost.cols(); c++)
				{
					if (cost(r, c) < 0.0001)
					{
						num_rows_0[r]++;
						num_cols_0[c]++;
					}
				}
			}
		}

		// label '0' by circle or xxxxxx
		{
			// labeling
			int circle_num = 0;
			int xxxxxx_num = 0;
			int circle_crt = 0;
			int xxxxxx_crt = 0;
		labeling:
			circle_crt = 0;
			xxxxxx_crt = 0;
			for (int r = 0; r < cost.rows(); r++)
			{
				int count = 0;
				int index = -1;
				for (int c = 0; c < cost.cols(); c++)
					//if (cost(r, c) < 0.0001)
					if (cost(r, c) < 0.0001 && !xxxxxx[r][c])
					{
						count++;
						index = c;
					}
				if (count == 1)
				{
					circle[r][index] = true;
					for (int tr = 0; tr < cost.rows(); tr++)
						if (cost(tr, index) < 0.0001 && !circle[tr][index])
							xxxxxx[tr][index] = true;
				}
			}
			for (int c = 0; c < cost.cols(); c++)
			{
				int count = 0;
				int index = -1;
				for (int r = 0; r < cost.rows(); r++)
					if (cost(r, c) < 0.0001 && !xxxxxx[r][c])
					{
						count++;
						index = r;
					}
				if (count == 1)
				{
					circle[index][c] = true;
					for (int tc = 0; tc < cost.cols(); tc++)
						if (cost(index, tc) < 0.0001 && !circle[index][tc])
							xxxxxx[index][tc] = true;
				}
			}
			// update crt
			for (int r = 0; r < cost.rows(); r++)
			{
				for (int c = 0; c < cost.cols(); c++)
				{
					if (circle[r][c])
						circle_crt++;
					if (xxxxxx[r][c])
						xxxxxx_crt++;
				}
			}
			// crt vs. num
			if (circle_crt == circle_num && xxxxxx_crt == xxxxxx_num)
			{

			}
			else
			{
				circle_num = circle_crt;
				xxxxxx_num = xxxxxx_crt;
				goto labeling;
			}
		}
		// check if left '0' unlabeled
	Check:
		{
			for (int r = 0; r < cost.rows(); r++)
			{
				for (int c = 0; c < cost.cols(); c++)
				{
					if (!circle[r][c] && !xxxxxx[r][c] && cost(r, c) < 0.0001)
					{
						circle[r][c] = true;
						for (int tr = 0; tr < cost.rows(); tr++)
						{
							if (cost(tr, c) < 0.0001
								&& !circle[tr][c] && !xxxxxx[tr][c])
							{
								xxxxxx[tr][c] = true;
							}
						}
						for (int tc = 0; tc < cost.cols(); tc++)
						{
							if (cost(r, tc) < 0.0001
								&& !circle[r][tc] && !xxxxxx[r][tc])
							{
								xxxxxx[r][tc] = true;
							}
						}
						goto Check3;
					}
				}
			}
		}

		// line cover
		{
			// tick
			{
				// init
				for (int r = 0; r < tick_rows.size(); r++)
					tick_rows[r] = false;
				for (int c = 0; c < tick_cols.size(); c++)
					tick_cols[c] = false;
				// tick step 1
				for (int r = 0; r < cost.rows(); r++)
				{
					bool exist = false;
					for (int c = 0; c < cost.cols(); c++)
						if (circle[r][c])
							exist = true;
					if (!exist)
					{
						tick_rows[r] = true;
					}
				}
				// loop
				int tick_num = 0;
				for (int r = 0; r < tick_rows.size(); r++)
					if (tick_rows[r])
						tick_num++;
				for (int c = 0; c < tick_cols.size(); c++)
					if (tick_cols[c])
						tick_num++;
				int crt_num = 0;
				while (crt_num != tick_num)
				{
					tick_num = crt_num;
					// tick step 2
					for (int r = 0; r < cost.rows(); r++)
					{
						if (tick_rows[r])
						{
							for (int c = 0; c < cost.cols(); c++)
							{
								if (cost(r, c) < 0.0001 && xxxxxx[r][c])
								{
									tick_cols[c] = true;
								}
							}
						}
					}
					// tick step 3
					for (int c = 0; c < cost.cols(); c++)
					{
						if (tick_cols[c])
						{
							for (int r = 0; r < cost.rows(); r++)
							{
								if (cost(r, c) < 0.0001 && circle[r][c])
								{
									tick_rows[r] = true;
								}
							}
						}
					}
					// update crt_num
					crt_num = 0;
					for (int r = 0; r < tick_rows.size(); r++)
						if (tick_rows[r])
							crt_num++;
					for (int c = 0; c < tick_cols.size(); c++)
						if (tick_cols[c])
							crt_num++;
				}
			}

			// check
			if (log)
			{
				Eigen::MatrixXd lmat(cost.rows(), cost.cols());
				for (int r = 0; r < lmat.rows(); r++)
					for (int c = 0; c < lmat.cols(); c++)
						lmat(r, c) = 0;
				for (int r = 0; r < cost.rows(); r++)
					if (!tick_rows[r])
						for (int c = 0; c < cost.cols(); c++)
							lmat(r, c) = 1;
				for (int c = 0; c < cost.cols(); c++)
					if (tick_cols[c])
						for (int r = 0; r < cost.rows(); r++)
							lmat(r, c) = 1;
				cerr << "Cover all zeros with a minimum number of lines" << endl
					<< lmat << endl << endl;
			}

			// check number of line cover
			{
				int line_num = 0;
				for (int r = 0; r < cost.rows(); r++)
					if (!tick_rows[r])
					{
						line_num++;
					}
				for (int c = 0; c < cost.cols(); c++)
					if (tick_cols[c])
					{
						line_num++;
					}

				if (line_num == cost.rows())
					goto Final;
				else
					goto Step4;
			}
		}
	}

	// Step 4: Create additional zeros
Step4:
	{
		// line covered
		vector<vector<bool>> covered;
		// init
		covered.resize(cost.rows());
		for (int r = 0; r < cost.rows(); r++)
		{
			covered[r].resize(cost.cols());
			for (int c = 0; c < cost.cols(); c++)
			{
				covered[r][c] = false;
			}
		}
		// check number of line cover
		{
			for (int r = 0; r < cost.rows(); r++)
				if (!tick_rows[r])
				{
					// line cover
					for (int c = 0; c < cost.cols(); c++)
					{
						covered[r][c] = true;
					}
				}
			for (int c = 0; c < cost.cols(); c++)
				if (tick_cols[c])
				{
					// line cover
					for (int r = 0; r < cost.rows(); r++)
					{
						covered[r][c] = true;
					}
				}

		}
		// minima of uncovered
		{
			double minima = DBL_MAX;
			for (int r = 0; r < cost.rows(); r++)
			{
				for (int c = 0; c < cost.cols(); c++)
				{
					if (!covered[r][c])
					{
						if (cost(r, c) < minima)
						{
							minima = cost(r, c);
						}
					}
				}
			}

			// plus and minus
			{
				for (int r = 0; r < cost.rows(); r++)
				{
					for (int c = 0; c < cost.cols(); c++)
					{
						if (!covered[r][c])
						{
							cost(r, c) -= minima;
						}
						else
						{
							if (!tick_rows[r] && tick_cols[c])
								cost(r, c) += minima;
						}
					}
				}
			}
		}

		if (log)
			cerr << "Create additional zeros" << endl << cost << endl << endl;

		goto Step3;
	}

	// final step
Final:
	{
		// save assignment
		for (int r = 0; r < cost.rows(); r++)
		{
			for (int c = 0; c < cost.cols(); c++)
			{
				if (circle[r][c])
				{
					assignment[r] = c;
				}
			}
		}
		// log
		if (log)
		{
			Eigen::MatrixXd opt(cost.rows(), cost.cols());
			for (int r = 0; r < opt.rows(); r++)
				for (int c = 0; c < opt.cols(); c++)
					opt(r, c) = 0;
			for (int r = 0; r < cost.rows(); r++)
				for (int c = 0; c < cost.cols(); c++)
					if (circle[r][c])
						opt(r, c) = 1;
			cerr << "result" << endl << opt << endl << endl;
		}
	}

	return assignment;
}

// visualize cdt
void DiscreteSolver::CerrDomainCDT()
{
	// cdt
	for (auto it = m_domain.finite_faces_begin(); it != m_domain.finite_faces_end(); it++)
	{
		if (it->info().in_domain())
		{
			//// 
			//cerr << "plt.plot([" << it->vertex(0)->point().x() << ", " << it->vertex(1)->point().x() << "], ["
			//	<< it->vertex(0)->point().y() << ", " << it->vertex(1)->point().y() << "], 'k-')" << endl;
			//cerr << "plt.plot([" << it->vertex(1)->point().x() << ", " << it->vertex(2)->point().x() << "], ["
			//	<< it->vertex(1)->point().y() << ", " << it->vertex(2)->point().y() << "], 'k-')" << endl;
			//cerr << "plt.plot([" << it->vertex(2)->point().x() << ", " << it->vertex(0)->point().x() << "], ["
			//	<< it->vertex(2)->point().y() << ", " << it->vertex(0)->point().y() << "], 'k-')" << endl;

			//
			cerr << "plt.fill([" << it->vertex(0)->point().x() << ", " << it->vertex(1)->point().x() << ", " << it->vertex(2)->point().x()
				<< "], [" << it->vertex(0)->point().y() << ", " << it->vertex(1)->point().y() << ", " << it->vertex(2)->point().y()
				<< "], color=[0.7, 0.7, 0.7])" << endl;
		}
	}
	// end
	return;
}

// visualize clusters
void DiscreteSolver::CerrClusters(vector<Cluster> clusters)
{
	for (int cid = 0; cid < clusters.size(); cid++)
	{
		double r = (double)rand() / RAND_MAX;
		double g = (double)rand() / RAND_MAX;
		double b = (double)rand() / RAND_MAX;
		// samples
		for (int sid = 0; sid < clusters[cid].samples.size(); sid++)
		{
			int tid = clusters[cid].samples[sid];
			char sentence[200];
			sprintf(sentence, "plt.plot(%f, %f, color=[%f, %f, %f])",
				m_targets[tid].x(), m_targets[tid].y(), r, g, b);
			cerr << sentence << endl;
		}
		// centroid
		char sentence[200];
		sprintf(sentence, "plt.plot(%f, %f, marker='x', color=[%f, %f, %f])",
			clusters[cid].centroid.x(), clusters[cid].centroid.y(), r, g, b);
		cerr << sentence << endl;
	}
	return;
}

// cluster targets
vector<Cluster> DiscreteSolver::ClusterTargets(int mode)
{
	// timing
	double t_beg = clock();

	// set up params
	int max_iter_times = 10;

	// initialization
	vector<Cluster> clusters;
	clusters.resize(m_sources.size());
	for (int idx = 0; idx < clusters.size(); idx++)
		clusters[idx].centroid = m_sources[idx];

	// iteration begin
	for (int it = 0; it < max_iter_times; it++)
	{
		cerr << "cluster iteration = " << it << endl;

		// record last centroid
		vector<Eigen::Vector2d> last_centroids(clusters.size());
		for (int cid = 0; cid < clusters.size(); cid++)
			last_centroids[cid] = Eigen::Vector2d(clusters[cid].centroid.x(), clusters[cid].centroid.y());

		// clear samples
		for (int cid = 0; cid < clusters.size(); cid++)
			clusters[cid].samples.clear();

		// compute label for each sample
		{
			// memory alloc
			int dRange = max_iter_times * m_targets.size();
			double ** m_tid_cid_d = new double*[dRange];
			while (m_tid_cid_d == 0)
			{
				cerr << "memory error, re assign memory..." << endl;
				sleep(0.1);
				m_tid_cid_d = new double*[dRange];
			}
			for (int tid = 0; tid < dRange; tid++)
			{
				m_tid_cid_d[tid] = new double[dRange];
				while (m_tid_cid_d[tid] == 0)
				{
					cerr << "memory error, re assign memory..." << endl;
					sleep(0.1);
					m_tid_cid_d[tid] = new double[dRange];
				}
			}
			// compute distance
#pragma omp parallel for num_threads(omp_get_num_procs())
			for (int cid = 0; cid < clusters.size(); cid++)
			{
				//vector<double> cDistances = CostFunc(clusters[cid].centroid, m_targets); // compute distance
				vector<double> cDistances = m_metric->GeodesicDistances(clusters[cid].centroid, m_targets); // compute distance
				for (int tid = 0; tid < m_targets.size(); tid++)
				{
					if (cid >= dRange)
					{
						char str[200];
						sprintf(str, "cid = %d, dRange = %d", cid, dRange);
						cout << str << endl;
						cout << "error: cid out bound" << endl;
					}
					if (tid >= dRange)
					{
						char str[200];
						sprintf(str, "tid = %d, dRange = %d", tid, dRange);
						cout << str << endl;
						cout << "error: tid out bound" << endl;
					}
					m_tid_cid_d[tid][cid] = cDistances[tid];
				}
			}
			// compute label
			for (int tid = 0; tid < m_targets.size(); tid++)
			{
				int c = 0;
				double min_distance = DBL_MAX;
				for (int cid = 0; cid < clusters.size(); cid++)
				{
					double distance = m_tid_cid_d[tid][cid];
					if (distance < min_distance)
					{
						min_distance = distance;
						c = cid;
					}
				}
				clusters[c].samples.push_back(tid);
			}
			// memory release
			for (int tid = 0; tid < dRange; tid++)
				delete[] m_tid_cid_d[tid];
			delete[] m_tid_cid_d;
		}

		// compute centroids 
#pragma omp parallel for num_threads(omp_get_num_procs())
		for (int cid = 0; cid < clusters.size(); cid++)
		{
			// check empty
			if (clusters[cid].samples.empty())
			{
				clusters[cid].centroid = Eigen::Vector2d(999999, 999999);
				continue;
			}
			// compute centroid.
			Eigen::Vector2d val(0, 0);
			for (int sid = 0; sid < clusters[cid].samples.size(); sid++)
			{
				int sample = clusters[cid].samples[sid];
				val = Eigen::Vector2d(val.x() + m_targets[sample].x(), val.y() + m_targets[sample].y());
			}
			clusters[cid].centroid = Eigen::Vector2d(val.x() / clusters[cid].samples.size(), val.y() / clusters[cid].samples.size());
			// inner centroid. cellmap?
			{
				int r = (int)round(-clusters[cid].centroid.y());
				int c = (int)round(clusters[cid].centroid.x());
				// check out bound
				if (m_p_de->m_recon2D.m_cellmap[r][c].isScanned && m_p_de->m_recon2D.m_cellmap[r][c].isFree)
				{
					; // nothing to do here.
				}
				else
				{
					// replease with the closest point in domain
					double min_d = DBL_MAX;
					Eigen::Vector2d min_p;
					for (auto it = m_domain.vertices_begin(); it != m_domain.vertices_end(); it++)
					{
						Eigen::Vector2d p(it->point().x(), it->point().y());
						double d = (clusters[cid].centroid - p).norm();
						if (d < min_d)
						{
							min_d = d;
							min_p = Eigen::Vector2d(p);
						}
					}
					clusters[cid].centroid = min_p;
				}
			}
		}

		// split (compactness & capacity)
		for (int cid = 0; cid < clusters.size(); cid++) // for each cluster, check innner distance
		{
			if (clusters[cid].samples.size() <= 1) continue;
			// compute iner distances
			vector<Eigen::Vector2d> targets;
			for (int tid = 0; tid < clusters[cid].samples.size(); tid++)
				targets.push_back(Eigen::Vector2d(m_targets[clusters[cid].samples[tid]].x(), m_targets[clusters[cid].samples[tid]].y()));
			//vector<double> inDistances = CostFunc(clusters[cid].centroid, targets);
			vector<double> inDistances = m_metric->GeodesicDistances(clusters[cid].centroid, targets);
			// max distance element
			int maxID = -1;
			double maxDistance = -1;
			for (int i = 0; i < inDistances.size(); i++)
			{
				if (maxDistance < inDistances[i])
				{
					maxDistance = inDistances[i];
					maxID = i;
				}
			}
			if (maxID == -1)
			{
				cerr << "error: maxID == -1." << endl;
				getchar(); getchar(); getchar();
			}
			if (maxID >= clusters[cid].samples.size() || maxID >= m_targets.size())
			{
				cerr << "error: maxID out bound." << endl;
				cerr << "maxID = " << maxID << endl;
				cerr << "cluster samples num = " << clusters[cid].samples.size() << endl;
				cerr << "targets num = " << m_targets.size() << endl;
				getchar(); getchar(); getchar();
			}
			// if need split
			if (maxDistance > compactParam)
			{
				// insert new cluster
				Cluster new_cluster;
				new_cluster.label = -1; // todo: id.
				new_cluster.samples.push_back(clusters[cid].samples[maxID]);
				new_cluster.centroid = Eigen::Vector2d(m_targets[clusters[cid].samples[maxID]].x(), m_targets[clusters[cid].samples[maxID]].y());
				clusters.push_back(new_cluster);
				// erase the sample from origin cluster
				clusters[cid].samples.erase(clusters[cid].samples.begin() + maxID);
			}
		}

		// merge
		int valid_cluster_num = 0;
		for (int cid = 0; cid < clusters.size(); cid++)
		{
			if (clusters[cid].samples.empty())
				continue;
			valid_cluster_num++;
		}
		if (valid_cluster_num <= m_sources.size()) // useless?
		{
		}
		//else if (!allowFreeRobot) // useless?
		//{
		//}
		else
		{
			bool nMerge = false;
			for (int cid = 0; cid < clusters.size() - 1; cid++)
			{
				if (clusters[cid].samples.empty()) continue;
				for (int oid = cid + 1; oid < clusters.size(); oid++)
				{
					if (clusters[oid].samples.empty()) continue;
					// compute distance
					Point_2 p1(clusters[cid].centroid.x(), clusters[cid].centroid.y());
					Point_2 p2(clusters[oid].centroid.x(), clusters[oid].centroid.y());
					double outerD = m_metric->get_geodesic_distance_fast(p1, p2); // todo: change it to func pointer.
					if (outerD < compactParam) // need to merge
					{
						if (clusters[oid].samples.size() == 1) // todo: .
						{
							clusters[cid].samples.push_back(clusters[oid].samples[0]);
							clusters[oid].samples.clear();
							nMerge = true;
						}
						else if (clusters[cid].samples.size() == 1)
						{
							clusters[oid].samples.push_back(clusters[cid].samples[0]);
							clusters[cid].samples.clear();
							nMerge = true;
						}
						// merge done.
						valid_cluster_num--;
						//// valid--
						//if (validNumber <= rbtPositions.size())
						//	break;
					}
				}
			}
		}

		// delete empty clusters
		{
			vector<Cluster> temp_clusters;
			for (int cid = 0; cid < clusters.size(); cid++)
			{
				if (!clusters[cid].samples.empty())
				{
					temp_clusters.push_back(Cluster(clusters[cid]));
				}
			}
			vector<Cluster>().swap(clusters);
			clusters = temp_clusters;
		}

		// update centroids 
		{
			for (int cid = 0; cid < clusters.size(); cid++)
			{
				if (clusters[cid].samples.empty())
				{
					clusters[cid].centroid = Eigen::Vector2d(999999, 999999);
					continue;
				}
				// update centroid
				Eigen::Vector2d val(0, 0);
				for (int sid = 0; sid < clusters[cid].samples.size(); sid++)
				{
					int sample = clusters[cid].samples[sid];
					val = Eigen::Vector2d(val.x() + m_targets[sample].x(), val.y() + m_targets[sample].y());
				}
				clusters[cid].centroid = Eigen::Vector2d(val.x() / clusters[cid].samples.size(), val.y() / clusters[cid].samples.size());
				// inner centroid. cellmap?
				{
					int r = (int)round(-clusters[cid].centroid.y());
					int c = (int)round(clusters[cid].centroid.x());
					// check out bound
					if (m_p_de->m_recon2D.m_cellmap[r][c].isScanned && m_p_de->m_recon2D.m_cellmap[r][c].isFree)
					{
						; // nothing to do here.
					}
					else
					{
						// replease with the closest point in domain
						double min_d = DBL_MAX;
						Eigen::Vector2d min_p;
						for (auto it = m_domain.vertices_begin(); it != m_domain.vertices_end(); it++)
						{
							Eigen::Vector2d p(it->point().x(), it->point().y());
							double d = (clusters[cid].centroid - p).norm();
							if (d < min_d)
							{
								min_d = d;
								min_p = Eigen::Vector2d(p);
							}
						}
						clusters[cid].centroid = min_p;
					}
				}
			}
		}

		// iteration difference
		{
			bool terminate = false;
			if (last_centroids.size() == clusters.size())
			{
				double diff = 0;
				for (int cid = 0; cid < clusters.size(); cid++)
				{
					diff += (last_centroids[cid] - clusters[cid].centroid).norm(); // todo: func pointer.
					if (diff > 0.001)
					{
						break;
					}
				}
				if (diff < 0.001) terminate = true;
			}
			// terminate
			if (terminate)
			{
				cerr << "converged at iter " << it << "." << endl;
				break;
			}
		}

		//		if (!allowFreeRobot) // avoid #cluster_not_empty < #robot
		//		{
		//			// compute valid cluster number
		//			int cnt = 0;
		//			for (int rid = 0; rid < regions.size(); rid++)
		//			{
		//				if (regions[rid].samples.empty())
		//					continue;
		//				cnt++;
		//			}
		//			while (cnt < rbtPositions.size() && tasks.size() >= rbtPositions.size()) // if #cluster_not_empty < #robot, split a cluster.
		//			{
		//				//cerr << "robot free of tasks, finding split..." << endl;
		//				//cout << "robot free of tasks, finding split..." << endl;
		//				//cout << "#rbt = " << rbtPositions.size() << ", #cluster = " << cnt << endl;
		//				// find the selected region: farthest iner distance
		//				double farInerDistance = -1;
		//				int selectedRid = -1;
		//				int selectedSid = -1;
		//				for (int orid = 0; orid < regions.size(); orid++)
		//				{
		//					if (regions[orid].samples.size() <= 1)
		//						continue;
		//					for (int sid = 0; sid < regions[orid].samples.size(); sid++)
		//					{
		//						int tid = regions[orid].samples[sid];
		//						// compare option 2: distance from centroid. 2018-10-22.
		//						//double distance = m_metric->get_geodesic_distance(regions[orid].centroid, Point_2(tasks[tid].pose.translation().x(), tasks[tid].pose.translation().y()));
		//						double distance = m_metric->get_geodesic_distance_fast(regions[orid].centroid, Point_2(tasks[tid].pose.translation().x(), tasks[tid].pose.translation().y()));
		//						if (distance > farInerDistance)
		//						{
		//							farInerDistance = distance;
		//							selectedRid = orid;
		//							selectedSid = sid;
		//						}
		//					}
		//				}
		//				// no candidate
		//				if (selectedRid == -1 || selectedSid == -1)
		//				{
		//					break;
		//				}
		//
		//				// insert the sample
		//				Region nRegion;
		//				nRegion.label = -1;
		//				nRegion.samples.push_back(regions[selectedRid].samples[selectedSid]);
		//				nRegion.centroid = Point_2(tasks[regions[selectedRid].samples[selectedSid]].pose.translation().x(), tasks[regions[selectedRid].samples[selectedSid]].pose.translation().y());
		//				regions.push_back(nRegion);
		//				cnt++; // update count.
		//
		//				// remove the sample
		//				regions[selectedRid].samples.erase(regions[selectedRid].samples.begin() + selectedSid);
		//
		//				//cerr << "split cluster because of free robot." << endl;
		//				//cout << "split cluster because of free robot." << endl;
		//				//cout << "current state: " << endl;
		//
		//				int tmpCnt = 0;
		//				for (int tmpID = 0; tmpID < regions.size(); tmpID++)
		//				{
		//					if (regions[tmpID].samples.empty())
		//						continue;
		//					//cout << "region_" << tmpID << " sample_0 " << tasks[regions[tmpID].samples[0]].pose.translation().transpose() << endl;
		//					//cout << "region_" << tmpID << " centroid " << regions[tmpID].centroid.x() << " " << regions[tmpID].centroid.y() << endl;
		//					tmpCnt++;
		//				}
		//				//cout << tmpCnt << " valid clusters." << endl << endl;
		//			}
		//		}
		//
		//		// difference of centroids
		//		double crtResCentroids = 0;
		//
		//		/* update centroid */ // update 2018-11-14.
		//		for (int rid = 0; rid < regions.size(); rid++)
		//		{
		//			if (regions[rid].samples.empty())
		//			{
		//				regions[rid].centroid = Point_2(999999, 999999);
		//				continue;
		//			}
		//			// update centroid
		//			Point_2 val(0, 0);
		//			for (int sid = 0; sid < regions[rid].samples.size(); sid++)
		//			{
		//				int sample = regions[rid].samples[sid];
		//				val = Point_2(val.x() + tasks[sample].pose.translation().x(), val.y() + tasks[sample].pose.translation().y());
		//			}
		//			regions[rid].centroid = Point_2(val.x() / regions[rid].samples.size(), val.y() / regions[rid].samples.size());
		//			crtResCentroids += get_euclidean_distance(lastCentroids[rid], regions[rid].centroid);
		//		}
		//
		//		///* test plot */
		//		//{
		//		//	char file_dir[1000];
		//		//	sprintf(file_dir, "data/OfflineOutput/clusters_%d.m", it);
		//		//	ofstream ofs(file_dir);
		//		//	vector<vector<double>> color(regions.size());
		//		//	for (int i = 0; i < color.size(); i++)
		//		//	{
		//		//		color[i].resize(3);
		//		//		for (int j = 0; j < color[i].size(); j++)
		//		//		{
		//		//			color[i][j] = (double)rand() / RAND_MAX;
		//		//		}
		//		//	}
		//		//	for (int cid = 0; cid < regions.size(); cid++)
		//		//	{
		//		//		for (int sid = 0; sid < regions[cid].samples.size(); sid++)
		//		//		{
		//		//			char plotCode[200];
		//		//			sprintf(plotCode, "plot([%f, %f], [%f, %f], 'Color', [%f, %f, %f]); hold on;",
		//		//				regions[cid].centroid.x(), tasks[regions[cid].samples[sid]].pose.translation().x(),
		//		//				regions[cid].centroid.y(), tasks[regions[cid].samples[sid]].pose.translation().y(),
		//		//				color[cid][0], color[cid][1], color[cid][2]);
		//		//			ofs << plotCode << endl;
		//		//		}
		//		//	}
		//		//	ofs.close();
		//		//}
		//
		//		/* res */
		//		if (crtResCentroids < 0.001)
		//		{
		//			cout << "converged at iter " << it << "." << endl;
		//			break;
		//		}

	}

	// delete empty clusters
	{
		vector<Cluster> temp_clusters;
		for (int cid = 0; cid < clusters.size(); cid++)
		{
			if (!clusters[cid].samples.empty())
			{
				temp_clusters.push_back(Cluster(clusters[cid]));
			}
		}
		vector<Cluster>().swap(clusters);
		clusters = temp_clusters;
	}

	// timing
	double t_end = clock();
	cerr << "DiscreteSolver cluster timing " << (t_end - t_beg)/CLOCKS_PER_SEC << " s" << endl;

	//// test show results. ckecked correct.
	//{
	//	// seed points
	//	for (int sid = 0; sid < m_sources.size(); sid++)
	//	{
	//		//char color[100];
	//		//sprintf(color, "[%f, %f, %f]", (double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
	//		char sentence[100];
	//		//sprintf(sentence, "plt.plot(%f, %f, color=%s, marker='x')", m_sources[sid].x(), m_sources[sid].y(), color);
	//		sprintf(sentence, "plt.plot(%f, %f, color=[1, 0, 0], marker='x')", m_sources[sid].x(), m_sources[sid].y());
	//		cerr << sentence << endl;
	//	}
	//	// clusters
	//	for (int cid = 0; cid < clusters.size(); cid++)
	//	{
	//		char color[100];
	//		sprintf(color, "[%f, %f, %f]", (double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
	//		for (int sid = 0; sid < clusters[cid].samples.size(); sid++)
	//		{
	//			int id = clusters[cid].samples[sid];
	//			char sentence[100];
	//			sprintf(sentence, "plt.plot(%f, %f, color=%s, marker='.')", m_targets[id].x(), m_targets[id].y(), color);
	//			cerr << sentence << endl;
	//		}
	//		char sentence[100];
	//		sprintf(sentence, "plt.plot(%f, %f, color=%s, marker='+')", clusters[cid].centroid.x(), clusters[cid].centroid.y(), color);
	//		cerr << sentence << endl;
	//	}
	//}

	// end.
	return clusters;
}

// match
vector<int> DiscreteSolver::Match(vector<Cluster> clusters)
{
	//// test hungarian
	//{
	//	//// example#1
	//	//Eigen::MatrixXd cost(4, 4);
	//	//cost << 82, 83, 69, 92,
	//	//		77, 37, 49, 92,
	//	//		11, 69,  5, 86,
	//	//		 8,  9, 98, 23;
	//	//// example#2
	//	//Eigen::MatrixXd cost(5, 5);
	//	//cost << 5, 0, 2, 0, 2,
	//	//	2, 3, 0, 0, 0,
	//	//	0, 10, 5, 7, 2,
	//	//	9, 8, 0, 0, 4,
	//	//	0, 6, 3, 6, 5;
	//	// example#3
	//	Eigen::MatrixXd cost(10, 10);
	//	cost << 7, 54, 42, 4, 84, 24, 53, 80, 61, 14,
	//		43, 30, 24, 65, 95, 9, 1, 87, 24, 28,
	//		63, 23, 61, 68, 33, 37, 53, 45, 80, 44,
	//		78, 99, 4, 9, 81, 65, 37, 92, 98, 85,
	//		61, 15, 82, 85, 89, 11, 2, 58, 5, 24,
	//		41, 39, 29, 45, 55, 19, 36, 95, 31, 51,
	//		31, 98, 73, 19, 18, 66, 65, 23, 40, 8,
	//		56, 37, 10, 49, 56, 64, 29, 21, 45, 90,
	//		15, 81, 24, 33, 76, 3, 44, 54, 97, 70,
	//		83, 4, 85, 25, 15, 9, 4, 83, 58, 94;
	//	// test the case
	//	cerr << "cost" << endl << cost << endl;
	//	hungarian_algorithm(cost);
	//}

	// init
	vector<Eigen::Vector2d> centroids(clusters.size());
	for (int cid = 0; cid < clusters.size(); cid++)
		centroids[cid] = clusters[cid].centroid;
	vector<int> matches(m_sources.size());
	for (int sid = 0; sid < m_sources.size(); sid++)
		matches[sid] = sid;
	
	// compute distances
	vector<vector<double>> d_s_c(m_sources.size());
	for (int s = 0; s < d_s_c.size(); s++)
		//d_s_c[s] = CostFunc(m_sources[s], centroids);
		d_s_c[s] = m_metric->GeodesicDistances(m_sources[s], centroids);

	// tune
	{
		// replaced by hungarian.
	}

	// match
	int dim = max(m_sources.size(), centroids.size());
	Eigen::MatrixXd cost(dim, dim);
	if (m_sources.size() < centroids.size())
	{
		for (int r = 0; r < dim; r++)
			for (int c = 0; c < dim; c++)
			{
				if (r >= m_sources.size())
					cost(r, c) = 0.0;
				else
					cost(r, c) = d_s_c[r][c];
			}

		// match by hungarian algorithm
		//cerr << "cost" << endl << cost << endl;
		vector<int> assignment = hungarian_algorithm(cost);
		// final match
		for (int r = 0; r < m_sources.size(); r++)
			matches[r] = assignment[r];
	}
	else if (m_sources.size() == centroids.size())
	{
		for (int r = 0; r < dim; r++)
			for (int c = 0; c < dim; c++)
				cost(r, c) = d_s_c[r][c];

		// match by hungarian algorithm
		//cerr << "cost" << endl << cost << endl;
		matches = hungarian_algorithm(cost);
	}
	else
	{
		for (int r = 0; r < dim; r++)
			for (int c = 0; c < dim; c++)
			{
				if (c >= centroids.size())
					cost(r, c) = 0.0;
				else
					cost(r, c) = d_s_c[r][c];
			}

		// match by hungarian algorithm
		//cerr << "cost" << endl << cost << endl;
		vector<int> assignment = hungarian_algorithm(cost);
		// final match
		for (int r = 0; r < m_sources.size(); r++)
		{
			if (assignment[r] >= centroids.size())
				matches[r] = -1;
			else
				matches[r] = assignment[r];
		}
	}

	//// test. checked correct.
	//{
	//	for (int sid = 0; sid < matches.size(); sid++)
	//	{
	//		cerr << "source " << sid << " assigned target " << matches[sid] << endl;
	//	}
	//	//cerr << "con?" << endl;
	//	//getchar();
	//}

	// end.
	return matches;
}

// solve
bool DiscreteSolver::SolveOMT(int mode)
{
	// reset
	if (!m_result_s_t.empty())
	{
		for (int i = 0; i < m_result_s_t.size(); i++)
			if (!m_result_s_t[i].empty()) vector<int>().swap(m_result_s_t[i]);
		vector<vector<int>>().swap(m_result_s_t);
	}
	m_result_s_t.resize(m_sources.size());
	
	// lloyd for term 1&3
	vector<Cluster> clusters = ClusterTargets(mode);

	// match for term 2
	vector<int> matches = Match(clusters);

	// final assignment
	for (int sid = 0; sid < m_sources.size(); sid++)
	{
		int cid = matches[sid];
		if (cid == -1)
			continue;
		for (int idx = 0; idx < clusters[cid].samples.size(); idx++)
		{
			int tid = clusters[cid].samples[idx];
			m_result_s_t[sid].push_back(tid);
		}
	}

	//end.
	return true;
}