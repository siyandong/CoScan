#include "navigation.h"
#include <cmath> // _isnan
#include <math.h> // _isnan

using namespace std;

// todo: seam
void Navigation::processCurrentScene()
{
	// robot positions
	{
		m_robot_sites.clear();
		for (int rid = 0; rid < m_p_de->m_pose2d.size(); ++rid)
			m_robot_sites.push_back(Point_2(m_p_de->m_pose2d[rid].translation().x(), m_p_de->m_pose2d[rid].translation().y()));
	}

	// current scene boundary...
	{
		// boundary
		cv::Mat map_mat = m_p_de->visCellMap();
		// find boundary
		cv::Mat binary_mat(map_rows, map_cols, CV_8UC1);
		for (int i = 0; i < binary_mat.rows; i++)
		{
			for (int j = 0; j < binary_mat.cols; j++)
			{
				binary_mat.ptr<uchar>(i)[j] = 0;
				if (map_mat.ptr<cv::Vec3b>(i)[j][0] + map_mat.ptr<cv::Vec3b>(i)[j][1] + map_mat.ptr<cv::Vec3b>(i)[j][2] != 0)
					binary_mat.ptr<uchar>(i)[j] = 255;
			}
		}
		// dilate, to avoid self interact.
		cv::Mat bd_gray_mat(map_rows, map_cols, CV_8UC1);
		for (int i = 0; i < bd_gray_mat.rows; i++)
		{
			for (int j = 0; j < bd_gray_mat.cols; j++)
			{
				bd_gray_mat.ptr<uchar>(i)[j] = binary_mat.ptr<uchar>(i)[j];
			}
		}
		cv::dilate(bd_gray_mat, bd_gray_mat, cv::getStructuringElement(0, cv::Size(3, 3))); // dilate.
		cv::Mat bd_bin_mat;
		cv::threshold(bd_gray_mat, bd_bin_mat, 0, 255, cv::THRESH_BINARY);
		cv::dilate(bd_bin_mat, bd_bin_mat, cv::getStructuringElement(0, cv::Size(3, 3))); // dilate.
		cv::erode(bd_bin_mat, bd_bin_mat, cv::getStructuringElement(0, cv::Size(3, 3))); // erode. 
		cv::erode(bd_bin_mat, bd_bin_mat, cv::getStructuringElement(0, cv::Size(3, 3))); // erode. 
		// contours
		vector<vector<cv::Point>> bd_contours;
		vector<vector<cv::Point>> dilate_contours_0;
		cv::findContours(binary_mat, bd_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		cv::findContours(bd_bin_mat, dilate_contours_0, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); 
		// find biggest contour 
		int dc_max_size_0 = 0;
		int dc_max_index_0 = 0;
		for (int i = 0; i < dilate_contours_0.size(); i++)
		{
			if (dilate_contours_0[i].size()>dc_max_size_0)
			{
				dc_max_size_0 = dilate_contours_0[i].size();
				dc_max_index_0 = i;
			}
		}
		// save boundary
		m_boundary.clear();
		m_boundary = dilate_contours_0[dc_max_index_0];
/*
		// draw contours
        cv::Mat pc_resultImage = cv::Mat::zeros(map_rows, map_cols, CV_8U);
        cv::drawContours(pc_resultImage, dilate_contours_0, -1, cv::Scalar(255, 0, 255));
        cv::imshow("contours", pc_resultImage);
        cv::waitKey(0);
//*/
		// find biggest contour (origin boundary to detect frontiers)
		int c_max_size = 0;
		int c_max_index = 0;
		{
			for (int i = 0; i < bd_contours.size(); i++)
			{
				if (bd_contours[i].size()>c_max_size){
					c_max_size = bd_contours[i].size();
					c_max_index = i;
				}
			}
		}
		
		// frontiers
		{
			vector<cv::Point> samples_exploration; // samples of frontiers_exploration
			vector<cv::Point> all_exploration; // all of frontiers_exploration
			{
				// frontier mat
				cv::Mat frontier_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC1); // mat use to compute frontiers for exploration
				// all frontiers for exploration
				for (int contour_id = 0; contour_id < bd_contours.size(); contour_id++)
				{
					if (bd_contours[contour_id].size() <= 4)
						continue;
					for (int i = 0; i < bd_contours[contour_id].size(); i++)
					{
						if (m_p_de->m_recon2D.m_cellmap[bd_contours[contour_id][i].y][bd_contours[contour_id][i].x].isScanned && m_p_de->m_recon2D.m_cellmap[bd_contours[contour_id][i].y][bd_contours[contour_id][i].x].isFree)
						{ // 修改后: 筛选一些
							bool is_frontier = true;
							for (int r = -1; r <= 1; r++)
							{
								for (int c = -1; c <= 1; c++)
								{
									if (m_p_de->m_recon2D.m_cellmap[bd_contours[contour_id][i].y + r][bd_contours[contour_id][i].x + c].isScanned && m_p_de->m_recon2D.m_cellmap[bd_contours[contour_id][i].y + r][bd_contours[contour_id][i].x + c].isOccupied)
									{
										is_frontier = false;
										break;
									}
								}
							}
							if (is_frontier)
								frontier_mat.ptr<uchar>(bd_contours[contour_id][i].y)[bd_contours[contour_id][i].x] = 255;
						}
					}
				}
				// distance sample
				for (int r = 0; r < map_rows; r++)
					for (int c = 0; c < map_cols; c++)
						if (frontier_mat.ptr<uchar>(r)[c] == 255)
							all_exploration.push_back(cv::Point(c, r));
				for (int i = 0; i < all_exploration.size(); i++) // sampling
				{
					// distance 
					double min_dis = DBL_MAX;
					for (int tid = 0; tid < samples_exploration.size(); tid++) // compute distance from the closest frontier sample
					{
						Eigen::Vector2f pf(all_exploration[i].x, all_exploration[i].y);
						Eigen::Vector2f pt(samples_exploration[tid].x, samples_exploration[tid].y);
						double crt_dis = (pf - pt).norm();
						if (__isnan(crt_dis) != 0)
							crt_dis = 0;
						if (crt_dis < min_dis)
							min_dis = crt_dis;
					}
					// filter based on distance
					if (min_dis > frontier_exploration_sample_range_pixel) //10 pixel = 0.5m, 20 initial
						samples_exploration.push_back(cv::Point(all_exploration[i].x, all_exploration[i].y));
				}
			}
			// save of all exploration
			int ft_num = all_exploration.size();
			m_frontiers.clear();
			m_frontiers = all_exploration;
			if (ft_num == 0)
			{
				cerr << "no frontier." << endl;
				//getchar();
			}
		} // frontiers

		// holes
		{
			{ // find holes
				cv::Mat img = map_mat.clone();
				for (int i = 0; i < img.rows; i++)
				{
					for (int j = 0; j < img.cols; j++)
					{
						img.ptr<cv::Vec3b>(i)[j][0] = 0;
						img.ptr<cv::Vec3b>(i)[j][1] = 0;
					}
				}
				cv::Mat gray_map(img.rows, img.cols, CV_8UC1);
				cv::Mat bin_map;
				cv::cvtColor(img, gray_map, CV_BGR2GRAY);
				cv::threshold(gray_map, bin_map, 0, 255, cv::THRESH_BINARY);
				// offset
				dilate(bin_map, bin_map, cv::getStructuringElement(0, cv::Size(offset_size * 2 + 1, offset_size * 2 + 1))); // offset of obsticles, used to aviod collision between robot and object 
		


// todo: cut off scene outer boundary into pices
				if (!g_scene_boundary.empty())
				{
					// select a split point
					cv::Point mid( (g_scene_boundary[0].x + g_scene_boundary[1].x)/2, (g_scene_boundary[0].y + g_scene_boundary[1].y)/2 );
					if (m_p_de->m_recon2D.m_cellmap[mid.y][mid.x].isScanned)
					{
						if (m_p_de->m_recon2D.m_cellmap[mid.y][mid.x].isOccupied)
						{
							int dx = abs(g_scene_boundary[0].x - g_scene_boundary[1].x);
							int dy = abs(g_scene_boundary[0].y - g_scene_boundary[1].y);
							if (dx>dy)
							{
								int min_x = mid.x-1, 
									max_x = mid.x+1,
									min_y = mid.y-10,
									max_y = mid.y+10;
								for (int r = min_y; r <= max_y; r++)
									for (int c = min_x; c <= max_x; c++)
										bin_map.ptr<uchar>(r)[c] = 0;
								char pth[100];
								sprintf(pth, "result/obstical_%d.png", g_plan_iteration);
								cv::imwrite(pth, bin_map);
							}
							else
							{
								int min_x = mid.x-10, 
									max_x = mid.x+10,
									min_y = mid.y-1,
									max_y = mid.y+1;
								for (int r = min_y; r <= max_y; r++)
									for (int c = min_x; c <= max_x; c++)
										bin_map.ptr<uchar>(r)[c] = 0;
								char pth[100];
								sprintf(pth, "result/obstical_%d.png", g_plan_iteration);
								cv::imwrite(pth, bin_map);
							}
						}
						else
						{
							cerr<<"not a occupied voxel. continue?"<<endl;
							//getchar(); getchar(); getchar();
						}
					}
/*
					int min_x = col, min_y = row, max_x = 0, max_y = 0;
					for (int i = 0; i < scene_seam.size(); i++)
					{
						if (scene_seam[i].x < min_x)
							min_x = scene_seam[i].x;
						if (scene_seam[i].x > max_x)
							max_x = scene_seam[i].x;
						if (scene_seam[i].y < min_y)
							min_y = scene_seam[i].y;
						if (scene_seam[i].y > max_y)
							max_y = scene_seam[i].y;
					}
					for (int r = min_y; r <= max_y; r++)
						for (int c = min_x; c <= max_x; c++)
							bin_map.ptr<uchar>(r)[c] = 0;
					char pth[100];
					sprintf(pth, "data/offline/obstical%d.png", exe_count);
					cv::imwrite(pth, bin_map);
//*/
				}
/*
				if (!scene_seam.empty())
				{
					int min_x = col, min_y = row, max_x = 0, max_y = 0;
					for (int i = 0; i < scene_seam.size(); i++)
					{
						if (scene_seam[i].x < min_x)
							min_x = scene_seam[i].x;
						if (scene_seam[i].x > max_x)
							max_x = scene_seam[i].x;
						if (scene_seam[i].y < min_y)
							min_y = scene_seam[i].y;
						if (scene_seam[i].y > max_y)
							max_y = scene_seam[i].y;
					}
					for (int r = min_y; r <= max_y; r++)
						for (int c = min_x; c <= max_x; c++)
							bin_map.ptr<uchar>(r)[c] = 0;
					char pth[100];
					sprintf(pth, "data/offline/obstical%d.png", exe_count);
					cv::imwrite(pth, bin_map);
				}
//*/
				// contours
				vector<vector<cv::Point>> contours;
				vector<cv::Vec4i> hierarchy;
				cv::findContours(bin_map, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); // tree
				// filter hole in hole
				vector<int> dlt_idx;
				for (int i = 0; i < hierarchy.size(); i++)
				{
					if (hierarchy[i][3] != -1)
						dlt_idx.push_back(i);
				}
				// delete hole in hole
				if (dlt_idx.size() > 0){
					for (int i = dlt_idx.size() - 1; i >= 0; i--)
					{
						//cout << "i: " << i << endl;
						auto it = contours.begin() + dlt_idx[i];
						contours.erase(it);
					}
				}
		/*
				// 判断是否只有一个大边界 hole
				if (contours.size() == 1)
				{ // 判断是否是墙组成的大边界
					int min_x = col, min_y = row;
					int max_x = 0, max_y = 0;
					for (int i = 0; i < contours[0].size(); i++)
					{
						if (contours[0][i].x < min_x)
							min_x = contours[0][i].x;
						if (contours[0][i].x > max_x)
							max_x = contours[0][i].x;
						if (contours[0][i].y < min_y)
							min_y = contours[0][i].y;
						if (contours[0][i].y > max_y)
							max_y = contours[0][i].y;
					}
					if (max_y - min_y >= 190 && max_x - min_x >= 90)
					{ // todo: 如果是,把四周的墙都去掉一块把墙分开
						
					}
				}
		//*/
				// check hole[i] size, remove which size<3. 2018-10-16
				{
					vector<int> deleteIndexes;
					for (int hid = 0; hid < contours.size(); hid++)
						if (contours[hid].size() < 3) // if contour size < 3, it cant be operated as polygon. on the other hand this contour is trival.
							deleteIndexes.push_back(hid);
					if (!deleteIndexes.empty())
						for (int i = deleteIndexes.size() - 1; i >= 0; i--)
							contours.erase(contours.begin() + deleteIndexes[i]);
				}
				// save
				m_holes.clear();
				m_holes = contours;
			}
		}
	}

	return;
}

// polygon simplification. reduce number of vertexes.
bool Navigation::simplifyPolygon(Polygon_2 & poly, const double colinearThresh, bool addNoise)
{
	// check input
	if (poly.size() < 3)
		return false;
	cerr << "polygon point number = " << poly.size() << endl;
	// set up
	vector<Eigen::Vector2d> polygon;
	for (int i = 0; i < poly.size(); i++)
	{
		if (i >= 1)
		{
			Eigen::Vector2d lst(polygon.back());
			Eigen::Vector2d crt(poly[i].x(), poly[i].y());
			if ((crt - lst).norm() < 0.01)
				continue;
		}
		polygon.push_back(Eigen::Vector2d(poly[i].x(), poly[i].y()));
	}
	// find keypoints
	vector<int> keyIndexes;
	for (int i = 0; i < polygon.size(); i++)
	{
		int last = (i - 1 + polygon.size()) % polygon.size();
		int next = (i + 1) % polygon.size();
		Eigen::Vector2d vec1 = polygon[i] - polygon[last];
		vec1.normalize();
		Eigen::Vector2d vec2 = polygon[next] - polygon[i];
		vec2.normalize();
		double dangle = acos(vec1.dot(vec2));
		//if (1.000 - vec1.dot(vec2) < colinearThresh)
		if (dangle < colinearThresh)
			continue;
		keyIndexes.push_back(i); // push a keypoint
	}
	// save keypoints polygon
	vector<Point_2> newPolygon;
	for (int i = 0; i < keyIndexes.size(); i++)
		if (addNoise)
			newPolygon.push_back(Point_2(polygon[keyIndexes[i]].x() + (double)rand() / RAND_MAX / 10, polygon[keyIndexes[i]].y() + (double)rand() / RAND_MAX / 10));
		else
			newPolygon.push_back(Point_2(polygon[keyIndexes[i]].x(), polygon[keyIndexes[i]].y()));
	if (newPolygon.size() < 3)
	{
		cerr << "simplify error, polygon.size() = " << newPolygon.size() << endl;
		return false;
	}
	poly = Polygon_2(newPolygon.begin(), newPolygon.end());
	cerr << "simplified point number = " << poly.size() << endl;
	//polygon_count++;
	return true;
}

// diff between polygons.
Pwh_list_2 Navigation::differenceCGALExactKernel(Polygon_with_holes_2 domain, Polygon_2 hole)
{
	// set up 
	Pwh_list_2 res;
	ofstream ofs;
	ifstream ifs;
	// save
	ofs.open(domainFilePath);
	ofs << domain;
	ofs.clear();
	ofs.close();
	ofs.open(holeFilePath);
	ofs << hole;
	ofs.clear();
	ofs.close();
	ofs.open(communicateFile);
	ofs << 0 << endl; // initialization
	ofs.clear();
	ofs.close();
	// difference
	system(poly_diff.data());
	// result 
	int rtn;
	ifs.open(communicateFile);
	ifs >> rtn;
	ifs.clear();
	ifs.close();
	int steps = 0;
	while (rtn != 1)
	{
		if (rtn == 2)
		{
			cerr << "dsy: polygons difference result = 0" << endl;
			Pwh_list_2 res;
			res.push_back(domain);
			return res;
		}
		//if (steps > 10) // test.
		//{
		//	Pwh_list_2 res;
		//	res.push_back(domain);
		//	return res;
		//}
		ofs.open(communicateFile);
		ofs << -1 << endl;
		ofs.clear();
		ofs.close();
		system(poly_diff.data());
		ifs.open(communicateFile);
		ifs >> rtn;
		ifs.clear();
		ifs.close();
		steps++;
	}
	// load
	Polygon_with_holes_2 pass;
	ifs.open(resultFilePath);
	ifs >> pass;
	ifs.clear();
	ifs.close();
	// format
	res.push_back(pass);
	return res;
}

// load and check boundary.
Polygon_2 Navigation::load_and_check_boundary()
{
	// set up
	vector<Point_2> vertexes;
	for (int i = 0; i < m_boundary.size(); i++)
		vertexes.push_back(Point_2(m_boundary[i].x, -m_boundary[i].y));
	Polygon_2 boundary(vertexes.begin(), vertexes.end());
	if (boundary.area() < 0) // anti clock wise
		boundary.reverse_orientation();
	// check same points caused self intersection
	bool self_intersect = false;
	vector<Point_2> points;
	set<Point_2> test_points;
	for (size_t pid = 0; pid < boundary.size(); pid++)
	{
		if (test_points.find(Point_2(boundary[pid].x(), boundary[pid].y())) == test_points.end()){
			test_points.insert(Point_2(boundary[pid].x(), boundary[pid].y()));
			points.push_back(Point_2(boundary[pid].x(), boundary[pid].y()));
		}
		else{
			self_intersect = true;
			// 朝外侧扰动
			double vx = boundary[pid].x() - boundary[pid - 1].x();
			double vy = boundary[pid].y() - boundary[pid - 1].y();
			double theta = -0.1; //逆时针排序时为负
			vx = cos(theta)*vx - sin(theta)*vy;
			vy = sin(theta)*vx + cos(theta)*vy;
			double x = boundary[pid - 1].x() + vx;
			double y = boundary[pid - 1].y() + vy;
			test_points.insert(Point_2(x, y));
			points.push_back(Point_2(x, y));
		}
	}
	if (self_intersect)
		boundary = Polygon_2(points.begin(), points.end());
	if (boundary.area() < 0)
		boundary.reverse_orientation();
	// simplify boundary polygon
	simplifyPolygon(boundary, 0.2, false);
	// return.
	return boundary;
}

// load and check holes.
vector<Polygon_2> Navigation::load_and_check_holes(Polygon_2 boundary, vector<Polygon_2> & origin_holes)
{
	// set up
	vector<Polygon_2> holes;
	origin_holes.clear();
	vector<cv::Point> bdry;
	for (int i = 0; i < boundary.size(); i++)
		bdry.push_back(cv::Point(round(boundary[i].x()), -round(boundary[i].y())));
	// load holes
	for (int hid = 0; hid < m_holes.size(); hid++)
	{
		Polygon_2 p_pass;
		for (int vid = 0; vid < m_holes[hid].size(); vid++)
		{
			p_pass.push_back(Point_2(m_holes[hid][vid].x, -m_holes[hid][vid].y));
		}
		origin_holes.push_back(p_pass);
	}
	// compute holes
	{
		// holes赋值
		for (int i = 0; i < origin_holes.size(); i++)
		{
			if (origin_holes[i].area() > 0)
				origin_holes[i].reverse_orientation(); // 需要顺时针排序即面积为负
			holes.push_back(origin_holes[i]);
		}
		// 对于每个hole判断其边界是否自交
		for (int hid = 0; hid < holes.size(); hid++)
		{
			bool self_intersect = false;
			vector<Point_2> points;
			set<Point_2> test_points;
			for (int pid = 0; pid < holes[hid].size(); pid++)
			{
				if (test_points.find(Point_2(holes[hid][pid].x(), holes[hid][pid].y())) == test_points.end())
				{
					test_points.insert(Point_2(holes[hid][pid].x(), holes[hid][pid].y()));
					points.push_back(Point_2(holes[hid][pid].x(), holes[hid][pid].y()));
				}
				else
				{
					self_intersect = true;
					// 朝外侧扰动
					double vx = holes[hid][pid].x() - holes[hid][pid - 1].x();
					double vy = holes[hid][pid].y() - holes[hid][pid - 1].y();
					double theta = 0.1;//顺时针排序时为正
					vx = cos(theta)*vx - sin(theta)*vy;
					vy = sin(theta)*vx + cos(theta)*vy;
					double x = holes[hid][pid - 1].x() + vx;
					double y = holes[hid][pid - 1].y() + vy;
					test_points.insert(Point_2(x, y));
					points.push_back(Point_2(x, y));
				}
			}
			if (self_intersect)
				holes[hid] = Polygon_2(points.begin(), points.end());
			if (holes[hid].area() < 0)
				holes[hid].reverse_orientation();
		}
	}
	// simplify
	for (int hid = 0; hid < holes.size(); hid++)
	{
		simplifyPolygon(holes[hid], 0.1, true);
	}
	// end.
	return holes;
}

// done.
void Navigation::correct_boundary_holes(Polygon_2 & boundary, std::vector<Polygon_2> & holes)
{
	// pre check
	vector<int> dlt_idx;
	for (size_t hid = 0; hid < holes.size(); hid++)
		if (holes[hid].size() == 0)
			dlt_idx.push_back(hid);
	if (!dlt_idx.empty())
	{
		for (int id = dlt_idx.size() - 1; id >= 0; id--)
		{
			holes.erase(holes.begin() + dlt_idx[id]);
		}
	}
	if (holes.empty())
	{
		cerr << "error, holes empty" << endl;
	}
	// domain
	Pwh_list_2 results;
	Polygon_with_holes_2 domain;
	domain.outer_boundary() = boundary;
	for (int hid = 0; hid < holes.size(); hid++)
	{
		// difference use exact kernel to avoid numerical error
		results = differenceCGALExactKernel(domain, holes[hid]);
		//cerr << "results(polygons) size = " << results.size() << endl;
		//cerr << "outer boundary area = " << results.begin()->outer_boundary().area() << endl;
		// update domain
		if (results.size() != 1
		){ // if domain become not continious
			int max_size = 0;
			for (auto i = results.begin(); i != results.end(); i++)
				if (i->outer_boundary().size() > max_size)
					max_size = i->outer_boundary().size();
			for (auto i = results.begin(); i != results.end(); i++)
			{
				if (i->outer_boundary().size() == max_size)
				{
					domain = Polygon_with_holes_2(*i);
					break;
				}
			}
			results.clear();
			continue;
		}
		domain = Polygon_with_holes_2(*results.begin());
		results.clear();
	}
	// reset boundary and holes
	boundary.clear();
	boundary = Polygon_2(domain.outer_boundary());
	holes.clear();
	for (auto i = domain.holes_begin(); i != domain.holes_end(); i++)
		holes.push_back(*i);
	// 防止数值错误的扰动
	{
		vector<Point_2> pass_polygon;
		for (int pid = 0; pid < boundary.size(); pid++)
			pass_polygon.push_back(Point_2(boundary[pid].x() + (rand() / RAND_MAX) / 10, boundary[pid].y() + (rand() / RAND_MAX) / 10));
		boundary = Polygon_2(pass_polygon.begin(), pass_polygon.end());
		// 判断boundary的边界是否自交
		bool self_intersect = false;
		vector<Point_2> points;
		set<Point_2> test_points;
		for (size_t pid = 0; pid < boundary.size(); pid++)
		{
			if (test_points.find(Point_2(boundary[pid].x(), boundary[pid].y())) == test_points.end())
			{
				test_points.insert(Point_2(boundary[pid].x(), boundary[pid].y()));
				points.push_back(Point_2(boundary[pid].x(), boundary[pid].y()));
			}
			else
			{
				self_intersect = true;
				// 朝外侧扰动
				double vx = boundary[pid].x() - boundary[pid - 1].x();
				double vy = boundary[pid].y() - boundary[pid - 1].y();
				double theta = -0.1;//逆时针排序时为负
				vx = cos(theta)*vx - sin(theta)*vy;
				vy = sin(theta)*vx + cos(theta)*vy;
				double x = boundary[pid - 1].x() + vx;
				double y = boundary[pid - 1].y() + vy;
				test_points.insert(Point_2(x, y));
				points.push_back(Point_2(x, y));
			}
		}
		if (self_intersect)
			boundary = Polygon_2(points.begin(), points.end());
		if (boundary.area() < 0)
			boundary.reverse_orientation();
	}
	// end.
	return;
}

// generate robot move domain.
bool Navigation::robotMoveDomainProcess(Polygon_2 & boundary, vector<Polygon_2> & holes, vector<Polygon_2> & origin_holes)
{
	// domain boundary
	boundary = load_and_check_boundary();
	// domain holes
	holes = load_and_check_holes(boundary, origin_holes);
	// correction: compute domain and reset boundary & holes
	correct_boundary_holes(boundary, holes); // by the difference of polygons
	// end.
	return true;
}

// cgal: mark_domains
void
mark_domains(CDT& ct,
CDT::Face_handle start,
int index,
std::list<CDT::Edge>& border)
{
	if (start->info().nesting_level != -1){
		return;
	}
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);
	while (!queue.empty()){
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if (fh->info().nesting_level == -1){
			fh->info().nesting_level = index;
			for (int i = 0; i < 3; i++){
				CDT::Edge e(fh, i);
				CDT::Face_handle n = fh->neighbor(i);
				if (n->info().nesting_level == -1){
					if (ct.is_constrained(e)) border.push_back(e);
					else queue.push_back(n);
				}
			}
		}
	}
}

// cgal: mark_domains
void
mark_domains(CDT& cdt)
{
	for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
		it->info().nesting_level = -1;
	}
	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);
	while (!border.empty()){
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);
		if (n->info().nesting_level == -1){
			mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
		}
	}
}

// save to obj and off
void saveCDT_geodesic(CDT cdt)
{ // save mesh
	vector<int> rbt_sites_indexes;
	vector<Point_2> vertices; // vertices in domain
	// filter vertices in domain
	{
		vector<Point_2> vertices_all;
		for (CDT::Finite_vertices_iterator vit = cdt.finite_vertices_begin();
			vit != cdt.finite_vertices_end(); ++vit){
			vertices_all.push_back(Point_2(vit->point().x(), vit->point().y()));
		}
		//cout << vertices_all.size() << endl;
		// flag: to remain only indomain vertices
		//int flag[10000];
		int* flag = new int[60000];
		for (size_t i = 0; i < vertices_all.size(); i++)
		{
			flag[i] = 0;
		}
		for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
			fit != cdt.finite_faces_end(); ++fit)
		{
			if (fit->info().in_domain()) {
				for (size_t i = 0; i < 3; i++)
				{
					for (size_t j = 0; j < vertices_all.size(); j++)
					{
						if (fit->vertex(i)->point() == vertices_all[j]){
							flag[j] = 1;
							break;
						}
					}
				}
			}
		}
		for (size_t i = 0; i < vertices_all.size(); i++)
		{
			if (flag[i] == 1){
				vertices.push_back(Point_2(vertices_all[i].x(), vertices_all[i].y()));
			}
		}
		delete[] flag;
	}
	// save cdt to obj file: use for interior distance
	{
		ofstream obj_out(cdt_obj_path);
		//cerr << "saving obj file..."; // 12-25
		{// write to obj file
			for (size_t i = 0; i < vertices.size(); i++)
			{
				obj_out << "v " << vertices[i].x() << " " << vertices[i].y() << " 0" << endl;
			}
			// write faces
			for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
				fit != cdt.finite_faces_end(); ++fit)
			{
				if (fit->info().in_domain()) {
					obj_out << "f ";
					for (size_t i = 0; i < 3; i++)
					{
						for (size_t j = 0; j < vertices.size(); j++)
						{
							if (fit->vertex(i)->point() == vertices[j]){
								obj_out << j + 1 << " ";
								break;
							}
						}
					}
					obj_out << endl;
				}
			}
		}
		obj_out.close();
		//cerr << "done." << endl; // 12-25
		//getchar(); // 12-25
	}
/*
	// save cdt to off file: use for polyhedron
	{
		ofstream off_out;
		off_out.open(cdt_off_path);
		off_out << "OFF" << endl;
		// vertices
		vector<Point_2> vertices_all;
		for (CDT::Finite_vertices_iterator vit = cdt.finite_vertices_begin();
			vit != cdt.finite_vertices_end(); ++vit){
			vertices_all.push_back(Point_2(vit->point().x(), vit->point().y()));
		}
		//cout << vertices_all.size() << endl;
		// flag: to remain only indomain vertices
		//int flag[10000];
		int* flag = new int[60000];
		for (size_t i = 0; i < vertices_all.size(); i++)
		{
			flag[i] = 0;
		}
		for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
			fit != cdt.finite_faces_end(); ++fit)
		{
			if (fit->info().in_domain()) {
				for (size_t i = 0; i < 3; i++)
				{
					for (size_t j = 0; j < vertices_all.size(); j++)
					{
						if (fit->vertex(i)->point() == vertices_all[j]){
							flag[j] = 1;
							break;
						}
					}
				}
			}
		}
		vector<Point_2> vertices;
		for (size_t i = 0; i < vertices_all.size(); i++)
		{
			if (flag[i] == 1){
				vertices.push_back(Point_2(vertices_all[i].x(), vertices_all[i].y()));
			}
		}
		delete[] flag;
		//cout << vertices.size() << endl;
		int f_count = 0;
		for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
			fit != cdt.finite_faces_end(); ++fit)
			if (fit->info().in_domain())
				++f_count;
		off_out << vertices.size() << " " << f_count << " 0" << endl;
		// write vertices
		for (size_t i = 0; i < vertices.size(); i++)
		{
			off_out << vertices[i].x() << " " << vertices[i].y() << " 0" << endl;
		}
		// write f
		for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
			fit != cdt.finite_faces_end(); ++fit)
		{
			if (fit->info().in_domain()) {
				off_out << "3 ";
				for (size_t i = 0; i < 3; i++)
				{
					for (size_t j = 0; j < vertices.size(); j++)
					{
						if (fit->vertex(i)->point() == vertices[j]){
							off_out << j << " ";
							break;
						}
					}
				}
				off_out << endl;
			}
		}
		off_out.close();
	}
//*/
	return;
}

// CDT.
bool Navigation::generateMoveDomainCDT(Polygon_2 boundary, vector<Polygon_2> holes, vector<Polygon_2> origin_holes, vector<ScanningTask> tasks, CDT & cdt)
{
	set<Point_2> tmpSet; // sites in cdt: include boundary, holes, tasks, rbts and random
	{ // Insert the polygons into a constrained triangulation
		// set boundary
		cdt.insert_constraint(boundary.vertices_begin(), boundary.vertices_end(), true);
		// insert boundary points
		for (size_t i = 0; i < boundary.size(); i++)
		{
			tmpSet.insert(Point_2(boundary[i].x(), boundary[i].y()));
		}
		// set holes
		for (size_t id = 0; id < holes.size(); id++)
			cdt.insert_constraint(holes[id].vertices_begin(), holes[id].vertices_end(), true);
		// insert holes points
		for (size_t i = 0; i < holes.size(); i++)
		{
			for (size_t j = 0; j < holes[i].size(); j++)
			{
				tmpSet.insert(Point_2(holes[i][j].x(), holes[i][j].y()));
			}
		}
		//// insert frontiers points
		//for (size_t i = 0; i < frontiers.size(); i++)
		//{
		//	tmpSet.insert(Point_2(frontiers[i].x(), frontiers[i].y()));
		//}
		// insert task points
		for (size_t i = 0; i < tasks.size(); i++)
		{
			tmpSet.insert(Point_2(tasks[i].view.pose.translation().x(), tasks[i].view.pose.translation().y()));
		}
		// set robot sites
		{
			// check if in hole
			{ 
				for (size_t i = 0; i < m_robot_sites.size(); i++)
				{
					for (size_t j = 0; j < holes.size(); j++)
					{
						CGAL::Bounded_side bs = CGAL::bounded_side_2(holes[j].vertices_begin(), holes[j].vertices_end(), m_robot_sites[i]);
						if (bs == CGAL::ON_BOUNDED_SIDE)
						{
							// 如果在障碍物（原始）内部，报错
							if (CGAL::bounded_side_2(origin_holes[j].vertices_begin(), origin_holes[j].vertices_end(), m_robot_sites[i]) == CGAL::ON_BOUNDED_SIDE)
							{
								cerr << "error, move into a hole" << endl;
							}
							// find closest point
							double min_dis = DBL_MAX;
							int min_index = -1;
							for (size_t k = 0; k < holes[j].size(); k++)
							{
								if (
									sqrt((holes[j][k].x() - m_robot_sites[i].x())*(holes[j][k].x() - m_robot_sites[i].x())
									+ (holes[j][k].y() - m_robot_sites[i].y())*(holes[j][k].y() - m_robot_sites[i].y()))
									< min_dis
									)
								{
									min_dis = sqrt((holes[j][k].x() - m_robot_sites[i].x())*(holes[j][k].x() - m_robot_sites[i].x())
										+ (holes[j][k].y() - m_robot_sites[i].y())*(holes[j][k].y() - m_robot_sites[i].y()));
									min_index = k;
								}
							}
							// 改为边缘处的点
							m_robot_sites[i] = Point_2(holes[j][min_index].x(), holes[j][min_index].y());
						}
					}
				}
			}
		}
		// set tssk sites
		{
			for (int i = 0; i < tasks.size(); i++)
				tmpSet.insert(Point_2(tasks[i].view.pose.translation().x(), tasks[i].view.pose.translation().y()));
		}
		// set m_robot_sites and random sites
		{
			for (size_t i = 0; i < m_robot_sites.size(); i++)
				tmpSet.insert(Point_2(m_robot_sites[i].x(), m_robot_sites[i].y()));
			{// ragular random sites
				CGAL::Bbox_2 box = boundary.bbox();
				int site_num = ceil(sqrt(random_site_num));
				double delta_x = (box.xmax() - box.xmin()) / site_num;
				double delta_y = (box.ymax() - box.ymin()) / site_num;
				for (int i = 0; i < site_num; i++)
				{
					for (int j = 0; j < site_num; j++)
					{
						double x = box.xmin() + i*delta_x + (double)(rand() % 10) / 6;
						double y = box.ymin() + j*delta_y + (double)(rand() % 10) / 6;
						Point_2 pt(x, y);
						//cout << "plot(" << x << ", " << y << ", 'bo'); hold on;" << endl;
						bool rand_ok = true;
						if (CGAL::bounded_side_2(boundary.vertices_begin(), boundary.vertices_end(), pt) != CGAL::ON_BOUNDED_SIDE)
							rand_ok = false;
						for (size_t hid = 0; hid < holes.size(); hid++)
							if (CGAL::bounded_side_2(holes[hid].vertices_begin(), holes[hid].vertices_end(), pt) != CGAL::ON_UNBOUNDED_SIDE)
							{
								rand_ok = false;
								break;
							}
						//if (CGAL::bounded_side_2(boundary.vertices_begin(), boundary.vertices_end(), pt) == CGAL::ON_BOUNDED_SIDE)
						if (rand_ok)
						{
							double distanceSquare = FLT_MAX;
							for (set<Point_2>::const_iterator it = tmpSet.begin(); it != tmpSet.end(); ++it)
							{
								if ((*it - pt).squared_length() < distanceSquare)
									distanceSquare = (*it - pt).squared_length();
							}
							if (distanceSquare > 2e-3 * 2e-3)
								tmpSet.insert(pt);
						}
					}
				}
			}
		}
		// insert sites
		//cdt.insert(tmpSet.begin(), tmpSet.end(), false); // origin code.
		cdt.insert(tmpSet.begin(), tmpSet.end()); 		   // linux func.. is it ok?
		// Mark facets that are inside the domain bounded by the polygon
		mark_domains(cdt);
	}
	return true;
}

// load and check frontiers, saved in the variable frontiers
vector<Point_2> Navigation::load_and_check_frontiers(Polygon_2 & boundary, vector<Polygon_2> & holes)
{
	for (int fid = 0; fid < m_frontiers.size(); ++fid)
	{
		m_frontiers_p2.push_back(Point_2(m_frontiers[fid].x, -m_frontiers[fid].y));
	}
	return m_frontiers_p2;
}

// remove invalid frontiers, not finish
vector<int> Navigation::preprocess_frontiers(Polygon_2 outer, vector<Polygon_2> iner)
{
	// set up
	m_frontierList.clear();
	vector<int> indexes;
	vector<Point_2> tasks;
	vector<cv::Point> boundary;
	for (int pid = 0; pid < outer.size(); pid++)
	{
		boundary.push_back(cv::Point(round(outer[pid].x()), -round(outer[pid].y())));
	}
	vector<vector<cv::Point>> holes;
	for (int hid = 0; hid < iner.size(); hid++)
	{
		vector<cv::Point> hole;
		for (int pid = 0; pid < iner[hid].size(); pid++)
		{
			hole.push_back(cv::Point(round(iner[hid][pid].x()), -round(iner[hid][pid].y())));
		}
		holes.push_back(hole);
	}
	// determine if the frontier is a task
	for (int i = 0; i < m_frontiers_p2.size(); i++)
	{
		// check reachable
		bool visible = true;
		string err = "";
		// voxels that cant scan
		if (visible)
			for (auto unvisible_task = task_maybe_invalid.begin(); unvisible_task != task_maybe_invalid.end(); unvisible_task++)
			{
				if (fabs(m_frontiers_p2[i].x() - unvisible_task->x()) < 1 && fabs(m_frontiers_p2[i].y() - unvisible_task->y()) < 1)
				{
					visible = false;
					err = "cant scan.";
					break;
				}
			}
		// voxels that out of boundary
		if (visible)
			if (cv::pointPolygonTest(boundary, cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), false) < 0)
			{
				// if distance < offset_size, its may be valid.
				double d = fabs(cv::pointPolygonTest(boundary, cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), true));
				if (d > offset_size*1.41)
				{
					visible = false;
					err = "out of boundary.";
				}
			}
		// voxels that in holes
		if (visible)
			for (int hid = 0; hid < holes.size(); hid++)
			{
				if (cv::pointPolygonTest(holes[hid], cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), false) > 0)
				{
					// if distance < offset_size, its may be valid.
					double d = fabs(cv::pointPolygonTest(holes[hid], cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), true));
					if (d > (offset_size-1)*1.41)
					{
						visible = false;
						err = "in hole.";
						break;
					}
					//visible = false;
					//err = "in hole.";
					//break;
				}
			}
		// voxels that out of scene boundary
		if (visible)
			if (!g_scene_boundary.empty()) // 20200311
			{
				//if (cv::pointPolygonTest(g_scene_boundary, cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), false) < 0)
				if (cv::pointPolygonTest(g_scene_boundary, cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), false) <= 0) 
				{
					visible = false;
					err = "out of scene_boundary.";
				}
				else if(cv::pointPolygonTest(g_scene_boundary, cv::Point(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y())), true) < 2) // < 3
				{
					visible = false;
					err = "out of scene_boundary. too close to boundary.";
				}
			}
//*/
		// voxels that trivial
		if (visible)
		{
			const int delta = 3;
			const int thresh = 6; // default 7 is ok.
			int unknownCounter = 0;
			cv::Point vox(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y()));
			int beg_r = vox.y - delta < 0 ? 0 : vox.y - delta;
			int end_r = vox.y + delta > map_rows - 1 ? map_rows - 1 : vox.y + delta;
			int beg_c = vox.x - delta < 0 ? 0 : vox.x - delta;
			int end_c = vox.x + delta > map_cols - 1 ? map_cols - 1 : vox.x + delta;
			for (int r = beg_r; r <= end_r; r++)
				for (int c = beg_c; c <= end_c; c++)
					if (!m_p_de->m_recon2D.m_cellmap[r][c].isScanned)
						unknownCounter++;
			if (unknownCounter <= thresh)
			{
				visible = false;
				err = "trivial.";
			}
		}
/*
// todo: confidence map 2d
		// if unvisible, continue
		if (!visible)
		{
			int r = -(int)round(m_frontiers_p2[i].y());
			int c = (int)round(m_frontiers_p2[i].x());
			confidence_map_2d[r][c] = confidence_max;
			continue;
		}
//*/

		// store visible frontier to m_frontierList
/* origin code	    
		tasks.push_back(Point_2(m_frontiers_p2[i].x(), m_frontiers_p2[i].y()));
		indexes.push_back(i);
		FrontierElement fe(Point_2(m_frontiers_p2[i].x(), m_frontiers_p2[i].y()));
		m_frontierList.push_back(fe);
//*/

		// dsy 20-03-21
		if (visible)
		{
			cv::Point crt_p(round(m_frontiers_p2[i].x()), -round(m_frontiers_p2[i].y()));
			bool already_explored = true;
			int radius = 1;
			for (int dr = -radius; dr <= radius; ++dr)
			{
				for (int dc = -radius; dc <= radius; ++dc)
				{
					if (!m_p_de->m_recon2D.m_cellmap[crt_p.y+dr][crt_p.x+dc].isScanned)
					{
						already_explored = false;
					}
				}
			}
			if (already_explored)
			{
				visible = false;
				err = 'already explored.';
			}
		}

		// dsy 19-10-07
		if (visible)
		{
			tasks.push_back(Point_2(m_frontiers_p2[i].x(), m_frontiers_p2[i].y()));
			indexes.push_back(i);
			FrontierElement fe(Point_2(m_frontiers_p2[i].x(), m_frontiers_p2[i].y()));
			m_frontierList.push_back(fe);
		}

	}
//*/
	return indexes;
}

// compute location map
cv::Mat Navigation::computeLoationMap()
{
	// set up
	const int thresh = 30;
	const int kernelR = 3;
	// input
	cv::Mat img = m_p_de->visCellMap();
	// free space
	cv::Mat freeSpaces = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (int r = 0; r < img.rows; r++)
	{
		for (int c = 0; c < img.cols; c++)
		{
			if (img.ptr<cv::Vec3b>(r)[c][1] == 255)
				freeSpaces.ptr<uchar>(r)[c] = 10;
		}
	}
	// location map
	cv::Mat LocationMap = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	LocationMap += freeSpaces;
	// computing
	for (int step = 0; step < thresh; step++)
	{
		cv::erode(freeSpaces, freeSpaces, cv::getStructuringElement(0, cv::Size(kernelR * 2, kernelR * 2)));
		LocationMap += freeSpaces;
	}
	return LocationMap;
}

// check view ray validness(without occlusion) 2018-09-12. not finish
bool Navigation::checkViewRayValidness(cv::Point source, cv::Point target)
{
	// set up
	const int invalidValue = 255;
	cv::Point beg(source);
	cv::Point end(target);
	cv::Mat domain = cv::Mat::zeros(map_rows, map_cols, CV_8UC1);
	for (int r = 0; r < domain.rows; r++)
	{
		for (int c = 0; c < domain.cols; c++)
		{
			// not allow view through unknown
			if (m_p_de->m_recon2D.m_cellmap[r][c].isScanned && m_p_de->m_recon2D.m_cellmap[r][c].isFree)
			{
				// free spaces value '0' 
			}
			else
			{
				// unknown and occupied spaces value 'invalidValue'
				domain.ptr<uchar>(r)[c] = invalidValue;
			}
		}
	}
	// DDA
	bool occlusion = false;
	int dx = end.x - beg.x;
	int dy = end.y - beg.y;
	if (dx == 0)
	{
		int beg_y = beg.y < end.y ? beg.y : end.y;
		int end_y = beg.y > end.y ? beg.y : end.y;
		for (int y = beg_y; y < end_y; y++)
			if (domain.ptr<uchar>(y)[beg.x] == invalidValue)
			{
				occlusion = true;
				break;
			}
	}
	else if (dy == 0)
	{
		int beg_x = beg.x < end.x ? beg.x : end.x;
		int end_x = beg.x > end.x ? beg.x : end.x;
		for (int x = beg_x; x < end_x; x++)
			if (domain.ptr<uchar>(beg.y)[x] == invalidValue)
			{
				occlusion = true;
				break;
			}
	}
	else if (dx == 0 && dy == 0){}
	else 
	{
		int MaxStep = abs(dx) > abs(dy) ? abs(dx) : abs(dy); // iteration step
		double fXUnitLen = 1.0;  // delta x
		double fYUnitLen = 1.0;  // delta y
		fYUnitLen = (double)(dy) / (double)(MaxStep);
		fXUnitLen = (double)(dx) / (double)(MaxStep);
		double x = (double)(beg.x);
		double y = (double)(beg.y);
		// step
		for (long i = 1; i < MaxStep; i++) // '<' or '<=', not include end point
		{
			x = x + fXUnitLen;
			y = y + fYUnitLen;
			if (domain.ptr<uchar>((int)round(y))[(int)round(x)] == invalidValue)
			{
				occlusion = true;
				break;
			}
		}
	}
	return !occlusion; // return valid if not occluded
}

// frustum contour.
vector<cv::Point> Navigation::get_frustum_contuor(iro::SE2 pose)
{
	// set up
	Eigen::Vector2d position(pose.translation().x(), pose.translation().y());
	Eigen::Vector2d direction(0, 1);
	{
		double angle = pose.rotation().angle();
		Eigen::Matrix2d r;
		r << cos(angle), -sin(angle),
			sin(angle), cos(angle);
		direction = r*direction;
	}
	//cerr << "position = " << position.transpose() << endl;
	//cerr << "direction = " << direction.transpose() << endl;
	// range
	double scale = (double)m_p_de->scan_max_range / 1000 / map_cellsize;
	scale /= cos(PI / 6);
	direction = scale*direction;
	// left end point
	Eigen::Vector2d lp;
	{
		Eigen::Vector2d vec(direction);
		double angle = PI / 6;
		Eigen::Matrix2d r;
		r << cos(angle), -sin(angle),
			sin(angle), cos(angle);
		vec = r*vec;
		lp = position + vec;
	}
	//cerr << "lp = " << lp.transpose() << endl;
	// right end point
	Eigen::Vector2d rp;
	{
		Eigen::Vector2d vec(direction);
		double angle = -PI / 6;
		Eigen::Matrix2d r;
		r << cos(angle), -sin(angle),
			sin(angle), cos(angle);
		vec = r*vec;
		rp = position + vec;
	}
	//cerr << "rp = " << rp.transpose() << endl;
	// coor sys trans
	cv::Point p_o((int)round(position.x()), -(int)round(position.y()));
	cv::Point p_l((int)round(lp.x()), -(int)round(lp.y()));
	cv::Point p_r((int)round(rp.x()), -(int)round(rp.y()));
	//cerr << "p_o = " << p_o << endl;
	//cerr << "p_l = " << p_l << endl;
	//cerr << "p_r = " << p_r << endl;
	vector<cv::Point> result;
	result.push_back(p_o);
	result.push_back(p_l);
	result.push_back(p_r);
	return result;
}

// NBV for frontiers
vector<NextBestView> Navigation::generateViewsFrontiers(bool enableSampling, double sampleRate)
{
	// if srand
	srand((unsigned)time(NULL)); 
	// set up
	vector<NextBestView> nbvs;
	vector<FrontierElement> frontier_list;
	for (int i = 0; i < m_frontierList.size(); i++)
	{
		if (enableSampling)
		{
			if ((double)rand() / RAND_MAX < sampleRate)
			{
				frontier_list.push_back(m_frontierList[i]);
			}
		}
		else
		{
			frontier_list.push_back(m_frontierList[i]);
		}
	}

	// test record sampled frontier
	{
		cv::Mat statement = m_p_de->visCellMap();
		cv::Mat temp = statement.clone();
		// draw frontiers in list
		for (int fid = 0; fid < m_frontierList.size(); fid++)
		{
			int x = m_frontierList[fid].position.x();
			int y = -m_frontierList[fid].position.y();
			cv::circle(temp, cv::Point(x, y), 0, CV_RGB(255, 255, 255));
			cv::circle(temp, cv::Point(x, y), 1, CV_RGB(255, 255, 255));
			cv::circle(temp, cv::Point(x, y), 2, CV_RGB(255, 255, 255));
		}
		// draw sampled frontiers 
		for (int fid = 0; fid < frontier_list.size(); fid++)
		{
			int x = frontier_list[fid].position.x();
			int y = -frontier_list[fid].position.y();
			cv::circle(temp, cv::Point(x, y), 1, CV_RGB(255, 0, 255));
		}
		// draw invalid frontiers
		for (auto it = task_maybe_invalid.begin(); it != task_maybe_invalid.end(); it++)
		{
			int x = it->x();
		 	int y = -it->y();
		 	cv::circle(temp, cv::Point(x, y), 1, CV_RGB(0, 255, 255));
		}
		// for (int fid = 0; fid < task_maybe_invalid.size(); fid++)
		// {
		// 	int x = task_maybe_invalid[fid].x();
		// 	int y = -task_maybe_invalid[fid].y();
		// 	cv::circle(temp, cv::Point(x, y), 1, CV_RGB(0, 255, 255));
		// }
		// save file
		char output_path[200];
		sprintf(output_path, "result/sampled_frontier_%d.png", g_plan_iteration);
		cv::imwrite(output_path, temp);
	}

	cv::Mat locaMap = computeLoationMap(); // known region structure
	// generate NBVs
	while (!frontier_list.empty())
	{
		FrontierElement current_frontier = frontier_list.front();
		cv::Point fp(current_frontier.position.x(), -current_frontier.position.y());
		vector<cv::Point> vps;
		const int rangeMIN = 10; // pixel
		const int rangeMAX = 40; // pixel
		int delta = rangeMAX;
		int beg_r = fp.y - delta < 0 ? 0 : fp.y - delta;
		int end_r = fp.y + delta > map_rows - 1 ? map_rows - 1 : fp.y + delta;
		int beg_c = fp.x - delta < 0 ? 0 : fp.x - delta;
		int end_c = fp.x + delta > map_cols - 1 ? map_cols - 1 : fp.x + delta;
		for (int r = beg_r; r <= end_r; r++)
		{
			for (int c = beg_c; c <= end_c; c++)
			{
				//cv::circle(visual, cv::Point(c, r), 2, CV_RGB(0, 255, 0)); // test
				if (m_p_de->m_recon2D.m_cellmap[r][c].isScanned && m_p_de->m_recon2D.m_cellmap[r][c].isFree) // valid position in domain
				{
					if ((double)rand() / RAND_MAX < 0.995)	// random sample, 0.9 is ok but not efficient. 
						continue;						
					double eDistance = m_metric.get_euclidean_distance(cv::Point(c, r), fp);
					if (eDistance > rangeMIN) // outside min of scan range
					{
						if (checkViewRayValidness(cv::Point(c, r), fp))
						{
							//cv::circle(visual, cv::Point(c, r), 2, CV_RGB(255, 0, 255)); // test
							vps.push_back(cv::Point(c, r));
						}
					}
				}
			}
		}
		// sample
		const int numMax = 10;
		if (vps.size() > numMax)
		{
			int num = vps.size() - numMax;
			for (int i = 0; i < num; i++)
			{
				int idx = (int)floor(rand() % (vps.size() - 1));
				vps.erase(vps.begin() + idx);
			}
		}
		// score these candidate positions
		vector<double> scores;
		scores.resize(vps.size());
		for (int idx = 0; idx < scores.size(); idx++)
		{
			scores[idx] = locaMap.ptr<uchar>(vps[idx].y)[vps[idx].x];
		}
		double max_score = -1;
		double max_index = -1;
		for (int idx = 0; idx < vps.size(); idx++) // select the best
		{
			if (max_score < scores[idx])
			{
				max_score = scores[idx];
				max_index = idx;
			}
		}
		// valid check
		if (max_index == -1)
		{
			task_maybe_invalid.insert(Point_2(frontier_list.begin()->position.x() + (double)rand() / RAND_MAX / 10, frontier_list.begin()->position.y()));
			frontier_list.erase(frontier_list.begin());
			//cout << "no valid views, continue to next frontier..." << endl;
			continue;
		}
		//cerr<<"task_maybe_invalid.size() = "<<task_maybe_invalid.size()<<endl;
		//getchar();getchar();getchar();
		// save NBV
		cv::Point nbp = vps[max_index];
		Eigen::Vector2d base(0.0, 1.0); // se2 coor
		Eigen::Vector2d dire(fp.x - nbp.x, -fp.y + nbp.y); // se2 coor
		dire.normalize();
		double theta = acos(base.dot(dire) / (dire.norm() * base.norm()));
		if (dire.x() > 0)
			theta = -theta;
		NextBestView nbv(iro::SE2(nbp.x, -nbp.y, theta), 1, nbvs.size());
		//cerr << "theta = " << theta << endl; // test
		nbvs.push_back(nbv);
		//cout << "generated a view stored." << endl;
		// delete covered frontiers // frustum and visibility check. without openmp. better than with openmp, why?
		{
			// erase this frontier
			frontier_list.erase(frontier_list.begin()); 
			// erase covered frontiers by this view
			vector<int> dlt_indexes;
			vector<cv::Point> nbv_frustum = get_frustum_contuor(nbv.pose);
			for (int i = 0; i < frontier_list.size(); i++)
			{
				// check view frustum
				if (cv::pointPolygonTest(nbv_frustum, cv::Point((int)round(frontier_list[i].position.x()), -(int)round(frontier_list[i].position.y())), false) > 0) // in field of frustum
				{
					// check visiability 
					cv::Point beg((int)round(nbv.pose.translation().x()), -(int)round(nbv.pose.translation().y()));
					cv::Point end((int)round(frontier_list[i].position.x()), -(int)round(frontier_list[i].position.y()));
					// if visiable, delete the covered frontiers from queue
					if (checkViewRayValidness(beg, end))
						dlt_indexes.push_back(i);
				}
			}
			if (!dlt_indexes.empty())
			{
				for (int i = dlt_indexes.size() - 1; i >= 0; i--)
				{
					frontier_list.erase(frontier_list.begin() + dlt_indexes[i]);
				}
			}	
		}
	}
	// end.
	return nbvs;
}

// tasks func
vector<NextBestView> Navigation::compute_frontier_tasks(Polygon_2 boundary, vector<Polygon_2> holes)
{
	cerr << "computing views for frontiers...";
	// set up
	vector<NextBestView> valid_frontier_nbvs;
	//valid_frontier_nbvs.clear();
	vector<int> task_index_in_valid_frontier_nbvs; 
	//task_index_in_valid_frontier_nbvs.clear();
	vector<int> frontier_task_indexes; // frontier task indexes
	int frt_num = 0; // number of frontier tasks
	preprocess_frontiers(boundary, holes); // preprocess to m_frontierList
	for (int i = 0; i < m_frontierList.size(); i++)
		frontier_task_indexes.push_back(i);
	frt_num = frontier_task_indexes.size();
	vector<NextBestView> frontier_task_nbvs;
	// NBV
	vector<NextBestView> nbvs_frt;  // 2018-11-05.
	//if (scene_name == "wallrect" && holes.empty()) // test in wall_rectangle scene
	//	nbvs_frt = generateViewsFrontiersNoneObstacle(true, 0.5);
	//else
	//	nbvs_frt = generateViewsFrontiers(true, 0.5); 
	nbvs_frt = generateViewsFrontiers(true, 0.5);
	valid_frontier_nbvs = nbvs_frt;
	// final tasks // intersect; whats meannig of these code???
	for (int i = 0; i < nbvs_frt.size(); i++)
	{
		bool valid = false;
		for (int j = 0; j < frontier_task_indexes.size(); j++)
		{
			if (nbvs_frt[i].index == frontier_task_indexes[j])
			{
				valid = true;
				break;
			}
		}
		if (valid)
		{
			frontier_task_nbvs.push_back(nbvs_frt[i]);
			task_index_in_valid_frontier_nbvs.push_back(i);
		}
	}
	frt_num = frontier_task_nbvs.size();
/*
	// test
	if (frontier_task_nbvs.size() == 0)
	{
		cout << m_frontierList.size() << " origin valid frontiers remain." << endl;
		cout << nbvs_frt.size() << " views for frontiers." << endl;
		cout << frontier_task_nbvs.size() << " views for frontiers remain." << endl;
	}
//*/
	cerr<<"done."<<endl;
	return frontier_task_nbvs;
}

// exe func: return nbvs, not finish. todo: NBV for quality.
vector<NextBestView> Navigation::extractQualityTaskViews(Polygon_2 boundary, vector<Polygon_2> holes)
{
	// set up
	m_valid_object_nbvs.clear();
	m_task_index_in_valid_object_nbvs.clear();
	vector<NextBestView> object_task_nbvs; // results return
/*
	// set up ???
	int obj_num = 0; // number of object tasks
	// extract initial task targets
	vector<Eigen::Vector3i> initial_targets = findQualityTargets();
	// check quit impossible positions // need to check if correct
	vector<Eigen::Vector3i> targets;
	for (int i = 0; i < initial_targets.size(); i++)
	{
		// target task in se2 coordinate
		Eigen::Vector2d targ(initial_targets[i][1], -initial_targets[i][0]);
		// check
		bool visible = true;
		for (auto unvisible_task = task_that_should_be_scan.begin(); unvisible_task != task_that_should_be_scan.end(); unvisible_task++)
		{
			// impoosible task in se2 coordinate
			Eigen::Vector2d hard(unvisible_task->x(), unvisible_task->y());
			if ((targ - hard).norm() <= offset_size)
			{
				visible = false;
				break;
			}
		}
		if (!visible)
			continue;
		targets.push_back(initial_targets[i]);
	}
	// compute NBV to cover targets
	vector<NextBestView> nbvs_obj = computeViewsForTargets(targets, boundary, holes);
	m_valid_object_nbvs = nbvs_obj;
	object_task_nbvs = nbvs_obj;
	obj_num = object_task_nbvs.size();
	// sort and filtering
	{
		// sort
		InfoGain ig;
		map<float, NextBestView> sorted_nbvs;
		for (int i = 0; i < object_task_nbvs.size(); i++)
		{
			float gain = ig.ComputeInfoGain_Target(object_task_nbvs[i]);
			map<float, NextBestView>::iterator finder = sorted_nbvs.find(gain);
			while (finder != sorted_nbvs.end())
			{
				//cerr << "repeat gain..." << endl;
				//getchar();
				// add noise.
				float noise = (float)rand() / RAND_MAX * 0.0001;
				gain += noise;
				finder = sorted_nbvs.find(gain);
			}
			sorted_nbvs.insert(pair<float, NextBestView>(gain, object_task_nbvs[i]));
		}
		//// test
		//{
		//	cerr << "info_gain_thresh = " << info_gain_thresh << endl;
		//	for (auto it = sorted_nbvs.begin(); it != sorted_nbvs.end(); it++)
		//	{
		//		cerr << "sorted gain = " << it->first << endl;
		//	}
		//	cerr << "continue?" << endl;
		//	getchar();
		//}
		// save
		{
			object_task_nbvs.clear();
			vector<NextBestView> temp;
			for (auto it = sorted_nbvs.begin(); it != sorted_nbvs.end(); it++)
			{
				if (it->first < info_gain_thresh) continue;
				//cerr << "pushed info gain " << it->first << endl;
				temp.push_back(it->second);
			}
			for (int idx = temp.size() - 1; idx >= 0; idx--)
			{
				//cerr << "gain: " << ig.ComputeInfoGain_Target(temp[idx]) << endl;
				object_task_nbvs.push_back(temp[idx]);
			}
		}
	}
//*/
	// end.
	return object_task_nbvs;
}

// extract tasks, not finish
bool Navigation::extractTasks(Polygon_2 boundary, vector<Polygon_2> holes, vector<ScanningTask> & tasks)
{
	// frontiers tasks
	vector<NextBestView> frontier_tasks = compute_frontier_tasks(boundary, holes);
	int frt_num = frontier_tasks.size();
	vector<double> frontier_task_distances(frontier_tasks.size());
	// priority of frontier tasks. less distance higher priority.
	{
		// load robot poses
		vector<Point_2> rbtPoses(m_robot_sites);
		// compute distance to robots
		for (int fid = 0; fid < frontier_tasks.size(); fid++)
		{
			// check nan
			if (__isnan(frontier_tasks[fid].pose.translation().x()) || __isnan(frontier_tasks[fid].pose.translation().y()))
			{
				cerr<<"frontier_tasks["<<fid<<"] nan: "<<frontier_tasks[fid].pose.translation().transpose()<<endl;
				getchar(); getchar(); getchar();
				continue;
			}
			// compute distance from closest robot
			double min_d = 999999999;
			{
				vector<Point_2> targets;
				for (int rid = 0; rid < rbtPoses.size(); rid++)
					targets.push_back(rbtPoses[rid]);
				Point_2 taskP(frontier_tasks[fid].pose.translation().x(), frontier_tasks[fid].pose.translation().y());
				vector<double> distances = m_metric.getGeodesicDistances(taskP, targets);
				for (int rid = 0; rid < distances.size(); rid++)
					if (min_d > distances[rid])
						min_d = distances[rid];
			}
			frontier_task_distances[fid] = min_d;
		}
	}
	//// confidence_threshold flexiable.
	//if (frontier_tasks.size() == 0) confidence_threshold = 1.5;
	// object tasks. modify
	vector<NextBestView> object_tasks = extractQualityTaskViews(boundary, holes);
	int ldmk_num = object_tasks.size();

	//// confidence_threshold flexiable.
	//if (object_tasks.size() == 0)
	//{
	//	confidence_threshold = 2.5;
	//	object_tasks = extractQualityTaskViews(boundary, holes);
	//	ldmk_num = object_tasks.size();
	//}

	// confidence_threshold fixed.
	object_tasks = extractQualityTaskViews(boundary, holes);
	ldmk_num = object_tasks.size();

	//// test: remove all tasks for quality, to only explre.
	//object_tasks.clear();
	//ldmk_num = 0;

	// tasks conclusion // ScanningTask data structure
	int task_num;
	{
		// task explore
		for (int i = 0; i < frontier_tasks.size(); i++) // exploration tasks
			tasks.push_back(ScanningTask(1, frontier_tasks[i]));
		// task number control 
		{
			if (tasks.size() > m_max_task_num)
			{
				int d_num = tasks.size() - m_max_task_num;
				for (int i = 0; i < d_num; i++)
				{
					// delete farthest
					{
						double maxValue = 0;
						int maxID = -1;
						for (int index = 0; index < frontier_task_distances.size(); index++)
						{
							if (frontier_task_distances[index] > maxValue)
							{
								maxValue = frontier_task_distances[index];
								maxID = index;
							}
						}
						if (maxID == -1)
						{
							cerr << "error, ID = -1" << endl; getchar(); getchar(); getchar();
						}
						tasks.erase(tasks.begin() + maxID);
						frontier_task_distances.erase(frontier_task_distances.begin() + maxID);
					}
				}
			}
		}
		//{ // test
		//	vector<Point_2> rbtPoses;
		//	loadRobotSites(rbtPoses);
		//	for (int index = 0; index < rbtPoses.size(); index++)
		//	{
		//		cerr << "plot(" << rbtPoses[index].x() << ", " << rbtPoses[index].y() << ", 'bx'); hold on;" << endl;
		//	}
		//	for (int index = 0; index < frontier_tasks.size(); index++)
		//	{
		//		cerr << "plot(" << frontier_tasks[index].pose.translation().x() << ", " << frontier_tasks[index].pose.translation().y() << ", 'go'); hold on;" << endl;
		//	}
		//	for (int index = 0; index < tasks.size(); index++)
		//	{
		//		cerr << "plot(" << tasks[index].view.pose.translation().x() << ", " << tasks[index].view.pose.translation().y() << ", 'ro'); hold on;" << endl;
		//	}
		//	getchar(); getchar(); getchar();
		//}
		// task quality
		if (tasks.size() < m_max_task_num)
		{
			for (int i = 0; i < object_tasks.size(); i++) // quality tasks
				tasks.push_back(ScanningTask(0, object_tasks[i]));
		}
		// task number control
		{
			if (tasks.size() > m_max_task_num)
			{
				int d_num = tasks.size() - m_max_task_num;
				for (int i = 0; i < d_num; i++)
					tasks.pop_back();
			}
		}
		task_num = tasks.size();
		cerr << frt_num << " views for exploration, " << ldmk_num << " views for quality" << endl;
		cerr << task_num << " task views selected. " << endl;
	}
//*/
	return true;
}

// not finished...
void Navigation::scanFinished()
{
	// todo...
	printf("scan finished.\n");
	exit(0);
	return;
}

// TSP.
vector<int> Navigation::TSP_path(vector<Point_2> points, vector<vector<double>> weights)
{
	// set up
	vector<int> result;
	// few points
	if (points.size() <= 3)
	{
		if (points.size() == 1)
		{
			result.push_back(0);
		}
		else if (points.size() == 2)
		{
			result.push_back(0);
			result.push_back(1);
		}
		else // 3
		{
			result.push_back(0);
			if (weights[0][1] < weights[0][2])
			{
				result.push_back(1);
				result.push_back(2);
			}
			else
			{
				result.push_back(2);
				result.push_back(1);
			}
		}
		// return
		return result;
	}
	// set up input for TSP solver
	vector<Eigen::Vector2d> nodes;
	for (int i = 0; i < points.size(); i++)
		nodes.push_back(Eigen::Vector2d(points[i].x(), points[i].y()));
	// TSP solver
	TSP tsp(nodes, weights);

	// Christofides Algorithm
	{
		// Find a MST T in graph G
		tsp.findMST_old();
		// Find a minimum weighted matching M for odd vertices in T
		tsp.perfect_matching();
		// Find the node that leads to the best path - - -
		// Create array of thread objects
		MyThread threads[NUM_THREADS];
		int best = INT_MAX;
		int bestIndex;
		int stop_here = NUM_THREADS;
		// Amount to increment starting node by each time
		int increment = 1; // by 1 if n < 1040
		int n = tsp.get_size();
		if (n >= 600 && n < 1040)
			increment = 3;
		else if (n >= 1040 && n < 1800)
			increment = 8;
		else if (n >= 1800 && n < 3205)
			increment = 25; 		// ~ 220s @ 3200
		else if (n >= 3205 && n < 4005)
			increment = 50; 		// ~ 230s @ 4000
		else if (n >= 4005 && n < 5005)
			increment = 120;		// ~ 200 @ 5000
		else if (n >= 5005 && n < 6500)
			increment = 250;		// ~ 220s @ 6447
		else if (n >= 6500)
			increment = 500;
		int remaining = n;
		// Start at node zero
		int node = 0;
		// Count to get thread ids
		int count = 0;
		while (remaining >= increment) 
		{
			// Keep track iteration when last node will be reached
			if (remaining < (NUM_THREADS * increment)) 
			{
				// each iteration advances NUM_THREADS * increment nodes
				stop_here = remaining / increment;
			}
			for (long t = 0; t < stop_here; t++) 
			{
				//cout << "Thread " << count << " starting at node " << node << endl;
				threads[t].start_node = node;
				threads[t].my_id = count;
				threads[t].mytsp = &tsp;
				threads[t].start();
				node += increment;
				count++;
			}
			// Wait for all the threads
			for (long t = 0; t < stop_here; t++) 
			{
				threads[t].join();
			}
			remaining -= (stop_here * increment);
		}
		// Loop through each index used and find shortest path
		for (long t = 0; t < count; t++) {
			if (tsp.path_vals[t][1] < best) {
				bestIndex = tsp.path_vals[t][0];
				best = tsp.path_vals[t][1];
			}
		}
		// Store best path
		tsp.create_tour(bestIndex);
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();
	}
	// path // remove the first/last path, depend on which it longer
	int start_id = -1;
	bool posetive_sequence = true;
	// find start id and sequence direction
	for (int i = 0; i < tsp.circuit.size(); i++)
	{
		if (tsp.circuit[i] == 0)
		{
			start_id = i;
			int ni = (i + 1) % tsp.circuit.size();
			int li = (i - 1 + tsp.circuit.size()) % tsp.circuit.size();
			//cerr << "weights[0][tsp.circuit[ni]] = " << weights[0][tsp.circuit[ni]] << endl;
			//cerr << "weights[0][tsp.circuit[li]] = " << weights[0][tsp.circuit[li]] << endl;
			if (weights[0][tsp.circuit[ni]] < weights[0][tsp.circuit[li]])
				posetive_sequence = true;
			else
				posetive_sequence = false;
			break;
		}
	}
	for (int counter = 0; counter < tsp.circuit.size(); counter++)
	{
		if (posetive_sequence)
		{
			int id = (start_id + counter) % tsp.circuit.size();
			result.push_back(tsp.circuit[id]);
		}
		else
		{
			int id = (start_id - counter + tsp.circuit.size()) % tsp.circuit.size();
			result.push_back(tsp.circuit[id]);
		}
	}
	return result;
}

// OMT with TSP, not finish
void Navigation::OMT_TSP()
{

// : Generate Domain. outer boundary and inner holes.
	Polygon_2 boundary; // domain outer boundary
	vector<Polygon_2> holes; // domain invalid holes
	vector<Polygon_2> origin_holes; // origin domain incalid holes
	{
		robotMoveDomainProcess(boundary, holes, origin_holes); // generate robot move domain
		//double t_beg = clock(); // timing
		// domain CDT
		CDT cdt;
		vector<ScanningTask> temp; // useless
		generateMoveDomainCDT(boundary, holes, origin_holes, temp, cdt);
		// save mesh for geodesic computation 
		saveCDT_geodesic(cdt);
		m_metric.get_geodesic_distance_fast_initialization(); // init geodesic
		//double t_end = clock(); // timing
		//cerr << "- CDT timing " << t_end - t_beg << " ms" << endl;
	}
	//double t1 = clock(); // timing

// : Generate Views. 
	vector<ScanningTask> tasks; // scan tasks
	vector<Point_2> frontiers; // frontier
	{
		// frontiers
		frontiers = load_and_check_frontiers(boundary, holes); // frontier positions, global variable
		// tasks
		extractTasks(boundary, holes, tasks);
		// check if scan finished
		if (tasks.size() == 0)
		{
			scanFinished();
		}
		// update domain cdt sites with task_sites
		//double t_beg = clock(); // timing
		CDT cdt;
		generateMoveDomainCDT(boundary, holes, origin_holes, tasks, cdt);
		// save mesh for geodesic computation
		saveCDT_geodesic(cdt); // save mesh to compute geodesic 
		m_metric.get_geodesic_distance_fast_initialization(); // init geodesic
		//double t_end = clock(); // timing
		//cerr << "- CDT timing " << t_end - t_beg << " ms" << endl;
	}

// : OMT Task Assignment 
	vector<vector<NextBestView>> assigned_tasks; // assignment: output from assignment method
	// discrete omt
	{
		// domain
		CDT cdt; // input
		generateMoveDomainCDT(boundary, holes, origin_holes, tasks, cdt);
		saveCDT_geodesic(cdt); // save mesh to compute geodesic 
		m_metric.get_geodesic_distance_fast_initialization(); // init geodesic
		// robot pose input
		vector<Eigen::Vector2d> robots(m_robot_sites.size()); // input
		for (int rid = 0; rid < m_robot_sites.size(); rid++)
		{
			robots[rid] = Eigen::Vector2d(m_robot_sites[rid].x(), m_robot_sites[rid].y());
			//cerr<<"robots["<<rid<<"] "<<robots[rid].transpose()<<endl;
		}
		// task view input
		vector<Eigen::Vector2d> task_views(tasks.size()); // input 
		for (int vid = 0; vid < tasks.size(); vid++)
			task_views[vid] = Eigen::Vector2d(
				tasks[vid].view.pose.translation().x(),
				tasks[vid].view.pose.translation().y()
			);
		// omt solver
		DiscreteSolver d_solver(*m_p_de, cdt, m_metric, robots, task_views, compactParam);
		//DiscreteSolver d_solver(*m_p_de, cdt, m_metric, m_metric.GeodesicDistances, robots, task_views, compactParam);
		d_solver.SolveOMT();
		vector<vector<int>> assignment = d_solver.getSolution();
		// save the assignment
		assigned_tasks.resize(assignment.size());
		for (int rid = 0; rid < assignment.size(); rid++)
		{
			assigned_tasks[rid].resize(assignment[rid].size());
			for (int tid = 0; tid < assignment[rid].size(); tid++)
			{
				int vid = assignment[rid][tid];
				assigned_tasks[rid][tid] = NextBestView(tasks[vid].view);
			}
		}
	}

// : TSP Order Planning. 
	{
		// setup
		m_robot_move_views.clear();
		m_robot_move_views.resize(rbt_num);
		for (int rid = 0; rid < rbt_num; rid++)
			m_robot_move_views[rid].clear();
		// solve TSP (scan order of tasks) for each robot
		for (int rid = 0; rid < rbt_num; rid++)
		{
			// nodes
			vector<Point_2> nodes; // index == assigned_tasks[rid].index+1
			nodes.push_back(Point_2(m_robot_sites[rid].x(), m_robot_sites[rid].y()));
			for (int tid = 0; tid < assigned_tasks[rid].size(); tid++)
				nodes.push_back(Point_2(assigned_tasks[rid][tid].pose.translation().x(), assigned_tasks[rid][tid].pose.translation().y()));
			// weights
			vector<vector<double>> weights; // weights
			weights.resize(nodes.size());
			for (int i = 0; i < nodes.size(); i++)
			{
				weights[i].resize(nodes.size());
			}
			// compute weights batch & paralell.
			{
				// weights between i j
				vector<vector<double>> wij(nodes.size());
				for (int i = 0; i < wij.size(); i++)
					wij[i].resize(nodes.size());
#pragma omp parallel for num_threads(8) 
				//for (int i = 0; i < nodes.size(); i++)
				for (int i = 0; i < nodes.size() - 1; i++)
				{
					vector<Point_2> targets;
					for (int j = i + 1; j < nodes.size(); j++)
						targets.push_back(nodes[j]);
					vector<double> cDistances = m_metric.getGeodesicDistances(nodes[i], targets); // compute geodesic
					for (int tid = 0; tid < targets.size(); tid++)
					{
						int j = tid + i + 1;
						wij[i][j] = cDistances[tid];
						wij[j][i] = cDistances[tid];
					}
				}
				// weights
				//for (int i = 0; i < nodes.size(); i++)
				for (int i = 0; i < nodes.size() - 1; i++)
				{
					for (int j = i + 1; j < nodes.size(); j++)
					{
						// distance weight
						double distance_weight = wij[i][j];
						// angle weight
						double angle_weight(0);
						{
							// theta of node i
							double theta1;
							if (i == 0) // i from 0 to size-1, so it may be the first node
							{
								theta1 = m_p_de->m_pose2d[rid].rotation().angle();
							}
							else
							{
								theta1 = assigned_tasks[rid][i - 1].pose.rotation().angle();
							}
							if (theta1 < 0) theta1 = 2 * PI + theta1;
							// theta of node j
							double theta2 = assigned_tasks[rid][j - 1].pose.rotation().angle();
							if (theta2 < 0) theta2 = 2 * PI + theta2;
							double delta = fabs(theta1 - theta2) < 2 * PI - fabs(theta1 - theta2) ? fabs(theta1 - theta2) : 2 * PI - fabs(theta1 - theta2);
							angle_weight = delta / PI;
							// check
							//if (_isnan(angle_weight))
							if (__isnan(angle_weight))
							{
								cerr << "angle_weight = " << angle_weight << " is nan" << endl;
								cerr << "theta1 = " << theta1 << endl;
								cerr << "theta2 = " << theta2 << endl;
								cerr << "delta = " << delta << endl;
								exit(-1);
							}
						}
						// final weight
						double final_weight = max(distance_weight, angle_weight);
						weights[i][j] = weights[j][i] = final_weight;
					}
				}
				// release memory
				for (int i = 0; i < wij.size(); i++)
					vector<double>().swap(wij[i]);
				wij.clear();
				vector<vector<double>>().swap(wij);
			}
			// path computation
			vector<int> point_path;
			if (nodes.size() == 1)
			{
				point_path.push_back(0); // only one noed
			}
			else
			{
				point_path = TSP_path(nodes, weights); // TSP solver
			}
			// store path
			for (int i = 0; i < point_path.size(); i++)
			{
				if (point_path[i] == 0)
				{
					// robot current pose
					m_robot_move_views[rid].push_back(m_p_de->m_pose2d[rid]);
				}
				else
				{
					// robot next best view poses
					int tid = point_path[i] - 1;
					m_robot_move_views[rid].push_back(iro::SE2(assigned_tasks[rid][tid].pose.translation().x(), assigned_tasks[rid][tid].pose.translation().y(), assigned_tasks[rid][tid].pose.rotation().angle()));
				}
			}
			// release memory
			for (int i = 0; i < weights.size(); i++)
				vector<double>().swap(weights[i]);
			weights.clear();
			vector<vector<double>>().swap(weights);
			// check
			if (m_robot_move_views[rid].size() != point_path.size())
			{
				cerr << "size error: " << m_robot_move_views[rid].size() << ", " << point_path.size() << endl;
				getchar();
			}
		}
	}
	// timing
/*
	// test
	{
		cv::Mat statement = m_p_de->visCellMap();
		for (int rid = 0; rid < rbt_num; ++rid)
		{
			for (int vid = 0; vid < m_robot_move_views[rid].size(); ++vid)
			{
				cv::circle(statement, cv::Point((int)m_robot_move_views[rid][vid].translation().x(), -(int)m_robot_move_views[rid][vid].translation().y()), 3, CV_RGB(0, 0, 255));
				if (vid>0)
				{
					cv::line(statement, 
						cv::Point((int)m_robot_move_views[rid][vid].translation().x(), -(int)m_robot_move_views[rid][vid].translation().y()),
						cv::Point((int)m_robot_move_views[rid][vid-1].translation().x(), -(int)m_robot_move_views[rid][vid-1].translation().y()),
						CV_RGB(0, 0, 255));
				}
			}
		}
		cv::imshow("statement", statement);
		cv::waitKey(0);
	}
//*/
	//done.
	return;
}

/*
// check trojectories
void Navigation::checkTrojectories()
{
	// check: remove invalid point
	for (int rid = 0; rid < rbt_num; rid++)
	{
		// detect invalid
		vector<int> idxs;
		for (int pid = 0; pid < g_rbtTrajectories[rid].size(); pid++)
		{
			if (__isnan(g_rbtTrajectories[rid][pid].x()))
			{
				cout << "g_rbtTrajectories node nan..." << endl; getchar(); getchar();
				idxs.push_back(pid);
				continue;
			}
			if (g_rbtTrajectories[rid][pid].x() <= 0 || g_rbtTrajectories[rid][pid].y() >= 0)
			{
				cout << "g_rbtTrajectories node out of boundary..." << endl; getchar(); getchar();
				idxs.push_back(pid);
				continue;
			}
		}
		// delete invalid
		for (int i = idxs.size() - 1; i >= 0; i--)
		{
			cout << "delete invalid trajectory node_" << idxs[i] << " " << (g_rbtTrajectories[rid].begin() + idxs[i])->x() << ", " << (g_rbtTrajectories[rid].begin() + idxs[i])->y() << endl;
			g_rbtTrajectories[rid].erase(g_rbtTrajectories[rid].begin() + idxs[i]);
		}
	}
	return;
}
//*/

// path_occlusion_check
bool Navigation::path_occlusion_check(cv::Mat background, cv::Point beg, cv::Point end)
{
	Eigen::Vector2d b(beg.x, beg.y);
	Eigen::Vector2d e(end.x, end.y);
	if ((b - e).norm() < 1)
	{
		return false;
	}
	// offset
	cv::Mat bin_map;
	cv::threshold(background, bin_map, 0, 255, cv::THRESH_BINARY);
	dilate(bin_map, bin_map, cv::getStructuringElement(0, cv::Size(offset_size * 2, offset_size * 2))); // offset of obsticles, used to aviod collision between robot and object 
	//cv::imshow("before reset", background);
	// reset background
	for (int r = 0; r < map_rows; r++)
	{
		for (int c = 0; c < map_cols; c++)
		{
			if (bin_map.ptr<uchar>(r)[c] == 0)
				background.ptr<uchar>(r)[c] = 0;
			else
				background.ptr<uchar>(r)[c] = 255;
		}
	}
	//cv::imshow("after reset", background);
	//cv::waitKey(0);
	// dda check
	bool occlusion = false;
	int dx = end.x - beg.x;
	int dy = end.y - beg.y;
	if (dx == 0){
		int beg_y = beg.y < end.y ? beg.y : end.y;
		int end_y = beg.y > end.y ? beg.y : end.y;
		for (int y = beg_y; y < end_y; y++)
			if (background.ptr<uchar>(y)[beg.x] == 255)
			{
				occlusion = true;
				break;
			}
	}
	else if (dy == 0){
		int beg_x = beg.x < end.x ? beg.x : end.x;
		int end_x = beg.x > end.x ? beg.x : end.x;
		for (int x = beg_x; x < end_x; x++)
			if (background.ptr<uchar>(beg.y)[x] == 255)
			{
				occlusion = true;
				break;
			}
	}
	else if (dx == 0 && dy == 0){}
	else {
		int MaxStep = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
		double fXUnitLen = 1.0;  
		double fYUnitLen = 1.0; 
		fYUnitLen = (double)(dy) / (double)(MaxStep);
		fXUnitLen = (double)(dx) / (double)(MaxStep);
		double x = (double)(beg.x);
		double y = (double)(beg.y);
		// 循环步进  
		for (long i = 1; i < MaxStep; i++) // '<' or '<=', not include end point
		{
			x = x + fXUnitLen;
			y = y + fYUnitLen;
			if (background.ptr<uchar>((int)round(y))[(int)round(x)] == 255)
				//if (background.ptr<uchar>((int)floor(y))[(int)floor(x)] == 255)
			{
				occlusion = true;
				break;
			}
		}
	}
	return occlusion;
}

// compute m_robot_move_nodes
bool Navigation::compute_robot_move_nodes(vector<double> & distances)
{
	// set up
	m_robot_move_nodes.clear();
	m_robot_move_nodes.resize(rbt_num);
	m_robot_move_nodes_nbv_sign.clear();
	m_robot_move_nodes_nbv_sign.resize(rbt_num);
	distances.clear();
	distances.resize(rbt_num);
	for (int rid = 0; rid < rbt_num; rid++)
		distances[rid] = 0;
/*// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //todo: smooth path
	// smooth interpolation
	if (exe_count > 0)
	{
		for (int rid = 0; rid < rbt_num; rid++)
		{
			if (m_robot_move_views[rid].size() == 1)
				continue;
			if ((m_p_de->m_pose2d[rid].translation() - m_robot_move_views[rid][1].translation()).norm() < 0.01) // same pose
			{
			}
			else // need smooth
			{
				vector<Point_2> goedesics;
				m_metric.get_geodesic_distance_fast(Point_2(m_p_de->m_pose2d[rid].translation().x(), m_p_de->m_pose2d[rid].translation().y()), Point_2(m_robot_move_views[rid][1].translation().x(), m_robot_move_views[rid][1].translation().y()), goedesics);
				if (goedesics.size() >= 2)
				{
					iro::SE2 nextViewPose(goedesics[1].x(), goedesics[1].y(), m_robot_move_views[rid][1].rotation().angle());
					// interpolation view position init
					Eigen::Vector2d itpViewPosition = (m_p_de->m_pose2d[rid].translation() + nextViewPose.translation()) / 2;
					double scale = (m_p_de->m_pose2d[rid].translation() - nextViewPose.translation()).norm() / 2 / 2; // scale
					//if (scale > 3) scale = 3;
					Eigen::Vector2d base;
					// - base angle: use path direction angle
					if (g_rbtTrajectories[rid].size() >= 2)
					{
						int idx = g_rbtTrajectories[rid].size() - 1;
						Eigen::Vector2d lastNode(g_rbtTrajectories[rid][idx].x(), g_rbtTrajectories[rid][idx].y());
						Eigen::Vector2d llstNode(g_rbtTrajectories[rid][idx - 1].x(), g_rbtTrajectories[rid][idx - 1].y());
						base = lastNode - llstNode;
						base.normalize();
						base *= scale; // scale
					}
					// - base angle: use current view angle
					else
					{
						base = Eigen::Vector2d(0, 1);
						base *= scale; // scale
						double angle = m_p_de->m_pose2d[rid].rotation().angle();
						Eigen::Matrix2d rotation;
						rotation << cos(angle), -sin(angle),
							sin(angle), cos(angle);
						base = rotation*base;
					}
					// interpolation view poisition final
					itpViewPosition += base;
					// interpolation view direction
					double itpViewDirection = nextViewPose.rotation().angle(); //double direction = (m_p_de->m_pose2d[rid].rotation().angle() + nextViewPose.rotation().angle()) / 2;
					// save
					{
						vector<iro::SE2> passViews;
						passViews.push_back(m_robot_move_views[rid][0]);
						if (!_isnan(itpViewPosition.x())) passViews.push_back(iro::SE2(itpViewPosition.x(), itpViewPosition.y(), itpViewDirection));
						for (int idx = 1; idx < m_robot_move_views[rid].size(); idx++)
							passViews.push_back(m_robot_move_views[rid][idx]);
						m_robot_move_views[rid].clear();
						m_robot_move_views[rid] = passViews;
					}
				}
			}
		}
	}
//*/// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// compute m_robot_move_nodes geodesic nodes
	for (int rid = 0; rid < rbt_num; rid++)
	{
		// only one node(nbv) in path
		if (m_robot_move_views[rid].size() == 1)
		{
			distances[rid] = 0.0;
			m_robot_move_nodes[rid].push_back(m_robot_move_views[rid][0]);
			m_robot_move_nodes_nbv_sign[rid].push_back(false); // this node is not nbv node
			continue;
		}
		// multi nodes(nbvs) in path
		for (int vid = 0; vid < m_robot_move_views[rid].size() - 1; vid++)
		{
			int nid = vid + 1;
			// check same position
			{
				Eigen::Vector2d beg(m_robot_move_views[rid][vid].translation().x(), m_robot_move_views[rid][vid].translation().y());
				Eigen::Vector2d end(m_robot_move_views[rid][nid].translation().x(), m_robot_move_views[rid][nid].translation().y());
				if ((beg - end).norm() < 0.001) // same position of two points
				{
					if (m_robot_move_nodes[rid].empty()) // first node
					{
						m_robot_move_nodes[rid].push_back(iro::SE2(m_robot_move_views[rid][vid].translation().x(), m_robot_move_views[rid][vid].translation().y(), m_robot_move_views[rid][vid].rotation().angle()));
						m_robot_move_nodes_nbv_sign[rid].push_back(true); // this node is nbv node
						m_robot_move_nodes[rid].push_back(iro::SE2(m_robot_move_views[rid][nid].translation().x(), m_robot_move_views[rid][nid].translation().y(), m_robot_move_views[rid][nid].rotation().angle()));
						m_robot_move_nodes_nbv_sign[rid].push_back(true); // this node is nbv node
					}
					else
					{
						if (!se2_equal(m_robot_move_nodes[rid].back(), m_robot_move_views[rid][vid])) // avoid repeated node
						{
							m_robot_move_nodes[rid].push_back(iro::SE2(m_robot_move_views[rid][vid].translation().x(), m_robot_move_views[rid][vid].translation().y(), m_robot_move_views[rid][vid].rotation().angle()));
							m_robot_move_nodes_nbv_sign[rid].push_back(true); // this node is nbv node
						}
						m_robot_move_nodes[rid].push_back(iro::SE2(m_robot_move_views[rid][nid].translation().x(), m_robot_move_views[rid][nid].translation().y(), m_robot_move_views[rid][nid].rotation().angle()));
						m_robot_move_nodes_nbv_sign[rid].push_back(true); // this node is nbv node
					}
					continue;
				}
			}
			// not same position
			// occlusion check
			cv::Point beg(round(m_robot_move_views[rid][vid].translation().x()), -round(m_robot_move_views[rid][vid].translation().y()));
			cv::Point end(round(m_robot_move_views[rid][nid].translation().x()), -round(m_robot_move_views[rid][nid].translation().y()));
			cv::Mat background(map_rows, map_cols, CV_8UC1); // background
			for (int r = 0; r < background.rows; r++)
			{
				for (int c = 0; c < background.cols; c++)
				{
					if (m_p_de->m_recon2D.m_cellmap[r][c].isScanned && m_p_de->m_recon2D.m_cellmap[r][c].isFree)
						background.ptr<uchar>(r)[c] = 0;
					else
						background.ptr<uchar>(r)[c] = 255;
				}
			}
			vector<Point_2> path; // path
			Point_2 p1(m_robot_move_views[rid][vid].translation().x(), m_robot_move_views[rid][vid].translation().y());
			Point_2 p2(m_robot_move_views[rid][nid].translation().x(), m_robot_move_views[rid][nid].translation().y());
			if (path_occlusion_check(background, beg, end)) // obsticle
			{
				distances[rid] += m_metric.get_geodesic_distance(p1, p2, path); // distance 
				//cerr << "text(" << p2.x() << ", " << p2.y() << ", '_g_" << distances[rid] << "'); hold on;" << endl;
			}
			else // not
			{
				distances[rid] += m_metric.get_euclidean_distance(p1, p2, path); // distance
				//cerr << "text(" << p2.x() << ", " << p2.y() << ", '_e_" << distances[rid] << "'); hold on;" << endl;
			}
			for (int tid = 0; tid < path.size(); tid++)
			{
				if (m_robot_move_nodes[rid].empty()) // first node
				{
					m_robot_move_nodes[rid].push_back(iro::SE2(path[tid].x(), path[tid].y(), m_robot_move_views[rid][vid].rotation().angle()));
					m_robot_move_nodes_nbv_sign[rid].push_back(false); // this node is not nbv node
				}
				else
				{
					if (!points_equal(Point_2(m_robot_move_nodes[rid].back().translation().x(), m_robot_move_nodes[rid].back().translation().y()), path[tid])) // avoid repeated node
					{
						if (tid == 0) // this node is nbv node
						{
							m_robot_move_nodes[rid].push_back(iro::SE2(path[tid].x(), path[tid].y(), m_robot_move_views[rid][vid].rotation().angle()));
							m_robot_move_nodes_nbv_sign[rid].push_back(true);
						}
						else if (tid == path.size() - 1) // this node is nbv node
						{
							m_robot_move_nodes[rid].push_back(iro::SE2(path[tid].x(), path[tid].y(), m_robot_move_views[rid][nid].rotation().angle()));
							m_robot_move_nodes_nbv_sign[rid].push_back(true);
						}
						else // this node is not nbv node
						{
							//m_robot_move_nodes[rid].push_back(iro::SE2(path[tid].x(), path[tid].y(), m_robot_move_views[rid][nid].rotation().angle()));
							m_robot_move_nodes[rid].push_back(iro::SE2(path[tid].x(), path[tid].y(), empty_angle));
							m_robot_move_nodes_nbv_sign[rid].push_back(false);
						}
					}
				}
			}
		}
		// error check
		if (!se2_equal(m_robot_move_nodes[rid].back(), m_robot_move_views[rid].back()))
		{
			// error
			cerr << "error, end point not equal" << endl;
			//getchar();
			m_robot_move_nodes[rid].push_back(m_robot_move_views[rid].back());
			m_robot_move_nodes_nbv_sign[rid].push_back(true);
		}
	}
	return true;
}

// check Point_2 equal
bool Navigation::points_equal(Point_2 p1, Point_2 p2)
{
	Eigen::Vector2d pose1(p1.x(), p1.y());
	Eigen::Vector2d pose2(p2.x(), p2.y());
	if ((pose1 - pose2).norm() < 0.00001)
		return true;
	return false;
}

// check se2 equal
bool Navigation::se2_equal(iro::SE2 p1, iro::SE2 p2)
{
	Eigen::Vector3d pose1(p1.translation().x(), p1.translation().y(), p1.rotation().angle());
	Eigen::Vector3d pose2(p2.translation().x(), p2.translation().y(), p2.rotation().angle());
	if ((pose1 - pose2).norm() < 0.00001)
		return true;
	return false;
}

// uniform sample nodes from nodes inside length
vector<iro::SE2> Navigation::uniformSampleWithNBVInfo(const int robot_id, const double step, const double length)
{
	// no move path 
	if (m_robot_move_nodes[robot_id].size() == 1)
	{
		vector<iro::SE2> samples;
		samples.push_back(m_robot_move_nodes[robot_id].front()); // push the first node, that is robot current pose 
		return samples;
	}
	// set up
	vector<Eigen::Vector2d> curve;
	for (int i = 0; i < m_robot_move_nodes[robot_id].size(); i++)
		curve.push_back(Eigen::Vector2d(m_robot_move_nodes[robot_id][i].translation().x(), m_robot_move_nodes[robot_id][i].translation().y()));
	// curve length
	vector<double> accLen;
	accLen.push_back(.0f);
	for (int i = 1; i < curve.size(); i++)
	{
		auto l = (curve[i - 1] - curve[i]).norm();
		accLen.push_back(accLen.back() + l);
		if (accLen.back() > length) // length
			break;
	}
	int numSamples = (int)floor(accLen.back() / step) + 1;
	bool debug = false;
	// same position of all points 
	if (accLen.back() < 0.001)
	{
		vector<iro::SE2> samples;
		for (int i = 0; i < m_robot_move_nodes[robot_id].size(); i++)
		{
			samples.push_back(m_robot_move_nodes[robot_id][i]); // push the first node, that is robot current pose
		}
		return samples; // force return!

		numSamples = m_robot_move_nodes[robot_id].size();
		debug = true;
	}
	// sample
	vector<iro::SE2> samples;
	samples.push_back(m_robot_move_nodes[robot_id].front()); // push the first node
	double t = 0;
	int left = 0, right = 1;
	for (int i = 1; i < numSamples; i++)
	{
		// time
		t = i * step;
		// move to the segment containing t
		// t \in [accLen[left], accLen[right])
		while (t >= accLen[right])
		{
			//if (sample_save_origin_node)
			{
				if (m_robot_move_nodes_nbv_sign[robot_id][right])
				{
					samples.push_back(m_robot_move_nodes[robot_id][right]); // push origin NBV node 
				}
			}
			left++;
			right++;
		}
		// sample in the range of [left, right]
		auto a = t - accLen[left];
		auto b = accLen[right] - t;
		auto s = (a * curve[right] + b * curve[left]) / (a + b);
		//samples.push_back(iro::SE2(s.x(), s.y(), (m_robot_move_nodes[robot_id][left].rotation().angle() + m_robot_move_nodes[robot_id][right].rotation().angle()) / 2)); // push sampled node
		//samples.push_back(iro::SE2(s.x(), s.y(), compute_optimal_scan_direction(cv::Point(approx(s.x()), -approx(s.y()))))); // push sampled node with max_gain 
		samples.push_back(iro::SE2(s.x(), s.y(), empty_angle)); /* push sampled node, angle = 0 */ // optimization after path optimization
	}
	// test 
	{
		for (int i = 0; i < samples.size(); i++)
		{
			if ((int)round(samples[i].translation().x()) < 10 && -(int)round(samples[i].translation().y()) < 10)
			{

				cerr << "error uniform sampled a invalid se2" << endl;

				getchar();
				// reset
				samples.clear();
				for (int i = 0; i < m_robot_move_nodes[robot_id].size(); i++)
				{
					samples.push_back(m_robot_move_nodes[robot_id][i]); // push the first node, that is robot current pose 
				}
				return samples; // force return! 
			}
		}
	}
	// force debug // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	if (samples.size() == 1 && m_robot_move_nodes[robot_id].size() > 1)
	{
		samples.clear();
		for (int i = 0; i < m_robot_move_nodes[robot_id].size(); i++)
		{
			samples.push_back(m_robot_move_nodes[robot_id][i]); // push the first node, that is robot current pose
		}
		return samples; // force return! 
	}
	return samples;
}

// compute m_sync_move_paths, not finish
bool Navigation::compute_sync_move_paths(vector<double> distances)
{
	// set up 
	m_sync_move_paths.clear();
	m_sync_move_paths.resize(rbt_num);
	// compute m_sync_move_paths 
	double distance_step = g_distance_step/map_cellsize;
	double min_distance = DBL_MAX;
	for (int rid = 0; rid < rbt_num; rid++)
	{
		if (distances[rid] != 0 && min_distance > distances[rid])
			min_distance = distances[rid];
	}
	for (int rid = 0; rid < rbt_num; rid++)
	{
		m_sync_move_paths[rid] = uniformSampleWithNBVInfo(rid, distance_step, min_distance);
		// path optimization
		{
			// optimize local: cut to segments and optimize each segment independent 
			int start_id = -1, end_id = -1;
			for (int nid = 0; nid < m_sync_move_paths[rid].size(); nid++)
			{
				if (fabs(m_sync_move_paths[rid][nid].rotation().angle() - empty_angle) > 0.01) // nbv node
				{
					if (start_id == -1)
					{
						start_id = nid;
					}
					else
					{
						end_id = nid;
						if (end_id - start_id > 1) // apply optimization
						{
							// path segment set up 
							vector<iro::SE2> segment;
							for (int tid = start_id; tid <= end_id; tid++)
								segment.push_back(m_sync_move_paths[rid][tid]);
							// optimize by energy function 
							bool rtn = Optimize_Path(m_p_de, segment);
cerr << "Optimize_Path success " << rtn << endl;
							// save 
							for (int sid = 0, tid = start_id + 1; tid < end_id; sid++, tid++)
							{
								m_sync_move_paths[rid][tid].translation().x() = segment[sid].translation().x();
								m_sync_move_paths[rid][tid].translation().y() = segment[sid].translation().y();
							}
						}
						start_id = end_id;
						end_id = -1;
					}
				}
			}// optimization end
		}
		// delta_angle
		double delta_angle = g_angleDifference;
		// angle optimization quick // to achieve cover uncertainty along path
		{
			// interpolate temp_path
			vector<iro::SE2> temp_path;
			// node 0 ~ last-1
			for (int nid = 0; nid < m_sync_move_paths[rid].size() - 1; nid++)
			{
				// this node not nbv
				if (fabs(m_sync_move_paths[rid][nid].rotation().angle() - empty_angle) < 0.01)
				{
					// position
					Eigen::Vector2d beg(m_sync_move_paths[rid][nid].translation().x(), m_sync_move_paths[rid][nid].translation().y());
					Eigen::Vector2d end(m_sync_move_paths[rid][nid + 1].translation().x(), m_sync_move_paths[rid][nid + 1].translation().y());
					Eigen::Vector2d move_vector(end - beg);
					// angle
					Eigen::Vector2d base(0, 1);
					Eigen::Vector2d dire(m_sync_move_paths[rid][nid + 1].translation().x() - m_sync_move_paths[rid][nid].translation().x(), m_sync_move_paths[rid][nid + 1].translation().y() - m_sync_move_paths[rid][nid].translation().y());
					// move direction
					double d_angle = acos(base.dot(dire) / (dire.norm() * base.norm()));
					if (dire.x() > 0)
						d_angle = -d_angle;
					// 2 branches
					{
						if (nid != 0)
						{
							// last angle
							double lst_ang = m_sync_move_paths[rid][nid - 1].rotation().angle();
							{
								if (fabs(lst_ang - empty_angle) < 0.01)
								{
									lst_ang = temp_path.back().rotation().angle();
								}
								if (lst_ang < 0)
								{
									lst_ang += 2 * PI;
								}
							}
							// current angle
							double crt_ang = d_angle;
							{
								if (crt_ang < 0)
								{
									crt_ang += 2 * PI;
								}
							}
							if (crt_ang - lst_ang < 0)
							{
								// left
								{
									double l_angle = d_angle + delta_angle;
									if (l_angle > PI)
										l_angle -= 2 * PI;
									Eigen::Vector2d p(beg);
									temp_path.push_back(iro::SE2(p.x(), p.y(), l_angle));
								}
								// mid 
								{
									Eigen::Vector2d p(beg + 0.33*move_vector);
									temp_path.push_back(iro::SE2(p.x(), p.y(), d_angle));
								}
								// right
								{
									double r_angle = d_angle - delta_angle;
									if (r_angle < -PI)
										r_angle += 2 * PI;
									Eigen::Vector2d p(beg + 0.67*move_vector);
									temp_path.push_back(iro::SE2(p.x(), p.y(), r_angle));
								}
							}
							else
							{
								// right
								{
									double r_angle = d_angle - delta_angle;
									if (r_angle < -PI)
										r_angle += 2 * PI;
									Eigen::Vector2d p(beg);
									temp_path.push_back(iro::SE2(p.x(), p.y(), r_angle));
								}
								// mid
								{
									Eigen::Vector2d p(beg + 0.33*move_vector);
									temp_path.push_back(iro::SE2(p.x(), p.y(), d_angle));
								}
								// left
								{
									double l_angle = d_angle + delta_angle;
									if (l_angle > PI)
										l_angle -= 2 * PI;
									Eigen::Vector2d p(beg + 0.67*move_vector);
									temp_path.push_back(iro::SE2(p.x(), p.y(), l_angle));
								}
							}
						}
						else
						{
							// left
							{
								double l_angle = d_angle + delta_angle;
								if (l_angle > PI)
									l_angle -= 2 * PI;
								Eigen::Vector2d p(beg);
								temp_path.push_back(iro::SE2(p.x(), p.y(), l_angle));
							}
							// mid
							{
								Eigen::Vector2d p(beg + 0.33*move_vector);
								temp_path.push_back(iro::SE2(p.x(), p.y(), d_angle));
							}
							// right
							{
								double r_angle = d_angle - delta_angle;
								if (r_angle < -PI)
									r_angle += 2 * PI;
								Eigen::Vector2d p(beg + 0.67*move_vector);
								temp_path.push_back(iro::SE2(p.x(), p.y(), r_angle));
							}
						}
					}
				} // end of interpolate
				// this node not nbv				
				else
				{// interpolate for nbv 
					// interpolate method 2 
					{
						// push nbv
						temp_path.push_back(m_sync_move_paths[rid][nid]);
					}
				}
			}
			// node last
			if (fabs(m_sync_move_paths[rid].back().rotation().angle() - empty_angle) < 0.01)
				m_sync_move_paths[rid].back().rotation().angle() = m_robot_move_views[rid].back().rotation().angle();
			temp_path.push_back(m_sync_move_paths[rid].back());
			// save temp_path 2 syn_move_path 
			m_sync_move_paths[rid].clear();
			for (int nid = 0; nid < temp_path.size(); nid++)
				m_sync_move_paths[rid].push_back(temp_path[nid]);
		}
//*/
	}
	return true;
}

// camera trajectory interpolation: delta distance/angle constraints.
void Navigation::trajectoryInterpolation(vector<iro::SE2> & path, double dAngle)
{
	vector<iro::SE2> result;
	for (int i = 0; i < path.size() - 1; i++)
	{
		int ii = i + 1;
		double nTheta = path[ii].rotation().angle();
		if (nTheta < 0) nTheta += 2 * PI;
		double cTheta = path[i].rotation().angle();
		if (cTheta < 0) cTheta += 2 * PI;
		double difference = nTheta - cTheta;
		if (difference > PI) difference = difference - 2 * PI;
		if (difference < -PI) difference = difference + 2 * PI;
		// push node i
		result.push_back(path[i]);
		// if > aAngle, push interpolated node
		if (fabs(difference) > dAngle)
		{
			int num = ceil(fabs(difference) / dAngle) + 1; // #segment
			Eigen::Vector2d crt(path[i].translation());
			Eigen::Vector2d nxt(path[ii].translation());
			Eigen::Vector2d dlt = (nxt - crt) / num;
			cerr << "two end point" << endl << path[i].translation().transpose() << " " << path[i].rotation().angle() << endl << path[ii].translation().transpose() << " " << path[ii].rotation().angle() << endl;
			// interpolation
			for (int id = 1; id < num; id++)
			{
				// position
				Eigen::Vector2d itp_position = crt + dlt*id;
				// orientation
				double itp_theta = path[i].rotation().angle() + difference / num * id;
				if (itp_theta > PI) itp_theta = itp_theta - 2 * PI;
				if (itp_theta < -PI) itp_theta = itp_theta + 2 * PI;
				result.push_back(iro::SE2(itp_position.x(), itp_position.y(), itp_theta));
				cerr << "interpolate " << itp_position.transpose() << " " << itp_theta << endl;
			}
		}
	}
	// push the last node
	result.push_back(path.back());
	// save
	path.clear();
	for (int i = 0; i < result.size(); i++)
		path.push_back(result[i]);
	return;
}

// traj opt, not finish
void Navigation::Trajectory_Optimization()
{ 
// input: m_robot_move_views
// output: m_sync_move_paths
	// setup
/*
	cover_map.clear();
	cover_map.resize(row);
	for (int r = 0; r < cover_map.size(); r++)
	{
		cover_map[r].resize(col);
		for (int c = 0; c < cover_map[r].size(); c++)
		{
			cover_map[r][c] = 0;
		}
	}
//*/
	//checkTrojectories(); 
	// optimize move paths
	vector<double> distances;
	// get all geodesic nodes
	cerr << "compute geodesic nodes...";
	compute_robot_move_nodes(distances);
	cerr << "done." << endl;
/*
	// test vis. ok.
	{
		cv::Mat statement = m_p_de->visCellMap();
		for (int rid = 0; rid < rbt_num; ++rid)
		{
			for (int vid = 0; vid < m_robot_move_nodes[rid].size(); ++vid)
			{
				cv::circle(statement, cv::Point((int)m_robot_move_nodes[rid][vid].translation().x(), -(int)m_robot_move_nodes[rid][vid].translation().y()), 3, CV_RGB(0, 0, 255));
				if (vid>0)
				{
					cv::line(statement, 
						cv::Point((int)m_robot_move_nodes[rid][vid].translation().x(), -(int)m_robot_move_nodes[rid][vid].translation().y()),
						cv::Point((int)m_robot_move_nodes[rid][vid-1].translation().x(), -(int)m_robot_move_nodes[rid][vid-1].translation().y()),
						CV_RGB(0, 0, 255));
				}
			}
		}
		cv::imshow("plan move nodes", statement);
		cv::waitKey(0);
	}
//*/
	// uniform sample and optimize
	cerr << "path optimization...";
	compute_sync_move_paths(distances);
	cerr << "done." << endl;
/*
	// test vis. ok.
	{
		cv::Mat statement = m_p_de->visCellMap();
		for (int rid = 0; rid < rbt_num; ++rid)
		{
			for (int vid = 0; vid < m_sync_move_paths[rid].size(); ++vid)
			{
				cv::circle(statement, cv::Point((int)m_sync_move_paths[rid][vid].translation().x(), -(int)m_sync_move_paths[rid][vid].translation().y()), 3, CV_RGB(0, 0, 255));
				if (vid>0)
				{
					cv::line(statement, 
						cv::Point((int)m_sync_move_paths[rid][vid].translation().x(), -(int)m_sync_move_paths[rid][vid].translation().y()),
						cv::Point((int)m_sync_move_paths[rid][vid-1].translation().x(), -(int)m_sync_move_paths[rid][vid-1].translation().y()),
						CV_RGB(0, 0, 255));
				}
			}
		}
		cv::imshow("plan sync move paths", statement);
		cv::waitKey(0);
	}
//*/
	// trajectory interpolation
	if (g_fpInterpolation)
	{
		// first node orientation
		for (int rid = 0; rid < m_sync_move_paths.size(); rid++)
		{
			m_sync_move_paths[rid][0] = m_p_de->m_pose2d[rid];
		}
		// interpolation
		for (int rid = 0; rid < m_sync_move_paths.size(); rid++)
		{
			trajectoryInterpolation(m_sync_move_paths[rid]);
		}
	}
	else
	{
		// first node orientation
		for (int rid = 0; rid < m_sync_move_paths.size(); rid++)
		{
			if (m_sync_move_paths[rid].size() > 1)
			{
				Eigen::Vector2d xy(m_sync_move_paths[rid][0].translation().x(), m_sync_move_paths[rid][0].translation().y());
				Eigen::Vector2d nxy(m_sync_move_paths[rid][1].translation().x(), m_sync_move_paths[rid][1].translation().y());
				// compute direction
				Eigen::Vector2d direction(nxy - xy);
				// same position of two nodes
				if (direction.norm() < 0.1) continue;
				// compute orientation
				direction.normalize();
				// convert format
				Eigen::Vector2d base(0, 1);
				double theta = acos(direction.dot(base));
				// convert format
				if (direction.x() > 0)
					theta = -theta;
				double angle = theta;
				// save
				m_sync_move_paths[rid][0].rotation().angle() = angle;
			}
		}
	}
	// vis.
	{
		cv::Mat statement = m_p_de->visCellMap();
		// draw sync path
		for (int rid = 0; rid < rbt_num; ++rid)
		{
			for (int vid = 0; vid < m_sync_move_paths[rid].size(); ++vid)
			{
				if (vid>0)
				{
					cv::line(statement, 
						cv::Point((int)m_sync_move_paths[rid][vid].translation().x(), -(int)m_sync_move_paths[rid][vid].translation().y()),
						cv::Point((int)m_sync_move_paths[rid][vid-1].translation().x(), -(int)m_sync_move_paths[rid][vid-1].translation().y()),
						CV_RGB(255, 0, 255));
				}
				cv::circle(statement, cv::Point((int)m_sync_move_paths[rid][vid].translation().x(), -(int)m_sync_move_paths[rid][vid].translation().y()), 2, CV_RGB(0, 0, 255));
			}
		}
		
		// test show frontiers
		{
			// draw frontiers in list
			cv::Mat temp = statement.clone();
			for (int fid = 0; fid < m_frontierList.size(); fid++)
			{
				int x = m_frontierList[fid].position.x();
				int y = -m_frontierList[fid].position.y();
				//cv::circle(temp, cv::Point(x, y), 0, CV_RGB(255, 255, 255));
				//cv::circle(temp, cv::Point(x, y), 1, CV_RGB(255, 255, 255));
				cv::circle(temp, cv::Point(x, y), 2, CV_RGB(255, 255, 255));
			}
			// save file
			char output_path[200];
			sprintf(output_path, "result/frontierList_%d.png", g_plan_iteration);
			cv::imwrite(output_path, temp);
		}

		// save file
		char output_path[200];
		sprintf(output_path, "result/plan_%d.png", g_plan_iteration);
		cv::imwrite(output_path, statement);
		// show 
		cv::imshow("plan sync move paths", statement);
		cv::waitKey(1);
	}
//*/
	return;
}

// ask robot to move and scan, not finish
void Navigation::moveRobotsAndScan()
{
	// set up
	//next_iter_sign = false;
	//explore_counter = 0;
	//char mode = 's';
	//// ask robot move and scan
	//if (mode == 's') // synchronize robots' movement
	{
		// set up
		int min_size = INT_MAX; // max size
		bool exist_robot_no_task = false;
		vector<bool> robot_no_task_signs(rbt_num);
		for (int rid = 0; rid < rbt_num; rid++)
			robot_no_task_signs.push_back(false);
		// min size of path
		for (int rid = 0; rid < rbt_num; rid++)
		{
			if (m_sync_move_paths[rid].size() == 1)
			{
				exist_robot_no_task = true;
				robot_no_task_signs[rid] = true;
				continue;
			}
			if (min_size > m_sync_move_paths[rid].size())
				min_size = m_sync_move_paths[rid].size();
		}
		if (min_size == INT_MAX)
		{
			min_size = 1;
		}
		// move and scan // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// some robots have been assigned no task
		if (exist_robot_no_task)
		{
			// move and scan
			for (int vid = 0; vid < min_size; vid++)
			{
/*
				// check if need new planning 
				if (method_num != 4)
				{
					if (rbt_num == 1)
					{
						//if (next_iter_sign)
						//{
						//	cerr << " - - - require next motion plan iteration - - - " << endl;
						//	next_iter_sign = false;
						//	explore_counter = 0;
						//	break;
						//}
					}
					else
					{
						////if (frontierList.empty() && next_iter_sign)
						//if (next_iter_sign)
						//{
						//	cerr << " - - - require next motion plan iteration - - - " << endl;
						//	next_iter_sign = false;
						//	explore_counter = 0;
						//	break;
						//}
					}
				}
//*/
				// move 
				vector<vector<double>> pose7s; // get views
				for (int rid = 0; rid < rbt_num; rid++)
				{
					// coordinate transform 
					vector<double> pose7;
					if (robot_no_task_signs[rid])
						pose7 = m_p_de->coord_trans_se22gazebo(m_sync_move_paths[rid][0]); // use the first node(only one node in path) pose
					else
						pose7 = m_p_de->coord_trans_se22gazebo(m_sync_move_paths[rid][vid]); // use current node pose
					pose7s.push_back(pose7);
					if (robot_no_task_signs[rid])
						continue;
					// total distance 
					if (vid != 0)
					{
						int lid = vid - 1;
						// todo: total distance
						//total_distance += get_euclidean_distance(Point_2(m_sync_move_paths[rid][vid].translation().x(), m_sync_move_paths[rid][vid].translation().y()), Point_2(m_sync_move_paths[rid][lid].translation().x(), m_sync_move_paths[rid][lid].translation().y()));
					}
				}
				// drive robots to move and scan
				cerr<<"socket move to views..."<<endl;
				m_p_de->socket_move_to_views(pose7s);
				cerr<<"done."<<endl;
				// save trajectories
				for (int rid = 0; rid < rbt_num; rid++)
				{
					if (robot_no_task_signs[rid])
					{
						if (g_rbtTrajectories[rid].empty())
						{
							g_rbtTrajectories[rid].push_back(Point_2(m_sync_move_paths[rid][0].translation().x(), m_sync_move_paths[rid][0].translation().y()));
							//cout << "- record syn_move_path " << m_sync_move_paths[rid][0].translation().x() << ", " << m_sync_move_paths[rid][0].translation().y() << endl;
						}
						g_camTrajectories[rid].push_back(m_sync_move_paths[rid][0]); // for camera trajecoty. 2018-12-30.
					}
					else
					{
						g_rbtTrajectories[rid].push_back(Point_2(m_sync_move_paths[rid][vid].translation().x(), m_sync_move_paths[rid][vid].translation().y()));
						//cout << "- record syn_move_path " << m_sync_move_paths[rid][vid].translation().x() << ", " << m_sync_move_paths[rid][vid].translation().y() << endl;
						g_camTrajectories[rid].push_back(m_sync_move_paths[rid][vid]); // for camera trajecoty. 2018-12-30.
					}
				}
				// scan and get data 
				m_p_de->getPoseFromServer();
				m_p_de->getRGBDFromServer();
				// scan data fusion 
				// insert scans to tree
				//cerr << "fuseScans2MapAndTree() begin" << endl;
				m_p_de->fuseScans2MapAndTree();
				//cerr << "fuseScans2MapAndTree() done." << endl;
				// project octotree to a 2d map
				//cerr << "projectOctree2Map() begin" << endl;
				m_p_de->projectOctree2Map();
				//cerr << "projectOctree2Map() done." << endl;
				// vis 
				vector<iro::SE2> current_views;
				for (int rid = 0; rid < rbt_num; rid++)
				{
					if (m_sync_move_paths[rid].size() <= vid)
						current_views.push_back(m_sync_move_paths[rid].back());
					else
						current_views.push_back(m_sync_move_paths[rid][vid]);
				}

// todo:
				visualizeScan(current_views, vid);
/*
// todo:				
				move_counter++;
//*/
			}
		}
		// each robot have been assigned tasks 
		else
		{
			// move and scan 
			for (int vid = 0; vid < min_size; vid++)
			{
/*
				// check if need new plan
				if (method_num != 4)
				{
					if (rbt_num == 1)
					{
						//if (next_iter_sign)
						//{
						//	cerr << " - - - require next motion plan iteration - - - " << endl;
						//	next_iter_sign = false;
						//	explore_counter = 0;
						//	break;
						//}
					}
					else
					{
						////if (frontierList.empty() && next_iter_sign)
						//if (next_iter_sign)
						//{
						//	cerr << " - - - require next motion plan iteration - - - " << endl;
						//	next_iter_sign = false;
						//	explore_counter = 0;
						//	break;
						//}
					}
				}
//*/
				// move 
				vector<vector<double>> pose7s; // get views
				for (int rid = 0; rid < rbt_num; rid++)
				{
					vector<double> pose7 = m_p_de->coord_trans_se22gazebo(m_sync_move_paths[rid][vid]); // coordinate transform
					pose7s.push_back(pose7);
					// total distance 
					if (vid != 0)
					{
						int lid = vid - 1;
						// todo: total distance
						//total_distance += get_euclidean_distance(Point_2(m_sync_move_paths[rid][vid].translation().x(), m_sync_move_paths[rid][vid].translation().y()), Point_2(m_sync_move_paths[rid][lid].translation().x(), m_sync_move_paths[rid][lid].translation().y()));
					}
				}
				// drive robots to move and scan
				cerr<<"socket move to views..."<<endl;
				m_p_de->socket_move_to_views(pose7s);
				cerr<<"done."<<endl;
				// save trajectories 
				for (int rid = 0; rid < rbt_num; rid++)
				{
					g_rbtTrajectories[rid].push_back(Point_2(m_sync_move_paths[rid][vid].translation().x(), m_sync_move_paths[rid][vid].translation().y()));
					g_camTrajectories[rid].push_back(m_sync_move_paths[rid][vid]); // for camera trajecoty. 2018-12-30.
				}
				// scan and get data 
				m_p_de->getPoseFromServer();
				m_p_de->getRGBDFromServer();
				// scan data fusion 
				// insert scans to tree
				//cerr << "fuseScans2MapAndTree() begin" << endl;
				m_p_de->fuseScans2MapAndTree();
				//cerr << "fuseScans2MapAndTree() done." << endl;
				// project octotree to a 2d map
				//cerr << "projectOctree2Map() begin" << endl;
				m_p_de->projectOctree2Map();
				//cerr << "projectOctree2Map() done." << endl;
				// vis
				vector<iro::SE2> current_views; // use to vis
				for (int rid = 0; rid < rbt_num; rid++)
				{
					if (m_sync_move_paths[rid].size() <= vid)
						current_views.push_back(m_sync_move_paths[rid].back());
					else
						current_views.push_back(m_sync_move_paths[rid][vid]);
				}

// todo:				
				visualizeScan(current_views, vid);
/*
// todo:
				move_counter++;
//*/
			}
		}
	}
//*/
	return;
}

// draw views
bool draw_robot_views(cv::Mat & input, vector<iro::SE2> views)
{
	const double frustumSize = 20; // show in 20 voxels lenth.
	// draw frustums
	for (int vid = 0; vid < views.size(); vid++)
	{
		// position
		cv::Point p((int)round(views[vid].translation().x()), -(int)round(views[vid].translation().y()));
		cv::circle(input, p, 0, CV_RGB(0, 0, 255), 2, CV_AA);
		cv::circle(input, p, 1, CV_RGB(0, 0, 255), 2, CV_AA);
		cv::circle(input, p, 2, CV_RGB(0, 0, 255), 2, CV_AA);
		cv::circle(input, p, 3, CV_RGB(0, 0, 255), 2, CV_AA);
		cv::circle(input, p, 4, CV_RGB(0, 0, 255), 2, CV_AA);
		cv::circle(input, p, 5, CV_RGB(0, 0, 255), 2, CV_AA);
		// orentation
		Eigen::Vector2d d(0, frustumSize);
		Eigen::Matrix2d m;
		double theta = views[vid].rotation().angle();
		m << cos(theta), -sin(theta),
			sin(theta), cos(theta);
		d = m*d;
		// view frustum l
		m << cos(PI / 6), -sin(PI / 6),
			sin(PI / 6), cos(PI / 6);
		d = m*d;
		cv::Point l((int)round(views[vid].translation().x() + d.x()), -(int)round(views[vid].translation().y() + d.y()));
		cv::line(input, p, l, CV_RGB(0, 0, 255), 1, CV_AA);
		// view frustum r
		m << cos(-PI / 3), -sin(-PI / 3),
			sin(-PI / 3), cos(-PI / 3);
		d = m*d;
		cv::Point r((int)round(views[vid].translation().x() + d.x()), -(int)round(views[vid].translation().y() + d.y()));
		cv::line(input, p, r, CV_RGB(0, 0, 255), 1, CV_AA);
		// view frustum lr
		cv::line(input, l, r, CV_RGB(0, 0, 255), 1, CV_AA);
	}
	return true;
}

// visualization, not finish
bool Navigation::visualizeScan(vector<iro::SE2> current_views, int vid) // todo: remove vid.
{
	// recon
	cv::Mat statement = m_p_de->visCellMap();
	// robot position
	for (int i = 0; i < current_views.size(); ++i)
	{
		cv::circle(statement, cv::Point((int)round(current_views[i].translation().x()), (int)round(-current_views[i].translation().y())), 3, CV_RGB(0, 0, 255));
	}
	draw_robot_views(statement, current_views);
	//cv::imshow("robot positions", statement);
	//cv::waitKey(1);
	char output_path[200];
	sprintf(output_path, "result/_progressive_%d_%d.png", g_plan_iteration, vid);
	cv::imwrite(output_path, statement);
	return true;
}